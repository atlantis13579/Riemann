#include "../Src/Modules/Robotics/RobotController.h"
#include "SceneWorld.h"
#include "../Src/Modules/Vehicle/VehicleDynamics.h"
#include "../Src/RigidBodyDynamics/Ragdoll.h"
#include "../Src/RigidBodyDynamics/PhysicalAnimation.h"
#include "../Src/CollisionPrimitive/AxisAlignedBox3.h"
#include "../Src/CollisionPrimitive/Capsule3.h"
#include "../Src/CollisionPrimitive/Sphere3.h"

#include <ctype.h>
#include <float.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include <algorithm>
#include <map>
#include <cmath>
#include <fstream>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>

#include "../Src/Collision/GeometryObject.h"
#include "../Src/Collision/GeometryQuery.h"
#include "../Src/CollisionPrimitive/StaticMesh.h"
#include "../Src/CollisionPrimitive/TriangleMesh.h"
#include "../Src/CollisionPrimitive/DistanceFieldShape.h"
#include "../Src/Destruction/DestructionSet.h"
#include "../Src/Destruction/Fracture.h"
#include "../Src/Geometry/DynamicMesh.h"
#include "../Src/RigidBodyDynamics/PhysicsWorld.h"
#include "../Src/RigidBodyDynamics/ForceField.h"
#include "../Src/Deformation/Rope.h"
#include "../Src/Deformation/StrandSystem.h"
#include "../Src/Deformation/Cloth.h"
#include "../Src/Deformation/DeformableBody.h"
#include "../Src/Modules/Vehicle/VehicleDeformation.h"
#include "../Src/CollisionPrimitive/HeightField3.h"
#include "../Src/Geometry/VoxelField.h"
#include "../Src/Destruction/StrainSolver.h"

namespace Riemann
{
	namespace
	{
		const float kPi = 3.14159265358979323846f;

		bool AttachClothPatchToSphere(ClothSystem& Cloth, Geometry* Shape, int Count, float Compliance)
		{
			auto* Body = Shape ? Shape->GetParent<RigidBody>() : nullptr;
			const auto* Sphere = Shape ? Shape->GetShapeObj<Sphere3>() : nullptr;
			if (!Body || !Sphere || Count < 1 || Count > 16 || !std::isfinite(Compliance) || Compliance < 0) return false;
			const Vector3 Center = Body->GetGeometryTransform(Shape).LocalToWorld(Sphere->Center);
			std::vector<std::pair<float, int>> Vertices;
			for (int I = 0; I < Cloth.GetParticleCount(); ++I)
			{
				const auto& P = Cloth.GetParticle(I);
				if (!P.Pinned && P.InverseMass > 0) Vertices.emplace_back((P.Position - Center).SquareLength(), I);
			}
			if ((int)Vertices.size() < Count) return false;
			std::sort(Vertices.begin(), Vertices.end());
			for (int I = 0; I < Count; ++I)
			{
				const int Vertex = Vertices[I].second;
				Vector3 Normal = (Cloth.GetParticle(Vertex).Position - Center).SafeUnit();
				if (Normal.IsZero()) Normal = Vector3::UnitZ();
				const Vector3 Anchor = Center + Normal * (Sphere->Radius + Cloth.GetSettings().Thickness);
				Cloth.SetParticlePosition(Vertex, Anchor);
				Cloth.AttachToRigid(BarycentricPoint::Vertex(Vertex), Body, Anchor, Compliance);
			}
			return true;
		}

		class AttachmentPayloadGravity final : public ForceField
		{
		public:
			AttachmentPayloadGravity(GeometryHandle Target, const Vector3& Acceleration) : m_Target(Target), m_Acceleration(Acceleration) {}
			bool ApplyForce(RigidBodyDynamic* Body) override
			{
				if (Body->GetNumGeometries() && Body->GetGeometryHandle(0) == m_Target && !Body->Sleeping)
					Body->ApplyLinearAcceleration(m_Acceleration);
				return true;
			}
		private:
			GeometryHandle m_Target;
			Vector3 m_Acceleration;
		};

		std::string DirectoryOf(const std::string& fileName)
		{
			const size_t pos = fileName.find_last_of("/\\");
			if (pos == std::string::npos)
			{
				return std::string();
			}
			return fileName.substr(0, pos + 1);
		}

		std::string DefaultRagdollDefinition(const std::string& SceneDirectory, const std::string& Fallback)
		{
			// Resolve the built-in definition from the loaded scene so launching the
			// Viewer from another working directory does not remove its ragdolls.
			if (SceneDirectory.empty()) return Fallback;
			const std::string Candidate = SceneDirectory + "../Config/default_ragdoll.json";
			std::ifstream File(Candidate.c_str(), std::ios::in | std::ios::binary);
			return File ? Candidate : Fallback;
		}

		float ToRadians(float degrees)
		{
			return degrees * kPi / 180.0f;
		}

		void ResolveVoxelGridCounts(int& PiecesX, int& PiecesY, int& PiecesZ)
		{
			const int MaxPieceCount = 4096;
			PiecesX = std::max(1, std::min(PiecesX, 32));
			PiecesY = std::max(1, std::min(PiecesY, 32));
			PiecesZ = std::max(1, std::min(PiecesZ, 32));
			while (PiecesX * PiecesY * PiecesZ > MaxPieceCount)
			{
				if (PiecesX >= PiecesY && PiecesX >= PiecesZ && PiecesX > 1)
				{
					--PiecesX;
				}
				else if (PiecesY >= PiecesZ && PiecesY > 1)
				{
					--PiecesY;
				}
				else if (PiecesZ > 1)
				{
					--PiecesZ;
				}
				else
				{
					break;
				}
			}
		}

		bool BuildBoxDynamicMesh(const Vector3& Center, const Vector3& HalfExtent, const Quaternion& Rotation, DynamicMesh* Mesh)
		{
			if (Mesh == nullptr || HalfExtent.x <= 0.0f || HalfExtent.y <= 0.0f || HalfExtent.z <= 0.0f)
			{
				return false;
			}

			Mesh->Clear();
			const Vector3 Corners[8] =
			{
				Vector3(-HalfExtent.x, -HalfExtent.y, -HalfExtent.z),
				Vector3( HalfExtent.x, -HalfExtent.y, -HalfExtent.z),
				Vector3( HalfExtent.x,  HalfExtent.y, -HalfExtent.z),
				Vector3(-HalfExtent.x,  HalfExtent.y, -HalfExtent.z),
				Vector3(-HalfExtent.x, -HalfExtent.y,  HalfExtent.z),
				Vector3( HalfExtent.x, -HalfExtent.y,  HalfExtent.z),
				Vector3( HalfExtent.x,  HalfExtent.y,  HalfExtent.z),
				Vector3(-HalfExtent.x,  HalfExtent.y,  HalfExtent.z),
			};
			for (const Vector3& Corner : Corners)
			{
				Mesh->AppendVertex(Center + Rotation * Corner);
			}

			const int Triangles[12][3] =
			{
				{ 0, 2, 1 }, { 0, 3, 2 },
				{ 4, 5, 6 }, { 4, 6, 7 },
				{ 0, 4, 7 }, { 0, 7, 3 },
				{ 1, 2, 6 }, { 1, 6, 5 },
				{ 0, 1, 5 }, { 0, 5, 4 },
				{ 3, 7, 6 }, { 3, 6, 2 },
			};
			for (const int* Triangle : Triangles)
			{
				Mesh->AppendTriangle(Triangle[0], Triangle[1], Triangle[2]);
			}

			Mesh->BuildBounds();
			Mesh->CalculateWeightAverageNormals();
			return true;
		}

		void AddFlatShadedBox(StaticMesh* mesh, const Vector3& minimum, const Vector3& maximum)
		{
			Vector3 vertices[8];
			Box3::GetVertices(minimum, maximum, vertices);
			const int triangles[12][3] =
			{
				{ 0, 1, 2 }, { 1, 3, 2 }, { 4, 5, 6 }, { 5, 7, 6 },
				{ 0, 1, 4 }, { 5, 4, 1 }, { 1, 3, 5 }, { 7, 5, 3 },
				{ 2, 4, 0 }, { 6, 4, 2 }, { 3, 2, 6 }, { 6, 7, 3 }
			};
			for (const int* triangle : triangles)
			{
				const uint32_t baseVertex = mesh->GetVertexCount();
				mesh->AddVertex(vertices[triangle[0]]);
				mesh->AddVertex(vertices[triangle[1]]);
				mesh->AddVertex(vertices[triangle[2]]);
				mesh->AddTriangle(baseVertex, baseVertex + 1, baseVertex + 2);
			}
		}

		Vector3 SafeUnitVector(const Vector3& value, const Vector3& fallback)
		{
			const float lenSq = value.SquareLength();
			if (lenSq <= 1e-8f)
			{
				return fallback;
			}
			return value / sqrtf(lenSq);
		}
	}

	class JsonValue
	{
	public:
		enum Type
		{
			Null,
			Bool,
			Number,
			String,
			Array,
			Object,
		};

		Type type = Null;
		bool boolValue = false;
		double numberValue = 0.0;
		std::string stringValue;
		std::vector<JsonValue> arrayValue;
		std::map<std::string, JsonValue> objectValue;

		bool IsObject() const { return type == Object; }
		bool IsArray() const { return type == Array; }
		bool IsString() const { return type == String; }
		bool IsNumber() const { return type == Number; }
		bool IsBool() const { return type == Bool; }

		const JsonValue* Find(const char* key) const
		{
			if (!IsObject())
			{
				return nullptr;
			}

			auto it = objectValue.find(key);
			return it != objectValue.end() ? &it->second : nullptr;
		}
	};

	class JsonParser
	{
	public:
		explicit JsonParser(const std::string& text)
			: m_Text(text)
			, m_Pos(0)
		{
		}

		bool Parse(JsonValue* out, std::string* errorMessage)
		{
			SkipWhitespace();
			if (!ParseValue(out, errorMessage))
			{
				return false;
			}
			SkipWhitespace();
			if (m_Pos != m_Text.size())
			{
				SetError("unexpected trailing characters", errorMessage);
				return false;
			}
			return true;
		}

	private:
		void SkipWhitespace()
		{
			while (m_Pos < m_Text.size())
			{
				const char c = m_Text[m_Pos];
				if (isspace(static_cast<unsigned char>(c)))
				{
					++m_Pos;
					continue;
				}

				if (c == '/' && m_Pos + 1 < m_Text.size() && m_Text[m_Pos + 1] == '/')
				{
					m_Pos += 2;
					while (m_Pos < m_Text.size() && m_Text[m_Pos] != '\n')
					{
						++m_Pos;
					}
					continue;
				}

				if (c == '/' && m_Pos + 1 < m_Text.size() && m_Text[m_Pos + 1] == '*')
				{
					m_Pos += 2;
					while (m_Pos + 1 < m_Text.size() && !(m_Text[m_Pos] == '*' && m_Text[m_Pos + 1] == '/'))
					{
						++m_Pos;
					}
					if (m_Pos + 1 < m_Text.size())
					{
						m_Pos += 2;
					}
					continue;
				}

				break;
			}
		}

		bool ParseValue(JsonValue* out, std::string* errorMessage)
		{
			SkipWhitespace();
			if (m_Pos >= m_Text.size())
			{
				SetError("unexpected end of JSON", errorMessage);
				return false;
			}

			const char c = m_Text[m_Pos];
			if (c == '{')
			{
				return ParseObject(out, errorMessage);
			}
			if (c == '[')
			{
				return ParseArray(out, errorMessage);
			}
			if (c == '"')
			{
				out->type = JsonValue::String;
				return ParseString(&out->stringValue, errorMessage);
			}
			if (c == '-' || (c >= '0' && c <= '9'))
			{
				return ParseNumber(out, errorMessage);
			}
			if (Match("true"))
			{
				out->type = JsonValue::Bool;
				out->boolValue = true;
				return true;
			}
			if (Match("false"))
			{
				out->type = JsonValue::Bool;
				out->boolValue = false;
				return true;
			}
			if (Match("null"))
			{
				out->type = JsonValue::Null;
				return true;
			}

			SetError("invalid JSON value", errorMessage);
			return false;
		}

		bool ParseObject(JsonValue* out, std::string* errorMessage)
		{
			out->type = JsonValue::Object;
			out->objectValue.clear();
			++m_Pos;
			SkipWhitespace();

			if (Consume('}'))
			{
				return true;
			}

			while (m_Pos < m_Text.size())
			{
				std::string key;
				if (!ParseString(&key, errorMessage))
				{
					return false;
				}

				SkipWhitespace();
				if (!Consume(':'))
				{
					SetError("expected ':' after object key", errorMessage);
					return false;
				}

				JsonValue value;
				if (!ParseValue(&value, errorMessage))
				{
					return false;
				}
				out->objectValue[key] = value;

				SkipWhitespace();
				if (Consume('}'))
				{
					return true;
				}
				if (!Consume(','))
				{
					SetError("expected ',' or '}' in object", errorMessage);
					return false;
				}
				SkipWhitespace();
			}

			SetError("unterminated object", errorMessage);
			return false;
		}

		bool ParseArray(JsonValue* out, std::string* errorMessage)
		{
			out->type = JsonValue::Array;
			out->arrayValue.clear();
			++m_Pos;
			SkipWhitespace();

			if (Consume(']'))
			{
				return true;
			}

			while (m_Pos < m_Text.size())
			{
				JsonValue value;
				if (!ParseValue(&value, errorMessage))
				{
					return false;
				}
				out->arrayValue.push_back(value);

				SkipWhitespace();
				if (Consume(']'))
				{
					return true;
				}
				if (!Consume(','))
				{
					SetError("expected ',' or ']' in array", errorMessage);
					return false;
				}
			}

			SetError("unterminated array", errorMessage);
			return false;
		}

		bool ParseString(std::string* out, std::string* errorMessage)
		{
			SkipWhitespace();
			if (!Consume('"'))
			{
				SetError("expected string", errorMessage);
				return false;
			}

			out->clear();
			while (m_Pos < m_Text.size())
			{
				const char c = m_Text[m_Pos++];
				if (c == '"')
				{
					return true;
				}

				if (c == '\\')
				{
					if (m_Pos >= m_Text.size())
					{
						SetError("unterminated string escape", errorMessage);
						return false;
					}
					const char escaped = m_Text[m_Pos++];
					switch (escaped)
					{
					case '"': out->push_back('"'); break;
					case '\\': out->push_back('\\'); break;
					case '/': out->push_back('/'); break;
					case 'b': out->push_back('\b'); break;
					case 'f': out->push_back('\f'); break;
					case 'n': out->push_back('\n'); break;
					case 'r': out->push_back('\r'); break;
					case 't': out->push_back('\t'); break;
					case 'u':
						if (m_Pos + 4 <= m_Text.size())
						{
							m_Pos += 4;
							out->push_back('?');
							break;
						}
						SetError("invalid unicode escape", errorMessage);
						return false;
					default:
						SetError("invalid string escape", errorMessage);
						return false;
					}
				}
				else
				{
					out->push_back(c);
				}
			}

			SetError("unterminated string", errorMessage);
			return false;
		}

		bool ParseNumber(JsonValue* out, std::string* errorMessage)
		{
			const char* start = m_Text.c_str() + m_Pos;
			char* end = nullptr;
			const double value = strtod(start, &end);
			if (end == start)
			{
				SetError("invalid number", errorMessage);
				return false;
			}

			out->type = JsonValue::Number;
			out->numberValue = value;
			m_Pos += static_cast<size_t>(end - start);
			return true;
		}

		bool Match(const char* text)
		{
			const size_t len = strlen(text);
			if (m_Text.compare(m_Pos, len, text) == 0)
			{
				m_Pos += len;
				return true;
			}
			return false;
		}

		bool Consume(char c)
		{
			SkipWhitespace();
			if (m_Pos < m_Text.size() && m_Text[m_Pos] == c)
			{
				++m_Pos;
				return true;
			}
			return false;
		}

		void SetError(const char* message, std::string* errorMessage) const
		{
			if (errorMessage == nullptr)
			{
				return;
			}

			int line = 1;
			int column = 1;
			for (size_t i = 0; i < m_Pos && i < m_Text.size(); ++i)
			{
				if (m_Text[i] == '\n')
				{
					++line;
					column = 1;
				}
				else
				{
					++column;
				}
			}

			std::ostringstream ss;
			ss << message << " at line " << line << ", column " << column;
			*errorMessage = ss.str();
		}

	private:
		const std::string& m_Text;
		size_t m_Pos;
	};

	struct SceneWorld::SceneObjectDesc
	{
		std::string Id;
		std::string Type = "box";
		RigidType BodyType = RigidType::Static;
		Vector3 Position = Vector3::Zero();
		Quaternion Rotation = Quaternion::One();
		Vector3 HalfExtent = Vector3::One();
		Vector3 Normal = Vector3::UnitY();
		Vector3 X0 = Vector3(0.0f, -1.0f, 0.0f);
		Vector3 X1 = Vector3(0.0f, 1.0f, 0.0f);
		Vector3 LinearVelocity = Vector3::Zero();
		Vector3 AngularVelocity = Vector3::Zero();
		Vector4 Color = Vector4(0.72f, 0.76f, 0.82f, 1.0f);
		float Radius = 1.0f;
		float RingRadius = 3.0f;
		float Height = 2.0f;
		int TerrainRows = 65;
		int TerrainColumns = 65;
		float HeightScale = 0.01f;
		float BaseHeight = 0.0f;
		std::vector<float> Heights;
		bool TerrainValid = true;
		Vector3i VoxelGrid = Vector3i(0, 0, 0);
		Vector3 VoxelOrigin = Vector3::Zero();
		float VoxelSize = 0.5f;
		float VoxelHeight = 0.5f;
		std::vector<int> VoxelSpans;
		bool VoxelsValid = true;
		int SphereCount = 24;
		int StepCount = 10;
		float StepWidth = 1.2f;
		float StepHeight = 0.5f;
		float StepDepth = 6.0f;
		float InvMass = 1.0f;
		float Restitution = 0.0f;
		bool HasInvMass = false;
		bool HasRestitution = false;
		float FrictionDynamic = 1.0f, FrictionStatic = 1.0f;
		bool HasFriction = false, FrictionValid = true;
		bool HasX0 = false;
		bool HasX1 = false;
		bool DisableGravity = false;
		bool EnableCCD = false;
		bool MouseControl = false;
		bool RenderBounds = false;
		bool RenderEnabled = true;
		bool QueryEnabled = true;
		bool SimulationEnabled = true;
		CollisionData FilterData;
		bool FilterDataValid = true;
		float ContactOffset = 0.0f, RestOffset = 0.0f;
		bool ContactOffsetsValid = true;
		bool HasSDFOption = false, SDFCollision = false, SDFValid = true;
		int SDFResolution = 24;
		float Density = 0.0f;
		bool HasDensity = false;
		int RepeatCount[3] = { 1, 1, 1 };
		int DestructionPiecesX = 5;
		int DestructionPiecesY = 5;
		int DestructionPiecesZ = 5;
		int DestructionPieceCount = 28;
		int DestructionSeed = 17;
		float DestructionBondStrength = 1.0f;
		float DestructionBreakThreshold = 2.0f;
		int DestructionPartitionSize = 2;
		int DestructionPartitionProcessPerTick = 4;
		int DestructionPartitionProcessIntervalTicks = 1;
		float DestructionPartitionSeparationAttenuation = 0.5f;
		bool DestructionClusterCollisionProxy = true;
		float DestructionMinClusterVolume = 0.0f;
		float DestructionImpulseRadius = 2.0f;
		float DestructionImpulseBreakStrength = 0.5f;
		bool DestructionInitiallyFixed = false;
		bool DestructionUseHierarchy = false;
		DestructionFractureSettings DestructionFracture;
		Vector3 RepeatSpacing = Vector3::Zero();
		std::string MeshFile;
	};

	namespace
	{
		float ReadFloat(const JsonValue* value, float fallback)
		{
			return value && value->IsNumber() ? static_cast<float>(value->numberValue) : fallback;
		}

		int ReadInt(const JsonValue* value, int fallback)
		{
			return value && value->IsNumber() ? static_cast<int>(value->numberValue) : fallback;
		}

		bool ReadParticleIndices(const JsonValue& value, int indexCount, int particleCount, int* indices)
		{
			if (!value.IsArray() || value.arrayValue.size() != (size_t)indexCount) return false;
			for (int i = 0; i < indexCount; ++i)
			{
				const JsonValue& index = value.arrayValue[(size_t)i];
				if (!index.IsNumber() || !std::isfinite(index.numberValue)
					|| index.numberValue < 0.0 || index.numberValue >= particleCount
					|| std::floor(index.numberValue) != index.numberValue) return false;
				indices[i] = static_cast<int>(index.numberValue);
			}
			return true;
		}

		bool ReadBool(const JsonValue* value, bool fallback)
		{
			return value && value->IsBool() ? value->boolValue : fallback;
		}

		bool ReadFiniteNumber(const JsonValue& Object, const char* Name, float& Result)
		{
			const JsonValue* Value = Object.Find(Name);
			if (!Value) return true;
			if (!Value->IsNumber() || !std::isfinite(Value->numberValue)
				|| std::abs(Value->numberValue) > std::numeric_limits<float>::max()) return false;
			Result = (float)Value->numberValue;
			return true;
		}

		bool ReadTypedBool(const JsonValue& Object, const char* Name, bool& Result)
		{
			const JsonValue* Value = Object.Find(Name);
			if (!Value) return true;
			if (!Value->IsBool()) return false;
			Result = Value->boolValue;
			return true;
		}

		bool ReadDriveSettings(const JsonValue& Object, JointDriveSettings& Settings)
		{
			return Object.IsObject()
				&& ReadTypedBool(Object, "positionEnabled", Settings.PositionEnabled)
				&& ReadTypedBool(Object, "velocityEnabled", Settings.VelocityEnabled)
				&& ReadTypedBool(Object, "acceleration", Settings.Acceleration)
				&& ReadFiniteNumber(Object, "stiffness", Settings.Stiffness)
				&& ReadFiniteNumber(Object, "damping", Settings.Damping)
				&& ReadFiniteNumber(Object, "maxForce", Settings.MaxForce) && Settings.IsValid();
		}

		bool ReadScalarDrive(const JsonValue& Object, ScalarJointDrive& Drive)
		{
			return ReadDriveSettings(Object, Drive.Settings)
				&& ReadFiniteNumber(Object, "targetPosition", Drive.TargetPosition)
				&& ReadFiniteNumber(Object, "targetVelocity", Drive.TargetVelocity) && Drive.IsValid();
		}

		bool ReadDriveArray(const JsonValue* Array, ScalarJointDrive (&Drives)[3])
		{
			if (!Array) return true;
			if (!Array->IsArray() || Array->arrayValue.size() != 3) return false;
			for (int Axis = 0; Axis < 3; ++Axis)
				if (!ReadScalarDrive(Array->arrayValue[Axis], Drives[Axis])) return false;
			return true;
		}

		bool ReadDriveVector(const JsonValue* Array, float* Values, size_t Count)
		{
			if (!Array) return true;
			if (!Array->IsArray() || Array->arrayValue.size() != Count) return false;
			for (size_t Index = 0; Index < Count; ++Index)
			{
				const JsonValue& Value = Array->arrayValue[Index];
				if (!Value.IsNumber() || !std::isfinite(Value.numberValue)
					|| std::abs(Value.numberValue) > std::numeric_limits<float>::max()) return false;
				Values[Index] = (float)Value.numberValue;
			}
			return true;
		}

		// Hinge/slider use "drive". Six-DOF uses "drives" with optional three-entry
		// linear/angular arrays and angularMode "axes" or "slerp". Drive targets use
		// meters and radians, unlike the legacy degree-valued angular limit fields.
		// SLERP targets are unit quaternions [x,y,z,w] relative to the initial frame.
		bool ConfigureJointDrives(const JsonValue& Object, PositionConstraint* Joint)
		{
			const JsonValue* Scalar = Object.Find("drive");
			const JsonValue* Block = Object.Find("drives");
			if (!Scalar && !Block) return true;
			if (Scalar && Block) return false;
			if (Scalar)
			{
				ScalarJointDrive Drive;
				if (!ReadScalarDrive(*Scalar, Drive)) return false;
				if (Joint->GetType() == PositionConstraintType::Hinge) static_cast<HingeConstraint*>(Joint)->SetDrive(Drive);
				else if (Joint->GetType() == PositionConstraintType::Slider) static_cast<SliderConstraint*>(Joint)->SetDrive(Drive);
				else return false;
				return true;
			}
			if (Joint->GetType() != PositionConstraintType::SixDOF || !Block->IsObject()) return false;
			SixDOFDriveSettings Drives;
			if (!ReadDriveArray(Block->Find("linear"), Drives.Linear)
				|| !ReadDriveArray(Block->Find("angular"), Drives.Angular)) return false;
			if (const JsonValue* Mode = Block->Find("angularMode"))
			{
				if (!Mode->IsString()) return false;
				if (Mode->stringValue == "slerp") Drives.AngularMode = AngularDriveMode::Slerp;
				else if (Mode->stringValue != "axes") return false;
			}
			if (const JsonValue* Slerp = Block->Find("slerp"))
				if (!ReadDriveSettings(*Slerp, Drives.Slerp)) return false;
			float Orientation[4] = { 0, 0, 0, 1 }, Velocity[3] = {};
			if (!ReadDriveVector(Block->Find("orientationTarget"), Orientation, 4)
				|| !ReadDriveVector(Block->Find("angularVelocityTarget"), Velocity, 3)) return false;
			Drives.OrientationTarget = Quaternion(Orientation[0], Orientation[1], Orientation[2], Orientation[3]);
			Drives.AngularVelocityTarget = Vector3(Velocity[0], Velocity[1], Velocity[2]);
			if (!Drives.IsValid()) return false;
			static_cast<SixDOFConstraint*>(Joint)->SetDrives(Drives);
			return true;
		}

		std::string ReadString(const JsonValue* value, const std::string& fallback)
		{
			return value && value->IsString() ? value->stringValue : fallback;
		}

		Vector3 ReadVector3(const JsonValue* value, const Vector3& fallback)
		{
			if (value == nullptr || !value->IsArray() || value->arrayValue.size() < 3)
			{
				return fallback;
			}
			return Vector3(
				ReadFloat(&value->arrayValue[0], fallback.x),
				ReadFloat(&value->arrayValue[1], fallback.y),
				ReadFloat(&value->arrayValue[2], fallback.z));
		}

		Vector3i ReadVector3i(const JsonValue* value, const Vector3i& fallback)
		{
			if (value == nullptr || !value->IsArray() || value->arrayValue.size() < 2)
			{
				return fallback;
			}
			return Vector3i(
				ReadInt(&value->arrayValue[0], fallback.x),
				ReadInt(&value->arrayValue[1], fallback.y),
				value->arrayValue.size() >= 3
					? ReadInt(&value->arrayValue[2], fallback.z) : fallback.z);
		}

		Vector4 ReadColor(const JsonValue* value, const Vector4& fallback)
		{
			if (value == nullptr || !value->IsArray() || value->arrayValue.size() < 3)
			{
				return fallback;
			}

			const float alpha = value->arrayValue.size() >= 4 ? ReadFloat(&value->arrayValue[3], fallback.w) : fallback.w;
			return Vector4(
				ReadFloat(&value->arrayValue[0], fallback.x),
				ReadFloat(&value->arrayValue[1], fallback.y),
				ReadFloat(&value->arrayValue[2], fallback.z),
				alpha);
		}

		Quaternion ReadQuaternion(const JsonValue* object, const Quaternion& fallback)
		{
			if (object == nullptr || !object->IsObject())
			{
				return fallback;
			}

			const JsonValue* rotation = object->Find("rotation");
			if (rotation != nullptr && rotation->IsArray() && rotation->arrayValue.size() >= 4)
			{
				return Quaternion(
					ReadFloat(&rotation->arrayValue[0], fallback.x),
					ReadFloat(&rotation->arrayValue[1], fallback.y),
					ReadFloat(&rotation->arrayValue[2], fallback.z),
					ReadFloat(&rotation->arrayValue[3], fallback.w));
			}

			const JsonValue* axis = object->Find("rotationAxis");
			const JsonValue* degrees = object->Find("rotationDegrees");
			if (axis != nullptr && degrees != nullptr)
			{
				Quaternion q;
				q.FromRotationAxis(SafeUnitVector(ReadVector3(axis, Vector3::UnitY()), Vector3::UnitY()), ToRadians(ReadFloat(degrees, 0.0f)));
				return q;
			}

			return fallback;
		}

		RigidType ReadBodyType(const std::string& text)
		{
			if (text == "dynamic")
			{
				return RigidType::Dynamic;
			}
			return RigidType::Static;
		}

		std::string Lowercase(const std::string& text)
		{
			std::string result = text;
			std::transform(result.begin(), result.end(), result.begin(), [](unsigned char value)
			{
				return static_cast<char>(tolower(value));
			});
			return result;
		}

		ResolutionPhaseSolver ReadResolutionSolver(const JsonValue* value,
			ResolutionPhaseSolver fallback)
		{
			const std::string solver = Lowercase(ReadString(value, std::string()));
			if (solver == "lcp" || solver == "globallcp" || solver == "lcpglobal")
			{
				return ResolutionPhaseSolver::LCPGlobal;
			}
			if (solver == "sequentialimpulse" || solver == "sequential")
			{
				return ResolutionPhaseSolver::SequentialImpulse;
			}
			if (solver == "pbd") return ResolutionPhaseSolver::PBD;
			if (solver == "xpbd") return ResolutionPhaseSolver::XPBD;
			return fallback;
		}

		SolverBackend ReadSolverBackend(const JsonValue* value, SolverBackend fallback)
		{
			const std::string backend = Lowercase(ReadString(value, std::string()));
			if (backend == "cpu")
			{
				return SolverBackend::CPU;
			}
			if (backend == "simd")
			{
				return SolverBackend::SIMD;
			}
			if (backend == "compute" || backend == "computeshader"
				|| backend == "directcompute")
			{
				return SolverBackend::ComputeShader;
			}
			if (backend == "cuda")
			{
				return SolverBackend::CUDA;
			}
            if (backend == "metal") return SolverBackend::Metal;
			return fallback;
		}

		SceneFieldSimulationDesc ReadFieldSimulation(const JsonValue* value)
		{
			SceneFieldSimulationDesc desc;
			if (value == nullptr || !value->IsObject())
			{
				return desc;
			}
			desc.Type = Lowercase(ReadString(value->Find("type"), std::string()));
			const bool LegacyTracer = desc.Type == "smoke3d";
			const std::string Algorithm = Lowercase(ReadString(value->Find("algorithm"), LegacyTracer ? "incompressible" : "barotropic"));
			desc.Aerodynamics = AerodynamicsSimulationSettings(Algorithm == "incompressible" ? AerodynamicsAlgorithm::Incompressible
				: Algorithm == "barotropic" ? AerodynamicsAlgorithm::Barotropic : (AerodynamicsAlgorithm)255);
			if (LegacyTracer || desc.Type == "aerodynamics_simulation") desc.Type = "aerodynamics3d";
			desc.Method = Lowercase(ReadString(value->Find("method"), desc.Method));
			desc.Backend = Lowercase(ReadString(value->Find("backend"), desc.Backend));
			desc.AllowBackendFallback = ReadBool(value->Find("allowBackendFallback"), desc.AllowBackendFallback);
			desc.RigidCoupling = ReadBool(value->Find("rigidCoupling"), desc.RigidCoupling);
			desc.BoundsMin = ReadVector3(value->Find("boundsMin"), desc.BoundsMin);
			desc.BoundsMax = ReadVector3(value->Find("boundsMax"), desc.BoundsMax);
			desc.Resolution = ReadVector3i(value->Find("resolution"), desc.Resolution);
			desc.Gravity = ReadVector3(value->Find("gravity"), desc.Gravity);
			desc.RestDensity = ReadFloat(value->Find("restDensity"), desc.RestDensity);
			desc.AmbientDensity = ReadFloat(value->Find("ambientDensity"), desc.AmbientDensity);
			desc.AmbientPressure = ReadFloat(value->Find("ambientPressure"), desc.AmbientPressure);
			desc.SpeedOfSound = ReadFloat(value->Find("speedOfSound"), desc.SpeedOfSound);
			desc.AdiabaticExponent = ReadFloat(value->Find("adiabaticExponent"), desc.AdiabaticExponent);
			desc.DynamicViscosity = ReadFloat(value->Find("dynamicViscosity"), desc.DynamicViscosity);
			desc.ArtificialViscosity = ReadFloat(value->Find("artificialViscosity"), desc.ArtificialViscosity);
			desc.PressureStiffness = ReadFloat(value->Find("pressureStiffness"), desc.PressureStiffness);
			desc.ParticleSpacing = ReadFloat(value->Find("particleSpacing"), desc.ParticleSpacing);
			desc.SmoothingRadius = ReadFloat(value->Find("smoothingRadius"), desc.SmoothingRadius);
			desc.BoundaryRestitution = ReadFloat(value->Find("boundaryRestitution"), desc.BoundaryRestitution);
			desc.BoundaryFriction = ReadFloat(value->Find("boundaryFriction"), desc.BoundaryFriction);
			desc.OpenTopBoundary = ReadBool(value->Find("openTopBoundary"), desc.OpenTopBoundary);
			desc.MaxTimeStep = ReadFloat(value->Find("maxTimeStep"), desc.MaxTimeStep);
			desc.AcousticCFL = ReadFloat(value->Find("acousticCFL"), desc.AcousticCFL);
			desc.MaxSubsteps = ReadInt(value->Find("maxSubsteps"), desc.MaxSubsteps);
			desc.PressureIterations = ReadInt(value->Find("pressureIterations"), desc.PressureIterations);
			desc.SurfaceSamplesPerAxis = ReadInt(value->Find("surfaceSamplesPerAxis"), desc.SurfaceSamplesPerAxis);
			desc.DragCoefficient = ReadFloat(value->Find("dragCoefficient"), desc.DragCoefficient);
			desc.MaxLinearAcceleration = ReadFloat(value->Find("maxLinearAcceleration"), desc.MaxLinearAcceleration);
			desc.MaxAngularAcceleration = ReadFloat(value->Find("maxAngularAcceleration"), desc.MaxAngularAcceleration);
			desc.VisualizationThreshold = ReadFloat(value->Find("visualizationThreshold"), desc.VisualizationThreshold);
			desc.InteractionRadius = ReadFloat(value->Find("interactionRadius"), desc.InteractionRadius);
			desc.InteractionStrength = ReadFloat(value->Find("interactionStrength"), desc.InteractionStrength);
			desc.Aerodynamics.Gravity = desc.Gravity;
			desc.Aerodynamics.AmbientDensity = desc.AmbientDensity;
			desc.Aerodynamics.AmbientPressure = desc.AmbientPressure;
			desc.Aerodynamics.SpeedOfSound = desc.SpeedOfSound;
			desc.Aerodynamics.AdiabaticExponent = desc.AdiabaticExponent;
			desc.Aerodynamics.DynamicViscosity = desc.DynamicViscosity;
			desc.Aerodynamics.ArtificialViscosity = desc.ArtificialViscosity;
			desc.Aerodynamics.ParticleSpacing = desc.ParticleSpacing;
			desc.Aerodynamics.SmoothingRadius = desc.SmoothingRadius;
			desc.Aerodynamics.BoundaryRestitution = desc.BoundaryRestitution;
			desc.Aerodynamics.AcousticCFL = desc.AcousticCFL;
			desc.Aerodynamics.Method = desc.Method == "eulerian" ? FluidSimulationMethod::Eulerian
				: desc.Method == "lagrangian" ? FluidSimulationMethod::Lagrangian : (FluidSimulationMethod)255;
			desc.Aerodynamics.BoundsMin = desc.BoundsMin;
			desc.Aerodynamics.Backend = desc.Backend == "metal" ? FluidBackend::Metal : desc.Backend == "cuda" ? FluidBackend::CUDA
				: (desc.Backend == "compute" || desc.Backend == "computeshader" || desc.Backend == "compute_shader") ? FluidBackend::ComputeShader
				: desc.Backend == "simd" ? FluidBackend::SIMD : desc.Backend == "cpu" ? FluidBackend::CPU : (FluidBackend)255;
			desc.Aerodynamics.AllowBackendFallback = desc.AllowBackendFallback;
			desc.Aerodynamics.BoundsMax = desc.BoundsMax;
			desc.Aerodynamics.Resolution = desc.Resolution;
			desc.Aerodynamics.PressureIterations = desc.PressureIterations;
			desc.Aerodynamics.MaxTimeStep = desc.MaxTimeStep;
			desc.Aerodynamics.MaxSubsteps = desc.MaxSubsteps;
			desc.Aerodynamics.Buoyancy = ReadFloat(value->Find("buoyancy"), desc.Aerodynamics.Buoyancy);
			desc.Aerodynamics.TracerWeight = ReadFloat((value->Find("tracerWeight") ? value->Find("tracerWeight") : value->Find("smokeWeight")), desc.Aerodynamics.TracerWeight);
			desc.Aerodynamics.Vorticity = ReadFloat(value->Find("vorticity"), desc.Aerodynamics.Vorticity);
			desc.Aerodynamics.DensityDissipation = ReadFloat(value->Find("densityDissipation"), desc.Aerodynamics.DensityDissipation);
			desc.Aerodynamics.Cooling = ReadFloat(value->Find("cooling"), desc.Aerodynamics.Cooling);
			desc.TracerExtinction = ReadFloat((value->Find("tracerExtinction") ? value->Find("tracerExtinction") : value->Find("smokeExtinction")), desc.TracerExtinction);
			desc.TracerColor = ReadVector3((value->Find("tracerColor") ? value->Find("tracerColor") : value->Find("smokeColor")), desc.TracerColor);
			const JsonValue* emitters = value->Find("emitters");
			if (emitters && emitters->IsArray())
			{
				for (const JsonValue& emitter : emitters->arrayValue)
				{
					AerodynamicsEmitter source;
					source.Position = ReadVector3(emitter.Find("position"), source.Position);
					source.Velocity = ReadVector3(emitter.Find("velocity"), source.Velocity);
					source.Radius = ReadFloat(emitter.Find("radius"), source.Radius);
					source.Density = ReadFloat(emitter.Find("density"), source.Density);
					source.Temperature = ReadFloat(emitter.Find("temperature"), source.Temperature);
					desc.Aerodynamics.Emitters.push_back(source);
				}
			}
			const JsonValue* obstacles = (value->Find("obstacles") ? value->Find("obstacles") : value->Find("smokeObstacles"));
			if (obstacles && obstacles->IsArray())
			{
				for (const JsonValue& obstacle : obstacles->arrayValue)
				{
					AerodynamicsObstacle solid;
					solid.Center = ReadVector3(obstacle.Find("center"), solid.Center);
					solid.Radius = ReadFloat(obstacle.Find("radius"), solid.Radius);
					desc.Aerodynamics.Obstacles.push_back(solid);
				}
			}

			const JsonValue* regions = value->Find("regions");
			if (regions != nullptr && regions->IsArray())
			{
				for (const JsonValue& regionValue : regions->arrayValue)
				{
					if (!regionValue.IsObject())
					{
						continue;
					}
					SceneFieldRegion region;
					region.Min = ReadVector3(regionValue.Find("min"), region.Min);
					region.Max = ReadVector3(regionValue.Find("max"), region.Max);
					const JsonValue* density = regionValue.Find("density");
					if (density != nullptr && density->IsNumber())
					{
						region.Density = ReadFloat(density, 0.0f);
						region.SetDensity = true;
					}
					const JsonValue* velocity = regionValue.Find("velocity");
					if (velocity != nullptr)
					{
						region.Velocity = ReadVector3(velocity, Vector3::Zero());
						region.SetVelocity = true;
					}
					desc.Regions.push_back(region);
				}
			}
			return desc;
		}

		void ReadRepeat(const JsonValue* object, SceneWorld::SceneObjectDesc* desc)
		{
			if (object == nullptr || !object->IsObject())
			{
				return;
			}

			const JsonValue* repeat = object->Find("repeat");
			if (repeat == nullptr || !repeat->IsObject())
			{
				return;
			}

			const JsonValue* count = repeat->Find("count");
			if (count != nullptr && count->IsArray() && count->arrayValue.size() >= 3)
			{
				desc->RepeatCount[0] = std::max(1, static_cast<int>(ReadFloat(&count->arrayValue[0], 1.0f)));
				desc->RepeatCount[1] = std::max(1, static_cast<int>(ReadFloat(&count->arrayValue[1], 1.0f)));
				desc->RepeatCount[2] = std::max(1, static_cast<int>(ReadFloat(&count->arrayValue[2], 1.0f)));
			}
			desc->RepeatSpacing = ReadVector3(repeat->Find("spacing"), desc->RepeatSpacing);
		}

		SceneWorld::SceneObjectDesc ReadObjectDesc(const JsonValue& object)
		{
			SceneWorld::SceneObjectDesc desc;
			desc.Id = ReadString(object.Find("id"), std::string());
			desc.Type = ReadString(object.Find("type"), desc.Type);
			desc.BodyType = ReadBodyType(ReadString(object.Find("body"), "static"));
			desc.Position = ReadVector3(object.Find("position"), desc.Position);
			desc.Rotation = ReadQuaternion(&object, desc.Rotation);
			desc.HalfExtent = ReadVector3(object.Find("halfExtent"), desc.HalfExtent);
			desc.Normal = ReadVector3(object.Find("normal"), desc.Normal);
			desc.Radius = ReadFloat(object.Find("radius"), desc.Radius);
			desc.RingRadius = ReadFloat(object.Find("ringRadius"), desc.RingRadius);
			desc.SphereCount = std::max(3, ReadInt(object.Find("sphereCount"), desc.SphereCount));
			desc.StepCount = std::max(1, ReadInt(object.Find("stepCount"), desc.StepCount));
			desc.StepWidth = ReadFloat(object.Find("stepWidth"), desc.StepWidth);
			desc.StepHeight = ReadFloat(object.Find("stepHeight"), desc.StepHeight);
			desc.StepDepth = ReadFloat(object.Find("stepDepth"), desc.StepDepth);
			desc.Height = ReadFloat(object.Find("height"), desc.Height);
			if (desc.Type == "heightfield")
			{
				auto ReadDimension = [](const JsonValue* value, int fallback) -> int
				{
					if (!value) return fallback;
					return value->IsNumber() && value->numberValue >= 2 && value->numberValue <= 255
						&& std::floor(value->numberValue) == value->numberValue ? (int)value->numberValue : -1;
				};
				desc.TerrainRows = ReadDimension(object.Find("rows"), desc.TerrainRows);
				desc.TerrainColumns = ReadDimension(object.Find("columns"), desc.TerrainColumns);
				desc.HeightScale = ReadFloat(object.Find("heightScale"), desc.HeightScale);
				desc.BaseHeight = ReadFloat(object.Find("baseHeight"), desc.BaseHeight);
				const JsonValue* heights = object.Find("heights");
				if (heights)
				{
					desc.TerrainValid = heights->IsArray() && heights->arrayValue.size() == (size_t)(desc.TerrainRows * desc.TerrainColumns);
					if (desc.TerrainValid)
						for (const JsonValue& height : heights->arrayValue)
						{
							desc.TerrainValid = desc.TerrainValid && height.IsNumber() && std::isfinite(height.numberValue) && std::abs(height.numberValue) <= FLT_MAX;
							desc.Heights.push_back(ReadFloat(&height, 0.0f));
						}
				}
			}
			if (desc.Type == "voxelMesh")
			{
				auto ReadCell = [](const JsonValue& value, int maximum) -> int
				{
					return value.IsNumber() && std::isfinite(value.numberValue) && value.numberValue >= 0
						&& value.numberValue <= maximum && std::floor(value.numberValue) == value.numberValue
						? (int)value.numberValue : -1;
				};
				const JsonValue* grid = object.Find("grid");
				desc.VoxelsValid = grid && grid->IsArray() && grid->arrayValue.size() == 3;
				if (desc.VoxelsValid)
				{
					for (int axis = 0; axis < 3; ++axis)
					{
						desc.VoxelGrid[axis] = ReadCell(grid->arrayValue[axis], 256);
						desc.VoxelsValid = desc.VoxelsValid && desc.VoxelGrid[axis] > 0;
					}
				}
				desc.VoxelOrigin = ReadVector3(object.Find("voxelOrigin"), desc.VoxelOrigin);
				desc.VoxelSize = ReadFloat(object.Find("voxelSize"), desc.VoxelSize);
				desc.VoxelHeight = ReadFloat(object.Find("voxelHeight"), desc.VoxelHeight);
				const JsonValue* spans = object.Find("spans");
				desc.VoxelsValid = desc.VoxelsValid && spans && spans->IsArray() && spans->arrayValue.size() <= 1048576;
				if (desc.VoxelsValid)
				{
					for (const JsonValue& span : spans->arrayValue)
					{
						if (!span.IsArray() || span.arrayValue.size() != 4) { desc.VoxelsValid = false; break; }
						const int x = ReadCell(span.arrayValue[0], desc.VoxelGrid.x - 1);
						const int z = ReadCell(span.arrayValue[1], desc.VoxelGrid.z - 1);
						const int low = ReadCell(span.arrayValue[2], desc.VoxelGrid.y - 1);
						const int high = ReadCell(span.arrayValue[3], desc.VoxelGrid.y - 1);
						if (x < 0 || z < 0 || low < 0 || high < low) { desc.VoxelsValid = false; break; }
						desc.VoxelSpans.insert(desc.VoxelSpans.end(), { x, z, low, high });
					}
				}
			}
			desc.LinearVelocity = ReadVector3(object.Find("linearVelocity"), desc.LinearVelocity);
			desc.AngularVelocity = ReadVector3(object.Find("angularVelocity"), desc.AngularVelocity);
			desc.DisableGravity = ReadBool(object.Find("disableGravity"), desc.DisableGravity);
			desc.EnableCCD = ReadBool(object.Find("enableCCD"), desc.EnableCCD);
			desc.MouseControl = ReadBool(object.Find("mouseControl"), desc.MouseControl);
			auto ReadFilterValue = [&](const char* Name, uint32_t Maximum, unsigned int& Value)
			{
				const JsonValue* Field = object.Find(Name);
				if (!Field) return;
				if (!Field->IsNumber() || !std::isfinite(Field->numberValue) || Field->numberValue < 0
					|| Field->numberValue > Maximum || std::floor(Field->numberValue) != Field->numberValue)
					desc.FilterDataValid = false;
				else Value = (uint32_t)Field->numberValue;
			};
			for (const char* Name : { "contactOffset", "restOffset" })
			{
				const JsonValue* Field = object.Find(Name);
				if (Field && (!Field->IsNumber() || !std::isfinite(Field->numberValue) || std::abs(Field->numberValue) > FLT_MAX))
					desc.ContactOffsetsValid = false;
			}
			desc.HasSDFOption = object.Find("sdfCollision") != nullptr;
			desc.SDFValid = !desc.HasSDFOption || object.Find("sdfCollision")->IsBool();
			desc.SDFCollision = ReadBool(object.Find("sdfCollision"), false);
			if (const JsonValue* Value = object.Find("sdfResolution"))
			{
				const int MaximumResolution = SignedDistanceFieldBuildOptions().MaxResolution;
				desc.SDFValid = desc.SDFValid && Value->IsNumber() && std::isfinite(Value->numberValue) && Value->numberValue >= 2
					&& Value->numberValue <= MaximumResolution && std::floor(Value->numberValue) == Value->numberValue;
				if (desc.SDFValid) desc.SDFResolution = (int)Value->numberValue;
			}
			desc.ContactOffset = ReadFloat(object.Find("contactOffset"), desc.ContactOffset);
			desc.RestOffset = ReadFloat(object.Find("restOffset"), desc.RestOffset);
			desc.ContactOffsetsValid = desc.ContactOffsetsValid && desc.ContactOffset >= 0.0f && desc.RestOffset <= desc.ContactOffset;
			ReadFilterValue("collisionLayer", CollisionData::LayerCount - 1, desc.FilterData.v0);
			ReadFilterValue("collisionMask", UINT32_MAX, desc.FilterData.v1);
			desc.QueryEnabled = ReadBool(object.Find("query"), desc.QueryEnabled);
			desc.SimulationEnabled = ReadBool(object.Find("simulation"), desc.SimulationEnabled);
			if (object.Find("query") == nullptr && desc.Id == "ground" && desc.Type == "plane")
			{
				desc.QueryEnabled = false;
			}
			desc.DestructionPiecesX = std::max(1, ReadInt(object.Find("piecesX"), desc.DestructionPiecesX));
			desc.DestructionPiecesY = std::max(1, ReadInt(object.Find("piecesY"), desc.DestructionPiecesY));
			desc.DestructionPiecesZ = std::max(1, ReadInt(object.Find("piecesZ"), desc.DestructionPiecesZ));
			desc.DestructionPieceCount = std::max(2, ReadInt(object.Find("pieceCount"), desc.DestructionPieceCount));
			desc.DestructionSeed = ReadInt(object.Find("seed"), desc.DestructionSeed);
			const JsonValue* pieces = object.Find("pieces");
			if (pieces != nullptr && pieces->IsArray() && pieces->arrayValue.size() >= 2)
			{
				desc.DestructionPiecesX = std::max(1, ReadInt(&pieces->arrayValue[0], desc.DestructionPiecesX));
				desc.DestructionPiecesZ = std::max(1, ReadInt(&pieces->arrayValue[1], desc.DestructionPiecesZ));
				if (pieces->arrayValue.size() >= 3)
				{
					desc.DestructionPiecesY = std::max(1, ReadInt(&pieces->arrayValue[2], desc.DestructionPiecesY));
				}
			}
			desc.DestructionBondStrength = ReadFloat(object.Find("bondStrength"), desc.DestructionBondStrength);
			desc.DestructionBreakThreshold = ReadFloat(object.Find("breakThreshold"), desc.DestructionBreakThreshold);
			desc.DestructionPartitionSize = std::max(2, ReadInt(object.Find("partitionSize"), desc.DestructionPartitionSize));
			desc.DestructionPartitionProcessPerTick = std::max(1, ReadInt(object.Find("partitionProcessPerTick"), desc.DestructionPartitionProcessPerTick));
			desc.DestructionPartitionProcessIntervalTicks = std::max(1, ReadInt(object.Find("partitionProcessIntervalTicks"), desc.DestructionPartitionProcessIntervalTicks));
			desc.DestructionPartitionSeparationAttenuation = ReadFloat(object.Find("partitionSeparationAttenuation"), desc.DestructionPartitionSeparationAttenuation);
			desc.DestructionClusterCollisionProxy = ReadBool(object.Find("clusterCollisionProxy"), desc.DestructionClusterCollisionProxy);
			desc.DestructionMinClusterVolume = ReadFloat(object.Find("minClusterVolume"), desc.DestructionMinClusterVolume);
			desc.DestructionImpulseRadius = ReadFloat(object.Find("impulseRadius"), desc.DestructionImpulseRadius);
			desc.DestructionImpulseBreakStrength = ReadFloat(object.Find("impulseBreakStrength"), desc.DestructionImpulseBreakStrength);
			desc.DestructionInitiallyFixed = ReadBool(object.Find("initiallyFixed"), desc.DestructionInitiallyFixed);
			const JsonValue* HierarchicalFracture = object.Find("hierarchicalFracture");
			if (HierarchicalFracture != nullptr)
			{
				desc.DestructionUseHierarchy = true;
			}
			if (HierarchicalFracture != nullptr && HierarchicalFracture->IsObject())
			{
				DestructionFractureSettings& Settings = desc.DestructionFracture;
				Settings.RandomSeed = ReadInt(HierarchicalFracture->Find("seed"), desc.DestructionSeed);
				Settings.MaxChunkCount = std::max(2, ReadInt(HierarchicalFracture->Find("maxChunkCount"), Settings.MaxChunkCount));
				Settings.CutRetryCount = std::max(1, ReadInt(HierarchicalFracture->Find("cutRetryCount"), Settings.CutRetryCount));
				Settings.MinChunkVolume = std::max(0.0f, ReadFloat(HierarchicalFracture->Find("minChunkVolume"), Settings.MinChunkVolume));
				Settings.BondStrength = std::max(0.0f, ReadFloat(HierarchicalFracture->Find("bondStrength"), desc.DestructionBondStrength));
				Settings.DamageAttenuation = ReadFloat(HierarchicalFracture->Find("damageAttenuation"), Settings.DamageAttenuation);
				Settings.NeighborDamageFraction = ReadFloat(HierarchicalFracture->Find("neighborDamageFraction"), Settings.NeighborDamageFraction);
				Settings.Grout = std::max(0.0f, ReadFloat(HierarchicalFracture->Find("grout"), Settings.Grout));

				const JsonValue* Levels = HierarchicalFracture->Find("levels");
				if (Levels != nullptr && Levels->IsArray())
				{
					Settings.Levels.clear();
					for (const JsonValue& LevelValue : Levels->arrayValue)
					{
						if (!LevelValue.IsObject())
						{
							continue;
						}
						DestructionFractureLevel Level;
						const int PieceCount = std::max(2, ReadInt(LevelValue.Find("pieceCount"), Level.MinPieceCount));
						Level.MinPieceCount = std::max(2, ReadInt(LevelValue.Find("minPieceCount"), PieceCount));
						Level.MaxPieceCount = std::max(Level.MinPieceCount, ReadInt(LevelValue.Find("maxPieceCount"), PieceCount));
						Level.PreserveLargestPieces = std::max(0, ReadInt(LevelValue.Find("preserveLargestPieces"), Level.PreserveLargestPieces));
						Level.ChildFractureProbability = ReadFloat(LevelValue.Find("childFractureProbability"), Level.ChildFractureProbability);
						Level.BreakThreshold = std::max(0.0f, ReadFloat(LevelValue.Find("breakThreshold"), Level.BreakThreshold));
						Level.StrengthVariation = ReadFloat(LevelValue.Find("strengthVariation"), Level.StrengthVariation);
						Level.UseClusteredVoronoi = ReadBool(LevelValue.Find("clusteredVoronoi"), Level.UseClusteredVoronoi);
						Settings.Levels.push_back(Level);
					}
				}
			}
			desc.RenderBounds = ReadBool(object.Find("renderBounds"), desc.RenderBounds);
			desc.RenderEnabled = ReadBool(object.Find("render"), desc.RenderEnabled);
			desc.Color = ReadColor(object.Find("color"), desc.BodyType == RigidType::Dynamic
				? Vector4(0.28f, 0.52f, 0.95f, 1.0f)
				: Vector4(0.58f, 0.62f, 0.66f, 1.0f));
			desc.MeshFile = ReadString(object.Find("mesh"), desc.MeshFile);

			const JsonValue* invMass = object.Find("invMass");
			if (invMass != nullptr && invMass->IsNumber())
			{
				desc.InvMass = ReadFloat(invMass, desc.InvMass);
				desc.HasInvMass = true;
			}
			const JsonValue* restitution = object.Find("restitution");
			if (restitution != nullptr && restitution->IsNumber())
			{
				desc.Restitution = std::max(0.0f, std::min(1.0f, ReadFloat(restitution, 0.0f)));
				desc.HasRestitution = true;
			}
			const JsonValue* density = object.Find("density");
			const JsonValue* DynamicFriction = object.Find("frictionDynamic");
			const JsonValue* StaticFriction = object.Find("frictionStatic");
			desc.HasFriction = DynamicFriction || StaticFriction;
			if (desc.HasFriction)
			{
				auto ReadFriction = [&](const JsonValue* Value, float& Coefficient)
				{
					if (!Value) return;
					desc.FrictionValid = desc.FrictionValid && Value->IsNumber() && std::isfinite(Value->numberValue)
						&& Value->numberValue >= 0.0 && Value->numberValue <= FLT_MAX;
					Coefficient = ReadFloat(Value, Coefficient);
				};
				ReadFriction(DynamicFriction, desc.FrictionDynamic);
				ReadFriction(StaticFriction, desc.FrictionStatic);
				desc.FrictionValid = desc.FrictionValid && desc.FrictionStatic >= desc.FrictionDynamic;
			}
			if (density != nullptr && density->IsNumber())
			{
				desc.Density = ReadFloat(density, 0.0f);
				desc.HasDensity = true;
			}

			const JsonValue* x0 = object.Find("x0");
			const JsonValue* x1 = object.Find("x1");
			if (x0 != nullptr)
			{
				desc.X0 = ReadVector3(x0, desc.X0);
				desc.HasX0 = true;
			}
			if (x1 != nullptr)
			{
				desc.X1 = ReadVector3(x1, desc.X1);
				desc.HasX1 = true;
			}

			ReadRepeat(&object, &desc);
			return desc;
		}

		bool ApplyBodyDensity(RigidBody* body, float density)
		{
			RigidBodyDynamic* dynamicBody = body ? body->CastDynamic() : nullptr;
			if (dynamicBody == nullptr || !(density > 0.0f) || !std::isfinite(density)
				|| !(dynamicBody->InvMass > 0.0f))
			{
				return false;
			}
			float volume = 0.0f;
			for (Geometry* geometry : dynamicBody->Geometries())
			{
				if (geometry != nullptr && geometry->GetMassParameters() != nullptr)
				{
					volume += geometry->GetMassParameters()->Volume;
				}
			}
			if (!(volume > 0.0f) || !std::isfinite(volume))
			{
				return false;
			}
			const float targetInverseMass = 1.0f / (density * volume);
			const float inertiaScale = targetInverseMass / dynamicBody->InvMass;
			dynamicBody->InvMass = targetInverseMass;
			dynamicBody->InvInertia *= inertiaScale;
			dynamicBody->MaxContactImpulse = 100000.0f;
			dynamicBody->SleepThreshold = 1.0e-5f;
			return true;
		}
	}

	SceneWorld::SceneWorld() : m_RagdollAlgorithmOverride(RagdollAlgorithm::JointConstraints)
	{
		Reset();
	}

	SceneWorld::~SceneWorld()
	{
	}

	void SceneWorld::Reset(const Vector3& gravity)
	{
		PhysicsWorldParam param;
		param.gravityAcc = gravity;
		ResetWorld(param);
	}

	void SceneWorld::ResetWorld(const PhysicsWorldParam& param)
	{
		PhysicsWorldParam worldParam = param;
		worldParam.sceneQueryType = SceneQueryType::DynamicAABB;
		worldParam.broadphase = BroadPhaseSolver::SAP;
		if (!worldParam.collisionFilter) worldParam.collisionFilter.reset(CollisionFilter::CreateLayerMask());
		std::unique_ptr<PhysicsWorld> replacement(new PhysicsWorld(worldParam));
		m_DemoFieldIds.clear(); m_DemoFreezeField = 0; m_PhysicsFieldFreeze = false;
		m_DestructionPlaybacks.clear(); m_DestructionRecorders.clear(); m_DestructionCaches.clear();
		m_DestructionPlaybackEvents.clear(); m_DestructionRecordingTime = 0;
		m_DestructionRenderStates.clear();
		m_DestructionSets.clear();
		m_DestructionEvents.clear(); m_DestructionEventCount = m_DestructionDroppedEvents = 0;
		m_Objects.clear();
		m_WalkingRobot = nullptr; m_RobotSurfaces.clear(); m_TrainingDisplay = false; m_TrainingBuildIndex = 0;
		m_SweepWalkingRobot = nullptr; m_SweepRobotPose.reset();
		m_RoboticsWalkingDemo = false; m_RobotAutoWalk = false;
		m_RobotDriveInput = m_RobotDriveCommand = m_SweepRobotPrevious = Vector3::Zero();
		m_RobotDriveHeading = m_SweepRobotPhase = 0;
		m_Vehicles.clear();
		m_RopeSurfaces.clear();
		m_DemoStrands=nullptr; m_DemoStrandInstanceId=0; m_StrandDemoTwist=0; m_StrandRootLocal.clear();
		m_AnimationSurfaces.clear(); m_AnimationRestLocal.clear(); m_AnimationAnchors.clear(); m_AnimationAxes.clear();
		m_DemoAnimation = nullptr; m_PhysicalAnimationTime = 0;
		m_DemoCloth = nullptr; m_DemoClothGeometry = nullptr; m_ClothRestPose.clear(); m_ClothAnimationTime = 0;
		m_DemoClothPartner = nullptr; m_DemoClothPartnerGeometry = nullptr; m_ClothPartnerRestPose.clear(); m_ClothDemoMesh.reset();
		m_ClothSoftMaterial = false;
		m_DemoSoftBodies.clear(); m_CompliantAttachmentDemo = false;
		m_DemoContactBodies.clear();
		m_SoftRigidContactDemo = false; m_DemoContactHandle = GeometryHandle(); m_DemoContactGeometry = nullptr; m_DemoContactGravity = nullptr;
		m_RigidAttachmentDemo = false; m_DemoAttachmentHandle = GeometryHandle(); m_DemoAttachmentGeometry = nullptr;
		m_DemoAttachmentGravity = nullptr;
		m_MechanismDemoDrivers.clear(); m_MechanismDemoTime = 0;
		m_TerrainRevisions.clear();
		m_ConstraintInstances.clear();
		m_Character = nullptr; m_CharacterPlatform = nullptr; m_CharacterDemoTime = 0;
		m_MouseControlGeometries.clear();
		m_MouseControlGeometry = nullptr;
		m_MouseControlBody = nullptr;
		m_MouseControlRope = nullptr;
		m_MouseControlRopeParticle = -1;
		TouchRenderRevision();

		m_World = std::move(replacement);

		m_Camera = CameraDesc();
		m_CameraSidebarPadding = 0;
		m_Light = DirectionalLightDesc();
		m_FieldSimulation = SceneFieldSimulationDesc();
		m_Name.clear();
		m_Configurations.clear();
		m_ActiveConfigurationId.clear();
	}

	void SceneWorld::RegisterMouseControlGeometry(Geometry* geometry)
	{
		if (geometry == nullptr)
		{
			return;
		}
		if (std::find(m_MouseControlGeometries.begin(), m_MouseControlGeometries.end(), geometry) == m_MouseControlGeometries.end())
		{
			m_MouseControlGeometries.push_back(geometry);
		}
		m_MouseControlGeometry = geometry;
		m_MouseControlBody = geometry->GetParent<RigidBody>();
	}

	bool SceneWorld::IsMouseControlGeometry(const Geometry* geometry) const
	{
		return geometry != nullptr
			&& std::find(m_MouseControlGeometries.begin(), m_MouseControlGeometries.end(), geometry) != m_MouseControlGeometries.end();
	}

	bool SceneWorld::RayCastMouseControlGeometry(const Vector3& origin, const Vector3& direction, float maxDistance,
		Geometry** hitGeometry, Vector3* hitPoint) const
	{
		Geometry* nearestGeometry = nullptr;
		float nearestTime = maxDistance;
		RayCastOption option;
		option.Type = RayCastOption::RAYCAST_NEAREST;
		option.HitBothSides = true;
		for (Geometry* geometry : m_MouseControlGeometries)
		{
			if (geometry == nullptr)
			{
				continue;
			}
			RigidBody* body = geometry->GetParent<RigidBody>();
			const Transform transform = body ? body->GetGeometryTransform(geometry) : *geometry->GetTransform();
			option.MaxDist = nearestTime;
			RayCastResult result;
			if (geometry->RayCast(transform, origin, direction, &option, &result)
				&& result.hitTime >= 0.0f && result.hitTime < nearestTime)
			{
				nearestGeometry = geometry;
				nearestTime = result.hitTime;
			}
		}
		if (nearestGeometry == nullptr)
		{
			return false;
		}
		if (hitGeometry)
		{
			*hitGeometry = nearestGeometry;
		}
		if (hitPoint)
		{
			*hitPoint = origin + direction * nearestTime;
		}
		return true;
	}

	bool SceneWorld::SelectMouseControlGeometry(Geometry* geometry)
	{
		if (!IsMouseControlGeometry(geometry))
		{
			return false;
		}
		m_MouseControlGeometry = geometry;
		m_MouseControlBody = geometry->GetParent<RigidBody>();
		return m_MouseControlBody != nullptr;
	}

	bool SceneWorld::SetMouseControlPosition(const Vector3& worldPosition, float deltaTime)
	{
		if (m_MouseControlBody == nullptr)
		{
			return false;
		}
		const Vector3 OldPosition = m_MouseControlBody->X;
		m_MouseControlBody->X = worldPosition;
		if (RigidBodyDynamic* Dynamic = m_MouseControlBody->CastDynamic())
		{
			const float SafeTime = std::max(1.0f / 240.0f, std::min(deltaTime, 0.1f));
			Vector3 ThrowVelocity = (worldPosition - OldPosition) / SafeTime;
			const float MaxThrowSpeed = 35.0f;
			const float Speed = ThrowVelocity.Length();
			if (Speed > MaxThrowSpeed) ThrowVelocity *= MaxThrowSpeed / Speed;
			Dynamic->V = Dynamic->V * 0.25f + ThrowVelocity * 0.75f;
			Dynamic->W = Vector3::Zero();
			Dynamic->Wakeup();
		}
		else
		{
			m_MouseControlBody->V = Vector3::Zero();
			m_MouseControlBody->W = Vector3::Zero();
		}
		if (m_MouseControlRope != nullptr && m_MouseControlRopeParticle >= 0)
		{
			m_MouseControlRope->PinParticle(m_MouseControlRopeParticle, worldPosition);
		}
		return true;
	}

	bool SceneWorld::SetMouseControlPosition(const Vector3& worldPosition, float deltaTime,
		const Vector3& boundsMin, const Vector3& boundsMax)
	{
		return SetMouseControlPosition(worldPosition, deltaTime, boundsMin, boundsMax,
			Transform::Identity());
	}

	bool SceneWorld::SetMouseControlPosition(const Vector3& worldPosition, float deltaTime,
		const Vector3& boundsMin, const Vector3& boundsMax, const Transform& boundsTransform)
	{
		if (m_MouseControlBody == nullptr)
		{
			return false;
		}

		Box3 bodyBounds;
		bool hasBodyBounds = false;
		for (Geometry* geometry : m_MouseControlBody->Geometries())
		{
			if (geometry == nullptr)
			{
				continue;
			}
			const Transform geometryTransform = boundsTransform.TransformInv(
				m_MouseControlBody->GetGeometryTransform(geometry));
			const Box3 geometryBounds = Box3::Transform(geometry->GetShapeBounds(),
				geometryTransform.pos, geometryTransform.quat);
			if (hasBodyBounds)
			{
				bodyBounds.Encapsulate(geometryBounds);
			}
			else
			{
				bodyBounds = geometryBounds;
				hasBodyBounds = true;
			}
		}

		const Vector3 bodyPosition = boundsTransform.WorldToLocal(m_MouseControlBody->X);
		const Vector3 targetPosition = boundsTransform.WorldToLocal(worldPosition);
		Vector3 clampedPosition = targetPosition;
		if (hasBodyBounds)
		{
			for (int axis = 0; axis < 3; ++axis)
			{
				const float minimumPosition = boundsMin[axis]
					- (bodyBounds.Min[axis] - bodyPosition[axis]);
				const float maximumPosition = boundsMax[axis]
					- (bodyBounds.Max[axis] - bodyPosition[axis]);
				clampedPosition[axis] = minimumPosition <= maximumPosition
					? std::max(minimumPosition, std::min(targetPosition[axis], maximumPosition))
					: 0.5f * (boundsMin[axis] + boundsMax[axis]);
			}
		}
		return SetMouseControlPosition(boundsTransform.LocalToWorld(clampedPosition), deltaTime);
	}

	bool SceneWorld::RotateTank(const Vector3& center, const Quaternion& deltaRotation)
	{
		if (!std::isfinite(deltaRotation.x) || !std::isfinite(deltaRotation.y)
			|| !std::isfinite(deltaRotation.z) || !std::isfinite(deltaRotation.w)
			|| deltaRotation.SquareLength() <= 1.0e-12f)
		{
			return false;
		}
		const Quaternion rotation = deltaRotation.Unit();
		bool rotated = false;
		for (const SceneObjectInstance& object : m_Objects)
		{
			if (object.Id.compare(0, 5, "tank_") != 0 || object.GeometryPtr == nullptr)
			{
				continue;
			}
			RigidBody* body = object.GeometryPtr->GetParent<RigidBody>();
			if (body == nullptr || body->mRigidType != RigidType::Static)
			{
				continue;
			}
			body->X = center + rotation * (body->X - center);
			body->Q = (rotation * body->Q).Unit();
			rotated = true;
		}
		return rotated;
	}

	void SceneWorld::ReleaseMouseControl()
	{
		if (m_MouseControlBody == nullptr) return;
		if (RigidBodyDynamic* Dynamic = m_MouseControlBody->CastDynamic()) Dynamic->Wakeup();
	}

	std::vector<ConstraintControlState> SceneWorld::GetConstraintControls() const
	{
		std::vector<ConstraintControlState> states;
		states.reserve(m_ConstraintInstances.size());
		for (size_t index = 0; index < m_ConstraintInstances.size(); ++index)
		{
			PositionConstraint* constraint = m_ConstraintInstances[index].Constraint;
			if (constraint == nullptr)
			{
				continue;
			}
			ConstraintControlState state;
			state.Index = index;
			state.Name = m_ConstraintInstances[index].Name;
			state.Enabled = constraint->IsEnabled();
			state.Broken = constraint->IsBroken();
			state.ReactionForce = constraint->GetReaction().Force.Length();
			state.ReactionTorque = constraint->GetReaction().Torque.Length();
			switch (constraint->GetType())
			{
			case PositionConstraintType::Point:
				state.Type = ConstraintControlType::Point;
				break;
			case PositionConstraintType::Distance:
			{
				state.Type = ConstraintControlType::Distance;
				DistanceConstraint* distance = static_cast<DistanceConstraint*>(constraint);
				state.CurrentValue = distance->GetCurrentDistance();
				state.MinValue = distance->GetMinDistance();
				state.MaxValue = distance->GetMaxDistance();
				break;
			}
			case PositionConstraintType::Hinge:
			{
				state.Type = ConstraintControlType::Hinge;
				HingeConstraint* hinge = static_cast<HingeConstraint*>(constraint);
				state.CurrentValue = hinge->GetCurrentAngle() * RAD_TO_DEG;
				state.MinValue = hinge->GetMinAngle() * RAD_TO_DEG;
				state.MaxValue = hinge->GetMaxAngle() * RAD_TO_DEG;
				break;
			}
			case PositionConstraintType::Slider:
			{
				state.Type = ConstraintControlType::Slider;
				SliderConstraint* slider = static_cast<SliderConstraint*>(constraint);
				state.CurrentValue = slider->GetCurrentPosition();
				state.MinValue = slider->GetMinDistance();
				state.MaxValue = slider->GetMaxDistance();
				break;
			}
			case PositionConstraintType::Cone:
			{
				state.Type = ConstraintControlType::Cone;
				ConeConstraint* cone = static_cast<ConeConstraint*>(constraint);
				state.CurrentValue = cone->GetCurrentAngle() * RAD_TO_DEG;
				state.MaxValue = cone->GetMaxAngle() * RAD_TO_DEG;
				break;
			}
			case PositionConstraintType::SixDOF:
			{
				state.Type = ConstraintControlType::SixDOF;
				const SixDOFConstraintSettings& settings = static_cast<SixDOFConstraint*>(constraint)->GetSettings();
				state.LinearMin = settings.LinearMin;
				state.LinearMax = settings.LinearMax;
				state.AngularMinDegrees = settings.AngularMin * RAD_TO_DEG;
				state.AngularMaxDegrees = settings.AngularMax * RAD_TO_DEG;
				break;
			}
			case PositionConstraintType::Fixed:
			default:
				state.Type = ConstraintControlType::Fixed;
				break;
			}
			states.push_back(state);
		}
		return states;
	}

	bool SceneWorld::ApplyConstraintControl(const ConstraintControlState& state)
	{
		if (state.Index >= m_ConstraintInstances.size())
		{
			return false;
		}
		PositionConstraint* constraint = m_ConstraintInstances[state.Index].Constraint;
		if (constraint == nullptr)
		{
			return false;
		}
		constraint->SetEnabled(state.Enabled);
		if (state.RepairRequested) constraint->Repair();
		switch (constraint->GetType())
		{
		case PositionConstraintType::Distance:
			static_cast<DistanceConstraint*>(constraint)->SetDistanceLimits(state.MinValue, state.MaxValue);
			break;
		case PositionConstraintType::Hinge:
			static_cast<HingeConstraint*>(constraint)->SetAngleLimits(state.MinValue * DEG_TO_RAD, state.MaxValue * DEG_TO_RAD);
			break;
		case PositionConstraintType::Slider:
			static_cast<SliderConstraint*>(constraint)->SetDistanceLimits(state.MinValue, state.MaxValue);
			break;
		case PositionConstraintType::Cone:
			static_cast<ConeConstraint*>(constraint)->SetMaxAngle(state.MaxValue * DEG_TO_RAD);
			break;
		case PositionConstraintType::SixDOF:
		{
			SixDOFConstraintSettings settings;
			settings.LinearMin = state.LinearMin;
			settings.LinearMax = state.LinearMax;
			settings.AngularMin = state.AngularMinDegrees * DEG_TO_RAD;
			settings.AngularMax = state.AngularMaxDegrees * DEG_TO_RAD;
			static_cast<SixDOFConstraint*>(constraint)->SetSettings(settings);
			break;
		}
		default:
			break;
		}
		RigidBodyDynamic* dynamicA = constraint->GetBodyA() ? constraint->GetBodyA()->CastDynamic() : nullptr;
		RigidBodyDynamic* dynamicB = constraint->GetBodyB() ? constraint->GetBodyB()->CastDynamic() : nullptr;
		if (dynamicA) dynamicA->Wakeup();
		if (dynamicB) dynamicB->Wakeup();
		return true;
	}

	bool SceneWorld::LoadFromFile(const std::string& fileName, std::string* errorMessage, const std::string& configurationId)
	{
		std::ifstream file(fileName.c_str(), std::ios::in | std::ios::binary);
		if (!file)
		{
			if (errorMessage)
			{
				*errorMessage = "failed to open scene file: " + fileName;
			}
			return false;
		}

		std::ostringstream ss;
		ss << file.rdbuf();
		return LoadFromText(ss.str(), fileName, errorMessage, configurationId);
	}

	bool SceneWorld::LoadFromText(const std::string& text, const std::string& sourceName, std::string* errorMessage,
		const std::string& configurationId)
	{
		JsonValue root;
		JsonParser parser(text);
		if (!parser.Parse(&root, errorMessage))
		{
			if (errorMessage)
			{
				*errorMessage = sourceName + ": " + *errorMessage;
			}
			return false;
		}

		std::vector<SceneConfiguration> configurations;
		std::string selectedId;
		const JsonValue* selectedScene = nullptr;
		const JsonValue* entries = root.Find("configurations");
		const JsonValue* defaultConfiguration = root.Find("defaultConfiguration");
		auto Fail = [&](const std::string& message)
		{
			if (errorMessage) *errorMessage = sourceName + ": " + message;
			return false;
		};
		if (entries)
		{
			if (!entries->IsArray() || entries->arrayValue.empty())
				return Fail("configurations must be a nonempty array");
			if (defaultConfiguration && (!defaultConfiguration->IsString() || defaultConfiguration->stringValue.empty()))
				return Fail("defaultConfiguration must be a nonempty configuration id");
			for (const JsonValue& entry : entries->arrayValue)
			{
				const JsonValue* id = entry.Find("id");
				const JsonValue* name = entry.Find("name");
				const JsonValue* scene = entry.Find("scene");
				if (!id || !id->IsString() || id->stringValue.empty()
					|| !name || !name->IsString() || name->stringValue.empty() || !scene || !scene->IsObject())
					return Fail("each configuration requires a nonempty id, name and scene object");
				for (const SceneConfiguration& configuration : configurations)
					if (configuration.Id == id->stringValue) return Fail("duplicate configuration id: " + configuration.Id);
				configurations.push_back({ id->stringValue, name->stringValue });
				if (selectedId.empty()) selectedId = configurationId.empty()
					? (defaultConfiguration ? defaultConfiguration->stringValue : id->stringValue) : configurationId;
				if (id->stringValue == selectedId) selectedScene = scene;
			}
			if (defaultConfiguration && std::none_of(configurations.begin(), configurations.end(), [&](const SceneConfiguration& configuration)
				{ return configuration.Id == defaultConfiguration->stringValue; }))
				return Fail("unknown defaultConfiguration: " + defaultConfiguration->stringValue);
			if (!selectedScene) return Fail("unknown configuration id: " + selectedId);
		}
		else if (!configurationId.empty() || defaultConfiguration)
		{
			return Fail("configuration selection requires a configurations array");
		}

		JsonValue scene = root;
		scene.objectValue.erase("configurations");
		scene.objectValue.erase("defaultConfiguration");
		if (selectedScene)
		{
			// Configuration sections replace shared sections; object arrays are never concatenated.
			for (const auto& section : selectedScene->objectValue)
				scene.objectValue[section.first] = section.second;
		}
		m_BaseDirectory = DirectoryOf(sourceName);
		if (!LoadParsedScene(scene, sourceName, errorMessage)) return false;
		m_Configurations = std::move(configurations);
		m_ActiveConfigurationId = selectedId;
		return true;
	}

	bool SceneWorld::LoadParsedScene(const JsonValue& root, const std::string& sourceName, std::string* errorMessage)
	{
		if (!root.IsObject())
		{
			if (errorMessage)
			{
				*errorMessage = sourceName + ": root must be a JSON object";
			}
			return false;
		}

		const JsonValue* physics = root.Find("physics");
		PhysicsWorldParam worldParam;
		worldParam.collisionFilteringEnabled = false;
		worldParam.gravityAcc = Vector3(0.0f, -9.8f, 0.0f);
		if (physics && physics->IsObject())
		{
			worldParam.gravityAcc = ReadVector3(physics->Find("gravity"), worldParam.gravityAcc);
			const JsonValue* Filtering = physics->Find("collisionFiltering");
			if (Filtering && !Filtering->IsBool())
			{
				if (errorMessage) *errorMessage = sourceName + ": collisionFiltering must be a boolean";
				return false;
			}
			worldParam.collisionFilteringEnabled = ReadBool(Filtering, worldParam.collisionFilteringEnabled);
			const JsonValue* Unified = physics->Find("unifiedConstraints");
			if (Unified && !Unified->IsBool())
			{
				if (errorMessage) *errorMessage = sourceName + ": unifiedConstraints must be a boolean";
				return false;
			}
			worldParam.unifiedConstraints = ReadBool(Unified, worldParam.unifiedConstraints);
			const JsonValue* Friction = physics->Find("frictionPatches");
			if (Friction && !Friction->IsBool())
			{
				if (errorMessage) *errorMessage = sourceName + ": frictionPatches must be a boolean";
				return false;
			}
			worldParam.frictionPatches.Enabled = ReadBool(Friction, worldParam.frictionPatches.Enabled);
			const JsonValue* Drives = physics->Find("jointDrives");
			if (Drives && !Drives->IsBool())
			{
				if (errorMessage) *errorMessage = sourceName + ": jointDrives must be a boolean";
				return false;
			}
			worldParam.jointDrives = ReadBool(Drives, worldParam.jointDrives);
			const JsonValue* Breaking = physics->Find("jointBreaking");
			if (Breaking && !Breaking->IsBool())
			{
				if (errorMessage) *errorMessage = sourceName + ": jointBreaking must be a boolean";
				return false;
			}
			worldParam.jointBreaking = ReadBool(Breaking, worldParam.jointBreaking);
			const JsonValue* Sleeping = physics->Find("islandSleeping");
			if (Sleeping)
			{
				bool Valid = Sleeping->IsObject(); auto& Settings = worldParam.islandSleeping;
				const JsonValue* Enabled = Sleeping->Find("enabled");
				if (Enabled) { Valid = Valid && Enabled->IsBool(); Settings.Enabled = ReadBool(Enabled, false); }
				auto Scalar = [&](const char* Name, float& Value)
				{
					const JsonValue* Field = Sleeping->Find(Name); if (!Field) return;
					Valid = Valid && Field->IsNumber(); if (Field->IsNumber()) Value = (float)Field->numberValue;
				};
				Scalar("quietTime", Settings.QuietTime); Scalar("linearThreshold", Settings.LinearThreshold);
				Scalar("angularThreshold", Settings.AngularThreshold); Scalar("hysteresisRatio", Settings.HysteresisRatio);
				Scalar("contactTolerance", Settings.ContactTolerance);
				if (!Valid || !Settings.IsValid())
				{
					if (errorMessage) *errorMessage = sourceName + ": invalid islandSleeping settings";
					return false;
				}
			}
			const JsonValue* Support = physics->Find("supportStabilization");
			if (Support)
			{
				bool Valid = Support->IsObject();
				auto& Settings = worldParam.supportStabilization;
				const JsonValue* Enabled = Support->Find("enabled");
				if (Enabled) { Valid = Valid && Enabled->IsBool(); Settings.Enabled = ReadBool(Enabled, false); }
				auto Count = [&](const char* Name, int& Value)
				{
					const JsonValue* Field = Support->Find(Name); if (!Field) return;
					const bool Integer = Field->IsNumber() && std::isfinite(Field->numberValue) && Field->numberValue >= 1
						&& Field->numberValue <= INT_MAX && std::floor(Field->numberValue) == Field->numberValue;
					Valid = Valid && Integer; if (Integer) Value = (int)Field->numberValue;
				};
				auto Scalar = [&](const char* Name, float& Value)
				{
					const JsonValue* Field = Support->Find(Name); if (!Field) return;
					Valid = Valid && Field->IsNumber(); if (Field->IsNumber()) Value = (float)Field->numberValue;
				};
				Count("positionIterations", Settings.PositionIterations); Count("velocityIterations", Settings.VelocityIterations);
				Scalar("parentInverseMassScale", Settings.ParentInverseMassScale); Scalar("contactTolerance", Settings.ContactTolerance);
				Scalar("maximumPositionCorrection", Settings.MaximumPositionCorrection); Scalar("maximumAngularCorrection", Settings.MaximumAngularCorrection);
				if (!Valid || !Settings.IsValid())
				{
					if (errorMessage) *errorMessage = sourceName + ": invalid supportStabilization settings";
					return false;
				}
			}
			const JsonValue* Temporal = physics->Find("temporalSubsteps");
			if (Temporal)
			{
				bool Valid = Temporal->IsObject();
				const JsonValue* Enabled = Temporal->Find("enabled");
				if (Enabled) { Valid = Valid && Enabled->IsBool(); worldParam.temporalSubsteps.Enabled = ReadBool(Enabled, false); }
				const JsonValue* Count = Temporal->Find("substeps");
				if (Count)
				{
					const bool Integer = Count->IsNumber() && std::isfinite(Count->numberValue)
						&& Count->numberValue >= 1 && Count->numberValue <= TemporalSubstepSettings::MaximumSubsteps
						&& std::floor(Count->numberValue) == Count->numberValue;
					Valid = Valid && Integer; if (Integer) worldParam.temporalSubsteps.Substeps = (int)Count->numberValue;
				}
				if (!Valid)
				{
					if (errorMessage) *errorMessage = sourceName + ": invalid temporalSubsteps settings";
					return false;
				}
			}
			const JsonValue* Sequential = physics->Find("sequentialImpulse");
			if (Sequential)
			{
				bool Valid = Sequential->IsObject();
				auto ReadCount = [&](const char* Name, int& Value)
				{
					const JsonValue* Field = Sequential->Find(Name); if (!Field) return;
					const bool Integer = Field->IsNumber() && std::isfinite(Field->numberValue)
						&& Field->numberValue >= 1 && Field->numberValue <= INT_MAX && std::floor(Field->numberValue) == Field->numberValue;
					Valid = Valid && Integer; if (Integer) Value = (int)Field->numberValue;
				};
				ReadCount("positionIterations", worldParam.sequentialImpulseSettings.PositionIterations);
				ReadCount("velocityIterations", worldParam.sequentialImpulseSettings.VelocityIterations);
				if (!Valid)
				{
					if (errorMessage) *errorMessage = sourceName + ": invalid sequentialImpulse settings";
					return false;
				}
			}
			const JsonValue* CCD = physics->Find("ccdContinuation");
			if (CCD)
			{
				bool Valid = CCD->IsObject();
				auto& Settings = worldParam.ccdContinuation;
				const JsonValue* Enabled = CCD->Find("enabled");
				if (Enabled) { Valid = Valid && Enabled->IsBool(); Settings.Enabled = ReadBool(Enabled, false); }
				auto ReadCount = [&](const char* Name, int& Value)
				{
					const JsonValue* Field = CCD->Find(Name); if (!Field) return;
					const bool Integer = Field->IsNumber() && std::isfinite(Field->numberValue)
						&& Field->numberValue >= 1 && Field->numberValue <= INT_MAX
						&& std::floor(Field->numberValue) == Field->numberValue;
					Valid = Valid && Integer; if (Integer) Value = (int)Field->numberValue;
				};
				auto ReadTolerance = [&](const char* Name, float& Value)
				{
					const JsonValue* Field = CCD->Find(Name); if (!Field) return;
					Valid = Valid && Field->IsNumber() && std::isfinite(Field->numberValue) && std::abs(Field->numberValue) <= FLT_MAX;
					Value = ReadFloat(Field, Value);
				};
				ReadCount("maxPasses", Settings.MaxPasses); ReadCount("impactIterations", Settings.ImpactIterations);
				ReadTolerance("simultaneousTolerance", Settings.SimultaneousTolerance);
				ReadTolerance("velocityTolerance", Settings.VelocityTolerance);
				ReadTolerance("contactTolerance", Settings.ContactTolerance);
				ReadTolerance("restitutionThreshold", Settings.RestitutionThreshold);
				if (!Valid || !Settings.IsValid())
				{
					if (errorMessage) *errorMessage = sourceName + ": invalid ccdContinuation settings";
					return false;
				}
			}
			const JsonValue* Contacts = physics->Find("contactGeneration");
			if (Contacts)
			{
				bool Valid = Contacts->IsObject();
				auto ReadFlag = [&](const char* Name, bool& Value)
				{
					const JsonValue* Field = Contacts->Find(Name);
					if (!Field) return;
					Valid = Valid && Field->IsBool(); Value = ReadBool(Field, Value);
				};
				auto ReadLength = [&](const char* Name, float& Value)
				{
					const JsonValue* Field = Contacts->Find(Name);
					if (!Field) return;
					Valid = Valid && Field->IsNumber() && std::isfinite(Field->numberValue) && std::abs(Field->numberValue) <= FLT_MAX;
					Value = ReadFloat(Field, Value);
				};
				ReadFlag("useShapeOffsets", worldParam.contactGeneration.UseShapeOffsets);
				ReadFlag("speculativeContacts", worldParam.contactGeneration.SpeculativeContacts);
				ReadLength("lengthScale", worldParam.contactGeneration.LengthScale);
				ReadLength("relativeTolerance", worldParam.contactGeneration.RelativeTolerance);
				ReadLength("minimumTolerance", worldParam.contactGeneration.MinimumTolerance);
				ReadLength("relativePenetrationSlop", worldParam.contactGeneration.RelativePenetrationSlop);
				ReadLength("maximumSpeculativeDistance", worldParam.contactGeneration.MaximumSpeculativeDistance);
				ReadLength("faceClipSquaredDistance", worldParam.contactGeneration.FaceClipSquaredDistance);
				const JsonValue* Iterations = Contacts->Find("distanceIterations");
				if (Iterations)
				{
					const bool Integer = Iterations->IsNumber() && std::isfinite(Iterations->numberValue)
						&& Iterations->numberValue >= 1 && Iterations->numberValue <= INT_MAX
						&& std::floor(Iterations->numberValue) == Iterations->numberValue;
					Valid = Valid && Integer;
					if (Integer) worldParam.contactGeneration.DistanceIterations = (int)Iterations->numberValue;
				}
				if (!Valid || !worldParam.contactGeneration.IsValid())
				{
					if (errorMessage) *errorMessage = sourceName + ": invalid contactGeneration settings";
					return false;
				}
			}
			worldParam.resolutionSolver = ReadResolutionSolver(
				physics->Find("resolutionSolver"), worldParam.resolutionSolver);
			worldParam.solverBackend = ReadSolverBackend(
				physics->Find("solverBackend"), worldParam.solverBackend);
			const JsonValue* fallback = physics->Find("allowBackendFallback");
			if (fallback && !fallback->IsBool())
			{
				if (errorMessage) *errorMessage = sourceName + ": allowBackendFallback must be a boolean";
				return false;
			}
			worldParam.allowBackendFallback = ReadBool(fallback, worldParam.allowBackendFallback);
			if (worldParam.resolutionSolver == ResolutionPhaseSolver::PBD || worldParam.resolutionSolver == ResolutionPhaseSolver::XPBD)
			{
				const JsonValue* requested = physics->Find("solverBackend");
				if (requested && ReadSolverBackend(requested, static_cast<SolverBackend>(255)) == static_cast<SolverBackend>(255))
				{
					if (errorMessage) *errorMessage = sourceName + ": unknown PBD/XPBD solverBackend";
					return false;
				}
			}
			worldParam.contactSolverIterations = std::max(1, ReadInt(
				physics->Find("contactSolverIterations"), worldParam.contactSolverIterations));
			worldParam.contactSolverTolerance = std::max(0.0f, ReadFloat(
				physics->Find("contactSolverTolerance"), worldParam.contactSolverTolerance));
			worldParam.contactSolverRelaxation = std::max(0.01f, std::min(1.99f, ReadFloat(
				physics->Find("contactSolverRelaxation"), worldParam.contactSolverRelaxation)));
			// Keep both formulations configured for in-place solver changes in the Viewer.
			for (int formulation = 0; formulation < 2; ++formulation)
			{
				const bool useXPBD = formulation == 1;
				const JsonValue* pbd = physics->Find(useXPBD ? "xpbd" : "pbd");
				if (pbd && pbd->IsObject())
				{
					PBDSolverSettings& settings = useXPBD
						? static_cast<PBDSolverSettings&>(worldParam.xpbdSettings) : worldParam.pbdSettings;
					auto ReadIterations = [](const JsonValue* value, int fallback) -> int
					{
						return value && value->IsNumber() && std::isfinite(value->numberValue)
							? (int)std::max(1.0, std::min(256.0, value->numberValue)) : fallback;
					};
					settings.PositionIterations = ReadIterations(pbd->Find("positionIterations"), settings.PositionIterations);
					settings.VelocityIterations = ReadIterations(pbd->Find("velocityIterations"), settings.VelocityIterations);
					settings.PenetrationSlop = ReadFloat(pbd->Find("penetrationSlop"), settings.PenetrationSlop);
					settings.MaxPositionCorrection = ReadFloat(pbd->Find("maxPositionCorrection"), settings.MaxPositionCorrection);
					settings.MaxAngularCorrection = ReadFloat(pbd->Find("maxAngularCorrection"), settings.MaxAngularCorrection);
					settings.RestitutionThreshold = ReadFloat(pbd->Find("restitutionThreshold"), settings.RestitutionThreshold);
					if (useXPBD)
						worldParam.xpbdSettings.ContactCompliance = ReadFloat(pbd->Find("contactCompliance"), worldParam.xpbdSettings.ContactCompliance);
				}
			}
		}
		try { ResetWorld(worldParam); }
		catch (const std::runtime_error& error)
		{
			if (errorMessage) *errorMessage = sourceName + ": " + error.what();
			return false;
		}
		m_Name = ReadString(root.Find("name"), sourceName);
		m_FieldSimulation = ReadFieldSimulation(root.Find("fieldSimulation"));

		const JsonValue* camera = root.Find("camera");
		if (camera && camera->IsObject())
		{
			m_Camera.At = ReadVector3(camera->Find("center"), m_Camera.At);
			const Vector3 orbit = ReadVector3(camera->Find("orbit"), Vector3(1.0f, 0.6f, 15.0f));
			m_Camera.Eye = m_Camera.At + Vector3(sinf(orbit.x) * cosf(orbit.y), sinf(orbit.y), cosf(orbit.x) * cosf(orbit.y)) * orbit.z;
			m_Camera.FovY = ReadFloat(camera->Find("fovY"), m_Camera.FovY);
			if (!ReadFiniteNumber(*camera, "sidebarPadding", m_CameraSidebarPadding) || m_CameraSidebarPadding < 0)
			{
				if (errorMessage) *errorMessage = sourceName + ": invalid camera sidebarPadding";
				return false;
			}
			m_Camera.NearPlane = ReadFloat(camera->Find("near"), m_Camera.NearPlane);
			m_Camera.FarPlane = ReadFloat(camera->Find("far"), m_Camera.FarPlane);
			m_Camera.BackgroundColor = ReadColor(
				camera->Find("backgroundColor"), m_Camera.BackgroundColor);
		}

		const JsonValue* light = root.Find("light");
		if (light && light->IsObject())
		{
			m_Light.Direction = ReadVector3(light->Find("direction"), m_Light.Direction);
			m_Light.Color = ReadVector3(light->Find("color"), m_Light.Color);
			m_Light.ShadowCenter = ReadVector3(light->Find("shadowCenter"), m_Light.ShadowCenter);
			m_Light.Ambient = ReadFloat(light->Find("ambient"), m_Light.Ambient);
			m_Light.ShadowDistance = ReadFloat(light->Find("shadowDistance"), m_Light.ShadowDistance);
			m_Light.ShadowSize = ReadFloat(light->Find("shadowSize"), m_Light.ShadowSize);
			m_Light.ShadowsEnabled = ReadBool(
				light->Find("shadowsEnabled"), m_Light.ShadowsEnabled);
		}

		const JsonValue* objects = root.Find("objects");
		if (objects == nullptr || !objects->IsArray())
		{
			if (errorMessage)
			{
				*errorMessage = sourceName + ": scene requires an objects array";
			}
			return false;
		}

		int unnamedIndex = 0;
		for (const JsonValue& object : objects->arrayValue)
		{
			if (!object.IsObject())
			{
				if (errorMessage)
				{
					*errorMessage = sourceName + ": every object entry must be a JSON object";
				}
				return false;
			}

			SceneObjectDesc base = ReadObjectDesc(object);
			if (base.Id.empty())
			{
				std::ostringstream ss;
				ss << "object_" << unnamedIndex++;
				base.Id = ss.str();
			}

			const int total = base.RepeatCount[0] * base.RepeatCount[1] * base.RepeatCount[2];
			for (int ix = 0; ix < base.RepeatCount[0]; ++ix)
			{
				for (int iy = 0; iy < base.RepeatCount[1]; ++iy)
				{
					for (int iz = 0; iz < base.RepeatCount[2]; ++iz)
					{
						SceneObjectDesc desc = base;
						const Vector3 offset(
							desc.RepeatSpacing.x * ix,
							desc.RepeatSpacing.y * iy,
							desc.RepeatSpacing.z * iz);
						desc.Position += offset;
						if (desc.HasX0)
						{
							desc.X0 += offset;
						}
						if (desc.HasX1)
						{
							desc.X1 += offset;
						}

						if (total > 1)
						{
							std::ostringstream ss;
							ss << base.Id << "_" << ix << "_" << iy << "_" << iz;
							desc.Id = ss.str();
						}

						if (!CreateObject(desc, errorMessage))
						{
							if (errorMessage)
							{
								*errorMessage = sourceName + ": " + *errorMessage;
							}
							return false;
						}
					}
				}
			}
		}

		const JsonValue* constraints = root.Find("constraints");
		const JsonValue* stress = root.Find("stress");
		if (stress && stress->IsObject() && ReadBool(stress->Find("enabled"), true))
		{
			StrainSolverSettings settings;
			settings.Hardness = ReadFloat(stress->Find("hardness"), settings.Hardness);
			settings.MaxIterations = (uint32_t)std::max(1, ReadInt(stress->Find("iterations"), 128));
			settings.RelativeTolerance = ReadFloat(stress->Find("tolerance"), settings.RelativeTolerance);
			const Vector3 gravity = ReadVector3(stress->Find("gravity"), Vector3::Zero());
			for (const auto& set : m_DestructionSets) set->EnableStressSolver(settings, gravity);
		}
		if (m_DestructionEventsDemo || m_DestructionCacheDemo)
		{
			DestructionEventSettings Settings; Settings.Enabled = true;
			for (const auto& Set : m_DestructionSets) Set->ConfigureEvents(Settings);
		}
		if (constraints && constraints->IsArray())
		{
			auto findBody = [this](const std::string& id) -> RigidBody*
			{
				const std::string compoundPrefix = id + "_sphere_";
				for (const SceneObjectInstance& instance : m_Objects)
				{
					if ((instance.Id == id || instance.Id.compare(0, compoundPrefix.size(), compoundPrefix) == 0)
						&& instance.GeometryPtr)
					{
						return instance.GeometryPtr->GetParent<RigidBody>();
					}
				}
				return nullptr;
			};

			for (const JsonValue& constraint : constraints->arrayValue)
			{
				if (!constraint.IsObject())
				{
					if (errorMessage) *errorMessage = sourceName + ": every constraint entry must be a JSON object";
					return false;
				}
				const std::string type = ReadString(constraint.Find("type"), std::string());
				const std::string bodyAName = ReadString(constraint.Find("bodyA"), std::string());
				const std::string bodyBName = ReadString(constraint.Find("bodyB"), std::string());
				RigidBody* bodyA = findBody(bodyAName);
				RigidBody* bodyB = findBody(bodyBName);
				if (bodyA == nullptr || bodyB == nullptr)
				{
					if (errorMessage) *errorMessage = sourceName + ": constraint references an unknown body";
					return false;
				}

				const Vector3 defaultAnchor = (bodyA->X + bodyB->X) * 0.5f;
				const Vector3 anchor = ReadVector3(constraint.Find("anchor"), defaultAnchor);
				const Vector3 axis = SafeUnitVector(ReadVector3(constraint.Find("axis"), Vector3::UnitY()), Vector3::UnitY());
				PositionConstraint* created = nullptr;
				if (type == "fixed")
				{
					created = m_World->CreateFixedConstraint(bodyA, bodyB);
				}
				else if (type == "point" || type == "ballSocket")
				{
					created = m_World->CreatePointConstraint(bodyA, bodyB, anchor);
				}
				else if (type == "distance")
				{
					const Vector3 anchorA = ReadVector3(constraint.Find("anchorA"), bodyA->X);
					const Vector3 anchorB = ReadVector3(constraint.Find("anchorB"), bodyB->X);
					created = m_World->CreateDistanceConstraint(bodyA, bodyB, anchorA, anchorB,
						ReadFloat(constraint.Find("minDistance"), 0.0f), ReadFloat(constraint.Find("maxDistance"), (anchorB - anchorA).Length()));
				}
				else if (type == "hinge")
				{
					created = m_World->CreateHingeConstraint(bodyA, bodyB, anchor, axis,
						ToRadians(ReadFloat(constraint.Find("minAngleDegrees"), -180.0f)), ToRadians(ReadFloat(constraint.Find("maxAngleDegrees"), 180.0f)));
				}
				else if (type == "slider")
				{
					created = m_World->CreateSliderConstraint(bodyA, bodyB, anchor, axis,
						ReadFloat(constraint.Find("minDistance"), -1.0f), ReadFloat(constraint.Find("maxDistance"), 1.0f));
				}
				else if (type == "cone")
				{
					const Vector3 axisA = SafeUnitVector(ReadVector3(constraint.Find("axisA"), axis), axis);
					const Vector3 axisB = SafeUnitVector(ReadVector3(constraint.Find("axisB"), axis), axis);
					created = m_World->CreateConeConstraint(bodyA, bodyB, anchor, axisA, axisB,
						ToRadians(ReadFloat(constraint.Find("maxAngleDegrees"), 45.0f)));
				}
				else if (type == "sixDOF")
				{
					SixDOFConstraintSettings settings;
					settings.LinearMin = ReadVector3(constraint.Find("linearMin"), settings.LinearMin);
					settings.LinearMax = ReadVector3(constraint.Find("linearMax"), settings.LinearMax);
					settings.AngularMin = ReadVector3(constraint.Find("angularMinDegrees"), settings.AngularMin) * PI_OVER_180;
					settings.AngularMax = ReadVector3(constraint.Find("angularMaxDegrees"), settings.AngularMax) * PI_OVER_180;
					created = m_World->CreateSixDOFConstraint(bodyA, bodyB, anchor, settings);
				}

				if (created == nullptr)
				{
					if (errorMessage) *errorMessage = sourceName + ": unsupported or invalid constraint type '" + type + "'";
					return false;
				}
				created->SetCompliance(ReadFloat(constraint.Find("linearCompliance"), 0.0f),
					ReadFloat(constraint.Find("angularCompliance"), 0.0f));
				if (!ConfigureJointDrives(constraint, created))
				{
					if (errorMessage) *errorMessage = sourceName + ": invalid drive configuration for '" + type + "'";
					return false;
				}
				if (const JsonValue* Breaking = constraint.Find("breaking"))
				{
					JointBreakSettings Settings;
					if (!Breaking->IsObject() || !ReadTypedBool(*Breaking, "enabled", Settings.Enabled)
						|| !ReadFiniteNumber(*Breaking, "force", Settings.Force)
						|| !ReadFiniteNumber(*Breaking, "torque", Settings.Torque) || !Settings.IsValid())
					{
						if (errorMessage) *errorMessage = sourceName + ": invalid joint breaking settings";
						return false;
					}
					created->SetBreakSettings(Settings);
				}
				ConstraintInstance constraintInstance;
				constraintInstance.Name = type + " - " + bodyBName;
				constraintInstance.Constraint = created;
				m_ConstraintInstances.push_back(constraintInstance);
				if (ReadBool(constraint.Find("disableCollision"), false))
				{
					bodyA->IgnoreCollisionWith(bodyB);
				}
				if (const JsonValue* Visual = constraint.Find("springVisual"))
				{
					float Radius = 0.3f, WireRadius = 0.05f, Turns = 8;
					float AnchorA[3] = { bodyA->X.x, bodyA->X.y, bodyA->X.z };
					float AnchorB[3] = { bodyB->X.x, bodyB->X.y, bodyB->X.z };
					if (!Visual->IsObject() || !ReadFiniteNumber(*Visual, "radius", Radius)
						|| !ReadFiniteNumber(*Visual, "wireRadius", WireRadius) || !ReadFiniteNumber(*Visual, "turns", Turns)
						|| !ReadDriveVector(Visual->Find("anchorA"), AnchorA, 3) || !ReadDriveVector(Visual->Find("anchorB"), AnchorB, 3)
						|| Radius <= 0 || WireRadius <= 0 || WireRadius >= Radius || Turns < 1 || Turns > 64 || std::floor(Turns) != Turns)
					{
						if (errorMessage) *errorMessage = sourceName + ": invalid springVisual settings";
						return false;
					}
					RopeSurfaceState Surface;
					Surface.SpringBodyA = bodyA; Surface.SpringBodyB = bodyB;
					Surface.SpringAnchorA = bodyA->WorldToBodyLocal(Vector3(AnchorA[0], AnchorA[1], AnchorA[2]));
					Surface.SpringAnchorB = bodyB->WorldToBodyLocal(Vector3(AnchorB[0], AnchorB[1], AnchorB[2]));
					Surface.SpringRadius = Radius; Surface.SpringTurns = (int)Turns; Surface.Radius = WireRadius;
					Surface.Surface.reset(GeometryFactory::CreateTriangleMesh());
					Surface.Surface->SetSimulationEnabled(false); Surface.Surface->SetQueryEnabled(false);
					SceneObjectInstance Object;
					Object.Id = bodyBName + "_spring_" + std::to_string(m_ConstraintInstances.size());
					Object.GeometryPtr = Surface.Surface.get();
					Object.Color = ReadColor(Visual->Find("color"), Vector4(0.75f, 0.8f, 0.85f, 1));
					m_Objects.push_back(Object); m_RopeSurfaces.push_back(std::move(Surface));
				}
			}
			SyncRopeSurfaces();
		}

		const JsonValue* ropes = root.Find("ropes");
		if (ropes && ropes->IsArray())
		{
			for (const JsonValue& ropeValue : ropes->arrayValue)
			{
				if (!ropeValue.IsObject())
				{
					if (errorMessage) *errorMessage = sourceName + ": every rope entry must be a JSON object";
					return false;
				}
				const std::string id = ReadString(ropeValue.Find("id"), "rope");
				const Vector3 start = ReadVector3(ropeValue.Find("start"), Vector3::Zero());
				const Vector3 end = ReadVector3(ropeValue.Find("end"), start + Vector3::UnitY());
				RopeSettings settings;
				settings.SegmentCount = std::max(1, ReadInt(ropeValue.Find("segments"), settings.SegmentCount));
				settings.SolverIterations = std::max(1, ReadInt(ropeValue.Find("iterations"), settings.SolverIterations));
				settings.ParticleRadius = ReadFloat(ropeValue.Find("radius"), settings.ParticleRadius);
				settings.RestLength = ReadFloat(ropeValue.Find("restLength"), settings.RestLength);
				settings.StretchCompliance = ReadFloat(ropeValue.Find("compliance"), settings.StretchCompliance);
				settings.TensionOnly = ReadBool(ropeValue.Find("tensionOnly"), settings.TensionOnly);
				settings.ContinuousCollision = ReadBool(ropeValue.Find("continuousCollision"), settings.ContinuousCollision);
				settings.CollisionFriction = ReadFloat(ropeValue.Find("collisionFriction"), settings.CollisionFriction);
				settings.ParticleInverseMass = ReadFloat(ropeValue.Find("inverseMass"), settings.ParticleInverseMass);
				settings.Damping = ReadFloat(ropeValue.Find("damping"), settings.Damping);
				settings.Gravity = ReadVector3(ropeValue.Find("gravity"), settings.Gravity);
				RopeSystem* rope = m_World->CreateRope(settings, start, end);
				const JsonValue* colliders = ropeValue.Find("colliders");
				if (colliders && colliders->IsArray())
				{
					for (const JsonValue& collider : colliders->arrayValue)
					{
						const std::string colliderName = ReadString(&collider, std::string());
						bool added = false;
						for (const SceneObjectInstance& instance : m_Objects)
						{
							if (instance.Id == colliderName && instance.GeometryPtr)
							{
								added = rope->AddCollider(instance.GeometryPtr);
								break;
							}
						}
						if (!added)
						{
							if (errorMessage) *errorMessage = sourceName + ": invalid rope sphere or box collider '" + colliderName + "'";
							return false;
						}
					}
				}
				if (ReadBool(ropeValue.Find("pinStart"), true))
				{
					rope->PinParticle(0, start);
				}

				const std::string bodyName = ReadString(ropeValue.Find("body"), std::string());
				if (!bodyName.empty())
				{
					RigidBody* body = nullptr;
					for (const SceneObjectInstance& instance : m_Objects)
					{
						if (instance.Id == bodyName && instance.GeometryPtr)
						{
							body = instance.GeometryPtr->GetParent<RigidBody>();
							break;
						}
					}
					if (body == nullptr)
					{
						if (errorMessage) *errorMessage = sourceName + ": rope references an unknown body '" + bodyName + "'";
						return false;
					}
					const int particleIndex = ReadInt(ropeValue.Find("particle"), rope->GetParticleCount() - 1);
					const Vector3 bodyAnchor = ReadVector3(ropeValue.Find("bodyAnchor"), body->X);
					if (!rope->AttachRigidBody(particleIndex, body, bodyAnchor))
					{
						if (errorMessage) *errorMessage = sourceName + ": invalid rope body attachment";
						return false;
					}
				}

				const std::string mouseBodyName = ReadString(ropeValue.Find("mouseBody"), std::string());
				if (!mouseBodyName.empty())
				{
					const int mouseParticle = ReadInt(ropeValue.Find("mouseParticle"), 0);
					for (const SceneObjectInstance& instance : m_Objects)
					{
						if (instance.Id == mouseBodyName && instance.GeometryPtr)
						{
							RegisterMouseControlGeometry(instance.GeometryPtr);
							m_MouseControlRope = rope;
							m_MouseControlRopeParticle = mouseParticle;
							break;
						}
					}
					if (m_MouseControlBody == nullptr || mouseParticle < 0 || mouseParticle >= rope->GetParticleCount())
					{
						if (errorMessage) *errorMessage = sourceName + ": rope has an invalid mouse control target";
						return false;
					}
					rope->PinParticle(mouseParticle, m_MouseControlBody->X);
				}

				const Vector4 color = ReadColor(ropeValue.Find("color"), Vector4(0.72f, 0.52f, 0.22f, 1.0f));
				if (ReadString(ropeValue.Find("renderStyle"), "particles") == "tube")
				{
					RopeSurfaceState surface;
					surface.Rope = rope;
					surface.Radius = std::isfinite(settings.ParticleRadius) ? std::max(0.001f, settings.ParticleRadius) : 0.08f;
					const JsonValue* sides = ropeValue.Find("tubeSides");
					if (sides && sides->IsNumber() && std::isfinite(sides->numberValue))
						surface.Sides = (int)std::max(4.0, std::min(32.0, sides->numberValue));
					surface.Surface.reset(GeometryFactory::CreateTriangleMesh());
					surface.Surface->SetSimulationEnabled(false);
					surface.Surface->SetQueryEnabled(false);
					SceneObjectInstance instance;
					instance.Id = id + "_surface";
					instance.GeometryPtr = surface.Surface.get();
					instance.Color = color;
					m_Objects.push_back(instance);
					m_RopeSurfaces.push_back(std::move(surface));
					continue;
				}
				const std::vector<Geometry*>& renderGeometries = rope->GetRenderGeometries();
				for (size_t i = 0; i < renderGeometries.size(); ++i)
				{
					SceneObjectInstance instance;
					std::ostringstream name;
					name << id << "_particle_" << i;
					instance.Id = name.str();
					instance.GeometryPtr = renderGeometries[i];
					instance.Color = color;
					m_Objects.push_back(instance);
				}
			}
			SyncRopeSurfaces();
			TouchRenderRevision();
		}

		const JsonValue* cloths = root.Find("cloths");
		if (cloths && cloths->IsArray())
		{
			for (const JsonValue& clothValue : cloths->arrayValue)
			{
				if (!clothValue.IsObject())
				{
					if (errorMessage) *errorMessage = sourceName + ": every cloth entry must be a JSON object";
					return false;
				}
				ClothSettings settings;
				settings.Rows = std::max(2, ReadInt(clothValue.Find("rows"), settings.Rows));
				settings.Columns = std::max(2, ReadInt(clothValue.Find("columns"), settings.Columns));
				settings.SolverIterations = std::max(1, ReadInt(clothValue.Find("iterations"), settings.SolverIterations));
				settings.Width = ReadFloat(clothValue.Find("width"), settings.Width);
				settings.Height = ReadFloat(clothValue.Find("height"), settings.Height);
				settings.Thickness = ReadFloat(clothValue.Find("thickness"), settings.Thickness);
				settings.Damping = ReadFloat(clothValue.Find("damping"), settings.Damping);
				settings.Gravity = ReadVector3(clothValue.Find("gravity"), settings.Gravity);
				settings.Model = ReadString(clothValue.Find("model"), "distanceLinks") == "triangleStrain" ? ClothModel::TriangleStrain : ClothModel::DistanceLinks;
				settings.Substeps = std::max(1, ReadInt(clothValue.Find("substeps"), settings.Substeps));
				settings.UseSurfaceDensity = clothValue.Find("surfaceDensity") != nullptr;
				settings.SurfaceDensity = ReadFloat(clothValue.Find("surfaceDensity"), settings.SurfaceDensity);
				settings.Collision.RigidCoupling = ReadBool(clothValue.Find("rigidCoupling"), false);
				settings.AerodynamicsEnabled = ReadBool(clothValue.Find("aerodynamics"), true);
				settings.Material.StretchUCompliance = ReadFloat(clothValue.Find("stretchUCompliance"), settings.Material.StretchUCompliance);
				settings.Material.StretchVCompliance = ReadFloat(clothValue.Find("stretchVCompliance"), settings.Material.StretchVCompliance);
				settings.Material.ShearCompliance = ReadFloat(clothValue.Find("shearCompliance"), settings.Material.ShearCompliance);
				settings.Material.BendingCompliance = ReadFloat(clothValue.Find("bendingCompliance"), settings.Material.BendingCompliance);
				const Vector3 center = ReadVector3(clothValue.Find("center"), Vector3::Zero());
				const Vector3 horizontalAxis = ReadVector3(clothValue.Find("horizontalAxis"), Vector3::UnitX());
				const Vector3 verticalAxis = ReadVector3(clothValue.Find("verticalAxis"), Vector3::UnitY());
				const bool ModernDemo = m_ClothModelDemo && m_Name == "Cloth" && !m_DemoCloth;
				const bool PinTop = ReadBool(clothValue.Find("pinTop"), true);
				ClothSystem* cloth = ModernDemo ? CreateClothDemo(settings, center, horizontalAxis, verticalAxis, PinTop)
					: m_World->CreateCloth(settings, center, horizontalAxis, verticalAxis);
				if (!ModernDemo && PinTop)
				{
					const bool Corners = ReadBool(clothValue.Find("pinTopCorners"), false);
					const float PinWidthScale = ReadFloat(clothValue.Find("pinWidthScale"), 1);
					if (!std::isfinite(PinWidthScale) || PinWidthScale <= 0 || PinWidthScale > 1) throw std::invalid_argument("Cloth pinWidthScale must be in (0, 1]");
					for (int column = 0; column < cloth->GetColumns(); ++column)
					{
						if (Corners && column != 0 && column + 1 != cloth->GetColumns()) continue;
						const Vector3 Position = cloth->GetParticle(0, column).Position;
						const Vector3 Axis = horizontalAxis.SafeUnit();
						cloth->PinParticle(0, column, Position + Axis * ((Position - center).Dot(Axis) * (PinWidthScale - 1)));
					}
				}

				const std::string colliderName = ReadString(clothValue.Find("sphereBody"), std::string());
				if (!colliderName.empty())
				{
					RigidBody* colliderBody = nullptr;
					for (const SceneObjectInstance& instance : m_Objects)
					{
						if (instance.Id == colliderName && instance.GeometryPtr)
						{
							colliderBody = instance.GeometryPtr->GetParent<RigidBody>();
							break;
						}
					}
					if (colliderBody == nullptr || !cloth->AddSphereCollider(colliderBody,
						ReadFloat(clothValue.Find("sphereRadius"), 1.0f), ReadVector3(clothValue.Find("sphereCenter"), colliderBody ? colliderBody->X : Vector3::Zero())))
					{
						if (errorMessage) *errorMessage = sourceName + ": cloth references an invalid sphere body";
						return false;
					}
				}

				const std::string attachmentName = ReadString(clothValue.Find("attachBody"), std::string());
				if (!attachmentName.empty())
				{
					Geometry* Shape = nullptr;
					for (const auto& Object : m_Objects) if (Object.Id == attachmentName) Shape = Object.GeometryPtr;
					if (!AttachClothPatchToSphere(*cloth, Shape, ReadInt(clothValue.Find("attachmentCount"), 3), ReadFloat(clothValue.Find("attachmentCompliance"), 1e-5f)))
					{
						if (errorMessage) *errorMessage = sourceName + ": cloth attachment requires a valid sphere and movable cloth vertices";
						return false;
					}
				}

				const std::string mouseBodyName = ReadString(clothValue.Find("mouseBody"), std::string());
				if (!mouseBodyName.empty())
				{
					for (const SceneObjectInstance& object : m_Objects)
					{
						if (object.Id == mouseBodyName && object.GeometryPtr)
						{
							RegisterMouseControlGeometry(object.GeometryPtr);
							break;
						}
					}
					if (m_MouseControlBody == nullptr)
					{
						if (errorMessage) *errorMessage = sourceName + ": cloth has an invalid mouse control target";
						return false;
					}
				}

				SceneObjectInstance instance;
				instance.Id = ReadString(clothValue.Find("id"), "cloth");
				instance.GeometryPtr = cloth->GetRenderGeometry();
				instance.Color = ReadColor(clothValue.Find("color"), Vector4(0.32f, 0.58f, 0.92f, 1.0f));
				m_Objects.push_back(instance);
			}
			TouchRenderRevision();
		}

		const JsonValue* deformables = root.Find("deformables");
		if (deformables && !deformables->IsArray())
		{
			if (errorMessage) *errorMessage = sourceName + ": deformables must be an array";
			return false;
		}
		if (deformables)
		{
			for (const JsonValue& value : deformables->arrayValue)
			{
				if (!value.IsObject())
				{
					if (errorMessage) *errorMessage = sourceName + ": every deformable entry must be a JSON object";
					return false;
				}
				const std::string id = ReadString(value.Find("id"), "deformable");
				DeformableBodySettings settings;
				settings.Substeps = ReadInt(value.Find("substeps"), settings.Substeps);
				settings.SolverIterations = ReadInt(value.Find("iterations"), settings.SolverIterations);
				settings.EdgeCompliance = ReadFloat(value.Find("compliance"), settings.EdgeCompliance);
				settings.VolumeCompliance = ReadFloat(value.Find("volumeCompliance"), settings.VolumeCompliance);
				settings.YieldStrain = ReadFloat(value.Find("yieldStrain"), settings.YieldStrain);
				settings.PlasticCreep = ReadFloat(value.Find("plasticCreep"), settings.PlasticCreep);
				settings.MaxPlasticStrain = ReadFloat(value.Find("maxPlasticStrain"), settings.MaxPlasticStrain);
				settings.Damping = ReadFloat(value.Find("damping"), settings.Damping);
				settings.Gravity = ReadVector3(value.Find("gravity"), settings.Gravity);
				settings.ParticleRadius = ReadFloat(value.Find("radius"), settings.ParticleRadius);
				settings.Restitution = ReadFloat(value.Find("restitution"), settings.Restitution);
				settings.Friction = ReadFloat(value.Find("friction"), settings.Friction);
				const JsonValue* floor = value.Find("floor");
				settings.GroundEnabled = ReadBool(value.Find("floorEnabled"),
					ReadBool(floor, floor && floor->IsNumber() ? true : settings.GroundEnabled));
				settings.GroundHeight = ReadFloat(floor, settings.GroundHeight);
				const float inverseMass = ReadFloat(value.Find("inverseMass"), 1.0f);
				DeformableBody* body = m_World->CreateDeformableBody(settings);
				auto FailDeformable = [&](const char* message) -> bool
				{
					if (errorMessage) *errorMessage = sourceName + ": deformable '" + id + "': " + message;
					if (body) m_World->RemoveDeformableBody(body);
					return false;
				};
				if (!body) return FailDeformable("failed to create body");
				const JsonValue* particles = value.Find("particles");
				if (!particles || !particles->IsArray() || particles->arrayValue.empty())
				{
					return FailDeformable("requires a nonempty particles array");
				}
				for (const JsonValue& particle : particles->arrayValue)
				{
					if (!particle.IsArray() || particle.arrayValue.size() != 3)
					{
						return FailDeformable("particle positions must contain three numbers");
					}
					for (const JsonValue& coordinate : particle.arrayValue)
					{
						if (!coordinate.IsNumber() || !std::isfinite(coordinate.numberValue)
							|| std::abs(coordinate.numberValue) > FLT_MAX)
						{
							return FailDeformable("particle positions must be finite");
						}
					}
					if (body->AddParticle(ReadVector3(&particle, Vector3::Zero()), inverseMass) < 0)
					{
						return FailDeformable("invalid particle position or inverse mass");
					}
				}
				const int particleCount = body->GetParticleCount();
				const JsonValue* edges = value.Find("edges");
				if (edges && !edges->IsArray()) return FailDeformable("edges must be an array");
				if (edges)
				{
					for (const JsonValue& edge : edges->arrayValue)
					{
						int indices[2];
						if (!ReadParticleIndices(edge, 2, particleCount, indices)
							|| body->AddEdge(indices[0], indices[1]) < 0)
						{
							return FailDeformable("invalid edge indices or zero rest length");
						}
					}
				}
				const JsonValue* tetrahedra = value.Find("tetrahedra");
				if (tetrahedra && !tetrahedra->IsArray()) return FailDeformable("tetrahedra must be an array");
				if (tetrahedra)
				{
					for (const JsonValue& tetrahedron : tetrahedra->arrayValue)
					{
						int indices[4];
						if (!ReadParticleIndices(tetrahedron, 4, particleCount, indices)
							|| body->AddTetrahedron(indices[0], indices[1], indices[2], indices[3]) < 0)
						{
							return FailDeformable("invalid tetrahedron indices or degenerate volume");
						}
					}
				}
				const JsonValue* pins = value.Find("pins");
				if (pins && !pins->IsArray()) return FailDeformable("pins must be an array");
				if (pins)
				{
					for (const JsonValue& pin : pins->arrayValue)
					{
						if (!pin.IsNumber() || !std::isfinite(pin.numberValue) || pin.numberValue < 0.0
							|| pin.numberValue >= particleCount || std::floor(pin.numberValue) != pin.numberValue)
						{
							return FailDeformable("invalid pin index");
						}
						const int index = static_cast<int>(pin.numberValue);
						body->PinParticle(index, body->GetParticle(index).Position);
					}
				}
				const JsonValue* triangles = value.Find("triangles");
				if (triangles && !triangles->IsArray()) return FailDeformable("triangles must be an array");
				if (triangles)
				{
					std::vector<Index3> surface;
					surface.reserve(triangles->arrayValue.size());
					for (const JsonValue& triangle : triangles->arrayValue)
					{
						int indices[3];
						if (!ReadParticleIndices(triangle, 3, particleCount, indices))
						{
							return FailDeformable("invalid surface triangle indices");
						}
						surface.push_back(Index3(indices[0], indices[1], indices[2]));
					}
					if (!body->SetSurfaceTriangles(surface)) return FailDeformable("invalid surface triangles");
				}
				if (body->GetRenderGeometry())
				{
					SceneObjectInstance instance;
					instance.Id = id;
					instance.GeometryPtr = body->GetRenderGeometry();
					instance.Color = ReadColor(value.Find("color"), Vector4(0.9f, 0.42f, 0.18f, 1.0f));
					instance.RenderBounds = ReadBool(value.Find("renderBounds"), false);
					instance.RenderEnabled = ReadBool(value.Find("render"), true);
					m_Objects.push_back(instance);
				}
			}
			TouchRenderRevision();
		}

		if (ReadBool(root.Find("walkingRobot"), false) && !CreateWalkingRobot(ReadBool(root.Find("robotCourse"), false),
			ReadString(root.Find("robotModel"), "SpiderPolicy"), ReadBool(root.Find("robotLandmarks"), false), errorMessage))
			return false;
		if (!CreateCharacterDemo(root, errorMessage)) return false;
		if (!CreateReducedRagdollDemo(root, errorMessage)) return false;
		if (!CreateVehicles(root, sourceName, errorMessage)) return false;

		const JsonValue* ragdolls = root.Find("ragdolls");
		if (ragdolls && ragdolls->IsArray())
		{
			for (const JsonValue& ragdollValue : ragdolls->arrayValue)
			{
				if (!ragdollValue.IsObject())
				{
					if (errorMessage) *errorMessage = sourceName + ": every ragdoll entry must be a JSON object";
					return false;
				}
				RagdollSettings settings;
				const std::string algorithm = ReadString(ragdollValue.Find("algorithm"), "jointConstraints");
				if (algorithm == "reducedCoordinates") settings.Algorithm = RagdollAlgorithm::ReducedCoordinates;
				else if (algorithm != "jointConstraints")
				{
					if (errorMessage) *errorMessage = sourceName + ": unsupported ragdoll algorithm";
					return false;
				}
				if (m_HasRagdollAlgorithmOverride) settings.Algorithm = m_RagdollAlgorithmOverride;
				const std::string definitionFile = ReadString(
					ragdollValue.Find("definition"), std::string());
				settings.DefinitionFile = definitionFile.empty()
					? DefaultRagdollDefinition(m_BaseDirectory, settings.DefinitionFile)
					: m_BaseDirectory + definitionFile;
				settings.Scale = ReadFloat(ragdollValue.Find("scale"), settings.Scale);
				settings.LinearDamping = ReadFloat(ragdollValue.Find("linearDamping"), settings.LinearDamping);
				settings.AngularDamping = ReadFloat(ragdollValue.Find("angularDamping"), settings.AngularDamping);
				const SceneObjectDesc bodyOverrides = ReadObjectDesc(ragdollValue);
				if (!bodyOverrides.FrictionValid || !std::isfinite(settings.LinearDamping)
					|| !std::isfinite(settings.AngularDamping) || settings.LinearDamping < 0.0f
					|| settings.LinearDamping > 1.0f || settings.AngularDamping < 0.0f || settings.AngularDamping > 1.0f)
				{
					if (errorMessage) *errorMessage = sourceName + ": invalid ragdoll friction or damping";
					return false;
				}
				settings.LinearVelocity = ReadVector3(ragdollValue.Find("linearVelocity"), settings.LinearVelocity);
				settings.AngularVelocity = ReadVector3(ragdollValue.Find("angularVelocity"), settings.AngularVelocity);
				settings.JointVelocity = ReadFloat(ragdollValue.Find("jointVelocity"), settings.JointVelocity);
				settings.EnableCCD = ReadBool(ragdollValue.Find("enableCCD"), settings.EnableCCD);
				settings.DisableSelfCollision = ReadBool(ragdollValue.Find("disableSelfCollision"), settings.DisableSelfCollision);
				const Vector3 position = ReadVector3(ragdollValue.Find("position"), Vector3::Zero());
				const Quaternion rotation = ReadQuaternion(&ragdollValue, Quaternion::One());
				Ragdoll* ragdoll = m_World->CreateRagdoll(settings, Transform(position, rotation));
				if (ragdoll == nullptr)
				{
					if (errorMessage) *errorMessage = sourceName + ": failed to create ragdoll";
					return false;
				}
				const std::string id = ReadString(ragdollValue.Find("id"), "ragdoll");
				const Vector4 color = ReadColor(ragdollValue.Find("color"), Vector4(0.86f, 0.54f, 0.24f, 1.0f));
				for (size_t i = 0; i < ragdoll->GetBodies().size(); ++i)
				{
					RigidBodyDynamic* body = ragdoll->GetBodies()[i];
					if (bodyOverrides.HasFriction) body->SetFriction(bodyOverrides.FrictionDynamic, bodyOverrides.FrictionStatic);
					if (bodyOverrides.HasRestitution) body->SetRestitution(bodyOverrides.Restitution);
					for (size_t shapeIndex = 0; shapeIndex < body->Geometries().size(); ++shapeIndex)
					{
						SceneObjectInstance instance;
						std::ostringstream name;
						name << id << "_bone_" << i << "_shape_" << shapeIndex;
						instance.Id = name.str();
						instance.GeometryPtr = body->Geometries()[shapeIndex];
						instance.Color = color;
						m_Objects.push_back(instance);
					}
				}
			}
			TouchRenderRevision();
		}

		if (!CreatePhysicalAnimationDemo(root,errorMessage)) return false;
		if (!CreateStrandDemo()) { if (errorMessage) *errorMessage="Failed to create strand preview"; return false; }
		if (m_DestructionCacheDemo) for (const auto& Set : m_DestructionSets)
			m_DestructionRecorders.emplace_back(new DestructionCacheRecorder(*Set));
		if (m_PhysicsFieldsDemo) ApplyPhysicsFieldsDemo(true);
		return true;
	}

	ClothSystem* SceneWorld::CreateClothDemo(ClothSettings Settings, const Vector3& Center, const Vector3& Horizontal, const Vector3& Vertical, bool PinTop)
	{
		/* The existing cloth-and-sphere scene supplies dimensions and interaction.
		   Remove cells for a hole and an asymmetric hem, then compact only referenced
		   vertices. This exercises arbitrary topology without a new garment asset.
		   Material UVs orient warp horizontally and weft vertically. Top vertices
		   remain fixed unless the sampled animation explicitly moves those pins. */
		const Vector3 U = Horizontal.Unit(), V = (Vertical - U * Vertical.Dot(U)).Unit();
		constexpr float HoleU = -0.28f, HoleV = 0.03f, HoleRadius = 0.10f, NotchU = 0.2f, NotchV = -0.35f;
		constexpr float SurfaceDensity = 0.25f;
		ClothMesh Mesh; std::vector<int> Remap((size_t)Settings.Rows * Settings.Columns, -1); std::vector<int> Pins;
		std::vector<float> Weights;
		auto AddVertex = [&](int Row, int Column)
		{
			int& Index = Remap[(size_t)Row * Settings.Columns + Column]; if (Index >= 0) return Index;
			const float X = (float)Column / (Settings.Columns - 1) - 0.5f, Y = 0.5f - (float)Row / (Settings.Rows - 1);
			Index = (int)Mesh.Positions.size(); Mesh.Positions.push_back(Center + U * (X * Settings.Width) + V * (Y * Settings.Height));
			Mesh.MaterialCoordinates.emplace_back(X, Y); Weights.push_back(Y + 0.5f); if (Row == 0) Pins.push_back(Index); return Index;
		};
		for (int Row = 0; Row + 1 < Settings.Rows; ++Row) for (int Column = 0; Column + 1 < Settings.Columns; ++Column)
		{
			const float X = (Column + 0.5f) / (Settings.Columns - 1) - 0.5f, Y = 0.5f - (Row + 0.5f) / (Settings.Rows - 1);
			if ((X - HoleU) * (X - HoleU) + (Y - HoleV) * (Y - HoleV) < HoleRadius * HoleRadius || (X > NotchU && Y < NotchV)) continue;
			const int A = AddVertex(Row, Column), B = AddVertex(Row, Column + 1), C = AddVertex(Row + 1, Column), D = AddVertex(Row + 1, Column + 1);
			Mesh.Triangles.emplace_back(A, C, B); Mesh.Triangles.emplace_back(B, C, D);
		}
		Settings.Model = ClothModel::TriangleStrain; Settings.SurfaceDensity = SurfaceDensity;
		m_DemoCloth = m_World->CreateCloth(Settings, Mesh); m_ClothRestPose = Mesh.Positions;
		m_ClothDemoMesh.reset(new ClothMesh(Mesh));
		m_DemoClothGeometry = m_DemoCloth->GetRenderGeometry(); m_DemoCloth->SetAnimationWeights(Weights);
		if (PinTop) for (int Index : Pins) m_DemoCloth->PinParticle(Index, Mesh.Positions[Index]);
		ApplyClothDemo(true, false, true, false); return m_DemoCloth;
	}
	ClothSystem* SceneWorld::GetDemoCloth() const
	{
		if (m_World && m_DemoCloth) for (size_t I = 0; I < m_World->GetClothCount(); ++I)
			if (m_World->GetCloth(I) == m_DemoCloth) return m_DemoCloth;
		return nullptr;
	}
	bool SceneWorld::SetClothRigidInteraction(bool Enabled)
	{
		auto* Cloth = GetDemoCloth();
		if (!Cloth && m_World && m_Name == "Cloth" && m_World->GetClothCount()) Cloth = m_World->GetCloth(0);
		if (!Cloth) return false;
		if (Enabled && (m_World->GetResolutionSolver() != ResolutionPhaseSolver::PBD && m_World->GetResolutionSolver() != ResolutionPhaseSolver::XPBD)) return false;
		Geometry* Sphere = nullptr;
		for (const auto& Object : m_Objects) if (Object.Id == "cloth_ball") Sphere = Object.GeometryPtr;
		if (!Sphere || !Sphere->GetParent<RigidBody>()) return false;
		for (auto* C : { Cloth, GetDemoClothPartner() }) if (C)
		{
			auto Settings = C->GetSettings().Collision; Settings.RigidCoupling = Enabled;
			if (Enabled) Settings.Continuous = false;
			C->SetCollisionSettings(Settings);
			if (!Enabled) C->DetachRigidBody(Sphere->GetParent<RigidBody>());
		}
		return true;
	}
	bool SceneWorld::SetClothSelfCollision(bool Enabled)
	{
		auto* Cloth = GetDemoCloth();
		if (!Cloth && m_World && m_Name == "Cloth" && m_World->GetClothCount()) Cloth = m_World->GetCloth(0);
		if (!Cloth) return false;
		for (auto* C : { Cloth, GetDemoClothPartner() }) if (C)
		{
			auto Settings = C->GetSettings().Collision;
			Settings.SelfCollision = Enabled;
			C->SetCollisionSettings(Settings);
		}
		return true;
	}
	bool SceneWorld::ApplyClothParameters(ClothModel Model, int Iterations, int Substeps, float Damping,
		float StretchUCompliance, float StretchVCompliance, float ShearCompliance, float BendingCompliance)
	{
		if (!m_World || m_Name != "Cloth" || m_World->GetClothCount() == 0 || Iterations < 1 || Substeps < 1
			|| (Model != ClothModel::DistanceLinks && Model != ClothModel::TriangleStrain)
			|| !std::isfinite(Damping) || Damping < 0 || Damping > 1
			|| !std::isfinite(StretchUCompliance) || StretchUCompliance < 0
			|| !std::isfinite(StretchVCompliance) || StretchVCompliance < 0
			|| !std::isfinite(ShearCompliance) || ShearCompliance < 0
			|| !std::isfinite(BendingCompliance) || BendingCompliance < 0) return false;
		for (size_t Index = 0; Index < m_World->GetClothCount(); ++Index)
		{
			auto* Cloth = m_World->GetCloth(Index);
			Cloth->SetModel(Model);
			Cloth->SetSolverParameters(Iterations, Substeps, Damping);
			auto Material = Cloth->GetSettings().Material;
			Material.StretchUCompliance = StretchUCompliance;
			Material.StretchVCompliance = StretchVCompliance;
			Material.ShearCompliance = ShearCompliance;
			Material.BendingCompliance = BendingCompliance;
			Cloth->SetMaterial(Material);
		}
		return true;
	}
	ClothSystem* SceneWorld::GetDemoClothPartner() const
	{
		if (m_World && m_DemoClothPartner) for (size_t I = 0; I < m_World->GetClothCount(); ++I)
			if (m_World->GetCloth(I) == m_DemoClothPartner) return m_DemoClothPartner;
		return nullptr;
	}
	void SceneWorld::PruneClothDemo()
	{
		if (!GetDemoCloth() && GetDemoClothPartner()) m_World->RemoveCloth(m_DemoClothPartner);
		auto Prune = [&](ClothSystem*& Cloth, Geometry*& Geometry, std::vector<Vector3>& Rest, bool Present)
		{
			if (!Cloth || Present) return;
			m_Objects.erase(std::remove_if(m_Objects.begin(), m_Objects.end(), [&](const SceneObjectInstance& Object)
				{ return Object.GeometryPtr == Geometry; }), m_Objects.end());
			Cloth = nullptr; Geometry = nullptr; Rest.clear(); TouchRenderRevision();
		};
		Prune(m_DemoCloth, m_DemoClothGeometry, m_ClothRestPose, GetDemoCloth() != nullptr);
		Prune(m_DemoClothPartner, m_DemoClothPartnerGeometry, m_ClothPartnerRestPose, GetDemoClothPartner() != nullptr);
		if (!m_DemoCloth) m_ClothDemoMesh.reset();
	}
	bool SceneWorld::ApplyClothDemo(bool Wind, bool Animation, bool Tethers, bool SoftMaterial, bool SurfaceContacts, bool Friction,
		bool SelfCollision, bool InterCollision, bool Continuous, bool SoftBodyAttachments, bool CompliantAttachments, bool RigidAttachments, bool SoftBodyContacts, bool SoftRigidContacts,
		bool CalibratedSoftMaterial, bool StVKSoftMaterial, bool PreparedSoftAsset, bool EmbeddedSoftRender, bool CorotatedSoftMaterial, bool ProjectiveSoftMaterial)
	{
		PruneClothDemo();
		auto* Cloth = GetDemoCloth(); if (!Cloth) return false;
		if (Continuous && Cloth->HasRigidCoupling()) return false;
		m_ClothSoftMaterial = SoftMaterial;
		constexpr float StretchU = 1e-4f, StretchV = 2e-4f, Shear = 0.01f, Bend = 25, Softness = 20;
		constexpr float NormalDrag = 1, TangentialDrag = 0.02f, WindSpeed = 3, AnimationCompliance = 0.04f, AnimationDamping = 2;
		auto Material = Cloth->GetSettings().Material; const float Scale = SoftMaterial ? Softness : 1;
		Material.StretchUCompliance = StretchU * Scale; Material.StretchVCompliance = StretchV * Scale;
		Material.ShearCompliance = Shear * Scale; Material.BendingCompliance = Bend; Material.TethersEnabled = Tethers; Cloth->SetMaterial(Material);
		auto Flow = Cloth->GetSettings().Wind; Flow.Velocity = Vector3(0, 0, WindSpeed); Flow.NormalDrag = Wind ? NormalDrag : 0;
		Flow.TangentialDrag = Wind ? TangentialDrag : 0; Cloth->SetWind(Flow);
		auto Drive = Cloth->GetSettings().AnimationDrive; Drive.Enabled = Animation; Drive.Compliance = AnimationCompliance; Drive.Damping = AnimationDamping;
		Cloth->SetAnimationDrive(Drive);
		auto Collision = Cloth->GetSettings().Collision; Collision.SurfaceContacts = SurfaceContacts;
		Collision.SelfCollision = SelfCollision; Collision.InterCollision = InterCollision;
		Collision.Continuous = Continuous;
		Collision.Material = SurfaceContactMaterial();
		if (!Friction) Collision.Material.StaticFriction = Collision.Material.DynamicFriction = 0;
		Cloth->SetCollisionSettings(Collision);
		if (InterCollision && !GetDemoClothPartner())
		{
			// A second layer reuses the procedural garment and existing sphere. Keep
			// it when collision is disabled so the switch compares the same geometry.
			// Opposing winds bring the layers together without any imported assets.
			constexpr float LayerClearance = 0.1f;
			auto Mesh = *m_ClothDemoMesh; const Vector3 Offset(0, 0, 2 * Cloth->GetSettings().Thickness + LayerClearance);
			for (auto& Position : Mesh.Positions) Position += Offset;
			m_DemoClothPartner = m_World->CreateCloth(Cloth->GetSettings(), Mesh); m_ClothPartnerRestPose = Mesh.Positions;
			m_DemoClothPartnerGeometry = m_DemoClothPartner->GetRenderGeometry();
			std::vector<float> Weights;
			for (int I = 0; I < Cloth->GetParticleCount(); ++I)
			{
				Weights.push_back(Mesh.MaterialCoordinates[I].y + 0.5f);
				if (Cloth->GetParticle(I).Pinned) m_DemoClothPartner->PinParticle(I, Mesh.Positions[I]);
			}
			m_DemoClothPartner->SetAnimationWeights(Weights);
			for (const auto& Object : m_Objects) if (Object.GeometryPtr)
			{
				const auto* Sphere = Object.GeometryPtr->GetShapeObj<Sphere3>(); auto* Body = Object.GeometryPtr->GetParent<RigidBody>();
				if (Sphere && Body) m_DemoClothPartner->AddSphereCollider(Body, Sphere->Radius, Body->GetGeometryTransform(Object.GeometryPtr).LocalToWorld(Sphere->Center));
			}
			SceneObjectInstance Object; Object.Id = "cloth_partner"; Object.GeometryPtr = m_DemoClothPartnerGeometry;
			Object.Color = Vector4(0.3f, 0.85f, 0.5f, 1); m_Objects.push_back(Object); TouchRenderRevision();
		}
		if (auto* Partner = GetDemoClothPartner())
		{
			Flow.Velocity = -Flow.Velocity;
			Partner->SetMaterial(Material); Partner->SetWind(Flow); Partner->SetAnimationDrive(Drive); Partner->SetCollisionSettings(Collision);
		}
		return ApplyDeformableAttachmentDemo(SoftBodyAttachments, CompliantAttachments, RigidAttachments) && ApplyDeformableContactDemo(SoftBodyContacts, SoftRigidContacts, PreparedSoftAsset, EmbeddedSoftRender)
			&& (!SoftBodyContacts || ApplyDeformableMaterialDemo(CalibratedSoftMaterial, StVKSoftMaterial, CorotatedSoftMaterial, ProjectiveSoftMaterial));
	}
	bool SceneWorld::GetCalibratedSoftMaterialDemo() const
	{
		auto* Body = GetDemoContactBody(0); return Body && !Body->GetTetrahedra().empty() && Body->GetTetrahedra()[0].Material >= 0;
	}
	bool SceneWorld::GetStVKSoftMaterialDemo() const
	{
		if (!GetCalibratedSoftMaterialDemo()) return false; auto* Body = GetDemoContactBody(0);
		return Body->GetMaterials()[Body->GetTetrahedra()[0].Material].Model == TetrahedralMaterialModel::SaintVenantKirchhoff;
	}
	bool SceneWorld::GetCorotatedSoftMaterialDemo() const
	{
		if (!GetCalibratedSoftMaterialDemo()) return false; auto* Body = GetDemoContactBody(0);
		return Body->GetMaterials()[Body->GetTetrahedra()[0].Material].Model == TetrahedralMaterialModel::Corotated;
	}
	bool SceneWorld::GetProjectiveSoftMaterialDemo() const
	{
		auto* Body = GetDemoContactBody(0); return Body && Body->GetSettings().ProjectiveMaterials;
	}
	bool SceneWorld::ApplyDeformableMaterialDemo(bool Enabled, bool StVK, bool Corotated, bool Projective)
	{
		if (!m_World || m_Name != "Cloth" || !GetDemoContactBody(0) || !GetDemoContactBody(1)) return false;
		if (Enabled && ((Projective && !Corotated) || (StVK && Corotated))) return false;
		// These existing procedural bodies have one element each. Preserve their
		// original four-kilogram mass while comparing calibrated elastic laws.
		constexpr double LowerYoungModulus = 20000, UpperYoungModulus = 2000, PoissonRatio = 0.3;
		for (size_t I = 0; I < 2; ++I)
		{
			auto* Body = GetDemoContactBody(I);
			Body->SetProjectiveMaterialSolver(Enabled && Projective);
			if (!Enabled) { Body->SetTetrahedronMaterial(0, -1); continue; }
			TetrahedralMaterial Material; Material.Model = Corotated ? TetrahedralMaterialModel::Corotated : StVK ? TetrahedralMaterialModel::SaintVenantKirchhoff : TetrahedralMaterialModel::StableNeoHookean;
			Material.YoungModulus = I ? UpperYoungModulus : LowerYoungModulus; Material.PoissonRatio = PoissonRatio;
			const int Id = Body->GetMaterials().empty() ? Body->AddMaterial(Material) : 0;
			Body->SetMaterial(Id, Material); Body->SetTetrahedronMaterial(0, Id);
			Material.Density = Body->GetParticleCount() / Body->GetTetrahedronRestShape(0).Volume;
			Body->SetMaterial(Id, Material); Body->ApplyMaterialDensities();
		}
		return true;
	}
	DeformableBody* SceneWorld::GetDemoContactBody(size_t Index) const
	{
		if (!m_World || Index >= m_DemoContactBodies.size()) return nullptr;
		auto* Body = m_DemoContactBodies[Index].first; const auto& Bodies = m_World->GetDeformableBodies();
		return std::find(Bodies.begin(), Bodies.end(), Body) != Bodies.end() && Body->GetRenderGeometry() == m_DemoContactBodies[Index].second ? Body : nullptr;
	}
	RigidBodyDynamic* SceneWorld::GetDemoContactRigid() const
	{
		if (!m_World || !m_DemoContactGeometry) return nullptr;
		const auto* Proxy = m_World->GetGeometryQuery()->GetProxy(m_DemoContactHandle);
		if (!Proxy || !Proxy->IsActive() || Proxy->Geom != m_DemoContactGeometry) return nullptr;
		auto* Body = Proxy->Geom->GetParent<RigidBody>(); return Body ? Body->CastDynamic() : nullptr;
	}
	bool SceneWorld::GetPreparedSoftAssetDemo() const
	{
		auto* Body = GetDemoContactBody(0); return Body && Body->GetPreparedAsset();
	}
	bool SceneWorld::GetEmbeddedSoftRenderDemo() const
	{
		auto* Body = GetDemoContactBody(0); return Body && Body->IsEmbeddedRenderMeshEnabled();
	}
	bool SceneWorld::ApplyDeformableContactDemo(bool Enabled, bool Rigid, bool Prepared, bool Embedded)
	{
		if (!m_World || m_Name != "Cloth") return false;
		if (Enabled && Rigid && ((m_World->GetResolutionSolver() != ResolutionPhaseSolver::PBD
			&& m_World->GetResolutionSolver() != ResolutionPhaseSolver::XPBD) || m_World->GetSupportStabilizationSettings().Enabled)) return false;
		if (Enabled && GetDemoContactBody(0) && GetDemoContactBody(1) && m_SoftRigidContactDemo == Rigid && GetPreparedSoftAssetDemo() == Prepared && (!Rigid || GetDemoContactRigid()))
		{
			if (Prepared && GetEmbeddedSoftRenderDemo() != Embedded)
			{
				for (size_t I = 0; I < 2; ++I) GetDemoContactBody(I)->UseEmbeddedRenderMesh(Embedded);
				TouchRenderRevision();
			}
			return true;
		}
		if (!Enabled && m_DemoContactBodies.empty() && !m_DemoContactGeometry && !m_DemoContactGravity) return true;
		if (m_DemoContactGravity) m_World->RemoveForceField(m_DemoContactGravity);
		m_DemoContactGravity = nullptr;
		m_Objects.erase(std::remove_if(m_Objects.begin(), m_Objects.end(), [&](const SceneObjectInstance& Object)
			{ return m_DemoContactGeometry && Object.GeometryPtr == m_DemoContactGeometry; }), m_Objects.end());
		if (auto* Body = GetDemoContactRigid()) { m_World->RemoveRigidBody(Body); Body->ReleaseGeometries(); delete Body; }
		m_DemoContactGeometry = nullptr; m_DemoContactHandle = GeometryHandle(); m_SoftRigidContactDemo = Enabled && Rigid;
		for (const auto& Entry : m_DemoContactBodies)
		{
			m_Objects.erase(std::remove_if(m_Objects.begin(), m_Objects.end(), [&](const SceneObjectInstance& Object)
				{ return Object.GeometryPtr == Entry.second; }), m_Objects.end());
			const auto& Bodies = m_World->GetDeformableBodies();
			if (std::find(Bodies.begin(), Bodies.end(), Entry.first) != Bodies.end() && Entry.first->GetRenderGeometry() == Entry.second)
				m_World->RemoveDeformableBody(Entry.first);
		}
		m_DemoContactBodies.clear(); TouchRenderRevision();
		if (!Enabled) return true;
		// Separate procedural tetrahedra exercise surface contact without fighting
		// the original attachment demo's zero-gap face anchor. The lower volume is
		// held at its center; the upper volume falls onto its deformable top face.
		constexpr float SidePosition = -5.8f, ContactHeight = 2, HalfWidth = 0.5f, Height = 1, InitialGap = 0.15f;
		constexpr float BallRadius = 0.35f, BallMass = 1.5f;
		DeformableBodySettings Settings; Settings.Substeps = 8; Settings.SolverIterations = 32;
		Settings.EdgeCompliance = 1e-5f; Settings.VolumeCompliance = 1e-7f; Settings.Damping = 0.8f;
		Settings.GroundEnabled = false; Settings.ParticleRadius = 0.035f;
		Settings.Collision.InterCollision = Settings.Collision.SelfCollision = true;
		Settings.Collision.RigidCollision = Rigid;
		try
		{
			m_DemoContactBodies.reserve(2);
			for (int I = 0; I < 2; ++I)
			{
				DeformableBody* Body = nullptr;
				const Vector3 Origin(SidePosition, ContactHeight + (I ? InitialGap + (Rigid ? BallRadius * 2 : 0) : 0), 0);
				const std::vector<Vector3> Positions = { Vector3(-HalfWidth, 0, -HalfWidth), Vector3(HalfWidth, 0, -HalfWidth),
					Vector3(0, 0, 2 * HalfWidth), Vector3(0, I ? Height : -Height, 0) };
				if (!Prepared)
				{
					Body = m_World->CreateDeformableBody(Settings); m_DemoContactBodies.push_back({ Body, nullptr });
					for (const auto& P : Positions) Body->AddParticle(Origin + P);
					Body->AddTetrahedron(0, 1, 2, 3);
					Body->SetSurfaceTriangles({ Index3(0, 2, 1), Index3(0, 1, 3), Index3(1, 2, 3), Index3(2, 0, 3) });
				}
				else
				{
					// Refine only the visible surface. The same four simulation nodes and
					// four collision faces exercise embedding without new resource files.
					constexpr int RenderRefinements = 2; constexpr double BodyMass = 4;
					TetrahedralAssetInput Input; Input.Positions = Positions; Input.Elements.push_back({ {0,1,2,3}, 0 });
					const auto Base = TetrahedralAsset::Prepare(Input);
					Input.Materials[0].Density = BodyMass / Base.GetStats().TotalVolume;
					Input.RenderPositions = Positions; Input.RenderTriangles = Base.GetBoundary();
					for (int Level = 0; Level < RenderRefinements; ++Level)
					{
						std::map<std::pair<int,int>,int> Midpoints; std::vector<Index3> Refined;
						auto Midpoint = [&](int A, int B)
						{
							const auto Key = std::minmax(A, B); auto Found = Midpoints.find(Key); if (Found != Midpoints.end()) return Found->second;
							const int Id = (int)Input.RenderPositions.size(); Input.RenderPositions.push_back((Input.RenderPositions[A] + Input.RenderPositions[B]) / 2);
							Midpoints.emplace(Key, Id); return Id;
						};
						for (const auto& F : Input.RenderTriangles)
						{
							const int AB = Midpoint(F.a,F.b), BC = Midpoint(F.b,F.c), CA = Midpoint(F.c,F.a);
							Refined.insert(Refined.end(), { Index3(F.a,AB,CA), Index3(AB,F.b,BC), Index3(CA,BC,F.c), Index3(AB,BC,CA) });
						}
						Input.RenderTriangles.swap(Refined);
					}
					Body = m_World->CreateDeformableBody(TetrahedralAsset::Prepare(Input), Settings, Transform(Origin));
					m_DemoContactBodies.push_back({ Body, Body->GetRenderGeometry() }); Body->UseEmbeddedRenderMesh(Embedded);
				}
				m_DemoContactBodies.back().second = Body->GetRenderGeometry();
				SceneObjectInstance Object; Object.Id = I ? "soft_contact_upper" : "soft_contact_lower";
				Object.GeometryPtr = Body->GetRenderGeometry(); Object.Color = I ? Vector4(0.2f, 0.8f, 0.95f, 1) : Vector4(0.4f, 0.85f, 0.3f, 1);
				m_Objects.push_back(Object);
			}
			auto* Lower = GetDemoContactBody(0);
			const auto Center = BarycentricPoint::Tetrahedron({ 0, 1, 2, 3 }, { 0.25, 0.25, 0.25, 0.25 });
			Lower->AttachToWorld(Center, Lower->GetPointPosition(Center));
			if (Rigid)
			{
				// A free rigid ball between the two soft volumes makes their contact
				// reactions visible without introducing any imported geometry.
				Geometry* Shape = GeometryFactory::CreateSphere(Vector3(SidePosition, ContactHeight + BallRadius + InitialGap / 2, 0), BallRadius);
				auto* Mass = Shape->GetMassParameters(); Mass->InertiaMat *= BallMass / Mass->Mass; Mass->Mass = BallMass;
				RigidBodyParam Param; Param.rigidType = RigidType::Dynamic; Param.disableGravity = true;
				auto* Body = m_World->CreateRigidBody(Shape, Param)->CastDynamic();
				m_DemoContactGeometry = Shape; m_DemoContactHandle = Body->GetGeometryHandle(0);
				m_DemoContactGravity = m_World->AddForceField(new AttachmentPayloadGravity(m_DemoContactHandle, Settings.Gravity));
				SceneObjectInstance Object; Object.Id = "soft_contact_ball"; Object.GeometryPtr = Shape; Object.Color = Vector4(1, 0.65f, 0.15f, 1); m_Objects.push_back(Object);
			}
		}
		catch (...) { ApplyDeformableContactDemo(false); throw; }
		TouchRenderRevision(); return true;
	}
	DeformableBody* SceneWorld::GetDemoDeformableBody(size_t Index) const
	{
		if (!m_World || Index >= m_DemoSoftBodies.size()) return nullptr;
		auto* Body = m_DemoSoftBodies[Index].first; const auto& Bodies = m_World->GetDeformableBodies();
		return std::find(Bodies.begin(), Bodies.end(), Body) != Bodies.end() && Body->GetRenderGeometry() == m_DemoSoftBodies[Index].second ? Body : nullptr;
	}
	RigidBodyDynamic* SceneWorld::GetDemoAttachmentRigid() const
	{
		if (!m_World || !m_DemoAttachmentGeometry) return nullptr;
		const auto* Proxy = m_World->GetGeometryQuery()->GetProxy(m_DemoAttachmentHandle);
		if (!Proxy || !Proxy->IsActive() || Proxy->Geom != m_DemoAttachmentGeometry) return nullptr;
		auto* Body = Proxy->Geom->GetParent<RigidBody>(); return Body ? Body->CastDynamic() : nullptr;
	}
	bool SceneWorld::ApplyDeformableAttachmentDemo(bool Enabled, bool Compliant, bool Rigid)
	{
		if (!m_World || m_Name != "Cloth") return false;
		if (Enabled && Rigid && ((m_World->GetResolutionSolver() != ResolutionPhaseSolver::PBD
			&& m_World->GetResolutionSolver() != ResolutionPhaseSolver::XPBD) || m_World->GetSupportStabilizationSettings().Enabled)) return false;
		if (Enabled && GetDemoDeformableBody(0) && GetDemoDeformableBody(1) && m_CompliantAttachmentDemo == Compliant
			&& m_RigidAttachmentDemo == Rigid && (!Rigid || GetDemoAttachmentRigid())) return true;
		if (!Enabled && m_DemoSoftBodies.empty() && !m_DemoAttachmentGeometry && !m_DemoAttachmentGravity) return true;
		if (m_DemoAttachmentGravity) m_World->RemoveForceField(m_DemoAttachmentGravity);
		m_DemoAttachmentGravity = nullptr;
		m_Objects.erase(std::remove_if(m_Objects.begin(), m_Objects.end(), [&](const SceneObjectInstance& Object)
			{ return m_DemoAttachmentGeometry && Object.GeometryPtr == m_DemoAttachmentGeometry; }), m_Objects.end());
		if (auto* Body = GetDemoAttachmentRigid())
		{
			m_World->RemoveRigidBody(Body); Body->ReleaseGeometries(); delete Body;
		}
		m_DemoAttachmentHandle = GeometryHandle(); m_DemoAttachmentGeometry = nullptr; m_RigidAttachmentDemo = Enabled && Rigid;
		for (const auto& Entry : m_DemoSoftBodies)
		{
			m_Objects.erase(std::remove_if(m_Objects.begin(), m_Objects.end(), [&](const SceneObjectInstance& Object)
				{ return Object.GeometryPtr == Entry.second; }), m_Objects.end());
			const auto& Bodies = m_World->GetDeformableBodies();
			if (std::find(Bodies.begin(), Bodies.end(), Entry.first) != Bodies.end() && Entry.first->GetRenderGeometry() == Entry.second)
				m_World->RemoveDeformableBody(Entry.first);
		}
		m_DemoSoftBodies.clear(); m_CompliantAttachmentDemo = Compliant; TouchRenderRevision();
		if (!Enabled) return true;
		// Reuse scene 08 with two procedural tetrahedra beside the garment. Their
		// interior/face anchors exercise barycentric response without imported assets.
		// The existing cloth remains selectable, as do hard and compliant attachments.
		constexpr float SidePosition = 5.8f, TopHeight = 5, HalfWidth = 0.5f, Height = 1;
		constexpr double AttachmentCompliance = 0.002;
		DeformableBodySettings Settings; Settings.Substeps = 4; Settings.SolverIterations = 24;
		Settings.EdgeCompliance = 1e-5f; Settings.VolumeCompliance = 1e-6f; Settings.Damping = 0.8f;
		try
		{
			m_DemoSoftBodies.reserve(2);
			for (int I = 0; I < 2; ++I)
			{
				auto* Body = m_World->CreateDeformableBody(Settings); m_DemoSoftBodies.push_back({ Body, nullptr });
				const Vector3 Origin(SidePosition, TopHeight - I * Height, 0);
				Body->AddParticle(Origin + Vector3(-HalfWidth, 0, -HalfWidth));
				Body->AddParticle(Origin + Vector3(HalfWidth, 0, -HalfWidth));
				Body->AddParticle(Origin + Vector3(0, 0, 2 * HalfWidth));
				Body->AddParticle(Origin + Vector3(0, -Height, 0));
				Body->AddTetrahedron(0, 1, 2, 3);
				Body->SetSurfaceTriangles({ Index3(0, 2, 1), Index3(0, 1, 3), Index3(1, 2, 3), Index3(2, 0, 3) });
				if (I) for (int J = 0; J < Body->GetParticleCount(); ++J) Body->SetParticleVelocity(J, Vector3(0, 0, 1));
				m_DemoSoftBodies.back().second = Body->GetRenderGeometry();
				SceneObjectInstance Object; Object.Id = I ? "soft_attachment_lower" : "soft_attachment_upper";
				Object.GeometryPtr = Body->GetRenderGeometry(); Object.Color = I ? Vector4(0.95f, 0.35f, 0.6f, 1) : Vector4(0.95f, 0.75f, 0.2f, 1);
				m_Objects.push_back(Object);
			}
			auto* Upper = GetDemoDeformableBody(0); auto* Lower = GetDemoDeformableBody(1);
			const auto Center = BarycentricPoint::Tetrahedron({ 0, 1, 2, 3 }, { 0.25, 0.25, 0.25, 0.25 });
			const auto Face = BarycentricPoint::Triangle(Index3(0, 1, 2), Vector3(1.0f / 3));
			const double Compliance = Compliant ? AttachmentCompliance : 0;
			Upper->AttachToWorld(Center, Upper->GetPointPosition(Center), Compliance);
			Lower->AttachToBody(Face, Upper, BarycentricPoint::Vertex(3), Compliance);
			if (Rigid)
			{
				// An off-center procedural payload makes the angular and two-way
				// response visible while preserving the original soft-only option.
				const Vector3 Anchor = Lower->GetParticle(3).Position;
				const Vector3 Offset(0.35f, -0.4f, 0), Extent(0.4f, 0.3f, 0.35f);
				constexpr float PayloadMass = 2;
				RigidBodyParam Param; Param.rigidType = RigidType::Dynamic; Param.disableGravity = true;
				Geometry* Shape = GeometryFactory::CreateOBB(Anchor + Offset, Extent, Quaternion::One());
				auto* Mass = Shape->GetMassParameters(); Mass->InertiaMat *= PayloadMass / Mass->Mass; Mass->Mass = PayloadMass;
				auto* Payload = m_World->CreateRigidBody(Shape, Param)->CastDynamic();
				m_DemoAttachmentGeometry = Shape; m_DemoAttachmentHandle = Payload->GetGeometryHandle(0);
				// Scene 08 has zero rigid-world gravity. Give this payload the same
				// acceleration as the soft bodies, evaluated once per physical substep.
				m_DemoAttachmentGravity = m_World->AddForceField(new AttachmentPayloadGravity(m_DemoAttachmentHandle, Settings.Gravity));
				m_World->AttachDeformableToRigid(Lower, BarycentricPoint::Vertex(3), Payload, Anchor, Compliance);
				SceneObjectInstance Object; Object.Id = "soft_attachment_payload"; Object.GeometryPtr = Shape;
				Object.Color = Vector4(0.2f, 0.65f, 0.95f, 1); m_Objects.push_back(Object);
			}
		}
		catch (...) { ApplyDeformableAttachmentDemo(false, Compliant); throw; }
		TouchRenderRevision(); return true;
	}
	void SceneWorld::SampleClothDemo(float Dt)
	{
		if (!m_DemoContactBodies.empty() && (!GetDemoContactBody(0) || !GetDemoContactBody(1) || (m_SoftRigidContactDemo && !GetDemoContactRigid()))) ApplyDeformableContactDemo(false);
		if (!m_DemoSoftBodies.empty() && (!GetDemoDeformableBody(0) || !GetDemoDeformableBody(1) || (m_RigidAttachmentDemo && !GetDemoAttachmentRigid()))) ApplyDeformableAttachmentDemo(false, false);
		PruneClothDemo(); auto* Cloth = GetDemoCloth(); if (!Cloth) return;
		if (Cloth->GetSettings().AnimationDrive.Enabled) m_ClothAnimationTime += Dt;
		constexpr double Frequency = 1.1; constexpr float SwayAmplitude = 0.6f;
		const Vector3 Offset(0, 0, SwayAmplitude * (float)std::sin(Frequency * m_ClothAnimationTime));
		auto Pose = m_ClothRestPose; for (auto& Position : Pose) Position += Offset;
		Cloth->SetAnimationPose(Pose, Dt);
		if (auto* Partner = GetDemoClothPartner())
		{
			Pose = m_ClothPartnerRestPose; for (auto& Position : Pose) Position += Offset; Partner->SetAnimationPose(Pose, Dt);
		}
	}

	bool SceneWorld::CreatePhysicalAnimationDemo(const JsonValue& Root, std::string* Error)
	{
		if (!m_PhysicalAnimationDemo || ReadString(Root.Find("name"), "") != "Ragdoll") return true;
		/* The existing stair scene keeps its passive falling character. On the
		   landing, gold is a driven physical ragdoll and cyan is its presentation
		   pose, translated sideways for comparison. Cyan geometry belongs only to
		   the Viewer: it never enters collision queries or the constraint graph.
		   This adapter samples a procedural shoulder/elbow animation around the
		   authored joint anchors, then submits WORLD COM transforms. Rotating a
		   bone around its COM instead would separate the animation's joint anchors.
		   A world-space pelvis servo supplies external support; this demonstrates
		   physical animation, not autonomous balance or an authored get-up motion. */
		const Vector3 Origin(-12.8f, 0.5f, 0);
		constexpr float MuscleStiffness = 200, MuscleDamping = 20, MuscleTorque = 100;
		constexpr float RootStiffness = 400, RootDamping = 40, RootForce = 500, InitialPhysicsBlend = 0.5f;
		constexpr int DemoSubsteps = 4;
		RagdollSettings Settings; Settings.DisableSelfCollision = true;
		Settings.DefinitionFile = DefaultRagdollDefinition(m_BaseDirectory, Settings.DefinitionFile);
		auto* Character = m_World->CreateRagdoll(Settings, Transform(Origin));
		if (!Character) { if (Error) *Error = "Failed to create physical animation ragdoll."; return false; }
		m_DemoAnimation = m_World->CreatePhysicalAnimation(Character);
		auto* Animation = m_DemoAnimation;
		auto Muscles = Animation->GetControl(0); Muscles.Angular.Acceleration = false;
		Muscles.Angular.Stiffness = MuscleStiffness; Muscles.Angular.Damping = MuscleDamping; Muscles.Angular.MaxForce = MuscleTorque;
		Muscles.PhysicsBlend = InitialPhysicsBlend; Animation->SetControlBelow(0, Muscles);
		auto Pelvis = Muscles; Pelvis.Space = PhysicalAnimationSpace::World;
		Pelvis.Linear.PositionEnabled = Pelvis.Linear.VelocityEnabled = true; Pelvis.Linear.Acceleration = false;
		Pelvis.Linear.Stiffness = RootStiffness; Pelvis.Linear.Damping = RootDamping; Pelvis.Linear.MaxForce = RootForce;
		Pelvis.Angular.Damping = RootDamping; Pelvis.Angular.MaxForce = RootForce; Animation->SetControl(0, Pelvis);
		auto Temporal = m_World->GetTemporalSubstepSettings(); Temporal.Enabled = true; Temporal.Substeps = DemoSubsteps;
		m_World->SetTemporalSubstepSettings(Temporal);
		const auto Rest = Animation->GetPhysicsPose(); const auto& Bones = Animation->GetBones();
		m_AnimationRestLocal.resize(Bones.size()); m_AnimationAnchors.resize(Bones.size()); m_AnimationAxes.resize(Bones.size());
		for (size_t I = 0; I < Bones.size(); ++I)
		{
			const int Parent = Bones[I].Parent;
			m_AnimationRestLocal[I] = Parent < 0 ? Rest[I] : Rest[Parent].TransformInv(Rest[I]);
			if (Parent >= 0)
			{
				m_AnimationAxes[I] = Rest[Parent].WorldToLocalDirection(Vector3::UnitZ());
				for (auto* Joint : Character->GetConstraints()) if (Joint->GetBodyB() == Bones[I].Body)
					m_AnimationAnchors[I] = Rest[Parent].WorldToLocal(static_cast<PointConstraint*>(Joint)->GetWorldAnchorA());
			}
			for (size_t J = 0; J < Bones[I].Body->Geometries().size(); ++J)
			{
				auto* Shape = Bones[I].Body->Geometries()[J]; Geometry* Copy = nullptr;
				// Only the three primitive types used by the existing ragdoll asset are
				// copied. Preserve local shape data as well as the COM-to-shape offset.
				if (const auto* Box = Shape->GetShapeObj<AxisAlignedBox3>())
				{
					Copy = GeometryFactory::CreateOBB(Vector3::Zero(), Vector3(1)); *Copy->GetShapeObj<AxisAlignedBox3>() = *Box;
				}
				else if (const auto* Sphere = Shape->GetShapeObj<Sphere3>())
				{
					Copy = GeometryFactory::CreateSphere(Vector3::Zero(), Sphere->Radius); *Copy->GetShapeObj<Sphere3>() = *Sphere;
				}
				else if (const auto* Capsule = Shape->GetShapeObj<Capsule3>())
				{
					Copy = GeometryFactory::CreateCapsule(Capsule->X0, Capsule->X1, Capsule->Radius); *Copy->GetShapeObj<Capsule3>() = *Capsule;
				}
				if (!Copy) { if (Error) *Error = "Unsupported physical animation demo shape."; return false; }
				PhysicalAnimationRenderState Surface; Surface.Bone = I; Surface.Surface.reset(Copy);
				Surface.LocalTransform = Rest[I].TransformInv(Bones[I].Body->GetGeometryTransform(Shape));
				Copy->UpdateVolumeProperties(); Copy->SetSimulationEnabled(false); Copy->SetQueryEnabled(false);
				m_AnimationSurfaces.push_back(std::move(Surface));
				SceneObjectInstance Object; const std::string Suffix = std::to_string(I) + "_" + std::to_string(J);
				Object.Id = "physical_animation_body_" + Suffix; Object.GeometryPtr = Shape; Object.Color = Vector4(0.98f, 0.75f, 0.16f, 1);
				m_Objects.push_back(Object);
				Object.Id = "physical_animation_blend_" + Suffix; Object.GeometryPtr = Copy; Object.Color = Vector4(0.15f, 0.78f, 0.91f, 1);
				m_Objects.push_back(Object);
			}
		}
		SyncPhysicalAnimationDemo(); TouchRenderRevision(); return true;
	}
	PhysicalAnimation* SceneWorld::GetDemoPhysicalAnimation() const
	{
		if (m_World && m_DemoAnimation) for (size_t I = 0; I < m_World->GetPhysicalAnimationCount(); ++I)
			if (m_World->GetPhysicalAnimation(I) == m_DemoAnimation) return m_DemoAnimation;
		return nullptr;
	}
	bool SceneWorld::ApplyPhysicalAnimationDemo(bool Passive, float LeftArmStrength, float PhysicsBlend)
	{
		auto* Animation = GetDemoPhysicalAnimation();
		if (!Animation || !std::isfinite(LeftArmStrength) || !std::isfinite(PhysicsBlend)
			|| LeftArmStrength < 0 || LeftArmStrength > 1 || PhysicsBlend < 0 || PhysicsBlend > 1) return false;
		for (size_t I = 0; I < Animation->GetBones().size(); ++I)
		{
			auto Control = Animation->GetControl(I); Control.PhysicsBlend = PhysicsBlend; Animation->SetControl(I, Control);
		}
		const size_t Arm = (size_t)RagdollBone::LeftUpperArm;
		auto Control = Animation->GetControl(Arm); Control.Strength = LeftArmStrength; Animation->SetControlBelow(Arm, Control);
		constexpr float RecoveryDuration = 1.5f, KnockdownAngularSpeed = 1.5f;
		if (Passive && Animation->GetState() != PhysicalAnimationState::Passive)
		{
			Animation->SetActive(false);
			Animation->GetBones()[(size_t)RagdollBone::Torso].Body->W += Vector3::UnitZ() * KnockdownAngularSpeed;
		}
		else if (!Passive && Animation->GetState() == PhysicalAnimationState::Passive) Animation->BeginRecovery(RecoveryDuration);
		SyncPhysicalAnimationDemo(); return true;
	}
	void SceneWorld::SamplePhysicalAnimationDemo(float Dt)
	{
		auto* Animation = GetDemoPhysicalAnimation(); if (!Animation) return;
		m_PhysicalAnimationTime += Dt;
		constexpr double Frequency = 1.2;
		constexpr float ShoulderAmplitude = 20 * PI / 180, ElbowAmplitude = 35 * PI / 180;
		const float Wave = (float)((1 - std::cos(m_PhysicalAnimationTime * Frequency)) / 2);
		std::vector<Transform> Pose(m_AnimationRestLocal.size());
		for (size_t I = 0; I < Pose.size(); ++I)
		{
			float Angle = 0; const auto Bone = (RagdollBone)I;
			if (Bone == RagdollBone::LeftUpperArm) Angle = -ShoulderAmplitude * Wave;
			if (Bone == RagdollBone::RightUpperArm) Angle = ShoulderAmplitude * Wave;
			if (Bone == RagdollBone::LeftLowerArm) Angle = -ElbowAmplitude * Wave;
			if (Bone == RagdollBone::RightLowerArm) Angle = ElbowAmplitude * Wave;
			Transform Local = m_AnimationRestLocal[I];
			if (Angle != 0)
			{
				Quaternion Rotation; Rotation.FromRotationAxis(m_AnimationAxes[I], Angle);
				Local.pos = m_AnimationAnchors[I] + Rotation * (Local.pos - m_AnimationAnchors[I]);
				Local.quat = (Rotation * Local.quat).Unit();
			}
			const int Parent = Animation->GetBones()[I].Parent; Pose[I] = Parent < 0 ? Local : Pose[Parent] * Local;
		}
		Animation->SetAnimationPose(Pose, Dt);
	}
	void SceneWorld::SyncPhysicalAnimationDemo()
	{
		auto* Animation = GetDemoPhysicalAnimation();
		if (!Animation)
		{
			if (!m_DemoAnimation) return;
			// Removing the controller or a bound body invalidates this demo adapter.
			// Remove its render bindings before any borrowed geometry can be read.
			m_Objects.erase(std::remove_if(m_Objects.begin(), m_Objects.end(), [](const SceneObjectInstance& Object)
				{ return Object.Id.find("physical_animation_") == 0; }), m_Objects.end());
			m_AnimationSurfaces.clear(); m_DemoAnimation = nullptr; TouchRenderRevision(); return;
		}
		const auto Pose = Animation->GetBlendedPose(); const Vector3 ComparisonOffset(3.6f, 0, 0);
		for (auto& Surface : m_AnimationSurfaces)
		{
			const Transform World = Pose[Surface.Bone] * Surface.LocalTransform;
			Surface.Surface->SetTransform(World.pos + ComparisonOffset, World.quat);
		}
	}

	bool SceneWorld::CreateReducedRagdollDemo(const JsonValue& Root, std::string* Error)
	{
		if (!m_ReducedRagdollDemo || ReadString(Root.Find("name"), "") != "Constraints") return true;
		if (m_World->GetResolutionSolver() != ResolutionPhaseSolver::SequentialImpulse || m_World->GetSolverBackend() != SolverBackend::CPU
			|| m_World->GetSupportStabilizationSettings().Enabled)
		{
			if (Error) *Error = "The reduced-coordinate ragdoll demo requires CPU sequential impulses with support stabilization disabled.";
			return false;
		}
		/* These procedural mechanisms extend the existing constraint scene. The
		   anchored chain demonstrates exact joint kinematics, the arm combines two
		   hinges with a telescoping slider, and the floating chain tests contact
		   response throughout a tree. Shapes are centered on the model's COM; joint
		   frames live at the ends of the bars. No render or collision asset is needed.
		   The Joint drives switch powers the arm, transmission and cable winches
		   without changing their topology or limits. Substeps improve long chains;
		   they do not replace contact/joint iterations or increase drive force caps. */
		constexpr int ChainLinks = 16, ArmLinks = 3, FloatingLinks = 5, DemoSubsteps = 4;
		constexpr float ChainLength = 0.45f, ArmLength = 1.5f, FloatingLength = 1.0f, JointDamping = 0.05f;
		const Vector3 ChainExtent(0.12f, ChainLength / 2, 0.12f), ArmExtent(0.2f, ArmLength / 2, 0.2f);
		const Vector3 FloatingExtent(FloatingLength / 2, 0.18f, 0.18f), BaseExtent(0.25f);
		auto AddTree = [&](const std::string& Name, RagdollSettings& Settings, const std::vector<Vector3>& Extents, const Vector4& Color)
		{
			std::vector<RigidBodyDynamic*> Bodies;
			for (size_t I = 0; I < Settings.Links.size(); ++I)
			{
				auto& Link = Settings.Links[I]; const Vector3& H = Extents[I];
				Link.Inertia = Matrix3(H.y * H.y + H.z * H.z, H.x * H.x + H.z * H.z, H.x * H.x + H.y * H.y) * (Link.Mass / 3);
				Link.Damping = JointDamping;
				auto* Shape = GeometryFactory::CreateOBB(Vector3::Zero(), H);
				RigidBodyParam Parameters; Parameters.rigidType = RigidType::Dynamic; Parameters.maxContactImpulse = FLT_MAX;
				auto* Body = m_World->CreateRigidBody(Shape, Parameters)->CastDynamic();
				Body->SetRestitution(0); Body->SetFriction(0.7f, 0.8f); Bodies.push_back(Body);
				SceneObjectInstance Object; Object.Id = Name + "_" + std::to_string(I); Object.GeometryPtr = Shape;
				Object.Color = Link.Joint == RagdollReducedJointType::Fixed ? Vector4(0.9f, 0.9f, 0.9f, 1) : Color;
				m_Objects.push_back(Object);
			}
			return m_World->CreateRagdoll(Settings, Bodies);
		};
		RagdollSettings Chain = RagdollSettings::Reduced(); Chain.Links.resize(ChainLinks + 1);
		std::vector<Vector3> ChainExtents(Chain.Links.size(), ChainExtent); ChainExtents[0] = BaseExtent;
		Chain.Links[0].ParentToJoint.pos = Vector3(-8, 7, -6);
		for (int I = 1; I <= ChainLinks; ++I)
		{
			auto& Link = Chain.Links[I]; Link.Parent = I - 1; Link.Joint = RagdollReducedJointType::Revolute;
			Link.ParentToJoint.pos = Vector3(0, I == 1 ? 0 : -ChainLength, 0); Link.CenterOfMass = Vector3(0, -ChainLength / 2, 0);
		}
		auto* Hanging = AddTree("ragdoll_chain", Chain, ChainExtents, Vector4(0.95f, 0.56f, 0.18f, 1));
		auto Configuration = Hanging->GetDynamics().GetConfiguration(); Configuration[1].Position = 0.5;
		Hanging->SetConfiguration(Configuration);

		RagdollSettings Arm = RagdollSettings::Reduced(); Arm.Links.resize(ArmLinks);
		for (int I = 0; I < ArmLinks; ++I)
		{
			auto& Link = Arm.Links[I]; Link.Parent = I - 1;
			Link.Joint = I + 1 == ArmLinks ? RagdollReducedJointType::Prismatic : RagdollReducedJointType::Revolute;
			Link.Axis = Link.Joint == RagdollReducedJointType::Prismatic ? -Vector3::UnitY() : Vector3::UnitZ();
			Link.ParentToJoint.pos = I == 0 ? Vector3(0, 5, -6) : Vector3(0, -ArmLength, 0);
			Link.CenterOfMass = Vector3(0, -ArmLength / 2, 0);
		}
		auto* Driven = AddTree("ragdoll_arm", Arm, std::vector<Vector3>(ArmLinks, ArmExtent), Vector4(0.2f, 0.8f, 0.7f, 1));
		const float Targets[] = { 0.6f, -0.8f, 0.4f };
		constexpr float AngularLimit = 1.2f, SliderTravel = 0.6f, DriveStiffness = 40, DriveDamping = 12, DriveForce = 200;
		for (int I = 0; I < ArmLinks; ++I)
		{
			const bool Slider = I + 1 == ArmLinks; RagdollControl Control;
			Control.LimitEnabled = true; Control.Lower = Slider ? 0 : -AngularLimit; Control.Upper = Slider ? SliderTravel : AngularLimit;
			Control.Drive.Settings.PositionEnabled = true; Control.Drive.Settings.Acceleration = true;
			Control.Drive.Settings.Stiffness = DriveStiffness; Control.Drive.Settings.Damping = DriveDamping; Control.Drive.Settings.MaxForce = DriveForce;
			Control.Drive.TargetPosition = Targets[I]; Driven->SetControl(I, Control);
		}

		RagdollSettings Floating = RagdollSettings::Reduced(); Floating.Links.resize(FloatingLinks);
		for (int I = 0; I < FloatingLinks; ++I)
		{
			auto& Link = Floating.Links[I]; Link.Parent = I - 1;
			Link.Joint = I == 0 ? RagdollReducedJointType::Floating : RagdollReducedJointType::Spherical;
			Link.ParentToJoint.pos = I == 0 ? Vector3(4, 4, -6) : Vector3(FloatingLength, 0, 0);
			Link.CenterOfMass = Vector3(FloatingLength / 2, 0, 0);
		}
		auto* Falling = AddTree("ragdoll_floating", Floating, std::vector<Vector3>(FloatingLinks, FloatingExtent), Vector4(0.6f, 0.4f, 0.9f, 1));
		auto Velocity = Falling->GetDynamics().GetVelocity(); Velocity[2] = 0.2; Velocity[5] = 0.5; Falling->SetVelocity(Velocity);

		// Independent shafts share a fixed root so gear, rack and mimic rows close
		// loops inside one reduced ragdoll. Bars visualize rotation; teeth are not
		// collision geometry. The periodic input keeps the rack within the scene.
		RagdollSettings Transmission = RagdollSettings::Reduced(); Transmission.Links.resize(5);
		Transmission.Links[0].ParentToJoint.pos = Vector3(-5, 5, -10);
		const Vector3 ShaftPositions[] = { Vector3(-1.3f, 0, 0), Vector3(1.3f, 0, 0), Vector3(1.3f, -1.4f, 0), Vector3(4.2f, 0, 0) };
		for (size_t I = 1; I < Transmission.Links.size(); ++I)
		{
			auto& Link = Transmission.Links[I]; Link.Parent = 0; Link.ParentToJoint.pos = ShaftPositions[I - 1];
			Link.Joint = I == 3 ? RagdollReducedJointType::Prismatic : RagdollReducedJointType::Revolute;
			Link.Axis = I == 3 ? Vector3::UnitX() : Vector3::UnitZ();
		}
		const Vector3 InputExtent(1, 0.12f, 0.18f), OutputExtent(0.65f, 0.12f, 0.18f), RackExtent(1, 0.16f, 0.25f);
		auto* Gears = AddTree("mechanism_transmission", Transmission, { BaseExtent, InputExtent, OutputExtent, RackExtent, OutputExtent }, Vector4(0.9f, 0.3f, 0.2f, 1));
		constexpr double GearRatio = 1.5, PinionRadius = 0.65, MimicRatio = -1;
		Gears->CreateMechanism(RagdollMechanismSettings::Gear(1, 2, GearRatio));
		Gears->CreateMechanism(RagdollMechanismSettings::Rack(2, 3, PinionRadius));
		Gears->CreateMechanism(RagdollMechanismSettings::Mimic(1, 4, MimicRatio));
		RagdollControl Motor; Motor.Drive.Settings.PositionEnabled = true; Motor.Drive.Settings.Stiffness = DriveStiffness;
		Motor.Drive.Settings.Damping = DriveDamping; Motor.Drive.Settings.MaxForce = DriveForce;
		Gears->SetControl(0, Motor); m_MechanismDemoDrivers.push_back(Gears);

		// Two winches compare an inextensible cable with a compliant one. Moving one
		// weight pays cable through two authored guides and raises the other weight.
		// Both use the same rendered tube, sampled directly from tendon attachments;
		// no extra rope simulation or contact constraint changes their dynamics.
		constexpr float GuideOffset = 1.25f, HangingLength = 2.5f, CableRadius = 0.04f;
		constexpr double CableStiffness = 400, CableDamping = 30;
		for (int Mode = 0; Mode < 2; ++Mode)
		{
			const std::string Name = "mechanism_cable_" + std::to_string(Mode);
			const Vector4 Color = Mode == 0 ? Vector4(0.95f, 0.75f, 0.15f, 1) : Vector4(0.85f, 0.35f, 0.75f, 1);
			RagdollSettings Cable = RagdollSettings::Reduced(); Cable.Links.resize(3); Cable.Links[0].ParentToJoint.pos = Vector3(5 + Mode * 6.0f, 6, -10);
			for (int I = 1; I < 3; ++I)
			{
				auto& Link = Cable.Links[I]; Link.Parent = 0; Link.Joint = RagdollReducedJointType::Prismatic; Link.Axis = Vector3::UnitY();
				Link.ParentToJoint.pos = Vector3(I == 1 ? -GuideOffset : GuideOffset, -HangingLength, 0);
			}
			auto* Winch = AddTree(Name, Cable, { Vector3(GuideOffset + BaseExtent.x, 0.12f, 0.12f), Vector3(0.35f), Vector3(0.35f) }, Color);
			RagdollMechanismSettings Tendon; Tendon.Type = RagdollMechanismType::SpatialTendon;
			Tendon.Path = { { 1, Vector3::Zero() }, { 0, Vector3(-GuideOffset, 0, 0) }, { 0, Vector3(GuideOffset, 0, 0) }, { 2, Vector3::Zero() } };
			Tendon.Target = 2 * (HangingLength + GuideOffset); Tendon.TensionOnly = true; Tendon.Compliant = Mode != 0;
			Tendon.Stiffness = CableStiffness; Tendon.Damping = CableDamping; Winch->CreateMechanism(Tendon);
			Winch->SetControl(0, Motor); m_MechanismDemoDrivers.push_back(Winch);
			RopeSurfaceState Surface; Surface.TendonOwner = Winch; Surface.Radius = CableRadius;
			Surface.Surface.reset(GeometryFactory::CreateTriangleMesh()); Surface.Surface->SetSimulationEnabled(false); Surface.Surface->SetQueryEnabled(false);
			SceneObjectInstance Object; Object.Id = Name + "_surface"; Object.GeometryPtr = Surface.Surface.get(); Object.Color = Color;
			m_Objects.push_back(Object); m_RopeSurfaces.push_back(std::move(Surface));
		}
		SyncRopeSurfaces();
		auto Substeps = m_World->GetTemporalSubstepSettings(); Substeps.Enabled = true; Substeps.Substeps = DemoSubsteps;
		m_World->SetTemporalSubstepSettings(Substeps);
		TouchRenderRevision(); return true;
	}

	bool SceneWorld::CreateCharacterDemo(const JsonValue& Root, std::string* Error)
	{
		const JsonValue* Demo = Root.Find("characterDemo");
		if (!Demo || !m_CharacterDemo) return true;
		if (!Demo->IsObject()) { if (Error) *Error = "characterDemo must be an object"; return false; }
		const Vector3 Origin = ReadVector3(Demo->Find("origin"), Vector3::Zero());
		if (!std::isfinite(Origin.x) || !std::isfinite(Origin.y) || !std::isfinite(Origin.z))
		{ if (Error) *Error = "Invalid character demo origin"; return false; }
		// This optional course is procedural and belongs to the existing constraint
		// demo. It exercises stairs, a walkable ramp, a steep ramp and a lift.
		auto AddBox = [&](const std::string& Id, const Vector3& Position, const Vector3& Extent, const Quaternion& Rotation, const Vector4& Color)
		{
			Geometry* Shape = GeometryFactory::CreateOBB(Origin + Position, Extent, Rotation);
			RigidBody* Body = m_World->CreateRigidBody(Shape, RigidBodyParam());
			SceneObjectInstance Object; Object.Id = Id; Object.GeometryPtr = Shape; Object.Color = Color;
			m_Objects.push_back(Object); return Body;
		};
		const Quaternion Identity = Quaternion::One();
		AddBox("character_course_floor", Vector3(7, -0.25f, 0), Vector3(10, 0.25f, 4), Identity, Vector4(0.24f, 0.3f, 0.35f, 1));
		const int StairCount = 3;
		const float StairRise = 0.2f, StairRun = 1.0f;
		for (int I = 0; I < StairCount; ++I)
		{
			const float Height = (I + 1) * StairRise;
			AddBox("character_step_" + std::to_string(I), Vector3(2 + I * StairRun, Height / 2, 0),
				Vector3(StairRun / 2, Height / 2, 1.2f), Identity, Vector4(0.3f, 0.65f, 0.4f, 1));
		}
		Quaternion Ramp; Ramp.FromRotateZ(0.3f);
		AddBox("character_walkable_ramp", Vector3(7, 0.5f, 0), Vector3(1.8f, 0.15f, 1.2f), Ramp, Vector4(0.3f, 0.55f, 0.8f, 1));
		Ramp.FromRotateZ(1.0f);
		AddBox("character_steep_ramp", Vector3(10, 0.8f, -2), Vector3(1, 0.15f, 0.7f), Ramp, Vector4(0.8f, 0.35f, 0.3f, 1));
		m_CharacterPlatform = AddBox("character_lift", Vector3(12, 0.1f, 0), Vector3(1.2f, 0.1f, 1.2f), Identity, Vector4(0.7f, 0.5f, 0.9f, 1));
		m_CharacterPlatformOrigin = m_CharacterPlatform->X;
		RobotControllerSettings Settings;
		Settings.Shape = m_CharacterBox ? RobotShape::Box : RobotShape::Capsule;
		m_Character = m_World->CreateRobotController(Settings, Origin + Vector3(0, 1.1f, 0));
		SceneObjectInstance Object; Object.Id = "character"; Object.GeometryPtr = m_Character->GetGeometry();
		Object.Color = Vector4(1, 0.8f, 0.2f, 1); m_Objects.push_back(Object);
		TouchRenderRevision(); return true;
	}

	bool SceneWorld::CreateVehicles(const JsonValue& root, const std::string& sourceName, std::string* errorMessage)
	{
		const JsonValue* vehicles = root.Find("vehicles");
		if (!vehicles) return true;
		auto Fail = [&](const std::string& message) -> bool
		{
			if (errorMessage) *errorMessage = sourceName + ": vehicles: " + message;
			return false;
		};
		if (!vehicles->IsArray()) return Fail("expected an array");
		for (const JsonValue& value : vehicles->arrayValue)
		{
			if (!value.IsObject()) return Fail("each entry must be an object");
			const std::string id = ReadString(value.Find("id"), "vehicle");
			const std::string chassisId = ReadString(value.Find("chassis"), "");
			RigidBody* chassis = nullptr;
			for (const SceneObjectInstance& object : m_Objects)
			{
				if (object.Id == id + "_body") return Fail("duplicate vehicle id '" + id + "'");
				for (int wheel = 0; wheel < 4; ++wheel)
					if (object.Id == id + "_wheel_" + std::to_string(wheel))
						return Fail("duplicate wheel id for '" + id + "'");
				if (object.Id == chassisId && object.GeometryPtr)
					chassis = object.GeometryPtr->GetParent<RigidBody>();
			}
			if (id.empty() || chassisId.empty() || !chassis || !chassis->CastDynamic())
				return Fail("a vehicle requires an id and an existing dynamic chassis");
			bool valid = true;
			auto Scalar = [&](const JsonValue& object, const char* key, float& result)
			{
				const JsonValue* number = object.Find(key);
				if (!number) return;
				if (!number->IsNumber() || !std::isfinite(number->numberValue)
					|| number->numberValue < 0.0 || number->numberValue > FLT_MAX) valid = false;
				else result = (float)number->numberValue;
			};
			auto Vector = [&](const char* key, Vector3& result)
			{
				const JsonValue* array = value.Find(key);
				if (!array) return;
				if (!array->IsArray() || array->arrayValue.size() != 3) { valid = false; return; }
				for (int axis = 0; axis < 3; ++axis)
				{
					const JsonValue& number = array->arrayValue[axis];
					if (!number.IsNumber() || !std::isfinite(number.numberValue) || std::abs(number.numberValue) > FLT_MAX)
						valid = false;
					else result[axis] = (float)number.numberValue;
				}
			};
			VehicleDeformationSettings settings;
			Vector("halfExtent", settings.HalfExtent);
			const JsonValue* cells = value.Find("cells");
			if (cells)
			{
				if (!cells->IsArray() || cells->arrayValue.size() != 3) valid = false;
				else for (int axis = 0; axis < 3; ++axis)
				{
					const JsonValue& number = cells->arrayValue[axis];
					if (!number.IsNumber() || !std::isfinite(number.numberValue) || number.numberValue < 2
						|| number.numberValue > 16 || std::floor(number.numberValue) != number.numberValue) valid = false;
					else settings.Cells[axis] = (int)number.numberValue;
				}
			}
			Scalar(value, "cabinHeight", settings.CabinHeight);
			Scalar(value, "shellMass", settings.ShellMass);
			Scalar(value, "crushZoneLength", settings.CrushZoneLength);
			float railYieldForce = 0;
			Scalar(value, "railYieldForce", railYieldForce);
			Scalar(value, "impactRadius", settings.ImpactRadius);
			Scalar(value, "impulseScale", settings.ImpulseScale);
			Scalar(value, "minImpactImpulse", settings.MinImpactImpulse);
			Scalar(value, "maxDeformationImpulse", settings.MaxDeformationImpulse);
			const JsonValue* material = value.Find("material");
			if (material && !material->IsObject()) return Fail("material must be an object");
			if (material)
			{
				auto Iterations = [&](const char* key, int& result)
				{
					const JsonValue* number = material->Find(key);
					if (!number) return;
					if (!number->IsNumber() || !std::isfinite(number->numberValue) || number->numberValue < 1
						|| number->numberValue > 128 || std::floor(number->numberValue) != number->numberValue) valid = false;
					else result = (int)number->numberValue;
				};
				Iterations("substeps", settings.Material.Substeps);
				Iterations("iterations", settings.Material.SolverIterations);
				Scalar(*material, "compliance", settings.Material.EdgeCompliance);
				Scalar(*material, "volumeCompliance", settings.Material.VolumeCompliance);
				Scalar(*material, "damping", settings.Material.Damping);
				Scalar(*material, "yieldStrain", settings.Material.YieldStrain);
				Scalar(*material, "plasticCreep", settings.Material.PlasticCreep);
				Scalar(*material, "maxPlasticStrain", settings.Material.MaxPlasticStrain);
			}
			Vector3 position = Vector3::Zero();
			Vector("position", position);
			auto FiniteNumber = [](const JsonValue* number)
			{
				return number && number->IsNumber() && std::isfinite(number->numberValue)
					&& std::abs(number->numberValue) <= FLT_MAX;
			};
			const JsonValue* quaternion = value.Find("rotation");
			if (quaternion)
			{
				if (!quaternion->IsArray() || quaternion->arrayValue.size() != 4) valid = false;
				else for (const JsonValue& number : quaternion->arrayValue) valid = valid && FiniteNumber(&number);
			}
			else if (value.Find("rotationAxis") || value.Find("rotationDegrees"))
			{
				Vector3 axis = Vector3::Zero();
				Vector("rotationAxis", axis);
				valid = valid && std::isfinite(axis.SquareLength()) && axis.SquareLength() > 1.0e-12f
					&& FiniteNumber(value.Find("rotationDegrees"));
			}
			Quaternion rotation = ReadQuaternion(&value, Quaternion::One());
			const float rotationLength = rotation.SquareLength();
			float radius = 0.4f, width = 0.25f;
			float track = 2.0f * settings.HalfExtent.x + 0.1f, wheelbase = 1.3f * settings.HalfExtent.z;
			float wheelHeight = -settings.HalfExtent.y;
			float wheelBreakStrain = 0, wheelMass = settings.ShellMass * 0.20f;
			Scalar(value, "wheelBreakStrain", wheelBreakStrain);
			Scalar(value, "wheelMass", wheelMass);
			Scalar(value, "wheelRadius", radius);
			Scalar(value, "wheelWidth", width);
			Scalar(value, "wheelTrack", track);
			Scalar(value, "wheelbase", wheelbase);
			const JsonValue* height = value.Find("wheelHeight");
			if (height)
			{
				if (!height->IsNumber() || !std::isfinite(height->numberValue) || std::abs(height->numberValue) > FLT_MAX) valid = false;
				else wheelHeight = (float)height->numberValue;
			}
			if (!valid || settings.HalfExtent.x <= 0.0f || settings.HalfExtent.y <= 0.0f || settings.HalfExtent.z <= 0.0f
				|| settings.ShellMass <= 0.0f || wheelMass <= 0.0f
				|| settings.CrushZoneLength > 0.75f * settings.HalfExtent.z
				|| (railYieldForce > 0 && settings.CrushZoneLength <= 0)
				|| settings.ImpactRadius <= 0.0f || radius <= 0.0f || width <= 0.0f || track <= 0.0f || wheelbase <= 0.0f
				|| !std::isfinite(rotationLength) || rotationLength <= 1.0e-12f)
				return Fail("invalid dimensions, material, or transform for '" + id + "'");
			rotation = rotation.Unit();
			VehicleRenderState state;
			state.Vehicle.reset(new VehicleDeformation(settings));
			if (!state.Vehicle->Attach(chassis, Transform(position, rotation)))
				return Fail("chassis is already owned by another aggregate");
			state.WheelRadius = radius;
			state.WheelBreakStrain = wheelBreakStrain;
			state.WheelMass = wheelMass;
			if (m_VehicleDynamicsDemo)
			{
				const float mass = 1.0f / chassis->InvMass;
				if (!(mass > 0) || !std::isfinite(mass)) return Fail("vehicle dynamics requires a finite positive chassis mass");
				auto Dynamics = VehicleDynamicsSettings::Roadster(mass, wheelbase, track);
				Dynamics.Forward = rotation * Vector3::UnitZ();
				Dynamics.Up = rotation * Vector3::UnitY();
				Dynamics.CenterOfPressure = position;
				for (auto& Wheel : Dynamics.Wheels)
				{
					Wheel.Radius = radius;
					Wheel.Attachment.y = wheelHeight;
					Wheel.Attachment = position + rotation * Wheel.Attachment;
					Wheel.SuspensionDirection = -Dynamics.Up;
				}
				if (!Dynamics.IsValid()) return Fail("invalid roadster settings for '" + id + "'");
				for (const auto& Wheel : Dynamics.Wheels)
					chassis->X.y = std::max(chassis->X.y, radius + Wheel.Travel - (chassis->Q * Wheel.Attachment).y);
				chassis->V = chassis->W = Vector3::Zero();
				chassis->CastDynamic()->MaxContactImpulse = FLT_MAX;
				state.Dynamics = m_World->CreateVehicle(chassis->CastDynamic(), Dynamics);
				VehicleControls Controls; Controls.Gear = 0; Controls.Brake = 1;
				state.Dynamics->SetControls(Controls);
			}
			if (railYieldForce > 0 && !state.Vehicle->CreateFrontRails(*m_World, railYieldForce))
				return Fail("invalid front crush rails for '" + id + "'");
			state.PreviousPosition = state.Vehicle->GetWorldTransform().pos;
			SceneObjectInstance shell;
			shell.Id = id + "_body";
			shell.GeometryPtr = state.Vehicle->GetRenderGeometry();
			shell.Vehicle = state.Vehicle.get();
			shell.Color = ReadColor(value.Find("color"), Vector4(0.85f, 0.23f, 0.08f, 1.0f));
			m_Objects.push_back(shell);
			// Store meshes in axle order for the solver. Scene IDs run rear to front on each side.
			for (int end : { 1, -1 })
			for (int side : { -1, 1 })
			{
				const Vector3 center(side * track * 0.5f, wheelHeight, end * wheelbase * 0.5f);
				std::unique_ptr<Geometry> wheel(GeometryFactory::CreateCylinder(
					center - Vector3(width * 0.5f, 0, 0), center + Vector3(width * 0.5f, 0, 0), radius));
				wheel->SetFilterData(chassis->Geometries().front()->GetFilterData());
				wheel->SetSimulationEnabled(false);
				wheel->SetQueryEnabled(false);
				VehicleRenderState::Wheel mount;
				mount.LocalPose = *wheel->GetTransform();
				if (wheelBreakStrain > 0)
				{
					const auto& cage = state.Vehicle->GetCage();
					float nearest = FLT_MAX;
					for (int i = 0; i < cage.GetParticleCount(); ++i)
					{
						const float distance = (cage.GetParticle(i).InitialPosition - center).SquareLength();
						if (distance < nearest) { nearest = distance; mount.MountNode = i; }
					}
					const auto& edges = cage.GetEdges();
					for (int i = 0; i < (int)edges.size(); ++i)
						if (edges[i].ParticleA == mount.MountNode || edges[i].ParticleB == mount.MountNode)
							mount.MountEdges.push_back(i);
				}
				SceneObjectInstance instance;
				const int sceneWheel = (side > 0 ? 2 : 0) + (end > 0 ? 1 : 0);
				instance.Id = id + "_wheel_" + std::to_string(sceneWheel);
				instance.GeometryPtr = wheel.get();
				instance.Vehicle = state.Vehicle.get();
				instance.Color = Vector4(0.075f, 0.085f, 0.1f, 1.0f);
				m_Objects.push_back(instance);
				mount.AttachedShape = std::move(wheel);
				state.Wheels.push_back(std::move(mount));
			}
			m_Vehicles.push_back(std::move(state));
		}
		SyncVehicles(0.0f);
		return true;
	}

	VehicleDeformation* SceneWorld::GetVehicle(int index)
	{
		return index >= 0 && (size_t)index < m_Vehicles.size() ? m_Vehicles[(size_t)index].Vehicle.get() : nullptr;
	}

	const VehicleDeformation* SceneWorld::GetVehicle(int index) const
	{
		return index >= 0 && (size_t)index < m_Vehicles.size() ? m_Vehicles[(size_t)index].Vehicle.get() : nullptr;
	}

	VehicleDynamics* SceneWorld::GetVehicleDynamics(int Index) const
	{
		return Index >= 0 && (size_t)Index < m_Vehicles.size() ? m_Vehicles[(size_t)Index].Dynamics : nullptr;
	}

	void SceneWorld::SyncVehicles(float dt)
	{
		bool changed = false;
		for (VehicleRenderState& state : m_Vehicles)
		{
			if (dt > 0.0f) state.Vehicle->Simulate(dt);
			const Transform frame = state.Vehicle->GetWorldTransform();
			const float travel = (frame.pos - state.PreviousPosition).Dot(frame.quat * Vector3::UnitZ());
			state.WheelAngle = std::fmod(state.WheelAngle + travel / state.WheelRadius, 2.0f * kPi);
			state.PreviousPosition = frame.pos;
			Quaternion roll;
			roll.FromRotationAxis(Vector3::UnitX(), state.WheelAngle);
			for (size_t i = 0; i < state.Wheels.size(); ++i)
			{
				auto& mount = state.Wheels[i];
				if (!mount.AttachedShape) continue;
				Transform pose = state.Dynamics ? state.Dynamics->GetWheelTransform(i) :
					Transform(frame.LocalToWorld(mount.LocalPose.pos), frame.quat * roll);
				pose.quat *= mount.LocalPose.quat;
				float strain = 0;
				Vector3 localVelocity = Vector3::Zero();
				if (mount.MountNode >= 0)
				{
					const auto& cage = state.Vehicle->GetCage();
					const auto& node = cage.GetParticle(mount.MountNode);
					pose.pos += frame.quat * (node.Position - node.InitialPosition);
					localVelocity = node.Velocity;
					for (int j : mount.MountEdges)
					{
						const auto& edge = cage.GetEdges()[j];
						strain = std::max(strain, std::abs(edge.RestLength / edge.InitialRestLength - 1.0f));
					}
				}
				mount.AttachedShape->SetTransform(pose.pos, pose.quat);
				if (mount.MountNode < 0 || strain < state.WheelBreakStrain) continue;

				const RigidBody* chassis = state.Vehicle->GetBody();
				const Vector3 axle = pose.quat * Vector3::UnitY();
				const float spin = state.Dynamics ? state.Dynamics->GetWheels()[i].AngularSpeed :
					(dt > 0 ? travel / (dt * state.WheelRadius) : 0);
				RigidBodyParam param;
				param.rigidType = RigidType::Dynamic;
				param.linearVelocity = chassis->V + chassis->W.Cross(pose.pos - chassis->X) + frame.quat * localVelocity;
				param.angularVelocity = chassis->W + axle * spin;
				param.maxContactImpulse = FLT_MAX;
				mount.AttachedShape->SetSimulationEnabled(true);
				mount.AttachedShape->SetQueryEnabled(true);
				auto* loose = m_World->CreateRigidBody(mount.AttachedShape.release(), param);
				const float invMass = 1.0f / state.WheelMass;
				loose->InvInertia *= invMass / loose->InvMass;
				loose->InvMass = invMass;
				loose->SetRestitution(0.12f);
				if (state.Dynamics) state.Dynamics->DetachWheel(i);
				changed = true;
			}
			if (state.LastRevision != state.Vehicle->GetRevision())
			{
				state.LastRevision = state.Vehicle->GetRevision();
				changed = true;
			}
		}
		if (changed) TouchRenderRevision();
	}

	bool SceneWorld::CreateObject(const SceneObjectDesc& desc, std::string* errorMessage)
	{
		if (!desc.FrictionValid)
		{
			if (errorMessage) *errorMessage = "invalid frictionDynamic or frictionStatic for object " + desc.Id;
			return false;
		}
		if (!desc.SDFValid)
		{
			if (errorMessage) *errorMessage = "invalid sdfCollision or sdfResolution for object " + desc.Id;
			return false;
		}
		if (!desc.ContactOffsetsValid)
		{
			if (errorMessage) *errorMessage = "invalid contactOffset or restOffset for object " + desc.Id;
			return false;
		}
		if (!desc.FilterDataValid)
		{
			if (errorMessage) *errorMessage = "invalid collisionLayer or collisionMask for object " + desc.Id;
			return false;
		}
		if (desc.Type == "destructionBox")
		{
			return CreateDestructionBox(desc, errorMessage);
		}
		if (desc.Type == "destructionGlass")
		{
			return CreateDestructionGlass(desc, errorMessage);
		}
		if (desc.Type == "sphereRing")
		{
			std::vector<Geometry*> geometries;
			geometries.reserve(desc.SphereCount);
			for (int i = 0; i < desc.SphereCount; ++i)
			{
				const float angle = 2.0f * PI * (float)i / (float)desc.SphereCount;
				const Vector3 localPosition(desc.RingRadius * cosf(angle), 0.0f, desc.RingRadius * sinf(angle));
				Geometry* geometry = GeometryFactory::CreateSphere(desc.Position + desc.Rotation * localPosition, desc.Radius);
				geometry->SetQueryEnabled(desc.QueryEnabled);
				geometry->SetSimulationEnabled(desc.SimulationEnabled);
				geometry->SetFilterData(desc.FilterData);
				geometry->SetContactOffsets(desc.ContactOffset, desc.RestOffset);
				geometries.push_back(geometry);
			}

			RigidBodyParam bodyParam;
			bodyParam.rigidType = desc.BodyType;
			bodyParam.linearVelocity = desc.LinearVelocity;
			bodyParam.angularVelocity = desc.AngularVelocity;
			bodyParam.disableGravity = desc.DisableGravity;
			bodyParam.motionType = desc.EnableCCD ? MorionType::Continuous : MorionType::Discrete;
			RigidBody* body = m_World->CreateRigidBody(geometries, bodyParam, Transform(desc.Position, desc.Rotation));
			if (body == nullptr)
			{
				for (Geometry* geometry : geometries)
				{
					delete geometry;
				}
				if (errorMessage)
				{
					*errorMessage = "failed to create sphere ring body for object " + desc.Id;
				}
				return false;
			}
			if (desc.HasInvMass && body->mRigidType == RigidType::Dynamic)
			{
				body->InvMass = desc.InvMass;
			}
			if (desc.HasRestitution)
			{
				body->SetRestitution(desc.Restitution);
			}
			if (desc.HasFriction) body->SetFriction(desc.FrictionDynamic, desc.FrictionStatic);
			if (desc.HasDensity && !ApplyBodyDensity(body, desc.Density))
			{
				if (errorMessage)
				{
					*errorMessage = "failed to apply density to object " + desc.Id;
				}
				return false;
			}
			if (desc.MouseControl && !geometries.empty())
			{
				RegisterMouseControlGeometry(geometries.front());
			}

			for (size_t i = 0; i < geometries.size(); ++i)
			{
				SceneObjectInstance instance;
				std::ostringstream id;
				id << desc.Id << "_sphere_" << i;
				instance.Id = id.str();
				instance.GeometryPtr = geometries[i];
				instance.Color = desc.Color;
				instance.RenderBounds = desc.RenderBounds;
				instance.RenderEnabled = desc.RenderEnabled;
				m_Objects.push_back(instance);
			}
			TouchRenderRevision();
			return true;
		}

		Geometry* geometry = CreateGeometry(desc, errorMessage);
		if (geometry == nullptr)
		{
			return false;
		}
		geometry->SetQueryEnabled(desc.QueryEnabled);
		geometry->SetSimulationEnabled(desc.SimulationEnabled);
		geometry->SetFilterData(desc.FilterData);
		geometry->SetContactOffsets(desc.ContactOffset, desc.RestOffset);

		RigidBodyParam bodyParam;
		bodyParam.rigidType = desc.BodyType;
		bodyParam.linearVelocity = desc.LinearVelocity;
		bodyParam.angularVelocity = desc.AngularVelocity;
		bodyParam.disableGravity = desc.DisableGravity;
		bodyParam.motionType = desc.EnableCCD ? MorionType::Continuous : MorionType::Discrete;

		RigidBody* body = m_World->CreateRigidBody(geometry, bodyParam);
		if (body == nullptr)
		{
			delete geometry;
			if (errorMessage)
			{
				*errorMessage = "failed to create rigid body for object " + desc.Id;
			}
			return false;
		}

		if (desc.HasInvMass && body->mRigidType == RigidType::Dynamic)
		{
			body->InvMass = desc.InvMass;
		}
		if (desc.HasRestitution)
		{
			body->SetRestitution(desc.Restitution);
		}
		if (desc.HasFriction) body->SetFriction(desc.FrictionDynamic, desc.FrictionStatic);
		if (desc.HasDensity && !ApplyBodyDensity(body, desc.Density))
		{
			if (errorMessage)
			{
				*errorMessage = "failed to apply density to object " + desc.Id;
			}
			return false;
		}
		if (desc.MouseControl)
		{
			RegisterMouseControlGeometry(geometry);
		}

		SceneObjectInstance instance;
		instance.Id = desc.Id;
		instance.GeometryPtr = geometry;
		instance.Color = desc.Color;
		instance.RenderBounds = desc.RenderBounds;
		instance.RenderEnabled = desc.RenderEnabled;
		m_Objects.push_back(instance);
		if (HeightField3* field = geometry->GetShapeObj<HeightField3>())
			m_TerrainRevisions.push_back(std::make_pair(geometry, field->GetRevision()));
		TouchRenderRevision();
		return true;
	}

	bool SceneWorld::CreateDestructionBox(const SceneObjectDesc& desc, std::string* errorMessage)
	{
		if (m_World == nullptr)
		{
			return false;
		}

		const Vector3 AuthoringPosition = (m_AuthoredDestructionDemo || m_DestructionCacheDemo) ? Vector3::Zero() : desc.Position;
		const Quaternion AuthoringRotation = (m_AuthoredDestructionDemo || m_DestructionCacheDemo) ? Quaternion::One() : desc.Rotation;
		std::unique_ptr<DestructionSet> destructSet(new DestructionSet((m_AuthoredDestructionDemo || m_DestructionCacheDemo) ? nullptr : m_World.get()));
		destructSet->GetDestructionConstants().FilterData = desc.FilterData;
		destructSet->GetDestructionConstants().ContactOffset = desc.ContactOffset;
		destructSet->GetDestructionConstants().RestOffset = desc.RestOffset;
		const Vector3 fullSize = desc.HalfExtent * 2.0f;
		const float sourceVolume = std::max(fullSize.x * fullSize.y * fullSize.z, 1e-3f);
		const float totalMass = desc.HasInvMass && desc.InvMass > 1e-6f ? 1.0f / desc.InvMass : sourceVolume;
		if (desc.DestructionUseHierarchy)
		{
			if (desc.DestructionFracture.Levels.empty())
			{
				if (errorMessage)
				{
					*errorMessage = "hierarchical fracture has no levels for object " + desc.Id;
				}
				return false;
			}

			DynamicMesh sourceMesh;
			if (!BuildBoxDynamicMesh(AuthoringPosition, desc.HalfExtent, AuthoringRotation, &sourceMesh)
				|| !destructSet->BuildHierarchicalFracture(sourceMesh, desc.DestructionFracture, totalMass,
					desc.BodyType == RigidType::Static, desc.Id))
			{
				if (errorMessage)
				{
					*errorMessage = "failed to build hierarchical fracture for object " + desc.Id;
				}
				return false;
			}
		}
		else
		{
			int piecesX = desc.DestructionPiecesX;
			int piecesY = desc.DestructionPiecesY;
			int piecesZ = desc.DestructionPiecesZ;
			ResolveVoxelGridCounts(piecesX, piecesY, piecesZ);

			const int targetPieceCount = piecesX * piecesY * piecesZ;
			const Vector3 cellSize(fullSize.x / piecesX, fullSize.y / piecesY, fullSize.z / piecesZ);
			const Vector3 cellHalfExtent = cellSize * 0.5f;
			const float chunkMass = std::max(totalMass / targetPieceCount, 1e-3f);
			std::vector<int> cellToChunk((size_t)targetPieceCount, -1);
			for (int z = 0; z < piecesZ; ++z)
			{
				for (int y = 0; y < piecesY; ++y)
				{
					for (int x = 0; x < piecesX; ++x)
					{
						const int cellIndex = x + piecesX * (y + piecesY * z);
						const Vector3 localCenter(
							-desc.HalfExtent.x + cellSize.x * (x + 0.5f),
							-desc.HalfExtent.y + cellSize.y * (y + 0.5f),
							-desc.HalfExtent.z + cellSize.z * (z + 0.5f));
						const Vector3 worldCenter = AuthoringPosition + AuthoringRotation * localCenter;
						DynamicMesh cellMesh;
						if (!BuildBoxDynamicMesh(worldCenter, cellHalfExtent, AuthoringRotation, &cellMesh))
						{
							if (errorMessage)
							{
								*errorMessage = "failed to build destruction cell for object " + desc.Id;
							}
							return false;
						}

						std::ostringstream chunkName;
						chunkName << desc.Id << "_cell_" << cellIndex;
						cellToChunk[(size_t)cellIndex] = destructSet->AddChunk(cellMesh, chunkMass, desc.BodyType == RigidType::Static, chunkName.str());
					}
				}
			}

			int bondIndex = 0;
			const int neighborOffsets[3][3] = { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } };
			for (int z = 0; z < piecesZ; ++z)
			{
				for (int y = 0; y < piecesY; ++y)
				{
					for (int x = 0; x < piecesX; ++x)
					{
						const int cellIndex = x + piecesX * (y + piecesY * z);
						const int chunkA = cellToChunk[(size_t)cellIndex];
						for (const int* offset : neighborOffsets)
						{
							const int neighborX = x + offset[0];
							const int neighborY = y + offset[1];
							const int neighborZ = z + offset[2];
							if (neighborX >= piecesX || neighborY >= piecesY || neighborZ >= piecesZ)
							{
								continue;
							}
							const int neighborIndex = neighborX + piecesX * (neighborY + piecesY * neighborZ);
							const int chunkB = cellToChunk[(size_t)neighborIndex];
							if (chunkA >= 0 && chunkB >= 0 && destructSet->AddBond(chunkA, chunkB, DestructionBondType::Connect, desc.DestructionBondStrength, bondIndex))
							{
								++bondIndex;
							}
						}
					}
				}
			}
		}

		DestructionConstants& constants = destructSet->GetDestructionConstants();
		constants.ImpulseProcessPerTick = 4;
		constants.PartitionProcessPerTick = desc.DestructionPartitionProcessPerTick;
		constants.PartitionProcessIntervalTicks = desc.DestructionPartitionProcessIntervalTicks;
		constants.PartitionSeparationAttenuation = std::max(0.0f, std::min(desc.DestructionPartitionSeparationAttenuation, 1.0f));
		constants.EnableClusterCollisionProxy = desc.DestructionClusterCollisionProxy;
		constants.ImpulseRadius = std::max(desc.DestructionImpulseRadius, 0.01f);
		constants.ImpulseBreakStrength = std::max(desc.DestructionImpulseBreakStrength, 0.0f);
		constants.MinClusterVolume = std::max(desc.DestructionMinClusterVolume, 0.0f);
		for (int level = 0; level < 8; ++level)
		{
			constants.Levels[level].BreakThreshold = std::max(desc.DestructionBreakThreshold, 0.0f);
			constants.Levels[level].PartitionSize = std::max(desc.DestructionPartitionSize, 2);
		}

		if (m_AuthoredDestructionDemo || m_DestructionCacheDemo)
		{
			try
			{
				const auto Asset = DestructionAsset::Capture(*destructSet);
				destructSet.reset(new DestructionSet(m_World.get())); destructSet->InitializeAsset(Asset,Transform(desc.Position,desc.Rotation));
			}
			catch (const std::exception& Error) { if (errorMessage) *errorMessage = "destruction asset " + desc.Id + ": " + Error.what(); return false; }
		}
		else destructSet->RebuildClustersFromGraph();
		for (DestructionCluster* cluster : destructSet->GetClusters())
		{
			RigidBodyDynamic* body = cluster ? cluster->GetRigidBody() : nullptr;
			if (body == nullptr)
			{
				continue;
			}
			body->DisableGravity = desc.DisableGravity;
			body->mMotionType = desc.EnableCCD ? MorionType::Continuous : MorionType::Discrete;
			body->SetLinearVelocity(desc.LinearVelocity);
			body->SetAngularVelocity(desc.AngularVelocity);
			if (desc.HasRestitution)
			{
				body->SetRestitution(desc.Restitution);
			}
			if (desc.HasFriction) body->SetFriction(desc.FrictionDynamic, desc.FrictionStatic);
			if (desc.DestructionInitiallyFixed)
			{
				body->Freeze();
			}
		}

		DestructionSet* rawSet = destructSet.get();

		m_DestructionSets.push_back(std::move(destructSet));

		DestructionRenderState renderState;
		renderState.Set = rawSet;
		renderState.Id = desc.Id;
		renderState.Color = desc.Color;
		renderState.RenderBounds = desc.RenderBounds;
		renderState.IntactGeometry.reset(GeometryFactory::CreateOBB(
			desc.Position, desc.HalfExtent, desc.Rotation));
		renderState.RenderIntactUntilFractured = renderState.IntactGeometry != nullptr;
		if (renderState.RenderIntactUntilFractured)
		{
			const std::vector<DestructionCluster*> clusters = rawSet->GetClusters();
			if (clusters.size() == 1 && clusters[0] != nullptr)
			{
				renderState.IntactLocalTransform = clusters[0]->GetClusterWorldTransform()
					.TransformInv(*renderState.IntactGeometry->GetTransform());
			}
		}
		m_DestructionRenderStates.push_back(std::move(renderState));
		SyncDestructionObjects(true);
		return true;
	}

	bool SceneWorld::CreateDestructionGlass(const SceneObjectDesc& desc, std::string* errorMessage)
	{
		if (m_World == nullptr)
		{
			return false;
		}

		const Vector3 AuthoringPosition = (m_AuthoredDestructionDemo || m_DestructionCacheDemo) ? Vector3::Zero() : desc.Position;
		const Quaternion AuthoringRotation = (m_AuthoredDestructionDemo || m_DestructionCacheDemo) ? Quaternion::One() : desc.Rotation;
		DynamicMesh sourceMesh;
		if (!BuildBoxDynamicMesh(AuthoringPosition, desc.HalfExtent, AuthoringRotation, &sourceMesh))
		{
			if (errorMessage)
			{
				*errorMessage = "failed to build source glass mesh for object " + desc.Id;
			}
			return false;
		}

		PlanarCutOptions cutOptions;
		cutOptions.SnapTolerance = 1e-5f;
		cutOptions.BoundsPaddingScale = 0.1f;
		cutOptions.RandomSeed = desc.DestructionSeed;
		cutOptions.WeldSharedEdges = true;
		cutOptions.MinTriangleCount = 4;

		std::vector<FracturePiece> pieces;
		const Vector3 glassNormal = AuthoringRotation * Vector3::UnitZ();
		if (!Fracture::VoronoiFracture2D(sourceMesh, glassNormal, pieces,
			desc.DestructionPieceCount, desc.DestructionSeed, cutOptions) || pieces.size() < 2)
		{
			if (errorMessage)
			{
				*errorMessage = "failed to create 2D Voronoi glass pieces for object " + desc.Id;
			}
			return false;
		}

		std::unique_ptr<DestructionSet> destructSet(new DestructionSet((m_AuthoredDestructionDemo || m_DestructionCacheDemo) ? nullptr : m_World.get()));
		destructSet->GetDestructionConstants().FilterData = desc.FilterData;
		destructSet->GetDestructionConstants().ContactOffset = desc.ContactOffset;
		destructSet->GetDestructionConstants().RestOffset = desc.RestOffset;
		const Vector3 fullSize = desc.HalfExtent * 2.0f;
		const float sourceVolume = std::max(fullSize.x * fullSize.y * fullSize.z, 1e-3f);
		const float totalMass = desc.HasInvMass && desc.InvMass > 1e-6f ? 1.0f / desc.InvMass : sourceVolume;
		const float pieceMass = std::max(totalMass / static_cast<float>(pieces.size()), 1e-3f);
		std::vector<int> pieceToChunk(pieces.size(), -1);
		for (size_t pieceIndex = 0; pieceIndex < pieces.size(); ++pieceIndex)
		{
			std::ostringstream chunkName;
			chunkName << desc.Id << "_piece_" << pieceIndex;
			pieceToChunk[pieceIndex] = destructSet->AddChunk(pieces[pieceIndex].Mesh, pieceMass, false, chunkName.str());
		}

		int bondIndex = 0;
		for (size_t pieceIndex = 0; pieceIndex < pieces.size(); ++pieceIndex)
		{
			for (int neighborIndex : pieces[pieceIndex].NeighborPieceIndices)
			{
				if (neighborIndex <= static_cast<int>(pieceIndex) || static_cast<size_t>(neighborIndex) >= pieces.size())
				{
					continue;
				}
				if (destructSet->AddBond(pieceToChunk[pieceIndex], pieceToChunk[static_cast<size_t>(neighborIndex)],
					DestructionBondType::Connect, desc.DestructionBondStrength, bondIndex))
				{
					++bondIndex;
				}
			}
		}
		if (bondIndex == 0)
		{
			if (errorMessage)
			{
				*errorMessage = "2D Voronoi glass produced no connected pieces for object " + desc.Id;
			}
			return false;
		}

		DestructionConstants& constants = destructSet->GetDestructionConstants();
		constants.ImpulseProcessPerTick = 8;
		constants.PartitionProcessPerTick = std::max(desc.DestructionPartitionProcessPerTick, 1);
		constants.PartitionProcessIntervalTicks = std::max(desc.DestructionPartitionProcessIntervalTicks, 1);
		constants.PartitionSeparationAttenuation = std::max(0.0f, std::min(desc.DestructionPartitionSeparationAttenuation, 1.0f));
		constants.EnableClusterCollisionProxy = desc.DestructionClusterCollisionProxy;
		constants.ImpulseRadius = std::max(desc.DestructionImpulseRadius, 0.01f);
		constants.ImpulseBreakStrength = std::max(desc.DestructionImpulseBreakStrength, 0.0f);
		constants.MinClusterVolume = std::max(desc.DestructionMinClusterVolume, 0.0f);
		for (int level = 0; level < 8; ++level)
		{
			constants.Levels[level].BreakThreshold = std::max(desc.DestructionBreakThreshold, 0.0f);
			constants.Levels[level].PartitionSize = std::max(desc.DestructionPartitionSize, 2);
		}

		if (m_AuthoredDestructionDemo || m_DestructionCacheDemo)
		{
			try
			{
				const auto Asset = DestructionAsset::Capture(*destructSet);
				destructSet.reset(new DestructionSet(m_World.get())); destructSet->InitializeAsset(Asset,Transform(desc.Position,desc.Rotation));
			}
			catch (const std::exception& Error) { if (errorMessage) *errorMessage = "glass asset " + desc.Id + ": " + Error.what(); return false; }
		}
		else destructSet->RebuildClustersFromGraph();
		for (DestructionCluster* cluster : destructSet->GetClusters())
		{
			RigidBodyDynamic* body = cluster ? cluster->GetRigidBody() : nullptr;
			if (body == nullptr)
			{
				continue;
			}
			body->DisableGravity = desc.DisableGravity;
			if (desc.DestructionInitiallyFixed)
			{
				body->Freeze();
			}
		}

		DestructionSet* rawSet = destructSet.get();
		m_DestructionSets.push_back(std::move(destructSet));

		DestructionRenderState renderState;
		renderState.Set = rawSet;
		renderState.Id = desc.Id;
		renderState.Color = desc.Color;
		renderState.RenderBounds = desc.RenderBounds;
		renderState.IntactGeometry.reset(GeometryFactory::CreateOBB(
			desc.Position, desc.HalfExtent, desc.Rotation));
		renderState.RenderIntactUntilFractured = renderState.IntactGeometry != nullptr;
		if (renderState.RenderIntactUntilFractured)
		{
			const std::vector<DestructionCluster*> clusters = rawSet->GetClusters();
			if (clusters.size() == 1 && clusters[0] != nullptr)
			{
				renderState.IntactLocalTransform = clusters[0]->GetClusterWorldTransform()
					.TransformInv(*renderState.IntactGeometry->GetTransform());
			}
		}
		m_DestructionRenderStates.push_back(std::move(renderState));
		SyncDestructionObjects(true);
		return true;
	}

	Geometry* SceneWorld::CreateGeometry(const SceneObjectDesc& desc, std::string* errorMessage) const
	{
		if (desc.SDFCollision || (desc.HasSDFOption && m_DistanceFieldDemo))
		{
			if (!desc.SDFValid || (desc.Type != "box" && desc.Type != "obb" && desc.Type != "sphere"))
			{
				if (errorMessage) *errorMessage = "SDF demo cooking requires a box/sphere and a valid sdfResolution: " + desc.Id;
				return nullptr;
			}
			const Vector3 Extent = desc.Type == "sphere" ? Vector3(desc.Radius) : desc.HalfExtent;
			const float PaddingRatio = 0.25f;
			const float Radius = std::max(Extent.x, std::max(Extent.y, Extent.z)) * (1 + PaddingRatio);
			const int Cells = desc.SDFResolution;
			SignedDistanceField3 Grid;
			if (!Grid.Reset(Box3(Vector3(-Radius), Vector3(Radius)), Cells, Cells, Cells, 2 * Radius / Cells))
			{
				if (errorMessage) *errorMessage = "Invalid SDF source dimensions: " + desc.Id;
				return nullptr;
			}
			for (int Z = 0; Z <= Cells; ++Z) for (int Y = 0; Y <= Cells; ++Y) for (int X = 0; X <= Cells; ++X)
			{
				const Vector3 P = Grid.GridPointToWorld(X, Y, Z);
				const Vector3 D = P.Abs() - Extent;
				Grid.At(X, Y, Z) = desc.Type == "sphere" ? P.Length() - desc.Radius
					: D.Max(Vector3::Zero()).Length() + std::min(0.0f, std::max(D.x, std::max(D.y, D.z)));
			}
			RuntimeDistanceField Runtime; DistanceFieldShape Field;
			if (!Runtime.Build(Grid, RuntimeDistanceFieldSettings(), errorMessage) || !Field.Build(Runtime, DistanceFieldShapeSettings(), errorMessage)) return nullptr;
			return GeometryFactory::CreateDistanceField(Field, desc.Position, desc.Rotation);
		}
		if (desc.Type == "voxelMesh")
		{
			if (!desc.VoxelsValid || desc.BodyType != RigidType::Static || desc.VoxelSpans.empty()
				|| !std::isfinite(desc.VoxelSize) || desc.VoxelSize <= 0.0f
				|| !std::isfinite(desc.VoxelHeight) || desc.VoxelHeight <= 0.0f)
			{
				if (errorMessage) *errorMessage = "invalid static voxel mesh: " + desc.Id;
				return nullptr;
			}
			VoxelField field;
			const Vector3 dimensions(desc.VoxelGrid.x * desc.VoxelSize,
				desc.VoxelGrid.y * desc.VoxelHeight, desc.VoxelGrid.z * desc.VoxelSize);
			field.MakeEmpty(Box3(desc.VoxelOrigin, desc.VoxelOrigin + dimensions),
				desc.VoxelGrid.x, desc.VoxelGrid.y, desc.VoxelGrid.z, desc.VoxelSize, desc.VoxelHeight);
			for (size_t i = 0; i < desc.VoxelSpans.size(); i += 4)
			{
				if (!field.AddVoxel(desc.VoxelSpans[i + 1] * desc.VoxelGrid.x + desc.VoxelSpans[i],
					(uint16_t)desc.VoxelSpans[i + 2], (uint16_t)desc.VoxelSpans[i + 3], 0.0f))
				{
					if (errorMessage) *errorMessage = "invalid voxel span: " + desc.Id;
					return nullptr;
				}
			}
			Geometry* geometry = GeometryFactory::CreateTriangleMesh();
			TriangleMesh* mesh = geometry->GetShapeObj<TriangleMesh>();
			if (!field.GenerateMesh(*mesh) || mesh->GetTriangleCount() == 0)
			{
				delete geometry;
				if (errorMessage) *errorMessage = "voxel mesh generation failed: " + desc.Id;
				return nullptr;
			}
			mesh->BuildBVH();
			geometry->SetShapeBounds(mesh->BoundingVolume);
			geometry->SetTransform(desc.Position, desc.Rotation);
			return geometry;
		}
		if (desc.Type == "heightfield")
		{
			if (!desc.TerrainValid || desc.BodyType != RigidType::Static || desc.TerrainRows < 2 || desc.TerrainColumns < 2
				|| !std::isfinite(desc.HeightScale) || desc.HeightScale <= 0.0f || !std::isfinite(desc.BaseHeight)
				|| !std::isfinite(desc.HalfExtent.x) || !std::isfinite(desc.HalfExtent.z)
				|| desc.HalfExtent.x <= 0.0f || desc.HalfExtent.z <= 0.0f)
			{
				if (errorMessage) *errorMessage = "invalid static heightfield dimensions or samples: " + desc.Id;
				return nullptr;
			}
			Geometry* geometry = GeometryFactory::CreateHeightField(
				Box3(Vector3(-desc.HalfExtent.x, 0.0f, -desc.HalfExtent.z), Vector3(desc.HalfExtent.x, 0.0f, desc.HalfExtent.z)),
				desc.TerrainColumns, desc.TerrainRows);
			HeightField3* field = geometry->GetShapeObj<HeightField3>();
			field->HeightScale = desc.HeightScale;
			field->AllocMemory();
			const std::vector<float> heights = desc.Heights.empty()
				? std::vector<float>((size_t)desc.TerrainRows * desc.TerrainColumns, desc.BaseHeight) : desc.Heights;
			field->SetHeightsRect(0, 0, desc.TerrainColumns, desc.TerrainRows, heights);
			field->RecalculateBounds();
			geometry->SetShapeBounds(field->BV);
			geometry->SetTransform(desc.Position, desc.Rotation);
			return geometry;
		}

		if (desc.Type == "box" || desc.Type == "obb")
		{
			return GeometryFactory::CreateOBB(desc.Position, desc.HalfExtent, desc.Rotation);
		}

		if (desc.Type == "plane")
		{
			return GeometryFactory::CreatePlane(desc.Position, desc.Normal);
		}

		if (desc.Type == "sphere")
		{
			return GeometryFactory::CreateSphere(desc.Position, desc.Radius);
		}

		if (desc.Type == "capsule")
		{
			const Vector3 x0 = desc.HasX0 ? desc.X0 : desc.Position - Vector3::UnitY() * (desc.Height * 0.5f);
			const Vector3 x1 = desc.HasX1 ? desc.X1 : desc.Position + Vector3::UnitY() * (desc.Height * 0.5f);
			return GeometryFactory::CreateCapsule(x0, x1, desc.Radius);
		}

		if (desc.Type == "cylinder")
		{
			const Vector3 x0 = desc.HasX0 ? desc.X0 : desc.Position - Vector3::UnitY() * (desc.Height * 0.5f);
			const Vector3 x1 = desc.HasX1 ? desc.X1 : desc.Position + Vector3::UnitY() * (desc.Height * 0.5f);
			return GeometryFactory::CreateCylinder(x0, x1, desc.Radius);
		}

		if (desc.Type == "ellipsoid")
		{
			if (desc.BodyType != RigidType::Static || !(desc.HalfExtent.x > 0 && desc.HalfExtent.y > 0 && desc.HalfExtent.z > 0))
			{
				if (errorMessage)
					*errorMessage = "Invalid static ellipsoid: " + desc.Id;
				return nullptr;
			}
			std::vector<Vector3> Vertices, Normals;
			std::vector<uint16_t> Indices;
			Sphere3(Vector3::Zero(), 1).GetMesh(Vertices, Indices, Normals);
			for (Vector3& V : Vertices) V = V * desc.HalfExtent;
			Geometry* G = GeometryFactory::CreateTriangleMesh();
			TriangleMesh* Mesh = G->GetShapeObj<TriangleMesh>();
			Mesh->SetData(Vertices.data(), Indices.data(), (uint32_t)Vertices.size(), (uint32_t)Indices.size() / 3, true, true);
			Mesh->CalculateBoundingBox(); Mesh->CalculateWeightAverageNormals(); Mesh->BuildBVH();
			G->UpdateVolumeProperties(); G->SetTransform(desc.Position, desc.Rotation);
			return G;
		}
		if (desc.Type == "stairMesh")
		{
			Geometry* geometry = GeometryFactory::CreateTriangleMesh();
			TriangleMesh* mesh = geometry->GetShapeObj<TriangleMesh>();
			mesh->Flags |= INDICES_16_BIT;
			const float totalWidth = desc.StepCount * desc.StepWidth;
			for (int step = 0; step < desc.StepCount; ++step)
			{
				const float minX = desc.Position.x - totalWidth * 0.5f + step * desc.StepWidth;
				const float maxX = minX + desc.StepWidth;
				AddFlatShadedBox(mesh,
					Vector3(minX, desc.Position.y, desc.Position.z - desc.StepDepth * 0.5f),
					Vector3(maxX, desc.Position.y + (step + 1) * desc.StepHeight, desc.Position.z + desc.StepDepth * 0.5f));
			}
			mesh->Compact();
			mesh->BuildBVH();
			geometry->SetShapeBounds(mesh->BoundingVolume);
			return geometry;
		}

		if (errorMessage)
		{
			*errorMessage = "unsupported object type '" + desc.Type + "' for object " + desc.Id;
		}
		return nullptr;
	}

	bool SceneWorld::SyncDestructionObjects(bool force)
	{
		bool changed = false;
		for (size_t StateIndex = 0; StateIndex < m_DestructionRenderStates.size(); ++StateIndex)
		{
			auto& state = m_DestructionRenderStates[StateIndex];
			if (auto* Playback = GetDestructionPlayback(StateIndex))
			{
				if (!force && state.LastPlaybackRevision == Playback->GetRevision()) continue;
				m_Objects.erase(std::remove_if(m_Objects.begin(),m_Objects.end(),[&](const SceneObjectInstance& O)
					{ return O.DestructSet == state.Set; }),m_Objects.end());
				for (const auto& Chunk : Playback->GetSample().Chunks) if (auto* Shape = Playback->GetChunkGeometry(Chunk.ChunkIndex))
				{
					SceneObjectInstance Object; Object.Id = state.Id+"_cache_"+std::to_string(Chunk.ChunkIndex);
					Object.GeometryPtr = Shape; Object.DestructSet = state.Set; Object.Color = state.Color; Object.RenderBounds = state.RenderBounds;
					m_Objects.push_back(Object);
				}
				state.LastPlaybackRevision = Playback->GetRevision(); changed = true; continue;
			}
			if (state.Set == nullptr)
			{
				continue;
			}

			const uint32_t revision = state.Set->GetClusterRevision();
			const bool renderIntact = state.RenderIntactUntilFractured
				&& state.IntactGeometry
				&& state.Set->GetGraph().GetBrokenBonds().empty();
			if (renderIntact)
			{
				const std::vector<DestructionCluster*> clusters = state.Set->GetClusters();
				if (clusters.size() == 1 && clusters[0] != nullptr)
				{
					const Transform worldTransform = clusters[0]->GetClusterWorldTransform()
						* state.IntactLocalTransform;
					state.IntactGeometry->SetTransform(worldTransform.pos, worldTransform.quat);
				}
			}
			if (!force && state.LastRevision == revision
				&& state.LastRenderedIntact == renderIntact)
			{
				continue;
			}

			m_Objects.erase(std::remove_if(m_Objects.begin(), m_Objects.end(),
				[&state](const SceneObjectInstance& object)
				{
					return object.DestructSet == state.Set;
				}), m_Objects.end());

			if (renderIntact)
			{
				SceneObjectInstance instance;
				instance.Id = state.Id + "_intact";
				instance.GeometryPtr = state.IntactGeometry.get();
				instance.DestructSet = state.Set;
				instance.Color = state.Color;
				instance.RenderBounds = state.RenderBounds;
				m_Objects.push_back(instance);
				state.LastRevision = revision;
				state.LastRenderedIntact = true;
				changed = true;
				continue;
			}

			std::vector<DestructionCluster*> clusters = state.Set->GetClusters();
			for (DestructionCluster* cluster : clusters)
			{
				if (cluster == nullptr)
				{
					continue;
				}

				RigidBodyDynamic* body = cluster->GetRigidBody();
				if (body == nullptr)
				{
					continue;
				}

				const std::vector<Geometry*>& geometries = body->Geometries();
				const std::vector<int>& sourceIndices = cluster->GetSourceIndices();
				Geometry* collisionProxy = cluster->GetCollisionProxy();
				if (collisionProxy)
				{
					std::ostringstream id;
					id << state.Id << "_cluster_" << cluster->GetID() << "_proxy";

					SceneObjectInstance instance;
					instance.Id = id.str();
					instance.GeometryPtr = collisionProxy;
					instance.DestructSet = state.Set;
					instance.Color = state.Color;
					instance.RenderBounds = state.RenderBounds;
					m_Objects.push_back(instance);
					continue;
				}

				const size_t chunkGeometryCount = std::min(geometries.size(), sourceIndices.size());
				for (size_t geometryIndex = 0; geometryIndex < chunkGeometryCount; ++geometryIndex)
				{
					Geometry* geometry = geometries[geometryIndex];
					if (geometry == nullptr)
					{
						continue;
					}

					std::ostringstream id;
					id << state.Id << "_cluster_" << cluster->GetID() << "_geom_";
					if (geometryIndex < sourceIndices.size())
					{
						id << sourceIndices[geometryIndex];
					}
					else
					{
						id << geometryIndex;
					}

					SceneObjectInstance instance;
					instance.Id = id.str();
					instance.GeometryPtr = geometry;
					instance.DestructSet = state.Set;
					instance.Color = state.Color;
					instance.RenderBounds = state.RenderBounds;
					m_Objects.push_back(instance);
				}
			}

			state.LastRevision = revision;
			state.LastRenderedIntact = false;
			changed = true;
		}

		if (changed)
		{
			TouchRenderRevision();
		}
		return changed;
	}

	std::vector<std::string> SceneWorld::GetTerrainObjectIds() const
	{
		std::vector<std::string> ids;
		for (const SceneObjectInstance& object : m_Objects)
			if (object.GeometryPtr && object.GeometryPtr->GetShapeType() == PrimitiveType::HEIGHTFIELD) ids.push_back(object.Id);
		return ids;
	}

	bool SceneWorld::RayCastTerrain(const std::string& id, const Vector3& origin, const Vector3& direction,
		float maxDistance, Vector3* hitPoint, float* localHeight) const
	{
		for (const SceneObjectInstance& object : m_Objects)
		{
			Geometry* geometry = object.GeometryPtr;
			if (object.Id != id || !geometry || geometry->GetShapeType() != PrimitiveType::HEIGHTFIELD) continue;
			RigidBody* body = geometry->GetParent<RigidBody>();
			const Transform transform = body ? body->GetGeometryTransform(geometry) : *geometry->GetTransform();
			RayCastOption option;
			option.Type = RayCastOption::RAYCAST_NEAREST;
			option.HitBothSides = true;
			option.MaxDist = maxDistance;
			RayCastResult result;
			if (!geometry->RayCast(transform, origin, direction, &option, &result)) return false;
			const Vector3 point = origin + direction * result.hitTime;
			if (hitPoint) *hitPoint = point;
			if (localHeight) *localHeight = transform.WorldToLocal(point).y;
			return true;
		}
		return false;
	}

	bool SceneWorld::SyncTerrainRevisions()
	{
		bool changed = false;
		for (auto& entry : m_TerrainRevisions)
		{
			const uint64_t revision = entry.first->GetShapeObj<HeightField3>()->GetRevision();
			changed = changed || revision != entry.second;
			entry.second = revision;
		}
		if (changed) TouchRenderRevision();
		return changed;
	}

	int SceneWorld::EditTerrain(const std::string& id, const Vector3& worldCenter, float radius, float value, int mode, float strength)
	{
		if (!m_World) return 0;
		for (const SceneObjectInstance& object : m_Objects)
		{
			if (object.Id != id || !object.GeometryPtr || object.GeometryPtr->GetShapeType() != PrimitiveType::HEIGHTFIELD) continue;
			int count = 0;
			if (mode == 0) count = m_World->ModifyHeightField(object.GeometryPtr, worldCenter, radius, value);
			else if (mode == 1) count = m_World->FillHeightField(object.GeometryPtr, worldCenter, radius, value, strength);
			else if (mode == 2) count = m_World->SmoothHeightField(object.GeometryPtr, worldCenter, radius, strength);
			SyncTerrainRevisions();
			return count;
		}
		return 0;
	}

	StrandSystem* SceneWorld::GetDemoStrands() const
	{
		if (m_World) for (size_t I=0; I<m_World->GetStrandSystemCount(); ++I)
			if (m_World->GetStrandSystem(I)==m_DemoStrands && m_DemoStrands->GetInstanceId()==m_DemoStrandInstanceId) return m_DemoStrands;
		return nullptr;
	}
	bool SceneWorld::CreateStrandDemo()
	{
		if (!m_StrandDemo || m_Name!="Rope Towing Around a Post") return true;
		RigidBody* Handle=nullptr;
		for (const auto& O : m_Objects) if (O.Id=="handle" && O.GeometryPtr) Handle=O.GeometryPtr->GetParent<RigidBody>();
		if (!Handle) return false;
		// Reuse scene 18's moving handle and tube renderer. Elliptical sections make
		// material-frame twist visible. Dimensions are demonstration parameters;
		// external groom/scalp authoring remains an asset-dependent extension.
		const int GuideCount=8, Segments=12; const float Length=1.8f, Spacing=0.14f, RootHeight=1;
		const double YoungModulus=2e7, Density=10, RadiusX=0.035, RadiusY=0.012;
		std::vector<StrandGuide> Guides;
		for (int G=0; G<GuideCount; ++G)
		{
			StrandGuide Guide; Guide.ReferenceNormal=Vector3::UnitZ(); Guide.Material.YoungModulus=YoungModulus;
			Guide.Material.Density=Density; Guide.Material.RadiusX=RadiusX; Guide.Material.RadiusY=RadiusY;
			const Vector3 Root=Handle->X+Vector3(0,RootHeight,(G-(GuideCount-1)*0.5f)*Spacing);
			for (int P=0; P<=Segments; ++P) Guide.Positions.push_back(Root+Vector3(Length*P/Segments,0,0));
			Guides.push_back(std::move(Guide));
		}
		StrandSettings Settings; Settings.LinearDamping=Settings.AngularDamping=2;
		m_DemoStrands=m_World->CreateStrands(Guides,Settings);
		m_DemoStrandInstanceId=m_DemoStrands->GetInstanceId();
		for (const auto& O : m_Objects) if ((O.Id=="ground" || O.Id=="post") && O.GeometryPtr) if (!m_DemoStrands->AddCollider(O.GeometryPtr)) return false;
		StrandGroomDefinition Groom; const int RenderCurves=32; const float RenderRadius=0.008f, RenderRootOffset=0.04f, CurlHeight=0.015f;
		for (int C=0; C<RenderCurves; ++C)
		{
			GroomStrand Curve; Curve.ReferenceNormal=Vector3::UnitZ(); Curve.RadiusX=RenderRadius; Curve.RadiusY=RenderRadius*0.5f;
			const Vector3 Root=Handle->X+Vector3(0,RootHeight+RenderRootOffset,(float)C*(GuideCount-1)*Spacing/(RenderCurves-1)-(GuideCount-1)*Spacing*0.5f);
			for (int P=0; P<=Segments; ++P) { const float S=(float)P/Segments; Curve.Positions.push_back(Root+Vector3(Length*S,CurlHeight*std::sin(kPi*S),0)); }
			Groom.Strands.push_back(std::move(Curve));
		}
		GroomLod Full; for (int I=0; I<GuideCount; ++I) Full.Guides.push_back(I); Groom.Lods.push_back(Full);
		GroomLod Medium; Medium.Guides={0,2,4,6}; Medium.Distance=10; Groom.Lods.push_back(Medium);
		GroomLod Low; Low.Guides={0,4}; Low.Distance=25; Groom.Lods.push_back(Low); m_DemoStrands->ConfigureGroom(Groom);
		for (int G=0; G<GuideCount; ++G)
		{
			const auto Root=m_DemoStrands->GetRootPose(G); m_DemoStrands->AttachRoot(G,Handle,Root);
			m_StrandRootLocal.emplace_back(Handle->WorldToBodyLocal(Root.pos),(Handle->Q.Conjugate()*Root.quat).Unit());
			RopeSurfaceState Surface; Surface.Strands=m_DemoStrands; Surface.GuideIndex=G; Surface.Radius=(float)RadiusX; Surface.RadiusY=(float)RadiusY;
			Surface.StrandInstanceId=m_DemoStrandInstanceId;
			Surface.Surface.reset(GeometryFactory::CreateTriangleMesh()); Surface.Surface->SetSimulationEnabled(false); Surface.Surface->SetQueryEnabled(false);
			SceneObjectInstance O; O.Id="cosserat_guide_"+std::to_string(G); O.GeometryPtr=Surface.Surface.get(); O.Color=Vector4(0.4f,0.75f,0.9f,1);
			m_Objects.push_back(O); m_RopeSurfaces.push_back(std::move(Surface));
		}
		for (int C=0; C<RenderCurves; ++C)
		{
			m_DemoStrands->AttachGroomRoot(C,Handle,Groom.Strands[C].Positions[0]);
			RopeSurfaceState Surface; Surface.Strands=m_DemoStrands; Surface.StrandInstanceId=m_DemoStrandInstanceId; Surface.GroomRender=true; Surface.GroomIndex=C;
			Surface.Radius=Groom.Strands[C].RadiusX; Surface.RadiusY=Groom.Strands[C].RadiusY;
			Surface.Surface.reset(GeometryFactory::CreateTriangleMesh()); Surface.Surface->SetSimulationEnabled(false); Surface.Surface->SetQueryEnabled(false);
			SceneObjectInstance O; O.Id="groom_strand_"+std::to_string(C); O.GeometryPtr=Surface.Surface.get(); O.Color=Vector4(0.85f,0.65f,0.3f,1);
			m_Objects.push_back(O); m_RopeSurfaces.push_back(std::move(Surface));
		}
		m_DemoStrands->RefreshGroom();
		SyncRopeSurfaces(); return true;
	}
	bool SceneWorld::ApplyStrandCollisionDemo(bool Enabled,bool Self,bool Continuous,bool Friction)
	{
		auto* S=GetDemoStrands(); if (!S) return false; auto Collision=S->GetSettings().Collision;
		Collision.Enabled=Enabled; Collision.SelfCollision=Collision.InterGuideCollision=Self; Collision.Continuous=Continuous;
		const double StaticFriction=0.5, DynamicFriction=0.3; Collision.Material.StaticFriction=Friction ? StaticFriction : 0; Collision.Material.DynamicFriction=Friction ? DynamicFriction : 0;
		S->SetCollisionSettings(Collision); return true;
	}
	size_t SceneWorld::GetStrandDemoLod() const { const auto* S=GetDemoStrands(); return S ? S->GetGroomLod() : 0; }
	bool SceneWorld::ApplyStrandDemo(float RootTwist, bool Coupled, size_t Lod)
	{
		auto* Strands=GetDemoStrands(); if (!Strands || !std::isfinite(RootTwist)) return false;
		if (!Strands->GetGroom() || Lod>=Strands->GetGroom()->GetLodCount()) return false;
		RigidBody* Handle=nullptr;
		for (const auto& O : m_Objects) if (O.Id=="handle" && O.GeometryPtr) Handle=O.GeometryPtr->GetParent<RigidBody>();
		if (!Handle) return false;
		Strands->SetGroomLod(Lod);
		Strands->SetSolver(Coupled ? StrandSolver::CoupledBanded : StrandSolver::SequentialBlocks);
		const auto Twist=StrandElasticity::Increment(StrandVector(0,0,RootTwist));
		for (size_t G=0; G<Strands->GetGuideCount(); ++G)
		{
			const auto& Local=m_StrandRootLocal[G]; Strands->AttachRoot(G,Handle,Transform(Handle->BodyLocalToWorld(Local.pos),(Handle->Q*Local.quat*Twist).Unit()));
		}
		m_StrandDemoTwist=RootTwist; return true;
	}
	void SceneWorld::SyncRopeSurfaces()
	{
		bool changed = false;
		for (RopeSurfaceState& surface : m_RopeSurfaces)
		{
			std::vector<Vector3> Positions;
			std::vector<Quaternion> Directors;
			if (surface.SpringBodyA && surface.SpringBodyB)
			{
				// The coil visualizes a joint spring; the joint alone supplies its forces.
				const Vector3 Start = surface.SpringBodyA->BodyLocalToWorld(surface.SpringAnchorA);
				const Vector3 End = surface.SpringBodyB->BodyLocalToWorld(surface.SpringAnchorB);
				const Vector3 Delta = End - Start;
				const Vector3 Axis = SafeUnitVector(Delta, Vector3::UnitY());
				Vector3 Normal, Binormal; Axis.GetPerpVectors(Normal, Binormal);
				const int Segments = surface.SpringTurns * 24;
				for (int I = 0; I <= Segments; ++I)
				{
					const float T = (float)I / Segments;
					const float Angle = T * surface.SpringTurns * 2 * PI;
					const float Radius = surface.SpringRadius * std::min(1.0f, std::min(T, 1 - T) * Segments / 6);
					Positions.push_back(Start + Delta * T + (Normal * std::cos(Angle) + Binormal * std::sin(Angle)) * Radius);
				}
			}
			else if (surface.Rope)
				for (int I = 0; I < surface.Rope->GetParticleCount(); ++I) Positions.push_back(surface.Rope->GetParticle(I).Position);
			else if (surface.Strands)
			{
				bool Exists=false;
				for (size_t I=0; I<m_World->GetStrandSystemCount(); ++I)
					if (m_World->GetStrandSystem(I)==surface.Strands) Exists=surface.Strands->GetInstanceId()==surface.StrandInstanceId;
				const bool Valid=Exists && (!surface.GroomRender || (surface.Strands->GetGroom() && surface.GroomIndex<surface.Strands->GetGroom()->GetStrandCount()));
				if (Valid && surface.GroomRender)
				{
					const auto& Curve=surface.Strands->GetGroomStrand(surface.GroomIndex); Positions=Curve.Positions; Directors=Curve.Directors;
				}
				else if (Valid)
				{
					for (size_t I=0; I<surface.Strands->GetParticleCount(surface.GuideIndex); ++I) Positions.push_back(surface.Strands->GetParticle(surface.GuideIndex,I).Position);
					for (size_t I=0; I<surface.Strands->GetDirectorCount(surface.GuideIndex); ++I) Directors.push_back(surface.Strands->GetDirector(surface.GuideIndex,I).Orientation);
				}
				else
				{
					if (!Exists) surface.Strands=nullptr;
					surface.LastPositions.clear(); surface.LastDirectors.clear(); auto* Mesh=surface.Surface->GetShapeObj<TriangleMesh>();
					if (Mesh->NumVertices) { Mesh->Clear(); changed=true; }
				}
			}
			else if (surface.TendonOwner)
			{
				bool Exists = false;
				for (size_t I = 0; I < m_World->GetReducedRagdollCount(); ++I) Exists |= m_World->GetReducedRagdoll(I) == surface.TendonOwner;
				if (Exists && surface.TendonIndex < surface.TendonOwner->GetMechanismCount())
				{
					const auto& Model = surface.TendonOwner->GetDynamics();
					for (const auto& P : surface.TendonOwner->GetMechanism(surface.TendonIndex)->GetSettings().Path)
						Positions.push_back(P.Link < 0 ? P.Point : Model.GetLinkTransform(P.Link).LocalToWorld(P.Point));
				}
				if (Positions.empty())
				{
					surface.TendonOwner = nullptr; surface.LastPositions.clear();
					surface.Surface->GetShapeObj<TriangleMesh>()->Clear(); changed = true;
				}
			}
			const int count = (int)Positions.size(), sides = surface.Sides;
			if (count < 2) continue;
			const bool moved = surface.LastPositions != Positions || surface.LastDirectors != Directors;
			if (!moved) continue;
			surface.LastPositions = std::move(Positions);
			surface.LastDirectors = std::move(Directors);
			TriangleMesh* mesh = surface.Surface->GetShapeObj<TriangleMesh>();
			const uint32_t vertexCount = (uint32_t)(count * sides + 2 * (sides + 1));
			const uint32_t triangleCount = (uint32_t)(2 * count * sides);
			if (mesh->NumVertices != vertexCount)
			{
				mesh->Clear();
				mesh->NumVertices = vertexCount;
				mesh->NumTriangles = triangleCount;
				mesh->mVertices.resize(vertexCount);
				mesh->mNormals.resize(vertexCount);
				mesh->mIndices.resize(size_t(triangleCount) * 6);
				mesh->Vertices = mesh->mVertices.data();
				mesh->Indices = mesh->mIndices.data();
				uint32_t* indices = mesh->GetIndices32();
				for (int i = 0; i < count - 1; ++i)
				for (int side = 0; side < sides; ++side)
				{
					const uint32_t a = i * sides + side, b = i * sides + (side + 1) % sides;
					*indices++ = a; *indices++ = b; *indices++ = a + sides;
					*indices++ = b; *indices++ = b + sides; *indices++ = a + sides;
				}
				for (int end = 0; end < 2; ++end)
				for (int side = 0; side < sides; ++side)
				{
					const uint32_t center = count * sides + end * (sides + 1);
					const uint32_t a = center + 1 + side, b = center + 1 + (side + 1) % sides;
					*indices++ = center; *indices++ = end ? a : b; *indices++ = end ? b : a;
				}
			}
			Vector3 tangent = Vector3::UnitY(), normal = surface.FirstNormal;
			for (int i = 0; i < count; ++i)
			{
				const Vector3 delta = surface.LastPositions[std::min(i + 1, count - 1)]
					- surface.LastPositions[std::max(i - 1, 0)];
				if (delta.SquareLength() > 1.0e-12f) tangent = delta.SafeUnit();
				// Material frames retain torsion; ropes use parallel transport instead.
				if (!surface.LastDirectors.empty()) normal=surface.LastDirectors[std::min((size_t)i,surface.LastDirectors.size()-1)]*Vector3::UnitX();
				normal -= tangent * normal.Dot(tangent);
				Vector3 binormal;
				if (normal.SquareLength() < 1.0e-8f) tangent.GetPerpVectors(normal, binormal);
				normal = normal.SafeUnit();
				binormal = tangent.Cross(normal).SafeUnit();
				if (i == 0) surface.FirstNormal = normal;
				for (int side = 0; side < sides; ++side)
				{
					const float angle = 2.0f * 3.14159265358979323846f * side / sides;

					const float RadiusY=surface.RadiusY>0 ? surface.RadiusY : surface.Radius;
					const int index = i * sides + side;
					mesh->mVertices[index] = surface.LastPositions[i] + normal*(std::cos(angle)*surface.Radius)+binormal*(std::sin(angle)*RadiusY);
					mesh->mNormals[index] = (normal*(std::cos(angle)/surface.Radius)+binormal*(std::sin(angle)/RadiusY)).SafeUnit();
					if (i == 0 || i == count - 1)
					{
						const int cap = count * sides + (i == 0 ? 0 : sides + 1);
						mesh->mVertices[cap + 1 + side] = mesh->mVertices[index];
						mesh->mNormals[cap + 1 + side] = i == 0 ? -tangent : tangent;
					}
				}
				if (i == 0 || i == count - 1)
				{
					const int cap = count * sides + (i == 0 ? 0 : sides + 1);
					mesh->mVertices[cap] = surface.LastPositions[i];
					mesh->mNormals[cap] = i == 0 ? -tangent : tangent;
				}
			}
			mesh->BoundingVolume = Box3(mesh->mVertices.data(), mesh->mVertices.size());
			surface.Surface->SetShapeBounds(mesh->BoundingVolume);
			changed = true;
		}
		if (changed) TouchRenderRevision();
	}

	bool SceneWorld::ApplyPhysicsFieldsDemo(bool Enabled, bool Frozen, bool StrainPulse, bool VelocityPulse)
	{
		if (!m_World || (VelocityPulse && m_World->GetReducedRagdollCount() > 0)) return false;
		if (Enabled && m_DemoFieldIds.empty())
		{
			// Derive the field region from existing actors; no resource or scene file.
			Box3 Bounds = Box3::Empty();
			for (const auto& Set : m_DestructionSets) Bounds.Encapsulate(Set->GetBounds());
			for (const auto& O : m_Objects) if (O.GeometryPtr)
				if (auto* B = O.GeometryPtr->GetParent<RigidBody>()) if (B->CastDynamic()) Bounds.Encapsulate(B->X);
			const float MinimumRadius = 1, RadiusScale = 1.2f, InnerFraction = 0.1f, BoxInnerFraction = 0.3f;
			const bool HasBounds = Bounds.Min.x <= Bounds.Max.x;
			const float Radius = !HasBounds ? MinimumRadius : std::max(MinimumRadius,Bounds.GetExtent().Length()*RadiusScale);
			m_DemoFieldCenter = !HasBounds ? Vector3::Zero() : Bounds.GetCenter();
			m_DemoFieldMask = ScalarField::Multiply(ScalarField::Radial(m_DemoFieldCenter,Radius*InnerFraction,Radius),
				ScalarField::Box(Transform(m_DemoFieldCenter),Vector3(Radius*BoxInnerFraction),Vector3(Radius)));
			const double Lift = 0.8, Swirl = 2, Modulation = 0.4, FrequencyHz = 0.35;
			PhysicsFieldCommand C; C.Persistent = true; C.Target = PhysicsFieldTarget::Acceleration; C.Mask = m_DemoFieldMask;
			const auto Direction = VectorField::Add(VectorField::Constant(Vector3(0,(float)Lift,0)),
				VectorField::Scale(VectorField::Cross(VectorField::Constant(Vector3::UnitY()),VectorField::Radial(m_DemoFieldCenter)),ScalarField::Constant(Swirl)));
			C.Vector = VectorField::Scale(Direction,ScalarField::Wave(1,Modulation,FrequencyHz)); m_DemoFieldIds.push_back(m_World->AddPhysicsField(C));
			const double MinimumFriction = 0.2, FrictionVariation = 0.6, DynamicRatio = 0.5, MaximumRestitution = 0.25;
			C.Target = PhysicsFieldTarget::Material;
			C.StaticFriction = ScalarField::Add(ScalarField::Constant(MinimumFriction),ScalarField::Multiply(m_DemoFieldMask,ScalarField::Constant(FrictionVariation)));
			C.DynamicFriction = ScalarField::Multiply(C.StaticFriction,ScalarField::Constant(DynamicRatio));
			C.Restitution = ScalarField::Multiply(m_DemoFieldMask,ScalarField::Constant(MaximumRestitution)); m_DemoFieldIds.push_back(m_World->AddPhysicsField(C));
		}
		if (m_PhysicsFieldFreeze != (Enabled && Frozen))
		{
			PhysicsFieldCommand C; C.Target = PhysicsFieldTarget::Activation; C.Mask = m_DemoFieldMask;
			if (Enabled && Frozen)
			{
				C.Activation = PhysicsFieldActivation::Freeze; C.Persistent = true; m_DemoFreezeField = m_World->AddPhysicsField(C);
			}
			else
			{
				m_World->RemovePhysicsField(m_DemoFreezeField); m_DemoFreezeField = 0;
				C.Activation = PhysicsFieldActivation::Unfreeze; m_World->AddPhysicsField(C);
			}
		}
		if (!Enabled) { for (auto Id : m_DemoFieldIds) m_World->RemovePhysicsField(Id); m_DemoFieldIds.clear(); }
		if (Enabled && StrainPulse)
		{
			double Capacity = 0;
			for (const auto& Set : m_DestructionSets) for (const auto& Row : Set->GetGraph().GetBondLists())
				for (const auto& Bond : Row.Bonds) Capacity = std::max(Capacity,(double)Bond.strength);
			constexpr double StrainCapacityFactor = 2;
			PhysicsFieldCommand C; C.Target = PhysicsFieldTarget::Strain; C.Mask = m_DemoFieldMask; C.Scalar = ScalarField::Constant(Capacity*StrainCapacityFactor); m_World->AddPhysicsField(C);
		}
		if (Enabled && VelocityPulse)
		{
			constexpr double PulseSpeed = 3;
			PhysicsFieldCommand C; C.Target = PhysicsFieldTarget::LinearVelocity; C.Mask = m_DemoFieldMask;
			C.Vector = VectorField::Scale(VectorField::Radial(m_DemoFieldCenter),ScalarField::Constant(PulseSpeed)); m_World->AddPhysicsField(C);
		}
		m_PhysicsFieldsDemo = Enabled; m_PhysicsFieldFreeze = Enabled && Frozen; return true;
	}

	// Cache demo reuses the authored scene: record live interaction, then replace
	// only destruction actors with evaluated kinematic compounds. No new assets.
	DestructionPlayback* SceneWorld::GetDestructionPlayback(size_t Index) const
	{ return Index < m_DestructionPlaybacks.size() ? m_DestructionPlaybacks[Index].get() : nullptr; }
	bool SceneWorld::IsDestructionCachePlaying() const
	{ return !m_DestructionPlaybacks.empty() && m_DestructionPlaybacks.front()->IsPlaying(); }
	double SceneWorld::GetDestructionCacheTime() const
	{ return m_DestructionPlaybacks.empty() ? m_DestructionRecordingTime : m_DestructionPlaybacks.front()->GetTime(); }
	double SceneWorld::GetDestructionCacheDuration() const
	{ return m_DestructionCaches.empty() ? m_DestructionRecordingTime : m_DestructionCaches.front().GetStats().Duration; }
	bool SceneWorld::ApplyDestructionCacheDemo(DestructionPlaybackCommand Command, float SeekFraction)
	{
		if (Command == DestructionPlaybackCommand::None) return true;
		if (!m_World || !m_DestructionCacheDemo || !std::isfinite(SeekFraction) || SeekFraction < 0 || SeekFraction > 1) return false;
		if (Command == DestructionPlaybackCommand::Replay)
		{
			if (!m_DestructionRecorders.empty())
			{
				if (!(m_DestructionRecordingTime > 0)) return false;
				for (const auto& Recorder : m_DestructionRecorders) m_DestructionCaches.push_back(Recorder->Finish());
				m_DestructionRecorders.clear();
			}
			if (m_DestructionCaches.empty()) return false;
			std::vector<std::unique_ptr<DestructionPlayback>> Players;
			if (m_DestructionPlaybacks.empty())
			{
				for (size_t I = 0; I < m_DestructionCaches.size(); ++I)
					Players.emplace_back(new DestructionPlayback(*m_World,m_DestructionCaches[I],m_DestructionSets[I]->GetAssetPlacement()));
				m_Objects.erase(std::remove_if(m_Objects.begin(),m_Objects.end(),[](const SceneObjectInstance& O)
					{ return O.DestructSet != nullptr; }),m_Objects.end());
				for (const auto& Set : m_DestructionSets) Set->Clear();
				m_DestructionPlaybacks.swap(Players);
			}
			for (const auto& Player : m_DestructionPlaybacks) { Player->Seek(0); Player->SetPlaying(true); }
			m_DestructionPlaybackEvents.clear(); SyncDestructionObjects(true); return true;
		}
		if (m_DestructionPlaybacks.empty()) return false;
		if (Command == DestructionPlaybackCommand::TogglePause)
		{
			const bool Play = !IsDestructionCachePlaying();
			for (const auto& Player : m_DestructionPlaybacks) Player->SetPlaying(Play);
			return true;
		}
		if (Command == DestructionPlaybackCommand::Seek)
		{
			for (const auto& Player : m_DestructionPlaybacks) Player->Seek(SeekFraction*Player->GetCache().GetStats().Duration);
			m_DestructionPlaybackEvents.clear(); SyncDestructionObjects(false); return true;
		}
		return false;
	}

	void SceneWorld::Step(float dt)
	{
		if (!std::isfinite(dt) || dt <= 0.0f) return;
		m_DestructionEvents.clear(); m_DestructionPlaybackEvents.clear();
		if (m_World)
		{
			SamplePhysicalAnimationDemo(dt);
			SampleClothDemo(dt);
			if (m_World->GetJointDrivesEnabled())
			{
				m_MechanismDemoTime += dt;
				constexpr double InputAmplitude = 1.5, WinchAmplitude = 0.7, Frequency = 1.2;
				for (size_t I = 0; I < m_MechanismDemoDrivers.size(); ++I)
				{
					auto* Owner = m_MechanismDemoDrivers[I]; bool Exists = false;
					for (size_t J = 0; J < m_World->GetReducedRagdollCount(); ++J) Exists |= m_World->GetReducedRagdoll(J) == Owner;
					if (!Exists) { m_MechanismDemoDrivers[I] = nullptr; continue; }
					auto Control = Owner->GetControl(0);
					Control.Drive.TargetPosition = (float)((I == 0 ? InputAmplitude : WinchAmplitude) * std::sin(Frequency * m_MechanismDemoTime));
					Owner->SetControl(0, Control);
				}
			}
			if (m_CharacterPlatform)
			{
				m_CharacterDemoTime += dt;
				const float LiftAmplitude = 0.5f, LiftFrequency = 0.8f;
				m_CharacterPlatform->X = m_CharacterPlatformOrigin + Vector3(0, LiftAmplitude * (1 - std::cos(LiftFrequency * m_CharacterDemoTime)), 0);
				m_CharacterPlatform->Q.FromRotateY(m_CharacterDemoTime * 0.2f);
			}
			UpdateRobotDrive(dt);
			m_World->Simulate(dt);
			m_DestructionDroppedEvents = 0;
			for (const auto& Set : m_DestructionSets) if (Set->GetEventSettings().Enabled)
			{
				auto Events = Set->TakeEvents(); m_DestructionEventCount += Events.size();
				m_DestructionDroppedEvents += Set->GetEventStats().Dropped;
				for (auto& Event : Events) m_DestructionEvents.push_back(std::move(Event));
			}
			if (!m_DestructionRecorders.empty())
			{
				m_DestructionRecordingTime += dt;
				for (size_t I = 0; I < m_DestructionRecorders.size(); ++I)
				{
					std::vector<DestructionEvent> Batch;
					for (const auto& E : m_DestructionEvents) if (E.InstanceId == m_DestructionSets[I]->GetInstanceId()) Batch.push_back(E);
					m_DestructionRecorders[I]->Record(m_DestructionRecordingTime,*m_DestructionSets[I],Batch);
				}
			}
			for (const auto& Playback : m_DestructionPlaybacks)
				for (auto& Event : Playback->TakeEvents()) m_DestructionPlaybackEvents.push_back(std::move(Event));
		}
		SyncDestructionObjects(false);
		SyncTerrainRevisions();
		SyncRopeSurfaces();
		SyncVehicles(dt);
		SyncWalkingRobot(dt);
		SyncPhysicalAnimationDemo();
	}

	int SceneWorld::GetAerodynamicsRigidBodyCount() const
	{
		if ((m_FieldSimulation.Type != "aerodynamics3d" || m_FieldSimulation.Aerodynamics.Algorithm != AerodynamicsAlgorithm::Incompressible)) return 0;
		std::vector<const RigidBody*> bodies;
		for (const SceneObjectInstance& object : m_Objects)
		{
			const RigidBody* body = object.GeometryPtr ? object.GeometryPtr->GetParent<RigidBody>() : nullptr;
			if (body && body->mRigidType == RigidType::Dynamic
				&& std::find(bodies.begin(), bodies.end(), body) == bodies.end()) bodies.push_back(body);
		}
		return (int)bodies.size();
	}

	Geometry* SceneWorld::AddProjectileSphere(const std::string& id, const Vector3& position, float radius, const Vector3& velocity, const Vector4& color)
	{
		SceneObjectDesc desc;
		desc.Id = id;
		desc.Type = "sphere";
		desc.BodyType = RigidType::Dynamic;
		desc.Position = position;
		desc.Radius = radius;
		desc.Color = color;
		desc.DisableGravity = true;

		std::string error;
		Geometry* geometry = CreateGeometry(desc, &error);
		if (geometry == nullptr)
		{
			return nullptr;
		}

		RigidBodyParam bodyParam;
		bodyParam.rigidType = RigidType::Dynamic;
		bodyParam.disableGravity = true;
		bodyParam.linearVelocity = velocity;
		bodyParam.motionType = MorionType::Discrete;
		RigidBody* body = m_World->CreateRigidBody(geometry, bodyParam);
		RigidBodyDynamic* dynamicBody = body ? body->CastDynamic() : nullptr;
		if (dynamicBody)
		{
			dynamicBody->DisableGravity = true;
		}

		SceneObjectInstance instance;
		instance.Id = id;
		instance.GeometryPtr = geometry;
		instance.Color = color;
		instance.RenderBounds = false;
		m_Objects.push_back(instance);
		TouchRenderRevision();
		return geometry;
	}

	Geometry* SceneWorld::AddTriangleMeshObject(const std::string& id, const StaticMesh& mesh, const Transform& transform, RigidType bodyType, const Vector4& color, bool renderBounds)
	{
		if (m_World == nullptr || mesh.GetVertexCount() == 0 || mesh.GetTriangleCount() == 0)
		{
			return nullptr;
		}

		Geometry* geometry = GeometryFactory::CreateTriangleMesh();
		TriangleMesh* triangleMesh = geometry ? geometry->GetShapeObj<TriangleMesh>() : nullptr;
		if (triangleMesh == nullptr)
		{
			delete geometry;
			return nullptr;
		}

		triangleMesh->Release();
		triangleMesh->SetData(
			const_cast<Vector3*>(mesh.mVertices.data()),
			const_cast<uint16_t*>(mesh.mIndices.data()),
			mesh.GetVertexCount(),
			mesh.GetTriangleCount(),
			mesh.Is16bitIndices(),
			true);
		triangleMesh->CalculateBoundingBox();
		triangleMesh->CalculateWeightAverageNormals();
		triangleMesh->BuildBVH();

		geometry->UpdateVolumeProperties();
		geometry->SetTransform(transform.pos, transform.quat);
		geometry->SetQueryEnabled(true);
		geometry->SetSimulationEnabled(bodyType != RigidType::Static);

		RigidBodyParam bodyParam;
		bodyParam.rigidType = bodyType;
		RigidBody* body = m_World->CreateRigidBody(geometry, bodyParam);
		if (body == nullptr)
		{
			delete geometry;
			return nullptr;
		}

		SceneObjectInstance instance;
		instance.Id = id;
		instance.GeometryPtr = geometry;
		instance.Color = color;
		instance.RenderBounds = renderBounds;
		m_Objects.push_back(instance);
		TouchRenderRevision();
		return geometry;
	}
}
