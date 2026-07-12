#include "SceneWorld.h"

#include <ctype.h>
#include <float.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include <algorithm>
#include <fstream>
#include <map>
#include <sstream>

#include "../Src/Collision/GeometryObject.h"
#include "../Src/Collision/GeometryQuery.h"
#include "../Src/CollisionPrimitive/StaticMesh.h"
#include "../Src/CollisionPrimitive/TriangleMesh.h"
#include "../Src/Destruction/DestructionSet.h"
#include "../Src/Destruction/Fracture.h"
#include "../Src/Geometry/DynamicMesh.h"
#include "../Src/RigidBodyDynamics/PhysicsWorld.h"
#include "../Src/Deformation/Rope.h"
#include "../Src/Deformation/Cloth.h"
#include "../Src/RigidBodyDynamics/Ragdoll.h"

namespace Riemann
{
	namespace
	{
		const float kPi = 3.14159265358979323846f;

		std::string DirectoryOf(const std::string& fileName)
		{
			const size_t pos = fileName.find_last_of("/\\");
			if (pos == std::string::npos)
			{
				return std::string();
			}
			return fileName.substr(0, pos + 1);
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
		int SphereCount = 24;
		int StepCount = 10;
		float StepWidth = 1.2f;
		float StepHeight = 0.5f;
		float StepDepth = 6.0f;
		float InvMass = 1.0f;
		float Restitution = 0.0f;
		bool HasInvMass = false;
		bool HasRestitution = false;
		bool HasX0 = false;
		bool HasX1 = false;
		bool DisableGravity = false;
		bool EnableCCD = false;
		bool MouseControl = false;
		bool RenderBounds = false;
		bool RenderEnabled = true;
		bool QueryEnabled = true;
		bool SimulationEnabled = true;
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

		bool ReadBool(const JsonValue* value, bool fallback)
		{
			return value && value->IsBool() ? value->boolValue : fallback;
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
#if defined(RIEMANN_ENABLE_CUDA)
			if (backend == "cuda")
			{
				return SolverBackend::CUDA;
			}
#endif
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
			desc.LinearVelocity = ReadVector3(object.Find("linearVelocity"), desc.LinearVelocity);
			desc.AngularVelocity = ReadVector3(object.Find("angularVelocity"), desc.AngularVelocity);
			desc.DisableGravity = ReadBool(object.Find("disableGravity"), desc.DisableGravity);
			desc.EnableCCD = ReadBool(object.Find("enableCCD"), desc.EnableCCD);
			desc.MouseControl = ReadBool(object.Find("mouseControl"), desc.MouseControl);
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

	SceneWorld::SceneWorld()
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
		m_DestructionRenderStates.clear();
		m_DestructionSets.clear();
		m_Objects.clear();
		m_ConstraintInstances.clear();
		m_MouseControlGeometries.clear();
		m_MouseControlGeometry = nullptr;
		m_MouseControlBody = nullptr;
		m_MouseControlRope = nullptr;
		m_MouseControlRopeParticle = -1;
		TouchRenderRevision();

		PhysicsWorldParam worldParam = param;
		worldParam.sceneQueryType = SceneQueryType::DynamicAABB;
		worldParam.broadphase = BroadPhaseSolver::SAP;
		m_World.reset(new PhysicsWorld(worldParam));

		m_Camera = CameraDesc();
		m_Light = DirectionalLightDesc();
		m_FieldSimulation = SceneFieldSimulationDesc();
		m_Name.clear();
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

	bool SceneWorld::LoadFromFile(const std::string& fileName, std::string* errorMessage)
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
		m_BaseDirectory = DirectoryOf(fileName);
		return LoadFromText(ss.str(), fileName, errorMessage);
	}

	bool SceneWorld::LoadFromText(const std::string& text, const std::string& sourceName, std::string* errorMessage)
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

		return LoadParsedScene(root, sourceName, errorMessage);
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
		worldParam.gravityAcc = Vector3(0.0f, -9.8f, 0.0f);
		if (physics && physics->IsObject())
		{
			worldParam.gravityAcc = ReadVector3(physics->Find("gravity"), worldParam.gravityAcc);
			worldParam.resolutionSolver = ReadResolutionSolver(
				physics->Find("resolutionSolver"), worldParam.resolutionSolver);
			worldParam.solverBackend = ReadSolverBackend(
				physics->Find("solverBackend"), worldParam.solverBackend);
			worldParam.contactSolverIterations = std::max(1, ReadInt(
				physics->Find("contactSolverIterations"), worldParam.contactSolverIterations));
			worldParam.contactSolverTolerance = std::max(0.0f, ReadFloat(
				physics->Find("contactSolverTolerance"), worldParam.contactSolverTolerance));
			worldParam.contactSolverRelaxation = std::max(0.01f, std::min(1.99f, ReadFloat(
				physics->Find("contactSolverRelaxation"), worldParam.contactSolverRelaxation)));
		}
		ResetWorld(worldParam);
		m_Name = ReadString(root.Find("name"), sourceName);
		m_FieldSimulation = ReadFieldSimulation(root.Find("fieldSimulation"));

		const JsonValue* camera = root.Find("camera");
		if (camera && camera->IsObject())
		{
			m_Camera.At = ReadVector3(camera->Find("center"), m_Camera.At);
			const Vector3 orbit = ReadVector3(camera->Find("orbit"), Vector3(1.0f, 0.6f, 15.0f));
			m_Camera.Eye = m_Camera.At + Vector3(sinf(orbit.x) * cosf(orbit.y), sinf(orbit.y), cosf(orbit.x) * cosf(orbit.y)) * orbit.z;
			m_Camera.FovY = ReadFloat(camera->Find("fovY"), m_Camera.FovY);
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
				ConstraintInstance constraintInstance;
				constraintInstance.Name = type + " - " + bodyBName;
				constraintInstance.Constraint = created;
				m_ConstraintInstances.push_back(constraintInstance);
				if (ReadBool(constraint.Find("disableCollision"), false))
				{
					bodyA->IgnoreCollisionWith(bodyB);
				}
			}
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
				settings.Damping = ReadFloat(ropeValue.Find("damping"), settings.Damping);
				settings.Gravity = ReadVector3(ropeValue.Find("gravity"), settings.Gravity);
				RopeSystem* rope = m_World->CreateRope(settings, start, end);
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
				const Vector3 center = ReadVector3(clothValue.Find("center"), Vector3::Zero());
				const Vector3 horizontalAxis = ReadVector3(clothValue.Find("horizontalAxis"), Vector3::UnitX());
				const Vector3 verticalAxis = ReadVector3(clothValue.Find("verticalAxis"), Vector3::UnitY());
				ClothSystem* cloth = m_World->CreateCloth(settings, center, horizontalAxis, verticalAxis);
				if (ReadBool(clothValue.Find("pinTop"), true))
				{
					for (int column = 0; column < cloth->GetColumns(); ++column)
					{
						cloth->PinParticle(0, column, cloth->GetParticle(0, column).Position);
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
				const std::string definitionFile = ReadString(
					ragdollValue.Find("definition"), std::string());
				if (!definitionFile.empty()) settings.DefinitionFile = m_BaseDirectory + definitionFile;
				settings.Scale = ReadFloat(ragdollValue.Find("scale"), settings.Scale);
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

		return true;
	}

	bool SceneWorld::CreateObject(const SceneObjectDesc& desc, std::string* errorMessage)
	{
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
		TouchRenderRevision();
		return true;
	}

	bool SceneWorld::CreateDestructionBox(const SceneObjectDesc& desc, std::string* errorMessage)
	{
		if (m_World == nullptr)
		{
			return false;
		}

		std::unique_ptr<DestructionSet> destructSet(new DestructionSet(m_World.get()));
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
			if (!BuildBoxDynamicMesh(desc.Position, desc.HalfExtent, desc.Rotation, &sourceMesh)
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
						const Vector3 worldCenter = desc.Position + desc.Rotation * localCenter;
						DynamicMesh cellMesh;
						if (!BuildBoxDynamicMesh(worldCenter, cellHalfExtent, desc.Rotation, &cellMesh))
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

		destructSet->RebuildClustersFromGraph();
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

		DynamicMesh sourceMesh;
		if (!BuildBoxDynamicMesh(desc.Position, desc.HalfExtent, desc.Rotation, &sourceMesh))
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
		const Vector3 glassNormal = desc.Rotation * Vector3::UnitZ();
		if (!Fracture::VoronoiFracture2D(sourceMesh, glassNormal, pieces,
			desc.DestructionPieceCount, desc.DestructionSeed, cutOptions) || pieces.size() < 2)
		{
			if (errorMessage)
			{
				*errorMessage = "failed to create 2D Voronoi glass pieces for object " + desc.Id;
			}
			return false;
		}

		std::unique_ptr<DestructionSet> destructSet(new DestructionSet(m_World.get()));
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

		destructSet->RebuildClustersFromGraph();
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
		m_DestructionRenderStates.push_back(std::move(renderState));
		SyncDestructionObjects(true);
		return true;
	}

	Geometry* SceneWorld::CreateGeometry(const SceneObjectDesc& desc, std::string* errorMessage) const
	{
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
		for (DestructionRenderState& state : m_DestructionRenderStates)
		{
			if (state.Set == nullptr)
			{
				continue;
			}

			const uint32_t revision = state.Set->GetClusterRevision();
			const bool renderIntact = state.RenderIntactUntilFractured
				&& state.IntactGeometry
				&& state.Set->GetGraph().GetBrokenBonds().empty();
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

	void SceneWorld::Step(float dt)
	{
		(void)dt;
		if (m_World)
		{
			m_World->Simulate();
			if (m_World->GetClothCount() > 0)
			{
				TouchRenderRevision();
			}
		}
		SyncDestructionObjects(false);
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
