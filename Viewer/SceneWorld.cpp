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
#include "../Src/CollisionPrimitive/StaticMesh.h"
#include "../Src/CollisionPrimitive/TriangleMesh.h"
#include "../Src/RigidBodyDynamics/PhysicsWorld.h"

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
		float Height = 2.0f;
		float InvMass = 1.0f;
		bool HasInvMass = false;
		bool HasX0 = false;
		bool HasX1 = false;
		bool DisableGravity = false;
		bool RenderBounds = false;
		bool QueryEnabled = true;
		bool SimulationEnabled = true;
		int RepeatCount[3] = { 1, 1, 1 };
		Vector3 RepeatSpacing = Vector3::Zero();
		std::string MeshFile;
	};

	namespace
	{
		float ReadFloat(const JsonValue* value, float fallback)
		{
			return value && value->IsNumber() ? static_cast<float>(value->numberValue) : fallback;
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
			desc.Height = ReadFloat(object.Find("height"), desc.Height);
			desc.LinearVelocity = ReadVector3(object.Find("linearVelocity"), desc.LinearVelocity);
			desc.AngularVelocity = ReadVector3(object.Find("angularVelocity"), desc.AngularVelocity);
			desc.DisableGravity = ReadBool(object.Find("disableGravity"), desc.DisableGravity);
			desc.QueryEnabled = ReadBool(object.Find("query"), desc.QueryEnabled);
			desc.SimulationEnabled = ReadBool(object.Find("simulation"), desc.SimulationEnabled);
			if (object.Find("query") == nullptr && desc.Id == "ground" && desc.Type == "plane")
			{
				desc.QueryEnabled = false;
			}
			desc.RenderBounds = ReadBool(object.Find("renderBounds"), desc.RenderBounds);
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
		m_Objects.clear();

		PhysicsWorldParam param;
		param.gravityAcc = gravity;
		param.sceneQueryType = SceneQueryType::DynamicAABB;
		m_World.reset(new PhysicsWorld(param));

		m_Camera = CameraDesc();
		m_Light = DirectionalLightDesc();
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
		const Vector3 gravity = physics && physics->IsObject()
			? ReadVector3(physics->Find("gravity"), Vector3(0.0f, -9.8f, 0.0f))
			: Vector3(0.0f, -9.8f, 0.0f);
		Reset(gravity);

		const JsonValue* camera = root.Find("camera");
		if (camera && camera->IsObject())
		{
			m_Camera.At = ReadVector3(camera->Find("center"), m_Camera.At);
			const Vector3 orbit = ReadVector3(camera->Find("orbit"), Vector3(1.0f, 0.6f, 15.0f));
			m_Camera.Eye = m_Camera.At + Vector3(sinf(orbit.x) * cosf(orbit.y), sinf(orbit.y), cosf(orbit.x) * cosf(orbit.y)) * orbit.z;
			m_Camera.FovY = ReadFloat(camera->Find("fovY"), m_Camera.FovY);
			m_Camera.NearPlane = ReadFloat(camera->Find("near"), m_Camera.NearPlane);
			m_Camera.FarPlane = ReadFloat(camera->Find("far"), m_Camera.FarPlane);
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

		return true;
	}

	bool SceneWorld::CreateObject(const SceneObjectDesc& desc, std::string* errorMessage)
	{
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

		SceneObjectInstance instance;
		instance.Id = desc.Id;
		instance.GeometryPtr = geometry;
		instance.Color = desc.Color;
		instance.RenderBounds = desc.RenderBounds;
		m_Objects.push_back(instance);
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

		if (errorMessage)
		{
			*errorMessage = "unsupported object type '" + desc.Type + "' for object " + desc.Id;
		}
		return nullptr;
	}

	void SceneWorld::Step(float dt)
	{
		(void)dt;
		if (m_World)
		{
			m_World->Simulate();
		}
	}

	Geometry* SceneWorld::AddProjectileSphere(const std::string& id, const Vector3& position, float radius, const Vector3& acceleration, const Vector4& color)
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
		RigidBody* body = m_World->CreateRigidBody(geometry, bodyParam);
		RigidBodyDynamic* dynamicBody = body ? body->CastDynamic() : nullptr;
		if (dynamicBody)
		{
			dynamicBody->DisableGravity = true;
			dynamicBody->ApplyLinearAcceleration(acceleration);
		}

		SceneObjectInstance instance;
		instance.Id = id;
		instance.GeometryPtr = geometry;
		instance.Color = color;
		instance.RenderBounds = false;
		m_Objects.push_back(instance);
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
		geometry->SetWorldTransform(transform.pos, transform.quat);
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
		return geometry;
	}
}
