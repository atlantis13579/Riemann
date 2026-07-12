#include "RagdollDefinition.h"

#include <cctype>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <map>
#include <sstream>

namespace Riemann
{
	namespace
	{
		constexpr float DegreesToRadians = 0.01745329251994329577f;

		struct JsonValue
		{
			enum class Type { Null, Number, String, Array, Object } ValueType = Type::Null;
			double Number = 0.0;
			std::string String;
			std::vector<JsonValue> Array;
			std::map<std::string, JsonValue> Object;

			const JsonValue* Find(const char* Name) const
			{
				const auto It = Object.find(Name ? Name : "");
				return It == Object.end() ? nullptr : &It->second;
			}
		};

		class JsonParser
		{
		public:
			explicit JsonParser(const std::string& Text) : m_Text(Text), m_Position(0) {}

			bool Parse(JsonValue* Value, std::string* Error)
			{
				SkipWhitespace();
				if (!ParseValue(Value)) return Fail(Error);
				SkipWhitespace();
				if (m_Position != m_Text.size())
				{
					m_Error = "unexpected trailing JSON data";
					return Fail(Error);
				}
				return true;
			}

		private:
			void SkipWhitespace()
			{
				while (m_Position < m_Text.size()
					&& std::isspace((unsigned char)m_Text[m_Position])) ++m_Position;
			}

			bool ParseValue(JsonValue* Value)
			{
				SkipWhitespace();
				if (m_Position >= m_Text.size()) return SetError("unexpected end of JSON");
				const char Token = m_Text[m_Position];
				if (Token == '{') return ParseObject(Value);
				if (Token == '[') return ParseArray(Value);
				if (Token == '"')
				{
					Value->ValueType = JsonValue::Type::String;
					return ParseString(&Value->String);
				}
				if (Token == '-' || std::isdigit((unsigned char)Token)) return ParseNumber(Value);
				if (m_Text.compare(m_Position, 4, "null") == 0)
				{
					m_Position += 4;
					return true;
				}
				return SetError("unsupported JSON value");
			}

			bool ParseObject(JsonValue* Value)
			{
				Value->ValueType = JsonValue::Type::Object;
				++m_Position;
				SkipWhitespace();
				if (Consume('}')) return true;
				for (;;)
				{
					std::string Name;
					if (!ParseString(&Name)) return false;
					SkipWhitespace();
					if (!Consume(':')) return SetError("expected ':'");
					JsonValue Child;
					if (!ParseValue(&Child)) return false;
					Value->Object[Name] = Child;
					SkipWhitespace();
					if (Consume('}')) return true;
					if (!Consume(',')) return SetError("expected ',' or '}'");
					SkipWhitespace();
				}
			}

			bool ParseArray(JsonValue* Value)
			{
				Value->ValueType = JsonValue::Type::Array;
				++m_Position;
				SkipWhitespace();
				if (Consume(']')) return true;
				for (;;)
				{
					JsonValue Child;
					if (!ParseValue(&Child)) return false;
					Value->Array.push_back(Child);
					SkipWhitespace();
					if (Consume(']')) return true;
					if (!Consume(',')) return SetError("expected ',' or ']'");
					SkipWhitespace();
				}
			}

			bool ParseString(std::string* Value)
			{
				if (!Consume('"')) return SetError("expected string");
				Value->clear();
				while (m_Position < m_Text.size())
				{
					const char Character = m_Text[m_Position++];
					if (Character == '"') return true;
					if (Character == '\\')
					{
						if (m_Position >= m_Text.size()) return SetError("invalid string escape");
						const char Escape = m_Text[m_Position++];
						if (Escape == 'n') Value->push_back('\n');
						else if (Escape == 'r') Value->push_back('\r');
						else if (Escape == 't') Value->push_back('\t');
						else if (Escape == '"' || Escape == '\\' || Escape == '/') Value->push_back(Escape);
						else return SetError("unsupported string escape");
					}
					else Value->push_back(Character);
				}
				return SetError("unterminated string");
			}

			bool ParseNumber(JsonValue* Value)
			{
				const char* Begin = m_Text.c_str() + m_Position;
				char* End = nullptr;
				const double Number = std::strtod(Begin, &End);
				if (End == Begin || !std::isfinite(Number)) return SetError("invalid number");
				m_Position += (size_t)(End - Begin);
				Value->ValueType = JsonValue::Type::Number;
				Value->Number = Number;
				return true;
			}

			bool Consume(char Character)
			{
				if (m_Position >= m_Text.size() || m_Text[m_Position] != Character) return false;
				++m_Position;
				return true;
			}

			bool SetError(const char* Message)
			{
				std::ostringstream Stream;
				Stream << Message << " at byte " << m_Position;
				m_Error = Stream.str();
				return false;
			}

			bool Fail(std::string* Error) const
			{
				if (Error) *Error = m_Error;
				return false;
			}

			const std::string& m_Text;
			size_t m_Position;
			std::string m_Error;
		};

		bool ReadVector3(const JsonValue* Value, Vector3* Result)
		{
			if (Value == nullptr || Result == nullptr
				|| Value->ValueType != JsonValue::Type::Array || Value->Array.size() != 3) return false;
			for (int Axis = 0; Axis < 3; ++Axis)
				if (Value->Array[(size_t)Axis].ValueType != JsonValue::Type::Number) return false;
			*Result = Vector3((float)Value->Array[0].Number, (float)Value->Array[1].Number,
				(float)Value->Array[2].Number);
			return true;
		}

		bool ReadString(const JsonValue* Value, std::string* Result)
		{
			if (Value == nullptr || Result == nullptr
				|| Value->ValueType != JsonValue::Type::String) return false;
			*Result = Value->String;
			return true;
		}

		bool ReadFloat(const JsonValue* Value, float* Result)
		{
			if (Value == nullptr || Result == nullptr
				|| Value->ValueType != JsonValue::Type::Number) return false;
			*Result = (float)Value->Number;
			return std::isfinite(*Result);
		}

		bool SetLoadError(std::string* Error, const std::string& Message)
		{
			if (Error) *Error = Message;
			return false;
		}
	}

	bool LoadRagdollDefinition(const char* FileName, RagdollDefinition* Definition,
		std::string* ErrorMessage)
	{
		if (Definition == nullptr || FileName == nullptr || FileName[0] == '\0')
			return SetLoadError(ErrorMessage, "missing ragdoll definition path");
		std::ifstream File(FileName, std::ios::binary);
		if (!File) return SetLoadError(ErrorMessage, "cannot open ragdoll definition");
		std::ostringstream Buffer;
		Buffer << File.rdbuf();
		const std::string Text = Buffer.str();
		JsonValue Root;
		std::string ParseError;
		JsonParser Parser(Text);
		if (!Parser.Parse(&Root, &ParseError)) return SetLoadError(ErrorMessage, ParseError);
		if (Root.ValueType != JsonValue::Type::Object)
			return SetLoadError(ErrorMessage, "ragdoll definition root must be an object");

		const JsonValue* Bones = Root.Find("bones");
		const JsonValue* Joints = Root.Find("joints");
		if (Bones == nullptr || Bones->ValueType != JsonValue::Type::Array || Bones->Array.empty())
			return SetLoadError(ErrorMessage, "ragdoll definition has no bones");
		if (Joints == nullptr || Joints->ValueType != JsonValue::Type::Array)
			return SetLoadError(ErrorMessage, "ragdoll definition has no joints");

		RagdollDefinition Loaded;
		for (const JsonValue& Value : Bones->Array)
		{
			RagdollBoneDefinition Bone;
			std::string Shape;
			if (Value.ValueType != JsonValue::Type::Object
				|| !ReadString(Value.Find("name"), &Bone.Name)
				|| !ReadString(Value.Find("shape"), &Shape))
				return SetLoadError(ErrorMessage, "invalid ragdoll bone");
			ReadVector3(Value.Find("jointVelocityAxis"), &Bone.JointVelocityAxis);
			if (Shape == "box")
			{
				Bone.Shape = RagdollBoneShape::Box;
				if (!ReadVector3(Value.Find("center"), &Bone.Center)
					|| !ReadVector3(Value.Find("halfExtent"), &Bone.HalfExtent))
					return SetLoadError(ErrorMessage, "invalid box bone " + Bone.Name);
			}
			else if (Shape == "capsule")
			{
				Bone.Shape = RagdollBoneShape::Capsule;
				if (!ReadVector3(Value.Find("start"), &Bone.Start)
					|| !ReadVector3(Value.Find("end"), &Bone.End)
					|| !ReadFloat(Value.Find("radius"), &Bone.Radius))
					return SetLoadError(ErrorMessage, "invalid capsule bone " + Bone.Name);
			}
			else if (Shape == "sphere")
			{
				Bone.Shape = RagdollBoneShape::Sphere;
				if (!ReadVector3(Value.Find("center"), &Bone.Center)
					|| !ReadFloat(Value.Find("radius"), &Bone.Radius))
					return SetLoadError(ErrorMessage, "invalid sphere bone " + Bone.Name);
			}
			else return SetLoadError(ErrorMessage, "unknown shape for bone " + Bone.Name);
			Loaded.Bones.push_back(Bone);
		}

		for (const JsonValue& Value : Joints->Array)
		{
			RagdollJointDefinition Joint;
			std::string Type;
			if (Value.ValueType != JsonValue::Type::Object
				|| !ReadString(Value.Find("type"), &Type)
				|| !ReadString(Value.Find("bodyA"), &Joint.BodyA)
				|| !ReadString(Value.Find("bodyB"), &Joint.BodyB)
				|| !ReadVector3(Value.Find("anchor"), &Joint.Anchor))
				return SetLoadError(ErrorMessage, "invalid ragdoll joint");
			if (Type == "cone")
			{
				Joint.Type = RagdollJointType::Cone;
				float LimitDegrees = 0.0f;
				if (!ReadVector3(Value.Find("axisA"), &Joint.AxisA)
					|| !ReadVector3(Value.Find("axisB"), &Joint.AxisB)
					|| !ReadFloat(Value.Find("limitDegrees"), &LimitDegrees))
					return SetLoadError(ErrorMessage, "invalid cone joint");
				Joint.ConeLimit = LimitDegrees * DegreesToRadians;
			}
			else if (Type == "hinge")
			{
				Joint.Type = RagdollJointType::Hinge;
				float MinDegrees = 0.0f, MaxDegrees = 0.0f;
				if (!ReadVector3(Value.Find("axis"), &Joint.AxisA)
					|| !ReadFloat(Value.Find("minDegrees"), &MinDegrees)
					|| !ReadFloat(Value.Find("maxDegrees"), &MaxDegrees))
					return SetLoadError(ErrorMessage, "invalid hinge joint");
				Joint.MinAngle = MinDegrees * DegreesToRadians;
				Joint.MaxAngle = MaxDegrees * DegreesToRadians;
			}
			else return SetLoadError(ErrorMessage, "unknown ragdoll joint type");
			Loaded.Joints.push_back(Joint);
		}

		*Definition = Loaded;
		if (ErrorMessage) ErrorMessage->clear();
		return true;
	}
}
