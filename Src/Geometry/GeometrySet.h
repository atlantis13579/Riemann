#pragma once

#include <string>
#include <vector>
#include "../Maths/Box3.h"
#include "../Maths/Vector2.h"
#include "../Maths/Vector3.h"
#include "../Maths/Transform.h"

namespace Geometry
{
	struct VertexInfo
	{
		Vector3 Position { Vector3::Zero() };
		Vector3 Normal { Vector3::Zero() };
		Vector3 Color { Vector3::Zero() };
		Vector2 UV { Vector2::Zero() };
		bool bHaveNormal { false };
		bool bHaveColor { false };
		bool bHaveUV { false };
	};

	template<typename T>
	class AttributeBase
	{
	public:
		inline void SetNewValue(int index, const T& val)
		{
			mValuse.insert(mValuse.begin() + index, val);
		}

		inline T GetValue(int index) const
		{
			return mValuse[index];
		}

		inline void SetValue(int index, const T& val)
		{
			mValuse[index] = val;
		}

	private:
		std::vector<T>	mValuse;
	};

	struct GeometryAttributeSet
	{
		AttributeBase<int> MaterialIDAttrib;
	};

	class GeometryData
	{
	public:
		GeometryData();
		~GeometryData();

		bool LoadObj(const char* name);
		bool ExportObj(const char* name);
		static GeometryData* CreateFromObj(const char* name);

		int GetVerticesCount() const { return (int)VertexPositions.size(); }
		int AppendVertex(const VertexInfo& info);
		int AppendTriangle(const Vector3i& tri);

		void SetColor(int idx, const Vector3& val);
		void SetNormal(int idx, const Vector3& val);
		void SetUV(int idx, const Vector2& val, int layer);

		Vector2 GetUV(int idx, int layer) const
		{
			return VertexUVs[idx];
		}

		bool HaveVertexColors() const
		{
			return bHaveVertexColor;
		}

		bool HaveVertexNormals() const
		{
			return bHaveVertexNormals;
		}

		bool HaveVertexUVs() const
		{
			return bHaveVertexUVs;
		}

		std::vector<Vector3>	VertexPositions;
		std::vector<Vector3>	VertexNormals;
		std::vector<Vector3>	VertexColors;
		std::vector<Vector2>	VertexUVs;
		std::vector<Vector3i>	Triangles;
		GeometryAttributeSet	Attributes;
		Box3					Bounds;
		Transform				Pose;
		std::string				ResourceName;
		bool					bHaveVertexColor{ false };
		bool					bHaveVertexNormals{ false };
		bool					bHaveVertexUVs{ false };
	};

	class GeometrySet
	{
	public:
		GeometrySet();
		~GeometrySet();

		bool LoadObj(const char* name, const Transform &pose);
		void CreateGeometryData(int NumMeshes);

	public:
		std::vector<GeometryData*>	mMeshs;
		Box3						mBounds;
	};


}	// namespace Destruction
