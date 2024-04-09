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

		int GetNumVertices() const { return (int)VertexPositions.size(); }
		int GetNumTriangles() const { return (int)Triangles.size(); }
		Vector3* GetVertexBuffer() { return VertexPositions.data(); }
		Vector3i* GetIndexBuffer() { return Triangles.data(); }

		int AppendVertex(const VertexInfo& info);
		int AppendTriangle(const Vector3i& tri);

		void SetColor(int idx, const Vector3& val);
		void SetNormal(int idx, const Vector3& val);
		void SetUV(int idx, const Vector2& val, int layer);

		bool Simplify(float rate);

		void ApplyTransform(const Transform &trans, bool bReverseOrientationIfNeeded);
		void ApplyTransform(Transform3& trans, bool bReverseOrientationIfNeeded);
		void ReverseOrientation(bool bFlipNormals);

		Vector2 GetUV(int idx, int layer) const
		{
			return VertexUVs[idx];
		}

		Vector3 GetTriangleCentroid(int idx) const;
		Box3 GetTriangleBounds(int idx) const;
		void GetTriangleVertices(int idx, Vector3 &a, Vector3 &b, Vector3 &c) const;

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

		void CalculateBounds();

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

	class GeometryAABBTree
	{
	public:
		GeometryAABBTree(GeometryData *data);
		~GeometryAABBTree() { Mesh = nullptr; }

		struct PointIntersection
		{
			int TriangleID[2];
			Vector3 Point;
		};

		struct SegmentIntersection
		{
			int TriangleID[2];
			Vector3 Point[2];
		};

		struct PolygonIntersection
		{
			int TriangleID[2];
			Vector3 Point[6];
			int Quantity;
		};

		struct IntersectionsQueryResult
		{
			std::vector<PointIntersection> Points;
			std::vector<SegmentIntersection> Segments;
			std::vector<PolygonIntersection> Polygons;
		};

		IntersectionsQueryResult FindAllIntersections(const GeometryAABBTree& OtherTree, const Transform * TransformF = nullptr) const;

	private:
		void Build();
		void Build(std::vector<int> &Triangles, std::vector<Vector3> &Centers);

		struct FBoxesSet
		{
			std::vector<int> BoxToIndex;
			std::vector<Box3> AABB;
			std::vector<int> IndexList;
			int IBoxCur;
			int IIndicesCur;
			FBoxesSet()
			{
				IBoxCur = 0;
				IIndicesCur = 0;
			}
		};
		int SplitTriSetMidpoint(std::vector<int>& Triangles, std::vector<Vector3>& Centers, int IStart, int ICount, int Depth, int MinTriCount, FBoxesSet& Tris, FBoxesSet& Nodes, Box3& Box);

		Box3 GetAABB(int idx, const Transform* TransformF) const;

		void FindIntersections(
			int iBox, const GeometryAABBTree& OtherTree, const Transform* TransformF,
			int oBox, int depth, IntersectionsQueryResult& result) const;

	private:
		GeometryData	 *Mesh { nullptr };
		std::vector<int> BoxToIndex;
		std::vector<Box3> AABB;
		std::vector<int> IndexList;
		int TrianglesEnd = -1;
		int RootIndex = -1;
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
