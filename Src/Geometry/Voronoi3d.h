#pragma once

#include <vector>
#include "../Maths/Vector3.h"
#include "../Maths/Box3.h"

namespace voro
{
	class container;
}

namespace Geometry
{
	class GeometrySet;

	class Voronoi3d
	{
	public:
		struct Cell
		{
			std::vector<Vector3>	Vertices;
			std::vector<int>		Faces;
			std::vector<int>		Neighbors;
			std::vector<Vector3>	Normals;
		};

		// plane X * x + Y * y + Z * z = W.
		struct Plane
		{
			Vector3 GetOrigin() const
			{
				return GetNormal() * w;
			}

			Vector3 GetNormal() const
			{
				return Vector3(x, y, z);
			}

			float x;
			float y;
			float z;
			float w;
		};

		struct Edge
		{
			Edge(const Vector3& _v0, const Vector3& _v1)
			{
				v0 = _v0;
				v1 = _v1;
			}

			Vector3		v0;
			Vector3		v1;
		};

		Voronoi3d();
		Voronoi3d(const std::vector<Vector3>& points, const Box3& bounds, const float eps);
		~Voronoi3d();

	public:

		void Set(const std::vector<Vector3>& points, const Box3& bounds, const float eps);

		static void GenerateRandomPoints(const Box3& Bounds, int numPoints, std::vector<Vector3>& points);

		static Box3 GetVoronoiBounds(const Box3& Bounds, const std::vector<Vector3>& points);

		void ComputeAllCells(std::vector<Cell>& cells, bool paraller_build);
		void ComputeAllNeighbors(std::vector<std::vector<int>>& neighbors, bool exclude_bounds, bool paraller_build);
		void ComputeAllEdges(std::vector<Edge>& edges, std::vector<int>& members, bool paraller_build);

		void Build();

		int GetNumPoints() const { return (int)mNumPoints; }

	private:
		size_t								mNumPoints { 0 };
		voro::container						*mContainer { nullptr };
		std::vector<Vector3>				mPoints;
		Box3								mBounds;

	public:
		std::vector<Plane>					mPlanes;
		std::vector<std::pair<int, int>>	mCells;
		std::vector<std::vector<int>>		mBoundaries;
		std::vector<Vector3>				mBoundaryVertices;
	};

	class VoronoiMesh
	{
	public:
		VoronoiMesh(const std::vector<Vector3>& points, const Box3& bounds, const float eps);
		~VoronoiMesh();

	private:
		GeometrySet* mSet { nullptr };
	};
}