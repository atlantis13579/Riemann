#pragma once

#include <vector>
#include "../Maths/Vector3.h"
#include "../Maths/Box3d.h"

namespace voro
{
	class container;
}

namespace Geometry
{
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
		Voronoi3d(const std::vector<Vector3>& points, const Box3d& Bounds, const float eps);
		~Voronoi3d();

	public:
		static void GenerateRandomPoints(const Box3d& Bounds, int numPoints, std::vector<Vector3>& points);

		static Box3d GetVoronoiBounds(const Box3d& Bounds, const std::vector<Vector3>& points);

		void ComputeAllCells(std::vector<Cell>& cells, bool paraller_build);
		void ComputeAllNeighbors(std::vector<std::vector<int>>& neighbors, bool exclude_bounds, bool paraller_build);
		void ComputeAllEdges(std::vector<Edge>& edges, std::vector<int>& members, bool paraller_build);

		void Build();

	private:
		size_t mNumPts;
		voro::container *mContainer { nullptr };
		Box3d mBounds;
	};

	class VoronoiMesh3d
	{
	public:
		// plane X * x + Y * y + Z * z = W.
		struct Plane
		{
			float x;
			float y;
			float z;
			float w;
		};

		VoronoiMesh3d() {}

		void Build(const std::vector<Vector3>& points, const Box3d& bounds, const float eps);

		std::vector<Plane>					Planes;
		std::vector<std::pair<int, int>>	Cells;
		std::vector<std::vector<int>>		Boundaries;
		std::vector<Vector3>				BoundaryVertices;
	};
}