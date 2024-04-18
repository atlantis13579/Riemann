#pragma once

#include <vector>

#include "../Maths/Box3.h"
#include "../Maths/Vector3.h"
#include "../Geometry/Voronoi3.h"

namespace Riemann
{
	class DynamicMesh;

	class VoronoiMesh
	{
	public:
		VoronoiMesh(const std::vector<Vector3>& points, const Box3& bounds, const float eps);
		~VoronoiMesh();

	private:
		void BuildMesh_SinglePlane(const Voronoi3& v);
		void BuildMesh_WithoutNoise(const Voronoi3& v);
		void BuildMesh_WithNoise(const Voronoi3& v);

	private:
		std::vector<DynamicMesh*> mMeshs;
		bool AssumeConvexCells{ true };

		int OutsideCellIndex = -1;
		int NumUVLayers = 1;
	};
}	// namespace Riemann
