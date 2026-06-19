#pragma once

#include <vector>

#include "../Geometry/DynamicMesh.h"
#include "../Geometry/Voronoi3.h"
#include "../Maths/Vector3.h"

namespace Riemann
{
	struct FractureOptions
	{
		float SnapTolerance = 1e-5f;
		float BoundsPaddingScale = 0.10f;
		float Grout = 0.0f;
		bool WeldSharedEdges = true;
		bool SimplifyAlongCut = false;
		int MinTriangleCount = 1;
	};

	struct FracturePiece
	{
		DynamicMesh Mesh;
		Vector3 Site = Vector3::Zero();
		Vector3 Center = Vector3::Zero();
		int SiteIndex = -1;
	};

	class Fracture
	{
	public:
		static bool VoronoiFracture(
			const DynamicMesh& SourceMesh,
			const std::vector<Vector3>& Sites,
			std::vector<FracturePiece>& Pieces,
			const FractureOptions& Options = FractureOptions());

		static bool VoronoiFracture(
			const DynamicMesh& SourceMesh,
			const std::vector<Vector3>& Sites,
			std::vector<DynamicMesh>& Pieces,
			const FractureOptions& Options = FractureOptions());

		static bool BuildVoronoiCellMesh(
			const Voronoi3::Cell& Cell,
			const Vector3& Site,
			DynamicMesh& CellMesh);
	};

}	// namespace Riemann
