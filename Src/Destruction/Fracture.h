#pragma once

#include <vector>

#include "../Geometry/DynamicMesh.h"
#include "../Geometry/MeshCut.h"
#include "../Geometry/Voronoi3.h"
#include "../Maths/Vector3.h"

namespace Riemann
{
	struct FracturePiece
	{
		DynamicMesh Mesh;
		Vector3 Site = Vector3::Zero();
		Vector3 Center = Vector3::Zero();
		Vector3 Direction = Vector3::Zero();
		int SiteIndex = -1;
		int CellIndex = -1;
		std::vector<int> NeighborPieceIndices;
	};

	class Fracture
	{
	public:
		static bool ParallelCutX(
			const DynamicMesh& SourceMesh,
			std::vector<FracturePiece>& Pieces,
			int PieceCount,
			const PlanarCutOptions& CutOptions);

		static bool ParallelCutY(
			const DynamicMesh& SourceMesh,
			std::vector<FracturePiece>& Pieces,
			int PieceCount,
			const PlanarCutOptions& CutOptions);

		static bool ParallelCutZ(
			const DynamicMesh& SourceMesh,
			std::vector<FracturePiece>& Pieces,
			int PieceCount,
			const PlanarCutOptions& CutOptions);

		static bool VoronoiFracture3D(
			const DynamicMesh& SourceMesh,
			std::vector<FracturePiece>& Pieces,
			int PieceCount,
			int Seed,
			const PlanarCutOptions& CutOptions);

		static bool VoronoiFracture2D(
			const DynamicMesh& SourceMesh,
			const Vector3& Normal,
			std::vector<FracturePiece>& Pieces,
			int PieceCount,
			int Seed,
			const PlanarCutOptions& CutOptions);

		static bool ClusterVoronoiFracture(
			const DynamicMesh& SourceMesh,
			std::vector<FracturePiece>& Pieces,
			int PieceCount,
			int Seed,
			const PlanarCutOptions& CutOptions);

		static bool Voxel2D(
			const DynamicMesh& SourceMesh,
			const Vector3& Normal,
			std::vector<FracturePiece>& Pieces,
			int PiecesX,
			int PiecesY,
			int PieceCount,
			const PlanarCutOptions& CutOptions);

		static bool Voxel3D(
			const DynamicMesh& SourceMesh,
			std::vector<FracturePiece>& Pieces,
			int PiecesX,
			int PiecesY,
			int PiecesZ,
			int PieceCount,
			const PlanarCutOptions& CutOptions);

		static bool VoronoiFracture(
			const DynamicMesh& SourceMesh,
			const std::vector<Vector3>& Sites,
			std::vector<FracturePiece>& Pieces,
			const PlanarCutOptions& CutOptions);

		static bool VoronoiFracture(
			const DynamicMesh& SourceMesh,
			const std::vector<Vector3>& Sites,
			std::vector<DynamicMesh>& Pieces,
			const PlanarCutOptions& CutOptions);

		static bool BuildVoronoiCellMesh(
			const Voronoi3::Cell& Cell,
			const Vector3& Site,
			DynamicMesh& CellMesh);
	};

}	// namespace Riemann
