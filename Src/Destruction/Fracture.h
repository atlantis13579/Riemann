#pragma once

#include <vector>

#include "../Geometry/DynamicMesh.h"
#include "../Geometry/Voronoi3.h"
#include "../Maths/Vector3.h"

namespace Riemann
{
	enum FractureMode
	{
		FractureMode_ParallelX = 0,
		FractureMode_ParallelY,
		FractureMode_ParallelZ,
		FractureMode_VoronoiFracture2D,
		FractureMode_VoronoiFracture3D,
		FractureMode_Cluster,
		FractureMode_Voxel2D,
		FractureMode_Voxel3D,
		FractureMode_Count
	};

	struct FractureOptions
	{
		int Mode = FractureMode_VoronoiFracture3D;
		int PieceCount = 16;
		int PiecesX = 0;
		int PiecesY = 0;
		int PiecesZ = 0;
		int Seed = 7;
		float SnapTolerance = 1e-5f;
		float BoundsPaddingScale = 0.10f;
		float Grout = 0.0f;
		float GroutScale = 0.0f;
		bool WeldSharedEdges = true;
		bool SimplifyAlongCut = false;
		int MinTriangleCount = 1;
		Vector3 Normal = Vector3::Zero();
	};

	struct FracturePiece
	{
		DynamicMesh Mesh;
		Vector3 Site = Vector3::Zero();
		Vector3 Center = Vector3::Zero();
		Vector3 Direction = Vector3::Zero();
		int SiteIndex = -1;
		int CellIndex = -1;
	};

	class Fracture
	{
	public:
		static bool CutByMode(
			const DynamicMesh& SourceMesh,
			std::vector<FracturePiece>& Pieces,
			const FractureOptions& Options = FractureOptions());

		static bool ParallelCutX(
			const DynamicMesh& SourceMesh,
			std::vector<FracturePiece>& Pieces,
			const FractureOptions& Options = FractureOptions());

		static bool ParallelCutY(
			const DynamicMesh& SourceMesh,
			std::vector<FracturePiece>& Pieces,
			const FractureOptions& Options = FractureOptions());

		static bool ParallelCutZ(
			const DynamicMesh& SourceMesh,
			std::vector<FracturePiece>& Pieces,
			const FractureOptions& Options = FractureOptions());

		static bool VoronoiFracture3D(
			const DynamicMesh& SourceMesh,
			std::vector<FracturePiece>& Pieces,
			const FractureOptions& Options = FractureOptions());

		static bool VoronoiFracture2D(
			const DynamicMesh& SourceMesh,
			const Vector3& Normal,
			std::vector<FracturePiece>& Pieces,
			const FractureOptions& Options = FractureOptions());

		static bool ClusterVoronoiFracture(
			const DynamicMesh& SourceMesh,
			std::vector<FracturePiece>& Pieces,
			const FractureOptions& Options = FractureOptions());

		static bool Voxel2D(
			const DynamicMesh& SourceMesh,
			const Vector3& Normal,
			std::vector<FracturePiece>& Pieces,
			const FractureOptions& Options = FractureOptions());

		static bool Voxel3D(
			const DynamicMesh& SourceMesh,
			std::vector<FracturePiece>& Pieces,
			const FractureOptions& Options = FractureOptions());

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
