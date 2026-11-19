#pragma once

#include <string>
#include <vector>

#include "../Src/CollisionPrimitive/StaticMesh.h"
#include "../Src/Maths/Box3.h"
#include "../Src/Maths/Vector3.h"

namespace Riemann
{
	enum MeshCuttingMode
	{
		MeshCuttingMode_ParallelX = 0,
		MeshCuttingMode_ParallelY,
		MeshCuttingMode_ParallelZ,
		MeshCuttingMode_VoronoiFracture2D,
		MeshCuttingMode_VoronoiFracture3D,
		MeshCuttingMode_Cluster,
		MeshCuttingMode_Voxel2D,
		MeshCuttingMode_Voxel3D,
		MeshCuttingMode_VHACD,
		MeshCuttingMode_Count
	};

	struct MeshCuttingParams
	{
		std::string ObjPath;
		int Mode = MeshCuttingMode_VoronoiFracture3D;
		int PieceCount = 16;
		int PiecesX = 4;
		int PiecesY = 3;
		int PiecesZ = 2;
		int Seed = 7;
		bool CapCutFaces = true;
		Vector3 ViewRight = Vector3::UnitX();
		Vector3 ViewUp = Vector3::UnitY();
		float ViewAspect = 1.0f;
	};

	struct MeshCuttingPiece
	{
		StaticMesh Mesh;
		Vector3 Center = Vector3::Zero();
		Vector3 SeparationOffset = Vector3::Zero();
	};

	struct MeshCuttingSource
	{
		StaticMesh Mesh;
		Box3 Bounds = Box3::Empty();
		std::string Status;
	};

	struct MeshCuttingResult
	{
		Box3 SourceBounds = Box3::Empty();
		std::vector<MeshCuttingPiece> Pieces;
		std::string Status;
	};

	bool LoadMeshCuttingSource(const std::string& objPath, MeshCuttingSource* source);
	bool BuildMeshCuttingPanel(const MeshCuttingParams& params, MeshCuttingResult* result);
}
