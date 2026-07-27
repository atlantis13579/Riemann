#pragma once

#include <string>

#include "../Src/CollisionPrimitive/StaticMesh.h"
#include "../Src/Maths/Box3.h"

namespace Riemann
{
	enum MeshSimplificationAlgorithm
	{
		MeshSimplificationAlgorithm_HoppeQEM = 0,
		MeshSimplificationAlgorithm_GarlandQEM,
		MeshSimplificationAlgorithm_Count
	};

	struct MeshSimplificationParams
	{
		std::string MeshPath;
		float Ratio = 0.05f;
		int Algorithm = MeshSimplificationAlgorithm_HoppeQEM;
	};

	struct MeshSimplificationSource
	{
		StaticMesh Mesh;
		Box3 Bounds = Box3::Empty();
		float MaxSeparation = 1.0f;
		std::string Status;
	};

	struct MeshSimplificationResult
	{
		StaticMesh SourceMesh;
		StaticMesh SimplifiedMesh;
		Box3 SourceBounds = Box3::Empty();
		float MaxSeparation = 1.0f;
		std::string Status;
	};

	bool LoadMeshSimplificationSource(const std::string& meshPath, MeshSimplificationSource* source);
	bool BuildMeshSimplificationPanel(const MeshSimplificationParams& params, MeshSimplificationResult* result);
}
