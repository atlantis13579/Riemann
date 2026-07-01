#pragma once

#include <string>

#include "../Src/CollisionPrimitive/StaticMesh.h"
#include "../Src/Maths/Box3.h"

namespace Riemann
{
	struct MeshSimplificationParams
	{
		std::string MeshPath;
		float Ratio = 1.0f;
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
