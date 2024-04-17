#pragma once

#include <set>
#include <vector>

#include "../Maths/Transform.h"
#include "DynamicMesh.h"

namespace Riemann
{
	class GeometryBoolean
	{
	public:
		enum class BooleanOp
		{
			Union,
			Difference,
			Intersect,
		};
		BooleanOp Operation;

		float SnapTolerance = 1e-6f;
		bool bCollapseDegenerateEdgesOnCut = true;
		bool bSimplifyAlongNewEdges = false;
		bool bWeldSharedEdges = true;
		bool bTrackAllNewEdges = false;
		bool bPutResultInInputSpace = true;
		float SimplificationAngleTolerance = 0.1f;
		float TryToImproveTriQualityThreshold = 0.5f;
		bool bPreserveTriangleGroups = true;
		bool bPreserveVertexUVs = true;
		bool bPreserveOverlayUVs = true;
		float UVDistortTolerance = 1e-6f;
		bool bPreserveVertexNormals = true;
		float NormalDistortTolerance = 0.01f;
		int PreserveUVsOnlyForMesh = -1;

		// input
		const DynamicMesh* Meshes[2];
		const Transform Transforms[2];

		// output
		DynamicMesh* MeshNew { nullptr };
		Transform TransformNew;
		std::vector<int> CreatedBoundaryEdges;
		std::set<int> AllNewEdges;
		
		bool bPopulateSecondMeshGroupMap = false;
		FIndexMapi SecondMeshGroupMap;

		GeometryBoolean(const DynamicMesh* MeshA, const Transform& TransformA, const DynamicMesh* MeshB, const Transform& TransformB, BooleanOp Operation)
			: Meshes{ MeshA, MeshB }, Transforms{ TransformA, TransformB }, Operation(Operation)
		{
		}

		GeometryBoolean(const DynamicMesh* MeshA, const DynamicMesh* MeshB, BooleanOp Operation)
			: Meshes{ MeshA, MeshB }, Operation(Operation)
		{
		}

		~GeometryBoolean()
		{
		}

		bool Compute();

	};
}