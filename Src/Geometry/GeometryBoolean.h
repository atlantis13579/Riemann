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
		BooleanOp Operation = BooleanOp::Union;

		float SnapTolerance = 1e-6f;
		bool CollapseDegenerateEdgesOnCut = true;
		bool SimplifyAlongNewEdges = false;
		bool WeldSharedEdges = true;
		bool TrackAllNewEdges = false;
		bool PutResultInInputSpace = true;
		float SimplificationAngleTolerance = 0.1f;
		float TryToImproveTriQualityThreshold = 0.5f;
		bool PreserveTriangleGroups = true;
		bool PreserveVertexUVs = true;
		bool PreserveOverlayUVs = true;
		float UVDistortTolerance = 1e-6f;
		bool PreserveVertexNormals = true;
		float NormalDistortTolerance = 0.01f;
		int PreserveUVsOnlyForMesh = -1;

		// Inputs
		const DynamicMesh* Meshes[2]{ nullptr, nullptr };
		const Transform Transforms[2];

		// Outputs
		DynamicMesh* Result { nullptr };
		Transform ResultTransform;
		std::vector<int> CreatedBoundaryEdges;
		std::set<int> AllNewEdges;
		
		bool PopulateSecondMeshGroupMap = false;
		FIndexMapi SecondMeshGroupMap;

		GeometryBoolean(const DynamicMesh* MeshA, const Transform& TransformA, const DynamicMesh* MeshB, const Transform& TransformB, BooleanOp Op)
			: Operation(Op), Meshes{ MeshA, MeshB }, Transforms{ TransformA, TransformB }
		{
		}

		GeometryBoolean(const DynamicMesh* MeshA, const DynamicMesh* MeshB, BooleanOp Op)
			: Operation(Op), Meshes{ MeshA, MeshB }
		{
		}

		~GeometryBoolean() = default;

		bool Compute();

	};
}
