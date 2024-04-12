#pragma once

#include "../Maths/Transform.h"
#include "DynamicMesh.h"

namespace Riemann
{
	class GeometryCut
	{
	public:
		DynamicMesh* Meshe[2];

		GeometryCut(DynamicMesh* MeshA, DynamicMesh* MeshB)
			: Meshe{ MeshA, MeshB }
		{
		}

		bool Compute(const IntersectionsQueryResult& Intersections);

		float SnapTolerance = 1e-4f;
		bool bMutuallyCut = true;
		bool bCutCoplanar = false;

		std::vector<int> VertexChains[2];
		std::vector<int> SegmentToChain[2];
	};

	class GeometryBoolean
	{
	public:
		enum class BooleanOp
		{
			Union,
			Difference,
			Intersect,
			TrimInside,
			TrimOutside,
			NewGroupInside,
			NewGroupOutside
		};
		BooleanOp Operation;
		float SnapTolerance = 1e-6f;
		bool bCollapseDegenerateEdgesOnCut = true;

		// input
		const DynamicMesh* Meshes[2];
		const Transform Transforms[2];

		// output
		DynamicMesh* MeshNew { nullptr };
		Transform TransformNew;

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