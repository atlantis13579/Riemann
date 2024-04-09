#pragma once

#include "../Maths/Transform.h"

namespace Geometry
{
	class GeometryData;

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

		// input
		const GeometryData* Meshes[2];
		const Transform Transforms[2];

		// output
		GeometryData* MeshNew { nullptr };
		Transform TransformNew;

		GeometryBoolean(const GeometryData* MeshA, const Transform& TransformA, const GeometryData* MeshB, const Transform& TransformB, BooleanOp Operation)
			: Meshes{ MeshA, MeshB }, Transforms{ TransformA, TransformB }, Operation(Operation)
		{
		}

		GeometryBoolean(const GeometryData* MeshA, const GeometryData* MeshB, BooleanOp Operation)
			: Meshes{ MeshA, MeshB }, Operation(Operation)
		{
		}

		~GeometryBoolean()
		{
		}

		void Compute();
	};
}