#pragma once

#include <stdint.h>

#include "../Maths/Box3.h"
#include "../Maths/Transform.h"

namespace Riemann
{
	class Geometry;
	class RigidBody;

	struct GeometryWorldState
	{
		GeometryWorldState()
			: Geom(nullptr)
			, Body(nullptr)
			, Displacement(Vector3::Zero())
			, Version(0)
			, FrameId(0)
			, Moved(false)
			, MovingRigid(false)
		{
		}

		Geometry* Geom;
		RigidBody* Body;
		Transform BodyToWorld;
		Transform ShapeToWorld;
		Box3 WorldBounds;
		Vector3 Displacement;
		uint64_t Version;
		uint64_t FrameId;
		bool Moved;
		bool MovingRigid;
	};
}
