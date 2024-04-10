#pragma once

#include "../Maths/Box3.h"
#include "../Maths/Vector3.h"
#include "../Maths/Matrix3.h"

namespace Riemann
{
	enum class ShapeType : unsigned char
	{
		UNKNOWN = 0,
		BOX,
		PLANE,
		SPHERE,
		CAPSULE,
		CYLINDER,
		HEIGHTFIELD,
		CONVEX_MESH,
		TRIANGLE_MESH,
		TYPE_COUNT,
	};

	struct MassParameters
	{
		float Mass;
		float Volume;
		Matrix3 InertiaMat;
		Vector3 CenterOfMass;
		Box3	BoundingVolume;
	};

	enum class IntersectionType : unsigned char
	{
		Empty,
		Point,
		Segment,
		Line,
		Polygon,
		Plane,
		MultiSegment,
		Unknown
	};

	#define MAX_FACE_POINTS	16
}
