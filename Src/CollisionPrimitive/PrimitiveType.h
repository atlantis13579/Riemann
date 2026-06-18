#pragma once

#include "../Maths/Box3.h"
#include "../Maths/Vector3.h"
#include "../Maths/Matrix3.h"

namespace Riemann
{
	enum class PrimitiveType : unsigned char
	{
		BOX = 0,
		PLANE = 1,
		SPHERE = 2,
		CAPSULE = 3,
		CYLINDER = 4,
		HEIGHTFIELD = 5,
		CONVEX_MESH = 6,
		TRIANGLE_MESH = 7,
		TYPE_COUNT,
	};

	struct MassParameters
	{
		float	Mass;
		float	Volume;
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
