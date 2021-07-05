
#pragma once

#include "../Maths/BoundingBox3d.h"

const float kEpsilonGeometric = 0.000001f;

class GeometricObject
{
public:
	virtual bool			IntersectRay(const Vector3d& Origin, const Vector3d& Dir, float* t) const = 0;

	virtual bool			IntersectPoint(const Vector3d& Point) const = 0;

	virtual bool			IntersectAABB(const Vector3d& Min, const Vector3d& Max) const = 0;

	virtual BoundingBox3d	GetBoundingBox() const = 0;
};