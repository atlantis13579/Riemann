
#pragma once

#include <vector>

#include "../Maths/Vector3d.h"
#include "../Maths/Box3d.h"
#include "../Maths/Matrix3d.h"
#include "Plane3d.h"

struct HullFace3d
{
	explicit HullFace3d(const Plane3d& p, unsigned char n)
	{
		Plane = p;
		NumVerties = n;
	}
	Plane3d			Plane;
	unsigned char	NumVerties;
};

class ConvexMesh
{
public:
	Vector3d						CenterOfMass;
	Box3d							BoundingVolume;
	Matrix3d						Inertia;
	std::vector<HullFace3d>			Faces;
	unsigned int					NumVertices;
	unsigned int					NumEdges;
	unsigned int					NumFaces;

	int				EulerNumber() const
	{
		return NumVertices - NumEdges + NumFaces;
	}

	bool			IntersectRay(const Vector3d& Origin, const Vector3d& Dir, float* t) const
	{
		// TODO
		return false;
	}

	const Box3d&	GetBoundingVolume() const
	{
		return BoundingVolume;
	}

	const Matrix3d&	GetInertiaTensor(float Mass) const
	{
		return Inertia;
	}

	Vector3d		GetSupport(const Vector3d& dir) const
	{
		// TOOD
		return Vector3d::Zero();
	}

};