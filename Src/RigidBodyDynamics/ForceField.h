#pragma once

#include "../Geometry/DenseTensorField3d.h"

class	RigidBodyDynamic;

class ForceField : public DenseTensorField3d<Vector3d>
{
public:
	ForceField(const Vector3d& Force);
	~ForceField();

public:
	bool		ApplyForce(RigidBodyDynamic *Rigid);
};