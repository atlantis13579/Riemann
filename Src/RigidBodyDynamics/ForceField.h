#pragma once

#include "../Geometry/DenseTensorField3d.h"

class	RigidBody;

class ForceField : public DenseTensorField3d<Vector3d>
{
public:
	ForceField(const Vector3d& Force);
	~ForceField();

public:
	bool		ApplyForce(RigidBody *Rigid);
};