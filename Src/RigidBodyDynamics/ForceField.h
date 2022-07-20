#pragma once

#include "../Maths/Maths.h"
#include "../Maths/Vector3d.h"

class	RigidBodyDynamic;

struct ExplosionFieldParam
{
	enum class AttenuationType
	{
		LINEAR = 0,
		SQUARE,
		CUBIC,
		SQRT,
		ATTENUATION_COUNT
	};

	ExplosionFieldParam()
	{
		Attenuation = AttenuationType::LINEAR;
	}

	Vector3d		Center;
	float			Radius;
	Vector3d		ExplosionForce0;
	Vector3d		ExplosionForce1;
	AttenuationType	Attenuation;
};

class ForceField
{
public:
	virtual ~ForceField() {}
	virtual bool		ApplyForce(RigidBodyDynamic* Rigid) = 0;

public:
	static ForceField*	CreateGrivityField(const Vector3d& Gravity);
	static ForceField*	CreateExplosionField(const ExplosionFieldParam& param);
};

