#pragma once

#include "../Maths/Maths.h"
#include "../Maths/Vector3.h"

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

	Vector3		Center;
	float			Radius;
	Vector3		ExplosionForce0;
	Vector3		ExplosionForce1;
	AttenuationType	Attenuation;
};

class ForceField
{
public:
	virtual ~ForceField() {}
	virtual bool		ApplyForce(RigidBodyDynamic* Rigid) = 0;

public:
	static ForceField*	CreateGrivityField(const Vector3& Gravity);
	static ForceField*	CreateExplosionField(const ExplosionFieldParam& param);
};

