#pragma once

enum DefaultPhysicsMaterial
{
	Concrete = 0,
	Ice,
	BounceBall,
	NUM_DEFAULT_PHYSICS_MATERIALS,
};

struct PhysicsMaterial
{
	float		Restitution;
	float		FrictionDynamic;
	float		FrictionStatic;

	PhysicsMaterial(float r, float fd, float fs)
	{
		Restitution = r;
		FrictionDynamic = fd;
		FrictionStatic = fs;
	}

	// The coefficient of restitution between two objects, 
	// is defined as the ratio between the parting speed of 
	// two colliders after collisionand the closing speed 
	// before collision
	// 0 = completely inelastic collision response
	// 1 = completely elastic collision response)
	static constexpr float DefaultRestitution()
	{
		return 0.0f;
	}

	static constexpr float DefaultFrictionDynamic()
	{
		return 1.0f;
	}

	static constexpr float DefaultFrictionStatic()
	{
		return 1.0f;
	}

	static const PhysicsMaterial defaultMeterialTable[(int)NUM_DEFAULT_PHYSICS_MATERIALS];
};