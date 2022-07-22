#pragma once

struct PhysicsMaterial
{
	float	Restitution;
	float	FrictionDynamic;
	float	FrictionStatic;

	PhysicsMaterial()
	{
		Restitution = DefaultRestitution();
		FrictionDynamic = DefaultFrictionDynamic();
		FrictionStatic = DefaultFrictionStatic();
	}

	// The coefficient of restitution between two objects, 
	// is defined as the ratio between the parting speed of 
	// two colliders after collisionand the closing speed 
	// before collision
	// 0 = completely inelastic collision response
	// 1 = completely elastic collision response)
	static constexpr float DefaultRestitution()
	{
		return 1.0f;
	}

	static constexpr float DefaultFrictionDynamic()
	{
		return 1.0f;
	}

	static constexpr float DefaultFrictionStatic()
	{
		return 1.0f;
	}
};