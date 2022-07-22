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
	// before collision; for realistic result, you would 
	// want to set this value between zero and one.
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
};