#pragma once

struct PhysicsMaterial
{
	float	ContactBeta;
	float	Restitution;
	float	FrictionDynamic;
	float	FrictionStatic;

	PhysicsMaterial()
	{
		ContactBeta = DefaultContactBeta();
		Restitution = DefaultRestitution();
		FrictionDynamic = DefaultFrictionDynamic();
		FrictionStatic = DefaultFrictionStatic();
	}

	static constexpr float DefaultContactBeta()
	{
		return 0.3f;
	}

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