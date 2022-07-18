#pragma once

struct PhysicsMaterial
{
	PhysicsMaterial()
	{
		ContactBeta = 0.4f;
		Restitution = 0.0f;
		FrictionDynamic = 1.0f;
		FrictionStatic = 1.0f;
	}

	float	ContactBeta;
	float	Restitution;
	float	FrictionDynamic;
	float	FrictionStatic;
};
