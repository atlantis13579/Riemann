#pragma once

namespace Riemann
{
	enum DefaultPhysicsMaterial
	{
		Default = 0,
		Concrete,
		Ice,
		BounceBall,
		NUM_DEFAULT_PHYSICS_MATERIALS,
	};

	struct PhysicsMaterial
	{
		// The coefficient of restitution between two objects,
		// is defined as the ratio between the parting speed of
		// two colliders after collisionand the closing speed
		// before collision
		// 0 = completely inelastic collision response
		// 1 = completely elastic collision response)
		float		Restitution;
		float		FrictionDynamic;
		float		FrictionStatic;

		PhysicsMaterial(float r, float fd, float fs)
		{
			Restitution = r;
			FrictionDynamic = fd;
			FrictionStatic = fs;
		}
		static const PhysicsMaterial* getMaterial(unsigned short mid)
		{
			if (mid >= NUM_DEFAULT_PHYSICS_MATERIALS)
			{
				mid = 0;
			}
			return &materialTable[mid];
		}
		static const PhysicsMaterial materialTable[(int)NUM_DEFAULT_PHYSICS_MATERIALS];
	};
}
