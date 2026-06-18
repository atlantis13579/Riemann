
#include "PhysicsMaterial.h"

namespace Riemann
{
	const PhysicsMaterial PhysicsMaterial::materialTable[] =
	{
		PhysicsMaterial(0.0f, 1.0f, 1.0f),			// Default
		PhysicsMaterial(0.0f, 1.0f, 1.0f),			// Concrete
		PhysicsMaterial(0.0f, 0.0f, 0.0f),			// Ice
		PhysicsMaterial(1.0f, 0.0f, 0.0f),			// BounceBall
	};
}