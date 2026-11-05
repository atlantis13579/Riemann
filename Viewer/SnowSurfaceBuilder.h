#pragma once

#include "../Renderer/RiemannRenderer.h"
#include "../Src/Continuum/SnowSimulation.h"
#include "../Src/Continuum/SandSimulation.h"

namespace Riemann
{
	bool BuildSnowSurface(const SnowSimulation& Simulation, SnowQuality Quality, RenderMeshDesc& Mesh);
	bool BuildSandSurface(const SandSimulation& Simulation, RenderMeshDesc& Mesh);
	RenderMeshDesc BuildSnowCapsule(const SnowCapsule& Capsule);
}
