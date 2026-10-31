#pragma once

#include "DensityVolume.h"
#include "../Src/Fluid/SmokeSimulation.h"

namespace Riemann
{
	inline void BuildSmokeDensityVolume(const SmokeSimulation& Simulation, DensityVolumeDesc* Volume,
		float Extinction = 5.5f, const Vector3& Color = Vector3(0.85f, 0.89f, 0.96f))
	{
		const SmokeSimulationSettings& Settings = Simulation.GetSettings();
		Volume->BoundsMin = Settings.BoundsMin;
		Volume->BoundsMax = Settings.BoundsMax;
		Volume->Resolution = Settings.Resolution;
		Volume->Density = Simulation.GetDensity();
		Volume->Smoke = true;
		Volume->Extinction = Extinction;
		Volume->StepScale = 0.65f;
		Volume->CompressedColor = Vector4(Color.x, Color.y, Color.z, 1);
	}
}
