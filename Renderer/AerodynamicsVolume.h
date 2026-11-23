#pragma once

#include "DensityVolume.h"
#include "../Src/Fluid/AerodynamicsSimulation.h"

namespace Riemann
{
	inline bool BuildAerodynamicsDensityVolume(const AerodynamicsSimulation& Simulation, DensityVolumeDesc* Volume,
		float RelativeThreshold = 0.08f, float Extinction = 5.5f, const Vector3& Color = Vector3(0.85f, 0.89f, 0.96f))
	{
		if (!Volume || !Simulation.IsInitialized()) return false;
		*Volume = DensityVolumeDesc();
		const auto& Settings = Simulation.GetSettings();
		Volume->BoundsMin = Settings.BoundsMin; Volume->BoundsMax = Settings.BoundsMax; Volume->Resolution = Settings.Resolution;
		Volume->AmbientDensity = Settings.AmbientDensity; Volume->RelativeThreshold = RelativeThreshold;
		if (Simulation.HasTracerField())
		{
			Volume->Density = Simulation.GetTracerDensity(); Volume->Tracer = true; Volume->Extinction = Extinction;
			Volume->StepScale = 0.65f; Volume->CompressedColor = Vector4(Color.x, Color.y, Color.z, 1);
		}
		else if (!Simulation.BuildDensityField(&Volume->Density)) return false;
		return Volume->IsValid();
	}
}
