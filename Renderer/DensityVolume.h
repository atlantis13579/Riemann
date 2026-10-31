#pragma once

#include <cmath>
#include <vector>
#include "../Src/Maths/Vector3.h"
#include "../Src/Maths/Vector4.h"

namespace Riemann
{
	struct DensityVolumeDesc
	{
		Vector3 BoundsMin = Vector3(-1);
		Vector3 BoundsMax = Vector3(1);
		Vector3i Resolution = Vector3i(0, 0, 0);
		std::vector<float> Density;
		float AmbientDensity = 1.225f;
		float RelativeThreshold = 0.08f;
		float Extinction = 2.0f;
		float StepScale = 0.75f;
		bool Smoke = false;
		float ShadowStep = 0.18f;
		Vector4 CompressedColor = Vector4(1.0f, 0.27f, 0.07f, 1.0f);
		Vector4 RarefiedColor = Vector4(0.10f, 0.48f, 1.0f, 1.0f);

		bool IsValid() const
		{
			size_t Count = 1;
			for (int Axis = 0; Axis < 3; ++Axis)
			{
				if (Resolution[Axis] < 1 || Resolution[Axis] > 2048
					|| !std::isfinite(BoundsMin[Axis]) || !std::isfinite(BoundsMax[Axis])
					|| !std::isfinite(BoundsMax[Axis] - BoundsMin[Axis])
					|| BoundsMax[Axis] <= BoundsMin[Axis]) return false;
				Count *= (size_t)Resolution[Axis];
				if (Count > 16 * 1024 * 1024) return false;
			}
			if (Count != Density.size() || !std::isfinite(AmbientDensity) || AmbientDensity <= 0
				|| !std::isfinite(RelativeThreshold) || RelativeThreshold <= 0 || RelativeThreshold >= 1
				|| !std::isfinite(Extinction) || Extinction <= 0
				|| !std::isfinite(ShadowStep) || ShadowStep <= 0
				|| !std::isfinite(StepScale) || StepScale < 0.25f || StepScale > 2.0f) return false;
			for (int C = 0; C < 4; ++C)
				if (!std::isfinite(CompressedColor[C]) || !std::isfinite(RarefiedColor[C])
					|| CompressedColor[C] < 0 || CompressedColor[C] > 1
					|| RarefiedColor[C] < 0 || RarefiedColor[C] > 1) return false;
			for (float Value : Density) if (!std::isfinite(Value) || Value < 0) return false;
			return true;
		}
	};
}
