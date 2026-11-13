#pragma once

#include "CollidingContact.h"
#include "../Maths/Box3.h"

namespace Riemann
{
	struct ContactGenerationSettings
	{
		bool UseShapeOffsets = true;
		bool SpeculativeContacts = false;
		float LengthScale = 1.0f;
		float RelativeTolerance = 1.0e-5f;
		float MinimumTolerance = 1.0e-6f;
		float RelativePenetrationSlop = 0.001f;
		float MaximumSpeculativeDistance = 1000.0f;
		float FaceClipSquaredDistance = 0.02f;
		int DistanceIterations = 64;
		bool IsValid() const;
	};

	class ContactGeneration
	{
	public:
		static bool UsesOffsets(const GeometryWorldState& A, const GeometryWorldState& B,
			const ContactGenerationSettings& Settings);
		static float ShapeScale(const Geometry* Shape, const ContactGenerationSettings& Settings);
		static float PairScale(const GeometryWorldState& A, const GeometryWorldState& B,
			const ContactGenerationSettings& Settings);
		static float Tolerance(const GeometryWorldState& A, const GeometryWorldState& B,
			const ContactGenerationSettings& Settings);
		static uint32_t Feature(const GeometryWorldState& State, const Vector3& Position,
			const ContactGenerationSettings& Settings);
		static float MotionDistance(const GeometryWorldState& State, const ContactGenerationSettings& Settings, float Dt);
		static float GenerationDistance(const GeometryWorldState& A, const GeometryWorldState& B,
			const ContactGenerationSettings& Settings, float Dt);
		static Box3 Bounds(const GeometryWorldState& State, const ContactGenerationSettings& Settings, float Dt);
		static bool GenerateSeparated(int IndexA, int IndexB, const GeometryWorldState& A, const GeometryWorldState& B,
			const ContactGenerationSettings& Settings, float Dt, ContactManifold& Manifold, bool& SeparationKnown);
		static void ApplyOffsets(const GeometryWorldState& A, const GeometryWorldState& B,
			const ContactGenerationSettings& Settings, ContactManifold& Manifold);
	};
}
