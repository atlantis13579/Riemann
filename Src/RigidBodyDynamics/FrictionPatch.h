#pragma once

#include <cstdint>
#include <memory>
#include <vector>

namespace Riemann
{
	class Geometry;
	class RigidBody;
	class RagdollVelocityContext;
	class ContactManifold;
	struct Contact;
	struct GeneralizedVelocity;

	struct FrictionPatchSettings
	{
		bool Enabled = false;
		float AnchorMatchDistance = 0.05f;
		float AnchorBreakDistance = 0.1f;
		float MinimumAnchorSeparation = 0.01f;
		float ContactTolerance = 1.0e-5f;
		float NormalCosine = 0.95f;
		float PositionBias = 0.2f;
		float MaximumCorrectionSpeed = 0.2f;
		float RestickSpeed = 0.02f;
		float WarmStartFactor = 0.85f;
		float MinimumTimeStepRatio = 0.25f;
		float MaximumTimeStepRatio = 4.0f;
		float MinimumEffectiveMass = 1.0e-9f;
		float RelativeRankTolerance = 1.0e-6f;
		uint64_t RetentionFrames = 2;
		bool IsValid() const;
	};

	struct FrictionContact
	{
		Geometry* ShapeA;
		Geometry* ShapeB;
		uint32_t PatchId;
		Contact* Point;
	};

	struct FrictionPatchStatistics
	{
		size_t Patches = 0;
		size_t Anchors = 0;
		size_t RetainedAnchors = 0;
		size_t SlidingAnchors = 0;
	};

	class FrictionPatchSolver
	{
	public:
		explicit FrictionPatchSolver(const FrictionPatchSettings& Settings = FrictionPatchSettings());
		~FrictionPatchSolver();
		void SetSettings(const FrictionPatchSettings& Settings);
		void SetWarmStartingEnabled(bool Enabled);
		const FrictionPatchSettings& GetSettings() const;
		void Reset();
		void BeginFrame(uint64_t Frame);
		void BeginSolve(const std::vector<RigidBody*>& Bodies = {}, GeneralizedVelocity* Velocities = nullptr, bool CarryNormalLoad = true, RagdollVelocityContext* ReducedRagdolls = nullptr);
		void Prepare(const std::vector<FrictionContact>& Contacts, float Dt);
		void Prepare(const std::vector<Geometry*>& Geometries, const std::vector<ContactManifold*>& Contacts, float Dt);
		float Solve(float Dt);
		const FrictionPatchStatistics& GetStatistics() const;

	private:
		struct Storage;
		std::unique_ptr<Storage> m_Storage;
	};
}
