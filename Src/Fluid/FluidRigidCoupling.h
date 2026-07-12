#pragma once

#include <cstddef>
#include <vector>

#include "../RigidBodyDynamics/ForceField.h"

namespace Riemann
{
	class FluidSimulation;
	class RigidBodyDynamic;

	struct FluidRigidCouplingSettings
	{
		FluidRigidCouplingSettings();

		int SamplesPerAxis;
		float QuadraticDragCoefficient;
		float ViscousDragCoefficient;
		float MaxLinearAcceleration;
		float MaxAngularAcceleration;
		float FeedbackRadiusScale;
	};

	struct FluidRigidCouplingResult
	{
		FluidRigidCouplingResult();

		Vector3 Force;
		Vector3 Torque;
		Vector3 BuoyancyForce;
		Vector3 DragForce;
		Vector3 FeedbackPosition;
		float DisplacedMass;
		float SubmergedVolume;
		float ClampScale;
		int SampleCount;
		int ActiveSampleCount;
		bool Clamped;
	};

	// The fluid is externally owned and must outlive this force field.
	class FluidRigidCoupling final : public ForceField
	{
	public:
		explicit FluidRigidCoupling(FluidSimulation* Fluid,
			const FluidRigidCouplingSettings& Settings = FluidRigidCouplingSettings());

		void Update(float DeltaTime) override;
		bool ApplyForce(RigidBodyDynamic* Rigid) override;

		bool Evaluate(const RigidBodyDynamic* Rigid, FluidRigidCouplingResult* Result) const;
		bool IsValid() const;
		const FluidRigidCouplingSettings& GetSettings() const { return m_Settings; }
		size_t GetPendingFeedbackCount() const { return m_PendingFeedback.size(); }
		size_t GetLastFeedbackFailureCount() const { return m_LastFeedbackFailureCount; }
		size_t GetTotalFeedbackFailureCount() const { return m_TotalFeedbackFailureCount; }

	private:
		struct PendingImpulse
		{
			Vector3 Position;
			Vector3 Impulse;
			float Radius;
		};

		FluidSimulation* m_Fluid;
		FluidRigidCouplingSettings m_Settings;
		std::vector<PendingImpulse> m_PendingFeedback;
		float m_DeltaTime;
		bool m_SettingsValid;
		size_t m_LastFeedbackFailureCount;
		size_t m_TotalFeedbackFailureCount;
	};
}
