#pragma once

#include "../RigidBodyDynamics/ForceField.h"

namespace Riemann
{
	class AerodynamicsSimulation;
	class RigidBodyDynamic;

	struct AerodynamicsRigidCouplingSettings
	{
		AerodynamicsRigidCouplingSettings();

		int SurfaceSamplesPerAxis;
		float DragCoefficient;
		float MaxLinearAcceleration;
		float MaxAngularAcceleration;
	};

	struct AerodynamicsRigidCouplingResult
	{
		AerodynamicsRigidCouplingResult();

		Vector3 Force;
		Vector3 Torque;
		float ClampScale;
		float SampledSurfaceArea;
		int SampleCount;
		bool Clamped;
	};

	// The gas simulation is externally owned and must outlive this one-way coupling.
	class AerodynamicsRigidCoupling final : public ForceField
	{
	public:
		explicit AerodynamicsRigidCoupling(AerodynamicsSimulation* Simulation,
			const AerodynamicsRigidCouplingSettings& Settings = AerodynamicsRigidCouplingSettings());

		bool ApplyForce(RigidBodyDynamic* Rigid) override;
		bool Evaluate(const RigidBodyDynamic* Rigid,
			AerodynamicsRigidCouplingResult* Result) const;
		bool IsValid() const;
		const AerodynamicsRigidCouplingSettings& GetSettings() const { return m_Settings; }

	private:
		AerodynamicsSimulation* m_Simulation;
		AerodynamicsRigidCouplingSettings m_Settings;
		bool m_SettingsValid;
	};
}
