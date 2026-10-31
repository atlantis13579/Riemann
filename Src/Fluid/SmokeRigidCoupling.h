#pragma once

#include "SmokeSimulation.h"
#include "../RigidBodyDynamics/ForceField.h"
#include "../Collision/GeometryHandle.h"

namespace Riemann
{
	class Geometry;
	class RigidBodyDynamic;

	struct SmokeRigidCouplingSettings
	{
		float AirDensity = 1.225f;
		float MaxLinearAcceleration = 20.0f;
		float MaxAngularAcceleration = 30.0f;
	};

	// The externally owned smoke simulation must outlive this force field.
	// Synchronize after rigid integration, then step smoke before the next rigid step.
	class SmokeRigidCoupling final : public ForceField
	{
	public:
		explicit SmokeRigidCoupling(SmokeSimulation* Simulation,
			const SmokeRigidCouplingSettings& Settings = SmokeRigidCouplingSettings());
		void SetSimulation(SmokeSimulation* Simulation);
		bool Synchronize(const std::vector<Geometry*>& Geometries);
		void Update(float Dt) override;
		bool ApplyForce(RigidBodyDynamic* Rigid) override;
		bool Evaluate(const RigidBodyDynamic* Rigid, SmokeRigidLoad& Load) const;
		bool IsValid() const;
		static bool BuildObstacle(const Geometry* GeometryObject, SmokeObstacle& Obstacle);

	private:
		SmokeSimulation* m_Simulation;
		SmokeRigidCouplingSettings m_Settings;
		std::vector<GeometryHandle> m_Handles;
		std::vector<Vector3> m_Centers;
		std::vector<SmokeRigidLoad> m_Loads;
	};
}
