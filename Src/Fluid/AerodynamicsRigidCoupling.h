#pragma once

#include "../RigidBodyDynamics/ForceField.h"
#include "../Collision/GeometryHandle.h"
#include "AerodynamicsTypes.h"
#include <unordered_set>

namespace Riemann
{
	class AerodynamicsSimulation;
	class Geometry;
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

	// The externally owned simulation must outlive this field. Incompressible
	// coupling uses moving boundaries; barotropic coupling samples one-way loads.
	// Synchronize after rigid integration, then step gas before the next rigid step.
	class AerodynamicsRigidCoupling final : public ForceField
	{
	public:
		explicit AerodynamicsRigidCoupling(AerodynamicsSimulation* Simulation,
			const AerodynamicsRigidCouplingSettings& Settings = AerodynamicsRigidCouplingSettings());

		bool ApplyForce(RigidBodyDynamic* Rigid) override;
		void SetSimulation(AerodynamicsSimulation* Simulation);
		bool Synchronize(const std::vector<Geometry*>& Geometries, const std::vector<AerodynamicsSurfaceMesh>& Surfaces = {});
		void Update(float Dt) override;
		static bool BuildObstacle(const Geometry* Shape, AerodynamicsObstacle& Obstacle);
		bool Evaluate(const RigidBodyDynamic* Rigid,
			AerodynamicsRigidCouplingResult* Result) const;
		bool IsValid() const;
		const AerodynamicsRigidCouplingSettings& GetSettings() const { return m_Settings; }

	private:
		AerodynamicsSimulation* m_Simulation;
		AerodynamicsRigidCouplingSettings m_Settings;
		bool m_SettingsValid;
		std::vector<GeometryHandle> m_Handles;
		std::vector<Vector3> m_Centers;
		std::vector<AerodynamicsRigidLoad> m_Loads;
		bool m_FilterSamples = false;
		std::unordered_set<const Geometry*> m_SampleShapes;
	};
}
