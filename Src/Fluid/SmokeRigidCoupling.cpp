#include "SmokeRigidCoupling.h"
#include "../Collision/GeometryObject.h"
#include "../CollisionPrimitive/AxisAlignedBox3.h"
#include "../CollisionPrimitive/Sphere3.h"
#include "../RigidBodyDynamics/RigidBody.h"
#include <algorithm>
#include <cmath>

namespace Riemann
{
	SmokeRigidCoupling::SmokeRigidCoupling(SmokeSimulation* Simulation, const SmokeRigidCouplingSettings& Settings)
		: m_Simulation(Simulation), m_Settings(Settings) {}

	void SmokeRigidCoupling::SetSimulation(SmokeSimulation* Simulation)
	{
		m_Simulation = Simulation;
		m_Loads.clear(); m_Handles.clear(); m_Centers.clear();
	}

	bool SmokeRigidCoupling::IsValid() const
	{
		return m_Simulation && m_Simulation->IsInitialized()
			&& std::isfinite(m_Settings.AirDensity) && m_Settings.AirDensity > 0
			&& std::isfinite(m_Settings.MaxLinearAcceleration) && m_Settings.MaxLinearAcceleration > 0
			&& std::isfinite(m_Settings.MaxAngularAcceleration) && m_Settings.MaxAngularAcceleration > 0;
	}

	bool SmokeRigidCoupling::BuildObstacle(const Geometry* GeometryObject, SmokeObstacle& Obstacle)
	{
		if (!GeometryObject) return false;
		const RigidBody* Body = GeometryObject->GetParent<RigidBody>();
		const Transform T = Body ? Body->GetGeometryTransform(GeometryObject) : *GeometryObject->GetTransform();
		Obstacle = SmokeObstacle();
		Obstacle.Rotation = T.quat;
		if (const Sphere3* Sphere = GeometryObject->GetShapeObj<Sphere3>())
		{
			Obstacle.Center = T * Sphere->Center;
			Obstacle.Radius = Sphere->Radius;
		}
		else if (const AxisAlignedBox3* Box = GeometryObject->GetShapeObj<AxisAlignedBox3>())
		{
			Obstacle.Type = SmokeObstacle::Shape::Box;
			Obstacle.Center = T * ((Box->Min + Box->Max) * 0.5f);
			Obstacle.HalfExtent = (Box->Max - Box->Min) * 0.5f;
		}
		else return false;
		if (Body && Body->mRigidType != RigidType::Static)
		{
			Obstacle.LinearVelocity = Body->V + Body->W.Cross(Obstacle.Center - Body->X);
			Obstacle.AngularVelocity = Body->W;
		}
		return true;
	}

	bool SmokeRigidCoupling::Synchronize(const std::vector<Geometry*>& Geometries)
	{
		if (!IsValid()) return false;
		std::vector<SmokeObstacle> Obstacles;
		std::vector<GeometryHandle> Ids;
		std::vector<Vector3> Centers;
		for (const Geometry* G : Geometries)
		{
			SmokeObstacle O;
			if (!BuildObstacle(G, O)) continue;
			Obstacles.push_back(O);
			const RigidBody* Body = G->GetParent<RigidBody>();
			Ids.push_back(Body ? Body->FindGeometryHandle(G) : GeometryHandle());
			Centers.push_back(O.Center);
		}
		if (!m_Simulation->SetRigidObstacles(Obstacles)) return false;
		m_Handles.swap(Ids); m_Centers.swap(Centers); m_Loads.clear();
		return true;
	}

	void SmokeRigidCoupling::Update(float)
	{
		if (!IsValid() || !m_Simulation->ComputeRigidLoads(m_Settings.AirDensity, m_Loads)) m_Loads.clear();
	}

	bool SmokeRigidCoupling::Evaluate(const RigidBodyDynamic* Rigid, SmokeRigidLoad& Load) const
	{
		Load = SmokeRigidLoad();
		if (!IsValid() || !Rigid || m_Loads.size() != m_Handles.size()) return false;
		for (size_t I = 0; I < m_Loads.size(); ++I)
		{
			if (!m_Handles[I].IsValid() || std::find(Rigid->mGeometryHandles.begin(), Rigid->mGeometryHandles.end(), m_Handles[I]) == Rigid->mGeometryHandles.end()) continue;
			Load.Force += m_Loads[I].Force;
			Load.Torque += m_Loads[I].Torque + (m_Centers[I] - Rigid->X).Cross(m_Loads[I].Force);
		}
		const float Linear = Load.Force.Length() * Rigid->GetInverseMass();
		const float Angular = (Rigid->GetInverseInertia_WorldSpace() * Load.Torque).Length();
		if (!std::isfinite(Linear) || !std::isfinite(Angular)) return false;
		float Scale = 1;
		if (Linear > m_Settings.MaxLinearAcceleration) Scale = m_Settings.MaxLinearAcceleration / Linear;
		if (Angular > m_Settings.MaxAngularAcceleration) Scale = std::min(Scale, m_Settings.MaxAngularAcceleration / Angular);
		Load.Force *= Scale; Load.Torque *= Scale;
		return true;
	}

	bool SmokeRigidCoupling::ApplyForce(RigidBodyDynamic* Rigid)
	{
		SmokeRigidLoad Load;
		if (!Evaluate(Rigid, Load)) return false;
		Rigid->ApplyForce(Load.Force); Rigid->ApplyTorgue(Load.Torque);
		return true;
	}
}
