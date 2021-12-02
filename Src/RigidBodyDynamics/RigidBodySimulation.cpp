
#include "RigidBodySimulation.h"

#include <assert.h>

#include "PhysicsEntity.h"
#include "MotionIntegration.h"
#include "WarmStart.h"
#include "ForceField.h"
#include "../Collision/GeometryQuery.h"
#include "../Collision/GeometryObject.h"
#include "../Collision/BroadPhase.h"
#include "../Collision/NarrowPhase.h"

RigidBodySimulation::RigidBodySimulation(const RigidBodySimulationParam& param)
{
	m_BPhase = BroadPhase::Create_SAP();
	m_NPhase = NarrowPhase::Create_GJKEPA();
	m_GeometryQuery = new GeometryQuery;
	m_GravityField = new ForceField(param.Gravity);
	m_WindField = nullptr;
}

RigidBodySimulation::~RigidBodySimulation()
{
	if (m_GeometryQuery)
	{
		delete m_GeometryQuery;
		m_GeometryQuery = nullptr;
	}

	if (m_BPhase)
	{
		delete m_BPhase;
		m_BPhase = nullptr;
	}

	if (m_NPhase)
	{
		delete m_NPhase;
		m_NPhase = nullptr;
	}

	if (m_GravityField)
	{
		delete m_GravityField;
		m_GravityField = nullptr;
	}

	if (m_WindField)
	{
		delete m_WindField;
		m_WindField = nullptr;
	}

	for (size_t i = 0; i < m_Entities.size(); ++i)
	{
		delete m_Entities[i];
	}
	m_Entities.clear();
}

void		RigidBodySimulation::Simulate(float dt)
{
	MotionIntegration::Integrate(m_Entities, dt);

	std::vector<Geometry*> Shapes;
	for (size_t i = 0; i < m_Entities.size(); ++i)
	{
		m_Entities[i]->AppendShapes(&Shapes);
	}

	std::vector<OverlapPair> overlaps;
	if (m_BPhase)
	{
		m_BPhase->ProduceOverlaps(Shapes, &overlaps);
	}

	std::vector<ContactManifold> manifolds;
	if (m_NPhase && !overlaps.empty())
	{
		m_NPhase->CollisionDetection(overlaps, &manifolds);
	}

	WarmStart::Manifolds(manifolds, dt);

	if (m_GravityField)
	{
		ApplyGravity();
	}



	return;
}

void		RigidBodySimulation::ApplyGravity()
{
	if (m_GravityField == nullptr)
	{
		return;
	}

	for (size_t i = 0; i < m_Entities.size(); ++i)
	{
		if (m_Entities[i]->DisableGravity)
		{
			continue;
		}
		m_GravityField->ApplyForce(m_Entities[i]);
	}
}

void		RigidBodySimulation::ApplyWind()
{
	if (m_WindField == nullptr)
	{
		return;
	}

	for (size_t i = 0; i < m_Entities.size(); ++i)
	{
		m_WindField->ApplyForce(m_Entities[i]);
	}
}

RigidBodyDynamic*	RigidBodySimulation::CreateRigidBody(Geometry* Geom, const RigidBodyParam& param)
{
	RigidBodyDynamic* Rigid = RigidBodyDynamic::CreateRigidBody(Geom, param);
	Geom->SetEntity(Rigid);
	m_Entities.push_back(Rigid);
	return Rigid;
}