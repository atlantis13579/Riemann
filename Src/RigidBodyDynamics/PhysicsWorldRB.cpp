
#include "PhysicsWorldRB.h"
#include "PhysicsEntity.h"
#include "MotionIntegration.h"

#include "../Collision/GeometryQuery.h"
#include "../Collision/GeometryObject.h"
#include "../Collision/BroadPhase.h"
#include "../Collision/NarrowPhase.h"

PhysicsWorldRB::PhysicsWorldRB(const PhysicsWorldRBParam& param)
{
	m_BPhase = BroadPhase::Create_SAP();
	m_NPhase = NarrowPhase::Create_GJKEPA();
	m_GeometryQuery = new GeometryQuery;

	m_Gravity = param.Gravity;
}

PhysicsWorldRB::~PhysicsWorldRB()
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

	for (size_t i = 0; i < m_Entities.size(); ++i)
	{
		delete m_Entities[i];
	}
	m_Entities.clear();
}

void		PhysicsWorldRB::Simulate(float dt)
{
	MotionIntegration::Integrate(m_Entities, dt);

	std::vector<OverlapPair> overlaps;
	if (m_BPhase)
	{
		m_BPhase->ProduceOverlaps(m_Entities, &overlaps);
	}

	std::vector<ContactManifold> contacts;
	if (m_NPhase)
	{
		m_NPhase->CollisionDetection(overlaps, &contacts);
	}

}

void		PhysicsWorldRB::CreateRigidBody(Geometry* Geom, const RigidBodyParam& param)
{
	RigidBody* Rigid = RigidBody::CreateRigidBody(Geom, param);
	Geom->SetEntity(Rigid);
	m_Entities.push_back(Geom);
}