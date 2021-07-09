
#include "PhysicsWorld.h"
#include "PhysicsEntity.h"

#include "../Collision/GeometryObject.h"
#include "../Collision/BroadPhase.h"
#include "../Collision/NarrowPhase.h"

PhysicsWorld::PhysicsWorld(const PhysicsWorldParam& param)
{
	m_BPhase = BroadPhase::Create_SAP();
	m_NPhase = NarrowPhase::Create_GJK_EPA();
}

PhysicsWorld::~PhysicsWorld()
{
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

void		PhysicsWorld::Simulate(float dt)
{
	MotionIntegration(dt);

	// BroadPhase Collision Detection
	std::vector<OverlapPair> overlaps;
	if (m_BPhase)
	{
		m_BPhase->ProduceOverlaps(m_Entities, &overlaps);
	}

	// BroadPhase Collision Detection
	std::vector<CollisionPair> collides;
	if (m_NPhase)
	{
		m_NPhase->ProduceCollision(overlaps, &collides);
	}

	// Collstion Resolve
}

void		PhysicsWorld::MotionIntegration(float dt)
{
	for (size_t i = 0; i < m_Entities.size(); ++i)
	{
		PhysicsEntity* Entity = (PhysicsEntity*)m_Entities[i]->GetEntity();

		// -----
	}
}

void		PhysicsWorld::AddGeometry(Geometry* Geom)
{
	m_Entities.push_back(Geom);
}