
#include "RigidBodySimulation.h"

#include <assert.h>

#include "PhysicsEntity.h"
#include "MotionIntegration.h"
#include "WarmStart.h"
#include "ForceField.h"
#include "AnimationTree.h"
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

	for (size_t i = 0; i < m_RigidStatics.size(); ++i)
	{
		delete m_RigidStatics[i];
	}
	m_RigidStatics.clear();

	for (size_t i = 0; i < m_RigidDynamics.size(); ++i)
	{
		delete m_RigidDynamics[i];
	}
	m_RigidDynamics.clear();

	for (size_t i = 0; i < m_Animations.size(); ++i)
	{
		delete m_Animations[i];
	}
}

void		RigidBodySimulation::Simulate(float dt)
{
	MotionIntegration::Integrate(m_RigidDynamics, dt);

	std::vector<Geometry*> Shapes;
	for (size_t i = 0; i < m_RigidDynamics.size(); ++i)
	{
		m_RigidDynamics[i]->AppendShapes(&Shapes);
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

	for (size_t i = 0; i < m_Animations.size(); ++i)
	{
		m_Animations[i]->Simulate(dt);
	}

	return;
}

void		RigidBodySimulation::ApplyGravity()
{
	if (m_GravityField == nullptr)
	{
		return;
	}

	for (size_t i = 0; i < m_RigidDynamics.size(); ++i)
	{
		if (m_RigidDynamics[i]->DisableGravity)
		{
			continue;
		}
		m_GravityField->ApplyForce(m_RigidDynamics[i]);
	}
}

void		RigidBodySimulation::ApplyWind()
{
	if (m_WindField == nullptr)
	{
		return;
	}

	for (size_t i = 0; i < m_RigidDynamics.size(); ++i)
	{
		m_WindField->ApplyForce(m_RigidDynamics[i]);
	}
}

RigidBody*	RigidBodySimulation::CreateRigidBody(Geometry* Geom, const RigidBodyParam& param)
{
	if (param.Static)
	{
		RigidBodyStatic* Rigid = RigidBodyStatic::CreateRigidBody(Geom, param);
		Geom->SetEntity(Rigid);
		m_RigidStatics.push_back(Rigid);
		return Rigid;
	}

	RigidBodyDynamic* Rigid = RigidBodyDynamic::CreateRigidBody(Geom, param);
	Geom->SetEntity(Rigid);
	m_RigidDynamics.push_back(Rigid);
	return Rigid;
}

bool RigidBodySimulation::LoadAnimation(const std::string& filepath, float play_rate)
{
	AnimationTree* tree = new AnimationTree;
	if (!tree->Deserialize(filepath))
	{
		delete tree;
		return false;
	}
	tree->SetAnimationPlayRate(play_rate);
	m_Animations.push_back(tree);
	return true;
}

bool RigidBodySimulation::BindAnimationNode(const std::string& anim, const std::string& node, RigidBodyStatic *body)
{
	AnimationTree* tree = FindAnimation(anim);
	if (tree == nullptr)
	{
		return false;
	}
	return tree->Bind(node, body);
}

AnimationTree* RigidBodySimulation::FindAnimation(const std::string& anim)
{
	for (size_t i = 0; i < m_Animations.size(); ++i)
	{
		if (m_Animations[i]->GetName() == anim)
		{
			return m_Animations[i];
		}
	}
	return nullptr;
}
