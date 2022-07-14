
#include "RigidBodySimulation.h"

#include <assert.h>

#include "PhysicsEntity.h"
#include "MotionIntegration.h"
#include "WarmStart.h"
#include "ForceField.h"
#include "KinematicsTree.h"
#include "../Collision/GeometryQuery.h"
#include "../Collision/GeometryObject.h"
#include "../Collision/BroadPhase.h"
#include "../Collision/NarrowPhase.h"
#include "../Tools/PhysxBinaryParser.h"

RigidBodySimulation::RigidBodySimulation(const RigidBodySimulationParam& param)
{
	m_BPhase = BroadPhase::Create_SAP();
	m_NPhase = NarrowPhase::Create_GJKEPA();
	m_GeometryQuery = new GeometryQuery;
	m_GravityField = new ForceField(param.Gravity);
	m_WindField = nullptr;
	m_SharedMem = nullptr;
	m_SharedMemSize = 0;
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

	for (size_t i = 0; i < m_Kinematics.size(); ++i)
	{
		delete m_Kinematics[i];
	}

	if (m_SharedMem)
	{
		ReleaseSharedMem(m_SharedMem, m_SharedMemSize);
	}
}

void		RigidBodySimulation::Simulate(float dt)
{
	MotionIntegration::Integrate(m_RigidDynamics, dt);

	std::vector<Geometry*> Shapes;
	for (size_t i = 0; i < m_RigidStatics.size(); ++i)
	{
		m_RigidStatics[i]->AppendShapes(&Shapes);
	}
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

	ApplyGravity();
	ApplyWind();

	for (size_t i = 0; i < m_Kinematics.size(); ++i)
	{
		m_Kinematics[i]->Simulate(dt);
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

bool         RigidBodySimulation::LoadPhysxScene(const char *name, bool shared_mem)
{
    std::vector<Geometry*> collection;
	if (shared_mem)
	{
		m_SharedMem = ::LoadPhysxBinaryMmap(name, &collection, m_SharedMemSize);
		if (!m_SharedMem)
		{
			return false;
		}
	}
	else
	{
		bool load_succ = ::LoadPhysxBinary(name, &collection);
		if (!load_succ)
		{
			return false;
		}
	}

    assert(m_GeometryQuery);
    m_GeometryQuery->BuildStaticGeometry(collection, 5);
    return true;
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

bool RigidBodySimulation::LoadAnimation(const std::string& resname, const std::string& filepath, float play_rate, bool begin_play)
{
	for (size_t i = 0; i < m_Kinematics.size(); ++i)
	{
		if (m_Kinematics[i]->GetName() == filepath)
		{
			delete m_Kinematics[i];
			m_Kinematics.erase(m_Kinematics.begin() + i);
			break;
		}
	}

	KinematicsTree* tree = new KinematicsTree;
	if (!tree->Deserialize(filepath))
	{
		delete tree;
		return false;
	}
	tree->SetName(resname);
	tree->SetAnimationPlayRate(play_rate);
	tree->Pause(!begin_play);
	m_Kinematics.push_back(tree);
	return true;
}

bool RigidBodySimulation::BindKinematicsNode(const std::string& resname, const std::string& node, RigidBodyStatic *body)
{
	KinematicsTree* tree = FindKinematics(resname);
	if (tree == nullptr)
	{
		return false;
	}
	return tree->Bind(node, body);
}

KinematicsTree* RigidBodySimulation::FindKinematics(const std::string& resname)
{
	for (size_t i = 0; i < m_Kinematics.size(); ++i)
	{
		if (m_Kinematics[i]->GetName() == resname)
		{
			return m_Kinematics[i];
		}
	}
	return nullptr;
}
