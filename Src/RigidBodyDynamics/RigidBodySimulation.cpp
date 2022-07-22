
#include "RigidBodySimulation.h"

#include <assert.h>

#include "../Core/Base.h"
#include "../Collision/GeometryQuery.h"
#include "../Collision/GeometryObject.h"
#include "../Tools/PhysxBinaryParser.h"
#include "BroadPhase.h"
#include "NarrowPhase.h"
#include "MotionIntegration.h"
#include "SequentialImpulseSolver.h"
#include "ForceField.h"
#include "KinematicsTree.h"


RigidBodySimulation::RigidBodySimulation(const RigidBodySimulationParam& param)
{
	m_BPhase = BroadPhase::Create_SAP();
	m_NPhase = NarrowPhase::Create_GJKEPA();
	m_RPhase = ResolutionPhase::CreateSequentialImpulseSolver();
	m_GeometryQuery = new GeometryQuery;
	m_Fields.push_back(ForceField::CreateGrivityField(param.Gravity));
	m_SharedMem = nullptr;
	m_SharedMemSize = 0;
}

RigidBodySimulation::~RigidBodySimulation()
{
	SAFE_DELETE(m_GeometryQuery);
	SAFE_DELETE(m_BPhase);
	SAFE_DELETE(m_NPhase);
	SAFE_DELETE(m_RPhase);

	for (size_t i = 0; i < m_Fields.size(); ++i)
	{
		delete m_Fields[i];
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
	SimulateST(dt);
}

void		RigidBodySimulation::SimulateST(float dt)
{
	ApplyForceFields();

	MotionIntegration::Integrate(m_RigidDynamics, dt, MotionIntegration::IntegrateMethod::ExplicitEuler);

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
	if (!overlaps.empty())
	{
		m_NPhase->CollisionDetection(overlaps, &manifolds);
	}

	if (m_RPhase)
	{
		m_RPhase->ResolveContact(manifolds, dt);
	}

	for (size_t i = 0; i < m_Kinematics.size(); ++i)
	{
		m_Kinematics[i]->Simulate(dt);
	}

	return;
}

void		RigidBodySimulation::ApplyForceFields()
{
	for (size_t j = 0; j < m_Fields.size(); ++j)
	{
		for (size_t i = 0; i < m_RigidDynamics.size(); ++i)
		{
			m_Fields[j]->ApplyForce(m_RigidDynamics[i]);
		}
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
		RigidBodyStatic* Rigid = RigidBodyStatic::CreateRigidBody(Geom);
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
