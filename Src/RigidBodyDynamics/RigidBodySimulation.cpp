
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

	for (size_t i = 0; i < m_StaticBodies.size(); ++i)
	{
		delete m_StaticBodies[i];
	}

	for (size_t i = 0; i < m_DynamicBodies.size(); ++i)
	{
		delete m_DynamicBodies[i];
	}

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
	for (size_t i = 0; i < m_Kinematics.size(); ++i)
	{
		m_Kinematics[i]->Simulate(dt);
	}

	std::vector<Geometry*> geoms;
	for (size_t i = 0; i < m_StaticBodies.size(); ++i)
	{
		m_StaticBodies[i]->GetGeometries(&geoms);
	}
	for (size_t i = 0; i < m_DynamicBodies.size(); ++i)
	{
		m_DynamicBodies[i]->GetGeometries(&geoms);
	}

	std::vector<OverlapPair> overlaps;
	if (m_BPhase)
	{
		m_BPhase->ProduceOverlaps(geoms, &overlaps);
	}

	std::vector<ContactManifold*> manifolds;
	if (m_NPhase && !overlaps.empty())
	{
		m_NPhase->CollisionDetection(overlaps, &manifolds);
	}

	if (m_RPhase && !manifolds.empty())
	{
		m_RPhase->ResolveContact(manifolds, dt);
	}

	ApplyForceFields();

	MotionIntegration::Integrate(m_DynamicBodies, dt, MotionIntegration::IntegrateMethod::ExplicitEuler);
	return;
}

void		RigidBodySimulation::ApplyForceFields()
{
	for (size_t j = 0; j < m_Fields.size(); ++j)
	{
		for (size_t i = 0; i < m_DynamicBodies.size(); ++i)
		{
			m_Fields[j]->ApplyForce(m_DynamicBodies[i]);
		}
	}
}

bool         RigidBodySimulation::LoadPhysxScene(const char *name, bool shared_mem)
{
    std::vector<RigidBody*> collection;
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

	std::vector<Geometry*> geoms;
	for (size_t i = 0; i < collection.size(); ++i)
	{
		RigidBody* body = collection[i];
		if (body->mRigidType == RigidType::Static)
		{
			m_StaticBodies.push_back(body->CastStatic());
		}
		else if (body->mRigidType == RigidType::Dynamic)
		{
			m_DynamicBodies.push_back(body->CastDynamic());
		}
		body->GetGeometries(&geoms);
	}

    assert(m_GeometryQuery);
    m_GeometryQuery->BuildStaticGeometry(geoms, 5);
    return true;
}

RigidBody*	RigidBodySimulation::CreateRigidBody(Geometry* Geom, const RigidBodyParam& param)
{
	RigidBody *body = RigidBody::CreateRigidBody(param, Geom);
	if (body->mRigidType == RigidType::Static)
	{
		m_StaticBodies.push_back(body->CastStatic());
	}
	else if (body->mRigidType == RigidType::Dynamic)
	{
		m_DynamicBodies.push_back(body->CastDynamic());
	}
	return body;
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

KinematicsDriver* RigidBodySimulation::FindKinematics(const std::string& resname)
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

float RigidBodySimulation::GetSystemTotalEnergy() const
{
	float Energy = 0.0f;
	for (size_t i = 0; i < m_DynamicBodies.size(); ++i)
	{
		Energy += m_DynamicBodies[i]->GetKinematicsEnergy();
	}
	return Energy;
}

float RigidBodySimulation::GetSystemTotalLinearKinematicsEnergy() const
{
	float Energy = 0.0f;
	for (size_t i = 0; i < m_DynamicBodies.size(); ++i)
	{
		Energy += m_DynamicBodies[i]->GetLinearKinematicsEnergy();
	}
	return Energy;
}

float RigidBodySimulation::GetSystemTotalAngularKinematicsEnergy() const
{
	float Energy = 0.0f;
	for (size_t i = 0; i < m_DynamicBodies.size(); ++i)
	{
		Energy += m_DynamicBodies[i]->GetAngularKinematicsEnergy();
	}
	return Energy;
}

Vector3 RigidBodySimulation::GetSystemTotalLinearMomentum() const
{
	Vector3 Momentum(0.0f);
	for (size_t i = 0; i < m_DynamicBodies.size(); ++i)
	{
		Momentum += m_DynamicBodies[i]->GetLinearMomentum();
	}
	return Momentum;
}

Vector3 RigidBodySimulation::GetSystemTotalAngularMomentum() const
{
	Vector3 Momentum(0.0f);
	for (size_t i = 0; i < m_DynamicBodies.size(); ++i)
	{
		Momentum += m_DynamicBodies[i]->GetAngularMomentum();
	}
	return Momentum;
}
