#include <assert.h>

#include "RigidBodySimulation.h"

#include "../Core/Base.h"
#include "../Core/LogSystem.h"
#include "../Core/JobSystem.h"
#include "../Collision/DynamicAABBTree.h"
#include "../Collision/GeometryQuery.h"
#include "../Collision/GeometryObject.h"
#include "../Modules/Tools/PhysxBinaryParser.h"
#include "BroadPhase.h"
#include "NarrowPhase.h"
#include "MotionIntegration.h"
#include "SequentialImpulseSolver.h"
#include "ForceField.h"
#include "KinematicsTree.h"

RigidBodySimulation::RigidBodySimulation(const RigidBodySimulationParam& param)
{
	m_GeometryQuery = new GeometryQuery;
	if (param.broadphase == BroadPhaseSolver::SAP)
	{
		m_BPhase = BroadPhase::Create_SAP();
	}
	else if (param.broadphase == BroadPhaseSolver::Bruteforce)
	{
		m_BPhase = BroadPhase::Create_Bruteforce();
	}
	if (param.broadphase == BroadPhaseSolver::AllPairs)
	{
		m_BPhase = BroadPhase::Create_AllPairs();
	}
	if (param.broadphase == BroadPhaseSolver::DynamicAABB)
	{
		m_GeometryQuery->CreateDynamicGeometry();
		m_BPhase = BroadPhase::Create_DynamicAABB(m_GeometryQuery->GetDynamicTree());
	}
	m_NPhase = NarrowPhase::Create_GJKEPA();
	m_Solver = ConstraintSolver::CreateSequentialImpulseSolver();
	m_IntegrateMethod = param.integrateMethod;
	m_Fields.push_back(ForceField::CreateGrivityField(param.gravityAcc));
	m_Jobsystem = new JobSystem;
	if (param.workerThreads != 0)
	{
		m_Jobsystem->CreateWorkers(param.workerThreads);
	}
	m_SharedMem = nullptr;
	m_SharedMemSize = 0;
}

RigidBodySimulation::~RigidBodySimulation()
{
	SAFE_DELETE(m_GeometryQuery);
	SAFE_DELETE(m_BPhase);
	SAFE_DELETE(m_NPhase);
	SAFE_DELETE(m_Solver);

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

	if (m_Jobsystem)
	{
		m_Jobsystem->Terminate();
		delete m_Jobsystem;
	}

	if (m_SharedMem)
	{
		ReleaseSharedMem(m_SharedMem, m_SharedMemSize);
	}
}

void		RigidBodySimulation::Simulate()
{
	m_Clock.tick++;

	SimulateST(m_Clock.deltatime);
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
		m_NPhase->CollisionDetection(geoms, overlaps, &manifolds);
	}

	if (m_Solver && !manifolds.empty())
	{
		m_Solver->ResolveContact(geoms, manifolds, dt);
	}

	PreIntegrate(dt);

	MotionIntegration::Integrate(m_DynamicBodies, dt, (int)m_IntegrateMethod);

	PostIntegrate(dt);

	return;
}

void		RigidBodySimulation::PreIntegrate(float dt)
{
	for (size_t i = 0; i < m_DynamicBodies.size(); ++i)
	{
		RigidBodyDynamic* Body = m_DynamicBodies[i];
		Body->ExtForce /= (dt * 2.0f);	 // Force Unit  1 Newton =  1 kg m / s^2
		Body->ExtTorque /= (dt * 2.0f);
		if (!Body->Sleeping)
			continue;
		if (Body->GetKinematicsEnergy() > Body->SleepThreshold)
		{
			Body->Wakeup();
		}
	}

	for (size_t j = 0; j < m_Fields.size(); ++j)
	{
		m_Fields[j]->Update(dt);

		for (size_t i = 0; i < m_DynamicBodies.size(); ++i)
		{
			m_Fields[j]->ApplyForce(m_DynamicBodies[i]);
		}
	}
}

void		RigidBodySimulation::PostIntegrate(float dt)
{
	DynamicAABBTree* tree = m_GeometryQuery->GetDynamicTree();
	for (size_t i = 0; i < m_DynamicBodies.size(); ++i)
	{
		RigidBodyDynamic* Body = m_DynamicBodies[i];
		Body->ExtForce.SetZero();
		Body->ExtTorque.SetZero();
		if (Body->Sleeping)
			continue;
		Body->AutoSleep();
		if (tree)
		{
			tree->Update(Body->mNodeId, Body->mGeometry->GetBoundingVolume_WorldSpace(), Body->GetLinearVelocity());
		}
	}
}

bool         RigidBodySimulation::LoadPhysxScene(const char *name, bool shared_mem)
{
    std::vector<RigidBody*> collection;
	std::vector<Geometry*> geoms;
	if (shared_mem)
	{
		m_SharedMem = ::LoadPhysxBinaryMmap(name, nullptr, &geoms, m_SharedMemSize);
		if (!m_SharedMem)
		{
			return false;
		}
	}
	else
	{
		bool load_succ = ::LoadPhysxBinary(name, nullptr, &geoms);
		if (!load_succ)
		{
			return false;
		}
	}

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

	if (m_GeometryQuery->GetDynamicTree())
	{
		body->mNodeId = m_GeometryQuery->GetDynamicTree()->Add(Geom->GetBoundingVolume_WorldSpace(), Geom);
	}
	return body;
}

bool RigidBodySimulation::RemoveRigidBody(RigidBody* Body)
{
	if (Body->mRigidType == RigidType::Static)
	{
		for (size_t i = 0; i < m_StaticBodies.size(); ++i)
		{
			if (m_StaticBodies[i] == Body)
			{
				m_StaticBodies.erase(m_StaticBodies.begin() + i);
				if (m_GeometryQuery->GetDynamicTree())
				{
					m_GeometryQuery->GetDynamicTree()->Remove(Body->mNodeId);
				}
				return true;
			}
		}
	}
	else if (Body->mRigidType == RigidType::Dynamic)
	{
		for (size_t i = 0; i < m_DynamicBodies.size(); ++i)
		{
			if (m_DynamicBodies[i] == Body)
			{
				m_DynamicBodies.erase(m_DynamicBodies.begin() + i);
				if (m_GeometryQuery->GetDynamicTree())
				{
					m_GeometryQuery->GetDynamicTree()->Remove(Body->mNodeId);
				}
				return true;
			}
		}
	}
	return false;
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

void RigidBodySimulation::GetAllGeometries(std::vector<Geometry*> *geoms)
{
	for (size_t i = 0; i < m_StaticBodies.size(); ++i)
	{
		m_StaticBodies[i]->GetGeometries(geoms);
	}
	for (size_t i = 0; i < m_DynamicBodies.size(); ++i)
	{
		m_DynamicBodies[i]->GetGeometries(geoms);
	}
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
