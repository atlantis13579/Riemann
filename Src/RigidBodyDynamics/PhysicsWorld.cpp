#include <assert.h>

#include "PhysicsWorld.h"

#include "../Core/Base.h"
#include "../Core/File.h"
#include "../Core/LogSystem.h"
#include "../Core/JobSystem.h"
#include "../Collision/DynamicAABBTree.h"
#include "../Collision/GeometryQuery.h"
#include "../Collision/GeometryObject.h"
#include "../Modules/Tools/PhysxBinaryParser.h"
#include "BroadPhase.h"
#include "CollidingContact.h"
#include "NarrowPhase.h"
#include "MotionIntegration.h"
#include "SequentialImpulseSolver.h"
#include "ForceField.h"
#include "KinematicsTree.h"

namespace Riemann
{
	PhysicsWorld::PhysicsWorld(const PhysicsWorldParam& param)
	{
		m_GeometryQuery = new GeometryQuery;
		m_BPhase = nullptr;
		m_SceneQuery = param.sceneQueryType;
		if (m_SceneQuery == SceneQueryType::DynamicAABB)
		{
			m_GeometryQuery->CreateDynamicGeometry();
		}

		if (param.broadphase == BroadPhaseSolver::SAP)
		{
			m_BPhase = BroadPhase::Create_SAP();
		}
		else if (param.broadphase == BroadPhaseSolver::ABP)
		{
			m_BPhase = BroadPhase::Create_ABP();
		}
		else if (param.broadphase == BroadPhaseSolver::MBP)
		{
			m_BPhase = BroadPhase::Create_MBP();
		}
		else if (param.broadphase == BroadPhaseSolver::Bruteforce)
		{
			m_BPhase = BroadPhase::Create_Bruteforce();
		}
		else if (param.broadphase == BroadPhaseSolver::AllPairs)
		{
			m_BPhase = BroadPhase::Create_AllPairs();
		}
		else if (param.broadphase == BroadPhaseSolver::DynamicAABB)
		{
			DynamicAABBTree* tree = m_GeometryQuery->GetDynamicTree();
			m_BPhase = tree ? BroadPhase::Create_DynamicAABB(tree) : BroadPhase::Create_SAP();
		}
		if (m_BPhase == nullptr)
		{
			m_BPhase = BroadPhase::Create_SAP();
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
		m_SceneResource = nullptr;
	}

	PhysicsWorld::~PhysicsWorld()
	{
		SAFE_DELETE(m_BPhase);
		SAFE_DELETE(m_GeometryQuery);
		SAFE_DELETE(m_NPhase);
		SAFE_DELETE(m_Solver);

		for (size_t i = 0; i < m_Fields.size(); ++i)
		{
			delete m_Fields[i];
		}
		m_Fields.clear();

		for (size_t i = 0; i < m_StaticBodies.size(); ++i)
		{
			delete m_StaticBodies[i];
		}
		m_StaticBodies.clear();

		for (size_t i = 0; i < m_DynamicBodies.size(); ++i)
		{
			delete m_DynamicBodies[i];
		}
		m_DynamicBodies.clear();

		for (size_t i = 0; i < m_Kinematics.size(); ++i)
		{
			delete m_Kinematics[i];
		}
		m_Kinematics.clear();

		if (m_Jobsystem)
		{
			m_Jobsystem->Terminate();
			delete m_Jobsystem;
		}

		if (m_SceneResource)
		{
			delete m_SceneResource;
			m_SceneResource = nullptr;
		}
	}

	void		PhysicsWorld::Simulate()
	{
		m_Clock.tick++;

		SimulateST(m_Clock.deltatime);
	}

	void		PhysicsWorld::Reset()
	{
		for (size_t i = 0; i < m_StaticBodies.size(); ++i)
		{
			m_StaticBodies[i]->ReleaseGeometries();
			delete m_StaticBodies[i];
		}
		m_StaticBodies.clear();

		for (size_t i = 0; i < m_DynamicBodies.size(); ++i)
		{
			m_DynamicBodies[i]->ReleaseGeometries();
			delete m_DynamicBodies[i];
		}
		m_DynamicBodies.clear();

		for (size_t i = 0; i < m_Kinematics.size(); ++i)
		{
			delete m_Kinematics[i];
		}
		m_Kinematics.clear();

		if (m_SceneQuery == SceneQueryType::DynamicAABB)
		{
			m_GeometryQuery->ClearDynamicGeometry();
		}
		else
		{
			m_GeometryQuery->ClearStaticGeometry();
		}
	}

	void		PhysicsWorld::SimulateST(float dt)
	{
		for (size_t i = 0; i < m_Kinematics.size(); ++i)
		{
			m_Kinematics[i]->Simulate(dt);
		}

		std::vector<Geometry*> geoms;
		geoms.reserve(m_StaticBodies.size() + m_DynamicBodies.size());
		for (size_t i = 0; i < m_StaticBodies.size(); ++i)
		{
			for (Geometry* geom : m_StaticBodies[i]->Geometries())
			{
				if (geom && geom->IsSimulationEnabled())
				{
					geoms.push_back(geom);
				}
			}
		}
		for (size_t i = 0; i < m_DynamicBodies.size(); ++i)
		{
			for (Geometry* geom : m_DynamicBodies[i]->Geometries())
			{
				if (geom && geom->IsSimulationEnabled())
				{
					geoms.push_back(geom);
				}
			}
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
			m_Solver->PreResolve(geoms);

			const bool BuildIslands = true;
			if (BuildIslands)
			{
				ContactManifoldIslands manifold_islands;
				manifold_islands.BuildIslands(geoms, manifolds);
				manifolds.clear();

				for (size_t i = 0; i < manifold_islands.islands.size(); ++i)
				{
					m_Solver->ResolveContact(geoms, manifold_islands.islands[i], dt);
				}
			}
			else
			{
				m_Solver->ResolveContact(geoms, manifolds, dt);
			}

			m_Solver->PostResolve(geoms);
		}

		PreIntegrate(dt);

		MotionIntegration::Integrate(m_DynamicBodies, dt, (uint8_t)m_IntegrateMethod);

		PostIntegrate(dt);

		return;
	}

	void		PhysicsWorld::PreIntegrate(float dt)
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

	void		PhysicsWorld::PostIntegrate(float dt)
	{
		(void)dt;
		for (size_t i = 0; i < m_DynamicBodies.size(); ++i)
		{
			RigidBodyDynamic* Body = m_DynamicBodies[i];
			Body->ExtForce.SetZero();
			Body->ExtTorque.SetZero();
			if (Body->Sleeping)
				continue;
			Body->AutoSleep();
			UpdateSceneQuery(Body);
		}
	}

	void PhysicsWorld::BuildSceneQuery(const std::vector<Geometry*>& Objects)
	{
		if (m_SceneQuery == SceneQueryType::DynamicAABB)
		{
			m_GeometryQuery->BuildDynamicGeometry(Objects);
			return;
		}

		m_GeometryQuery->BuildStaticGeometry(Objects, 5);
	}

	void PhysicsWorld::AddGeometryToSceneQuery(Geometry* Object)
	{
		m_GeometryQuery->AddGeometry(Object);
	}

	void PhysicsWorld::RemoveGeometryFromSceneQuery(Geometry* Object)
	{
		m_GeometryQuery->RemoveGeometry(Object);
	}

	void PhysicsWorld::UpdateSceneQuery(RigidBodyDynamic* Body)
	{
		if (Body == nullptr)
		{
			return;
		}

		for (Geometry* g : Body->Geometries())
		{
			m_GeometryQuery->UpdateGeometry(g, Body->GetLinearVelocity());
		}
	}

	bool         PhysicsWorld::LoadScene(const char* name, bool shared_mem)
	{
		std::vector<RigidBody*> collection;
		std::vector<Geometry*> geoms;
		if (shared_mem)
		{
			m_SceneResource = Riemann::LoadPhysxBinaryMmap(name, nullptr, &geoms);
			if (!m_SceneResource)
			{
				return false;
			}
		}
		else
		{
			bool load_succ = Riemann::LoadPhysxBinary(name, nullptr, &geoms);
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
		BuildSceneQuery(geoms);
		return true;
	}

	RigidBody* PhysicsWorld::CreateRigidBody(const RigidBodyParam& param, const Transform& init_pose)
	{
		RigidBody* body = RigidBody::CreateRigidBody(param, init_pose);
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

	RigidBody* PhysicsWorld::CreateRigidBody(Geometry* Geom, const RigidBodyParam& param)
	{
		Transform init_pose(Geom->GetWorldPosition(), Geom->GetWorldRotation());
		RigidBody* body = CreateRigidBody(param, init_pose);
		body->AddGeometry(Geom);

		AddGeometryToSceneQuery(Geom);

		return body;
	}

	RigidBody* PhysicsWorld::CreateRigidBody(const std::vector<Geometry*>& Geoms, const RigidBodyParam& param, const Transform& init_transform)
	{
		if (Geoms.empty())
		{
			return nullptr;
		}

		RigidBody* body = CreateRigidBody(param, init_transform);
		if (!body)
		{
			return nullptr;
		}

		for (Geometry* Geom : Geoms)
		{
			if (!Geom)
			{
				continue;
			}

			const Transform world_transform = *Geom->GetWorldTransform();
			const Transform local_transform = init_transform.TransformInv(world_transform);
			Geom->SetLocalTransform(local_transform.pos, local_transform.quat);
			body->AddGeometry(Geom);

			AddGeometryToSceneQuery(Geom);
		}

		return body;
	}

	bool PhysicsWorld::RemoveRigidBody(RigidBody* Body)
	{
		if (Body->mRigidType == RigidType::Static)
		{
			for (size_t i = 0; i < m_StaticBodies.size(); ++i)
			{
				if (m_StaticBodies[i] == Body)
				{
					m_StaticBodies.erase(m_StaticBodies.begin() + i);
					for (Geometry* g : Body->Geometries())
					{
						RemoveGeometryFromSceneQuery(g);
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
					for (Geometry* g : Body->Geometries())
					{
						RemoveGeometryFromSceneQuery(g);
					}
					return true;
				}
			}
		}
		return false;
	}

	bool PhysicsWorld::LoadAnimation(const std::string& resname, const std::string& filepath, float play_rate, bool begin_play)
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

	KinematicsDriver* PhysicsWorld::FindKinematics(const std::string& resname)
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

	void PhysicsWorld::GetAllGeometries(std::vector<Geometry*>* geoms)
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

	float PhysicsWorld::GetSystemTotalEnergy() const
	{
		float Energy = 0.0f;
		for (size_t i = 0; i < m_DynamicBodies.size(); ++i)
		{
			Energy += m_DynamicBodies[i]->GetKinematicsEnergy();
		}
		return Energy;
	}

	float PhysicsWorld::GetSystemTotalLinearKinematicsEnergy() const
	{
		float Energy = 0.0f;
		for (size_t i = 0; i < m_DynamicBodies.size(); ++i)
		{
			Energy += m_DynamicBodies[i]->GetLinearKinematicsEnergy();
		}
		return Energy;
	}

	float PhysicsWorld::GetSystemTotalAngularKinematicsEnergy() const
	{
		float Energy = 0.0f;
		for (size_t i = 0; i < m_DynamicBodies.size(); ++i)
		{
			Energy += m_DynamicBodies[i]->GetAngularKinematicsEnergy();
		}
		return Energy;
	}

	Vector3 PhysicsWorld::GetSystemTotalLinearMomentum() const
	{
		Vector3 Momentum(0.0f);
		for (size_t i = 0; i < m_DynamicBodies.size(); ++i)
		{
			Momentum += m_DynamicBodies[i]->GetLinearMomentum();
		}
		return Momentum;
	}

	Vector3 PhysicsWorld::GetSystemTotalAngularMomentum() const
	{
		Vector3 Momentum(0.0f);
		for (size_t i = 0; i < m_DynamicBodies.size(); ++i)
		{
			Momentum += m_DynamicBodies[i]->GetAngularMomentum();
		}
		return Momentum;
	}
}
