
#include "Viewer.h"

#include "../Src/Collision/GeometryQuery.h"
#include "../Src/Collision/GeometryObject.h"
#include "../Src/CollisionPrimitive/Mesh.h"
#include "../Src/Geometry/VoxelField.h"
#include "../Src/RigidBodyDynamics/PhysicsMaterial.h"
#include "../Src/RigidBodyDynamics/RigidBodySimulation.h"
#include "../Src/RigidBodyDynamics/KinematicsTree.h"
#include "../Renderer/Renderer.h"
#include "../Src/Modules/Tools/PhysxBinaryParser.h"
#include "ObjRenderer.h"

namespace Riemann
{
	WorldViewer::WorldViewer(Renderer* renderer)
	{
		m_CamCenter = Vector3::Zero();
		m_CamParam = Vector3(1.0f, 0.6f, 15.0f);
		m_Renderer = renderer;
		CreateSimulator();
		UpdateCamera();
		CreateDemo();
	}

	WorldViewer::~WorldViewer()
	{
		delete m_World;
	}

	void WorldViewer::CreateDemo()
	{
		RigidBodyParam rp;
		rp.rigidType = RigidType::Static;
		GeometryBase* plane = GeometryFactory::CreatePlane(Vector3(0, -5.0f, 0), Vector3(0.0f, 1.0f, 0.0f));
		m_World->CreateRigidBody(plane, rp);

		rp.rigidType = RigidType::Dynamic;
		for (int i = 0; i < 5; ++i)
			for (int j = 0; j < 1; ++j)
				for (int k = 0; k < 1; ++k)
				{
					GeometryBase* aabb = GeometryFactory::CreateOBB(Vector3(j * 2.1f, 0.01f - 4.0f + i * 2.01f, k * 2.1f), Vector3(1.0f, 1.0f, 1.0f));
					RigidBodyDynamic* p = m_World->CreateRigidBody(aabb, rp)->CastDynamic();
				}

		AddToRender();

		m_KeyboardEvent = [=](char c)
		{
			if (c == 'i')
			{
				RigidBodyParam rp;
				rp.rigidType = RigidType::Dynamic;
				rp.invMass = 0.1f;
				Vector3 p0 = GetCameraPosition();
				RigidBodyDynamic* body = m_World->CreateRigidBody(rp, Pose(p0))->CastDynamic();

				{
					GeometryBase* sphere = GeometryFactory::CreateSphere(Vector3::Zero(), 1.0f);
					sphere->SetLocalPosition(Vector3::Zero());
					body->AddGeometry(sphere);
					AddGeometry(m_Renderer, sphere);
				}

				body->DisableGravity = true;
				body->ApplyLinearAcceleration((m_CamCenter - p0).Unit() * 100.0f);
			}
		};
	}

	void WorldViewer::CreateStackBoxesDemo()
	{
		m_CamParam = Vector3(1.0f, 0.5f, 25.0f);
		UpdateCamera();

		RigidBodyParam rp;
		rp.rigidType = RigidType::Static;
		GeometryBase* plane = GeometryFactory::CreatePlane(Vector3(0, -4.0f, 0), Vector3::UnitY());
		m_World->CreateRigidBody(plane, rp);

		rp.rigidType = RigidType::Dynamic;
		for (int i = 0; i < 10; ++i)
			for (int j = 0; j < 10 - i; ++j)
			{
				GeometryBase* aabb = GeometryFactory::CreateOBB(Vector3(0.0f, i * 3.0f, -10.0f + (j + i * 0.5f) * 2.1f), Vector3::One());
				m_World->CreateRigidBody(aabb, rp);
			}

		AddToRender();
	}

	void WorldViewer::CreateDominoDemo()
	{
		m_CamParam = Vector3(1.6f, 0.6f, 23.0f);
		UpdateCamera();

		RigidBodyParam rp;
		rp.rigidType = RigidType::Static;
		GeometryBase* plane = GeometryFactory::CreatePlane(Vector3(0, 0, 0), Vector3::UnitY());
		m_World->CreateRigidBody(plane, rp);

		{
			Quaternion q;
			q.FromRotationAxis(Vector3::UnitX(), 0.4f);
			GeometryBase* aabb = GeometryFactory::CreateOBB(Vector3(0.0f, 2.5f, -18.0f), Vector3(2.0f, 2.0f, 6.0f), q);
			m_World->CreateRigidBody(aabb, rp)->SetDefaultPhysicsMaterial(DefaultPhysicsMaterial::Ice);

			rp.rigidType = RigidType::Dynamic;
			GeometryBase* sp = GeometryFactory::CreateSphere(Vector3(0.0f, 20.0f, -20.0f), 1.0f);
			m_World->CreateRigidBody(sp, rp)->SetDefaultPhysicsMaterial(DefaultPhysicsMaterial::Ice);
		}

		for (int i = 0; i < 10; ++i)
		{
			GeometryBase* aabb = GeometryFactory::CreateOBB(Vector3(0.0f, 3.0f, -10.0f + i * 2.0f), Vector3(2.0f, 3.0f, 0.25f));
			m_World->CreateRigidBody(aabb, rp);
		}

		for (int i = 0; i < 7; ++i)
		{
			Quaternion q;
			q.FromRotationAxis(Vector3::UnitY(), i * 30 * (3.14f / 180));
			GeometryBase* aabb = GeometryFactory::CreateOBB(Vector3(2.0f * i, 3.0f, 10.0f + 1.5f * (i < 4 ? i : 6 - i)), Vector3(2.0f, 3.0f, 0.25f), q);
			m_World->CreateRigidBody(aabb, rp);
		}

		for (int i = 0; i < 10; ++i)
		{
			GeometryBase* aabb = GeometryFactory::CreateOBB(Vector3(12.0f, 3.0f, 10.0f - (i + 1) * 2.0f), Vector3(2.0f, 3.0f, 0.25f));
			m_World->CreateRigidBody(aabb, rp);
		}

		AddToRender();
	}

	void WorldViewer::CreateSeeSawDemo()
	{
		RigidBodyParam rp;
		rp.rigidType = RigidType::Static;
		GeometryBase* box = GeometryFactory::CreatePlane(Vector3(0, 0, 0), Vector3::UnitY());
		m_World->CreateRigidBody(box, rp);

		box = GeometryFactory::CreateOBB(Vector3(3, 0, 0), Vector3(1, 1, 1), Quaternion().FromRotateZ(0.7f));
		m_World->CreateRigidBody(box, rp);

		rp.rigidType = RigidType::Dynamic;
		box = GeometryFactory::CreateOBB(Vector3(2, 1.7f, 0), Vector3(5, 0.25f, 1));
		m_World->CreateRigidBody(box, rp);

		box = GeometryFactory::CreateSphere(Vector3(-1, 4, 0), 0.5f);
		m_World->CreateRigidBody(box, rp);
		rp.invMass = 0.5f;
		box = GeometryFactory::CreateOBB(Vector3(-2, 4, 0), Vector3(0.5f, 0.5f, 0.5f));
		m_World->CreateRigidBody(box, rp);

		rp.invMass = 0.01f;
		box = GeometryFactory::CreateCylinder(Vector3(7, 10, -1), Vector3(7, 10, 3), 0.5f);
		// box = GeometryFactory::CreateCapsule(Vector3(7, 10, -1), Vector3(7, 10, 3), 0.5f);
		m_World->CreateRigidBody(box, rp);

		AddToRender();
		return;
	}

	void WorldViewer::LoadAnimation(const std::string& anim_name, const std::vector<std::string>& nodes)
	{
		RigidBodyParam rp;
		rp.rigidType = RigidType::Kinematic;

		m_World->LoadAnimation(anim_name, anim_name, 10.0f, true);

		KinematicsTree* tree = static_cast<KinematicsTree*>(m_World->FindKinematics(anim_name));
		tree->SetRootTransform(Vector3(0, -10, 0), Quaternion::One());

		for (size_t i = 1; i < nodes.size(); ++i)
		{
			GeometryBase* aabb = GeometryFactory::CreateOBB(Vector3(0.0f, (float)i, 0.0f), Vector3(1.0f, 3.0f, 1.0f));

			const std::string& name = nodes[i];

			tree->BindGeometry(name, aabb);
			AddGeometry(m_Renderer, aabb);
		}
	}

	void WorldViewer::LoadPhysxScene(const std::string& file_name)
	{
		std::vector<GeometryBase*> geometries;
		LoadPhysxBinary(file_name.c_str(), nullptr, &geometries);

		for (GeometryBase* Geom : geometries)
		{
			AddGeometry(m_Renderer, Geom);
		}

		m_World->GetGeometryQuery()->BuildStaticGeometry(geometries, 5);
	}

	void WorldViewer::CreateSimulator()
	{
		RigidBodySimulationParam param;
		param.gravityAcc = Vector3(0, -9.8f, 0);
		m_World = new RigidBodySimulation(param);
	}

	void WorldViewer::UpdateSimulator(float dt)
	{
		m_World->Simulate();
	}

	Vector3 WorldViewer::GetCameraPosition()
	{
		return m_CamCenter + Vector3(sinf(m_CamParam.x) * cosf(m_CamParam.y), sinf(m_CamParam.y), cosf(m_CamParam.x) * cosf(m_CamParam.y)) * m_CamParam.z;
	}

	void WorldViewer::UpdateCamera()
	{
		Vector3 Eye = GetCameraPosition();
		Vector3 Center = m_CamCenter;
		m_Renderer->SetCameraLookAt(Eye, Center);
	}

	void WorldViewer::KeyboardMsg(char c)
	{
		if ('1' <= c && c <= '9')
		{
			std::function<void(WorldViewer*)> demos[] = {
				&WorldViewer::CreateDemo,
				&WorldViewer::CreateStackBoxesDemo,
				&WorldViewer::CreateDominoDemo,
				&WorldViewer::CreateSeeSawDemo,
			};

			int idx = c - '1';
			if (idx < sizeof(demos) / sizeof(demos[0]))
			{
				m_Renderer->Reset();
				m_World->Reset();

				demos[idx](this);
				return;
			}
		}

		float Scale = 5.0f;
		Vector3 Dir = m_CamCenter - GetCameraPosition();
		Dir.y = 0;
		Dir.Normalize();
		Dir *= Scale;

		if (c == 'a')
		{
			m_CamCenter.x -= Dir.z;
			m_CamCenter.z += Dir.x;
		}
		else if (c == 'd')
		{
			m_CamCenter.x += Dir.z;
			m_CamCenter.z -= Dir.x;
		}
		else if (c == 'w')
		{
			m_CamCenter.x += Dir.x;
			m_CamCenter.z += Dir.z;
		}
		else if (c == 's')
		{
			m_CamCenter.x -= Dir.x;
			m_CamCenter.z -= Dir.z;
		}
		else if (c == 'q')
		{
			m_CamCenter.y -= Scale;
		}
		else if (c == 'e')
		{
			m_CamCenter.y += Scale;
		}
		UpdateCamera();

		if (m_KeyboardEvent)
		{
			m_KeyboardEvent(c);
		}
	}

	void WorldViewer::MouseMsg(int x, int y, bool LButtonDown)
	{
		static int xPosPrev = 0, yPosPrev = 0;
		if (LButtonDown)
		{
			if (xPosPrev * yPosPrev != 0)
			{
				m_CamParam.x += (x - xPosPrev) * 0.01f;
				m_CamParam.y += (y - yPosPrev) * 0.01f;
				if (m_CamParam.y > 1.5)
					m_CamParam.y = 1.5;
				if (m_CamParam.y < -1.5)
					m_CamParam.y = -1.5;
				UpdateCamera();
			}

			xPosPrev = x;
			yPosPrev = y;
		}
		else
		{
			xPosPrev = yPosPrev = 0;
		}
	}

	void WorldViewer::MouseWheel(int zDelta, bool CtrlButtonDown)
	{
		float Scale = CtrlButtonDown ? 10.0f : 1.0f;
		Scale *= 1.5f;
		if (zDelta > 0)
			Scale = 1.0f / Scale;
		m_CamParam.z *= Scale;
		UpdateCamera();
	}

	void WorldViewer::AddToRender()
	{
		std::vector<GeometryBase*> geoms;
		m_World->GetAllGeometries(&geoms);
		for (size_t i = 0; i < geoms.size(); ++i)
		{
			AddGeometry(m_Renderer, geoms[i]);
		}
	}

}