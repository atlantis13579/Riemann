
#include "Viewer.h"

#include "../Src/Collision/GeometryQuery.h"
#include "../Src/Collision/GeometryObject.h"
#include "../Src/CollisionPrimitive/Mesh.h"
#include "../Src/Geometry/VoxelField.h"
#include "../Src/RigidBodyDynamics/RigidBodySimulation.h"
#include "../Src/RigidBodyDynamics/KinematicsTree.h"
#include "../Renderer/Renderer.h"
#include "ObjRenderer.h"
#include "../Src/Tools/PhysxBinaryParser.h"

WorldViewer::WorldViewer(Renderer* renderer)
{
	m_CamCenter = Vector3d::Zero();
	m_CamParam = Vector3d(1.0f, 1.0f, 10.0f);
	m_Renderer = renderer;
	CreateSimulator();
	UpdateCamera();
	CreateBoxesDemo();
}

WorldViewer::~WorldViewer()
{
	delete m_World;
}

void WorldViewer::CreateSimulator()
{
	RigidBodySimulationParam param;
	param.Gravity = Vector3d(0, -9.8f, 0);
	m_World = new RigidBodySimulation(param);
}

void WorldViewer::UpdateSimulator(float dt)
{
	m_World->Simulate(dt);
}

void WorldViewer::LoadAnimation(const std::string& anim_name, const std::vector<std::string>& nodes)
{
	RigidBodyParam rp;
	rp.Static = true;

	m_World->LoadAnimation(anim_name, anim_name, 10.0f, true);

	KinematicsTree* tree = m_World->FindKinematics(anim_name);
	tree->SetRootTransform(Vector3d(0, -10, 0), Quaternion::One());

	for (size_t i = 1; i < nodes.size(); ++i)
	{
		Geometry* aabb = GeometryFactory::CreateOBB(Vector3d(0.0f, (float)i, 0.0f), Vector3d(1.0f, 3.0f, 1.0f));
		RigidBodyStatic* p = (RigidBodyStatic*)m_World->CreateRigidBody(aabb, rp);
		
		const std::string& name = nodes[i];

		m_World->BindKinematicsNode(anim_name, name, p);
		AddGeometry(m_Renderer, aabb);
	}
}

void WorldViewer::LoadPhysxScene(const std::string& file_name)
{
	std::vector<Geometry*> collection;;
	LoadPhysxBinary(file_name.c_str(), &collection);

	for (Geometry* Geom : collection)
	{
		AddGeometry(m_Renderer, Geom);
	}

	m_World->GetGeometryQuery()->BuildStaticGeometry(collection, 5);
}

void WorldViewer::LoadFlatObj(const std::string& file_name)
{
	Mesh mesh;
	mesh.LoadFlat(file_name.c_str());
	Transform* t = new Transform;
	t->SetScale(Vector3d(0.01f, 0.01f, 0.01f));
	AddTriMesh(m_Renderer , &mesh, t, false);
}

void WorldViewer::LoadVoxelField(const std::string& file_name, const Vector3d &c, std::vector<Vector3d>& water_list)
{
	VoxelField field;

	field.SerializeFrom(file_name.c_str());

	int idx = field.WorldSpaceToVoxelIndex(c);
	int cz = idx / field.GetSizeX();
	int cx = idx - field.GetSizeX() * cz;
	Voxel* v = field.GetVoxel(idx);

	field.MakeComplementarySet();

	Vector3d pos_main = field.GetBoundingVolume().GetCenter();
	pos_main.y = field.GetBoundingVolume().Max.y - 5.0f;

	const Vector3d Thr = Vector3d(0, 0.5f, 0);

	uint64_t vol = field.Separate(pos_main, 1, 0.1f);
	for (size_t i = 0; i < water_list.size(); ++i)
	{
		vol = field.Separate(water_list[i] + Thr, (int)i + 2, 0.5f);
	}

	int Count = field.Filter(
		[&](Voxel* v) -> bool {
			return v->data == 0;
		});

	field.MakeComplementarySet();

	std::vector<int> data;
	field.IntersectYPlane(18.0f, data, 2.0f);

	Mesh* draw_mesh = field.CreateDebugMesh(cx - 100, cx + 100, cz - 100, cz + 100);

	float water_level = 19.9f;
	float water_level2 = water_level + 1.0f;

	int idx_water_level = field.WorldSpaceToVoxelSpaceY(water_level);
	int idx_water_level2 = field.WorldSpaceToVoxelSpaceY(water_level);

	Vector3d pmin = Vector3d(c.x - 200.0f, water_level, c.z - 200.0f);
	Vector3d pmax = Vector3d(c.x + 200.0f, water_level, c.z + 200.0f);

	Transform* t = new Transform;
	t->SetScale(Vector3d(0.02f, 0.02f, 0.02f));

	AddTriMesh(m_Renderer, draw_mesh, t, false);
}

void WorldViewer::CreateBoxesDemo()
{
	RigidBodyParam rp;
	rp.Static = true;
	Geometry* plane = GeometryFactory::CreatePlane(Vector3d(0, -5.0f, 0), Vector3d::UnitY(), 1.0f);
	// Geometry* plane = GeometryFactory::CreateOBB(Vector3d(0.0f, 0.0f, 0.0f), Vector3d(100.0f, 2.1f, 100.0f));
	m_World->CreateRigidBody(plane, rp);
	AddGeometry(m_Renderer, plane);

	rp.Static = false;
	rp.LinearDamping = 0.99f;
	rp.AngularDamping = 0.99f;
	for (int i = 0; i < 10; ++i)
	for (int j = 0; j < 5; ++j)
	for (int k = 0; k < 5; ++k)
	{
		Geometry* aabb = GeometryFactory::CreateOBB(Vector3d(j * 2.1f, 10.0f + i * 5.0f, k * 2.1f), Vector3d(1.0f, 1.0f, 1.0f));
		// Geometry* aabb = GeometryFactory::CreateSphere(Vector3d(j * 3.0f, 10.0f + i * 3.0f, k * 3.0f), 1.0f);
		RigidBodyDynamic* p = (RigidBodyDynamic*)m_World->CreateRigidBody(aabb, rp);
		// p->ApplyTorgue(Vector3d(0, -50, 0).Cross(Vector3d::UnitZ()) * aabb->GetBoundingVolume_WorldSpace().GetLengthZ());
		AddGeometry(m_Renderer, aabb);
	}
}

Vector3d WorldViewer::GetCameraPosition()
{
	return m_CamCenter + Vector3d(sinf(m_CamParam.x) * cosf(m_CamParam.y), sinf(m_CamParam.y), cosf(m_CamParam.x) * cosf(m_CamParam.y)) * m_CamParam.z;
}

void WorldViewer::UpdateCamera()
{
	Vector3d Eye = GetCameraPosition();
	Vector3d Center = m_CamCenter;
	m_Renderer->SetCameraLookAt(Eye, Center);
}

void WorldViewer::KeyboardMsg(char c)
{
	float Scale = 5.0f;
	Vector3d Dir = m_CamCenter - GetCameraPosition();
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

