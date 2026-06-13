#include "Viewer.h"

#include <math.h>
#include <stdio.h>

#include <algorithm>
#include <fstream>
#include <memory>
#include <sstream>

#include "ObjRenderer.h"
#include "RenderThread.h"
#include "SceneWorld.h"
#include "../Renderer/Renderer.h"
#include "../Src/Collision/GeometryObject.h"
#include "../Src/Collision/GeometryQuery.h"
#include "../Src/Modules/Tools/PhysxBinaryParser.h"
#include "../Src/RigidBodyDynamics/KinematicsTree.h"
#include "../Src/RigidBodyDynamics/RigidBodySimulation.h"

namespace Riemann
{
	namespace
	{
		bool FileExists(const std::string& fileName)
		{
			std::ifstream file(fileName.c_str(), std::ios::in | std::ios::binary);
			return !!file;
		}

		std::string DirectoryOf(const std::string& fileName)
		{
			const size_t pos = fileName.find_last_of("/\\");
			if (pos == std::string::npos)
			{
				return std::string();
			}
			return fileName.substr(0, pos + 1);
		}

		Vector3 SafeUnitVector(const Vector3& value, const Vector3& fallback)
		{
			const float lenSq = value.SquareLength();
			if (lenSq <= 1e-8f)
			{
				return fallback;
			}
			return value / sqrtf(lenSq);
		}
	}

	WorldViewer::WorldViewer(RenderThread* renderThread, const std::string& sceneFile)
		: m_World(new SceneWorld())
		, m_RenderThread(renderThread)
		, m_CamCenter(Vector3::Zero())
		, m_CamParam(Vector3(1.0f, 0.6f, 15.0f))
	{
		if (!LoadScene(sceneFile))
		{
			LoadScene(ResolveSceneFile("demo.json"));
		}

		m_KeyboardEvent = [this](char c)
		{
			if (c != 'i')
			{
				return;
			}

			static int projectileId = 0;
			const Vector3 p0 = GetCameraPosition();
			const Vector3 direction = SafeUnitVector(m_CamCenter - p0, Vector3::UnitZ());

			std::ostringstream ss;
			ss << "projectile_" << projectileId++;
			Geometry* geometry = m_World->AddProjectileSphere(ss.str(), p0, 1.0f, direction * 100.0f, Vector4(0.95f, 0.58f, 0.20f, 1.0f));
			if (geometry)
			{
				const SceneObjectInstance& instance = m_World->GetObjects().back();
				AddGeometryToRender(instance.Id, geometry, instance.Color, instance.RenderBounds);
			}
		};
	}

	WorldViewer::~WorldViewer()
	{
		if (m_RenderThread)
		{
			m_RenderThread->Submit([](Renderer& renderer) {
				renderer.Reset();
			});
		}
	}

	bool WorldViewer::LoadScene(const std::string& sceneFile)
	{
		const std::string resolved = ResolveSceneFile(sceneFile.c_str());
		if (resolved.empty())
		{
			return false;
		}

		std::string error;
		if (!m_World->LoadFromFile(resolved, &error))
		{
			fprintf(stderr, "%s\n", error.c_str());
			return false;
		}

		m_SceneDirectory = DirectoryOf(resolved);
		ApplySceneCamera();
		RebuildRenderScene();
		return true;
	}

	void WorldViewer::CreateSimulator()
	{
		m_World->Reset();
		m_RenderBindings.clear();
		RebuildRenderScene();
	}

	void WorldViewer::UpdateSimulator(float dt)
	{
		m_World->Step(dt);
		SubmitTransforms();
	}

	void WorldViewer::LoadAnimation(const std::string& animName, const std::vector<std::string>& nodes)
	{
		RigidBodySimulation* simulation = m_World->GetSimulation();
		if (simulation == nullptr)
		{
			return;
		}

		RigidBodyParam rp;
		rp.rigidType = RigidType::Kinematic;

		simulation->LoadAnimation(animName, animName, 10.0f, true);

		KinematicsTree* tree = static_cast<KinematicsTree*>(simulation->FindKinematics(animName));
		if (tree == nullptr)
		{
			return;
		}
		tree->SetRootTransform(Vector3(0, -10, 0), Quaternion::One());

		for (size_t i = 1; i < nodes.size(); ++i)
		{
			Geometry* aabb = GeometryFactory::CreateOBB(Vector3(0.0f, static_cast<float>(i), 0.0f), Vector3(1.0f, 3.0f, 1.0f));
			tree->BindGeometry(nodes[i], aabb);

			std::ostringstream ss;
			ss << "anim_" << animName << "_" << i;
			AddGeometryToRender(ss.str(), aabb, Vector4(0.55f, 0.72f, 0.95f, 1.0f), false);
		}
	}

	void WorldViewer::LoadPhysxScene(const std::string& fileName)
	{
		RigidBodySimulation* simulation = m_World->GetSimulation();
		if (simulation == nullptr)
		{
			return;
		}

		std::vector<Geometry*> geometries;
		LoadPhysxBinary(fileName.c_str(), nullptr, &geometries);

		for (size_t i = 0; i < geometries.size(); ++i)
		{
			std::ostringstream ss;
			ss << "physx_" << i;
			AddGeometryToRender(ss.str(), geometries[i], Vector4(0.65f, 0.68f, 0.72f, 1.0f), false);
		}

		simulation->GetGeometryQuery()->BuildStaticGeometry(geometries, 5);
	}

	void WorldViewer::LoadVoxelField(const std::string& fileName, const Vector3& c, std::vector<Vector3>& waterList)
	{
		(void)fileName;
		(void)c;
		waterList.clear();
	}

	void WorldViewer::CreateDemo()
	{
		LoadScene(ResolveSceneFile("demo.json"));
	}

	void WorldViewer::CreateStackBoxesDemo()
	{
		LoadScene(ResolveSceneFile("stack_boxes.json"));
	}

	void WorldViewer::CreateDominoDemo()
	{
		LoadScene(ResolveSceneFile("domino.json"));
	}

	void WorldViewer::CreateSeeSawDemo()
	{
		LoadScene(ResolveSceneFile("see_saw.json"));
	}

	Vector3 WorldViewer::GetCameraPosition() const
	{
		return m_CamCenter + Vector3(sinf(m_CamParam.x) * cosf(m_CamParam.y), sinf(m_CamParam.y), cosf(m_CamParam.x) * cosf(m_CamParam.y)) * m_CamParam.z;
	}

	void WorldViewer::UpdateCamera()
	{
		if (m_RenderThread == nullptr)
		{
			return;
		}

		CameraDesc camera = m_World->GetCamera();
		camera.Eye = GetCameraPosition();
		camera.At = m_CamCenter;
		m_RenderThread->Submit([camera](Renderer& renderer) {
			renderer.SetCamera(camera);
		});
	}

	void WorldViewer::KeyboardMsg(char c)
	{
		if ('1' <= c && c <= '9')
		{
			typedef void (WorldViewer::*DemoFunc)();
			DemoFunc demos[] = {
				&WorldViewer::CreateDemo,
				&WorldViewer::CreateStackBoxesDemo,
				&WorldViewer::CreateDominoDemo,
				&WorldViewer::CreateSeeSawDemo,
			};

			const int idx = c - '1';
			if (idx >= 0 && idx < static_cast<int>(sizeof(demos) / sizeof(demos[0])))
			{
				(this->*demos[idx])();
				return;
			}
		}

		const float scale = 5.0f;
		Vector3 dir = m_CamCenter - GetCameraPosition();
		dir.y = 0.0f;
		dir = SafeUnitVector(dir, Vector3::UnitZ()) * scale;

		if (c == 'a')
		{
			m_CamCenter.x -= dir.z;
			m_CamCenter.z += dir.x;
		}
		else if (c == 'd')
		{
			m_CamCenter.x += dir.z;
			m_CamCenter.z -= dir.x;
		}
		else if (c == 'w')
		{
			m_CamCenter.x += dir.x;
			m_CamCenter.z += dir.z;
		}
		else if (c == 's')
		{
			m_CamCenter.x -= dir.x;
			m_CamCenter.z -= dir.z;
		}
		else if (c == 'q')
		{
			m_CamCenter.y -= scale;
		}
		else if (c == 'e')
		{
			m_CamCenter.y += scale;
		}
		UpdateCamera();

		if (m_KeyboardEvent)
		{
			m_KeyboardEvent(c);
		}
	}

	void WorldViewer::MouseMsg(int x, int y, bool leftButtonDown)
	{
		static int xPosPrev = 0;
		static int yPosPrev = 0;
		if (leftButtonDown)
		{
			if (xPosPrev * yPosPrev != 0)
			{
				m_CamParam.x += (x - xPosPrev) * 0.01f;
				m_CamParam.y += (y - yPosPrev) * 0.01f;
				if (m_CamParam.y > 1.5f)
				{
					m_CamParam.y = 1.5f;
				}
				if (m_CamParam.y < -1.5f)
				{
					m_CamParam.y = -1.5f;
				}
				UpdateCamera();
			}

			xPosPrev = x;
			yPosPrev = y;
		}
		else
		{
			xPosPrev = 0;
			yPosPrev = 0;
		}
	}

	void WorldViewer::MouseWheel(int zDelta, bool ctrlButtonDown)
	{
		float scale = ctrlButtonDown ? 10.0f : 1.0f;
		scale *= 1.5f;
		if (zDelta > 0)
		{
			scale = 1.0f / scale;
		}
		m_CamParam.z *= scale;
		UpdateCamera();
	}

	void WorldViewer::AddToRender()
	{
		RebuildRenderScene();
	}

	void WorldViewer::RebuildRenderScene()
	{
		m_RenderBindings.clear();

		std::vector<RenderMeshDesc> meshes;
		for (const SceneObjectInstance& object : m_World->GetObjects())
		{
			const size_t begin = meshes.size();
			BuildGeometryMeshes(object.GeometryPtr, object.Id, object.Color, object.RenderBounds, &meshes);

			RenderBinding binding;
			binding.GeometryPtr = object.GeometryPtr;
			for (size_t i = begin; i < meshes.size(); ++i)
			{
				binding.MeshIds.push_back(meshes[i].Id);
			}
			m_RenderBindings.push_back(binding);
		}

		if (m_RenderThread)
		{
			CameraDesc camera = m_World->GetCamera();
			DirectionalLightDesc light = m_World->GetLight();
			std::shared_ptr<std::vector<RenderMeshDesc> > renderMeshes(new std::vector<RenderMeshDesc>(std::move(meshes)));
			m_RenderThread->Submit([renderMeshes, camera, light](Renderer& renderer) {
				renderer.Reset();
				renderer.SetCamera(camera);
				renderer.SetLight(light);
				for (const RenderMeshDesc& mesh : *renderMeshes)
				{
					renderer.AddMesh(mesh);
				}
			});
		}
	}

	void WorldViewer::AddGeometryToRender(const std::string& id, Geometry* geometry, const Vector4& color, bool renderBounds)
	{
		std::vector<RenderMeshDesc> meshes;
		BuildGeometryMeshes(geometry, id, color, renderBounds, &meshes);

		RenderBinding binding;
		binding.GeometryPtr = geometry;
		for (const RenderMeshDesc& mesh : meshes)
		{
			binding.MeshIds.push_back(mesh.Id);
		}
		m_RenderBindings.push_back(binding);

		if (m_RenderThread)
		{
			std::shared_ptr<std::vector<RenderMeshDesc> > renderMeshes(new std::vector<RenderMeshDesc>(std::move(meshes)));
			m_RenderThread->Submit([renderMeshes](Renderer& renderer) {
				for (const RenderMeshDesc& mesh : *renderMeshes)
				{
					renderer.AddMesh(mesh);
				}
			});
		}
	}

	void WorldViewer::SubmitTransforms()
	{
		std::vector<RenderTransformUpdate> updates;
		for (const RenderBinding& binding : m_RenderBindings)
		{
			if (binding.GeometryPtr == nullptr)
			{
				continue;
			}

			RenderTransformUpdate update;
			update.WorldTransform = *binding.GeometryPtr->GetWorldTransform();
			for (const std::string& meshId : binding.MeshIds)
			{
				update.Id = meshId;
				updates.push_back(update);
			}
		}

		if (updates.empty() || m_RenderThread == nullptr)
		{
			return;
		}

		std::shared_ptr<std::vector<RenderTransformUpdate> > renderUpdates(new std::vector<RenderTransformUpdate>(std::move(updates)));
		m_RenderThread->Submit([renderUpdates](Renderer& renderer) {
			for (const RenderTransformUpdate& update : *renderUpdates)
			{
				renderer.UpdateTransform(update);
			}
		});
	}

	std::string WorldViewer::ResolveSceneFile(const char* fileName) const
	{
		if (fileName == nullptr || fileName[0] == '\0')
		{
			fileName = "demo.json";
		}

		const std::string requested(fileName);
		if (FileExists(requested))
		{
			return requested;
		}

		std::vector<std::string> candidates;
		if (!m_SceneDirectory.empty())
		{
			candidates.push_back(m_SceneDirectory + requested);
		}
		candidates.push_back("RiemannViewer/Scenes/" + requested);
		candidates.push_back("../RiemannViewer/Scenes/" + requested);
		candidates.push_back("../../RiemannViewer/Scenes/" + requested);
		candidates.push_back("../../../RiemannViewer/Scenes/" + requested);
		candidates.push_back("../../../../RiemannViewer/Scenes/" + requested);

		for (const std::string& candidate : candidates)
		{
			if (FileExists(candidate))
			{
				return candidate;
			}
		}

		return requested;
	}

	void WorldViewer::ApplySceneCamera()
	{
		const CameraDesc& camera = m_World->GetCamera();
		m_CamCenter = camera.At;

		const Vector3 offset = camera.Eye - camera.At;
		const float radius = std::max(0.001f, offset.Length());
		m_CamParam.x = atan2f(offset.x, offset.z);
		m_CamParam.y = asinf(std::max(-1.0f, std::min(1.0f, offset.y / radius)));
		m_CamParam.z = radius;

		UpdateCamera();

		if (m_RenderThread)
		{
			DirectionalLightDesc light = m_World->GetLight();
			m_RenderThread->Submit([light](Renderer& renderer) {
				renderer.SetLight(light);
			});
		}
	}
}
