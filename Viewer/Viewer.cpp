#include "Viewer.h"

#if defined(_WIN32) && !defined(NOMINMAX)
#define NOMINMAX
#endif

#include <ctype.h>
#include <math.h>
#include <stdio.h>

#include <algorithm>
#include <fstream>
#include <memory>
#include <sstream>

#if defined(_WIN32)
#include <windows.h>
#else
#include <dirent.h>
#endif

#include "MeshRenderer.h"
#include "RenderThread.h"
#include "SceneWorld.h"
#include "../Renderer/imgui.h"
#include "../Renderer/RiemannRenderer.h"
#include "../Src/Collision/GeometryObject.h"
#include "../Src/Collision/GeometryQuery.h"
#include "../Src/CollisionPrimitive/PrimitiveType.h"
#include "../Src/Destruction/Fracture.h"
#include "../Src/Geometry/DynamicMesh.h"
#include "../Src/Modules/Tools/PhysxBinaryParser.h"
#include "../Src/RigidBodyDynamics/KinematicsTree.h"
#include "../Src/RigidBodyDynamics/PhysicsWorld.h"

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

		std::string FileNameOf(const std::string& fileName)
		{
			const size_t pos = fileName.find_last_of("/\\");
			if (pos == std::string::npos)
			{
				return fileName;
			}
			return fileName.substr(pos + 1);
		}

		std::string EnsureTrailingSlash(const std::string& directory)
		{
			if (directory.empty())
			{
				return directory;
			}

			const char last = directory[directory.size() - 1];
			if (last == '/' || last == '\\')
			{
				return directory;
			}
			return directory + "/";
		}

		std::string JoinPath(const std::string& directory, const std::string& fileName)
		{
			if (directory.empty())
			{
				return fileName;
			}
			return EnsureTrailingSlash(directory) + fileName;
		}

		bool DirectoryExists(const std::string& directory)
		{
#if defined(_WIN32)
			const DWORD attributes = GetFileAttributesA(directory.c_str());
			return attributes != INVALID_FILE_ATTRIBUTES && (attributes & FILE_ATTRIBUTE_DIRECTORY) != 0;
#else
			DIR* dir = opendir(directory.c_str());
			if (dir == nullptr)
			{
				return false;
			}
			closedir(dir);
			return true;
#endif
		}

		bool HasJsonExtension(const std::string& fileName)
		{
			if (fileName.size() < 5)
			{
				return false;
			}

			const size_t offset = fileName.size() - 5;
			return fileName[offset] == '.' &&
				tolower(static_cast<unsigned char>(fileName[offset + 1])) == 'j' &&
				tolower(static_cast<unsigned char>(fileName[offset + 2])) == 's' &&
				tolower(static_cast<unsigned char>(fileName[offset + 3])) == 'o' &&
				tolower(static_cast<unsigned char>(fileName[offset + 4])) == 'n';
		}

		std::vector<std::string> ListJsonFiles(const std::string& directory)
		{
			std::vector<std::string> files;
#if defined(_WIN32)
			const std::string searchPath = JoinPath(directory, "*.json");
			WIN32_FIND_DATAA findData;
			HANDLE findHandle = FindFirstFileA(searchPath.c_str(), &findData);
			if (findHandle != INVALID_HANDLE_VALUE)
			{
				do
				{
					if ((findData.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) == 0)
					{
						files.push_back(findData.cFileName);
					}
				} while (FindNextFileA(findHandle, &findData));
				FindClose(findHandle);
			}
#else
			DIR* dir = opendir(directory.c_str());
			if (dir != nullptr)
			{
				struct dirent* entry = nullptr;
				while ((entry = readdir(dir)) != nullptr)
				{
					if (HasJsonExtension(entry->d_name))
					{
						files.push_back(entry->d_name);
					}
				}
				closedir(dir);
			}
#endif
			std::sort(files.begin(), files.end());
			return files;
		}

		std::vector<std::string> SceneDirectoryCandidates(const std::string& currentDirectory)
		{
			std::vector<std::string> candidates;
			if (!currentDirectory.empty())
			{
				candidates.push_back(EnsureTrailingSlash(currentDirectory));
			}
			candidates.push_back("Contents/Scenes/");
			candidates.push_back("../Contents/Scenes/");
			candidates.push_back("../../Contents/Scenes/");
			candidates.push_back("../../../Contents/Scenes/");
			candidates.push_back("../../../../Contents/Scenes/");
			return candidates;
		}

		std::vector<std::string> TestDataDirectoryCandidates(const std::string& currentDirectory)
		{
			std::vector<std::string> candidates;
			if (!currentDirectory.empty())
			{
				candidates.push_back(EnsureTrailingSlash(currentDirectory));
			}
			candidates.push_back("Contents/TestData/");
			candidates.push_back("../Contents/TestData/");
			candidates.push_back("../../Contents/TestData/");
			candidates.push_back("../../../Contents/TestData/");
			candidates.push_back("../../../../Contents/TestData/");
			return candidates;
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

		float BoundsMaxDim(const Box3& bounds)
		{
			const Vector3 size = bounds.Max - bounds.Min;
			return std::max(size.x, std::max(size.y, size.z));
		}

		Vector4 PieceColor(size_t index)
		{
			static const Vector4 colors[] =
			{
				Vector4(0.92f, 0.42f, 0.36f, 1.0f),
				Vector4(0.35f, 0.67f, 0.96f, 1.0f),
				Vector4(0.44f, 0.76f, 0.46f, 1.0f),
				Vector4(0.96f, 0.72f, 0.34f, 1.0f),
				Vector4(0.70f, 0.52f, 0.92f, 1.0f),
				Vector4(0.32f, 0.82f, 0.78f, 1.0f),
				Vector4(0.92f, 0.56f, 0.82f, 1.0f),
				Vector4(0.72f, 0.78f, 0.86f, 1.0f),
			};
			return colors[index % (sizeof(colors) / sizeof(colors[0]))];
		}

		RenderMeshDesc BuildDynamicMeshRenderMesh(const DynamicMesh& mesh, const std::string& id, const Vector4& color)
		{
			RenderMeshDesc desc;
			desc.Id = id;
			desc.Color = color;
			desc.Topology = RenderPrimitiveTopology::Triangles;
			desc.CastShadow = true;

			std::vector<int> vertexMap(mesh.GetVertexCount(), -1);
			for (int tid = 0; tid < mesh.GetTriangleCount(); ++tid)
			{
				if (!mesh.IsTriangleFast(tid))
				{
					continue;
				}

				const Index3 tri = mesh.GetTriangle(tid);
				const Vector3 p0 = mesh.GetVertex(tri.a);
				const Vector3 p1 = mesh.GetVertex(tri.b);
				const Vector3 p2 = mesh.GetVertex(tri.c);
				const Vector3 faceNormal = SafeUnitVector((p1 - p0).Cross(p2 - p0), Vector3::UnitY());

				for (int corner = 0; corner < 3; ++corner)
				{
					const int sourceVertex = tri[corner];
					if (sourceVertex < 0 || sourceVertex >= mesh.GetVertexCount())
					{
						continue;
					}

					if (vertexMap[sourceVertex] < 0)
					{
						const int newIndex = (int)desc.Vertices.size();
						vertexMap[sourceVertex] = newIndex;
						const Vector3 normal = mesh.HasVertexNormals() ? mesh.GetVertexNormal(sourceVertex) : faceNormal;
						desc.Vertices.push_back(Vertex1(mesh.GetVertex(sourceVertex), SafeUnitVector(normal, faceNormal)));
					}
					desc.Indices.push_back((uint32_t)vertexMap[sourceVertex]);
				}
			}

			return desc;
		}

		bool IsHighlightableGeometry(const Geometry* geometry)
		{
			if (geometry == nullptr || geometry->GetShapeType() == PrimitiveType::PLANE)
			{
				return false;
			}

			const Box3& bounds = geometry->GetBoundingVolume_WorldSpace();
			return bounds.MaxDim() < 200.0f;
		}
	}

	WorldViewer::WorldViewer(RenderThread* renderThread, const std::string& sceneFile)
		: m_World(new SceneWorld())
		, m_RenderThread(renderThread)
		, m_HighlightedGeometry(nullptr)
		, m_CamCenter(Vector3::Zero())
		, m_CamParam(Vector3(1.0f, 0.6f, 15.0f))
		, m_BunnyFractureDemoActive(false)
		, m_FractureSeparation(0.0f)
		, m_FractureMaxSeparation(1.0f)
		, m_ShowGeometryQueryBounds(false)
		, m_ImguiDemoIndex(0)
		, m_ImguiObjectCount(0)
		, m_ImguiCameraDistance(15.0f)
		, m_ImguiBunnyFractureActive(false)
		, m_ImguiFractureSeparation(0.0f)
		, m_ImguiShowGeometryQueryBounds(false)
		, m_ImguiGeometryQueryBoundsCount(0)
		, m_ImguiPhysicsFps(0.0)
		, m_ImguiScroll(0)
		, m_ImguiPendingDemoIndex(-1)
		, m_ImguiPendingCameraDistance(false)
		, m_ImguiPendingCameraDistanceValue(15.0f)
		, m_ImguiPendingFractureSeparation(false)
		, m_ImguiPendingFractureSeparationValue(0.0f)
		, m_ImguiPendingShowGeometryQueryBounds(false)
		, m_ImguiPendingShowGeometryQueryBoundsValue(false)
		, m_PhysicsFpsTime(std::chrono::steady_clock::now())
		, m_PhysicsFrameCount(0)
		, m_PhysicsFps(0.0)
	{
		for (bool& key : m_MovementKeys)
		{
			key = false;
		}

		RefreshSceneList();
		std::string initialScene = sceneFile;
		if (initialScene.empty() && !m_SceneFiles.empty())
		{
			initialScene = m_SceneFiles.front();
		}

		if (!LoadScene(initialScene))
		{
			LoadScene(m_SceneFiles.empty() ? ResolveSceneFile("demo.json") : m_SceneFiles.front());
		}

		m_KeyboardEvent = [this](char c)
		{
			if (c != 'i')
			{
				return;
			}
			if (m_BunnyFractureDemoActive)
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
				UpdateImguiState();
			}
		};

		if (m_RenderThread)
		{
			m_RenderThread->Submit([this](Renderer& renderer) {
				renderer.SetImguiDrawCallback(&WorldViewer::DrawImguiCallback, this);
			});
		}
	}

	WorldViewer::~WorldViewer()
	{
		if (m_RenderThread)
		{
			m_RenderThread->Submit([](Renderer& renderer) {
				renderer.SetImguiDrawCallback(nullptr, nullptr);
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
		m_CurrentSceneName = FileNameOf(resolved);
		m_BunnyFractureDemoActive = false;
		m_FracturePieces.clear();
		m_FractureSeparation = 0.0f;
		RefreshSceneList();
		ApplySceneCamera();
		m_HighlightedGeometry = nullptr;
		RebuildRenderScene();
		UpdateImguiState();
		return true;
	}

	void WorldViewer::CreateSimulator()
	{
		m_World->Reset();
		m_RenderBindings.clear();
		m_HighlightedGeometry = nullptr;
		RebuildRenderScene();
	}

	void WorldViewer::UpdateSimulator(float dt)
	{
		ApplyImguiCommands();
		m_World->Step(dt);
		SubmitTransforms();
		SubmitGeometryQueryBounds();
		UpdatePhysicsFps();
		UpdateImguiState();
	}

	void WorldViewer::LoadAnimation(const std::string& animName, const std::vector<std::string>& nodes)
	{
		PhysicsWorld* simulation = m_World->GetSimulation();
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
		PhysicsWorld* simulation = m_World->GetSimulation();
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
		c = static_cast<char>(tolower(static_cast<unsigned char>(c)));
		if (c == 'a' || c == 'd' || c == 'w' || c == 's')
		{
			return;
		}

		const float scale = 5.0f;
		Vector3 dir = m_CamCenter - GetCameraPosition();
		dir.y = 0.0f;
		dir = SafeUnitVector(dir, Vector3::UnitZ()) * scale;

		if (c == 'q')
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

	void WorldViewer::SetMovementKey(char c, bool down)
	{
		c = static_cast<char>(tolower(static_cast<unsigned char>(c)));
		if (c == 'w')
		{
			m_MovementKeys[0] = down;
		}
		else if (c == 'a')
		{
			m_MovementKeys[1] = down;
		}
		else if (c == 's')
		{
			m_MovementKeys[2] = down;
		}
		else if (c == 'd')
		{
			m_MovementKeys[3] = down;
		}
	}

	void WorldViewer::UpdateCameraMovement(float dt)
	{
		Vector3 dir = m_CamCenter - GetCameraPosition();
		dir.y = 0.0f;
		dir = SafeUnitVector(dir, Vector3::UnitZ());

		Vector3 movement = Vector3::Zero();
		if (m_MovementKeys[0])
		{
			movement += dir;
		}
		if (m_MovementKeys[2])
		{
			movement -= dir;
		}
		if (m_MovementKeys[1])
		{
			movement.x -= dir.z;
			movement.z += dir.x;
		}
		if (m_MovementKeys[3])
		{
			movement.x += dir.z;
			movement.z -= dir.x;
		}

		if (movement.SquareLength() <= 1e-8f)
		{
			return;
		}

		const float cameraSpeed = 25.0f;
		movement = SafeUnitVector(movement, Vector3::Zero()) * (cameraSpeed * dt);
		m_CamCenter += movement;
		UpdateCamera();
	}

	void WorldViewer::MouseMsg(int x, int y, bool rotateButtonDown)
	{
		static int xPosPrev = 0;
		static int yPosPrev = 0;
		if (rotateButtonDown)
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

	void WorldViewer::SceneRayMsg(int x, int y, int width, int height)
	{
		if (width <= 0 || height <= 0)
		{
			return;
		}

		const Ray3 ray = BuildSceneRay(x, y, width, height);
		HandleSceneRay(ray);
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

	bool WorldViewer::IsImguiPanelHovered(int x, int y, int width, int height) const
	{
		if (width <= 0 || height <= 0)
		{
			return false;
		}

		const int panelLeft = 10;
		const int panelTop = 10;
		const int panelRight = std::min(width - 10, panelLeft + 250);
		const int panelBottom = height - 10;
		return x >= panelLeft && x <= panelRight && y >= panelTop && y <= panelBottom;
	}

	void WorldViewer::DrawImgui(int width, int height)
	{
		(void)width;

		std::string sceneName;
		std::vector<std::string> sceneFiles;
		int demoIndex = 0;
		size_t objectCount = 0;
		float cameraDistance = 0.0f;
		bool bunnyFractureActive = false;
		float fractureSeparation = 0.0f;
		bool showGeometryQueryBounds = false;
		size_t geometryQueryBoundsCount = 0;
		double physicsFps = 0.0;
		{
			std::lock_guard<std::mutex> lock(m_ImguiMutex);
			sceneName = m_ImguiSceneName;
			sceneFiles = m_ImguiSceneFiles;
			demoIndex = m_ImguiDemoIndex;
			objectCount = m_ImguiObjectCount;
			cameraDistance = m_ImguiCameraDistance;
			bunnyFractureActive = m_ImguiBunnyFractureActive;
			fractureSeparation = m_ImguiFractureSeparation;
			showGeometryQueryBounds = m_ImguiShowGeometryQueryBounds;
			geometryQueryBoundsCount = m_ImguiGeometryQueryBoundsCount;
			physicsFps = m_ImguiPhysicsFps;
		}

		char fpsText[128];
		const double renderFps = m_RenderThread ? m_RenderThread->GetRenderFps() : 0.0;
		snprintf(fpsText, sizeof(fpsText), "Physics: %.1f FPS", physicsFps);
		imguiDrawText(width - 10, 12, IMGUI_ALIGN_RIGHT, fpsText, imguiRGBA(230, 235, 240, 255));
		snprintf(fpsText, sizeof(fpsText), "Render: %.1f FPS", renderFps);
		imguiDrawText(width - 10, 30, IMGUI_ALIGN_RIGHT, fpsText, imguiRGBA(230, 235, 240, 255));

		const int panelHeight = std::max(120, height - 20);
		imguiBeginScrollArea("", 10, 10, 220, panelHeight, &m_ImguiScroll);

		imguiValue(sceneName.empty() ? "unknown" : sceneName.c_str());

		char text[128];
		snprintf(text, sizeof(text), "Objects: %u", static_cast<unsigned int>(objectCount));
		imguiValue(text);

		imguiSeparatorLine();
		imguiLabel("Debug");
		if (imguiCheck("Geometry Query AABB", showGeometryQueryBounds, !bunnyFractureActive))
		{
			std::lock_guard<std::mutex> lock(m_ImguiMutex);
			m_ImguiPendingShowGeometryQueryBounds = true;
			m_ImguiPendingShowGeometryQueryBoundsValue = !showGeometryQueryBounds;
		}
		if (showGeometryQueryBounds)
		{
			snprintf(text, sizeof(text), "Query AABBs: %u", static_cast<unsigned int>(geometryQueryBoundsCount));
			imguiValue(text);
		}

		imguiSeparatorLine();
		imguiLabel("Demo");
		std::vector<std::string> demoNames = sceneFiles;
		demoNames.push_back("Bunny Fracture");
		if (demoNames.empty())
		{
			imguiValue("No demos found");
		}
		else
		{
			if (demoIndex < 0)
			{
				demoIndex = 0;
			}
			if (demoIndex >= (int)demoNames.size())
			{
				demoIndex = (int)demoNames.size() - 1;
			}

			std::vector<const char*> demoLabels;
			demoLabels.reserve(demoNames.size());
			for (const std::string& demoName : demoNames)
			{
				demoLabels.push_back(demoName.c_str());
			}

			int selectedDemo = demoIndex;
			if (imguiCombo("Select", &selectedDemo, demoLabels.data(), (int)demoLabels.size()))
			{
				std::lock_guard<std::mutex> lock(m_ImguiMutex);
				m_ImguiPendingDemoIndex = selectedDemo;
			}

			if (bunnyFractureActive)
			{
				imguiSeparatorLine();
				float separation = fractureSeparation;
				if (imguiSlider("Separation", &separation, 0.0f, 1.0f, 0.01f))
				{
					std::lock_guard<std::mutex> lock(m_ImguiMutex);
					m_ImguiPendingFractureSeparation = true;
					m_ImguiPendingFractureSeparationValue = separation;
				}
			}
		}

		imguiEndScrollArea();
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

		SubmitGeometryQueryBounds();
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

	bool WorldViewer::LoadBunnyFractureDemo()
	{
		const std::string bunnyPath = ResolveTestDataFile("bunny.obj");
		if (bunnyPath.empty())
		{
			return false;
		}

		DynamicMesh bunny;
		if (!bunny.LoadObj(bunnyPath.c_str()))
		{
			return false;
		}

		const Box3 bounds = bunny.GetBounds();
		const Vector3 center = bounds.GetCenter();
		const Vector3 extent = bounds.GetExtent();
		const float maxSize = std::max(BoundsMaxDim(bounds), 1e-3f);

		std::vector<Vector3> sites;
		sites.push_back(center + Vector3( extent.x * 0.42f,  extent.y * 0.10f,  extent.z * 0.12f));
		sites.push_back(center + Vector3(-extent.x * 0.38f, -extent.y * 0.08f, -extent.z * 0.16f));
		sites.push_back(center + Vector3( extent.x * 0.08f,  extent.y * 0.45f, -extent.z * 0.12f));
		sites.push_back(center + Vector3(-extent.x * 0.10f, -extent.y * 0.42f,  extent.z * 0.10f));
		sites.push_back(center + Vector3( extent.x * 0.10f, -extent.y * 0.04f,  extent.z * 0.42f));
		sites.push_back(center + Vector3(-extent.x * 0.12f,  extent.y * 0.08f, -extent.z * 0.40f));
		sites.push_back(center + Vector3( extent.x * 0.28f, -extent.y * 0.30f, -extent.z * 0.22f));
		sites.push_back(center + Vector3(-extent.x * 0.26f,  extent.y * 0.30f,  extent.z * 0.22f));
		sites.push_back(center + Vector3( extent.x * 0.34f,  extent.y * 0.28f, -extent.z * 0.06f));
		sites.push_back(center + Vector3(-extent.x * 0.32f, -extent.y * 0.26f,  extent.z * 0.08f));
		sites.push_back(center + Vector3( extent.x * 0.20f, -extent.y * 0.18f,  extent.z * 0.32f));
		sites.push_back(center + Vector3(-extent.x * 0.22f,  extent.y * 0.16f, -extent.z * 0.34f));
		sites.push_back(center + Vector3( extent.x * 0.02f,  extent.y * 0.34f,  extent.z * 0.30f));
		sites.push_back(center + Vector3(-extent.x * 0.04f, -extent.y * 0.36f, -extent.z * 0.28f));
		sites.push_back(center + Vector3( extent.x * 0.40f, -extent.y * 0.02f, -extent.z * 0.34f));
		sites.push_back(center + Vector3(-extent.x * 0.40f,  extent.y * 0.02f,  extent.z * 0.34f));

		FractureOptions options;
		options.SnapTolerance = std::max(maxSize * 1e-5f, 1e-6f);
		options.BoundsPaddingScale = 0.20f;
		options.Grout = 0.0f;
		options.WeldSharedEdges = true;
		options.MinTriangleCount = 4;

		std::vector<FracturePiece> pieces;
		if (!Fracture::VoronoiFracture(bunny, sites, pieces, options) || pieces.empty())
		{
			return false;
		}

		m_World->Reset(Vector3::Zero());
		m_RenderBindings.clear();
		m_HighlightedGeometry = nullptr;
		m_FracturePieces.clear();
		m_FractureMaxSeparation = maxSize * 1.15f;
		m_FractureSeparation = 0.0f;
		m_BunnyFractureDemoActive = true;
		m_CurrentSceneName = "Bunny Fracture";

		std::vector<RenderMeshDesc> renderMeshes;
		renderMeshes.reserve(pieces.size());
		for (size_t pieceIndex = 0; pieceIndex < pieces.size(); ++pieceIndex)
		{
			FracturePiece& piece = pieces[pieceIndex];
			piece.Mesh.CalculateWeightAverageNormals();

			std::ostringstream id;
			id << "bunny_fracture_" << pieceIndex;
			RenderMeshDesc mesh = BuildDynamicMeshRenderMesh(piece.Mesh, id.str(), PieceColor(pieceIndex));
			if (mesh.Vertices.empty() || mesh.Indices.empty())
			{
				continue;
			}

			Vector3 direction = piece.Center - center;
			if (direction.SafeNormalize() == 0.0f)
			{
				const float angle = (float)pieceIndex * 2.39996323f;
				direction = Vector3(cosf(angle), 0.25f * ((pieceIndex & 1) ? 1.0f : -1.0f), sinf(angle)).SafeUnit();
			}

			FractureRenderPiece renderPiece;
			renderPiece.MeshId = mesh.Id;
			renderPiece.Direction = direction;
			m_FracturePieces.push_back(renderPiece);
			renderMeshes.push_back(mesh);
		}

		if (renderMeshes.empty())
		{
			m_BunnyFractureDemoActive = false;
			return false;
		}

		m_CamCenter = center;
		m_CamParam = Vector3(1.05f, 0.45f, maxSize * 4.0f);

		if (m_RenderThread)
		{
			CameraDesc camera = m_World->GetCamera();
			camera.At = m_CamCenter;
			camera.Eye = GetCameraPosition();
			camera.NearPlane = std::max(0.01f, maxSize * 0.01f);
			camera.FarPlane = std::max(1000.0f, maxSize * 20.0f);

			DirectionalLightDesc light = m_World->GetLight();
			light.ShadowCenter = center;
			light.ShadowDistance = maxSize * 6.0f;
			light.ShadowSize = maxSize * 4.0f;

			std::shared_ptr<std::vector<RenderMeshDesc> > meshes(new std::vector<RenderMeshDesc>(std::move(renderMeshes)));
			m_RenderThread->Submit([meshes, camera, light](Renderer& renderer) {
				renderer.Reset();
				renderer.SetCamera(camera);
				renderer.SetLight(light);
				for (const RenderMeshDesc& mesh : *meshes)
				{
					renderer.AddMesh(mesh);
				}
			});
		}

		UpdateImguiState();
		return true;
	}

	void WorldViewer::UpdateFractureSeparation(float separation)
	{
		m_FractureSeparation = std::max(0.0f, std::min(separation, 1.0f));
		SubmitFractureTransforms();
		UpdateImguiState();
	}

	void WorldViewer::SubmitFractureTransforms()
	{
		if (!m_BunnyFractureDemoActive || m_RenderThread == nullptr)
		{
			return;
		}

		std::shared_ptr<std::vector<RenderTransformUpdate> > updates(new std::vector<RenderTransformUpdate>());
		updates->reserve(m_FracturePieces.size());
		for (const FractureRenderPiece& piece : m_FracturePieces)
		{
			RenderTransformUpdate update;
			update.Id = piece.MeshId;
			update.WorldTransform = Transform(piece.Direction * (m_FractureSeparation * m_FractureMaxSeparation));
			updates->push_back(update);
		}

		m_RenderThread->Submit([updates](Renderer& renderer) {
			for (const RenderTransformUpdate& update : *updates)
			{
				renderer.UpdateTransform(update);
			}
		});
	}

	void WorldViewer::SubmitTransforms()
	{
		if (m_BunnyFractureDemoActive)
		{
			return;
		}

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

	void WorldViewer::SubmitGeometryQueryBounds()
	{
		if (!m_ShowGeometryQueryBounds)
		{
			return;
		}

		if (m_BunnyFractureDemoActive || m_RenderThread == nullptr)
		{
			ClearGeometryQueryBounds();
			return;
		}

		PhysicsWorld* simulation = m_World ? m_World->GetSimulation() : nullptr;
		GeometryQuery* geometryQuery = simulation ? simulation->GetGeometryQuery() : nullptr;
		if (geometryQuery == nullptr)
		{
			ClearGeometryQueryBounds();
			return;
		}

		std::vector<Box3> bounds;
		geometryQuery->CollectAABBs(&bounds);

		std::vector<RenderMeshDesc> meshes;
		std::vector<std::string> newMeshIds;
		meshes.reserve(bounds.size());
		newMeshIds.reserve(bounds.size());
		for (size_t i = 0; i < bounds.size(); ++i)
		{
			std::ostringstream id;
			id << "__debug.geometry_query_aabb." << i;
			const std::string meshId = id.str();
			BuildAabbWireMesh(bounds[i], meshId, Vector4(0.08f, 0.88f, 0.78f, 1.0f), &meshes);
			newMeshIds.push_back(meshId);
		}

		std::vector<std::string> oldMeshIds;
		oldMeshIds.swap(m_GeometryQueryBoundsMeshIds);
		m_GeometryQueryBoundsMeshIds = std::move(newMeshIds);

		std::shared_ptr<std::vector<std::string> > renderOldMeshIds(new std::vector<std::string>(std::move(oldMeshIds)));
		std::shared_ptr<std::vector<RenderMeshDesc> > renderMeshes(new std::vector<RenderMeshDesc>(std::move(meshes)));
		m_RenderThread->Submit([renderOldMeshIds, renderMeshes](Renderer& renderer) {
			for (const std::string& meshId : *renderOldMeshIds)
			{
				renderer.DeleteMesh(meshId.c_str());
			}
			for (const RenderMeshDesc& mesh : *renderMeshes)
			{
				renderer.AddMesh(mesh);
			}
		});
	}

	void WorldViewer::ClearGeometryQueryBounds()
	{
		std::vector<std::string> oldMeshIds;
		oldMeshIds.swap(m_GeometryQueryBoundsMeshIds);
		if (oldMeshIds.empty() || m_RenderThread == nullptr)
		{
			return;
		}

		std::shared_ptr<std::vector<std::string> > renderOldMeshIds(new std::vector<std::string>(std::move(oldMeshIds)));
		m_RenderThread->Submit([renderOldMeshIds](Renderer& renderer) {
			for (const std::string& meshId : *renderOldMeshIds)
			{
				renderer.DeleteMesh(meshId.c_str());
			}
		});
	}

	void WorldViewer::SetShowGeometryQueryBounds(bool show)
	{
		if (m_ShowGeometryQueryBounds == show)
		{
			if (show)
			{
				SubmitGeometryQueryBounds();
			}
			return;
		}

		m_ShowGeometryQueryBounds = show;
		if (m_ShowGeometryQueryBounds)
		{
			SubmitGeometryQueryBounds();
		}
		else
		{
			ClearGeometryQueryBounds();
		}
		UpdateImguiState();
	}

	void WorldViewer::ApplyImguiCommands()
	{
		std::string pendingSceneFile;
		int pendingDemoIndex = -1;
		bool pendingDistance = false;
		float pendingDistanceValue = 0.0f;
		bool pendingFractureSeparation = false;
		float pendingFractureSeparationValue = 0.0f;
		bool pendingGeometryQueryBounds = false;
		bool pendingGeometryQueryBoundsValue = false;
		{
			std::lock_guard<std::mutex> lock(m_ImguiMutex);
			pendingDemoIndex = m_ImguiPendingDemoIndex;
			m_ImguiPendingDemoIndex = -1;
			pendingSceneFile.swap(m_ImguiPendingSceneFile);
			pendingDistance = m_ImguiPendingCameraDistance;
			pendingDistanceValue = m_ImguiPendingCameraDistanceValue;
			m_ImguiPendingCameraDistance = false;
			pendingFractureSeparation = m_ImguiPendingFractureSeparation;
			pendingFractureSeparationValue = m_ImguiPendingFractureSeparationValue;
			m_ImguiPendingFractureSeparation = false;
			pendingGeometryQueryBounds = m_ImguiPendingShowGeometryQueryBounds;
			pendingGeometryQueryBoundsValue = m_ImguiPendingShowGeometryQueryBoundsValue;
			m_ImguiPendingShowGeometryQueryBounds = false;
		}

		if (pendingDemoIndex >= 0)
		{
			if (pendingDemoIndex < (int)m_SceneFiles.size())
			{
				LoadScene(m_SceneFiles[(size_t)pendingDemoIndex]);
			}
			else if (pendingDemoIndex == (int)m_SceneFiles.size())
			{
				LoadBunnyFractureDemo();
			}
		}

		if (!pendingSceneFile.empty())
		{
			LoadScene(pendingSceneFile);
		}

		if (pendingDistance)
		{
			m_CamParam.z = std::max(1.0f, pendingDistanceValue);
			UpdateCamera();
		}

		if (pendingFractureSeparation)
		{
			UpdateFractureSeparation(pendingFractureSeparationValue);
		}

		if (pendingGeometryQueryBounds)
		{
			SetShowGeometryQueryBounds(pendingGeometryQueryBoundsValue);
		}
	}

	void WorldViewer::UpdateImguiState()
	{
		std::lock_guard<std::mutex> lock(m_ImguiMutex);
		m_ImguiSceneName = m_CurrentSceneName;
		m_ImguiSceneFiles = m_SceneFiles;
		m_ImguiDemoIndex = m_BunnyFractureDemoActive ? (int)m_SceneFiles.size() : GetCurrentSceneDemoIndex();
		m_ImguiObjectCount = m_BunnyFractureDemoActive ? m_FracturePieces.size() : (m_World ? m_World->GetObjects().size() : 0);
		m_ImguiCameraDistance = m_CamParam.z;
		m_ImguiBunnyFractureActive = m_BunnyFractureDemoActive;
		m_ImguiFractureSeparation = m_FractureSeparation;
		m_ImguiShowGeometryQueryBounds = m_ShowGeometryQueryBounds;
		m_ImguiGeometryQueryBoundsCount = m_GeometryQueryBoundsMeshIds.size();
		m_ImguiPhysicsFps = m_PhysicsFps;
	}

	int WorldViewer::GetCurrentSceneDemoIndex() const
	{
		for (size_t i = 0; i < m_SceneFiles.size(); ++i)
		{
			if (m_SceneFiles[i] == m_CurrentSceneName)
			{
				return (int)i;
			}
		}
		return 0;
	}

	void WorldViewer::UpdatePhysicsFps()
	{
		++m_PhysicsFrameCount;
		const auto curr = std::chrono::steady_clock::now();
		const std::chrono::duration<double> elapsed = curr - m_PhysicsFpsTime;
		if (elapsed.count() < 1.0)
		{
			return;
		}

		m_PhysicsFps = m_PhysicsFrameCount / elapsed.count();
		m_PhysicsFrameCount = 0;
		m_PhysicsFpsTime = curr;
	}

	Ray3 WorldViewer::BuildSceneRay(int x, int y, int width, int height) const
	{
		CameraDesc camera = m_World ? m_World->GetCamera() : CameraDesc();
		camera.Eye = GetCameraPosition();
		camera.At = m_CamCenter;

		const float aspect = static_cast<float>(width) / static_cast<float>(height);
		const float ndcX = (2.0f * (static_cast<float>(x) + 0.5f) / static_cast<float>(width)) - 1.0f;
		const float ndcY = 1.0f - (2.0f * (static_cast<float>(y) + 0.5f) / static_cast<float>(height));
		const float tanHalfFovY = tanf(camera.FovY * 0.5f);

		const Vector3 forward = SafeUnitVector(camera.At - camera.Eye, Vector3::UnitZ());
		const Vector3 right = SafeUnitVector(camera.Up.Cross(forward), Vector3::UnitX());
		const Vector3 up = SafeUnitVector(forward.Cross(right), Vector3::UnitY());
		const Vector3 direction = SafeUnitVector(
			forward + right * (ndcX * aspect * tanHalfFovY) + up * (ndcY * tanHalfFovY),
			forward);

		return Ray3(camera.Eye, direction);
	}

	void WorldViewer::HandleSceneRay(const Ray3& ray)
	{
		if (m_BunnyFractureDemoActive)
		{
			SetHighlightedGeometry(nullptr);
			return;
		}

		PhysicsWorld* simulation = m_World ? m_World->GetSimulation() : nullptr;
		if (simulation == nullptr)
		{
			return;
		}

		GeometryQuery* geometryQuery = simulation->GetGeometryQuery();
		if (geometryQuery == nullptr)
		{
			return;
		}

		RayCastOption option;
		option.Type = RayCastOption::RAYCAST_NEAREST;
		option.HitBothSides = true;
		option.MaxDist = 1000.0f;

		RayCastResult result;
		const bool hit = geometryQuery->RayCastQuery(ray.Origin, ray.Dir, option, &result);
		SetHighlightedGeometry(hit && IsHighlightableGeometry(result.hitGeom) ? result.hitGeom : nullptr);
	}

	void WorldViewer::SetHighlightedGeometry(Geometry* geometry)
	{
		if (m_HighlightedGeometry == geometry)
		{
			return;
		}

		Geometry* previous = m_HighlightedGeometry;
		m_HighlightedGeometry = geometry;

		if (m_RenderThread == nullptr)
		{
			return;
		}

		std::vector<std::string> disableMeshIds;
		std::vector<std::string> enableMeshIds;
		for (const RenderBinding& binding : m_RenderBindings)
		{
			if (binding.GeometryPtr == previous)
			{
				disableMeshIds.insert(disableMeshIds.end(), binding.MeshIds.begin(), binding.MeshIds.end());
			}
			if (binding.GeometryPtr == geometry)
			{
				enableMeshIds.insert(enableMeshIds.end(), binding.MeshIds.begin(), binding.MeshIds.end());
			}
		}

		std::shared_ptr<std::vector<std::string> > renderDisableMeshIds(new std::vector<std::string>(std::move(disableMeshIds)));
		std::shared_ptr<std::vector<std::string> > renderEnableMeshIds(new std::vector<std::string>(std::move(enableMeshIds)));
		m_RenderThread->Submit([renderDisableMeshIds, renderEnableMeshIds](Renderer& renderer) {
			for (const std::string& meshId : *renderDisableMeshIds)
			{
				renderer.SetMeshOutline(meshId.c_str(), false, Vector4(0.0f, 0.0f, 0.0f, 1.0f), 0.0f);
			}
			for (const std::string& meshId : *renderEnableMeshIds)
			{
				renderer.SetMeshOutline(meshId.c_str(), true, Vector4(1.0f, 0.82f, 0.18f, 1.0f), 0.08f);
			}
		});
	}

	void WorldViewer::DrawImguiCallback(int width, int height, void* userData)
	{
		WorldViewer* viewer = static_cast<WorldViewer*>(userData);
		if (viewer)
		{
			viewer->DrawImgui(width, height);
		}
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
		const std::vector<std::string> sceneDirectories = SceneDirectoryCandidates(m_SceneDirectory);
		for (const std::string& directory : sceneDirectories)
		{
			candidates.push_back(JoinPath(directory, requested));
		}

		for (const std::string& candidate : candidates)
		{
			if (FileExists(candidate))
			{
				return candidate;
			}
		}

		return requested;
	}

	std::string WorldViewer::ResolveTestDataFile(const char* fileName) const
	{
		if (fileName == nullptr || fileName[0] == '\0')
		{
			return std::string();
		}

		const std::string requested(fileName);
		if (FileExists(requested))
		{
			return requested;
		}

		const std::vector<std::string> dataDirectories = TestDataDirectoryCandidates(m_SceneDirectory);
		for (const std::string& directory : dataDirectories)
		{
			const std::string candidate = JoinPath(directory, requested);
			if (FileExists(candidate))
			{
				return candidate;
			}
		}

		return std::string();
	}

	void WorldViewer::RefreshSceneList()
	{
		std::vector<std::string> sceneFiles;
		std::string sceneDirectory;

		const std::vector<std::string> sceneDirectories = SceneDirectoryCandidates(m_SceneDirectory);
		for (const std::string& directory : sceneDirectories)
		{
			if (!DirectoryExists(directory))
			{
				continue;
			}

			sceneFiles = ListJsonFiles(directory);
			if (!sceneFiles.empty())
			{
				sceneDirectory = EnsureTrailingSlash(directory);
				break;
			}
		}

		if (!sceneDirectory.empty())
		{
			m_SceneDirectory = sceneDirectory;
		}
		m_SceneFiles.swap(sceneFiles);
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
