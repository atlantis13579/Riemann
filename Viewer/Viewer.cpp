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
#include <commdlg.h>
#if defined(_MSC_VER)
#pragma comment(lib, "Comdlg32.lib")
#endif
#else
#include <dirent.h>
#endif

#include "MeshCuttingPanel.h"
#include "MeshRenderer.h"
#include "RenderThread.h"
#include "SceneWorld.h"
#include "../Renderer/imgui.h"
#include "../Renderer/RiemannRenderer.h"
#include "../Src/Collision/GeometryObject.h"
#include "../Src/Collision/GeometryQuery.h"
#include "../Src/CollisionPrimitive/PrimitiveType.h"
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

		Transform GetGeometryWorldTransform(Geometry* geometry)
		{
			if (geometry == nullptr)
			{
				return Transform::Identity();
			}

			RigidBody* body = geometry->GetParent<RigidBody>();
			return body ? body->GetGeometryTransform(geometry) : *geometry->GetTransform();
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

		bool HasObjExtension(const std::string& fileName)
		{
			if (fileName.size() < 4)
			{
				return false;
			}

			const size_t offset = fileName.size() - 4;
			return fileName[offset] == '.' &&
				tolower(static_cast<unsigned char>(fileName[offset + 1])) == 'o' &&
				tolower(static_cast<unsigned char>(fileName[offset + 2])) == 'b' &&
				tolower(static_cast<unsigned char>(fileName[offset + 3])) == 'j';
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

		std::vector<std::string> ListObjFiles(const std::string& directory)
		{
			std::vector<std::string> files;
#if defined(_WIN32)
			const std::string searchPath = JoinPath(directory, "*.obj");
			WIN32_FIND_DATAA findData;
			HANDLE findHandle = FindFirstFileA(searchPath.c_str(), &findData);
			if (findHandle != INVALID_HANDLE_VALUE)
			{
				do
				{
					if ((findData.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) == 0)
					{
						files.push_back(JoinPath(directory, findData.cFileName));
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
					if (HasObjExtension(entry->d_name))
					{
						files.push_back(JoinPath(directory, entry->d_name));
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

		void AddUniqueFile(std::vector<std::string>* files, const std::string& fileName)
		{
			if (files == nullptr || fileName.empty())
			{
				return;
			}
			if (std::find(files->begin(), files->end(), fileName) == files->end())
			{
				files->push_back(fileName);
			}
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

		bool IsHighlightableGeometry(const Geometry* geometry)
		{
			if (geometry == nullptr || geometry->GetShapeType() == PrimitiveType::PLANE)
			{
				return false;
			}

			const Box3& bounds = geometry->GetBounds();
			return bounds.MaxDim() < 200.0f;
		}

		bool IsConvexDecompositionMode(int mode)
		{
			return mode == MeshCuttingMode_VHACD || mode == MeshCuttingMode_COACD;
		}

		const char* CuttingPanelName(int mode)
		{
			return IsConvexDecompositionMode(mode) ? "Convex Decomposition" : "Mesh Cutting";
		}
	}

	WorldViewer::WorldViewer(RenderThread* renderThread, const std::string& sceneFile)
		: m_World(new SceneWorld())
		, m_RenderThread(renderThread)
		, m_HighlightedGeometry(nullptr)
		, m_CamCenter(Vector3::Zero())
		, m_CamParam(Vector3(1.0f, 0.6f, 15.0f))
		, m_CuttingPanelActive(false)
		, m_CuttingObjIndex(0)
		, m_CuttingMode(MeshCuttingMode_VoronoiFracture3D)
		, m_CuttingPieceCount(16)
		, m_CuttingPiecesX(4)
		, m_CuttingPiecesY(3)
		, m_CuttingPiecesZ(2)
		, m_CuttingSeed(7)
		, m_CuttingSeparation(0.0f)
		, m_CuttingMaxSeparation(1.0f)
		, m_ShowGeometryQueryBounds(false)
		, m_RenderRevision(0)
		, m_ImguiDemoIndex(0)
		, m_ImguiObjectCount(0)
		, m_ImguiCameraDistance(15.0f)
		, m_ImguiCuttingPanelActive(false)
		, m_ImguiCuttingObjIndex(0)
		, m_ImguiCuttingMode(MeshCuttingMode_VoronoiFracture3D)
		, m_ImguiCuttingPieceCount(16)
		, m_ImguiCuttingPiecesX(4)
		, m_ImguiCuttingPiecesY(3)
		, m_ImguiCuttingPiecesZ(2)
		, m_ImguiCuttingSeed(7)
		, m_ImguiCuttingSeparation(0.0f)
		, m_ImguiShowGeometryQueryBounds(false)
		, m_ImguiGeometryQueryBoundsCount(0)
		, m_ImguiPhysicsFps(0.0)
		, m_ImguiScroll(0)
		, m_ImguiPendingDemoIndex(-1)
		, m_ImguiPendingCameraDistance(false)
		, m_ImguiPendingCameraDistanceValue(15.0f)
		, m_ImguiPendingCuttingSeparation(false)
		, m_ImguiPendingCuttingSeparationValue(0.0f)
		, m_ImguiPendingCuttingObjIndex(-1)
		, m_ImguiPendingBrowseCuttingObj(false)
		, m_ImguiPendingApplyCutting(false)
		, m_ImguiPendingCuttingMode(false)
		, m_ImguiPendingCuttingModeValue(MeshCuttingMode_VoronoiFracture3D)
		, m_ImguiPendingCuttingPieceCount(false)
		, m_ImguiPendingCuttingPieceCountValue(16)
		, m_ImguiPendingCuttingPiecesX(false)
		, m_ImguiPendingCuttingPiecesXValue(4)
		, m_ImguiPendingCuttingPiecesY(false)
		, m_ImguiPendingCuttingPiecesYValue(3)
		, m_ImguiPendingCuttingPiecesZ(false)
		, m_ImguiPendingCuttingPiecesZValue(2)
		, m_ImguiPendingCuttingSeed(false)
		, m_ImguiPendingCuttingSeedValue(7)
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
		RefreshObjList();
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
			if (m_CuttingPanelActive)
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
		m_CuttingPanelActive = false;
		m_CuttingPieces.clear();
		m_CuttingSeparation = 0.0f;
		RefreshSceneList();
		RefreshObjList();
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
		m_CuttingPanelActive = false;
		m_CuttingPieces.clear();
		m_HighlightedGeometry = nullptr;
		RebuildRenderScene();
	}

	void WorldViewer::UpdateSimulator(float dt)
	{
		ApplyImguiCommands();
		m_World->Step(dt);
		if (m_RenderRevision != m_World->GetRenderRevision())
		{
			RebuildRenderScene();
		}
		else
		{
			SubmitTransforms();
			SubmitGeometryQueryBounds();
		}
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
		bool cuttingPanelActive = false;
		std::string cuttingObjPath;
		std::string cuttingStatus;
		int cuttingMode = 0;
		int cuttingPieceCount = 0;
		int cuttingPiecesX = 0;
		int cuttingPiecesY = 0;
		int cuttingPiecesZ = 0;
		int cuttingSeed = 0;
		float cuttingSeparation = 0.0f;
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
			cuttingPanelActive = m_ImguiCuttingPanelActive;
			cuttingObjPath = m_ImguiCuttingObjPath;
			cuttingStatus = m_ImguiCuttingStatus;
			cuttingMode = m_ImguiCuttingMode;
			cuttingPieceCount = m_ImguiCuttingPieceCount;
			cuttingPiecesX = m_ImguiCuttingPiecesX;
			cuttingPiecesY = m_ImguiCuttingPiecesY;
			cuttingPiecesZ = m_ImguiCuttingPiecesZ;
			cuttingSeed = m_ImguiCuttingSeed;
			cuttingSeparation = m_ImguiCuttingSeparation;
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
		if (imguiCheck("Geometry Query AABB", showGeometryQueryBounds, true))
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
		demoNames.push_back("Mesh Cutting");
		demoNames.push_back("Convex Decomposition");
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

			if (cuttingPanelActive)
			{
				imguiSeparatorLine();
				const bool convexDecompositionDemo = IsConvexDecompositionMode(cuttingMode) || demoIndex == (int)sceneFiles.size() + 1;
				imguiLabel(convexDecompositionDemo ? "Convex Decomposition" : "Mesh Cutting");

				const std::string objValue = cuttingObjPath.empty()
					? std::string("OBJ: bunny.obj")
					: std::string("OBJ: ") + FileNameOf(cuttingObjPath);
				imguiValue(objValue.c_str());

				if (imguiButton("Browse OBJ..."))
				{
					std::lock_guard<std::mutex> lock(m_ImguiMutex);
					m_ImguiPendingBrowseCuttingObj = true;
				}

				int selectedMode = std::max(0, std::min(cuttingMode, (int)MeshCuttingMode_Count - 1));
				if (convexDecompositionDemo)
				{
					const char* modes[] =
					{
						"VHACD",
						"COACD",
					};
					int selectedAlgorithm = selectedMode == MeshCuttingMode_COACD ? 1 : 0;
					if (imguiCombo("Algorithm", &selectedAlgorithm, modes, 2))
					{
						std::lock_guard<std::mutex> lock(m_ImguiMutex);
						m_ImguiPendingCuttingMode = true;
						m_ImguiPendingCuttingModeValue = selectedAlgorithm == 0 ? MeshCuttingMode_VHACD : MeshCuttingMode_COACD;
					}
					selectedMode = selectedAlgorithm == 0 ? MeshCuttingMode_VHACD : MeshCuttingMode_COACD;
				}
				else
				{
					const char* modes[] =
					{
						"Parallel X",
						"Parallel Y",
						"Parallel Z",
						"Voronoi 2D",
						"Voronoi 3D",
						"Cluster",
						"Voxel2D",
						"Voxel3D",
					};
					selectedMode = std::max(0, std::min(selectedMode, (int)MeshCuttingMode_Voxel3D));
					if (imguiCombo("Mode", &selectedMode, modes, (int)MeshCuttingMode_Voxel3D + 1))
					{
						std::lock_guard<std::mutex> lock(m_ImguiMutex);
						m_ImguiPendingCuttingMode = true;
						m_ImguiPendingCuttingModeValue = selectedMode;
					}
				}

				if (selectedMode == MeshCuttingMode_Voxel2D || selectedMode == MeshCuttingMode_Voxel3D)
				{
					const float gridMax = selectedMode == MeshCuttingMode_Voxel3D ? 8.0f : 32.0f;
					float piecesX = static_cast<float>(std::max(1, cuttingPiecesX));
					if (imguiSlider("Blocks X", &piecesX, 1.0f, gridMax, 1.0f))
					{
						std::lock_guard<std::mutex> lock(m_ImguiMutex);
						m_ImguiPendingCuttingPiecesX = true;
						m_ImguiPendingCuttingPiecesXValue = std::max(1, static_cast<int>(piecesX + 0.5f));
					}

					float piecesY = static_cast<float>(std::max(1, cuttingPiecesY));
					if (imguiSlider("Blocks Y", &piecesY, 1.0f, gridMax, 1.0f))
					{
						std::lock_guard<std::mutex> lock(m_ImguiMutex);
						m_ImguiPendingCuttingPiecesY = true;
						m_ImguiPendingCuttingPiecesYValue = std::max(1, static_cast<int>(piecesY + 0.5f));
					}

					if (selectedMode == MeshCuttingMode_Voxel3D)
					{
						float piecesZ = static_cast<float>(std::max(1, cuttingPiecesZ));
						if (imguiSlider("Blocks Z", &piecesZ, 1.0f, gridMax, 1.0f))
						{
							std::lock_guard<std::mutex> lock(m_ImguiMutex);
							m_ImguiPendingCuttingPiecesZ = true;
							m_ImguiPendingCuttingPiecesZValue = std::max(1, static_cast<int>(piecesZ + 0.5f));
						}
					}
				}
				else
				{
					float pieces = static_cast<float>(std::max(2, cuttingPieceCount));
					if (imguiSlider("Pieces", &pieces, 2.0f, 64.0f, 1.0f))
					{
						std::lock_guard<std::mutex> lock(m_ImguiMutex);
						m_ImguiPendingCuttingPieceCount = true;
						m_ImguiPendingCuttingPieceCountValue = std::max(2, static_cast<int>(pieces + 0.5f));
					}
				}

				if (selectedMode == MeshCuttingMode_VoronoiFracture2D ||
					selectedMode == MeshCuttingMode_VoronoiFracture3D ||
					selectedMode == MeshCuttingMode_Cluster)
				{
					float seed = static_cast<float>(cuttingSeed);
					if (imguiSlider("Seed", &seed, 0.0f, 999.0f, 1.0f))
					{
						std::lock_guard<std::mutex> lock(m_ImguiMutex);
						m_ImguiPendingCuttingSeed = true;
						m_ImguiPendingCuttingSeedValue = static_cast<int>(seed + 0.5f);
					}
				}

				float separation = cuttingSeparation;
				if (imguiSlider("Separation", &separation, 0.0f, 1.0f, 0.01f))
				{
					std::lock_guard<std::mutex> lock(m_ImguiMutex);
					m_ImguiPendingCuttingSeparation = true;
					m_ImguiPendingCuttingSeparationValue = separation;
				}

				if (imguiButton(convexDecompositionDemo ? "Apply" : "Apply Cut"))
				{
					std::lock_guard<std::mutex> lock(m_ImguiMutex);
					m_ImguiPendingApplyCutting = true;
				}

				if (!cuttingStatus.empty())
				{
					imguiValue(cuttingStatus.c_str());
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
		m_RenderRevision = m_World ? m_World->GetRenderRevision() : 0;

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

	bool WorldViewer::LoadCuttingPanel()
	{
		RefreshObjList();
		if (m_CuttingObjPath.empty())
		{
			m_CuttingObjPath = ResolveTestDataFile("bunny.obj");
		}
		if (m_CuttingObjPath.empty() && !m_CuttingObjFiles.empty())
		{
			m_CuttingObjPath = m_CuttingObjFiles.front();
		}
		if (m_CuttingObjPath.empty())
		{
			m_World->Reset(Vector3::Zero());
			m_RenderBindings.clear();
			m_HighlightedGeometry = nullptr;
			m_CuttingPieces.clear();
			m_CuttingPanelActive = true;
			m_CurrentSceneName = CuttingPanelName(m_CuttingMode);
			m_CuttingStatus = "Missing OBJ: bunny.obj";
			RebuildRenderScene();
			UpdateImguiState();
			return false;
		}

		MeshCuttingSource source;
		if (!LoadMeshCuttingSource(m_CuttingObjPath, &source))
		{
			m_World->Reset(Vector3::Zero());
			m_RenderBindings.clear();
			m_HighlightedGeometry = nullptr;
			m_CuttingPieces.clear();
			m_CuttingPanelActive = true;
			m_CurrentSceneName = CuttingPanelName(m_CuttingMode);
			m_CuttingStatus = source.Status.empty() ? "Failed to read OBJ" : source.Status;
			RebuildRenderScene();
			UpdateImguiState();
			return false;
		}

		const Box3 bounds = source.Bounds;
		const Vector3 center = bounds.GetCenter();
		m_CuttingMode = std::max(0, std::min(m_CuttingMode, (int)MeshCuttingMode_Count - 1));
		m_CuttingPieceCount = std::max(2, std::min(m_CuttingPieceCount, 64));
		m_CuttingPiecesX = std::max(1, std::min(m_CuttingPiecesX, 64));
		m_CuttingPiecesY = std::max(1, std::min(m_CuttingPiecesY, 64));
		m_CuttingPiecesZ = std::max(1, std::min(m_CuttingPiecesZ, 64));
		m_CuttingSeed = std::max(0, std::min(m_CuttingSeed, 999));
		m_CuttingSeparation = 0.0f;
		m_CuttingMaxSeparation = source.MaxSeparation;

		m_World->Reset(Vector3::Zero());
		m_RenderBindings.clear();
		m_HighlightedGeometry = nullptr;
		m_CuttingPieces.clear();
		m_CuttingPanelActive = true;
		m_CurrentSceneName = CuttingPanelName(m_CuttingMode);

		Geometry* geometry = m_World->AddTriangleMeshObject("mesh_source", source.Mesh, Transform(Vector3::Zero()), RigidType::Static, Vector4(0.72f, 0.76f, 0.82f, 1.0f), false);
		if (geometry == nullptr)
		{
			m_CuttingStatus = "No renderable mesh";
			RebuildRenderScene();
			UpdateImguiState();
			return false;
		}

		m_CamCenter = center;
		m_CamParam = Vector3(1.05f, 0.45f, std::max(1.0f, m_CuttingMaxSeparation * 3.5f));
		m_CuttingStatus = source.Status + (IsConvexDecompositionMode(m_CuttingMode) ? ". Click Apply" : ". Click Apply Cut");

		RebuildRenderScene();

		if (m_RenderThread)
		{
			CameraDesc camera = m_World->GetCamera();
			camera.At = m_CamCenter;
			camera.Eye = GetCameraPosition();
			camera.NearPlane = std::max(0.01f, m_CuttingMaxSeparation * 0.01f);
			camera.FarPlane = std::max(1000.0f, m_CuttingMaxSeparation * 20.0f);

			DirectionalLightDesc light = m_World->GetLight();
			light.ShadowCenter = center;
			light.ShadowDistance = m_CuttingMaxSeparation * 6.0f;
			light.ShadowSize = m_CuttingMaxSeparation * 4.0f;

			m_RenderThread->Submit([camera, light](Renderer& renderer) {
				renderer.SetCamera(camera);
				renderer.SetLight(light);
			});
		}

		UpdateImguiState();
		return true;
	}

	bool WorldViewer::ApplyCuttingPanel()
	{
		const Vector3 cameraCenter = m_CamCenter;
		const Vector3 cameraParam = m_CamParam;

		RefreshObjList();
		if (m_CuttingObjPath.empty())
		{
			m_CuttingObjPath = ResolveTestDataFile("bunny.obj");
		}
		if (m_CuttingObjPath.empty() && !m_CuttingObjFiles.empty())
		{
			m_CuttingObjPath = m_CuttingObjFiles.front();
		}
		if (m_CuttingObjPath.empty())
		{
			m_CuttingStatus = "Missing OBJ: bunny.obj";
			UpdateImguiState();
			return false;
		}

		MeshCuttingParams params;
		params.ObjPath = m_CuttingObjPath;
		params.Mode = m_CuttingMode;
		params.PieceCount = m_CuttingPieceCount;
		params.PiecesX = m_CuttingPiecesX;
		params.PiecesY = m_CuttingPiecesY;
		params.PiecesZ = m_CuttingPiecesZ;
		params.Seed = m_CuttingSeed;

		MeshCuttingResult cuttingResult;
		if (!BuildMeshCuttingPanel(params, &cuttingResult))
		{
			m_CuttingPanelActive = true;
			m_CuttingStatus = cuttingResult.Status.empty() ? "Cut failed" : cuttingResult.Status;
			UpdateImguiState();
			return false;
		}

		const Box3 bounds = cuttingResult.SourceBounds;
		const Vector3 center = bounds.GetCenter();
		m_CuttingMode = std::max(0, std::min(m_CuttingMode, (int)MeshCuttingMode_Count - 1));
		m_CuttingPieceCount = std::max(2, std::min(m_CuttingPieceCount, 64));
		m_CuttingPiecesX = std::max(1, std::min(m_CuttingPiecesX, 64));
		m_CuttingPiecesY = std::max(1, std::min(m_CuttingPiecesY, 64));
		m_CuttingPiecesZ = std::max(1, std::min(m_CuttingPiecesZ, 64));
		m_CuttingSeed = std::max(0, std::min(m_CuttingSeed, 999));
		m_CuttingMaxSeparation = cuttingResult.MaxSeparation;
		m_CamCenter = cameraCenter;
		m_CamParam = cameraParam;

		m_World->Reset(Vector3::Zero());
		m_RenderBindings.clear();
		m_HighlightedGeometry = nullptr;
		m_CuttingPieces.clear();
		m_CuttingPanelActive = true;
		m_CurrentSceneName = CuttingPanelName(m_CuttingMode);

		for (size_t pieceIndex = 0; pieceIndex < cuttingResult.Pieces.size(); ++pieceIndex)
		{
			const MeshCuttingPiece& piece = cuttingResult.Pieces[pieceIndex];

			std::ostringstream id;
			id << "mesh_cut_" << pieceIndex;
			const Transform transform(piece.Center + piece.Direction * (m_CuttingSeparation * m_CuttingMaxSeparation));
			Geometry* geometry = m_World->AddTriangleMeshObject(id.str(), piece.Mesh, transform, RigidType::Static, PieceColor(pieceIndex), false);
			if (geometry == nullptr)
			{
				continue;
			}

			CuttingRenderPiece renderPiece;
			renderPiece.GeometryPtr = geometry;
			renderPiece.MeshId = id.str();
			renderPiece.Center = piece.Center;
			renderPiece.Direction = piece.Direction;
			m_CuttingPieces.push_back(renderPiece);
		}

		if (m_CuttingPieces.empty())
		{
			m_CuttingPanelActive = true;
			m_CuttingStatus = "No renderable pieces";
			RebuildRenderScene();
			UpdateImguiState();
			return false;
		}

		m_CuttingStatus = cuttingResult.Status;

		RebuildRenderScene();
		SubmitCuttingTransforms();

		if (m_RenderThread)
		{
			CameraDesc camera = m_World->GetCamera();
			camera.At = m_CamCenter;
			camera.Eye = GetCameraPosition();
			camera.NearPlane = std::max(0.01f, m_CuttingMaxSeparation * 0.01f);
			camera.FarPlane = std::max(1000.0f, m_CuttingMaxSeparation * 20.0f);

			DirectionalLightDesc light = m_World->GetLight();
			light.ShadowCenter = center;
			light.ShadowDistance = m_CuttingMaxSeparation * 6.0f;
			light.ShadowSize = m_CuttingMaxSeparation * 4.0f;

			m_RenderThread->Submit([camera, light](Renderer& renderer) {
				renderer.SetCamera(camera);
				renderer.SetLight(light);
			});
		}

		UpdateImguiState();
		return true;
	}

	void WorldViewer::RefreshObjList()
	{
		std::vector<std::string> objFiles;
		const std::string defaultBunny = ResolveTestDataFile("bunny.obj");
		AddUniqueFile(&objFiles, defaultBunny);

		const std::vector<std::string> dataDirectories = TestDataDirectoryCandidates(m_SceneDirectory);
		for (const std::string& directory : dataDirectories)
		{
			if (!DirectoryExists(directory))
			{
				continue;
			}
			std::vector<std::string> files = ListObjFiles(directory);
			for (const std::string& file : files)
			{
				AddUniqueFile(&objFiles, file);
			}
		}

		if (!m_SceneDirectory.empty() && DirectoryExists(m_SceneDirectory))
		{
			std::vector<std::string> files = ListObjFiles(m_SceneDirectory);
			for (const std::string& file : files)
			{
				AddUniqueFile(&objFiles, file);
			}
		}

		AddUniqueFile(&objFiles, m_CuttingObjPath);
		m_CuttingObjFiles.swap(objFiles);

		m_CuttingObjIndex = 0;
		for (size_t i = 0; i < m_CuttingObjFiles.size(); ++i)
		{
			if (m_CuttingObjFiles[i] == m_CuttingObjPath)
			{
				m_CuttingObjIndex = (int)i;
				break;
			}
		}
		if (m_CuttingObjPath.empty() && !m_CuttingObjFiles.empty())
		{
			m_CuttingObjPath = m_CuttingObjFiles.front();
			m_CuttingObjIndex = 0;
		}
	}

	void WorldViewer::SubmitCuttingTransforms()
	{
		if (!m_CuttingPanelActive)
		{
			return;
		}

		GeometryQuery* geometryQuery = nullptr;
		PhysicsWorld* simulation = m_World ? m_World->GetSimulation() : nullptr;
		if (simulation)
		{
			geometryQuery = simulation->GetGeometryQuery();
		}

		for (const CuttingRenderPiece& piece : m_CuttingPieces)
		{
			if (piece.GeometryPtr == nullptr)
			{
				continue;
			}
			const Vector3 oldPosition = piece.GeometryPtr->GetPosition();
			const Vector3 newPosition = piece.Center + piece.Direction * (m_CuttingSeparation * m_CuttingMaxSeparation);
			piece.GeometryPtr->SetPosition(newPosition);
			if (geometryQuery)
			{
				geometryQuery->UpdateGeometry(piece.GeometryPtr, newPosition - oldPosition);
			}
		}

		SubmitTransforms();
		SubmitGeometryQueryBounds();
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
			update.WorldTransform = GetGeometryWorldTransform(binding.GeometryPtr);
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

		if (m_RenderThread == nullptr)
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
		bool pendingCuttingSeparation = false;
		float pendingCuttingSeparationValue = 0.0f;
		int pendingCuttingObjIndex = -1;
		bool pendingBrowseCuttingObj = false;
		bool pendingApplyCutting = false;
		bool pendingCuttingMode = false;
		int pendingCuttingModeValue = 0;
		bool pendingCuttingPieceCount = false;
		int pendingCuttingPieceCountValue = 0;
		bool pendingCuttingPiecesX = false;
		int pendingCuttingPiecesXValue = 0;
		bool pendingCuttingPiecesY = false;
		int pendingCuttingPiecesYValue = 0;
		bool pendingCuttingPiecesZ = false;
		int pendingCuttingPiecesZValue = 0;
		bool pendingCuttingSeed = false;
		int pendingCuttingSeedValue = 0;
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
			pendingCuttingSeparation = m_ImguiPendingCuttingSeparation;
			pendingCuttingSeparationValue = m_ImguiPendingCuttingSeparationValue;
			m_ImguiPendingCuttingSeparation = false;
			pendingCuttingObjIndex = m_ImguiPendingCuttingObjIndex;
			m_ImguiPendingCuttingObjIndex = -1;
			pendingBrowseCuttingObj = m_ImguiPendingBrowseCuttingObj;
			m_ImguiPendingBrowseCuttingObj = false;
			pendingApplyCutting = m_ImguiPendingApplyCutting;
			m_ImguiPendingApplyCutting = false;
			pendingCuttingMode = m_ImguiPendingCuttingMode;
			pendingCuttingModeValue = m_ImguiPendingCuttingModeValue;
			m_ImguiPendingCuttingMode = false;
			pendingCuttingPieceCount = m_ImguiPendingCuttingPieceCount;
			pendingCuttingPieceCountValue = m_ImguiPendingCuttingPieceCountValue;
			m_ImguiPendingCuttingPieceCount = false;
			pendingCuttingPiecesX = m_ImguiPendingCuttingPiecesX;
			pendingCuttingPiecesXValue = m_ImguiPendingCuttingPiecesXValue;
			m_ImguiPendingCuttingPiecesX = false;
			pendingCuttingPiecesY = m_ImguiPendingCuttingPiecesY;
			pendingCuttingPiecesYValue = m_ImguiPendingCuttingPiecesYValue;
			m_ImguiPendingCuttingPiecesY = false;
			pendingCuttingPiecesZ = m_ImguiPendingCuttingPiecesZ;
			pendingCuttingPiecesZValue = m_ImguiPendingCuttingPiecesZValue;
			m_ImguiPendingCuttingPiecesZ = false;
			pendingCuttingSeed = m_ImguiPendingCuttingSeed;
			pendingCuttingSeedValue = m_ImguiPendingCuttingSeedValue;
			m_ImguiPendingCuttingSeed = false;
			pendingGeometryQueryBounds = m_ImguiPendingShowGeometryQueryBounds;
			pendingGeometryQueryBoundsValue = m_ImguiPendingShowGeometryQueryBoundsValue;
			m_ImguiPendingShowGeometryQueryBounds = false;
		}

		if (pendingDemoIndex >= 0)
		{
			const int meshCuttingDemoIndex = (int)m_SceneFiles.size();
			const int convexDecompositionDemoIndex = meshCuttingDemoIndex + 1;
			if (pendingDemoIndex < meshCuttingDemoIndex)
			{
				LoadScene(m_SceneFiles[(size_t)pendingDemoIndex]);
			}
			else if (pendingDemoIndex == meshCuttingDemoIndex)
			{
				if (IsConvexDecompositionMode(m_CuttingMode))
				{
					m_CuttingMode = MeshCuttingMode_VoronoiFracture3D;
				}
				LoadCuttingPanel();
			}
			else if (pendingDemoIndex == convexDecompositionDemoIndex)
			{
				if (!IsConvexDecompositionMode(m_CuttingMode))
				{
					m_CuttingMode = MeshCuttingMode_VHACD;
				}
				LoadCuttingPanel();
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

		bool pendingLoadCuttingSource = false;
		if (pendingCuttingObjIndex >= 0 && pendingCuttingObjIndex < (int)m_CuttingObjFiles.size())
		{
			m_CuttingObjIndex = pendingCuttingObjIndex;
			m_CuttingObjPath = m_CuttingObjFiles[(size_t)pendingCuttingObjIndex];
			pendingLoadCuttingSource = true;
		}

		if (pendingBrowseCuttingObj)
		{
			const std::string selectedObj = OpenObjFileDialog();
			if (!selectedObj.empty())
			{
				m_CuttingObjPath = selectedObj;
				RefreshObjList();
				pendingLoadCuttingSource = true;
			}
		}

		bool pendingCuttingParams = false;
		if (pendingCuttingMode)
		{
			m_CuttingMode = std::max(0, std::min(pendingCuttingModeValue, (int)MeshCuttingMode_Count - 1));
			if (m_CuttingPanelActive)
			{
				m_CurrentSceneName = CuttingPanelName(m_CuttingMode);
			}
			pendingCuttingParams = true;
		}
		if (pendingCuttingPieceCount)
		{
			m_CuttingPieceCount = std::max(2, std::min(pendingCuttingPieceCountValue, 64));
			pendingCuttingParams = true;
		}
		if (pendingCuttingPiecesX)
		{
			m_CuttingPiecesX = std::max(1, std::min(pendingCuttingPiecesXValue, 64));
			pendingCuttingParams = true;
		}
		if (pendingCuttingPiecesY)
		{
			m_CuttingPiecesY = std::max(1, std::min(pendingCuttingPiecesYValue, 64));
			pendingCuttingParams = true;
		}
		if (pendingCuttingPiecesZ)
		{
			m_CuttingPiecesZ = std::max(1, std::min(pendingCuttingPiecesZValue, 64));
			pendingCuttingParams = true;
		}
		if (pendingCuttingSeed)
		{
			m_CuttingSeed = std::max(0, std::min(pendingCuttingSeedValue, 999));
			pendingCuttingParams = true;
		}
		if (pendingCuttingSeparation)
		{
			m_CuttingSeparation = std::max(0.0f, std::min(pendingCuttingSeparationValue, 1.0f));
		}

		if (pendingApplyCutting && m_CuttingPanelActive)
		{
			ApplyCuttingPanel();
		}
		else if (pendingLoadCuttingSource && m_CuttingPanelActive)
		{
			LoadCuttingPanel();
		}
		else if (pendingCuttingSeparation)
		{
			SubmitCuttingTransforms();
			UpdateImguiState();
		}
		else if (pendingCuttingParams && m_CuttingPanelActive)
		{
			m_CuttingStatus = IsConvexDecompositionMode(m_CuttingMode) ? "Ready. Click Apply" : "Ready. Click Apply Cut";
			UpdateImguiState();
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
		m_ImguiCuttingObjFiles = m_CuttingObjFiles;
		m_ImguiCuttingObjPath = m_CuttingObjPath;
		m_ImguiCuttingStatus = m_CuttingStatus;
		m_ImguiDemoIndex = m_CuttingPanelActive
			? ((int)m_SceneFiles.size() + (IsConvexDecompositionMode(m_CuttingMode) ? 1 : 0))
			: GetCurrentSceneDemoIndex();
		m_ImguiObjectCount = m_World ? m_World->GetObjects().size() : 0;
		m_ImguiCameraDistance = m_CamParam.z;
		m_ImguiCuttingPanelActive = m_CuttingPanelActive;
		m_ImguiCuttingObjIndex = m_CuttingObjIndex;
		m_ImguiCuttingMode = m_CuttingMode;
		m_ImguiCuttingPieceCount = m_CuttingPieceCount;
		m_ImguiCuttingPiecesX = m_CuttingPiecesX;
		m_ImguiCuttingPiecesY = m_CuttingPiecesY;
		m_ImguiCuttingPiecesZ = m_CuttingPiecesZ;
		m_ImguiCuttingSeed = m_CuttingSeed;
		m_ImguiCuttingSeparation = m_CuttingSeparation;
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
		if (m_World == nullptr)
		{
			return;
		}

		RayCastOption option;
		option.Type = RayCastOption::RAYCAST_NEAREST;
		option.HitBothSides = true;
		option.MaxDist = 1000.0f;

		RayCastResult result;
		bool hit = false;
		PhysicsWorld* simulation = m_World->GetSimulation();
		GeometryQuery* geometryQuery = simulation ? simulation->GetGeometryQuery() : nullptr;
		if (geometryQuery)
		{
			hit = geometryQuery->RayCastQuery(ray.Origin, ray.Dir, option, &result);
		}

		Geometry* hitGeometry = hit ? result.hitGeom : nullptr;
		if (m_CuttingPanelActive && !m_CuttingPieces.empty() && !IsCurrentCuttingGeometry(hitGeometry))
		{
			hitGeometry = nullptr;
		}
		SetHighlightedGeometry(hitGeometry && IsHighlightableGeometry(hitGeometry) ? hitGeometry : nullptr);
	}

	bool WorldViewer::IsCurrentCuttingGeometry(const Geometry* geometry) const
	{
		if (geometry == nullptr)
		{
			return false;
		}
		for (const CuttingRenderPiece& piece : m_CuttingPieces)
		{
			if (piece.GeometryPtr == geometry)
			{
				return true;
			}
		}
		return false;
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

	std::string WorldViewer::OpenObjFileDialog() const
	{
#if defined(_WIN32)
		char fileName[MAX_PATH] = {};
		if (!m_CuttingObjPath.empty())
		{
			snprintf(fileName, sizeof(fileName), "%s", m_CuttingObjPath.c_str());
		}
		OPENFILENAMEA ofn = {};
		ofn.lStructSize = sizeof(ofn);
		ofn.lpstrFile = fileName;
		ofn.nMaxFile = sizeof(fileName);
		ofn.lpstrFilter = "OBJ Files\0*.obj\0All Files\0*.*\0";
		ofn.lpstrDefExt = "obj";
		ofn.Flags = OFN_FILEMUSTEXIST | OFN_PATHMUSTEXIST | OFN_NOCHANGEDIR;
		return GetOpenFileNameA(&ofn) ? std::string(fileName) : std::string();
#else
		return std::string();
#endif
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
