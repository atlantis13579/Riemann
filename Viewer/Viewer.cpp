#include "../Src/Modules/Robotics/ArticulatedRobotController.h"
#include "Viewer.h"
#ifdef RIEMANN_SPIDER_TRAINING
#include "../Src/Modules/Robotics/SpiderTrainingController.h"
#endif

#if defined(_WIN32) && !defined(NOMINMAX)
#define NOMINMAX
#endif

#include <ctype.h>
#include <math.h>
#include <stdio.h>

#include <algorithm>
#include <fstream>
#include <limits>
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
#include "MeshSimplificationPanel.h"
#include "MeshRenderer.h"
#include "RenderThread.h"
#include "SceneWorld.h"
#include "../Src/Deformation/StrandSystem.h"
#include "../Src/Modules/Vehicle/VehicleDeformation.h"
#include "../Src/Modules/Vehicle/VehicleDynamics.h"
#include "../Renderer/imgui.h"
#include "../Renderer/RiemannRenderer.h"
#include "../Src/Collision/GeometryObject.h"
#include "../Src/Collision/GeometryQuery.h"
#include "../Src/CollisionPrimitive/PrimitiveType.h"
#include "../Src/CollisionPrimitive/TriangleMesh.h"
#include "../Src/Fluid/AerodynamicsRigidCoupling.h"
#include "../Src/Fluid/AerodynamicsSimulation.h"
#include "../Renderer/AerodynamicsVolume.h"
#include "../Src/Fluid/FluidRigidCoupling.h"
#include "../Src/Fluid/FluidSimulation.h"
#include "../Src/Modules/Tools/PhysxBinaryParser.h"
#include "../Src/RigidBodyDynamics/KinematicsTree.h"
#include "../Src/RigidBodyDynamics/PhysicsWorld.h"
#include "../Src/RigidBodyDynamics/PhysicalAnimation.h"
#include "../Src/RigidBodyDynamics/Ragdoll.h"
#include "../Src/Deformation/Cloth.h"
#include "../Src/Deformation/DeformableBody.h"

namespace Riemann
{
	namespace
	{
		const ResolutionPhaseSolver SolverPanelAlgorithms[] = {
			ResolutionPhaseSolver::SequentialImpulse, ResolutionPhaseSolver::LCPGlobal,
			ResolutionPhaseSolver::PBD, ResolutionPhaseSolver::XPBD
		};
		const SolverBackend SolverPanelBackends[] = {
			SolverBackend::CPU, SolverBackend::SIMD, SolverBackend::ComputeShader, SolverBackend::CUDA
		};

		int GetSolverPanelIndex(ResolutionPhaseSolver Solver)
		{
			for (int Index = 0; Index < 4; ++Index)
				if (SolverPanelAlgorithms[Index] == Solver) return Index;
			return 0;
		}

		int GetBackendPanelIndex(SolverBackend Backend)
		{
			for (int Index = 0; Index < 4; ++Index)
				if (SolverPanelBackends[Index] == Backend) return Index;
			return 0;
		}

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

		bool HasPathExtension(const std::string& fileName, const char* extension)
		{
			if (extension == nullptr)
			{
				return false;
			}

			std::string ext(extension);
			if (fileName.size() <= ext.size() || fileName[fileName.size() - ext.size() - 1] != '.')
			{
				return false;
			}

			const size_t offset = fileName.size() - ext.size();
			for (size_t i = 0; i < ext.size(); ++i)
			{
				const char a = static_cast<char>(tolower(static_cast<unsigned char>(fileName[offset + i])));
				const char b = static_cast<char>(tolower(static_cast<unsigned char>(ext[i])));
				if (a != b)
				{
					return false;
				}
			}
			return true;
		}

		bool HasObjExtension(const std::string& fileName)
		{
			return HasPathExtension(fileName, "obj");
		}

		bool HasFlatExtension(const std::string& fileName)
		{
			return HasPathExtension(fileName, "flat");
		}

		bool HasUassetExtension(const std::string& fileName)
		{
			return HasPathExtension(fileName, "uasset");
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

		std::vector<std::string> ListFlatFiles(const std::string& directory)
		{
			std::vector<std::string> files;
#if defined(_WIN32)
			const std::string searchPath = JoinPath(directory, "*.flat");
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
					if (HasFlatExtension(entry->d_name))
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

		std::vector<std::string> ListUassetFiles(const std::string& directory)
		{
			std::vector<std::string> files;
#if defined(_WIN32)
			const std::string searchPath = JoinPath(directory, "*.uasset");
			WIN32_FIND_DATAA findData;
			HANDLE findHandle = FindFirstFileA(searchPath.c_str(), &findData);
			if (findHandle != INVALID_HANDLE_VALUE)
			{
				do
				{
					if ((findData.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) == 0)
						files.push_back(JoinPath(directory, findData.cFileName));
				} while (FindNextFileA(findHandle, &findData));
				FindClose(findHandle);
			}
#else
			DIR* dir = opendir(directory.c_str());
			if (dir != nullptr)
			{
				struct dirent* entry = nullptr;
				while ((entry = readdir(dir)) != nullptr)
					if (HasUassetExtension(entry->d_name))
						files.push_back(JoinPath(directory, entry->d_name));
				closedir(dir);
			}
#endif
			std::sort(files.begin(), files.end());
			return files;
		}

		std::vector<std::string> ListMeshFiles(const std::string& directory)
		{
			std::vector<std::string> files = ListObjFiles(directory);
			const std::vector<std::string> flatFiles = ListFlatFiles(directory);
			files.insert(files.end(), flatFiles.begin(), flatFiles.end());
			const std::vector<std::string> uassetFiles = ListUassetFiles(directory);
			files.insert(files.end(), uassetFiles.begin(), uassetFiles.end());
			std::sort(files.begin(), files.end());
			files.erase(std::unique(files.begin(), files.end()), files.end());
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
			return mode == MeshCuttingMode_VHACD;
		}

		const char* CuttingPanelName(int mode)
		{
			return IsConvexDecompositionMode(mode) ? "Convex Decomposition" : "Mesh Cutting";
		}

		int CuttingPanelRight(int width)
		{
			return std::min(290, width / 2);
		}

		void SetToolBackgroundColor(CameraDesc* camera)
		{
			if (camera != nullptr)
			{
				camera->BackgroundColor = Vector4(0.82f, 0.82f, 0.82f, 1.0f);
			}
		}

		void ConfigureToolCamera(CameraDesc* camera, float sceneScale, float cameraDistance)
		{
			if (camera == nullptr)
			{
				return;
			}

			const float scale = std::max(sceneScale, 1.0e-4f);
			const float distance = std::max(cameraDistance, scale * 1.0e-4f);
			camera->NearPlane = std::max(1.0e-4f, std::min(scale * 0.01f, distance * 0.1f));
			camera->FarPlane = std::max(distance + scale * 4.0f, camera->NearPlane * 2.0f);
		}

		void ConfigureToolLight(const CameraDesc& camera, DirectionalLightDesc* light)
		{
			if (light == nullptr)
			{
				return;
			}

			light->Direction = SafeUnitVector(camera.At - camera.Eye, Vector3::UnitZ());
			light->ShadowsEnabled = false;
		}

		bool IntersectRayBounds(const Ray3& Ray, const Vector3& BoundsMin,
			const Vector3& BoundsMax, float* NearHitTime, float* FarHitTime)
		{
			float NearTime = 0.0f;
			float FarTime = 1000.0f;
			for (int Axis = 0; Axis < 3; ++Axis)
			{
				if (std::fabs(Ray.Dir[Axis]) <= 1.0e-7f)
				{
					if (Ray.Origin[Axis] < BoundsMin[Axis]
						|| Ray.Origin[Axis] > BoundsMax[Axis]) return false;
					continue;
				}
				float T0 = (BoundsMin[Axis] - Ray.Origin[Axis]) / Ray.Dir[Axis];
				float T1 = (BoundsMax[Axis] - Ray.Origin[Axis]) / Ray.Dir[Axis];
				if (T1 < T0) std::swap(T0, T1);
				NearTime = std::max(NearTime, T0);
				FarTime = std::min(FarTime, T1);
				if (FarTime < NearTime) return false;
			}
			if (NearHitTime) *NearHitTime = NearTime;
			if (FarHitTime) *FarHitTime = FarTime;
			return FarTime >= 0.0f;
		}

		bool FindFluidSurfaceHit(const Ray3& Ray, const FluidSimulation& Simulation,
			float NearTime, float FarTime, Vector3* HitPoint)
		{
			if (HitPoint == nullptr || FarTime < NearTime)
			{
				return false;
			}
			const FluidSimulationSettings& Settings = Simulation.GetSettings();
			const Vector3& CellSize = Simulation.GetCellSize();
			const float StepSize = 0.5f * std::min(CellSize.x, std::min(CellSize.y, CellSize.z));
			const float DensityThreshold = 0.5f * Settings.RestDensity;
			float PreviousTime = NearTime;
			if (Simulation.SampleDensity(Ray.Origin + Ray.Dir * PreviousTime) >= DensityThreshold)
			{
				*HitPoint = Ray.Origin + Ray.Dir * PreviousTime;
				return true;
			}
			for (float Time = NearTime + StepSize; Time <= FarTime + StepSize; Time += StepSize)
			{
				const float SampleTime = std::min(Time, FarTime);
				if (Simulation.SampleDensity(Ray.Origin + Ray.Dir * SampleTime) >= DensityThreshold)
				{
					float LowTime = PreviousTime;
					float HighTime = SampleTime;
					for (int Iteration = 0; Iteration < 8; ++Iteration)
					{
						const float MiddleTime = 0.5f * (LowTime + HighTime);
						if (Simulation.SampleDensity(Ray.Origin + Ray.Dir * MiddleTime)
							>= DensityThreshold)
						{
							HighTime = MiddleTime;
						}
						else
						{
							LowTime = MiddleTime;
						}
					}
					*HitPoint = Ray.Origin + Ray.Dir * HighTime;
					return true;
				}
				if (SampleTime >= FarTime) break;
				PreviousTime = SampleTime;
			}
			return false;
		}

		bool ApplyFluidSplash(FluidSimulation& Simulation, const Vector3& SurfacePoint,
			float InteractionRadius, float InteractionStrength)
		{
			const FluidSimulationSettings& Settings = Simulation.GetSettings();
			const Vector3 Extent = Settings.BoundsMax - Settings.BoundsMin;
			const Vector3& CellSize = Simulation.GetCellSize();
			const float MinimumCellSize = std::min(CellSize.x, std::min(CellSize.y, CellSize.z));
			const float MinimumExtent = std::min(Extent.x, std::min(Extent.y, Extent.z));
			if (!std::isfinite(InteractionRadius) || !std::isfinite(InteractionStrength)
				|| !(InteractionRadius > 0.0f) || !(InteractionStrength > 0.0f)
				|| !(MinimumCellSize > 0.0f) || !(MinimumExtent > 0.0f))
			{
				return false;
			}

			const float SplashRadius = std::min(
				std::max(2.0f * InteractionRadius, 4.0f * MinimumCellSize),
				0.3f * MinimumExtent);
			const Vector3 SplashDirection = SafeUnitVector(Settings.Gravity * -1.0f,
				Vector3::UnitY());
			const Vector3 SplashCenter = SurfacePoint - SplashDirection * (0.25f * SplashRadius);
			const float SplashSpeed = std::max(3.0f, std::min(8.0f,
				4.0f * InteractionStrength / Settings.RestDensity));
			const float HemisphereVolume = 2.0943951f * SplashRadius * SplashRadius * SplashRadius;
			const float SplashMass = Settings.RestDensity * HemisphereVolume;
			const bool BroadSplash = Simulation.ApplyImpulse(SplashCenter, SplashRadius,
				SplashDirection * (SplashMass * SplashSpeed));

			const float CoreRadius = 0.55f * SplashRadius;
			const float CoreVolume = 2.0943951f * CoreRadius * CoreRadius * CoreRadius;
			const bool CoreSplash = Simulation.ApplyImpulse(SplashCenter, CoreRadius,
				SplashDirection * (Settings.RestDensity * CoreVolume * SplashSpeed * 0.75f));
			return BroadSplash || CoreSplash;
		}

		FluidBackend ParseFluidBackend(const std::string& Name)
		{
			if (Name == "simd") return FluidBackend::SIMD;
			if (Name == "cuda") return FluidBackend::CUDA;
			if (Name == "compute" || Name == "compute_shader")
				return FluidBackend::ComputeShader;
			return FluidBackend::CPU;
		}

		AerodynamicsSimulation* CreateSmokeWithFallback(PhysicsWorld* world,
			const AerodynamicsSimulationSettings& requested,
			const AerodynamicsRigidCouplingSettings& coupling, bool coupleRigids)
		{
			auto attempt = [&](FluidBackend backend, bool interactiveCPU) -> AerodynamicsSimulation*
			{
				AerodynamicsSimulationSettings settings = requested;
				settings.Backend = backend;
				settings.AllowBackendFallback = false;
				if (interactiveCPU)
				{
					settings.Resolution = Vector3i(std::min(settings.Resolution.x, 48),
						std::min(settings.Resolution.y, 80), std::min(settings.Resolution.z, 48));
					settings.PressureIterations = std::min(settings.PressureIterations, 24);
				}
				return world->CreateAerodynamics(settings, coupling, coupleRigids, true);
			};
			if (AerodynamicsSimulation* simulation = attempt(requested.Backend, false)) return simulation;
			if (!requested.AllowBackendFallback) return nullptr;
			if (requested.Backend != FluidBackend::ComputeShader)
				if (AerodynamicsSimulation* simulation = attempt(FluidBackend::ComputeShader, false)) return simulation;
			if (requested.Backend != FluidBackend::CUDA)
				if (AerodynamicsSimulation* simulation = attempt(FluidBackend::CUDA, false)) return simulation;
			if (requested.Backend != FluidBackend::CPU) return attempt(FluidBackend::CPU, true);
			return nullptr;
		}
	}

	WorldViewer::WorldViewer(RenderThread* renderThread, const std::string& sceneFile)
		: m_World(new SceneWorld())
		, m_FluidCoupling(nullptr)
		, m_AerodynamicsCoupling(nullptr)
		, m_RenderThread(renderThread)
		, m_HighlightedGeometry(nullptr)
		, m_SceneDragActive(false)
		, m_FieldDragActive(false)
		, m_TankDragActive(false)
		, m_SceneDragPlanePoint(Vector3::Zero())
		, m_SceneDragPlaneNormal(Vector3::UnitZ())
		, m_SceneDragOffset(Vector3::Zero())
		, m_SceneDragLastTime(std::chrono::steady_clock::now())
		, m_FieldDragPoint(Vector3::Zero())
		, m_TankDragX(0)
		, m_TankDragY(0)
		, m_TankTiltX(0.0f)
		, m_TankTiltZ(0.0f)
		, m_TankRotation(Quaternion::One())
		, m_TankCenter(Vector3::Zero())
		, m_CamCenter(Vector3::Zero())
		, m_CamParam(Vector3(1.0f, 0.6f, 15.0f))
		, m_CuttingPanelActive(false)
		, m_MeshSimplificationPanelActive(false)
		, m_CuttingObjIndex(0)
		, m_CuttingMode(MeshCuttingMode_VoronoiFracture3D)
		, m_CuttingPieceCount(16)
		, m_CuttingPiecesX(4)
		, m_CuttingPiecesY(3)
		, m_CuttingPiecesZ(2)
		, m_CuttingSeed(7)
		, m_MeshSimplificationFileIndex(0)
		, m_MeshSimplificationAlgorithm(MeshSimplificationAlgorithm_HoppeQEM)
		, m_MeshSimplificationRatioLevel(1)
		, m_CuttingSeparation(0.0f)
		, m_CuttingSceneScale(1.0f)
		, m_MeshSimplificationMaxSeparation(1.0f)
		, m_ShowGeometryQueryBounds(false)
		, m_RenderMode(RenderMode::Solid)
		, m_RenderRevision(0)
		, m_FluidDemoActive(false)
		, m_FluidRenderFrame(0)
		, m_GasDemoActive(false)
		, m_GasRenderFrame(0)
		, m_ImguiDemoIndex(0)
		, m_ImguiObjectCount(0)
		, m_ImguiCameraDistance(15.0f)
		, m_ImguiCuttingPanelActive(false)
		, m_ImguiMeshSimplificationPanelActive(false)
		, m_ImguiCuttingObjIndex(0)
		, m_ImguiCuttingMode(MeshCuttingMode_VoronoiFracture3D)
		, m_ImguiCuttingPieceCount(16)
		, m_ImguiCuttingPiecesX(4)
		, m_ImguiCuttingPiecesY(3)
		, m_ImguiCuttingPiecesZ(2)
		, m_ImguiCuttingSeed(7)
		, m_ImguiMeshSimplificationFileIndex(0)
		, m_ImguiMeshSimplificationAlgorithm(MeshSimplificationAlgorithm_HoppeQEM)
		, m_ImguiMeshSimplificationRatioLevel(1)
		, m_ImguiCuttingSeparation(0.0f)
		, m_ImguiShowGeometryQueryBounds(false)
		, m_ImguiRenderMode(RenderMode::Solid)
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
		, m_ImguiPendingMeshSimplificationFileIndex(-1)
		, m_ImguiPendingBrowseMeshSimplificationFile(false)
		, m_ImguiPendingApplyMeshSimplification(false)
		, m_ImguiPendingMeshSimplificationAlgorithm(false)
		, m_ImguiPendingMeshSimplificationAlgorithmValue(MeshSimplificationAlgorithm_HoppeQEM)
		, m_ImguiPendingMeshSimplificationRatioLevel(false)
		, m_ImguiPendingMeshSimplificationRatioLevelValue(1)
		, m_ImguiPendingShowGeometryQueryBounds(false)
		, m_ImguiPendingShowGeometryQueryBoundsValue(false)
		, m_ImguiPendingRenderMode(false)
		, m_ImguiPendingRenderModeValue(RenderMode::Solid)
		, m_ImguiPendingShootProjectile(false)
		, m_PhysicsFpsTime(std::chrono::steady_clock::now())
		, m_PhysicsFrameCount(0)
		, m_PhysicsFps(0.0)
	{
#ifdef RIEMANN_SPIDER_TRAINING
		m_SpiderTraining = std::make_unique<SpiderTrainingController>();
#endif
		for (bool& key : m_MovementKeys)
		{
			key = false;
		}
		// Probe devices on the simulation thread once, never from the render callback.
		for (int solver = 0; solver < 4; ++solver)
			for (int backend = 0; backend < 4; ++backend)
				m_ImguiSolverPanel.Available[solver][backend] = PhysicsWorld::IsSolverBackendAvailable(
					SolverPanelAlgorithms[solver], SolverPanelBackends[backend]);

		RefreshSceneList();
		RefreshObjList();
		RefreshMeshFileList();
		std::string initialScene = sceneFile;
		m_World->SetRobotWalkMode(RobotWalkMode::Policy);
		m_World->SetClothModelDemo(true);
		if (initialScene.empty() && !m_SceneFiles.empty())
		{
			initialScene = m_SceneFiles.front();
		}

		if (!LoadScene(initialScene))
		{
			LoadScene(m_SceneFiles.empty() ? ResolveSceneFile("01_demo.json") : m_SceneFiles.front());
		}
#ifdef RIEMANN_SPIDER_TRAINING
		LoadElmanExecutablePath();
#endif

		m_KeyboardEvent = [this](char c)
		{
			if (c != 'i')
			{
				return;
			}
			{
				std::lock_guard<std::mutex> Lock(m_ImguiMutex);
				if (m_CharacterInputActive) return;
			}
			if (m_CuttingPanelActive || m_MeshSimplificationPanelActive)
			{
				return;
			}

			ShootProjectile();
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
#ifdef RIEMANN_SPIDER_TRAINING
		if (m_SpiderTraining) m_SpiderTraining->stop();
#endif
		DeactivateGasDemo();
		DeactivateFluidDemo();
		if (m_RenderThread)
		{
			m_RenderThread->Submit([](Renderer& renderer) {
				renderer.SetImguiDrawCallback(nullptr, nullptr);
				renderer.Reset();
			});
		}
	}

	bool WorldViewer::ActivateConfiguredFieldSimulation()
	{
		const SceneFieldSimulationDesc& desc = m_World->GetFieldSimulation();
		if (desc.Type == "snow_simulation_3d") return ActivateSnowDemo();
		if (desc.Type == "sand_simulation_3d") return ActivateSnowDemo(true);
		if (!desc.IsConfigured())
		{
			return true;
		}

		const FluidSimulationMethod method = desc.Method == "lagrangian"
			? FluidSimulationMethod::Lagrangian : FluidSimulationMethod::Eulerian;
		if (desc.Type == "fluid3d" || desc.Type == "fluid_simulation_3d")
		{
			FluidSimulationSettings settings;
			settings.Method = method;
			settings.Backend = ParseFluidBackend(desc.Backend);
			settings.AllowBackendFallback = desc.AllowBackendFallback;
			settings.BoundsMin = desc.BoundsMin;
			settings.BoundsMax = desc.BoundsMax;
			settings.Resolution = desc.Resolution;
			settings.Gravity = desc.Gravity;
			settings.RestDensity = desc.RestDensity;
			settings.DynamicViscosity = desc.DynamicViscosity;
			settings.PressureStiffness = desc.PressureStiffness;
			settings.ParticleSpacing = desc.ParticleSpacing;
			settings.SmoothingRadius = desc.SmoothingRadius;
			settings.BoundaryRestitution = desc.BoundaryRestitution;
			settings.BoundaryFriction = desc.BoundaryFriction;
			settings.OpenTopBoundary = desc.OpenTopBoundary;
			settings.MaxTimeStep = desc.MaxTimeStep;
			settings.MaxSubsteps = desc.MaxSubsteps;
			settings.PressureIterations = desc.PressureIterations;
			m_FluidSimulation.reset(new FluidSimulation(settings));
			if (!m_FluidSimulation->IsInitialized())
			{
				m_FluidSimulation.reset();
				return false;
			}
			for (const SceneFieldRegion& region : desc.Regions)
			{
				m_FluidSimulation->AddFluidBox(region.Min, region.Max,
					region.SetVelocity ? region.Velocity : Vector3::Zero());
			}
			if (desc.RigidCoupling)
			{
				m_FluidCoupling = new FluidRigidCoupling(m_FluidSimulation.get());
				if (!m_FluidCoupling->IsValid()
					|| !m_World->GetSimulation()->AddForceField(m_FluidCoupling))
				{
					delete m_FluidCoupling;
					m_FluidCoupling = nullptr;
					m_FluidSimulation.reset();
					return false;
				}
			}
			m_FluidDemoActive = true;
			m_FluidRenderFrame = 0;
			return true;
		}

		if (desc.Type == "aerodynamics3d")
		{
			const auto& Settings = desc.Aerodynamics;
			if (Settings.Algorithm == AerodynamicsAlgorithm::Incompressible
				&& (!m_RenderThread || !m_RenderThread->SupportsDensityVolume() || !desc.Regions.empty())) return false;
			AerodynamicsRigidCouplingSettings Coupling;
			Coupling.SurfaceSamplesPerAxis = desc.SurfaceSamplesPerAxis; Coupling.DragCoefficient = desc.DragCoefficient;
			Coupling.MaxLinearAcceleration = desc.MaxLinearAcceleration; Coupling.MaxAngularAcceleration = desc.MaxAngularAcceleration;
			m_AerodynamicsSimulation = Settings.Algorithm == AerodynamicsAlgorithm::Incompressible
				? CreateSmokeWithFallback(m_World->GetSimulation(), Settings, Coupling, desc.RigidCoupling)
				: m_World->GetSimulation()->CreateAerodynamics(Settings, Coupling, desc.RigidCoupling, true);
			if (!m_AerodynamicsSimulation) return false;
			m_AerodynamicsCoupling = m_World->GetSimulation()->GetAerodynamicsCoupling();
			for (const auto& Region : desc.Regions)
			{
				if (Region.SetDensity) m_AerodynamicsSimulation->SetDensityBox(Region.Min, Region.Max, Region.Density);
				if (Region.SetVelocity) m_AerodynamicsSimulation->SetVelocityBox(Region.Min, Region.Max, Region.Velocity);
			}
			m_GasDemoActive = true; m_GasRenderFrame = 0; m_AerodynamicsPaused = false;
			return true;
		}

		return false;
	}

	void WorldViewer::DeactivateFluidDemo()
	{
		const bool wasActive = m_FluidDemoActive;
		if (m_FluidCoupling != nullptr && m_World != nullptr && m_World->GetSimulation() != nullptr)
		{
			m_World->GetSimulation()->RemoveForceField(m_FluidCoupling);
		}
		m_FluidCoupling = nullptr;
		m_FluidSimulation.reset();
		m_FluidDemoActive = false;
		m_FluidRenderFrame = 0;
		if (wasActive && m_RenderThread != nullptr)
		{
			m_RenderThread->Submit([](Renderer& renderer) {
				renderer.DeleteMesh("fluid_demo.water");
				renderer.DeleteMesh("fluid_demo.tank");
			});
		}
	}


	void WorldViewer::UpdateFluidDemo()
	{
		if (!m_FluidDemoActive || m_FluidSimulation == nullptr) return;
		m_FluidSimulation->Step(1.0f / 60.0f);
		if ((++m_FluidRenderFrame & 1) == 0) SubmitFluidSurface();
	}

	void WorldViewer::SubmitFluidSurface()
	{
		if (!m_FluidDemoActive || m_FluidSimulation == nullptr || m_RenderThread == nullptr) return;
		std::shared_ptr<RenderMeshDesc> surface(new RenderMeshDesc());
		if (!BuildWaterSurfaceMesh(*m_FluidSimulation, "fluid_demo.water", surface.get())) return;
		surface->WorldTransform = m_FluidSimulation->GetContainerTransform()
			* surface->WorldTransform;
		if (surface->Vertices.size() > (size_t)std::numeric_limits<int>::max()
			|| surface->Indices.size() > (size_t)std::numeric_limits<int>::max()) return;
		m_RenderThread->Submit([surface](Renderer& renderer) {
			RenderTransformUpdate transform;
			transform.Id = surface->Id;
			transform.WorldTransform = surface->WorldTransform;
			renderer.UpdateTransform(transform);
			renderer.UpdateMeshGeometry(surface->Id.c_str(),
				surface->Vertices.empty() ? nullptr : surface->Vertices.data(),
				(int)surface->Vertices.size(),
				surface->Indices.empty() ? nullptr : surface->Indices.data(),
				(int)surface->Indices.size());
		});
	}

	void WorldViewer::DeactivateGasDemo()
	{
		DeactivateSnowDemo();
		const bool wasActive = m_GasDemoActive;
		if (m_World && m_World->GetSimulation()) m_World->GetSimulation()->RemoveAerodynamics();
		m_AerodynamicsCoupling = nullptr; m_AerodynamicsSimulation = nullptr;
		m_GasDemoActive = false;
		m_GasRenderFrame = 0;
		if (wasActive && m_RenderThread != nullptr)
		{
			m_RenderThread->Submit([](Renderer& renderer) {
				renderer.ClearDensityVolume();
				renderer.DeleteMesh("gas_demo.density.compressed");
				renderer.DeleteMesh("gas_demo.density.rarefied");
			});
		}
	}


	void WorldViewer::UpdateGasDemo(float DeltaTime)
	{
		if (!m_GasDemoActive || !m_AerodynamicsSimulation || m_AerodynamicsPaused) return;
		(void)DeltaTime; // PhysicsWorld has already advanced the coupled simulation.
		if (!m_AerodynamicsSimulation->IsInitialized())
		{
			m_AerodynamicsPaused = true;
			std::lock_guard<std::mutex> Lock(m_ImguiMutex); m_ImguiAerodynamicsPanel.Status = "Aerodynamics device error"; return;
		}
		++m_GasRenderFrame; SubmitGasDensity();
	}

	void WorldViewer::SubmitGasDensity()
	{
		if (!m_GasDemoActive || m_AerodynamicsSimulation == nullptr || m_RenderThread == nullptr) return;
		if (m_RenderThread->SupportsDensityVolume())
		{
			std::shared_ptr<DensityVolumeDesc> volume(new DensityVolumeDesc());
			if (!BuildAerodynamicsDensityVolume(*m_AerodynamicsSimulation, volume.get(),
				m_World->GetFieldSimulation().VisualizationThreshold, m_World->GetFieldSimulation().TracerExtinction, m_World->GetFieldSimulation().TracerColor)) return;
			m_RenderThread->Submit([volume](Renderer& renderer) { renderer.UpdateDensityVolume(*volume); });
			return;
		}
		if ((m_GasRenderFrame & 1) != 0) return;
		std::shared_ptr<std::vector<RenderMeshDesc>> meshes(new std::vector<RenderMeshDesc>());
		GasDensityVisualizationSettings visualization;
		visualization.RelativeThreshold = 0.08f;
		if (!BuildGasDensityMeshes(*m_AerodynamicsSimulation,
			"gas_demo.density", meshes.get(), visualization)) return;
		m_RenderThread->Submit([meshes](Renderer& renderer) {
			for (const RenderMeshDesc& mesh : *meshes)
			{
				if (mesh.Vertices.size() > (size_t)std::numeric_limits<int>::max()
					|| mesh.Indices.size() > (size_t)std::numeric_limits<int>::max()) continue;
				RenderTransformUpdate transform;
				transform.Id = mesh.Id;
				transform.WorldTransform = mesh.WorldTransform;
				renderer.UpdateTransform(transform);
				renderer.UpdateMeshGeometry(mesh.Id.c_str(),
					mesh.Vertices.empty() ? nullptr : mesh.Vertices.data(),
					(int)mesh.Vertices.size(),
					mesh.Indices.empty() ? nullptr : mesh.Indices.data(),
					(int)mesh.Indices.size());
			}
		});
	}

	void WorldViewer::ShootProjectile()
	{
		if (m_FluidDemoActive || m_GasDemoActive) return;
		static int projectileId = 0;
		const Vector3 p0 = GetCameraPosition();
		const Vector3 direction = SafeUnitVector(m_CamCenter - p0, Vector3::UnitZ());

		std::ostringstream ss;
		ss << "projectile_" << projectileId++;
		Geometry* geometry = m_World->AddProjectileSphere(ss.str(), p0, 0.5f, direction * 18.0f, Vector4(0.95f, 0.58f, 0.20f, 1.0f));
		if (geometry)
		{
			const SceneObjectInstance& instance = m_World->GetObjects().back();
			AddGeometryToRender(instance.Id, geometry, instance.Color, instance.RenderBounds);
			UpdateImguiState();
		}
	}

	bool WorldViewer::LoadScene(const std::string& sceneFile)
	{
#ifdef RIEMANN_SPIDER_TRAINING
		if (m_SpiderTrainingMode && m_SpiderTraining)
		{
			m_SpiderTraining->stop();
			m_SpiderTrainingMode = false;
		}
#endif
		const std::string resolved = ResolveSceneFile(sceneFile.c_str());
		if (resolved.empty())
		{
			return false;
		}
		DeactivateGasDemo();
		DeactivateFluidDemo();

		++m_SceneGeneration;
		std::string error;
		if (FileNameOf(resolved) == "09_ragdoll_stairs.json")
			m_World->SetRagdollAlgorithmOverride(m_RagdollDemoReduced
				? RagdollAlgorithm::ReducedCoordinates : RagdollAlgorithm::JointConstraints);
		else m_World->ClearRagdollAlgorithmOverride();
		if (!m_World->LoadFromFile(resolved, &error))
		{
			fprintf(stderr, "%s\n", error.c_str());
			return false;
		}
		if (!ActivateConfiguredFieldSimulation())
		{
			fprintf(stderr, "failed to initialize field simulation from %s\n", resolved.c_str());
			return false;
		}

		m_SceneDirectory = DirectoryOf(resolved);
		m_CurrentSceneName = FileNameOf(resolved);
		m_ViewerPaused = m_World->GetVehicleCount() > 0;
		m_VehicleSlowMotion = false;
		m_VehiclePendingTime = 0;
		m_CurrentSceneDisplayName = m_World->GetName().empty() ? m_CurrentSceneName : m_World->GetName();
		{
			std::lock_guard<std::mutex> Lock(m_ImguiMutex);
			std::fill(std::begin(m_MovementKeys), std::end(m_MovementKeys), false);
			std::fill(std::begin(m_RobotKeys), std::end(m_RobotKeys), false);
		}
		if (m_World->IsSpiderWalkingDemo()) m_World->SetRobotAutoWalk(m_RobotAutoWalk);
		m_SceneDragActive = false;
		m_FieldDragActive = false;
		m_TankDragActive = false;
		m_TerrainDragActive = false;
		m_TankTiltX = 0.0f;
		m_TankTiltZ = 0.0f;
		m_TankRotation = Quaternion::One();
		const SceneFieldSimulationDesc& Field = m_World->GetFieldSimulation();
		m_TankCenter = Field.IsConfigured()
			? 0.5f * (Field.BoundsMin + Field.BoundsMax) : Vector3::Zero();
		m_CuttingPanelActive = false;
		m_MeshSimplificationPanelActive = false;
		m_CuttingPieces.clear();
		m_CuttingSeparation = 0.0f;
		RefreshSceneList();
		RefreshObjList();
		RefreshMeshFileList();
		ApplySceneCamera();
		m_HighlightedGeometry = nullptr;
		RebuildRenderScene();
		UpdateImguiState();
		return true;
	}

	void WorldViewer::CreateSimulator()
	{
		m_ViewerPaused = false;
		DeactivateGasDemo();
		DeactivateFluidDemo();
		++m_SceneGeneration;
		m_World->Reset();
		m_RenderBindings.clear();
		m_CuttingPanelActive = false;
		m_MeshSimplificationPanelActive = false;
		m_CuttingPieces.clear();
		m_HighlightedGeometry = nullptr;
		RebuildRenderScene();
	}

	void WorldViewer::UpdateSimulator(float dt)
	{
		ApplyImguiCommands();
		if (m_ViewerPaused)
		{
			m_PhysicsFps = 0.0;
			UpdateImguiState();
			return;
		}
#ifdef RIEMANN_SPIDER_TRAINING
		if (m_SpiderTraining && m_RenderThread)
			m_SpiderTraining->set_display_active(m_RenderThread->IsActive());
#endif
		Vector3 CharacterInput = Vector3::Zero(), RobotInput = Vector3::Zero(); bool JumpRequested;
		{
			std::lock_guard<std::mutex> Lock(m_ImguiMutex);
			CharacterInput.x = float(m_CharacterKeys[3]) - float(m_CharacterKeys[1]);
			CharacterInput.z = float(m_CharacterKeys[2]) - float(m_CharacterKeys[0]);
			JumpRequested = m_CharacterJumpRequested; m_CharacterJumpRequested = false;
			RobotInput = Vector3(float(m_RobotKeys[0]) - float(m_RobotKeys[2]),
				float(m_RobotKeys[1]) - float(m_RobotKeys[3]), float(m_RobotKeys[5]) - float(m_RobotKeys[4]));
		}
		if (m_World && m_World->IsSpiderWalkingDemo())
		{
			m_World->SetRobotAutoWalk(m_RobotAutoWalk);
			m_World->SetRobotDriveInput(RobotInput.x, RobotInput.y, RobotInput.z);
		}
		if (m_World && m_World->GetDemoCharacter())
		{
			const float MoveSpeed = 3.0f, JumpSpeed = 5.0f;
			m_World->GetDemoCharacter()->SetMoveVelocity(CharacterInput.SafeUnit() * MoveSpeed);
			if (JumpRequested) m_World->GetDemoCharacter()->Jump(JumpSpeed);
		}
		ApplyVehicleCommands();
		ApplySnowCommands();
		UpdateFluidDemo();
		if (m_World->GetVehicleCount() > 0)
		{
			const auto* vehicle = m_World->GetVehicle(0);
			const Vector3 previous = vehicle->GetWorldTransform().pos;
			// Slow playback changes elapsed time, not the crash solver's timestep.
			m_VehiclePendingTime = std::min(0.1f, m_VehiclePendingTime + dt * (m_VehicleSlowMotion ? 0.25f : 1.0f));
			while (m_VehiclePendingTime >= 1.0f / 120.0f)
			{
				m_World->Step(1.0f / 120.0f);
				m_VehiclePendingTime -= 1.0f / 120.0f;
			}
			if (vehicle->GetFrontBumper(0) && !m_World->GetVehicleDynamics(0))
			{
				const Vector3 travel = vehicle->GetWorldTransform().pos - previous;
				m_CamCenter += travel;
				if (!travel.IsZero()) UpdateCamera();
			}
		}
		else if (const RobotController* Robot = m_World->GetRobotDemoController())
		{
#ifdef RIEMANN_SPIDER_TRAINING
			if (m_SpiderTrainingMode && m_SpiderTraining && m_RenderThread->IsActive())
			{
				m_World->ShowSpiderTrainingSnapshot(m_SpiderTraining->snapshot(),
					m_SpiderTrainingShowAll, size_t(m_SpiderTrainingSelected));
			}
			else
#endif
			m_World->Step(dt);
			if (!m_RobotAutoWalk
#ifdef RIEMANN_SPIDER_TRAINING
				&& !m_SpiderTrainingMode
#endif
			)
			{
				m_CamCenter = Robot->GetPosition();
				UpdateCamera();
			}
		}
		else if (!m_AerodynamicsSimulation || !m_AerodynamicsPaused) m_World->Step(dt);
		UpdateGasDemo(dt);
		UpdateSnowDemo(dt);
		if (!m_RenderThread || m_RenderThread->IsActive())
		{
			if (m_RenderRevision != m_World->GetRenderRevision())
			{
				RebuildRenderScene();
			}
			else
			{
				SubmitTransforms();
				SubmitDeformableMeshes();
				SubmitGeometryQueryBounds();
			}
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

	void WorldViewer::SetViewportSize(int width, int height)
	{
		if (width <= 0 || height <= 0) return;
		m_ViewportWidth = width;
		m_ViewportHeight = height;
		if (m_CuttingPanelActive)
		{
			FrameCuttingBounds();
			UpdateCamera();
		}
	}

	CameraDesc WorldViewer::GetViewCamera() const
	{
		CameraDesc camera = m_World ? m_World->GetCamera() : CameraDesc();
		camera.Eye = GetCameraPosition();
		camera.At = m_CamCenter;
		if (m_CuttingPanelActive)
		{
			camera.FovY = 0.55f;
			const Vector3 forward = SafeUnitVector(camera.At - camera.Eye, Vector3::UnitZ());
			const Vector3 right = SafeUnitVector(camera.Up.Cross(forward), Vector3::UnitX());
			const float panel = (float)CuttingPanelRight(m_ViewportWidth);
			const Vector3 shift = right * (panel / m_ViewportHeight * m_CamParam.z * tanf(camera.FovY * 0.5f));
			camera.Eye -= shift;
			camera.At -= shift;
		}
		return camera;
	}

	void WorldViewer::FrameCuttingBounds()
	{
		const CameraDesc camera = GetViewCamera();
		const Vector3 forward = SafeUnitVector(camera.At - camera.Eye, Vector3::UnitZ());
		const Vector3 right = SafeUnitVector(camera.Up.Cross(forward), Vector3::UnitX());
		const Vector3 up = forward.Cross(right);

		// Measure the visible surfaces in orbit coordinates, including their current offsets.
		Box3 bounds = Box3::Empty();
		for (const auto& object : m_World->GetObjects())
		{
			const TriangleMesh* mesh = object.GeometryPtr->GetShapeObj<TriangleMesh>();
			if (!mesh || !object.RenderEnabled) continue;
			const Transform pose = GetGeometryWorldTransform(object.GeometryPtr);
			for (uint32_t i = 0; i < mesh->GetVertexCount(); ++i)
			{
				const Vector3 v = pose.LocalToWorld(mesh->Vertices[i]);
				bounds.Encapsulate(Vector3(v.Dot(right), v.Dot(up), v.Dot(forward)));
			}
		}
		if (bounds.Min.x > bounds.Max.x) return;
		const Vector3 center = bounds.GetCenter(), extent = bounds.GetExtent();
		m_CamCenter = right * center.x + up * center.y + forward * center.z;
		m_CuttingSceneScale = std::max(extent.Length() * 2.0f, 1.0e-4f);
		const float panel = (float)CuttingPanelRight(m_ViewportWidth);
		const float tanY = tanf(camera.FovY * 0.5f);
		const float tanX = tanY * (m_ViewportWidth - panel) / m_ViewportHeight;
		const float parallax = tanY * panel / m_ViewportHeight * extent.z;
		m_CamParam.z = extent.z + 1.13f * std::max((extent.x + parallax) / tanX, extent.y / tanY);
	}

	void WorldViewer::UpdateCamera()
	{
		if (m_RenderThread == nullptr)
		{
			return;
		}

		CameraDesc camera = GetViewCamera();
		if (m_CuttingPanelActive || m_MeshSimplificationPanelActive)
		{
			SetToolBackgroundColor(&camera);
			ConfigureToolCamera(&camera, GetCameraSceneScale(), m_CamParam.z);
			DirectionalLightDesc light = m_World->GetLight();
			ConfigureToolLight(camera, &light);
			m_RenderThread->Submit([camera, light](Renderer& renderer) {
				renderer.SetCamera(camera);
				renderer.SetLight(light);
			});
			return;
		}
		m_RenderThread->Submit([camera](Renderer& renderer) {
			renderer.SetCamera(camera);
		});
	}

	void WorldViewer::KeyboardMsg(char c)
	{
		c = static_cast<char>(tolower(static_cast<unsigned char>(c)));
		{
			std::lock_guard<std::mutex> Lock(m_ImguiMutex);
			if (m_RobotInputActive && (c == 'w' || c == 'a' || c == 's' || c == 'd' || c == 'q' || c == 'e')) return;
		}
		if (c == 'a' || c == 'd' || c == 'w' || c == 's')
		{
			return;
		}

		const float scale = GetCameraControlScale() * 0.1f;
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
		{
			std::lock_guard<std::mutex> Lock(m_ImguiMutex);
			const char Key = static_cast<char>(tolower(static_cast<unsigned char>(c)));
			const char RobotKeys[] = {'w', 'a', 's', 'd', 'q', 'e'};
			for (int I = 0; I < 6; ++I) if (Key == RobotKeys[I]) m_RobotKeys[I] = down;
			if (m_RobotInputActive) return;
			if (Key == 'i') m_CharacterKeys[0] = down;
			if (Key == 'j') m_CharacterKeys[1] = down;
			if (Key == 'k') m_CharacterKeys[2] = down;
			if (Key == 'l') m_CharacterKeys[3] = down;
			if (Key == ' ') { if (down && !m_CharacterJumpHeld) m_CharacterJumpRequested = true; m_CharacterJumpHeld = down; }
		}
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
		{
			std::lock_guard<std::mutex> Lock(m_ImguiMutex);
			if (m_RobotInputActive) return;
		}
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

		const float cameraSpeed = GetCameraControlScale() * 1.5f;
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

	bool WorldViewer::PaintTerrainAtRay(const Ray3& ray, bool firstStroke)
	{
		if (!m_World || m_CuttingPanelActive || m_MeshSimplificationPanelActive) return false;
		TerrainBrushState brush;
		{
			std::lock_guard<std::mutex> lock(m_ImguiMutex);
			brush = m_ImguiTerrainBrush;
		}
		if (brush.SceneGeneration != m_SceneGeneration || brush.ObjectId.empty()) return false;
		Vector3 point;
		float localHeight = 0.0f;
		if (!m_World->RayCastTerrain(brush.ObjectId, ray.Origin, ray.Dir, 1000.0f, &point, &localHeight)) return false;
		RayCastOption option;
		option.Type = RayCastOption::RAYCAST_NEAREST;
		option.HitBothSides = true;
		option.MaxDist = 1000.0f;
		RayCastResult nearest;
		GeometryQuery* query = m_World->GetSimulation()->GetGeometryQuery();
		if (query && query->RayCastQuery(ray.Origin, ray.Dir, option, &nearest)
			&& nearest.hitTime + 0.01f < (point - ray.Origin).Length()) return false;
		if (!firstStroke && (point - m_TerrainDragLastPoint).Length() < brush.Radius * 0.15f) return true;
		if (firstStroke) m_TerrainDragTargetHeight = localHeight;
		m_TerrainDragActive = true;
		m_TerrainDragLastPoint = point;
		const int mode = brush.Mode < 2 ? 0 : brush.Mode == 2 ? 1 : 2;
		const float value = brush.Mode == 2 ? m_TerrainDragTargetHeight
			: (brush.Mode == 1 ? -brush.Value : brush.Value) * brush.Strength;
		const int changedSamples = m_World->EditTerrain(brush.ObjectId, point,
			brush.Radius, value, mode, brush.Strength);
		char status[96];
		snprintf(status, sizeof(status), "Changed samples: %d", changedSamples);
		{
			std::lock_guard<std::mutex> lock(m_ImguiMutex);
			m_ImguiTerrainStatus = status;
		}
		SetHighlightedGeometry(nullptr);
		return true;
	}

	void WorldViewer::SceneRayMsg(int x, int y, int width, int height)
	{
		if (m_SnowSimulation || m_SandSimulation) { MoveSnowCapsule(x,y,width,height); return; }
		if (width <= 0 || height <= 0)
		{
			return;
		}

		const Ray3 ray = BuildSceneRay(x, y, width, height);
		m_SceneDragActive = false;
		m_FieldDragActive = false;
		m_TankDragActive = false;
		m_TerrainDragActive = false;
		if (PaintTerrainAtRay(ray, true)) return;
		Geometry* hitGeometry = nullptr;
		Vector3 hitPoint = Vector3::Zero();
		if (m_World && m_World->RayCastMouseControlGeometry(ray.Origin, ray.Dir, 1000.0f, &hitGeometry, &hitPoint)
			&& m_World->SelectMouseControlGeometry(hitGeometry))
		{
			m_SceneDragActive = true;
			m_SceneDragPlanePoint = hitPoint;
			m_SceneDragPlaneNormal = ray.Dir;
			m_SceneDragOffset = m_World->GetMouseControlPosition() - hitPoint;
			m_SceneDragLastTime = std::chrono::steady_clock::now();
			SetHighlightedGeometry(hitGeometry);
			return;
		}
		const SceneFieldSimulationDesc& Field = m_World->GetFieldSimulation();
		float FieldNearTime = 0.0f;
		float FieldFarTime = 0.0f;
		Ray3 FieldRay = ray;
		if (m_FluidSimulation)
		{
			const Transform& ContainerTransform = m_FluidSimulation->GetContainerTransform();
			FieldRay = Ray3(ContainerTransform.WorldToLocal(ray.Origin),
				ContainerTransform.WorldToLocalDirection(ray.Dir));
		}
		if (Field.IsConfigured() && IntersectRayBounds(FieldRay, Field.BoundsMin,
			Field.BoundsMax, &FieldNearTime, &FieldFarTime))
		{
			Vector3 FieldHitPoint = ray.Origin + ray.Dir * FieldNearTime;
			if (m_FluidSimulation)
			{
				if (FindFluidSurfaceHit(ray, *m_FluidSimulation,
					FieldNearTime, FieldFarTime, &FieldHitPoint))
				{
					if (ApplyFluidSplash(*m_FluidSimulation, FieldHitPoint,
						Field.InteractionRadius, Field.InteractionStrength))
					{
						m_FluidRenderFrame = 1;
					}
				}
				m_TankDragActive = true;
				m_TankDragX = x;
				m_TankDragY = y;
			}
			else
			{
				m_FieldDragActive = true;
				if (m_AerodynamicsSimulation && m_AerodynamicsSimulation->HasTracerField())
					FieldHitPoint = ray.Origin + ray.Dir * (0.5f * (FieldNearTime + FieldFarTime));
				m_SceneDragPlanePoint = FieldHitPoint;
				m_SceneDragPlaneNormal = ray.Dir;
				m_FieldDragPoint = m_SceneDragPlanePoint;
			}
			SetHighlightedGeometry(nullptr);
			return;
		}
		HandleSceneRay(ray);
		if (m_FluidSimulation)
		{
			m_TankDragActive = true;
			m_TankDragX = x;
			m_TankDragY = y;
		}
	}

	void WorldViewer::SceneDragMsg(int x, int y, int width, int height)
	{
		if (m_SnowSimulation || m_SandSimulation) { MoveSnowCapsule(x,y,width,height); return; }
		if (m_TerrainDragActive)
		{
			if (width > 0 && height > 0) PaintTerrainAtRay(BuildSceneRay(x, y, width, height), false);
			return;
		}
		if ((!m_SceneDragActive && !m_FieldDragActive && !m_TankDragActive)
			|| m_World == nullptr || width <= 0 || height <= 0)
		{
			return;
		}
		if (m_TankDragActive && m_FluidSimulation)
		{
			const int DeltaX = x - m_TankDragX;
			const int DeltaY = y - m_TankDragY;
			m_TankDragX = x;
			m_TankDragY = y;
			if (DeltaX == 0 && DeltaY == 0) return;
			m_TankTiltX = std::max(-1.35f, std::min(1.35f,
				m_TankTiltX + (float)DeltaY * 0.005f));
			m_TankTiltZ = std::max(-1.35f, std::min(1.35f,
				m_TankTiltZ - (float)DeltaX * 0.005f));
			Quaternion RotationX;
			RotationX.FromRotateX(m_TankTiltX);
			Quaternion RotationZ;
			RotationZ.FromRotateZ(m_TankTiltZ);
			const Quaternion NewRotation = (RotationZ * RotationX).Unit();
			const Quaternion DeltaRotation = (NewRotation * m_TankRotation.Conjugate()).Unit();
			if (m_World->RotateTank(m_TankCenter, DeltaRotation))
			{
				m_TankRotation = NewRotation;
				const Transform ContainerTransform(
					m_TankCenter - NewRotation * m_TankCenter, NewRotation);
				m_FluidSimulation->SetContainerTransform(ContainerTransform);
				m_FluidRenderFrame = 1;
				if (m_RenderThread)
				{
					const Transform WaterTransform = ContainerTransform * Transform(m_TankCenter);
					m_RenderThread->Submit([ContainerTransform, WaterTransform](Renderer& renderer) {
						RenderTransformUpdate tankUpdate;
						tankUpdate.Id = "fluid_demo.tank";
						tankUpdate.WorldTransform = ContainerTransform;
						renderer.UpdateTransform(tankUpdate);
						RenderTransformUpdate waterUpdate;
						waterUpdate.Id = "fluid_demo.water";
						waterUpdate.WorldTransform = WaterTransform;
						renderer.UpdateTransform(waterUpdate);
					});
				}
			}
			return;
		}
		const Ray3 ray = BuildSceneRay(x, y, width, height);
		const float denominator = ray.Dir.Dot(m_SceneDragPlaneNormal);
		if (std::abs(denominator) <= 1.0e-5f)
		{
			return;
		}
		const float hitTime = (m_SceneDragPlanePoint - ray.Origin).Dot(m_SceneDragPlaneNormal) / denominator;
		if (hitTime >= 0.0f)
		{
			const Vector3 HitPoint = ray.Origin + ray.Dir * hitTime;
			if (m_FieldDragActive)
			{
				const SceneFieldSimulationDesc& Field = m_World->GetFieldSimulation();
				const Vector3 Delta = HitPoint - m_FieldDragPoint;
				Vector3 Impulse = Delta * Field.InteractionStrength;
				if (m_FluidSimulation)
				{
					Impulse.y += 0.5f * Delta.Length() * Field.InteractionStrength;
				}
				if (!Impulse.IsZero())
				{
					const Vector3 InteractionPoint(
						std::max(Field.BoundsMin.x, std::min(HitPoint.x, Field.BoundsMax.x)),
						m_FluidSimulation ? m_SceneDragPlanePoint.y
							: std::max(Field.BoundsMin.y, std::min(HitPoint.y, Field.BoundsMax.y)),
						std::max(Field.BoundsMin.z, std::min(HitPoint.z, Field.BoundsMax.z)));
					if (m_FluidSimulation && m_FluidSimulation->ApplyImpulse(
						InteractionPoint, Field.InteractionRadius, Impulse))
					{
						m_FluidRenderFrame = 1;
					}
					if (m_AerodynamicsSimulation)
					{
						if (m_AerodynamicsSimulation->HasTracerField()) m_AerodynamicsSimulation->ApplyVelocityImpulse(InteractionPoint, Field.InteractionRadius, Impulse);
						else m_AerodynamicsSimulation->ApplyImpulse(InteractionPoint, Field.InteractionRadius, Impulse);
					}
				}
				m_FieldDragPoint = HitPoint;
				return;
			}
			const std::chrono::steady_clock::time_point Now = std::chrono::steady_clock::now();
			const float DeltaTime = std::chrono::duration<float>(Now - m_SceneDragLastTime).count();
			m_SceneDragLastTime = Now;
			const Vector3 TargetPosition = HitPoint + m_SceneDragOffset;
			const SceneFieldSimulationDesc& Field = m_World->GetFieldSimulation();
			if (Field.IsConfigured())
			{
				if (m_FluidSimulation)
				{
					m_World->SetMouseControlPosition(TargetPosition, DeltaTime,
						Field.BoundsMin, Field.BoundsMax,
						m_FluidSimulation->GetContainerTransform());
				}
				else
				{
					m_World->SetMouseControlPosition(TargetPosition, DeltaTime,
						Field.BoundsMin, Field.BoundsMax);
				}
			}
			else
			{
				m_World->SetMouseControlPosition(TargetPosition, DeltaTime);
			}
		}
	}

	void WorldViewer::SceneDragEndMsg()
	{
		if (m_SceneDragActive && m_World != nullptr) m_World->ReleaseMouseControl();
		m_SceneDragActive = false;
		m_FieldDragActive = false;
		m_TankDragActive = false;
		m_TerrainDragActive = false;
	}

	void WorldViewer::MouseWheel(int zDelta, bool ctrlButtonDown)
	{
		if (zDelta == 0)
		{
			return;
		}

		const float sceneScale = GetCameraSceneScale();
		const float wheelSteps = static_cast<float>(zDelta) / 120.0f;
		const float speedMultiplier = ctrlButtonDown ? 4.0f : 1.0f;
		const float zoomFactor = powf(1.25f, -wheelSteps * speedMultiplier);
		const float minDistance = sceneScale * 1.0e-4f;
		const float maxDistance = sceneScale * 1.0e6f;
		m_CamParam.z = std::max(minDistance, std::min(m_CamParam.z * zoomFactor, maxDistance));
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
		const int panelRight = m_CuttingPanelActive ? CuttingPanelRight(width) : std::min(width - 10, panelLeft + 280);
		const int panelBottom = height - 10;
		return x >= panelLeft && x <= panelRight && y >= panelTop && y <= panelBottom;
	}

	void WorldViewer::DrawImgui(int width, int height)
	{
		std::string sceneName;
		bool ragdollDemoReduced = false;
		RobotPanelState robotPanel;
		std::vector<std::string> sceneFiles;
		int demoIndex = 0;
		size_t objectCount = 0;
		float cameraDistance = 0.0f;
		bool cuttingPanelActive = false;
		bool meshSimplificationPanelActive = false;
		std::string cuttingObjPath;
		std::string cuttingStatus;
		std::vector<ConstraintControlState> constraintControls;
		std::vector<std::string> terrainObjectIds;
		TerrainBrushState terrainBrush;
		std::string terrainStatus;
		SolverPanelState solverPanel;
		AerodynamicsPanelState aerodynamicsPanel;
		SnowPanelState snowPanel;
		VehiclePanelState vehiclePanel;
		bool viewerPaused = false;
		std::string meshSimplificationPath;
		std::string meshSimplificationStatus;
#ifdef RIEMANN_SPIDER_TRAINING
		std::string elmanExecutablePath;
		std::string elmanExecutableStatus;
#endif
		int cuttingMode = 0;
		int cuttingPieceCount = 0;
		int cuttingPiecesX = 0;
		int cuttingPiecesY = 0;
		int cuttingPiecesZ = 0;
		int cuttingSeed = 0;
		bool capCutFaces = true;
		int meshSimplificationAlgorithm = MeshSimplificationAlgorithm_HoppeQEM;
		int meshSimplificationRatioLevel = 1;
		float cuttingSeparation = 0.0f;
		bool showGeometryQueryBounds = false;
		RenderMode renderMode = RenderMode::Solid;
		size_t geometryQueryBoundsCount = 0;
		double physicsFps = 0.0;
		{
			std::lock_guard<std::mutex> lock(m_ImguiMutex);
			sceneName = m_ImguiSceneName;
			ragdollDemoReduced = m_ImguiRagdollDemoReduced;
			robotPanel = m_ImguiRobotPanel;
			sceneFiles = m_ImguiSceneFiles;
			demoIndex = m_ImguiDemoIndex;
			objectCount = m_ImguiObjectCount;
			cameraDistance = m_ImguiCameraDistance;
			cuttingPanelActive = m_ImguiCuttingPanelActive;
			meshSimplificationPanelActive = m_ImguiMeshSimplificationPanelActive;
			cuttingObjPath = m_ImguiCuttingObjPath;
			cuttingStatus = m_ImguiCuttingStatus;
			meshSimplificationPath = m_ImguiMeshSimplificationPath;
			meshSimplificationStatus = m_ImguiMeshSimplificationStatus;
#ifdef RIEMANN_SPIDER_TRAINING
			elmanExecutablePath = m_ElmanExecutablePath;
			elmanExecutableStatus = m_ElmanExecutableStatus;
#endif
			constraintControls = m_ImguiConstraintControls;
			terrainObjectIds = m_ImguiTerrainObjectIds;
			terrainBrush = m_ImguiTerrainBrush;
			terrainStatus = m_ImguiTerrainStatus;
			solverPanel = m_ImguiSolverPanel;
			aerodynamicsPanel = m_ImguiAerodynamicsPanel;
			snowPanel = m_ImguiSnowPanel;
			vehiclePanel = m_ImguiVehiclePanel;
			viewerPaused = m_ImguiViewerPaused;
			cuttingMode = m_ImguiCuttingMode;
			cuttingPieceCount = m_ImguiCuttingPieceCount;
			cuttingPiecesX = m_ImguiCuttingPiecesX;
			cuttingPiecesY = m_ImguiCuttingPiecesY;
			cuttingPiecesZ = m_ImguiCuttingPiecesZ;
			cuttingSeed = m_ImguiCuttingSeed;
			capCutFaces = m_ImguiCapCutFaces;
			meshSimplificationAlgorithm = m_ImguiMeshSimplificationAlgorithm;
			meshSimplificationRatioLevel = m_ImguiMeshSimplificationRatioLevel;
			cuttingSeparation = m_ImguiCuttingSeparation;
			showGeometryQueryBounds = m_ImguiShowGeometryQueryBounds;
			renderMode = m_ImguiRenderMode;
			geometryQueryBoundsCount = m_ImguiGeometryQueryBoundsCount;
			physicsFps = m_ImguiPhysicsFps;
		}

		char fpsText[128];
		const double renderFps = m_RenderThread ? m_RenderThread->GetRenderFps() : 0.0;
		const bool meshTool = cuttingPanelActive || meshSimplificationPanelActive;
		if (!meshTool)
		{
			snprintf(fpsText, sizeof(fpsText), "Physics: %.1f FPS", physicsFps);
			imguiDrawText(width - 10, 12, IMGUI_ALIGN_RIGHT, fpsText, imguiRGBA(230, 235, 240, 255));
		}
		snprintf(fpsText, sizeof(fpsText), "Render: %.1f FPS", renderFps);
		imguiDrawText(width - 10, meshTool ? 12 : 30, IMGUI_ALIGN_RIGHT, fpsText, imguiRGBA(230, 235, 240, 255));
		if (m_FluidDemoActive)
		{
			imguiDrawText(width - 10, 48, IMGUI_ALIGN_RIGHT,
				"LMB click water: large splash; LMB drag empty: tilt tank",
				imguiRGBA(150, 210, 255, 255));
		}

		const int panelHeight = std::max(120, height - 20);
		if (m_ImguiRenderedSceneGeneration != solverPanel.SceneGeneration)
		{
			m_ImguiScroll = 0;
			m_ImguiRenderedSceneGeneration = solverPanel.SceneGeneration;
		}
		const int panelWidth = cuttingPanelActive ? std::max(0, CuttingPanelRight(width) - 10) : std::min(280, width - 20);
		imguiBeginScrollArea("", 10, 10, panelWidth, panelHeight, &m_ImguiScroll);

		char text[128];
		imguiLabel("Demo");
		std::vector<std::string> demoNames = sceneFiles;
		demoNames.push_back("Mesh Cutting");
		demoNames.push_back("Convex Decomposition");
		demoNames.push_back("Mesh Simplification");
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
			if (imguiCombo("", &selectedDemo, demoLabels.data(), (int)demoLabels.size()))
			{
				std::lock_guard<std::mutex> lock(m_ImguiMutex);
				m_ImguiPendingDemoIndex = selectedDemo;
			}
		}
		if (imguiButton("Restart"))
		{
			std::lock_guard<std::mutex> lock(m_ImguiMutex);
			m_ImguiPendingRestartScene = true;
			m_ImguiPendingRestartGeneration = solverPanel.SceneGeneration;
		}
		if (imguiButton(viewerPaused ? "Resume" : "Pause"))
		{
			std::lock_guard<std::mutex> lock(m_ImguiMutex);
			m_ImguiPendingPauseToggle = true;
			m_ImguiPendingPauseGeneration = solverPanel.SceneGeneration;
		}
		imguiSeparatorLine();
		if (robotPanel.Active)
		{
			bool changed = false;
			if (robotPanel.Interactive)
			{
#ifdef RIEMANN_SPIDER_TRAINING
				if (robotPanel.WalkMode == RobotWalkMode::Policy)
				{
					imguiLabel("ELMAN EXECUTABLE");
					if (elmanExecutablePath.empty()) imguiValue("No Elman executable selected");
					else for (size_t offset = 0; offset < elmanExecutablePath.size(); offset += 34)
						imguiValue(elmanExecutablePath.substr(offset, 34).c_str());
					if (imguiButton("Select Elman EXE..."))
					{
						std::lock_guard<std::mutex> lock(m_ImguiMutex);
						m_PendingBrowseElmanExecutable = true;
					}
					if (!elmanExecutableStatus.empty()) imguiValue(elmanExecutableStatus.c_str());
					if (!m_SpiderTrainingMode)
					{
						const char* counts[] = {"1", "2", "3", "4", "5", "6", "7", "8"};
						int count = m_SpiderTrainingWorldCount - 1;
						if (imguiCombo("Training worlds", &count, counts, 8))
							m_SpiderTrainingWorldCount = count + 1;
					}
					else
					{
						const std::string count = "Training worlds: " + std::to_string(int(m_SpiderTrainingWorldCount));
						imguiValue(count.c_str());
					}
					if (imguiCheck("Training", m_SpiderTrainingMode))
					{
						std::lock_guard<std::mutex> lock(m_ImguiMutex);
						m_PendingSpiderTrainToggle = true;
					}
					if (m_SpiderTrainingMode)
					{
						if (m_SpiderTrainingWorldCount > 1 && imguiButton(m_SpiderTrainingShowAll ? "Show One" : "Show All"))
						{
							std::lock_guard<std::mutex> lock(m_ImguiMutex);
							m_PendingSpiderShowAllToggle = true;
						}
						const char* labels[] = {"Spider 1", "Spider 2", "Spider 3", "Spider 4", "Spider 5", "Spider 6", "Spider 7", "Spider 8"};
						int selected = m_SpiderTrainingSelected;
						if (m_SpiderTrainingWorldCount > 1 && imguiCombo("Observed spider", &selected, labels, m_SpiderTrainingWorldCount))
						{
							std::lock_guard<std::mutex> lock(m_ImguiMutex);
							m_PendingSpiderSelection = selected;
						}
						imguiValue(m_SpiderTraining->status().c_str());
					}
				}
#endif
				const char* modes[] = {"Learning policy", "Sweep controller"};
				int mode = static_cast<int>(robotPanel.WalkMode);
				if (imguiCombo("Walking", &mode, modes, 2))
				{
					robotPanel.WalkMode = static_cast<RobotWalkMode>(mode);
					changed = true;
				}
				if (imguiCheck("Auto walk", robotPanel.AutoWalk))
				{
					robotPanel.AutoWalk = !robotPanel.AutoWalk;
					changed = true;
				}
				imguiValue(robotPanel.AutoWalk ? "W/A/S/D: move camera" : "W/S: forward / back");
				if (!robotPanel.AutoWalk)
					imguiValue(robotPanel.WalkMode == RobotWalkMode::Sweep ? "A/D: turn, Q/E: strafe" : "A/D: steer, Q/E: strafe");
				imguiValue("Mouse: orbit / zoom");
				imguiValue("Mode change restarts scene");
				if (robotPanel.WalkMode == RobotWalkMode::Policy)
					imguiValue("Policy: steer while moving");
				snprintf(text, sizeof(text), "Command X %.2f Z %.2f m/s", robotPanel.Command.x, robotPanel.Command.z);
				imguiValue(text);
				snprintf(text, sizeof(text), "Target heading %.0f deg", robotPanel.Heading * 180.0f / 3.14159265f);
				imguiValue(text);
			}
			if (!robotPanel.Interactive || robotPanel.WalkMode == RobotWalkMode::Policy)
			{
				const char* backends[] = {"Featherstone ABA", "CRBA + Sparse LDLT"};
				int backend = static_cast<int>(robotPanel.Backend);
				if (imguiCombo("Backend", &backend, backends, 2))
				{
					robotPanel.Backend = static_cast<RobotDynamicsBackend>(backend);
					changed = true;
				}
			}
			if (changed)
			{
				std::lock_guard<std::mutex> lock(m_ImguiMutex);
				m_PendingRobotPanel = robotPanel;
			}
			imguiSeparatorLine();
		}
		const char* renderModeLabels[] = { "Solid", "Wireframe", "Solid + Wireframe" };
		imguiLabel("Render Mode");
		for (int modeIndex = 0; modeIndex < 3; ++modeIndex)
		{
			const RenderMode mode = static_cast<RenderMode>(modeIndex);
			if (imguiCheck(renderModeLabels[modeIndex], renderMode == mode, true)
				&& renderMode != mode)
			{
				std::lock_guard<std::mutex> lock(m_ImguiMutex);
				m_ImguiPendingRenderMode = true;
				m_ImguiPendingRenderModeValue = mode;
			}
		}
		if (aerodynamicsPanel.Active || snowPanel.Active || vehiclePanel.Active) imguiSeparatorLine();

		if (aerodynamicsPanel.Active)
		{
			imguiLabel("3D Smoke");
			const char* backends[] = { "CPU", "CUDA", "Compute Shader" };
			imguiLabel("Smoke backend");
			imguiCombo("", &aerodynamicsPanel.Backend, backends, 3, true);
			imguiSlider("Grid scale", &aerodynamicsPanel.GridScale, 0.5f, 1.5f, 0.05f);
			snprintf(text, sizeof(text), "Selected grid: %d x %d x %d",
				(int)std::lround(80.0f * aerodynamicsPanel.GridScale),
				(int)std::lround(128.0f * aerodynamicsPanel.GridScale),
				(int)std::lround(80.0f * aerodynamicsPanel.GridScale));
			imguiValue(text);
			const bool apply = imguiButton("Apply Smoke Settings");
			snprintf(text, sizeof(text), "Running: %s", backends[aerodynamicsPanel.ActualBackend]);
			imguiValue(text);
			snprintf(text, sizeof(text), "Running grid: %d x %d x %d", aerodynamicsPanel.Resolution.x, aerodynamicsPanel.Resolution.y, aerodynamicsPanel.Resolution.z);
			imguiValue(text);
			imguiValue("Drag bodies or stir smoke");
			if (!aerodynamicsPanel.Status.empty()) imguiValue(aerodynamicsPanel.Status.c_str());
			std::lock_guard<std::mutex> lock(m_ImguiMutex);
			if (aerodynamicsPanel.SceneGeneration == m_ImguiAerodynamicsPanel.SceneGeneration)
			{
				m_ImguiAerodynamicsPanel.Backend = aerodynamicsPanel.Backend;
				m_ImguiAerodynamicsPanel.GridScale = aerodynamicsPanel.GridScale;
				if (apply) { m_ImguiPendingAerodynamicsPanel = aerodynamicsPanel; m_ImguiPendingAerodynamicsApply = true; }
			}
		}
		if (snowPanel.Active) DrawSnowPanel(snowPanel);
		if (vehiclePanel.Active)
		{
			imguiLabel(vehiclePanel.Dynamics ? "Vehicle Dynamics" : "Vehicle Crash Test");
			int command = 0;
			if (imguiButton("Focus Vehicle")) command = 2;
			if (imguiCheck("Slow motion (0.25x)", vehiclePanel.SlowMotion)) command = 1;
			bool changed = false;
			if (vehiclePanel.Dynamics)
			{
				changed = imguiSlider("Throttle", &vehiclePanel.Throttle, 0, 1, 0.05f) || changed;
				changed = imguiSlider("Steering", &vehiclePanel.Steering, -1, 1, 0.05f) || changed;
				changed = imguiSlider("Brake", &vehiclePanel.Brake, 0, 1, 0.05f) || changed;
				changed = imguiSlider("Handbrake", &vehiclePanel.Handbrake, 0, 1, 0.05f) || changed;
				changed = imguiSlider("Clutch engagement", &vehiclePanel.Clutch, 0, 1, 0.05f) || changed;
				const char* Gears[] = { "Reverse", "Neutral", "1st", "2nd", "3rd", "4th", "5th", "6th" };
				const char* Differentials[] = { "Open", "Limited slip", "Locked" };
				changed = imguiCombo("Gear", &vehiclePanel.GearIndex, Gears, int(sizeof(Gears) / sizeof(Gears[0]))) || changed;
				changed = imguiCombo("Differential", &vehiclePanel.Differential, Differentials, 3) || changed;
				if (imguiButton("Drive forward")) { vehiclePanel.GearIndex = 2; vehiclePanel.Throttle = 0.5f; vehiclePanel.Brake = vehiclePanel.Handbrake = 0; changed = true; }
				if (imguiButton("Park")) { vehiclePanel.GearIndex = 1; vehiclePanel.Throttle = 0; vehiclePanel.Brake = 1; changed = true; }
				snprintf(text, sizeof(text), "Speed: %.1f m/s, engine: %.0f rpm", vehiclePanel.Speed, vehiclePanel.EngineRPM); imguiValue(text);
				snprintf(text, sizeof(text), "Grounded: %d/4, load: %.0f N", vehiclePanel.GroundedWheels, vehiclePanel.Load); imguiValue(text);
			}
			snprintf(text, sizeof(text), "Maximum dent: %.0f cm", vehiclePanel.Displacement * 100);
			imguiValue(text);
			snprintf(text, sizeof(text), "Plastic strain: %.0f%%", vehiclePanel.PlasticStrain * 100);
			imguiValue(text);
			imguiValue("RMB: orbit around the damage");
			if (command || changed)
			{
				std::lock_guard<std::mutex> lock(m_ImguiMutex);
				if (command) { m_PendingVehicleCommand = command; m_PendingVehicleGeneration = vehiclePanel.SceneGeneration; }
				if (changed) { m_PendingVehiclePanel = vehiclePanel; m_PendingVehicleControls = true; }
			}
		}
		if (aerodynamicsPanel.Active || snowPanel.Active) solverPanel.SolverSwitchable = false;
		if (!meshTool)
		{
			const bool applySolver = DrawSolverPanel(solverPanel);
			std::lock_guard<std::mutex> lock(m_ImguiMutex);
			if (solverPanel.SceneGeneration == m_ImguiSolverPanel.SceneGeneration && !m_ImguiSolverPanel.Pending)
			{
				m_ImguiSolverPanel.ResolutionIndex = solverPanel.ResolutionIndex;
				m_ImguiSolverPanel.BackendIndex = solverPanel.BackendIndex;
				m_ImguiSolverPanel.AllowFallback = solverPanel.AllowFallback;
				m_ImguiSolverPanel.PersistentContacts = solverPanel.PersistentContacts;
				m_ImguiSolverPanel.TriangleContacts = solverPanel.TriangleContacts;
				m_ImguiSolverPanel.CollisionFiltering = solverPanel.CollisionFiltering;
				m_ImguiSolverPanel.ShapeOffsets = solverPanel.ShapeOffsets;
				m_ImguiSolverPanel.SpeculativeContacts = solverPanel.SpeculativeContacts;
				m_ImguiSolverPanel.DistanceFieldDemo = solverPanel.DistanceFieldDemo;
				m_ImguiSolverPanel.StrandDemo=solverPanel.StrandDemo; m_ImguiSolverPanel.StrandRootTwist=solverPanel.StrandRootTwist; m_ImguiSolverPanel.StrandCoupled=solverPanel.StrandCoupled; m_ImguiSolverPanel.StrandLod=solverPanel.StrandLod;
				m_ImguiSolverPanel.StrandContacts=solverPanel.StrandContacts; m_ImguiSolverPanel.StrandSelfContacts=solverPanel.StrandSelfContacts;
				m_ImguiSolverPanel.StrandContinuous=solverPanel.StrandContinuous; m_ImguiSolverPanel.StrandFriction=solverPanel.StrandFriction;
				m_ImguiSolverPanel.PhysicsFieldsDemo = solverPanel.PhysicsFieldsDemo;
				m_ImguiSolverPanel.PhysicsFieldFreeze = solverPanel.PhysicsFieldFreeze;
				m_ImguiSolverPanel.AuthoredDestructionDemo = solverPanel.AuthoredDestructionDemo;
				m_ImguiSolverPanel.DestructionEventsDemo = solverPanel.DestructionEventsDemo;
				m_ImguiSolverPanel.DestructionCacheDemo = solverPanel.DestructionCacheDemo;
				m_ImguiSolverPanel.DestructionSeekFraction = solverPanel.DestructionSeekFraction;
				m_ImguiSolverPanel.CharacterDemo = solverPanel.CharacterDemo;
				m_ImguiSolverPanel.CharacterBox = solverPanel.CharacterBox;
				m_ImguiSolverPanel.VehicleDynamicsDemo = solverPanel.VehicleDynamicsDemo;
				m_ImguiSolverPanel.ReducedRagdollDemo = solverPanel.ReducedRagdollDemo;
				m_ImguiSolverPanel.PhysicalAnimationDemo = solverPanel.PhysicalAnimationDemo;
				m_ImguiSolverPanel.AnimationPassive = solverPanel.AnimationPassive;
				m_ImguiSolverPanel.AnimationArmStrength = solverPanel.AnimationArmStrength;
				m_ImguiSolverPanel.AnimationPhysicsBlend = solverPanel.AnimationPhysicsBlend;
				m_ImguiSolverPanel.ClothModelDemo = solverPanel.ClothModelDemo;
				m_ImguiSolverPanel.ClothAlgorithm = solverPanel.ClothAlgorithm;
				m_ImguiSolverPanel.ClothIterations = solverPanel.ClothIterations;
				m_ImguiSolverPanel.ClothSubsteps = solverPanel.ClothSubsteps;
				m_ImguiSolverPanel.ClothDamping = solverPanel.ClothDamping;
				m_ImguiSolverPanel.ClothStretchUCompliance = solverPanel.ClothStretchUCompliance;
				m_ImguiSolverPanel.ClothStretchVCompliance = solverPanel.ClothStretchVCompliance;
				m_ImguiSolverPanel.ClothShearCompliance = solverPanel.ClothShearCompliance;
				m_ImguiSolverPanel.ClothBendingCompliance = solverPanel.ClothBendingCompliance;
				m_ImguiSolverPanel.ClothWind = solverPanel.ClothWind;
				m_ImguiSolverPanel.ClothAnimation = solverPanel.ClothAnimation;
				m_ImguiSolverPanel.ClothTethers = solverPanel.ClothTethers;
				m_ImguiSolverPanel.ClothSoftMaterial = solverPanel.ClothSoftMaterial;
				m_ImguiSolverPanel.ClothSurfaceContacts = solverPanel.ClothSurfaceContacts;
				m_ImguiSolverPanel.ClothFriction = solverPanel.ClothFriction;
				m_ImguiSolverPanel.ClothSelfCollision = solverPanel.ClothSelfCollision;
				m_ImguiSolverPanel.ClothInterCollision = solverPanel.ClothInterCollision;
				m_ImguiSolverPanel.ClothContinuous = solverPanel.ClothContinuous;
				m_ImguiSolverPanel.ClothRigidCoupling = solverPanel.ClothRigidCoupling;
				m_ImguiSolverPanel.SoftBodyAttachments = solverPanel.SoftBodyAttachments;
				m_ImguiSolverPanel.CompliantAttachments = solverPanel.CompliantAttachments;
				m_ImguiSolverPanel.RigidAttachments = solverPanel.RigidAttachments;
				m_ImguiSolverPanel.SoftBodyContacts = solverPanel.SoftBodyContacts;
				m_ImguiSolverPanel.SoftRigidContacts = solverPanel.SoftRigidContacts;
				m_ImguiSolverPanel.CalibratedSoftMaterial = solverPanel.CalibratedSoftMaterial;
				m_ImguiSolverPanel.StVKSoftMaterial = solverPanel.StVKSoftMaterial;
				m_ImguiSolverPanel.CorotatedSoftMaterial = solverPanel.CorotatedSoftMaterial;
				m_ImguiSolverPanel.ProjectiveSoftMaterial = solverPanel.ProjectiveSoftMaterial;
				m_ImguiSolverPanel.PreparedSoftAsset = solverPanel.PreparedSoftAsset;
				m_ImguiSolverPanel.EmbeddedSoftRender = solverPanel.EmbeddedSoftRender;
				m_ImguiSolverPanel.SequentialWarmStart = solverPanel.SequentialWarmStart;
				m_ImguiSolverPanel.UnifiedConstraints = solverPanel.UnifiedConstraints;
				m_ImguiSolverPanel.FrictionPatches = solverPanel.FrictionPatches;
				m_ImguiSolverPanel.JointDrives = solverPanel.JointDrives;
				m_ImguiSolverPanel.JointBreaking = solverPanel.JointBreaking;
				m_ImguiSolverPanel.CCDContinuation = solverPanel.CCDContinuation;
				m_ImguiSolverPanel.TemporalSubsteps = solverPanel.TemporalSubsteps;
				m_ImguiSolverPanel.SupportStabilization = solverPanel.SupportStabilization;
				m_ImguiSolverPanel.IslandSleeping = solverPanel.IslandSleeping;
				if (applySolver)
				{
					m_ImguiPendingSolverPanel = solverPanel;
					m_ImguiPendingSolverSwitch = true;
					m_ImguiSolverPanel.Pending = true;
					m_ImguiSolverPanel.Status.clear();
				}
			}
		}
		if (!demoNames.empty())
		{
			if (demoIndex < (int)sceneFiles.size())
			{
				if (sceneName == "Ragdoll")
				{
					imguiSeparatorLine();
					imguiLabel("Ragdoll Algorithm");
					const char* algorithms[] = { "Joint Constraints", "Featherstone" };
					int selected = ragdollDemoReduced ? 1 : 0;
					if (imguiCombo("Algorithm", &selected, algorithms, 2))
					{
						std::lock_guard<std::mutex> lock(m_ImguiMutex);
						m_ImguiPendingRagdollAlgorithm = selected;
					}
				}
				if (sceneName.find("Glass Fracture") == 0 && imguiButton("Shoot Glass (I)"))
				{
					std::lock_guard<std::mutex> lock(m_ImguiMutex);
					m_ImguiPendingShootProjectile = true;
				}
				if (sceneName == "Constraints")
				{
					imguiSeparatorLine();
					imguiLabel("Constraint Controls");
					imguiValue("Drag white pins in the scene");
					for (ConstraintControlState state : constraintControls)
					{
						imguiSeparator();
						imguiLabel(state.Name.c_str());
						bool changed = false;
						snprintf(text, sizeof(text), "Force: %.2f N", state.ReactionForce);
						imguiValue(text);
						snprintf(text, sizeof(text), "Torque: %.2f Nm", state.ReactionTorque);
						imguiValue(text);
						if (state.Broken)
						{
							imguiValue("Broken");
							if (imguiButton("Repair joint", true))
							{
								state.RepairRequested = true; state.Enabled = true; changed = true;
							}
						}
						if (imguiCheck("Enabled", state.Enabled, !state.Broken))
						{
							state.Enabled = !state.Enabled;
							changed = true;
						}

						if (state.Type == ConstraintControlType::Distance)
						{
							snprintf(text, sizeof(text), "Current: %.2f", state.CurrentValue);
							imguiValue(text);
							changed = imguiSlider("Min Distance", &state.MinValue, 0.0f, 10.0f, 0.1f) || changed;
							changed = imguiSlider("Max Distance", &state.MaxValue, 0.0f, 10.0f, 0.1f) || changed;
							state.MaxValue = std::max(state.MinValue, state.MaxValue);
						}
						else if (state.Type == ConstraintControlType::Hinge)
						{
							snprintf(text, sizeof(text), "Current: %.1f deg", state.CurrentValue);
							imguiValue(text);
							changed = imguiSlider("Min Angle", &state.MinValue, -180.0f, 180.0f, 1.0f) || changed;
							changed = imguiSlider("Max Angle", &state.MaxValue, -180.0f, 180.0f, 1.0f) || changed;
							state.MaxValue = std::max(state.MinValue, state.MaxValue);
						}
						else if (state.Type == ConstraintControlType::Slider)
						{
							snprintf(text, sizeof(text), "Current: %.2f", state.CurrentValue);
							imguiValue(text);
							changed = imguiSlider("Min Position", &state.MinValue, -5.0f, 5.0f, 0.1f) || changed;
							changed = imguiSlider("Max Position", &state.MaxValue, -5.0f, 5.0f, 0.1f) || changed;
							state.MaxValue = std::max(state.MinValue, state.MaxValue);
						}
						else if (state.Type == ConstraintControlType::Cone)
						{
							snprintf(text, sizeof(text), "Current: %.1f deg", state.CurrentValue);
							imguiValue(text);
							changed = imguiSlider("Max Angle", &state.MaxValue, 0.0f, 180.0f, 1.0f) || changed;
						}
						else if (state.Type == ConstraintControlType::SixDOF)
						{
							const char* axes[] = { "X", "Y", "Z" };
							for (int axis = 0; axis < 3; ++axis)
							{
								char label[32];
								snprintf(label, sizeof(label), "Linear Min %s", axes[axis]);
								changed = imguiSlider(label, &state.LinearMin[axis], -5.0f, 5.0f, 0.1f) || changed;
								snprintf(label, sizeof(label), "Linear Max %s", axes[axis]);
								changed = imguiSlider(label, &state.LinearMax[axis], -5.0f, 5.0f, 0.1f) || changed;
								state.LinearMax[axis] = std::max(state.LinearMin[axis], state.LinearMax[axis]);
							}
							for (int axis = 0; axis < 3; ++axis)
							{
								char label[32];
								snprintf(label, sizeof(label), "Angular Min %s", axes[axis]);
								changed = imguiSlider(label, &state.AngularMinDegrees[axis], -180.0f, 180.0f, 1.0f) || changed;
								snprintf(label, sizeof(label), "Angular Max %s", axes[axis]);
								changed = imguiSlider(label, &state.AngularMaxDegrees[axis], -180.0f, 180.0f, 1.0f) || changed;
								state.AngularMaxDegrees[axis] = std::max(state.AngularMinDegrees[axis], state.AngularMaxDegrees[axis]);
							}
						}
						if (changed)
						{
							std::lock_guard<std::mutex> lock(m_ImguiMutex);
							m_ImguiPendingConstraintControls.push_back(state);
						}
					}
				}
			}
			if (!terrainObjectIds.empty() && !cuttingPanelActive && !meshSimplificationPanelActive)
			{
				imguiSeparatorLine();
				imguiLabel("Terrain Editing");
				std::vector<const char*> terrainLabels;
				int selectedTerrain = 0;
				for (size_t i = 0; i < terrainObjectIds.size(); ++i)
				{
					terrainLabels.push_back(terrainObjectIds[i].c_str());
					if (terrainObjectIds[i] == terrainBrush.ObjectId)
					{
						selectedTerrain = (int)i;
					}
				}
				bool changed = imguiCombo("Terrain", &selectedTerrain, terrainLabels.data(), (int)terrainLabels.size());
				terrainBrush.ObjectId = terrainObjectIds[(size_t)selectedTerrain];
				const char* modes[] = { "Raise", "Lower", "Flatten", "Smooth" };
				changed = imguiCombo("Brush", &terrainBrush.Mode, modes, 4) || changed;
				const float brushRange = terrainBrush.ControlRange;
				changed = imguiSlider("World Radius", &terrainBrush.Radius, 0.1f,
					std::max(brushRange, terrainBrush.Radius), 0.1f) || changed;
				if (terrainBrush.Mode < 2)
				{
					changed = imguiSlider("Height per stroke", &terrainBrush.Value, 0.01f, 1.0f, 0.01f) || changed;
				}
				changed = imguiSlider("Strength", &terrainBrush.Strength, 0.01f, 1.0f, 0.01f) || changed;
				imguiValue("Left-click and drag terrain");
				if (changed)
				{
					std::lock_guard<std::mutex> lock(m_ImguiMutex);
					if (terrainBrush.SceneGeneration == m_ImguiTerrainBrush.SceneGeneration)
					{
						m_ImguiTerrainBrush = terrainBrush;
					}
				}
				if (!terrainStatus.empty())
				{
					imguiValue(terrainStatus.c_str());
				}
			}
			if (cuttingPanelActive)
			{
				imguiSeparatorLine();
				const bool convexDecompositionDemo = IsConvexDecompositionMode(cuttingMode) || demoIndex == (int)sceneFiles.size() + 1;

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
					};
					int selectedAlgorithm = 0;
					if (imguiCombo("Algorithm", &selectedAlgorithm, modes, 1))
					{
						std::lock_guard<std::mutex> lock(m_ImguiMutex);
						m_ImguiPendingCuttingMode = true;
						m_ImguiPendingCuttingModeValue = MeshCuttingMode_VHACD;
					}
					selectedMode = MeshCuttingMode_VHACD;
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

				if (!convexDecompositionDemo && imguiCheck("Cap cut faces", capCutFaces, true))
				{
					std::lock_guard<std::mutex> lock(m_ImguiMutex);
					m_ImguiPendingCapCutFaces = capCutFaces ? 0 : 1;
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

			if (meshSimplificationPanelActive)
			{
				imguiSeparatorLine();

				const std::string meshValue = meshSimplificationPath.empty()
					? std::string("Mesh: bunny.obj")
					: std::string("Mesh: ") + FileNameOf(meshSimplificationPath);
				imguiValue(meshValue.c_str());

				if (imguiButton("Browse Mesh..."))
				{
					std::lock_guard<std::mutex> lock(m_ImguiMutex);
					m_ImguiPendingBrowseMeshSimplificationFile = true;
				}

				const char* algorithmLabels[] = { "Hoppe QEM", "Garland QEM" };
				meshSimplificationAlgorithm = std::max(0, std::min(
					meshSimplificationAlgorithm, MeshSimplificationAlgorithm_Count - 1));
				imguiLabel("Algorithm");
				for (int algorithmIndex = 0;
					algorithmIndex < MeshSimplificationAlgorithm_Count; ++algorithmIndex)
				{
					if (imguiCheck(algorithmLabels[algorithmIndex],
						meshSimplificationAlgorithm == algorithmIndex, true)
						&& meshSimplificationAlgorithm != algorithmIndex)
					{
						std::lock_guard<std::mutex> lock(m_ImguiMutex);
						m_ImguiPendingMeshSimplificationAlgorithm = true;
						m_ImguiPendingMeshSimplificationAlgorithmValue = algorithmIndex;
					}
				}

				meshSimplificationRatioLevel = std::max(1, std::min(meshSimplificationRatioLevel, 20));
				float ratio = static_cast<float>(meshSimplificationRatioLevel) * 0.05f;
				if (imguiSlider("", &ratio, 0.05f, 1.0f, 0.05f))
				{
					const int ratioLevel = std::max(1, std::min(static_cast<int>(ratio * 20.0f + 0.5f), 20));
					std::lock_guard<std::mutex> lock(m_ImguiMutex);
					m_ImguiPendingMeshSimplificationRatioLevel = true;
					m_ImguiPendingMeshSimplificationRatioLevelValue = ratioLevel;
				}

				if (imguiButton("Simplify"))
				{
					std::lock_guard<std::mutex> lock(m_ImguiMutex);
					m_ImguiPendingApplyMeshSimplification = true;
				}

				if (!meshSimplificationStatus.empty())
				{
					imguiValue(meshSimplificationStatus.c_str());
				}
			}
		}
		if (!meshTool)
		{
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
			if (!object.RenderEnabled)
			{
				continue;
			}
			const size_t begin = meshes.size();
			BuildGeometryMeshes(object.GeometryPtr, object.Id, object.Color, object.RenderBounds, &meshes);
			if (object.Vehicle && object.Vehicle->GetSettings().CrushZoneLength > 0)
				BuildVehicleDetails(*object.Vehicle, object.GeometryPtr, object.Id, &meshes);

			RenderBinding binding;
			binding.GeometryPtr = object.GeometryPtr;
			for (size_t i = begin; i < meshes.size(); ++i)
			{
				binding.MeshIds.push_back(meshes[i].Id);
			}
			m_RenderBindings.push_back(binding);
		}
		if (m_FluidDemoActive && m_FluidSimulation != nullptr)
		{
			const SceneFieldSimulationDesc& Field = m_World->GetFieldSimulation();
			const size_t tankMeshBegin = meshes.size();
			BuildAabbWireMesh(Box3(Field.BoundsMin, Field.BoundsMax), "fluid_demo.tank",
				Vector4(0.35f, 0.72f, 0.95f, 1.0f), &meshes);
			for (size_t index = tankMeshBegin; index < meshes.size(); ++index)
			{
				meshes[index].WorldTransform = m_FluidSimulation->GetContainerTransform()
					* meshes[index].WorldTransform;
			}
			RenderMeshDesc waterSurface;
			if (BuildWaterSurfaceMesh(*m_FluidSimulation, "fluid_demo.water", &waterSurface))
			{
				waterSurface.WorldTransform = m_FluidSimulation->GetContainerTransform()
					* waterSurface.WorldTransform;
				meshes.push_back(std::move(waterSurface));
			}
		}
		std::shared_ptr<DensityVolumeDesc> gasVolume;
		if (m_GasDemoActive && m_AerodynamicsSimulation != nullptr)
		{
			std::vector<RenderMeshDesc> gasMeshes;
			GasDensityVisualizationSettings visualization;
			visualization.RelativeThreshold = 0.08f;
			if (m_RenderThread && m_RenderThread->SupportsDensityVolume())
			{
				gasVolume.reset(new DensityVolumeDesc());
				if (!BuildAerodynamicsDensityVolume(*m_AerodynamicsSimulation, gasVolume.get(),
					m_World->GetFieldSimulation().VisualizationThreshold, m_World->GetFieldSimulation().TracerExtinction, m_World->GetFieldSimulation().TracerColor)) gasVolume.reset();
			}
			else if (BuildGasDensityMeshes(*m_AerodynamicsSimulation,
				"gas_demo.density", &gasMeshes, visualization))
			{
				for (RenderMeshDesc& gasMesh : gasMeshes) meshes.push_back(std::move(gasMesh));
			}
		}
		if (m_CuttingPanelActive || m_MeshSimplificationPanelActive)
		{
			for (RenderMeshDesc& mesh : meshes)
			{
				mesh.CastShadow = false;
			}
		}

		if (m_RenderThread)
		{
			CameraDesc camera = GetViewCamera();
			if (m_CuttingPanelActive || m_MeshSimplificationPanelActive)
			{
				SetToolBackgroundColor(&camera);
				ConfigureToolCamera(&camera, GetCameraSceneScale(), m_CamParam.z);
			}
			DirectionalLightDesc light = m_World->GetLight();
			if (m_CuttingPanelActive || m_MeshSimplificationPanelActive)
			{
				ConfigureToolLight(camera, &light);
			}
			const RenderMode renderMode = m_RenderMode;
			std::shared_ptr<std::vector<RenderMeshDesc> > renderMeshes(new std::vector<RenderMeshDesc>(std::move(meshes)));
			m_RenderThread->Submit([renderMeshes, camera, light, renderMode, gasVolume](Renderer& renderer) {
				renderer.Reset();
				renderer.SetRenderMode(renderMode);
				renderer.SetCamera(camera);
				renderer.SetLight(light);
				for (const RenderMeshDesc& mesh : *renderMeshes)
				{
					renderer.AddMesh(mesh);
				}
				if (gasVolume) renderer.UpdateDensityVolume(*gasVolume);
			});
		}

		SubmitGeometryQueryBounds();
		m_SnowMeshReady = false;
		SubmitSnowSurface();
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
		DeactivateGasDemo();
		DeactivateFluidDemo();
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
			++m_SceneGeneration;
			m_World->Reset(Vector3::Zero());
			m_RenderBindings.clear();
			m_HighlightedGeometry = nullptr;
			m_CuttingPieces.clear();
			m_CuttingPanelActive = true;
			m_MeshSimplificationPanelActive = false;
			m_CurrentSceneName = CuttingPanelName(m_CuttingMode);
			m_CuttingStatus = "Missing OBJ: bunny.obj";
			RebuildRenderScene();
			UpdateImguiState();
			return false;
		}

		MeshCuttingSource source;
		if (!LoadMeshCuttingSource(m_CuttingObjPath, &source))
		{
			++m_SceneGeneration;
			m_World->Reset(Vector3::Zero());
			m_RenderBindings.clear();
			m_HighlightedGeometry = nullptr;
			m_CuttingPieces.clear();
			m_CuttingPanelActive = true;
			m_MeshSimplificationPanelActive = false;
			m_CurrentSceneName = CuttingPanelName(m_CuttingMode);
			m_CuttingStatus = source.Status.empty() ? "Failed to read OBJ" : source.Status;
			RebuildRenderScene();
			UpdateImguiState();
			return false;
		}

		m_CuttingMode = std::max(0, std::min(m_CuttingMode, (int)MeshCuttingMode_Count - 1));
		m_CuttingPieceCount = std::max(2, std::min(m_CuttingPieceCount, 64));
		m_CuttingPiecesX = std::max(1, std::min(m_CuttingPiecesX, 64));
		m_CuttingPiecesY = std::max(1, std::min(m_CuttingPiecesY, 64));
		m_CuttingPiecesZ = std::max(1, std::min(m_CuttingPiecesZ, 64));
		m_CuttingSeed = std::max(0, std::min(m_CuttingSeed, 999));
		m_CuttingSeparation = IsConvexDecompositionMode(m_CuttingMode) ? 1.0f : 0.0f;

		++m_SceneGeneration;
		m_World->Reset(Vector3::Zero());
		m_RenderBindings.clear();
		m_HighlightedGeometry = nullptr;
		m_CuttingPieces.clear();
		m_CuttingPanelActive = true;
		m_MeshSimplificationPanelActive = false;
		m_CurrentSceneName = CuttingPanelName(m_CuttingMode);

		Geometry* geometry = m_World->AddTriangleMeshObject("mesh_source", source.Mesh, Transform(Vector3::Zero()), RigidType::Static, Vector4(0.72f, 0.76f, 0.82f, 1.0f), false);
		if (geometry == nullptr)
		{
			m_CuttingStatus = "No renderable mesh";
			RebuildRenderScene();
			UpdateImguiState();
			return false;
		}

		m_CamParam = Vector3(1.05f, 0.45f, 1.0f);
		FrameCuttingBounds();
		m_CuttingStatus = source.Status + (IsConvexDecompositionMode(m_CuttingMode) ? ". Click Apply" : ". Click Apply Cut");

		RebuildRenderScene();

		UpdateImguiState();
		return true;
	}

	bool WorldViewer::ApplyCuttingPanel()
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
		params.CapCutFaces = m_CapCutFaces;
		const CameraDesc view = GetViewCamera();
		const Vector3 forward = SafeUnitVector(view.At - view.Eye, Vector3::UnitZ());
		params.ViewRight = SafeUnitVector(view.Up.Cross(forward), Vector3::UnitX());
		params.ViewUp = forward.Cross(params.ViewRight);
		params.ViewAspect = (float)(m_ViewportWidth - CuttingPanelRight(m_ViewportWidth)) / m_ViewportHeight;

		MeshCuttingResult cuttingResult;
		if (!BuildMeshCuttingPanel(params, &cuttingResult))
		{
			m_CuttingPanelActive = true;
			m_MeshSimplificationPanelActive = false;
			m_CuttingStatus = cuttingResult.Status.empty() ? "Cut failed" : cuttingResult.Status;
			UpdateImguiState();
			return false;
		}

		if (!IsConvexDecompositionMode(params.Mode)) m_CuttingSeparation = 0.0f;
		m_CuttingMode = std::max(0, std::min(m_CuttingMode, (int)MeshCuttingMode_Count - 1));
		m_CuttingPieceCount = std::max(2, std::min(m_CuttingPieceCount, 64));
		m_CuttingPiecesX = std::max(1, std::min(m_CuttingPiecesX, 64));
		m_CuttingPiecesY = std::max(1, std::min(m_CuttingPiecesY, 64));
		m_CuttingPiecesZ = std::max(1, std::min(m_CuttingPiecesZ, 64));
		m_CuttingSeed = std::max(0, std::min(m_CuttingSeed, 999));

		++m_SceneGeneration;
		m_World->Reset(Vector3::Zero());
		m_RenderBindings.clear();
		m_HighlightedGeometry = nullptr;
		m_CuttingPieces.clear();
		m_CuttingPanelActive = true;
		m_MeshSimplificationPanelActive = false;
		m_CurrentSceneName = CuttingPanelName(m_CuttingMode);

		for (size_t pieceIndex = 0; pieceIndex < cuttingResult.Pieces.size(); ++pieceIndex)
		{
			const MeshCuttingPiece& piece = cuttingResult.Pieces[pieceIndex];

			std::ostringstream id;
			id << "mesh_cut_" << pieceIndex;
			const Transform transform(piece.Center + piece.SeparationOffset * m_CuttingSeparation);
			Geometry* geometry = m_World->AddTriangleMeshObject(id.str(), piece.Mesh, transform, RigidType::Static, PieceColor(pieceIndex), false);
			if (geometry == nullptr)
			{
				continue;
			}

			CuttingRenderPiece renderPiece;
			renderPiece.GeometryPtr = geometry;
			renderPiece.Center = piece.Center;
			renderPiece.SeparationOffset = piece.SeparationOffset;
			m_CuttingPieces.push_back(renderPiece);
		}

		if (m_CuttingPieces.empty())
		{
			m_CuttingPanelActive = true;
			m_MeshSimplificationPanelActive = false;
			m_CuttingStatus = "No renderable pieces";
			RebuildRenderScene();
			UpdateImguiState();
			return false;
		}

		m_CuttingStatus = cuttingResult.Status;

		SubmitCuttingTransforms();
		RebuildRenderScene();

		UpdateImguiState();
		return true;
	}

	bool WorldViewer::LoadMeshSimplificationPanel()
	{
		DeactivateGasDemo();
		DeactivateFluidDemo();
		RefreshMeshFileList();
		if (m_MeshSimplificationPath.empty())
		{
			m_MeshSimplificationPath = ResolveTestDataFile("bunny.obj");
		}
		if (m_MeshSimplificationPath.empty() && !m_MeshSimplificationFiles.empty())
		{
			m_MeshSimplificationPath = m_MeshSimplificationFiles.front();
		}
		if (m_MeshSimplificationPath.empty())
		{
			++m_SceneGeneration;
			m_World->Reset(Vector3::Zero());
			m_RenderBindings.clear();
			m_HighlightedGeometry = nullptr;
			m_CuttingPieces.clear();
			m_CuttingPanelActive = false;
			m_MeshSimplificationPanelActive = true;
			m_CurrentSceneName = "Mesh Simplification";
			m_MeshSimplificationStatus = "Missing mesh: bunny.obj";
			RebuildRenderScene();
			UpdateImguiState();
			return false;
		}

		MeshSimplificationSource source;
		if (!LoadMeshSimplificationSource(m_MeshSimplificationPath, &source))
		{
			++m_SceneGeneration;
			m_World->Reset(Vector3::Zero());
			m_RenderBindings.clear();
			m_HighlightedGeometry = nullptr;
			m_CuttingPieces.clear();
			m_CuttingPanelActive = false;
			m_MeshSimplificationPanelActive = true;
			m_CurrentSceneName = "Mesh Simplification";
			m_MeshSimplificationStatus = source.Status.empty() ? "Failed to read mesh" : source.Status;
			RebuildRenderScene();
			UpdateImguiState();
			return false;
		}

		const Box3 bounds = source.Bounds;
		const Vector3 center = bounds.GetCenter();
		m_MeshSimplificationRatioLevel = std::max(1, std::min(m_MeshSimplificationRatioLevel, 20));
		m_MeshSimplificationMaxSeparation = source.MaxSeparation;

		++m_SceneGeneration;
		m_World->Reset(Vector3::Zero());
		m_RenderBindings.clear();
		m_HighlightedGeometry = nullptr;
		m_CuttingPieces.clear();
		m_CuttingPanelActive = false;
		m_MeshSimplificationPanelActive = true;
		m_CurrentSceneName = "Mesh Simplification";

		Geometry* geometry = m_World->AddTriangleMeshObject("mesh_simplification_source", source.Mesh, Transform(Vector3::Zero()), RigidType::Static, Vector4(0.72f, 0.76f, 0.82f, 1.0f), false);
		if (geometry == nullptr)
		{
			m_MeshSimplificationStatus = "No renderable mesh";
			RebuildRenderScene();
			UpdateImguiState();
			return false;
		}

		m_CamCenter = center;
		m_CamParam = Vector3(1.05f, 0.45f, m_MeshSimplificationMaxSeparation * 3.5f);
		m_MeshSimplificationStatus = source.Status + ". Click Simplify";

		RebuildRenderScene();

		if (m_RenderThread)
		{
			CameraDesc camera = m_World->GetCamera();
			camera.At = m_CamCenter;
			camera.Eye = GetCameraPosition();
			SetToolBackgroundColor(&camera);
			ConfigureToolCamera(&camera, m_MeshSimplificationMaxSeparation, m_CamParam.z);

			DirectionalLightDesc light = m_World->GetLight();
			ConfigureToolLight(camera, &light);

			m_RenderThread->Submit([camera, light](Renderer& renderer) {
				renderer.SetCamera(camera);
				renderer.SetLight(light);
			});
		}

		UpdateImguiState();
		return true;
	}

	bool WorldViewer::ApplyMeshSimplificationPanel()
	{
		const Vector3 cameraCenter = m_CamCenter;
		Vector3 cameraParam = m_CamParam;

		RefreshMeshFileList();
		if (m_MeshSimplificationPath.empty())
		{
			m_MeshSimplificationPath = ResolveTestDataFile("bunny.obj");
		}
		if (m_MeshSimplificationPath.empty() && !m_MeshSimplificationFiles.empty())
		{
			m_MeshSimplificationPath = m_MeshSimplificationFiles.front();
		}
		if (m_MeshSimplificationPath.empty())
		{
			m_MeshSimplificationStatus = "Missing mesh: bunny.obj";
			UpdateImguiState();
			return false;
		}

		m_MeshSimplificationRatioLevel = std::max(1, std::min(m_MeshSimplificationRatioLevel, 20));

		MeshSimplificationParams params;
		params.MeshPath = m_MeshSimplificationPath;
		params.Ratio = static_cast<float>(m_MeshSimplificationRatioLevel) * 0.05f;
		params.Algorithm = m_MeshSimplificationAlgorithm;

		MeshSimplificationResult result;
		if (!BuildMeshSimplificationPanel(params, &result))
		{
			m_CuttingPanelActive = false;
			m_MeshSimplificationPanelActive = true;
			m_MeshSimplificationStatus = result.Status.empty() ? "Simplification failed" : result.Status;
			UpdateImguiState();
			return false;
		}

		const Box3 bounds = result.SourceBounds;
		const Box3 simplifiedBounds = result.SimplifiedMesh.BoundingVolume;
		m_MeshSimplificationMaxSeparation = result.MaxSeparation;

		++m_SceneGeneration;
		m_World->Reset(Vector3::Zero());
		m_RenderBindings.clear();
		m_HighlightedGeometry = nullptr;
		m_CuttingPieces.clear();
		m_CuttingPanelActive = false;
		m_MeshSimplificationPanelActive = true;
		m_CurrentSceneName = "Mesh Simplification";

		const float maxDim = std::max(std::max(bounds.MaxDim(), simplifiedBounds.MaxDim()), 1e-3f);
		const float gap = std::max(maxDim * 0.08f, 1e-3f);
		const float requiredDistanceX = std::max(0.0f, bounds.Max.x - simplifiedBounds.Min.x) + gap;
		const float offset = std::max(requiredDistanceX * 0.5f, gap);
		Geometry* sourceGeometry = m_World->AddTriangleMeshObject(
			"mesh_simplification_original",
			result.SourceMesh,
			Transform(Vector3(-offset, 0.0f, 0.0f)),
			RigidType::Static,
			Vector4(0.72f, 0.76f, 0.82f, 1.0f),
			false);
		Geometry* simplifiedGeometry = m_World->AddTriangleMeshObject(
			"mesh_simplification_result",
			result.SimplifiedMesh,
			Transform(Vector3(offset, 0.0f, 0.0f)),
			RigidType::Static,
			Vector4(0.42f, 0.78f, 0.58f, 1.0f),
			false);

		if (sourceGeometry == nullptr || simplifiedGeometry == nullptr)
		{
			m_MeshSimplificationStatus = "No renderable simplification result";
			RebuildRenderScene();
			UpdateImguiState();
			return false;
		}

		m_CamCenter = cameraCenter;
		m_CamParam = cameraParam;
		m_MeshSimplificationStatus = result.Status;

		RebuildRenderScene();

		if (m_RenderThread)
		{
			CameraDesc camera = m_World->GetCamera();
			camera.At = m_CamCenter;
			camera.Eye = GetCameraPosition();
			SetToolBackgroundColor(&camera);
			ConfigureToolCamera(&camera, m_MeshSimplificationMaxSeparation, m_CamParam.z);

			DirectionalLightDesc light = m_World->GetLight();
			ConfigureToolLight(camera, &light);

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
			std::vector<std::string> files = ListMeshFiles(directory);
			for (const std::string& file : files)
			{
				AddUniqueFile(&objFiles, file);
			}
		}

		if (!m_SceneDirectory.empty() && DirectoryExists(m_SceneDirectory))
		{
			std::vector<std::string> files = ListMeshFiles(m_SceneDirectory);
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

	void WorldViewer::RefreshMeshFileList()
	{
		std::vector<std::string> meshFiles;
		const std::string defaultBunny = ResolveTestDataFile("bunny.obj");
		AddUniqueFile(&meshFiles, defaultBunny);

		const std::vector<std::string> dataDirectories = TestDataDirectoryCandidates(m_SceneDirectory);
		for (const std::string& directory : dataDirectories)
		{
			if (!DirectoryExists(directory))
			{
				continue;
			}
			std::vector<std::string> files = ListMeshFiles(directory);
			for (const std::string& file : files)
			{
				AddUniqueFile(&meshFiles, file);
			}
		}

		if (!m_SceneDirectory.empty() && DirectoryExists(m_SceneDirectory))
		{
			std::vector<std::string> files = ListMeshFiles(m_SceneDirectory);
			for (const std::string& file : files)
			{
				AddUniqueFile(&meshFiles, file);
			}
		}

		AddUniqueFile(&meshFiles, m_MeshSimplificationPath);
		m_MeshSimplificationFiles.swap(meshFiles);

		m_MeshSimplificationFileIndex = 0;
		for (size_t i = 0; i < m_MeshSimplificationFiles.size(); ++i)
		{
			if (m_MeshSimplificationFiles[i] == m_MeshSimplificationPath)
			{
				m_MeshSimplificationFileIndex = (int)i;
				break;
			}
		}
		if (m_MeshSimplificationPath.empty() && !m_MeshSimplificationFiles.empty())
		{
			m_MeshSimplificationPath = m_MeshSimplificationFiles.front();
			m_MeshSimplificationFileIndex = 0;
		}
	}

	void WorldViewer::SubmitCuttingTransforms()
	{
		if (!m_CuttingPanelActive || m_CuttingPieces.empty()) return;
		for (const CuttingRenderPiece& piece : m_CuttingPieces)
		{
			RigidBody* body = piece.GeometryPtr->GetParent<RigidBody>();
			body->X = piece.Center + piece.SeparationOffset * m_CuttingSeparation;
		}
		m_World->GetSimulation()->SynchronizeSceneQueries();
		FrameCuttingBounds();
		UpdateCamera();
		SubmitTransforms();
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

	void WorldViewer::SubmitDeformableMeshes()
	{
		if (!m_World || !m_RenderThread) return;
		auto* World = m_World->GetSimulation(); if (!World) return;
		std::vector<Geometry*> Geometries;
		for (size_t I = 0; I < World->GetClothCount(); ++I) Geometries.push_back(World->GetCloth(I)->GetRenderGeometry());
		for (auto* Body : World->GetDeformableBodies()) Geometries.push_back(Body->GetRenderGeometry());
		if (Geometries.empty()) return;
		auto Meshes = std::make_shared<std::vector<RenderMeshDesc>>();
		for (const auto& Object : m_World->GetObjects())
			if (Object.RenderEnabled && std::find(Geometries.begin(), Geometries.end(), Object.GeometryPtr) != Geometries.end())
				BuildGeometryMeshes(Object.GeometryPtr, Object.Id, Object.Color, Object.RenderBounds, Meshes.get());
		m_RenderThread->Submit([Meshes](Renderer& Renderer) {
			for (const auto& Mesh : *Meshes)
			{
				RenderTransformUpdate Transform; Transform.Id = Mesh.Id; Transform.WorldTransform = Mesh.WorldTransform; Renderer.UpdateTransform(Transform);
				Renderer.UpdateMeshGeometry(Mesh.Id.c_str(), Mesh.Vertices.data(), (int)Mesh.Vertices.size(), Mesh.Indices.data(), (int)Mesh.Indices.size());
			}
		});
	}

	void WorldViewer::SubmitGeometryQueryBounds()
	{
		if (!m_ShowGeometryQueryBounds || m_CuttingPanelActive || m_MeshSimplificationPanelActive)
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

	void WorldViewer::SetRenderMode(RenderMode mode)
	{
		if (m_RenderMode == mode)
		{
			return;
		}

		m_RenderMode = mode;
		if (m_RenderThread)
		{
			m_RenderThread->Submit([mode](Renderer& renderer) {
				renderer.SetRenderMode(mode);
			});
		}
		UpdateImguiState();
	}

	void WorldViewer::ApplyImguiCommands()
	{
		AerodynamicsPanelState pendingAerodynamics;
		bool pendingAerodynamicsApply = false;
		bool pendingRestartScene = false;
		unsigned int pendingRestartGeneration = 0;
		bool pendingPauseToggle = false;
		unsigned int pendingPauseGeneration = 0;
		bool pendingSolverSwitch = false;
		SolverPanelState pendingSolverPanel;
		std::string pendingSceneFile;
		int pendingDemoIndex = -1;
		int pendingRagdollAlgorithm = -1;
		RobotPanelState pendingRobot;
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
		int pendingCapCutFaces = -1;
		bool pendingCuttingSeed = false;
		int pendingCuttingSeedValue = 0;
		int pendingMeshSimplificationFileIndex = -1;
		bool pendingBrowseMeshSimplificationFile = false;
		bool pendingApplyMeshSimplification = false;
		bool pendingMeshSimplificationAlgorithm = false;
		int pendingMeshSimplificationAlgorithmValue = MeshSimplificationAlgorithm_HoppeQEM;
		bool pendingMeshSimplificationRatioLevel = false;
		int pendingMeshSimplificationRatioLevelValue = 1;
		bool pendingGeometryQueryBounds = false;
		bool pendingGeometryQueryBoundsValue = false;
		bool pendingRenderMode = false;
		RenderMode pendingRenderModeValue = RenderMode::Solid;
		bool pendingShootProjectile = false;
#ifdef RIEMANN_SPIDER_TRAINING
		bool trainingToggle = false, showAllToggle = false;
		bool browseElmanExecutable = false;
		int selectedSpider = -1;
#endif
		std::vector<ConstraintControlState> pendingConstraintControls;
		{
			std::lock_guard<std::mutex> lock(m_ImguiMutex);
			pendingSolverSwitch = m_ImguiPendingSolverSwitch;
			pendingAerodynamics = m_ImguiPendingAerodynamicsPanel;
			pendingAerodynamicsApply = m_ImguiPendingAerodynamicsApply;
			m_ImguiPendingAerodynamicsApply = false;
			pendingRestartScene = m_ImguiPendingRestartScene;
			pendingRestartGeneration = m_ImguiPendingRestartGeneration;
			m_ImguiPendingRestartScene = false;
			pendingPauseToggle = m_ImguiPendingPauseToggle;
			pendingPauseGeneration = m_ImguiPendingPauseGeneration;
			m_ImguiPendingPauseToggle = false;
			pendingSolverPanel = m_ImguiPendingSolverPanel;
			m_ImguiPendingSolverSwitch = false;
			pendingDemoIndex = m_ImguiPendingDemoIndex;
			m_ImguiPendingDemoIndex = -1;
			pendingRobot = m_PendingRobotPanel;
			m_PendingRobotPanel.Active = false;
#ifdef RIEMANN_SPIDER_TRAINING
			trainingToggle = m_PendingSpiderTrainToggle;
			showAllToggle = m_PendingSpiderShowAllToggle;
			selectedSpider = m_PendingSpiderSelection;
			browseElmanExecutable = m_PendingBrowseElmanExecutable;
			m_PendingSpiderTrainToggle = m_PendingSpiderShowAllToggle = false;
			m_PendingBrowseElmanExecutable = false;
			m_PendingSpiderSelection = -1;
#endif
			pendingRagdollAlgorithm = m_ImguiPendingRagdollAlgorithm;
			m_ImguiPendingRagdollAlgorithm = -1;
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
			pendingCapCutFaces = m_ImguiPendingCapCutFaces;
			m_ImguiPendingCapCutFaces = -1;
			pendingCuttingSeed = m_ImguiPendingCuttingSeed;
			pendingCuttingSeedValue = m_ImguiPendingCuttingSeedValue;
			m_ImguiPendingCuttingSeed = false;
			pendingMeshSimplificationFileIndex = m_ImguiPendingMeshSimplificationFileIndex;
			m_ImguiPendingMeshSimplificationFileIndex = -1;
			pendingBrowseMeshSimplificationFile = m_ImguiPendingBrowseMeshSimplificationFile;
			m_ImguiPendingBrowseMeshSimplificationFile = false;
			pendingApplyMeshSimplification = m_ImguiPendingApplyMeshSimplification;
			m_ImguiPendingApplyMeshSimplification = false;
			pendingMeshSimplificationAlgorithm = m_ImguiPendingMeshSimplificationAlgorithm;
			pendingMeshSimplificationAlgorithmValue = m_ImguiPendingMeshSimplificationAlgorithmValue;
			m_ImguiPendingMeshSimplificationAlgorithm = false;
			pendingMeshSimplificationRatioLevel = m_ImguiPendingMeshSimplificationRatioLevel;
			pendingMeshSimplificationRatioLevelValue = m_ImguiPendingMeshSimplificationRatioLevelValue;
			m_ImguiPendingMeshSimplificationRatioLevel = false;
			pendingGeometryQueryBounds = m_ImguiPendingShowGeometryQueryBounds;
			pendingGeometryQueryBoundsValue = m_ImguiPendingShowGeometryQueryBoundsValue;
			m_ImguiPendingShowGeometryQueryBounds = false;
			pendingRenderMode = m_ImguiPendingRenderMode;
			pendingRenderModeValue = m_ImguiPendingRenderModeValue;
			m_ImguiPendingRenderMode = false;
			pendingShootProjectile = m_ImguiPendingShootProjectile;
			m_ImguiPendingShootProjectile = false;
			pendingConstraintControls.swap(m_ImguiPendingConstraintControls);
		}
		if (pendingPauseToggle && pendingPauseGeneration == m_SceneGeneration)
			m_ViewerPaused = !m_ViewerPaused;
		if (pendingRestartScene && pendingRestartGeneration == m_SceneGeneration)
		{
			m_ViewerPaused = false;
			if (m_CuttingPanelActive) LoadCuttingPanel();
			else if (m_MeshSimplificationPanelActive) LoadMeshSimplificationPanel();
			else LoadScene(m_SceneDirectory + "/" + m_CurrentSceneName);
			return;
		}

		if (pendingAerodynamicsApply && m_AerodynamicsSimulation && pendingAerodynamics.SceneGeneration == m_SceneGeneration)
		{
			AerodynamicsSimulationSettings settings = m_World->GetFieldSimulation().Aerodynamics;
			const FluidBackend Backends[] = { FluidBackend::CPU, FluidBackend::CUDA, FluidBackend::ComputeShader };
			settings.Backend = Backends[std::max(0, std::min(pendingAerodynamics.Backend, 2))];
			settings.AllowBackendFallback = m_World->GetFieldSimulation().AllowBackendFallback;
			const float scale = std::max(0.5f, std::min(pendingAerodynamics.GridScale, 1.5f));
			settings.Resolution = Vector3i((int)std::lround(80.0f * scale),
				(int)std::lround(128.0f * scale), (int)std::lround(80.0f * scale));
			const auto Coupling = m_AerodynamicsCoupling ? m_AerodynamicsCoupling->GetSettings() : AerodynamicsRigidCouplingSettings();
			auto* Next = CreateSmokeWithFallback(m_World->GetSimulation(), settings, Coupling,
				m_World->GetFieldSimulation().RigidCoupling);
			const bool ready = Next != nullptr;
			if (ready)
			{
				m_AerodynamicsSimulation = Next; m_AerodynamicsCoupling = m_World->GetSimulation()->GetAerodynamicsCoupling();
				m_AerodynamicsPaused = false; SubmitGasDensity();
			}
			std::lock_guard<std::mutex> lock(m_ImguiMutex);
			m_ImguiAerodynamicsPanel.Status = ready
				? (Next->GetBackend() == settings.Backend ? "Smoke settings applied" : "Smoke settings applied on another backend")
				: "Backend unavailable";
		}

		if (pendingDemoIndex >= 0)
		{
			const int meshCuttingDemoIndex = (int)m_SceneFiles.size();
			const int convexDecompositionDemoIndex = meshCuttingDemoIndex + 1;
			const int meshSimplificationDemoIndex = meshCuttingDemoIndex + 2;
			if (pendingDemoIndex >= meshCuttingDemoIndex) m_ViewerPaused = false;
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
			else if (pendingDemoIndex == meshSimplificationDemoIndex)
			{
				LoadMeshSimplificationPanel();
			}
		}

		if (!pendingSceneFile.empty())
		{
			LoadScene(pendingSceneFile);
		}
		if (pendingRobot.Active && pendingRobot.SceneGeneration == m_SceneGeneration && m_World->GetRobotDemoController())
		{
			const RobotDynamicsBackend previous = m_World->GetWalkingRobotBackend();
			const RobotWalkMode previousMode = m_World->GetRobotWalkMode();
			m_RobotAutoWalk = pendingRobot.AutoWalk;
			m_World->SetWalkingRobotBackend(pendingRobot.Backend);
			m_World->SetRobotWalkMode(pendingRobot.WalkMode);
			if ((previous != pendingRobot.Backend || previousMode != pendingRobot.WalkMode)
				&& !LoadScene(m_CurrentSceneName))
			{
				m_World->SetWalkingRobotBackend(previous);
				m_World->SetRobotWalkMode(previousMode);
				LoadScene(m_CurrentSceneName);
			}
		}
#ifdef RIEMANN_SPIDER_TRAINING
		if (browseElmanExecutable)
		{
			const std::string selectedExecutable = OpenElmanExecutableDialog();
			if (!selectedExecutable.empty()) SaveElmanExecutablePath(selectedExecutable);
		}
		if (trainingToggle && m_World->IsSpiderWalkingDemo())
		{
			if (m_SpiderTrainingMode)
			{
				m_SpiderTraining->stop();
				m_SpiderTrainingMode = false;
				LoadScene(m_CurrentSceneName);
			}
			else
			{
				const int worldCount = m_SpiderTrainingWorldCount;
				m_World->BeginSpiderTrainingDisplay(size_t(worldCount));
				m_SpiderTrainingMode = true;
				m_SpiderTrainingShowAll = false;
				m_SpiderTrainingSelected = 0;
				m_RobotAutoWalk = true;
				SpiderTrainingOptions options;
				options.policy_path = m_SceneDirectory + "../Robots/Spider/actor.bin";
				options.environment_count = worldCount;
				m_SpiderTraining->start(std::move(options));
			}
		}
		if (showAllToggle)
		{
			m_SpiderTrainingShowAll = !m_SpiderTrainingShowAll;
			m_CamCenter = m_SpiderTrainingShowAll ? Vector3(6.75f, 0.45f, 1.5f) : Vector3(0, 0.45f, 0);
			m_CamParam.z = m_SpiderTrainingShowAll ? 24.0f : 8.5f;
			UpdateCamera();
		}
		if (selectedSpider >= 0 && selectedSpider < m_SpiderTrainingWorldCount) m_SpiderTrainingSelected = selectedSpider;
#endif
		if (pendingRagdollAlgorithm >= 0 && m_CurrentSceneName == "09_ragdoll_stairs.json")
		{
			const bool previous = m_RagdollDemoReduced;
			m_RagdollDemoReduced = pendingRagdollAlgorithm == 1;
			if (!LoadScene(m_CurrentSceneName))
			{
				m_RagdollDemoReduced = previous;
				LoadScene(m_CurrentSceneName);
			}
		}
		if (pendingSolverSwitch && pendingSolverPanel.SceneGeneration == m_SceneGeneration)
		{
			if (pendingSolverPanel.SolverChanged)
			{
				PhysicsWorld* simulation = m_World ? m_World->GetSimulation() : nullptr;
				const bool allowed = simulation && !m_CuttingPanelActive && !m_MeshSimplificationPanelActive
					&& !m_FluidDemoActive && !m_GasDemoActive && !m_AerodynamicsSimulation && !m_SnowSimulation && !m_SandSimulation
					&& simulation->GetReducedRagdollCount() == 0;
				std::string error;
				bool applied = allowed && pendingSolverPanel.ResolutionIndex >= 0 && pendingSolverPanel.ResolutionIndex < 4
					&& pendingSolverPanel.BackendIndex >= 0 && pendingSolverPanel.BackendIndex < 4
					&& simulation->SetSolverConfiguration(SolverPanelAlgorithms[pendingSolverPanel.ResolutionIndex],
						SolverPanelBackends[pendingSolverPanel.BackendIndex], pendingSolverPanel.AllowFallback, &error);
				if (!allowed) error = "Solver switching is unavailable in this demo.";
				{
					std::lock_guard<std::mutex> lock(m_ImguiMutex);
					m_ImguiSolverPanel.Pending = false;
					m_ImguiSolverPanel.Status = applied ? std::string() : (error.empty() ? "Solver change is unavailable." : error);
					if (simulation)
					{
						m_ImguiSolverPanel.ResolutionIndex = GetSolverPanelIndex(simulation->GetResolutionSolver());
						m_ImguiSolverPanel.BackendIndex = GetBackendPanelIndex(simulation->GetSolverBackend());
					}
				}
				UpdateImguiState();
			}
			else if (pendingSolverPanel.HasClothDemo)
			{
				const bool applied = m_World && m_World->ApplyClothParameters(
					pendingSolverPanel.ClothAlgorithm == 0 ? ClothModel::DistanceLinks : ClothModel::TriangleStrain,
					(int)pendingSolverPanel.ClothIterations, (int)pendingSolverPanel.ClothSubsteps, pendingSolverPanel.ClothDamping,
					pendingSolverPanel.ClothStretchUCompliance, pendingSolverPanel.ClothStretchVCompliance,
					pendingSolverPanel.ClothShearCompliance, pendingSolverPanel.ClothBendingCompliance)
					&& m_World->SetClothRigidInteraction(pendingSolverPanel.ClothRigidCoupling)
					&& m_World->SetClothSelfCollision(pendingSolverPanel.ClothSelfCollision);
				{
					std::lock_guard<std::mutex> lock(m_ImguiMutex);
					m_ImguiSolverPanel.Pending = false;
					m_ImguiSolverPanel.Status = applied ? std::string() : "Cloth settings are unavailable in this demo.";
				}
				UpdateImguiState();
			}
			else
			{
			std::string error;
			bool reloaded = true;
			if (m_World && !m_CuttingPanelActive && !m_MeshSimplificationPanelActive
				&& (pendingSolverPanel.DistanceFieldDemo != m_World->GetDistanceFieldDemo()
					|| pendingSolverPanel.StrandDemo != m_World->GetStrandDemo()
					|| pendingSolverPanel.CharacterDemo != m_World->GetCharacterDemo() || pendingSolverPanel.CharacterBox != m_World->GetCharacterBox()
					|| pendingSolverPanel.VehicleDynamicsDemo != m_World->GetVehicleDynamicsDemo()
					|| pendingSolverPanel.ReducedRagdollDemo != m_World->GetReducedRagdollDemo()
					|| pendingSolverPanel.PhysicalAnimationDemo != m_World->GetPhysicalAnimationDemo()
					|| pendingSolverPanel.ClothModelDemo != m_World->GetClothModelDemo() || pendingSolverPanel.AuthoredDestructionDemo != m_World->GetAuthoredDestructionDemo()
					|| pendingSolverPanel.DestructionEventsDemo != m_World->GetDestructionEventsDemo()
					|| pendingSolverPanel.DestructionCacheDemo != m_World->GetDestructionCacheDemo()))
			{
				const bool Previous = m_World->GetDistanceFieldDemo();
				const bool PreviousStrands=m_World->GetStrandDemo();
				const bool PreviousCharacter = m_World->GetCharacterDemo(), PreviousBox = m_World->GetCharacterBox();
				const bool PreviousVehicle = m_World->GetVehicleDynamicsDemo();
				const bool PreviousReducedRagdoll = m_World->GetReducedRagdollDemo();
				const bool PreviousAnimation = m_World->GetPhysicalAnimationDemo();
				const bool PreviousCloth = m_World->GetClothModelDemo();
				const bool PreviousDestruction = m_World->GetAuthoredDestructionDemo();
				const bool PreviousEvents = m_World->GetDestructionEventsDemo();
				const bool PreviousCache = m_World->GetDestructionCacheDemo();
				const std::string SceneFile = m_SceneDirectory + "/" + m_CurrentSceneName;
				m_World->SetDistanceFieldDemo(pendingSolverPanel.DistanceFieldDemo);
				m_World->SetStrandDemo(pendingSolverPanel.StrandDemo);
				m_World->SetCharacterDemo(pendingSolverPanel.CharacterDemo, pendingSolverPanel.CharacterBox);
				m_World->SetVehicleDynamicsDemo(pendingSolverPanel.VehicleDynamicsDemo);
				m_World->SetReducedRagdollDemo(pendingSolverPanel.ReducedRagdollDemo);
				m_World->SetPhysicalAnimationDemo(pendingSolverPanel.PhysicalAnimationDemo);
				m_World->SetClothModelDemo(pendingSolverPanel.ClothModelDemo);
				m_World->SetAuthoredDestructionDemo(pendingSolverPanel.AuthoredDestructionDemo);
				m_World->SetDestructionEventsDemo(pendingSolverPanel.DestructionEventsDemo);
				m_World->SetDestructionCacheDemo(pendingSolverPanel.DestructionCacheDemo);
				reloaded = LoadScene(SceneFile);
				if (!reloaded)
				{
					m_World->SetDistanceFieldDemo(Previous);
					m_World->SetStrandDemo(PreviousStrands);
					m_World->SetCharacterDemo(PreviousCharacter, PreviousBox);
					m_World->SetVehicleDynamicsDemo(PreviousVehicle);
					m_World->SetReducedRagdollDemo(PreviousReducedRagdoll);
					m_World->SetPhysicalAnimationDemo(PreviousAnimation);
					m_World->SetClothModelDemo(PreviousCloth);
					m_World->SetAuthoredDestructionDemo(PreviousDestruction);
					m_World->SetDestructionEventsDemo(PreviousEvents);
					m_World->SetDestructionCacheDemo(PreviousCache);
					const bool Restored = LoadScene(SceneFile);
					error = Restored ? "Demo scene reload failed; restored the previous mode."
						: "Demo scene reload failed; restoring the previous scene also failed.";
				}
			}
			PhysicsWorld* simulation = m_World ? m_World->GetSimulation() : nullptr;
			const bool valid = reloaded && simulation && !m_CuttingPanelActive && !m_MeshSimplificationPanelActive
				&& pendingSolverPanel.ResolutionIndex >= 0 && pendingSolverPanel.ResolutionIndex < 4
				&& pendingSolverPanel.BackendIndex >= 0 && pendingSolverPanel.BackendIndex < 4;
			const bool SupportedProjection = !simulation || simulation->GetReducedRagdollCount() == 0 || !pendingSolverPanel.SupportStabilization;
			if (!SupportedProjection) error = "ReducedRagdolls require support stabilization to be disabled.";
			const bool CouplingRequested = (pendingSolverPanel.SoftBodyAttachments && pendingSolverPanel.RigidAttachments)
				|| (pendingSolverPanel.SoftBodyContacts && pendingSolverPanel.SoftRigidContacts);
			const bool SupportedCoupling = !CouplingRequested || (pendingSolverPanel.ResolutionIndex >= 2 && !pendingSolverPanel.SupportStabilization);
			if (!SupportedCoupling) error = "Soft-body rigid coupling requires PBD/XPBD and support stabilization disabled.";
			if (valid && SupportedProjection && SupportedCoupling && m_World->GetRigidAttachmentDemo()
				&& (!pendingSolverPanel.SoftBodyAttachments || !pendingSolverPanel.RigidAttachments))
				m_World->ApplyDeformableAttachmentDemo(pendingSolverPanel.SoftBodyAttachments, pendingSolverPanel.CompliantAttachments, false);
			if (valid && SupportedProjection && SupportedCoupling && m_World->GetSoftRigidContactDemo()
				&& (!pendingSolverPanel.SoftBodyContacts || !pendingSolverPanel.SoftRigidContacts))
				m_World->ApplyDeformableContactDemo(pendingSolverPanel.SoftBodyContacts, false, pendingSolverPanel.PreparedSoftAsset, pendingSolverPanel.EmbeddedSoftRender);
			bool applied = valid && SupportedProjection && SupportedCoupling && simulation->SetSolverConfiguration(
				SolverPanelAlgorithms[pendingSolverPanel.ResolutionIndex],
				SolverPanelBackends[pendingSolverPanel.BackendIndex], pendingSolverPanel.AllowFallback, &error);
			if (applied)
			{
				simulation->SetNarrowPhaseSolver(pendingSolverPanel.PersistentContacts ? NarrowPhaseSolver::PCM : NarrowPhaseSolver::GJKEPA);
				simulation->SetSequentialImpulseWarmStarting(pendingSolverPanel.SequentialWarmStart);
				simulation->SetUnifiedConstraintSolving(pendingSolverPanel.UnifiedConstraints);
				simulation->SetJointDrivesEnabled(pendingSolverPanel.JointDrives);
				simulation->SetJointBreakingEnabled(pendingSolverPanel.JointBreaking);
				auto TemporalSettings = simulation->GetTemporalSubstepSettings();
				TemporalSettings.Enabled = pendingSolverPanel.TemporalSubsteps;
				simulation->SetTemporalSubstepSettings(TemporalSettings);
				auto SleepSettings = simulation->GetIslandSleepingSettings();
				SleepSettings.Enabled = pendingSolverPanel.IslandSleeping;
				simulation->SetIslandSleepingSettings(SleepSettings);
				auto SupportSettings = simulation->GetSupportStabilizationSettings();
				SupportSettings.Enabled = pendingSolverPanel.SupportStabilization;
				simulation->SetSupportStabilizationSettings(SupportSettings);
				auto CCDSettings = simulation->GetCCDContinuationSettings();
				CCDSettings.Enabled = pendingSolverPanel.CCDContinuation;
				simulation->SetCCDContinuationSettings(CCDSettings);
				FrictionPatchSettings Friction = simulation->GetFrictionPatchSettings();
				Friction.Enabled = pendingSolverPanel.FrictionPatches;
				simulation->SetFrictionPatchSettings(Friction);
				simulation->SetTriangleContactsEnabled(pendingSolverPanel.TriangleContacts);
				simulation->SetCollisionFilteringEnabled(pendingSolverPanel.CollisionFiltering);
				ContactGenerationSettings Contacts = simulation->GetContactGenerationSettings();
				Contacts.UseShapeOffsets = pendingSolverPanel.ShapeOffsets;
				Contacts.SpeculativeContacts = pendingSolverPanel.SpeculativeContacts;
				simulation->SetContactGenerationSettings(Contacts);
				if (m_World->GetDemoStrands())
				{
					try
					{
						applied=m_World->ApplyStrandDemo(pendingSolverPanel.StrandRootTwist,pendingSolverPanel.StrandCoupled,(size_t)pendingSolverPanel.StrandLod);
						if (applied) applied=m_World->ApplyStrandCollisionDemo(pendingSolverPanel.StrandContacts,pendingSolverPanel.StrandSelfContacts,pendingSolverPanel.StrandContinuous,pendingSolverPanel.StrandFriction);
					}
					catch (const std::exception& E) { applied=false; error=E.what(); }
				}
				if (m_World->GetDemoPhysicalAnimation()) m_World->ApplyPhysicalAnimationDemo(pendingSolverPanel.AnimationPassive,
					pendingSolverPanel.AnimationArmStrength, pendingSolverPanel.AnimationPhysicsBlend);
				if (m_World->GetDemoCloth()) m_World->ApplyClothDemo(pendingSolverPanel.ClothWind, pendingSolverPanel.ClothAnimation,
					pendingSolverPanel.ClothTethers, pendingSolverPanel.ClothSoftMaterial, pendingSolverPanel.ClothSurfaceContacts, pendingSolverPanel.ClothFriction,
					pendingSolverPanel.ClothSelfCollision, pendingSolverPanel.ClothInterCollision, pendingSolverPanel.ClothContinuous,
					pendingSolverPanel.SoftBodyAttachments, pendingSolverPanel.CompliantAttachments, pendingSolverPanel.RigidAttachments, pendingSolverPanel.SoftBodyContacts, pendingSolverPanel.SoftRigidContacts,
					pendingSolverPanel.CalibratedSoftMaterial, pendingSolverPanel.StVKSoftMaterial, pendingSolverPanel.PreparedSoftAsset, pendingSolverPanel.EmbeddedSoftRender,
					pendingSolverPanel.CorotatedSoftMaterial, pendingSolverPanel.ProjectiveSoftMaterial);
				if (pendingSolverPanel.HasClothDemo) applied = m_World->ApplyClothParameters(
					pendingSolverPanel.ClothAlgorithm == 0 ? ClothModel::DistanceLinks : ClothModel::TriangleStrain,
					(int)pendingSolverPanel.ClothIterations, (int)pendingSolverPanel.ClothSubsteps, pendingSolverPanel.ClothDamping,
					pendingSolverPanel.ClothStretchUCompliance, pendingSolverPanel.ClothStretchVCompliance,
					pendingSolverPanel.ClothShearCompliance, pendingSolverPanel.ClothBendingCompliance) && applied;
			}
			if (applied)
			{
				try { applied = m_World->ApplyPhysicsFieldsDemo(pendingSolverPanel.PhysicsFieldsDemo,pendingSolverPanel.PhysicsFieldFreeze,
					pendingSolverPanel.FieldStrainPulse,pendingSolverPanel.FieldVelocityPulse); }
				catch (const std::exception& E) { applied = false; error = E.what(); }
				if (!applied && error.empty()) error = "Field velocity assignment requires independent rigid bodies.";
			}
			if (applied && pendingSolverPanel.DestructionCacheCommand != DestructionPlaybackCommand::None)
			{
				try { applied = m_World->ApplyDestructionCacheDemo(pendingSolverPanel.DestructionCacheCommand,pendingSolverPanel.DestructionSeekFraction); }
				catch (const std::exception& E) { applied = false; error = E.what(); }
				if (!applied && error.empty()) error = "Record destruction before replaying or seeking.";
			}
			{
				std::lock_guard<std::mutex> lock(m_ImguiMutex);
				m_ImguiSolverPanel.Pending = false;
				m_ImguiSolverPanel.DestructionCacheCommand = DestructionPlaybackCommand::None;
				m_ImguiSolverPanel.FieldStrainPulse = m_ImguiSolverPanel.FieldVelocityPulse = false;
				m_ImguiSolverPanel.Status = applied ? "Applied to current scene."
					: (error.empty() ? "Solver change is unavailable." : error);
				if (applied)
				{
					m_ImguiSolverPanel.AnimationPassive = pendingSolverPanel.AnimationPassive;
					m_ImguiSolverPanel.AnimationArmStrength = pendingSolverPanel.AnimationArmStrength;
					m_ImguiSolverPanel.AnimationPhysicsBlend = pendingSolverPanel.AnimationPhysicsBlend;
					m_ImguiSolverPanel.ClothWind = pendingSolverPanel.ClothWind;
					m_ImguiSolverPanel.ClothAnimation = pendingSolverPanel.ClothAnimation;
					m_ImguiSolverPanel.ClothTethers = pendingSolverPanel.ClothTethers;
					m_ImguiSolverPanel.ClothSoftMaterial = pendingSolverPanel.ClothSoftMaterial;
					m_ImguiSolverPanel.ClothSurfaceContacts = pendingSolverPanel.ClothSurfaceContacts;
					m_ImguiSolverPanel.ClothFriction = pendingSolverPanel.ClothFriction;
					m_ImguiSolverPanel.ClothSelfCollision = pendingSolverPanel.ClothSelfCollision;
					m_ImguiSolverPanel.ClothInterCollision = pendingSolverPanel.ClothInterCollision;
					m_ImguiSolverPanel.ClothContinuous = pendingSolverPanel.ClothContinuous;
					m_ImguiSolverPanel.ClothRigidCoupling = pendingSolverPanel.ClothRigidCoupling;
					m_ImguiSolverPanel.SoftBodyAttachments = pendingSolverPanel.SoftBodyAttachments;
					m_ImguiSolverPanel.CompliantAttachments = pendingSolverPanel.CompliantAttachments;
					m_ImguiSolverPanel.RigidAttachments = pendingSolverPanel.RigidAttachments;
					m_ImguiSolverPanel.SoftBodyContacts = pendingSolverPanel.SoftBodyContacts;
					m_ImguiSolverPanel.SoftRigidContacts = pendingSolverPanel.SoftRigidContacts;
					m_ImguiSolverPanel.CalibratedSoftMaterial = pendingSolverPanel.CalibratedSoftMaterial;
					m_ImguiSolverPanel.StVKSoftMaterial = pendingSolverPanel.StVKSoftMaterial;
					m_ImguiSolverPanel.CorotatedSoftMaterial = pendingSolverPanel.CorotatedSoftMaterial;
					m_ImguiSolverPanel.ProjectiveSoftMaterial = pendingSolverPanel.ProjectiveSoftMaterial;
					m_ImguiSolverPanel.PreparedSoftAsset = pendingSolverPanel.PreparedSoftAsset;
					m_ImguiSolverPanel.EmbeddedSoftRender = pendingSolverPanel.EmbeddedSoftRender;
				}
			}
			UpdateImguiState();
			}
		}
		if (pendingShootProjectile)
		{
			ShootProjectile();
		}
		for (const ConstraintControlState& state : pendingConstraintControls)
		{
			m_World->ApplyConstraintControl(state);
		}
		if (pendingDistance)
		{
			m_CamParam.z = std::max(GetCameraSceneScale() * 1.0e-4f, pendingDistanceValue);
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

		bool pendingLoadMeshSimplificationSource = false;
		if (pendingMeshSimplificationFileIndex >= 0 && pendingMeshSimplificationFileIndex < (int)m_MeshSimplificationFiles.size())
		{
			m_MeshSimplificationFileIndex = pendingMeshSimplificationFileIndex;
			m_MeshSimplificationPath = m_MeshSimplificationFiles[(size_t)pendingMeshSimplificationFileIndex];
			pendingLoadMeshSimplificationSource = true;
		}

		if (pendingBrowseMeshSimplificationFile)
		{
			const std::string selectedMesh = OpenMeshFileDialog();
			if (!selectedMesh.empty())
			{
				m_MeshSimplificationPath = selectedMesh;
				RefreshMeshFileList();
				pendingLoadMeshSimplificationSource = true;
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
		if (pendingCapCutFaces >= 0)
		{
			m_CapCutFaces = pendingCapCutFaces != 0;
			pendingCuttingParams = true;
		}
		if (pendingCuttingSeed)
		{
			m_CuttingSeed = std::max(0, std::min(pendingCuttingSeedValue, 999));
			pendingCuttingParams = true;
		}
		if (pendingMeshSimplificationRatioLevel)
		{
			m_MeshSimplificationRatioLevel = std::max(1, std::min(pendingMeshSimplificationRatioLevelValue, 20));
		}
		if (pendingMeshSimplificationAlgorithm)
		{
			m_MeshSimplificationAlgorithm = std::max(0, std::min(
				pendingMeshSimplificationAlgorithmValue, MeshSimplificationAlgorithm_Count - 1));
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

		if (pendingApplyMeshSimplification && m_MeshSimplificationPanelActive)
		{
			ApplyMeshSimplificationPanel();
		}
		else if (pendingLoadMeshSimplificationSource && m_MeshSimplificationPanelActive)
		{
			LoadMeshSimplificationPanel();
		}
		else if ((pendingMeshSimplificationRatioLevel || pendingMeshSimplificationAlgorithm) && m_MeshSimplificationPanelActive)
		{
			m_MeshSimplificationStatus = "Ready. Click Simplify";
			UpdateImguiState();
		}

		if (pendingGeometryQueryBounds)
		{
			SetShowGeometryQueryBounds(pendingGeometryQueryBoundsValue);
		}
		if (pendingRenderMode)
		{
			SetRenderMode(pendingRenderModeValue);
		}
	}

	void WorldViewer::ApplyVehicleCommands()
	{
		int command;
		unsigned int generation;
		VehiclePanelState controls;
		bool applyControls;
		{
			std::lock_guard<std::mutex> lock(m_ImguiMutex);
			command = m_PendingVehicleCommand;
			generation = m_PendingVehicleGeneration;
			m_PendingVehicleCommand = 0;
			controls = m_PendingVehiclePanel; applyControls = m_PendingVehicleControls; m_PendingVehicleControls = false;
		}
		if (applyControls && controls.SceneGeneration == m_SceneGeneration)
			if (auto* Vehicle = m_World->GetVehicleDynamics(0))
			{
				VehicleControls C; C.Throttle = controls.Throttle; C.Steering = controls.Steering; C.Brake = controls.Brake;
				C.Handbrake = controls.Handbrake; C.Clutch = controls.Clutch; C.Gear = controls.GearIndex - 1;
				Vehicle->SetControls(C); Vehicle->SetDifferential((VehicleDifferential)controls.Differential);
			}
		if (generation != m_SceneGeneration || m_World->GetVehicleCount() == 0) return;
		if (command == 1) m_VehicleSlowMotion = !m_VehicleSlowMotion;
		else if (command == 2)
		{
			m_CamCenter = m_World->GetVehicle(0)->GetWorldTransform().pos;
			m_CamParam = Vector3(1.75f, 0.42f, 7.5f);
			UpdateCamera();
		}
	}

	void WorldViewer::UpdateImguiState()
	{
		PhysicsWorld* simulation = m_World ? m_World->GetSimulation() : nullptr;
		std::vector<ConstraintControlState> constraintControls = m_World
			? m_World->GetConstraintControls() : std::vector<ConstraintControlState>();
		std::vector<std::string> terrainObjectIds = m_World
			? m_World->GetTerrainObjectIds() : std::vector<std::string>();
		std::lock_guard<std::mutex> lock(m_ImguiMutex);
		m_ImguiRobotPanel.Active = m_World && m_World->GetRobotDemoController();
		m_ImguiRobotPanel.SceneGeneration = m_SceneGeneration;
		if (m_ImguiRobotPanel.Active)
		{
			m_ImguiRobotPanel.Backend = m_World->GetWalkingRobotBackend();
			m_ImguiRobotPanel.Interactive = m_World->IsSpiderWalkingDemo();
			m_ImguiRobotPanel.WalkMode = m_World->GetRobotWalkMode();
			m_ImguiRobotPanel.AutoWalk = m_RobotAutoWalk;
			m_ImguiRobotPanel.Command = m_World->GetRobotDriveCommand();
			m_ImguiRobotPanel.Heading = m_World->GetRobotDriveHeading();
		}
		m_RobotInputActive = m_World && m_World->IsSpiderWalkingDemo() && !m_RobotAutoWalk
#ifdef RIEMANN_SPIDER_TRAINING
			&& !m_SpiderTrainingMode
#endif
			;
		m_CharacterInputActive = m_World && m_World->GetDemoCharacter();
		m_ImguiAerodynamicsPanel.Active = m_AerodynamicsSimulation && m_AerodynamicsSimulation->HasTracerField();
		UpdateSnowPanelState();
		m_ImguiVehiclePanel.Active = m_World && m_World->GetVehicleCount() > 0;
		m_ImguiVehiclePanel.SceneGeneration = m_SceneGeneration;
		m_ImguiViewerPaused = m_ViewerPaused;
		m_ImguiVehiclePanel.SlowMotion = m_VehicleSlowMotion;
		m_ImguiVehiclePanel.Dynamics = m_World && m_World->GetVehicleDynamics(0);
		if (m_ImguiVehiclePanel.Dynamics)
		{
			const auto* Vehicle = m_World->GetVehicleDynamics(0); const auto& C = Vehicle->GetControls();
			m_ImguiVehiclePanel.Throttle = C.Throttle; m_ImguiVehiclePanel.Steering = C.Steering; m_ImguiVehiclePanel.Brake = C.Brake;
			m_ImguiVehiclePanel.Handbrake = C.Handbrake; m_ImguiVehiclePanel.Clutch = C.Clutch; m_ImguiVehiclePanel.GearIndex = C.Gear + 1;
			m_ImguiVehiclePanel.Differential = (int)Vehicle->GetSettings().Differential;
			m_ImguiVehiclePanel.Speed = Vehicle->GetChassis()->V.Length();
			constexpr float RadiansPerSecondToRPM = 60.0f / (2 * 3.14159265358979323846f);
			m_ImguiVehiclePanel.EngineRPM = Vehicle->GetEngineSpeed() * RadiansPerSecondToRPM;
			m_ImguiVehiclePanel.Load = 0; m_ImguiVehiclePanel.GroundedWheels = 0;
			for (const auto& W : Vehicle->GetWheels())
			{ m_ImguiVehiclePanel.Load += W.SuspensionForce; m_ImguiVehiclePanel.GroundedWheels += W.Grounded ? 1 : 0; }
		}
		if (m_ImguiVehiclePanel.Active)
		{
			m_ImguiVehiclePanel.Displacement = m_World->GetVehicle(0)->GetMaxDisplacement();
			m_ImguiVehiclePanel.PlasticStrain = m_World->GetVehicle(0)->GetMaxPlasticStrain();
		}
		if (m_AerodynamicsSimulation && m_AerodynamicsSimulation->HasTracerField())
		{
			if (m_ImguiAerodynamicsPanel.SceneGeneration != m_SceneGeneration)
			{
				m_ImguiAerodynamicsPanel.SceneGeneration = m_SceneGeneration;
				m_ImguiAerodynamicsPanel.Backend = (m_AerodynamicsSimulation->GetBackend() == FluidBackend::CPU ? 0 : m_AerodynamicsSimulation->GetBackend() == FluidBackend::CUDA ? 1 : 2);
				const Vector3i requestedResolution = m_World->GetFieldSimulation().Aerodynamics.Resolution;
				m_ImguiAerodynamicsPanel.GridScale = std::max(0.5f, std::min(requestedResolution.x / 80.0f, 1.5f));
				m_ImguiAerodynamicsPanel.Status = m_AerodynamicsSimulation->GetBackend() == m_World->GetFieldSimulation().Aerodynamics.Backend
					? std::string() : (m_AerodynamicsSimulation->GetBackend() == FluidBackend::CPU
						? "CUDA unavailable; interactive CPU grid" : "CUDA unavailable; using Compute Shader");
				m_ImguiPendingAerodynamicsApply = false;
			}
			m_ImguiAerodynamicsPanel.RigidCoupling = m_AerodynamicsCoupling != nullptr;
			m_ImguiAerodynamicsPanel.RigidBodyCount = m_World->GetAerodynamicsRigidBodyCount();
			m_ImguiAerodynamicsPanel.ActualBackend = (m_AerodynamicsSimulation->GetBackend() == FluidBackend::CPU ? 0 : m_AerodynamicsSimulation->GetBackend() == FluidBackend::CUDA ? 1 : 2);
			m_ImguiAerodynamicsPanel.Resolution = m_AerodynamicsSimulation->GetSettings().Resolution;
		}
		if (simulation)
		{
			RefreshSolverPanelTelemetry(m_ImguiSolverPanel,*m_World);
			m_ImguiSolverPanel.HasReducedRagdolls = simulation->GetReducedRagdollCount() != 0;
			auto* Animation = m_World->GetDemoPhysicalAnimation();
			m_ImguiSolverPanel.HasPhysicalAnimation = Animation != nullptr;
			auto* DemoCloth = m_World->GetDemoCloth();
			if (!DemoCloth && m_World->GetName() == "Cloth" && simulation->GetClothCount() > 0) DemoCloth = simulation->GetCloth(0);
			m_ImguiSolverPanel.HasClothDemo = DemoCloth != nullptr;
			m_ImguiSolverPanel.AnimationState = Animation ? (int)Animation->GetState() : 0;
			m_ImguiSolverPanel.ActiveResolutionIndex = GetSolverPanelIndex(simulation->GetResolutionSolver());
			m_ImguiSolverPanel.ActiveBackendIndex = GetBackendPanelIndex(simulation->GetSolverBackend());
			m_ImguiSolverPanel.ActiveRequestedBackendIndex = GetBackendPanelIndex(simulation->GetRequestedSolverBackend());
			if (m_ImguiSolverPanel.SceneGeneration != m_SceneGeneration)
			{
				m_ImguiSolverPanel.ResolutionIndex = m_ImguiSolverPanel.ActiveResolutionIndex;
				m_ImguiSolverPanel.BackendIndex = m_ImguiSolverPanel.ActiveRequestedBackendIndex;
				m_ImguiSolverPanel.AllowFallback = simulation->GetAllowBackendFallback();
				m_ImguiSolverPanel.PersistentContacts = simulation->GetNarrowPhaseSolver() == NarrowPhaseSolver::PCM;
				m_ImguiSolverPanel.TriangleContacts = simulation->GetTriangleContactsEnabled();
				m_ImguiSolverPanel.CollisionFiltering = simulation->GetCollisionFilteringEnabled();
				m_ImguiSolverPanel.ShapeOffsets = simulation->GetContactGenerationSettings().UseShapeOffsets;
				m_ImguiSolverPanel.SpeculativeContacts = simulation->GetContactGenerationSettings().SpeculativeContacts;
				m_ImguiSolverPanel.DistanceFieldDemo = m_World->GetDistanceFieldDemo();
				m_ImguiSolverPanel.StrandDemo=m_World->GetStrandDemo(); m_ImguiSolverPanel.StrandRootTwist=m_World->GetStrandDemoTwist(); m_ImguiSolverPanel.StrandCoupled=true; m_ImguiSolverPanel.StrandLod=(int)m_World->GetStrandDemoLod();
				const auto* DemoStrands=m_World->GetDemoStrands(); const auto StrandContacts=DemoStrands ? DemoStrands->GetSettings().Collision : StrandCollisionSettings();
				m_ImguiSolverPanel.StrandContacts=StrandContacts.Enabled; m_ImguiSolverPanel.StrandSelfContacts=StrandContacts.SelfCollision;
				m_ImguiSolverPanel.StrandContinuous=StrandContacts.Continuous; m_ImguiSolverPanel.StrandFriction=StrandContacts.Material.StaticFriction>0;
				m_ImguiSolverPanel.PhysicsFieldsDemo = m_World->GetPhysicsFieldsDemo();
				m_ImguiSolverPanel.PhysicsFieldFreeze = m_World->GetPhysicsFieldFreeze();
				m_ImguiSolverPanel.AuthoredDestructionDemo = m_World->GetAuthoredDestructionDemo();
				m_ImguiSolverPanel.DestructionEventsDemo = m_World->GetDestructionEventsDemo();
				m_ImguiSolverPanel.DestructionCacheDemo = m_World->GetDestructionCacheDemo();
				m_ImguiSolverPanel.CharacterDemo = m_World->GetCharacterDemo();
				m_ImguiSolverPanel.VehicleDynamicsDemo = m_World->GetVehicleDynamicsDemo();
				m_ImguiSolverPanel.ReducedRagdollDemo = m_World->GetReducedRagdollDemo();
				m_ImguiSolverPanel.PhysicalAnimationDemo = m_World->GetPhysicalAnimationDemo();
				m_ImguiSolverPanel.ClothModelDemo = m_World->GetClothModelDemo();
				if (DemoCloth)
				{
					const auto& Settings = DemoCloth->GetSettings();
					m_ImguiSolverPanel.ClothAlgorithm = Settings.Model == ClothModel::DistanceLinks ? 0 : 1;
					m_ImguiSolverPanel.ClothIterations = (float)Settings.SolverIterations;
					m_ImguiSolverPanel.ClothSubsteps = (float)Settings.Substeps;
					m_ImguiSolverPanel.ClothDamping = Settings.Damping;
					m_ImguiSolverPanel.ClothStretchUCompliance = Settings.Material.StretchUCompliance;
					m_ImguiSolverPanel.ClothStretchVCompliance = Settings.Material.StretchVCompliance;
					m_ImguiSolverPanel.ClothShearCompliance = Settings.Material.ShearCompliance;
					m_ImguiSolverPanel.ClothBendingCompliance = Settings.Material.BendingCompliance;
				}
				m_ImguiSolverPanel.ClothWind = !DemoCloth || DemoCloth->GetSettings().Wind.NormalDrag > 0;
				m_ImguiSolverPanel.ClothAnimation = DemoCloth && DemoCloth->GetSettings().AnimationDrive.Enabled;
				m_ImguiSolverPanel.ClothTethers = !DemoCloth || DemoCloth->GetSettings().Material.TethersEnabled;
				m_ImguiSolverPanel.ClothSoftMaterial = m_World->GetClothDemoSoftMaterial();
				m_ImguiSolverPanel.ClothSurfaceContacts = DemoCloth && DemoCloth->GetSettings().Collision.SurfaceContacts;
				m_ImguiSolverPanel.ClothFriction = !DemoCloth || DemoCloth->GetSettings().Collision.Material.StaticFriction > 0;
				m_ImguiSolverPanel.ClothSelfCollision = DemoCloth && DemoCloth->GetSettings().Collision.SelfCollision;
				m_ImguiSolverPanel.ClothInterCollision = DemoCloth && DemoCloth->GetSettings().Collision.InterCollision;
				m_ImguiSolverPanel.ClothContinuous = DemoCloth && DemoCloth->GetSettings().Collision.Continuous;
				m_ImguiSolverPanel.ClothRigidCoupling = DemoCloth && DemoCloth->HasRigidCoupling();
				m_ImguiSolverPanel.SoftBodyAttachments = m_World->GetDemoDeformableBody(0) && m_World->GetDemoDeformableBody(1);
				m_ImguiSolverPanel.CompliantAttachments = m_World->GetCompliantAttachmentDemo();
				m_ImguiSolverPanel.RigidAttachments = m_World->GetRigidAttachmentDemo();
				m_ImguiSolverPanel.SoftBodyContacts = m_World->GetDemoContactBody(0) && m_World->GetDemoContactBody(1);
				m_ImguiSolverPanel.SoftRigidContacts = m_World->GetSoftRigidContactDemo();
				m_ImguiSolverPanel.CalibratedSoftMaterial = m_World->GetCalibratedSoftMaterialDemo();
				m_ImguiSolverPanel.StVKSoftMaterial = m_World->GetStVKSoftMaterialDemo();
				m_ImguiSolverPanel.CorotatedSoftMaterial = m_World->GetCorotatedSoftMaterialDemo();
				m_ImguiSolverPanel.ProjectiveSoftMaterial = m_World->GetProjectiveSoftMaterialDemo();
				m_ImguiSolverPanel.PreparedSoftAsset = m_World->GetPreparedSoftAssetDemo();
				if (m_ImguiSolverPanel.PreparedSoftAsset) m_ImguiSolverPanel.EmbeddedSoftRender = m_World->GetEmbeddedSoftRenderDemo();
				m_ImguiSolverPanel.AnimationPassive = Animation && Animation->GetState() == PhysicalAnimationState::Passive;
				m_ImguiSolverPanel.AnimationArmStrength = Animation ? Animation->GetControl((size_t)RagdollBone::LeftUpperArm).Strength : 1;
				m_ImguiSolverPanel.AnimationPhysicsBlend = Animation ? Animation->GetControl(0).PhysicsBlend : 0.5f;
				m_ImguiSolverPanel.CharacterBox = m_World->GetCharacterBox();
				m_ImguiSolverPanel.SequentialWarmStart = simulation->GetSequentialImpulseWarmStarting();
				m_ImguiSolverPanel.UnifiedConstraints = simulation->GetUnifiedConstraintSolving();
				m_ImguiSolverPanel.FrictionPatches = simulation->GetFrictionPatchSettings().Enabled;
				m_ImguiSolverPanel.JointDrives = simulation->GetJointDrivesEnabled();
				m_ImguiSolverPanel.JointBreaking = simulation->GetJointBreakingEnabled();
				m_ImguiSolverPanel.CCDContinuation = simulation->GetCCDContinuationSettings().Enabled;
				m_ImguiSolverPanel.TemporalSubsteps = simulation->GetTemporalSubstepSettings().Enabled;
				m_ImguiSolverPanel.SupportStabilization = simulation->GetSupportStabilizationSettings().Enabled;
				m_ImguiSolverPanel.IslandSleeping = simulation->GetIslandSleepingSettings().Enabled;
				m_ImguiSolverPanel.SceneGeneration = m_SceneGeneration;
				m_ImguiSolverPanel.Pending = false;
				m_ImguiSolverPanel.Status.clear();
				m_ImguiPendingSolverSwitch = false;
			}
		}
		m_ImguiSolverPanel.Enabled = simulation && !m_CuttingPanelActive && !m_MeshSimplificationPanelActive;
		m_ImguiSolverPanel.SolverSwitchable = m_ImguiSolverPanel.Enabled && !m_FluidDemoActive && !m_GasDemoActive
			&& !m_AerodynamicsSimulation && !m_SnowSimulation && !m_SandSimulation
			&& simulation->GetReducedRagdollCount() == 0;
		if (m_ImguiTerrainBrush.SceneGeneration != m_SceneGeneration)
		{
			m_ImguiTerrainBrush = TerrainBrushState();
			m_ImguiTerrainBrush.ControlRange = std::max(20.0f, m_CamParam.z * 2.0f);
			m_ImguiTerrainBrush.SceneGeneration = m_SceneGeneration;
			m_ImguiTerrainStatus.clear();
		}
		m_ImguiTerrainObjectIds = std::move(terrainObjectIds);
		if (std::find(m_ImguiTerrainObjectIds.begin(), m_ImguiTerrainObjectIds.end(), m_ImguiTerrainBrush.ObjectId)
			== m_ImguiTerrainObjectIds.end())
		{
			m_ImguiTerrainBrush.ObjectId = m_ImguiTerrainObjectIds.empty() ? std::string() : m_ImguiTerrainObjectIds.front();
			m_ImguiTerrainStatus.clear();
		}
		m_ImguiSceneName = m_CurrentSceneDisplayName;
		m_ImguiRagdollDemoReduced = m_RagdollDemoReduced;
		m_ImguiSceneFiles = m_SceneFiles;
		m_ImguiCuttingObjFiles = m_CuttingObjFiles;
		m_ImguiCuttingObjPath = m_CuttingObjPath;
		m_ImguiCuttingStatus = m_CuttingStatus;
		m_ImguiMeshSimplificationFiles = m_MeshSimplificationFiles;
		m_ImguiMeshSimplificationPath = m_MeshSimplificationPath;
		m_ImguiMeshSimplificationStatus = m_MeshSimplificationStatus;
		m_ImguiConstraintControls = std::move(constraintControls);
		if (m_CuttingPanelActive)
		{
			m_ImguiDemoIndex = (int)m_SceneFiles.size() + (IsConvexDecompositionMode(m_CuttingMode) ? 1 : 0);
		}
		else if (m_MeshSimplificationPanelActive)
		{
			m_ImguiDemoIndex = (int)m_SceneFiles.size() + 2;
		}
		else
		{
			m_ImguiDemoIndex = GetCurrentSceneDemoIndex();
		}
		m_ImguiObjectCount = m_World ? m_World->GetObjects().size() : 0;
		m_ImguiCameraDistance = m_CamParam.z;
		m_ImguiCuttingPanelActive = m_CuttingPanelActive;
		m_ImguiMeshSimplificationPanelActive = m_MeshSimplificationPanelActive;
		m_ImguiCuttingObjIndex = m_CuttingObjIndex;
		m_ImguiCuttingMode = m_CuttingMode;
		m_ImguiCuttingPieceCount = m_CuttingPieceCount;
		m_ImguiCuttingPiecesX = m_CuttingPiecesX;
		m_ImguiCuttingPiecesY = m_CuttingPiecesY;
		m_ImguiCuttingPiecesZ = m_CuttingPiecesZ;
		m_ImguiCuttingSeed = m_CuttingSeed;
		m_ImguiCapCutFaces = m_CapCutFaces;
		m_ImguiMeshSimplificationFileIndex = m_MeshSimplificationFileIndex;
		m_ImguiMeshSimplificationAlgorithm = m_MeshSimplificationAlgorithm;
		m_ImguiMeshSimplificationRatioLevel = m_MeshSimplificationRatioLevel;
		m_ImguiCuttingSeparation = m_CuttingSeparation;
		m_ImguiShowGeometryQueryBounds = m_ShowGeometryQueryBounds;
		m_ImguiRenderMode = m_RenderMode;
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

	float WorldViewer::GetCameraSceneScale() const
	{
		if (m_MeshSimplificationPanelActive)
		{
			return std::max(m_MeshSimplificationMaxSeparation, 1.0e-4f);
		}
		if (m_CuttingPanelActive)
		{
			return std::max(m_CuttingSceneScale, 1.0e-4f);
		}
		return std::max(m_CamParam.z, 1.0e-4f);
	}

	float WorldViewer::GetCameraControlScale() const
	{
		return std::max(GetCameraSceneScale(), m_CamParam.z);
	}

	Ray3 WorldViewer::BuildSceneRay(int x, int y, int width, int height) const
	{
		const CameraDesc camera = GetViewCamera();

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
			ofn.lpstrFilter = "Mesh Files\0*.obj;*.uasset\0OBJ Files\0*.obj\0UE5 StaticMesh Assets\0*.uasset\0All Files\0*.*\0";
		ofn.lpstrDefExt = "obj";
		ofn.Flags = OFN_FILEMUSTEXIST | OFN_PATHMUSTEXIST | OFN_NOCHANGEDIR;
		return GetOpenFileNameA(&ofn) ? std::string(fileName) : std::string();
#else
		return std::string();
#endif
	}

	std::string WorldViewer::OpenMeshFileDialog() const
	{
#if defined(_WIN32)
		char fileName[MAX_PATH] = {};
		if (!m_MeshSimplificationPath.empty())
		{
			snprintf(fileName, sizeof(fileName), "%s", m_MeshSimplificationPath.c_str());
		}
		OPENFILENAMEA ofn = {};
		ofn.lStructSize = sizeof(ofn);
		ofn.lpstrFile = fileName;
		ofn.nMaxFile = sizeof(fileName);
			ofn.lpstrFilter = "Mesh Files\0*.obj;*.flat;*.uasset\0OBJ Files\0*.obj\0Flat Files\0*.flat\0UE5 StaticMesh Assets\0*.uasset\0All Files\0*.*\0";
		ofn.lpstrDefExt = "obj";
		ofn.Flags = OFN_FILEMUSTEXIST | OFN_PATHMUSTEXIST | OFN_NOCHANGEDIR;
		return GetOpenFileNameA(&ofn) ? std::string(fileName) : std::string();
#else
		return std::string();
#endif
	}

#ifdef RIEMANN_SPIDER_TRAINING
	std::string WorldViewer::ElmanExecutableSettingsPath() const
	{
		const std::string root = m_SceneDirectory.empty() ? std::string() : m_SceneDirectory + "../../";
		return root + "DerivedData/Settings/ElmanExecutable.txt";
	}

	void WorldViewer::LoadElmanExecutablePath()
	{
		std::string path;
		std::ifstream settings(ElmanExecutableSettingsPath(), std::ios::binary);
		if (settings) std::getline(settings, path);
		if (!path.empty() && path.back() == '\r') path.pop_back();
		const bool saved = !path.empty();
		if (!saved)
		{
			const std::string root = m_SceneDirectory.empty() ? std::string() : m_SceneDirectory + "../../";
			path = root + "../ElmanLLM/Binaries/Windows/elman.exe";
		}
		char absolute[MAX_PATH] = {};
		const DWORD length = GetFullPathNameA(path.c_str(), MAX_PATH, absolute, nullptr);
		if (length > 0 && length < MAX_PATH) path = absolute;
		const DWORD attributes = GetFileAttributesA(path.c_str());
		const bool valid = attributes != INVALID_FILE_ATTRIBUTES && !(attributes & FILE_ATTRIBUTE_DIRECTORY);
		std::lock_guard<std::mutex> lock(m_ImguiMutex);
		m_ElmanExecutablePath = saved || valid ? path : std::string();
		m_ElmanExecutableStatus = saved && !valid ? "Saved Elman EXE is missing" : std::string();
	}

	bool WorldViewer::SaveElmanExecutablePath(const std::string& path)
	{
		char absolute[MAX_PATH] = {};
		const DWORD length = GetFullPathNameA(path.c_str(), MAX_PATH, absolute, nullptr);
		const DWORD attributes = length > 0 && length < MAX_PATH ? GetFileAttributesA(absolute) : INVALID_FILE_ATTRIBUTES;
		if (attributes == INVALID_FILE_ATTRIBUTES || (attributes & FILE_ATTRIBUTE_DIRECTORY))
		{
			std::lock_guard<std::mutex> lock(m_ImguiMutex);
			m_ElmanExecutableStatus = "Elman executable file is missing";
			return false;
		}
		const std::string root = m_SceneDirectory.empty() ? std::string() : m_SceneDirectory + "../../";
		CreateDirectoryA((root + "DerivedData").c_str(), nullptr);
		CreateDirectoryA((root + "DerivedData/Settings").c_str(), nullptr);
		std::ofstream settings(ElmanExecutableSettingsPath(), std::ios::binary | std::ios::trunc);
		if (settings) settings << absolute << '\n';
		settings.close();
		if (!settings)
		{
			std::lock_guard<std::mutex> lock(m_ImguiMutex);
			m_ElmanExecutableStatus = "Could not save Elman executable path";
			return false;
		}
		std::lock_guard<std::mutex> lock(m_ImguiMutex);
		m_ElmanExecutablePath = absolute;
		m_ElmanExecutableStatus = "Elman executable path saved";
		return true;
	}

	std::string WorldViewer::OpenElmanExecutableDialog() const
	{
		char fileName[MAX_PATH] = {};
		{
			std::lock_guard<std::mutex> lock(m_ImguiMutex);
			if (m_ElmanExecutablePath.size() < sizeof(fileName))
				snprintf(fileName, sizeof(fileName), "%s", m_ElmanExecutablePath.c_str());
		}
		OPENFILENAMEA ofn = {};
		ofn.lStructSize = sizeof(ofn);
		ofn.lpstrFile = fileName;
		ofn.nMaxFile = sizeof(fileName);
		ofn.lpstrFilter = "Elman Executable\0elman.exe\0Executable Files\0*.exe\0All Files\0*.*\0";
		ofn.lpstrDefExt = "exe";
		ofn.Flags = OFN_FILEMUSTEXIST | OFN_PATHMUSTEXIST | OFN_NOCHANGEDIR;
		return GetOpenFileNameA(&ofn) ? std::string(fileName) : std::string();
	}
#endif

	std::string WorldViewer::ResolveSceneFile(const char* fileName) const
	{
		if (fileName == nullptr || fileName[0] == '\0')
		{
			fileName = "01_demo.json";
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
