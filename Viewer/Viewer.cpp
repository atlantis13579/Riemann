#include "Viewer.h"

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
#include "../Src/Deformation/VehicleDeformation.h"
#include "../Renderer/imgui.h"
#include "../Renderer/RiemannRenderer.h"
#include "../Src/Collision/GeometryObject.h"
#include "../Src/Collision/GeometryQuery.h"
#include "../Src/CollisionPrimitive/PrimitiveType.h"
#include "../Src/Fluid/AerodynamicsRigidCoupling.h"
#include "../Src/Fluid/AerodynamicsSimulation.h"
#include "../Src/Fluid/SmokeSimulation.h"
#include "../Src/Fluid/SmokeRigidCoupling.h"
#include "../Renderer/SmokeVolume.h"
#include "../Src/Fluid/FluidRigidCoupling.h"
#include "../Src/Fluid/FluidSimulation.h"
#include "../Src/Modules/Tools/PhysxBinaryParser.h"
#include "../Src/RigidBodyDynamics/KinematicsTree.h"
#include "../Src/RigidBodyDynamics/PhysicsWorld.h"

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
		, m_CuttingMaxSeparation(1.0f)
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
		if (initialScene.empty() && !m_SceneFiles.empty())
		{
			initialScene = m_SceneFiles.front();
		}

		if (!LoadScene(initialScene))
		{
			LoadScene(m_SceneFiles.empty() ? ResolveSceneFile("01_demo.json") : m_SceneFiles.front());
		}

		m_KeyboardEvent = [this](char c)
		{
			if (c != 'i')
			{
				return;
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

		if (desc.Type == "smoke3d")
		{
			if (!m_RenderThread || !m_RenderThread->SupportsDensityVolume()) return false;
			m_SmokeSimulation.reset(new SmokeSimulation());
			if (!m_SmokeSimulation->Initialize(desc.Smoke))
			{
				m_SmokeSimulation.reset();
				return false;
			}
			m_GasDemoActive = true;
			m_GasRenderFrame = 0;
			m_SmokePaused = false;
			if (desc.RigidCoupling)
			{
				SmokeRigidCouplingSettings coupling;
				coupling.AirDensity = desc.AmbientDensity;
				coupling.MaxLinearAcceleration = desc.MaxLinearAcceleration;
				coupling.MaxAngularAcceleration = desc.MaxAngularAcceleration;
				m_SmokeCoupling = new SmokeRigidCoupling(m_SmokeSimulation.get(), coupling);
				m_World->GetSimulation()->AddForceField(m_SmokeCoupling);
				if (!SynchronizeSmokeObstacles()) { DeactivateGasDemo(); return false; }
			}
			return true;
		}

		if (desc.Type == "aerodynamics3d" || desc.Type == "aerodynamics_simulation")
		{
			AerodynamicsSimulationSettings settings;
			settings.Method = method;
			settings.Backend = ParseFluidBackend(desc.Backend);
			settings.AllowBackendFallback = desc.AllowBackendFallback;
			settings.BoundsMin = desc.BoundsMin;
			settings.BoundsMax = desc.BoundsMax;
			settings.Resolution = desc.Resolution;
			settings.Gravity = desc.Gravity;
			settings.AmbientDensity = desc.AmbientDensity;
			settings.AmbientPressure = desc.AmbientPressure;
			settings.SpeedOfSound = desc.SpeedOfSound;
			settings.AdiabaticExponent = desc.AdiabaticExponent;
			settings.DynamicViscosity = desc.DynamicViscosity;
			settings.ArtificialViscosity = desc.ArtificialViscosity;
			settings.ParticleSpacing = desc.ParticleSpacing;
			settings.SmoothingRadius = desc.SmoothingRadius;
			settings.BoundaryRestitution = desc.BoundaryRestitution;
			settings.MaxTimeStep = desc.MaxTimeStep;
			settings.AcousticCFL = desc.AcousticCFL;
			settings.MaxSubsteps = desc.MaxSubsteps;
			m_AerodynamicsSimulation.reset(new AerodynamicsSimulation(settings));
			if (!m_AerodynamicsSimulation->IsInitialized())
			{
				m_AerodynamicsSimulation.reset();
				return false;
			}
			for (const SceneFieldRegion& region : desc.Regions)
			{
				if (region.SetDensity)
				{
					m_AerodynamicsSimulation->SetDensityBox(region.Min, region.Max, region.Density);
				}
				if (region.SetVelocity)
				{
					m_AerodynamicsSimulation->SetVelocityBox(region.Min, region.Max, region.Velocity);
				}
			}
			if (desc.RigidCoupling)
			{
				AerodynamicsRigidCouplingSettings coupling;
				coupling.SurfaceSamplesPerAxis = desc.SurfaceSamplesPerAxis;
				coupling.DragCoefficient = desc.DragCoefficient;
				coupling.MaxLinearAcceleration = desc.MaxLinearAcceleration;
				coupling.MaxAngularAcceleration = desc.MaxAngularAcceleration;
				m_AerodynamicsCoupling = new AerodynamicsRigidCoupling(
					m_AerodynamicsSimulation.get(), coupling);
				if (!m_AerodynamicsCoupling->IsValid()
					|| !m_World->GetSimulation()->AddForceField(m_AerodynamicsCoupling))
				{
					delete m_AerodynamicsCoupling;
					m_AerodynamicsCoupling = nullptr;
					m_AerodynamicsSimulation.reset();
					return false;
				}
			}
			m_GasDemoActive = true;
			m_GasRenderFrame = 0;
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
		if (m_AerodynamicsCoupling != nullptr && m_World != nullptr
			&& m_World->GetSimulation() != nullptr)
		{
			m_World->GetSimulation()->RemoveForceField(m_AerodynamicsCoupling);
		}
		m_AerodynamicsCoupling = nullptr;
		m_AerodynamicsSimulation.reset();
		if (m_SmokeCoupling && m_World && m_World->GetSimulation())
			m_World->GetSimulation()->RemoveForceField(m_SmokeCoupling);
		m_SmokeCoupling = nullptr;
		m_SmokeSimulation.reset();
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


	bool WorldViewer::SynchronizeSmokeObstacles()
	{
		if (!m_SmokeCoupling) return true;
		std::vector<Geometry*> geometries;
		for (const SceneObjectInstance& object : m_World->GetObjects())
		{
			const RigidBody* body = object.GeometryPtr ? object.GeometryPtr->GetParent<RigidBody>() : nullptr;
			// Stage decoration and collision walls must not alter the empty smoke plume.
			if (body && body->mRigidType != RigidType::Static) geometries.push_back(object.GeometryPtr);
		}
		return m_SmokeCoupling->Synchronize(geometries);
	}

	void WorldViewer::UpdateGasDemo(float deltaTime)
	{
		if (m_GasDemoActive && m_SmokeSimulation)
		{
			if (m_SmokePaused) return;
			if (!SynchronizeSmokeObstacles()) { m_SmokePaused = true; return; }
			m_SmokeSimulation->Step(deltaTime);
			if (!m_SmokeSimulation->IsInitialized())
			{
				m_SmokePaused = true;
				std::lock_guard<std::mutex> lock(m_ImguiMutex);
				m_ImguiSmokePanel.Status = "Smoke device error";
				return;
			}
			SubmitGasDensity();
			return;
		}
		if (!m_GasDemoActive || m_AerodynamicsSimulation == nullptr) return;
		m_AerodynamicsSimulation->Step(deltaTime);
		++m_GasRenderFrame;
		SubmitGasDensity();
	}

	void WorldViewer::SubmitGasDensity()
	{
		if (m_GasDemoActive && m_SmokeSimulation && m_RenderThread)
		{
			std::shared_ptr<DensityVolumeDesc> volume(new DensityVolumeDesc());
			const SceneFieldSimulationDesc& field = m_World->GetFieldSimulation();
			BuildSmokeDensityVolume(*m_SmokeSimulation, volume.get(), field.SmokeExtinction, field.SmokeColor);
			m_RenderThread->Submit([volume](Renderer& renderer) { renderer.UpdateDensityVolume(*volume); });
			return;
		}
		if (!m_GasDemoActive || m_AerodynamicsSimulation == nullptr || m_RenderThread == nullptr) return;
		if (m_RenderThread->SupportsDensityVolume())
		{
			std::shared_ptr<DensityVolumeDesc> volume(new DensityVolumeDesc());
			if (!BuildGasDensityVolume(*m_AerodynamicsSimulation, volume.get(),
				m_World->GetFieldSimulation().VisualizationThreshold)) return;
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
		const std::string resolved = ResolveSceneFile(sceneFile.c_str());
		if (resolved.empty())
		{
			return false;
		}
		DeactivateGasDemo();
		DeactivateFluidDemo();

		++m_SceneGeneration;
		std::string error;
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
		m_VehiclePaused = true;
		m_VehicleSlowMotion = false;
		m_VehiclePendingTime = 0;
		m_CurrentSceneDisplayName = m_World->GetName().empty() ? m_CurrentSceneName : m_World->GetName();
		m_SceneDragActive = false;
		m_FieldDragActive = false;
		m_TankDragActive = false;
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
		ApplyVehicleCommands();
		ApplySnowCommands();
		UpdateFluidDemo();
		if (m_World->GetVehicleCount() > 0)
		{
			if (!m_VehiclePaused)
			{
				// Slow playback changes elapsed time, not the crash solver's timestep.
				m_VehiclePendingTime = std::min(0.1f, m_VehiclePendingTime + dt * (m_VehicleSlowMotion ? 0.25f : 1.0f));
				while (m_VehiclePendingTime >= 1.0f / 120.0f)
				{
					m_World->Step(1.0f / 120.0f);
					m_VehiclePendingTime -= 1.0f / 120.0f;
				}
			}
		}
		else if (!m_SmokeSimulation || !m_SmokePaused) m_World->Step(dt);
		UpdateGasDemo(dt);
		UpdateSnowDemo(dt);
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
				if (m_SmokeSimulation)
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
					if (m_AerodynamicsSimulation) m_AerodynamicsSimulation->ApplyImpulse(
						InteractionPoint, Field.InteractionRadius, Impulse);
					if (m_SmokeSimulation) m_SmokeSimulation->ApplyImpulse(
						InteractionPoint, Field.InteractionRadius, Impulse);
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
		const int panelRight = std::min(width - 10, panelLeft + 280);
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
		bool meshSimplificationPanelActive = false;
		std::string cuttingObjPath;
		std::string cuttingStatus;
		std::vector<ConstraintControlState> constraintControls;
		std::vector<std::string> terrainObjectIds;
		TerrainBrushState terrainBrush;
		std::string terrainStatus;
		SolverPanelState solverPanel;
		SmokePanelState smokePanel;
		SnowPanelState snowPanel;
		VehiclePanelState vehiclePanel;
		std::string meshSimplificationPath;
		std::string meshSimplificationStatus;
		int cuttingMode = 0;
		int cuttingPieceCount = 0;
		int cuttingPiecesX = 0;
		int cuttingPiecesY = 0;
		int cuttingPiecesZ = 0;
		int cuttingSeed = 0;
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
			constraintControls = m_ImguiConstraintControls;
			terrainObjectIds = m_ImguiTerrainObjectIds;
			terrainBrush = m_ImguiTerrainBrush;
			terrainStatus = m_ImguiTerrainStatus;
			solverPanel = m_ImguiSolverPanel;
			smokePanel = m_ImguiSmokePanel;
			snowPanel = m_ImguiSnowPanel;
			vehiclePanel = m_ImguiVehiclePanel;
			cuttingMode = m_ImguiCuttingMode;
			cuttingPieceCount = m_ImguiCuttingPieceCount;
			cuttingPiecesX = m_ImguiCuttingPiecesX;
			cuttingPiecesY = m_ImguiCuttingPiecesY;
			cuttingPiecesZ = m_ImguiCuttingPiecesZ;
			cuttingSeed = m_ImguiCuttingSeed;
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
		snprintf(fpsText, sizeof(fpsText), "Physics: %.1f FPS", physicsFps);
		imguiDrawText(width - 10, 12, IMGUI_ALIGN_RIGHT, fpsText, imguiRGBA(230, 235, 240, 255));
		snprintf(fpsText, sizeof(fpsText), "Render: %.1f FPS", renderFps);
		imguiDrawText(width - 10, 30, IMGUI_ALIGN_RIGHT, fpsText, imguiRGBA(230, 235, 240, 255));
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
		imguiBeginScrollArea("", 10, 10, std::min(280, width - 20), panelHeight, &m_ImguiScroll);

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
		imguiSeparatorLine();

		if (smokePanel.Active)
		{
			imguiLabel("3D Smoke");
			snprintf(text, sizeof(text), "Add Rigid Body (%d / %d)", smokePanel.RigidBodyCount, SceneWorld::MaxSmokeRigidBodies);
			const bool addBody = imguiButton(text, smokePanel.RigidCoupling && smokePanel.RigidBodyCount < SceneWorld::MaxSmokeRigidBodies);
			const char* backends[] = { "CPU", "CUDA", "Compute Shader" };
			const char* qualities[] = { "Interactive", "Balanced", "Detailed" };
			imguiLabel("Smoke backend");
			imguiCombo("", &smokePanel.Backend, backends, 3, true);
			imguiLabel("Grid quality");
			imguiCombo("", &smokePanel.Quality, qualities, 3, true);
			const bool restart = imguiButton("Apply / Restart Smoke");
			const bool pause = imguiButton(smokePanel.Paused ? "Resume Smoke" : "Pause Smoke");
			snprintf(text, sizeof(text), "Running: %s", backends[smokePanel.ActualBackend]);
			imguiValue(text);
			snprintf(text, sizeof(text), "Grid: %d x %d x %d", smokePanel.Resolution.x, smokePanel.Resolution.y, smokePanel.Resolution.z);
			imguiValue(text);
			imguiValue("Drag bodies or stir smoke");
			if (!smokePanel.Status.empty()) imguiValue(smokePanel.Status.c_str());
			std::lock_guard<std::mutex> lock(m_ImguiMutex);
			if (smokePanel.SceneGeneration == m_ImguiSmokePanel.SceneGeneration)
			{
				m_ImguiSmokePanel.Backend = smokePanel.Backend;
				m_ImguiSmokePanel.Quality = smokePanel.Quality;
				if (restart) { m_ImguiPendingSmokePanel = smokePanel; m_ImguiPendingSmokeRestart = true; }
				if (pause) m_ImguiPendingSmokePause = true;
				if (addBody)
				{
					m_ImguiPendingSmokeAddBodies = std::min(m_ImguiPendingSmokeAddBodies + 1, SceneWorld::MaxSmokeRigidBodies);
					m_ImguiPendingSmokeAddGeneration = smokePanel.SceneGeneration;
				}
			}
		}
		if (snowPanel.Active) DrawSnowPanel(snowPanel);
		if (vehiclePanel.Active)
		{
			imguiLabel("Vehicle Crash Test");
			int command = 0;
			if (imguiButton(vehiclePanel.Paused ? "Start / Resume Crash" : "Pause Crash")) command = 1;
			if (imguiButton("Replay Crash")) command = 2;
			if (imguiButton("Inspect Damage")) command = 4;
			if (imguiCheck("Slow motion (0.25x)", vehiclePanel.SlowMotion)) command = 3;
			snprintf(text, sizeof(text), "Maximum dent: %.0f cm", vehiclePanel.Displacement * 100);
			imguiValue(text);
			snprintf(text, sizeof(text), "Plastic strain: %.0f%%", vehiclePanel.PlasticStrain * 100);
			imguiValue(text);
			imguiValue("RMB: orbit around the damage");
			if (command)
			{
				std::lock_guard<std::mutex> lock(m_ImguiMutex);
				m_PendingVehicleCommand = command;
				m_PendingVehicleGeneration = vehiclePanel.SceneGeneration;
			}
		}
		const bool applySolver = !smokePanel.Active && !snowPanel.Active && DrawSolverPanel(solverPanel);
		{
			std::lock_guard<std::mutex> lock(m_ImguiMutex);
			if (solverPanel.SceneGeneration == m_ImguiSolverPanel.SceneGeneration && !m_ImguiSolverPanel.Pending)
			{
				m_ImguiSolverPanel.ResolutionIndex = solverPanel.ResolutionIndex;
				m_ImguiSolverPanel.BackendIndex = solverPanel.BackendIndex;
				m_ImguiSolverPanel.AllowFallback = solverPanel.AllowFallback;
				if (applySolver)
				{
					m_ImguiPendingSolverPanel = solverPanel;
					m_ImguiPendingSolverSwitch = true;
					m_ImguiSolverPanel.Pending = true;
					m_ImguiSolverPanel.Status.clear();
				}
			}
		}
		imguiSeparatorLine();
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

		if (!demoNames.empty())
		{
			if (demoIndex < (int)sceneFiles.size())
			{
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
						if (imguiCheck("Enabled", state.Enabled, true))
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
				const char* modes[] = { "Raise / Lower", "Flatten", "Smooth" };
				changed = imguiCombo("Brush", &terrainBrush.Mode, modes, 3) || changed;
				const float brushRange = terrainBrush.ControlRange;
				const char* axes[] = { "World Center X", "World Center Y", "World Center Z" };
				for (int axis = 0; axis < 3; ++axis)
				{
					changed = imguiSlider(axes[axis], &terrainBrush.Center[axis],
						terrainBrush.ControlCenter[axis] - brushRange, terrainBrush.ControlCenter[axis] + brushRange, 0.1f) || changed;
				}
				changed = imguiSlider("World Radius", &terrainBrush.Radius, 0.1f,
					std::max(brushRange, terrainBrush.Radius), 0.1f) || changed;
				if (terrainBrush.Mode != 2)
				{
					const char* valueLabel = terrainBrush.Mode == 0 ? "Height Delta (local)" : "Target Height (local)";
					changed = imguiSlider(valueLabel, &terrainBrush.Value, -brushRange, brushRange, 0.1f) || changed;
				}
				changed = imguiSlider("Strength", &terrainBrush.Strength, 0.0f, 1.0f, 0.01f, terrainBrush.Mode != 0) || changed;
				const bool apply = imguiButton("Apply Terrain Brush");
				if (changed || apply)
				{
					std::lock_guard<std::mutex> lock(m_ImguiMutex);
					if (terrainBrush.SceneGeneration == m_ImguiTerrainBrush.SceneGeneration)
					{
						m_ImguiTerrainBrush = terrainBrush;
						if (apply)
						{
							m_ImguiPendingTerrainEdits.push_back(terrainBrush);
						}
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
		if (m_GasDemoActive && m_SmokeSimulation)
		{
			gasVolume.reset(new DensityVolumeDesc());
			const SceneFieldSimulationDesc& field = m_World->GetFieldSimulation();
			BuildSmokeDensityVolume(*m_SmokeSimulation, gasVolume.get(), field.SmokeExtinction, field.SmokeColor);
		}
		if (m_GasDemoActive && m_AerodynamicsSimulation != nullptr)
		{
			std::vector<RenderMeshDesc> gasMeshes;
			GasDensityVisualizationSettings visualization;
			visualization.RelativeThreshold = 0.08f;
			if (m_RenderThread && m_RenderThread->SupportsDensityVolume())
			{
				gasVolume.reset(new DensityVolumeDesc());
				if (!BuildGasDensityVolume(*m_AerodynamicsSimulation, gasVolume.get(),
					m_World->GetFieldSimulation().VisualizationThreshold)) gasVolume.reset();
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
			CameraDesc camera = m_World->GetCamera();
			camera.At = m_CamCenter;
			camera.Eye = GetCameraPosition();
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

		m_CamCenter = center;
		m_CamParam = Vector3(1.05f, 0.45f, m_CuttingMaxSeparation * 3.5f);
		m_CuttingStatus = source.Status + (IsConvexDecompositionMode(m_CuttingMode) ? ". Click Apply" : ". Click Apply Cut");

		RebuildRenderScene();

		if (m_RenderThread)
		{
			CameraDesc camera = m_World->GetCamera();
			camera.At = m_CamCenter;
			camera.Eye = GetCameraPosition();
			SetToolBackgroundColor(&camera);
			ConfigureToolCamera(&camera, m_CuttingMaxSeparation, m_CamParam.z);

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
			m_MeshSimplificationPanelActive = false;
			m_CuttingStatus = cuttingResult.Status.empty() ? "Cut failed" : cuttingResult.Status;
			UpdateImguiState();
			return false;
		}

		m_CuttingMode = std::max(0, std::min(m_CuttingMode, (int)MeshCuttingMode_Count - 1));
		m_CuttingPieceCount = std::max(2, std::min(m_CuttingPieceCount, 64));
		m_CuttingPiecesX = std::max(1, std::min(m_CuttingPiecesX, 64));
		m_CuttingPiecesY = std::max(1, std::min(m_CuttingPiecesY, 64));
		m_CuttingPiecesZ = std::max(1, std::min(m_CuttingPiecesZ, 64));
		m_CuttingSeed = std::max(0, std::min(m_CuttingSeed, 999));
		m_CuttingMaxSeparation = cuttingResult.MaxSeparation;
		m_CamCenter = cameraCenter;
		m_CamParam = cameraParam;

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
			m_MeshSimplificationPanelActive = false;
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
			SetToolBackgroundColor(&camera);
			ConfigureToolCamera(&camera, m_CuttingMaxSeparation, m_CamParam.z);

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
		SmokePanelState pendingSmoke;
		bool pendingSmokeRestart = false, pendingSmokePause = false;
		int pendingSmokeAddBodies = 0;
		unsigned int pendingSmokeAddGeneration = 0;
		bool pendingSolverSwitch = false;
		SolverPanelState pendingSolverPanel;
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
		std::vector<ConstraintControlState> pendingConstraintControls;
		std::vector<TerrainBrushState> pendingTerrainEdits;
		{
			std::lock_guard<std::mutex> lock(m_ImguiMutex);
			pendingSolverSwitch = m_ImguiPendingSolverSwitch;
			pendingSmoke = m_ImguiPendingSmokePanel;
			pendingSmokeRestart = m_ImguiPendingSmokeRestart;
			pendingSmokePause = m_ImguiPendingSmokePause;
			m_ImguiPendingSmokeRestart = m_ImguiPendingSmokePause = false;
			pendingSmokeAddBodies = m_ImguiPendingSmokeAddBodies;
			pendingSmokeAddGeneration = m_ImguiPendingSmokeAddGeneration;
			m_ImguiPendingSmokeAddBodies = 0;
			pendingSolverPanel = m_ImguiPendingSolverPanel;
			m_ImguiPendingSolverSwitch = false;
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
			pendingTerrainEdits.swap(m_ImguiPendingTerrainEdits);
		}

		if (pendingSmokeRestart && m_SmokeSimulation && pendingSmoke.SceneGeneration == m_SceneGeneration)
		{
			SmokeSimulationSettings settings = m_World->GetFieldSimulation().Smoke;
			settings.Backend = static_cast<SmokeBackend>(std::max(0, std::min(pendingSmoke.Backend, 2)));
			settings.AllowBackendFallback = false;
			const Vector3i sizes[] = { Vector3i(48, 80, 48), Vector3i(80, 128, 80), Vector3i(112, 184, 112) };
			settings.Resolution = sizes[std::max(0, std::min(pendingSmoke.Quality, 2))];
			std::unique_ptr<SmokeSimulation> next(new SmokeSimulation());
			const bool ready = next->Initialize(settings);
			if (ready)
			{
				if (m_SmokeCoupling) m_SmokeCoupling->SetSimulation(next.get());
				m_SmokeSimulation = std::move(next);
				m_SmokePaused = !SynchronizeSmokeObstacles();
				SubmitGasDensity();
			}
			std::lock_guard<std::mutex> lock(m_ImguiMutex);
			m_ImguiSmokePanel.Status = ready ? "Smoke restarted" : "Backend unavailable";
		}
		if (pendingSmokePause && m_SmokeSimulation) m_SmokePaused = !m_SmokePaused;
		if (pendingSmokeAddBodies > 0 && m_SmokeCoupling && pendingSmokeAddGeneration == m_SceneGeneration)
		{
			for (int I = 0; I < pendingSmokeAddBodies; ++I)
				if (!m_World->AddSmokeRigidBody()) break;
			if (!SynchronizeSmokeObstacles()) m_SmokePaused = true;
			SubmitGasDensity();
		}

		if (pendingDemoIndex >= 0)
		{
			const int meshCuttingDemoIndex = (int)m_SceneFiles.size();
			const int convexDecompositionDemoIndex = meshCuttingDemoIndex + 1;
			const int meshSimplificationDemoIndex = meshCuttingDemoIndex + 2;
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
		if (pendingSolverSwitch && pendingSolverPanel.SceneGeneration == m_SceneGeneration)
		{
			std::string error;
			PhysicsWorld* simulation = m_World ? m_World->GetSimulation() : nullptr;
			const bool valid = simulation && !m_CuttingPanelActive && !m_MeshSimplificationPanelActive
				&& pendingSolverPanel.ResolutionIndex >= 0 && pendingSolverPanel.ResolutionIndex < 4
				&& pendingSolverPanel.BackendIndex >= 0 && pendingSolverPanel.BackendIndex < 4;
			const bool applied = valid && simulation->SetSolverConfiguration(
				SolverPanelAlgorithms[pendingSolverPanel.ResolutionIndex],
				SolverPanelBackends[pendingSolverPanel.BackendIndex], pendingSolverPanel.AllowFallback, &error);
			{
				std::lock_guard<std::mutex> lock(m_ImguiMutex);
				m_ImguiSolverPanel.Pending = false;
				m_ImguiSolverPanel.Status = applied ? "Applied to current scene."
					: (error.empty() ? "Solver change is unavailable." : error);
			}
			UpdateImguiState();
		}
		if (pendingShootProjectile)
		{
			ShootProjectile();
		}
		for (const ConstraintControlState& state : pendingConstraintControls)
		{
			m_World->ApplyConstraintControl(state);
		}
		for (const TerrainBrushState& brush : pendingTerrainEdits)
		{
			if (brush.SceneGeneration != m_SceneGeneration || m_CuttingPanelActive || m_MeshSimplificationPanelActive)
			{
				continue;
			}
			const int changedSamples = m_World->EditTerrain(brush.ObjectId, brush.Center,
				brush.Radius, brush.Value, brush.Mode, brush.Strength);
			char status[96];
			snprintf(status, sizeof(status), "Changed samples: %d", changedSamples);
			std::lock_guard<std::mutex> lock(m_ImguiMutex);
			m_ImguiTerrainStatus = status;
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
		{
			std::lock_guard<std::mutex> lock(m_ImguiMutex);
			command = m_PendingVehicleCommand;
			generation = m_PendingVehicleGeneration;
			m_PendingVehicleCommand = 0;
		}
		if (generation != m_SceneGeneration || m_World->GetVehicleCount() == 0) return;
		if (command == 1) m_VehiclePaused = !m_VehiclePaused;
		else if (command == 2)
		{
			const bool slowMotion = m_VehicleSlowMotion;
			if (LoadScene(m_SceneDirectory + "/" + m_CurrentSceneName))
			{
				m_VehiclePaused = false;
				m_VehicleSlowMotion = slowMotion;
			}
		}
		else if (command == 3) m_VehicleSlowMotion = !m_VehicleSlowMotion;
		else if (command == 4)
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
		m_ImguiSmokePanel.Active = m_SmokeSimulation != nullptr;
		UpdateSnowPanelState();
		m_ImguiVehiclePanel.Active = m_World && m_World->GetVehicleCount() > 0;
		m_ImguiVehiclePanel.SceneGeneration = m_SceneGeneration;
		m_ImguiVehiclePanel.Paused = m_VehiclePaused;
		m_ImguiVehiclePanel.SlowMotion = m_VehicleSlowMotion;
		if (m_ImguiVehiclePanel.Active)
		{
			m_ImguiVehiclePanel.Displacement = m_World->GetVehicle(0)->GetMaxDisplacement();
			m_ImguiVehiclePanel.PlasticStrain = m_World->GetVehicle(0)->GetMaxPlasticStrain();
		}
		if (m_SmokeSimulation)
		{
			if (m_ImguiSmokePanel.SceneGeneration != m_SceneGeneration)
			{
				m_ImguiSmokePanel.SceneGeneration = m_SceneGeneration;
				m_ImguiSmokePanel.Backend = (int)m_SmokeSimulation->GetBackend();
				m_ImguiSmokePanel.Quality = 1;
				m_ImguiSmokePanel.Status.clear();
				m_ImguiPendingSmokeRestart = m_ImguiPendingSmokePause = false;
				m_ImguiPendingSmokeAddBodies = 0;
			}
			m_ImguiSmokePanel.Paused = m_SmokePaused;
			m_ImguiSmokePanel.RigidCoupling = m_SmokeCoupling != nullptr;
			m_ImguiSmokePanel.RigidBodyCount = m_World->GetSmokeRigidBodyCount();
			m_ImguiSmokePanel.ActualBackend = (int)m_SmokeSimulation->GetBackend();
			m_ImguiSmokePanel.Resolution = m_SmokeSimulation->GetSettings().Resolution;
		}
		if (simulation)
		{
			m_ImguiSolverPanel.ActiveResolutionIndex = GetSolverPanelIndex(simulation->GetResolutionSolver());
			m_ImguiSolverPanel.ActiveBackendIndex = GetBackendPanelIndex(simulation->GetSolverBackend());
			m_ImguiSolverPanel.ActiveRequestedBackendIndex = GetBackendPanelIndex(simulation->GetRequestedSolverBackend());
			if (m_ImguiSolverPanel.SceneGeneration != m_SceneGeneration)
			{
				m_ImguiSolverPanel.ResolutionIndex = m_ImguiSolverPanel.ActiveResolutionIndex;
				m_ImguiSolverPanel.BackendIndex = m_ImguiSolverPanel.ActiveRequestedBackendIndex;
				m_ImguiSolverPanel.AllowFallback = simulation->GetAllowBackendFallback();
				m_ImguiSolverPanel.SceneGeneration = m_SceneGeneration;
				m_ImguiSolverPanel.Pending = false;
				m_ImguiSolverPanel.Status.clear();
				m_ImguiPendingSolverSwitch = false;
			}
		}
		m_ImguiSolverPanel.Enabled = simulation && !m_CuttingPanelActive && !m_MeshSimplificationPanelActive;
		if (m_ImguiTerrainBrush.SceneGeneration != m_SceneGeneration)
		{
			m_ImguiTerrainBrush = TerrainBrushState();
			m_ImguiTerrainBrush.Center = m_CamCenter;
			m_ImguiTerrainBrush.ControlCenter = m_CamCenter;
			m_ImguiTerrainBrush.ControlRange = std::max(20.0f, m_CamParam.z * 2.0f);
			m_ImguiTerrainBrush.SceneGeneration = m_SceneGeneration;
			m_ImguiTerrainStatus.clear();
			m_ImguiPendingTerrainEdits.clear();
		}
		m_ImguiTerrainObjectIds = std::move(terrainObjectIds);
		if (std::find(m_ImguiTerrainObjectIds.begin(), m_ImguiTerrainObjectIds.end(), m_ImguiTerrainBrush.ObjectId)
			== m_ImguiTerrainObjectIds.end())
		{
			m_ImguiTerrainBrush.ObjectId = m_ImguiTerrainObjectIds.empty() ? std::string() : m_ImguiTerrainObjectIds.front();
			m_ImguiTerrainStatus.clear();
		}
		m_ImguiSceneName = m_CurrentSceneDisplayName;
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
			return std::max(m_CuttingMaxSeparation, 1.0e-4f);
		}
		return std::max(m_CamParam.z, 1.0e-4f);
	}

	float WorldViewer::GetCameraControlScale() const
	{
		return std::max(GetCameraSceneScale(), m_CamParam.z);
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
