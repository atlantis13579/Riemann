#pragma once

#include <chrono>
#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "../Src/Maths/Vector3.h"
#include "../Src/Maths/Vector4.h"
#include "../Src/CollisionPrimitive/Ray3.h"
#include "SceneWorld.h"
#include "SolverPanel.h"
#include "../Src/Continuum/SnowSimulation.h"
#include "../Src/Continuum/SandSimulation.h"

namespace Riemann
{
	class SpiderTrainingController;
	class Geometry;
	class RenderThread;
	class FluidSimulation;
	class FluidRigidCoupling;
	class AerodynamicsSimulation;
	class AerodynamicsRigidCoupling;

	// Platform supplied file picker for the mesh panels. Windows keeps using
	// GetOpenFileName; the macOS shell installs an NSOpenPanel implementation.
	// "extensions" is a ';' separated list such as "*.obj;*.uasset".
	typedef std::string (*FileDialogHandler)(const char* extensions, const std::string& initialPath);
	void SetFileDialogHandler(FileDialogHandler handler);

	class WorldViewer
	{
	public:
		WorldViewer(RenderThread* renderThread, const std::string& sceneFile);
		~WorldViewer();

		bool LoadScene(const std::string& sceneFile, const std::string& configurationId = std::string());
		void CreateSimulator();
		void UpdateSimulator(float dt);
		void LoadAnimation(const std::string& animName, const std::vector<std::string>& nodes);
		void LoadPhysxScene(const std::string& fileName);
		void LoadVoxelField(const std::string& fileName, const Vector3& c, std::vector<Vector3>& waterList);

		Vector3 GetCameraPosition() const;
		void UpdateCamera();
		void SetViewportSize(int width, int height);
		void KeyboardMsg(char c);
		void SetMovementKey(char c, bool down);
		void UpdateCameraMovement(float dt);
		void MouseMsg(int x, int y, bool rotateButtonDown);
		void SceneRayMsg(int x, int y, int width, int height);
		void SceneDragMsg(int x, int y, int width, int height);
		void SceneDragEndMsg();
		void MoveSnowCapsule(int x, int y, int width, int height);
		void MouseWheel(int zDelta, bool ctrlButtonDown);
		bool IsImguiPanelHovered(int x, int y, int width, int height) const;
		void DrawImgui(int width, int height);

		void AddToRender();

private:
		struct RenderBinding
		{
			Geometry* GeometryPtr = nullptr;
			std::vector<std::string> MeshIds;
		};

		struct CuttingRenderPiece
		{
			Geometry* GeometryPtr = nullptr;
			Vector3 Center = Vector3::Zero();
			Vector3 SeparationOffset = Vector3::Zero();
		};

		struct TerrainBrushState
		{
			std::string ObjectId;
			float ControlRange = 20.0f;
			float Radius = 2.0f;
			float Value = 0.5f;
			float Strength = 0.5f;
			int Mode = 0;
			unsigned int SceneGeneration = 0;
		};

		void RebuildRenderScene();
		void AddGeometryToRender(const std::string& id, Geometry* geometry, const Vector4& color, bool renderBounds);
		bool ActivateConfiguredFieldSimulation();
		void DeactivateFluidDemo();
		void UpdateFluidDemo();
		void SubmitFluidSurface();
		void DeactivateGasDemo();
		void UpdateGasDemo(float deltaTime);
		void SubmitGasDensity();
		bool ActivateSnowDemo(bool Sand = false);
		void DeactivateSnowDemo();
		void UpdateSnowDemo(float dt);
		void SubmitSnowSurface();
		void ApplySnowCommands();
		void UpdateSnowPanelState();
		bool LoadCuttingPanel();
		bool ApplyCuttingPanel();
		bool LoadMeshSimplificationPanel();
		bool ApplyMeshSimplificationPanel();
		void RefreshObjList();
		void RefreshMeshFileList();
		void SubmitCuttingTransforms();
		void FrameCuttingBounds();
		CameraDesc GetViewCamera() const;
		void SubmitTransforms();
		void SubmitDeformableMeshes();
		void SubmitGeometryQueryBounds();
		void ClearGeometryQueryBounds();
		void SetShowGeometryQueryBounds(bool show);
		void SetRenderMode(RenderMode mode);
		float GetCameraSceneScale() const;
		float GetCameraControlScale() const;
		std::string OpenObjFileDialog() const;
		std::string OpenMeshFileDialog() const;
#ifdef RIEMANN_SPIDER_TRAINING
		std::string OpenElmanExecutableDialog() const;
		std::string ElmanExecutableSettingsPath() const;
		void LoadElmanExecutablePath();
		bool SaveElmanExecutablePath(const std::string& path);
#endif
		std::string ResolveSceneFile(const char* fileName) const;
		std::string ResolveTestDataFile(const char* fileName) const;
		void RefreshSceneList();
		void ApplySceneCamera();
		void ApplyImguiCommands();
		void UpdateImguiState();
		int GetCurrentSceneDemoIndex() const;
		void UpdatePhysicsFps();
		Ray3 BuildSceneRay(int x, int y, int width, int height) const;
		void HandleSceneRay(const Ray3& ray);
		bool PaintTerrainAtRay(const Ray3& ray, bool firstStroke);
		bool IsCurrentCuttingGeometry(const Geometry* geometry) const;
		void SetHighlightedGeometry(Geometry* geometry);
		void ShootProjectile();
		static void DrawImguiCallback(int width, int height, void* userData);

	private:
		std::unique_ptr<SceneWorld> m_World;
		std::unique_ptr<FluidSimulation> m_FluidSimulation;
		AerodynamicsSimulation* m_AerodynamicsSimulation = nullptr;
		FluidRigidCoupling* m_FluidCoupling;
		AerodynamicsRigidCoupling* m_AerodynamicsCoupling;
		RenderThread* m_RenderThread;
		std::vector<RenderBinding> m_RenderBindings;
		std::function<void(char)> m_KeyboardEvent;
		Geometry* m_HighlightedGeometry;
		bool m_SceneDragActive;
		bool m_FieldDragActive;
		bool m_TankDragActive;
		Vector3 m_SceneDragPlanePoint;
		Vector3 m_SceneDragPlaneNormal;
		Vector3 m_SceneDragOffset;
		std::chrono::steady_clock::time_point m_SceneDragLastTime;
		Vector3 m_FieldDragPoint;
		int m_TankDragX;
		int m_TankDragY;
		float m_TankTiltX;
		float m_TankTiltZ;
		Quaternion m_TankRotation;
		Vector3 m_TankCenter;

		std::string m_SceneDirectory;
		std::vector<std::string> m_SceneFiles;
		std::string m_CurrentSceneName;
		std::string m_CurrentSceneDisplayName;
		Vector3 m_CamCenter;
		Vector3 m_CamParam;
		int m_ViewportWidth = 1280;
		int m_ViewportHeight = 720;
		bool m_MovementKeys[4];
		bool m_RobotKeys[6]{};
		bool m_RobotInputActive = false, m_RobotAutoWalk = false;
#ifdef RIEMANN_SPIDER_TRAINING
		enum class SpiderPanelMode { Inference, Training };
		std::unique_ptr<SpiderTrainingController> m_SpiderTraining;
		std::atomic<SpiderPanelMode> m_SpiderPanelMode{SpiderPanelMode::Inference};
		std::atomic<bool> m_SpiderTrainingMode{false}, m_SpiderTrainingShowAll{false};
		std::atomic<int> m_SpiderTrainingSelected{0};
		std::atomic<int> m_SpiderTrainingWorldCount{1};
		bool m_PendingSpiderTrainAction = false, m_PendingSpiderShowAllToggle = false;
		unsigned int m_PendingSpiderTrainGeneration = 0;
		int m_PendingSpiderPanelMode = -1;
		unsigned int m_PendingSpiderPanelModeGeneration = 0;
		int m_PendingSpiderSelection = -1;
		bool m_PendingBrowseElmanExecutable = false;
		std::string m_ElmanExecutablePath, m_ElmanExecutableStatus;
#endif
		bool m_CharacterKeys[4]{};
		bool m_CharacterInputActive = false;
		bool m_CharacterJumpHeld = false, m_CharacterJumpRequested = false;
		bool m_CuttingPanelActive;
		bool m_MeshSimplificationPanelActive;
		std::vector<CuttingRenderPiece> m_CuttingPieces;
		std::vector<std::string> m_CuttingObjFiles;
		std::string m_CuttingObjPath;
		std::string m_CuttingStatus;
		std::vector<std::string> m_MeshSimplificationFiles;
		std::string m_MeshSimplificationPath;
		std::string m_MeshSimplificationStatus;
		std::vector<std::string> m_GeometryQueryBoundsMeshIds;
		int m_CuttingMode;
		int m_CuttingPieceCount;
		int m_CuttingPiecesX;
		int m_CuttingPiecesY;
		int m_CuttingPiecesZ;
		int m_CuttingSeed;
		bool m_CapCutFaces = true;
		int m_MeshSimplificationAlgorithm;
		int m_MeshSimplificationRatioLevel;
		float m_CuttingSeparation;
		float m_CuttingSceneScale;
		float m_MeshSimplificationMaxSeparation;
		bool m_ShowGeometryQueryBounds;
		RenderMode m_RenderMode;
		uint32_t m_RenderRevision;
		unsigned int m_SceneGeneration = 0;
		bool m_FluidDemoActive;
		int m_FluidRenderFrame;
		bool m_GasDemoActive;
		int m_GasRenderFrame;

		mutable std::mutex m_ImguiMutex;
		struct VehiclePanelState
		{
			bool Active = false, SlowMotion = true;
			bool Dynamics = false;
			float Throttle = 0, Steering = 0, Brake = 1, Handbrake = 0, Clutch = 1;
			float Speed = 0, EngineRPM = 0, Load = 0;
			int GearIndex = 1, Differential = 0, GroundedWheels = 0;
			float Displacement = 0, PlasticStrain = 0;
			unsigned int SceneGeneration = 0;
		};
		struct RobotPanelState
		{
			RobotDynamicsBackend Backend = RobotDynamicsBackend::SparseJointSpaceDynamics;
			RobotWalkMode WalkMode = RobotWalkMode::Policy;
			bool Interactive = false, AutoWalk = false;
			Vector3 Command = Vector3::Zero();
			float Heading = 0;
			unsigned int SceneGeneration = 0;
			bool Active = false;
		};
		RobotPanelState m_ImguiRobotPanel, m_PendingRobotPanel;
		VehiclePanelState m_ImguiVehiclePanel;
		VehiclePanelState m_PendingVehiclePanel;
		bool m_PendingVehicleControls = false;
		bool m_VehicleSlowMotion = true;
		float m_VehiclePendingTime = 0;
		int m_PendingVehicleCommand = 0;
		unsigned int m_PendingVehicleGeneration = 0;
		void ApplyVehicleCommands();
		struct SnowPanelState
		{
			bool Active = false, TimeLimited = false;
			bool Available[5]{};
			int Quality = 0, Backend = 0, Transfer = 2;
			float Fraction = 1, Viscosity = 2, SolveMilliseconds = 0, SurfaceMilliseconds = 0;
			size_t ParticleCount = 0;
			unsigned int SceneGeneration = 0;
			std::string Status;
		};
		void DrawSnowPanel(SnowPanelState State);
		std::unique_ptr<SnowSimulation> m_SnowSimulation;
		std::unique_ptr<SandSimulation> m_SandSimulation;
		SnowQuality m_SnowQuality = SnowQuality::Realtime;
		bool m_SnowPaused = false, m_SnowMeshReady = false;
		float m_SnowRenderedFraction = -2, m_SnowSolveMilliseconds = 0, m_SnowSurfaceMilliseconds = 0;
		std::string m_SnowStatus;
		SnowPanelState m_ImguiSnowPanel, m_ImguiPendingSnowPanel;
		bool m_ImguiPendingSnowChange = false;
		struct AerodynamicsPanelState
		{
			bool Active = false;
			int Backend = 2;
			float GridScale = 1.0f;
			int ActualBackend = 2;
			Vector3i Resolution = Vector3i(0, 0, 0);
			unsigned int SceneGeneration = 0;
			std::string Status;
		};
		AerodynamicsPanelState m_ImguiAerodynamicsPanel, m_ImguiPendingAerodynamicsPanel;
		bool m_ImguiPendingAerodynamicsApply = false;
		bool m_AerodynamicsPaused = false;
		bool m_ViewerPaused = false;
		bool m_ImguiViewerPaused = false;
		bool m_ImguiPendingPauseToggle = false;
		unsigned int m_ImguiPendingPauseGeneration = 0;
		SolverPanelState m_ImguiSolverPanel;
		SolverPanelState m_ImguiPendingSolverPanel;
		bool m_ImguiPendingSolverSwitch = false;
		std::string m_ImguiSceneName;
		bool m_RagdollDemoReduced = false;
		bool m_ImguiRagdollDemoReduced = false;
		std::vector<std::string> m_ImguiSceneFiles;
		std::vector<SceneConfiguration> m_ImguiSceneConfigurations;
		std::string m_ImguiActiveSceneConfiguration;
		std::string m_ImguiPendingSceneConfiguration;
		unsigned int m_ImguiPendingSceneConfigurationGeneration = 0;
		std::string m_ImguiCuttingObjPath;
		std::string m_ImguiCuttingStatus;
		std::vector<ConstraintControlState> m_ImguiConstraintControls;
		std::vector<std::string> m_ImguiTerrainObjectIds;
		TerrainBrushState m_ImguiTerrainBrush;
		std::string m_ImguiTerrainStatus;
		std::string m_ImguiMeshSimplificationPath;
		std::string m_ImguiMeshSimplificationStatus;
		int m_ImguiDemoIndex;
		bool m_ImguiCuttingPanelActive;
		bool m_ImguiMeshSimplificationPanelActive;
		int m_ImguiCuttingMode;
		int m_ImguiCuttingPieceCount;
		int m_ImguiCuttingPiecesX;
		int m_ImguiCuttingPiecesY;
		int m_ImguiCuttingPiecesZ;
		int m_ImguiCuttingSeed;
		bool m_ImguiCapCutFaces = true;
		int m_ImguiPendingCapCutFaces = -1;
		int m_ImguiMeshSimplificationAlgorithm;
		int m_ImguiMeshSimplificationRatioLevel;
		float m_ImguiCuttingSeparation;
		bool m_ImguiShowGeometryQueryBounds;
		RenderMode m_ImguiRenderMode;
		size_t m_ImguiGeometryQueryBoundsCount;
		double m_ImguiPhysicsFps;
		int m_ImguiScroll;
		unsigned int m_ImguiRenderedSceneGeneration = 0;
		int m_ImguiPendingDemoIndex;
		bool m_ImguiPendingRestartScene = false;
		unsigned int m_ImguiPendingRestartGeneration = 0;
		int m_ImguiPendingRagdollAlgorithm = -1;
		bool m_ImguiPendingCuttingSeparation;
		float m_ImguiPendingCuttingSeparationValue;
		bool m_ImguiPendingBrowseCuttingObj;
		bool m_ImguiPendingApplyCutting;
		bool m_ImguiPendingCuttingMode;
		int m_ImguiPendingCuttingModeValue;
		bool m_ImguiPendingCuttingPieceCount;
		int m_ImguiPendingCuttingPieceCountValue;
		bool m_ImguiPendingCuttingPiecesX;
		int m_ImguiPendingCuttingPiecesXValue;
		bool m_ImguiPendingCuttingPiecesY;
		int m_ImguiPendingCuttingPiecesYValue;
		bool m_ImguiPendingCuttingPiecesZ;
		int m_ImguiPendingCuttingPiecesZValue;
		bool m_ImguiPendingCuttingSeed;
		int m_ImguiPendingCuttingSeedValue;
		bool m_ImguiPendingBrowseMeshSimplificationFile;
		bool m_ImguiPendingApplyMeshSimplification;
		bool m_ImguiPendingMeshSimplificationAlgorithm;
		int m_ImguiPendingMeshSimplificationAlgorithmValue;
		bool m_ImguiPendingMeshSimplificationRatioLevel;
		int m_ImguiPendingMeshSimplificationRatioLevelValue;
		bool m_ImguiPendingShowGeometryQueryBounds;
		bool m_ImguiPendingShowGeometryQueryBoundsValue;
		bool m_ImguiPendingRenderMode;
		RenderMode m_ImguiPendingRenderModeValue;
		bool m_ImguiPendingShootProjectile;
		std::vector<ConstraintControlState> m_ImguiPendingConstraintControls;
		bool m_TerrainDragActive = false;
		Vector3 m_TerrainDragLastPoint = Vector3::Zero();
		float m_TerrainDragTargetHeight = 0.0f;

		std::chrono::steady_clock::time_point m_PhysicsFpsTime;
		int m_PhysicsFrameCount;
		double m_PhysicsFps;
	};
}
