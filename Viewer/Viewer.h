#pragma once

#include <chrono>
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

namespace Riemann
{
	class Geometry;
	class RenderThread;
	class FluidSimulation;
	class FluidRigidCoupling;
	class AerodynamicsSimulation;
	class SmokeSimulation;
	class SmokeRigidCoupling;
	class AerodynamicsRigidCoupling;

	class WorldViewer
	{
		friend struct SmokeViewerValidationAccess;
	public:
		WorldViewer(RenderThread* renderThread, const std::string& sceneFile);
		~WorldViewer();

		bool LoadScene(const std::string& sceneFile);
		void CreateSimulator();
		void UpdateSimulator(float dt);
		void LoadAnimation(const std::string& animName, const std::vector<std::string>& nodes);
		void LoadPhysxScene(const std::string& fileName);
		void LoadVoxelField(const std::string& fileName, const Vector3& c, std::vector<Vector3>& waterList);

		Vector3 GetCameraPosition() const;
		void UpdateCamera();
		void KeyboardMsg(char c);
		void SetMovementKey(char c, bool down);
		void UpdateCameraMovement(float dt);
		void MouseMsg(int x, int y, bool rotateButtonDown);
		void SceneRayMsg(int x, int y, int width, int height);
		void SceneDragMsg(int x, int y, int width, int height);
		void SceneDragEndMsg();
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
			std::string MeshId;
			Vector3 Center = Vector3::Zero();
			Vector3 Direction = Vector3::Zero();
		};

		struct TerrainBrushState
		{
			std::string ObjectId;
			Vector3 Center = Vector3::Zero();
			Vector3 ControlCenter = Vector3::Zero();
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
		bool SynchronizeSmokeObstacles();
		void SubmitGasDensity();
		bool LoadCuttingPanel();
		bool ApplyCuttingPanel();
		bool LoadMeshSimplificationPanel();
		bool ApplyMeshSimplificationPanel();
		void RefreshObjList();
		void RefreshMeshFileList();
		void SubmitCuttingTransforms();
		void SubmitTransforms();
		void SubmitGeometryQueryBounds();
		void ClearGeometryQueryBounds();
		void SetShowGeometryQueryBounds(bool show);
		void SetRenderMode(RenderMode mode);
		float GetCameraSceneScale() const;
		float GetCameraControlScale() const;
		std::string OpenObjFileDialog() const;
		std::string OpenMeshFileDialog() const;
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
		bool IsCurrentCuttingGeometry(const Geometry* geometry) const;
		void SetHighlightedGeometry(Geometry* geometry);
		void ShootProjectile();
		static void DrawImguiCallback(int width, int height, void* userData);

	private:
		std::unique_ptr<SceneWorld> m_World;
		std::unique_ptr<FluidSimulation> m_FluidSimulation;
		std::unique_ptr<AerodynamicsSimulation> m_AerodynamicsSimulation;
		std::unique_ptr<SmokeSimulation> m_SmokeSimulation;
		SmokeRigidCoupling* m_SmokeCoupling = nullptr;
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
		bool m_MovementKeys[4];
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
		int m_CuttingObjIndex;
		int m_CuttingMode;
		int m_CuttingPieceCount;
		int m_CuttingPiecesX;
		int m_CuttingPiecesY;
		int m_CuttingPiecesZ;
		int m_CuttingSeed;
		int m_MeshSimplificationFileIndex;
		int m_MeshSimplificationAlgorithm;
		int m_MeshSimplificationRatioLevel;
		float m_CuttingSeparation;
		float m_CuttingMaxSeparation;
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
		struct SmokePanelState
		{
			bool Active = false;
			bool Paused = false;
			bool RigidCoupling = false;
			int RigidBodyCount = 0;
			int Backend = 2;
			int Quality = 1;
			int ActualBackend = 2;
			Vector3i Resolution = Vector3i(0, 0, 0);
			unsigned int SceneGeneration = 0;
			std::string Status;
		};
		SmokePanelState m_ImguiSmokePanel, m_ImguiPendingSmokePanel;
		bool m_ImguiPendingSmokeRestart = false;
		bool m_ImguiPendingSmokePause = false;
		int m_ImguiPendingSmokeAddBodies = 0;
		unsigned int m_ImguiPendingSmokeAddGeneration = 0;
		bool m_SmokePaused = false;
		SolverPanelState m_ImguiSolverPanel;
		SolverPanelState m_ImguiPendingSolverPanel;
		bool m_ImguiPendingSolverSwitch = false;
		std::string m_ImguiSceneName;
		std::vector<std::string> m_ImguiSceneFiles;
		std::vector<std::string> m_ImguiCuttingObjFiles;
		std::string m_ImguiCuttingObjPath;
		std::string m_ImguiCuttingStatus;
		std::vector<std::string> m_ImguiMeshSimplificationFiles;
		std::vector<ConstraintControlState> m_ImguiConstraintControls;
		std::vector<std::string> m_ImguiTerrainObjectIds;
		TerrainBrushState m_ImguiTerrainBrush;
		std::string m_ImguiTerrainStatus;
		std::string m_ImguiMeshSimplificationPath;
		std::string m_ImguiMeshSimplificationStatus;
		int m_ImguiDemoIndex;
		size_t m_ImguiObjectCount;
		float m_ImguiCameraDistance;
		bool m_ImguiCuttingPanelActive;
		bool m_ImguiMeshSimplificationPanelActive;
		int m_ImguiCuttingObjIndex;
		int m_ImguiCuttingMode;
		int m_ImguiCuttingPieceCount;
		int m_ImguiCuttingPiecesX;
		int m_ImguiCuttingPiecesY;
		int m_ImguiCuttingPiecesZ;
		int m_ImguiCuttingSeed;
		int m_ImguiMeshSimplificationFileIndex;
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
		std::string m_ImguiPendingSceneFile;
		bool m_ImguiPendingCameraDistance;
		float m_ImguiPendingCameraDistanceValue;
		bool m_ImguiPendingCuttingSeparation;
		float m_ImguiPendingCuttingSeparationValue;
		int m_ImguiPendingCuttingObjIndex;
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
		int m_ImguiPendingMeshSimplificationFileIndex;
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
		std::vector<TerrainBrushState> m_ImguiPendingTerrainEdits;

		std::chrono::steady_clock::time_point m_PhysicsFpsTime;
		int m_PhysicsFrameCount;
		double m_PhysicsFps;
	};
}
