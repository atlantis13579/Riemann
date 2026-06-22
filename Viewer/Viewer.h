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

namespace Riemann
{
	class Geometry;
	class RenderThread;
	class SceneWorld;

	class WorldViewer
	{
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

		void RebuildRenderScene();
		void AddGeometryToRender(const std::string& id, Geometry* geometry, const Vector4& color, bool renderBounds);
		bool LoadCuttingPanel();
		bool ApplyCuttingPanel();
		void RefreshObjList();
		void SubmitCuttingTransforms();
		void SubmitTransforms();
		void SubmitGeometryQueryBounds();
		void ClearGeometryQueryBounds();
		void SetShowGeometryQueryBounds(bool show);
		std::string OpenObjFileDialog() const;
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
		static void DrawImguiCallback(int width, int height, void* userData);

	private:
		std::unique_ptr<SceneWorld> m_World;
		RenderThread* m_RenderThread;
		std::vector<RenderBinding> m_RenderBindings;
		std::function<void(char)> m_KeyboardEvent;
		Geometry* m_HighlightedGeometry;

		std::string m_SceneDirectory;
		std::vector<std::string> m_SceneFiles;
		std::string m_CurrentSceneName;
		Vector3 m_CamCenter;
		Vector3 m_CamParam;
		bool m_MovementKeys[4];
		bool m_CuttingPanelActive;
		std::vector<CuttingRenderPiece> m_CuttingPieces;
		std::vector<std::string> m_CuttingObjFiles;
		std::string m_CuttingObjPath;
		std::string m_CuttingStatus;
		std::vector<std::string> m_GeometryQueryBoundsMeshIds;
		int m_CuttingObjIndex;
		int m_CuttingMode;
		int m_CuttingPieceCount;
		int m_CuttingPiecesX;
		int m_CuttingPiecesY;
		int m_CuttingPiecesZ;
		int m_CuttingSeed;
		float m_CuttingSeparation;
		float m_CuttingMaxSeparation;
		bool m_ShowGeometryQueryBounds;

		mutable std::mutex m_ImguiMutex;
		std::string m_ImguiSceneName;
		std::vector<std::string> m_ImguiSceneFiles;
		std::vector<std::string> m_ImguiCuttingObjFiles;
		std::string m_ImguiCuttingObjPath;
		std::string m_ImguiCuttingStatus;
		int m_ImguiDemoIndex;
		size_t m_ImguiObjectCount;
		float m_ImguiCameraDistance;
		bool m_ImguiCuttingPanelActive;
		int m_ImguiCuttingObjIndex;
		int m_ImguiCuttingMode;
		int m_ImguiCuttingPieceCount;
		int m_ImguiCuttingPiecesX;
		int m_ImguiCuttingPiecesY;
		int m_ImguiCuttingPiecesZ;
		int m_ImguiCuttingSeed;
		float m_ImguiCuttingSeparation;
		bool m_ImguiShowGeometryQueryBounds;
		size_t m_ImguiGeometryQueryBoundsCount;
		double m_ImguiPhysicsFps;
		int m_ImguiScroll;
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
		bool m_ImguiPendingShowGeometryQueryBounds;
		bool m_ImguiPendingShowGeometryQueryBoundsValue;

		std::chrono::steady_clock::time_point m_PhysicsFpsTime;
		int m_PhysicsFrameCount;
		double m_PhysicsFps;
	};
}
