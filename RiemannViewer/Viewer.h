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

		void RebuildRenderScene();
		void AddGeometryToRender(const std::string& id, Geometry* geometry, const Vector4& color, bool renderBounds);
		void SubmitTransforms();
		std::string ResolveSceneFile(const char* fileName) const;
		void RefreshSceneList();
		void ApplySceneCamera();
		void ApplyImguiCommands();
		void UpdateImguiState();
		void UpdatePhysicsFps();
		Ray3 BuildSceneRay(int x, int y, int width, int height) const;
		void HandleSceneRay(const Ray3& ray);
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

		mutable std::mutex m_ImguiMutex;
		std::string m_ImguiSceneName;
		std::vector<std::string> m_ImguiSceneFiles;
		size_t m_ImguiObjectCount;
		float m_ImguiCameraDistance;
		double m_ImguiPhysicsFps;
		int m_ImguiScroll;
		std::string m_ImguiPendingSceneFile;
		bool m_ImguiPendingCameraDistance;
		float m_ImguiPendingCameraDistanceValue;

		std::chrono::steady_clock::time_point m_PhysicsFpsTime;
		int m_PhysicsFrameCount;
		double m_PhysicsFps;
	};
}
