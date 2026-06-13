#pragma once

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "../Src/Maths/Vector3.h"
#include "../Src/Maths/Vector4.h"

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

		void CreateDemo();
		void CreateStackBoxesDemo();
		void CreateDominoDemo();
		void CreateSeeSawDemo();

		Vector3 GetCameraPosition() const;
		void UpdateCamera();
		void KeyboardMsg(char c);
		void MouseMsg(int x, int y, bool leftButtonDown);
		void MouseWheel(int zDelta, bool ctrlButtonDown);

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
		void ApplySceneCamera();

	private:
		std::unique_ptr<SceneWorld> m_World;
		RenderThread* m_RenderThread;
		std::vector<RenderBinding> m_RenderBindings;
		std::function<void(char)> m_KeyboardEvent;

		std::string m_SceneDirectory;
		Vector3 m_CamCenter;
		Vector3 m_CamParam;
	};
}
