#pragma once

#include <functional>
#include <string>
#include <vector>
#include "../Src/Maths/Vector3.h"

namespace Riemann
{
	class RigidBodySimulation;
	class Renderer;

	class WorldViewer
	{
	public:
		WorldViewer(Renderer* renderer);

		~WorldViewer();

		void CreateSimulator();
		void UpdateSimulator(float dt);
		void LoadAnimation(const std::string& anim_name, const std::vector<std::string>& nodes);
		void LoadPhysxScene(const std::string& file_name);
		void LoadVoxelField(const std::string& file_name, const Vector3& c, std::vector<Vector3>& water_list);

		void CreateDemo();
		void CreateStackBoxesDemo();
		void CreateDominoDemo();
		void CreateSeeSawDemo();

		Vector3 GetCameraPosition();
		void UpdateCamera();
		void KeyboardMsg(char c);
		void MouseMsg(int x, int y, bool LButtonDown);
		void MouseWheel(int zDelta, bool CtrlButtonDown);

		void AddToRender();

	private:
		RigidBodySimulation* m_World;
		Renderer* m_Renderer;
		std::function<void(char)> m_KeyboardEvent;

		Vector3    m_CamCenter;
		Vector3    m_CamParam;
	};
}