#pragma once

#include <string>
#include <vector>
#include "../Src/Maths/Vector3.h"

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
	void LoadFlatObj(const std::string& file_name);
	void LoadVoxelField(const std::string& file_name, const Vector3d& c, std::vector<Vector3d>& water_list);

	void CreateBoxesDemo();

	Vector3d GetCameraPosition();
	void UpdateCamera();
	void KeyboardMsg(char c);
	void MouseMsg(int x, int y, bool LButtonDown);
	void MouseWheel(int zDelta, bool CtrlButtonDown);

private:
	RigidBodySimulation* m_World;
	Renderer* m_Renderer;

	Vector3d    m_CamCenter;
	Vector3d    m_CamParam;
};


