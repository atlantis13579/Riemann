#pragma once

#include <vector>

#include "../Maths/Vector3d.h"
#include "../Maths/Quaternion.h"

template<typename T>
struct Keyframe
{
	Keyframe() {}
	Keyframe(int _time, T _key)
	{
		time = _time;
		key = _key;
	}
	int time;
	T	key;
};

typedef Keyframe<Vector3d> KeyframePos;
typedef Keyframe<Quaternion> KeyframeQuat;

class Animation
{
public:
	Animation();

	void LoadKeyframes(const std::vector<KeyframePos>& frame_pos, const std::vector<KeyframeQuat>& frame_quat, bool is_loop);
	void LoadRotationY(const Quaternion& quat, float radian, int time_ms);

	bool CheckAnimData() const;
	void Begin();
	bool IsFinish() const;
	bool Advance(float elapsed, Vector3d* pos, Quaternion* quat);
	bool AdvancePos(float elapsed, Vector3d* pos);
	bool AdvanceQuat(float elapsed, Quaternion* quat);

private:
	float						m_CurrentPos, m_CurrentQuat;
	bool						m_IsFinishPos, m_IsFinishQuat;

	std::vector<KeyframePos>	m_FramesPosition;
	std::vector<KeyframeQuat>	m_FramesRotation;
	bool						m_IsLoop;
};
