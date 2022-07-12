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

template<typename T>
struct Timeline
{
	void SetTime(float start)
	{
		TimeMS = start;
		Idx = 0;
		IsFinish = false;
	}

	float	TimeMS;
	int		Idx;
	bool	IsFinish;
};

class KeyFrameAnimation
{
public:
	KeyFrameAnimation();

	void LoadKeyframes(const std::vector<KeyframePos>& frame_pos, const std::vector<KeyframeQuat>& frame_quat, bool is_loop);
	void LoadRotationY(const Quaternion& quat, float radian, int time_ms);

	bool CheckAnimData() const;
	void SetTime(float time);
	bool IsFinish() const;
	bool Advance(float elapsed, Vector3d* pos, Quaternion* quat);

private:

	Timeline<Vector3d>			m_TimelinePosition;
	Timeline<Quaternion>		m_TimelineRotation;
	std::vector<KeyframePos>	m_FramesPosition;
	std::vector<KeyframeQuat>	m_FramesRotation;
	bool						m_IsLoop;
};
