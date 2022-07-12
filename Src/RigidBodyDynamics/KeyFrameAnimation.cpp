
#include "KeyFrameAnimation.h"

KeyFrameAnimation::KeyFrameAnimation()
{
	m_TimelinePosition.SetTime(0.0f);
	m_TimelineRotation.SetTime(0.0f);
	m_IsLoop = false;
}

void KeyFrameAnimation::LoadKeyframes(const std::vector<KeyframePos>& frame_pos, const std::vector<KeyframeQuat>& frame_quat, bool is_loop)
{
	m_FramesPosition = frame_pos;
	m_FramesRotation = frame_quat;
	m_IsLoop = is_loop;
	SetTime(0.0f);
}

void KeyFrameAnimation::LoadRotationY(const Quaternion& quat, float radian, int time_ms)
{
	Quaternion rot(radian, Vector3d(0.0f, 1.0f, 0.0f));

	m_FramesPosition.clear();
	m_FramesRotation.resize(2);
	m_FramesRotation[0].time = 0;
	m_FramesRotation[0].key = quat;
	m_FramesRotation[1].time = time_ms;
	m_FramesRotation[1].key = quat * rot;
	m_IsLoop = false;
	SetTime(0.0f);
}

template<typename T>
bool IsFramesDataValid(const std::vector<Keyframe<T>>& frames)
{
	if (frames.empty())
	{
		return false;
	}

	if (frames[0].time != 0)
	{
		return false;
	}

	if (frames.size() > 1)
	{
		for (size_t i = 0; i < frames.size() - 1; ++i)
		{
			if (frames[i + 1].time == frames[i].time)
			{
				return false;
			}
		}
	}

	return true;
}

bool KeyFrameAnimation::CheckAnimData() const
{
	bool valid_pos = IsFramesDataValid(m_FramesPosition);
	bool valid_quat = IsFramesDataValid(m_FramesRotation);
	return valid_pos || valid_quat;
}

template<typename T>
T Lerp(const T& start, const T& end, float t)
{
}

template<>
Vector3d Lerp(const Vector3d& start, const Vector3d& end, float t)
{
	return Vector3d::Lerp(start, end, t);
}

template<>
Quaternion Lerp(const Quaternion& start, const Quaternion& end, float t)
{
	return Quaternion::Slerp(start, end, t);
}

void KeyFrameAnimation::SetTime(float time)
{
	m_TimelinePosition.SetTime(time);
	m_TimelineRotation.SetTime(time);
}

bool KeyFrameAnimation::IsFinish() const
{
	return m_TimelinePosition.IsFinish && m_TimelineRotation.IsFinish;
}

template <typename T>
static bool InterpFrame(float elapsed, const std::vector<Keyframe<T>>& frames, bool is_loop, Timeline<T>& timeline, T* result)
{
	float elapsed_ms = elapsed * 1000.0f;
	if (elapsed_ms <= 0.0f || frames.empty() || timeline.IsFinish)
	{
		return false;
	}

	if (frames.size() == 1)
	{
		*result = frames[0].key;
		if (!is_loop)
		{
			timeline.IsFinish = true;
		}
		return true;
	}

	timeline.TimeMS += elapsed_ms;
	if (timeline.TimeMS >= frames.back().time)
	{
		timeline.Idx = 0;
		if (!is_loop)
		{
			timeline.TimeMS = true;
			*result = frames.back().key;
			return true;
		}
		timeline.TimeMS = fmodf(timeline.TimeMS, (float)frames.back().time);
	}

	for (size_t i = timeline.Idx; i < frames.size() - 1; ++i)
	{
		if (frames[i].time <= timeline.TimeMS && timeline.TimeMS < frames[i + 1].time)
		{
			timeline.Idx = (int)i;
			float dt = 1.0f * (timeline.TimeMS - frames[i].time) / (frames[i + 1].time - frames[i].time);
			if (dt < 1e-6f)
			{
				*result = frames[i].key;
			}
			else
			{
				*result = Lerp(frames[i].key, frames[i + 1].key, dt);
			}
			return true;
		}
	}

	return false;
}

bool KeyFrameAnimation::Advance(float elapsed, Vector3d* pos, Quaternion* quat)
{
	bool success_pos = InterpFrame(elapsed, m_FramesPosition, m_IsLoop, m_TimelinePosition, pos);
	bool success_quat = InterpFrame(elapsed, m_FramesRotation, m_IsLoop, m_TimelineRotation, quat);
	return success_pos || success_quat;
}