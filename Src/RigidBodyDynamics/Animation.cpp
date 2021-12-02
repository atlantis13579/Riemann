
#include "Animation.h"

Animation::Animation()
{
	m_CurrentPos = m_CurrentQuat = 0.0f;
	m_IsFinishPos = m_IsFinishQuat = true;
	m_IsLoop = false;
}

void Animation::LoadKeyframes(const std::vector<KeyframePos>& frame_pos, const std::vector<KeyframeQuat>& frame_quat, bool is_loop)
{
	m_FramesPosition = frame_pos;
	m_FramesRotation = frame_quat;
	m_IsLoop = is_loop;
	Begin();
}

void Animation::LoadRotationY(const Quaternion& quat, float radian, int time_ms)
{
	Quaternion rot(radian, Vector3d(0.0f, 1.0f, 0.0f));

	m_FramesPosition.clear();
	m_FramesRotation.resize(2);
	m_FramesRotation[0].time = 0;
	m_FramesRotation[0].key = quat;
	m_FramesRotation[1].time = time_ms;
	m_FramesRotation[1].key = quat * rot;
	m_IsLoop = false;
	Begin();
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

bool Animation::CheckAnimData() const
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

void Animation::Begin()
{
	m_CurrentPos = m_CurrentQuat = 0.0f;
	m_IsFinishPos = m_IsFinishQuat = false;
}

bool Animation::IsFinish() const
{
	return m_IsFinishPos && m_IsFinishQuat;
}

template <typename T>
static bool InterpFrame(float elapsed, const std::vector<Keyframe<T>>& frames, bool is_loop, float& current_ms, bool& is_finish, T* result)
{
	float elapsed_ms = elapsed * 1000.0f;

	if (elapsed_ms <= 0.0f || frames.empty() || is_finish)
	{
		return false;
	}

	if (frames.size() == 1)
	{
		*result = frames[0].key;
		if (!is_loop)
		{
			is_finish = true;
		}
		return true;
	}

	current_ms += elapsed_ms;
	if (current_ms >= frames.back().time)
	{
		if (!is_loop)
		{
			is_finish = true;
			*result = frames.back().key;
			return true;
		}
		current_ms = fmodf(current_ms, (float)frames.back().time);
	}

	for (size_t i = 0; i < frames.size() - 1; ++i)
	{
		if (frames[i].time <= current_ms && current_ms < frames[i + 1].time)
		{
			float dt = 1.0f * (current_ms - frames[i].time) / (frames[i + 1].time - frames[i].time);
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

bool Animation::Advance(float elapsed, Vector3d* pos, Quaternion* quat)
{
	bool success_pos = InterpFrame(elapsed, m_FramesPosition, m_IsLoop, m_CurrentPos, m_IsFinishPos, pos);
	bool success_quat = InterpFrame(elapsed, m_FramesRotation, m_IsLoop, m_CurrentQuat, m_IsFinishQuat, quat);
	return success_pos || success_quat;
}

bool Animation::AdvancePos(float elapsed, Vector3d* pos)
{
	return InterpFrame(elapsed, m_FramesPosition, m_IsLoop, m_CurrentPos, m_IsFinishPos, pos);
}

bool Animation::AdvanceQuat(float elapsed, Quaternion* quat)
{
	return InterpFrame(elapsed, m_FramesRotation, m_IsLoop, m_CurrentQuat, m_IsFinishQuat, quat);
}
