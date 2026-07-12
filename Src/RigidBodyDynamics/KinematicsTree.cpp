
#include "KinematicsTree.h"
#include "RigidBody.h"

#include <cmath>

namespace Riemann
{
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
		T key;
	};

	typedef Keyframe<Vector3> KeyframePos;
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

		float TimeMS;
		int Idx;
		bool IsFinish;
	};

	class KeyFrameAnimation
	{
	public:
		KeyFrameAnimation()
		{
			m_TimelinePosition.SetTime(0.0f);
			m_TimelineRotation.SetTime(0.0f);
			m_IsLoop = false;
		}

		void LoadKeyframes(const std::vector<KeyframePos>& frame_pos, const std::vector<KeyframeQuat>& frame_quat, bool is_loop)
		{
			m_FramesPosition = frame_pos;
			m_FramesRotation = frame_quat;
			m_IsLoop = is_loop;
			SetTime(0.0f);
		}

		void LoadRotationY(const Quaternion& quat, float radian, int time_ms)
		{
			Quaternion rot(radian, Vector3(0.0f, 1.0f, 0.0f));

			m_FramesPosition.clear();
			m_FramesRotation.resize(2);
			m_FramesRotation[0].time = 0;
			m_FramesRotation[0].key = quat;
			m_FramesRotation[1].time = time_ms;
			m_FramesRotation[1].key = quat * rot;
			m_IsLoop = false;
			SetTime(0.0f);
		}

		bool CheckAnimData() const
		{
			bool valid_pos = IsFramesDataValid(m_FramesPosition);
			bool valid_quat = IsFramesDataValid(m_FramesRotation);
			return valid_pos || valid_quat;
		}

		void SetTime(float time)
		{
			m_TimelinePosition.SetTime(time);
			m_TimelineRotation.SetTime(time);
		}

		bool IsFinish() const
		{
			return m_TimelinePosition.IsFinish && m_TimelineRotation.IsFinish;
		}

		bool Advance(float elapsed, Vector3* pos, Quaternion* quat)
		{
			bool success_pos = InterpFrame(elapsed, m_FramesPosition, m_IsLoop, m_TimelinePosition, pos);
			bool success_quat = InterpFrame(elapsed, m_FramesRotation, m_IsLoop, m_TimelineRotation, quat);
			return success_pos || success_quat;
		}

	private:
		template<typename T>
		static bool IsFramesDataValid(const std::vector<Keyframe<T>>& frames)
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

		template<typename T>
		static T Lerp(const T& start, const T& end, float t);

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

	private:
		Timeline<Vector3> m_TimelinePosition;
		Timeline<Quaternion> m_TimelineRotation;
		std::vector<KeyframePos> m_FramesPosition;
		std::vector<KeyframeQuat> m_FramesRotation;
		bool m_IsLoop;
	};

	template<>
	Vector3 KeyFrameAnimation::Lerp(const Vector3& start, const Vector3& end, float t)
	{
		return Vector3::Lerp(start, end, t);
	}

	template<>
	Quaternion KeyFrameAnimation::Lerp(const Quaternion& start, const Quaternion& end, float t)
	{
		return Quaternion::Slerp(start, end, t);
	}

	class KeyframeKinematics : public RigidBodyKinematics
	{
	public:
		KeyframeKinematics()
		{
			Parent = -1;
		}

		KeyFrameAnimation			Anim;
		std::string					Name;
		int							Parent;
	};

	KinematicsTree::KinematicsTree()
	{
		m_Pause = true;
		m_PlayRate = 1.0f;
	}

	KinematicsTree::~KinematicsTree()
	{
		for (size_t i = 0; i < m_FlatTree.size(); ++i)
		{
			delete m_FlatTree[i];
		}
	}

	struct _Node
	{
		_Node() { Idx = -1; }
		_Node(int _Idx) { Idx = _Idx; }
		int	Idx;
		std::vector<_Node*> Childrens;
	};

	void _Flatten(std::vector<KeyframeKinematics*>& FlatTree, int Parent, const _Node* Tree, const AnimTreeData* data)
	{
		std::vector<KeyframePos> frames_pos;
		std::vector<KeyframeQuat> frames_quat;

		KeyframeKinematics* node = new KeyframeKinematics();
		FlatTree.push_back(node);

		if (Tree->Idx >= 0)
		{
			const AnimTreeData::Channel& channel = data->channels[Tree->Idx];
			bool is_loop = true;

			frames_pos.clear();
			for (size_t j = 0; j < channel.pos_frames.size(); ++j)
			{
				frames_pos.emplace_back(channel.pos_frames[j].first, channel.pos_frames[j].second);
			}

			frames_quat.clear();
			for (size_t j = 0; j < channel.quat_frames.size(); ++j)
			{
				frames_quat.emplace_back(channel.quat_frames[j].first, channel.quat_frames[j].second);
			}

			node->Name = channel.name;
			node->Anim.LoadKeyframes(frames_pos, frames_quat, is_loop);
		}

		node->Parent = Parent;

		int newParent = (int)FlatTree.size() - 1;
		for (size_t j = 0; j < Tree->Childrens.size(); ++j)
		{
			_Flatten(FlatTree, newParent, Tree->Childrens[j], data);
		}
	};


	bool KinematicsTree::BuildFlatTree(AnimTreeData* data)
	{
		m_FlatTree.clear();
		if (data->channels.empty())
		{
			return false;
		}
		int nBones = (int)data->channels.size();

		_Node Root(-1);
		std::vector<_Node>	Nodes;

		Nodes.resize(nBones);
		for (int i = 0; i < nBones; ++i)
		{
			Nodes[i] = i;
			const AnimTreeData::Channel& c = data->channels[i];
			if (c.parent < 0)
			{
				Root.Childrens.push_back(&Nodes[i]);
			}
			else if (c.parent == i || c.parent >= nBones)
			{
				return false;
			}
			else
			{
				_Node& parent = Nodes[c.parent];
				parent.Childrens.push_back(&Nodes[i]);
			}
		}

		_Flatten(m_FlatTree, -1, &Root, data);

		if (nBones + 1 != (int)m_FlatTree.size())
		{
			m_FlatTree.clear();
			return false;
		}

		for (size_t i = 1; i < m_FlatTree.size(); ++i)
		{
			if (!m_FlatTree[i]->Anim.CheckAnimData())
			{
				m_FlatTree.clear();
				return false;
			}
		}

		return true;
	}

	void KinematicsTree::Simulate(float elapsed)
	{
		if (m_Pause || m_FlatTree.empty())
		{
			return;
		}

		float elapsed_rated = elapsed * m_PlayRate;
		for (size_t i = 1; i < m_FlatTree.size(); ++i)
		{
			KeyframeKinematics* node = m_FlatTree[i];
			KeyframeKinematics* parent = m_FlatTree[node->Parent];

			Vector3 pos = Vector3::Zero();
			Quaternion quat = Quaternion::One();
			node->Anim.Advance(elapsed_rated, &pos, &quat);

			node->X = parent->X + parent->Q * pos;
			node->Q = parent->Q * quat;

			if (node)
			{
				node->SetTransform(node->X, node->Q);
			}
		}
	}

	void KinematicsTree::Pause(bool pause)
	{
		m_Pause = pause;
	}

	bool KinematicsTree::IsPause() const
	{
		return m_Pause;
	}

	void KinematicsTree::SetAnimationPlayRate(float play_rate)
	{
		m_PlayRate = play_rate;
	}

	bool KinematicsTree::BindGeometry(const std::string& node_name, Geometry* geom)
	{
		for (size_t i = 0; i < m_FlatTree.size(); ++i)
		{
			if (m_FlatTree[i]->Name == node_name)
			{
				m_FlatTree[i]->AddGeometry(geom);
				return true;
			}
		}

		return false;
	}

	bool KinematicsTree::Deserialize(const std::string& filepath)
	{
		m_ResPath = filepath;

		AnimTreeData* data = AnimTreeData::Deserialize(filepath.c_str());
		if (data == nullptr)
		{
			return false;
		}

		if (!BuildFlatTree(data))
		{
			delete data;

			m_FlatTree.clear();
			return false;
		}

		delete data;

		return true;
	}

	void KinematicsTree::SetRootTransform(const Vector3& pos, const Quaternion& rot)
	{
		if (m_FlatTree.size() > 0)
		{
			m_FlatTree[0]->X = pos;
			m_FlatTree[0]->Q = rot;
		}
	}
}
