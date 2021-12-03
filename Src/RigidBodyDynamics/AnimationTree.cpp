
#include "AnimationTree.h"
#include "../Tools/AnimBinaryParser.h"
#include "RigidBody.h"

struct _Node
{
	_Node() { Idx = -1; }
	_Node(int _Idx) { Idx = _Idx; }
	int	Idx;
	std::vector<_Node*> Childrens;
};

void _Flaten(std::vector<AnimTreeNode>& FlatTree, int Parent, const _Node* Tree, const AnimTreeData* data)
{
	std::vector<KeyframePos> frames_pos;
	std::vector<KeyframeQuat> frames_quat;

	FlatTree.push_back(AnimTreeNode());
	AnimTreeNode* node = &FlatTree.back();

	if (Tree->Idx >= 0)
	{
		const BoneChannel& channel = data->channels[Tree->Idx];
		bool is_loop = true;

		frames_pos.clear();
		for (size_t j = 0; j < channel.positionKeys.size(); ++j)
		{
			frames_pos.emplace_back(channel.positionKeys[j].first, channel.positionKeys[j].second);
		}

		frames_quat.clear();
		for (size_t j = 0; j < channel.rotationKeys.size(); ++j)
		{
			frames_quat.emplace_back(channel.rotationKeys[j].first, channel.rotationKeys[j].second);
		}

		node->Name = channel.boneName;
		node->Anim.LoadKeyframes(frames_pos, frames_quat, is_loop);
	}

	node->Parent = Parent;

	int newParent = (int)FlatTree.size() - 1;
	for (size_t j = 0; j < Tree->Childrens.size(); ++j)
	{
		_Flaten(FlatTree, newParent, Tree->Childrens[j], data);
	}
};


bool AnimationTree::BuildFlatTree(AnimTreeData* data)
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
		const BoneChannel& c = data->channels[i];
		if (c.parentPos < 0)
		{
			Root.Childrens.push_back(&Nodes[i]);
		}
		else if (c.parentPos == i || c.parentPos >= nBones)
		{
			return false;
		}
		else
		{
			_Node& parent = Nodes[c.parentPos];
			parent.Childrens.push_back(&Nodes[i]);
		}
	}

	_Flaten(m_FlatTree, -1, &Root, data);

	if (nBones + 1 != (int)m_FlatTree.size())
	{
		m_FlatTree.clear();
		return false;
	}

	for (size_t i = 1; i < m_FlatTree.size(); ++i)
	{
		if (!m_FlatTree[i].Anim.CheckAnimData())
		{
			m_FlatTree.clear();
			return false;
		}
	}

	return true;
}

AnimationTree::AnimationTree()
{
	m_Pause = true;
	m_PlayRate = 1.0f;
}

void AnimationTree::Simulate(float elapsed)
{
	if (m_Pause || m_FlatTree.empty())
	{
		return;
	}

	float elapsed_ms = elapsed * m_PlayRate;
	for (size_t i = 1; i < m_FlatTree.size(); ++i)
	{
		AnimTreeNode* node = &m_FlatTree[i];
		AnimTreeNode* parent = node->Parent < 0 ? nullptr : &m_FlatTree[node->Parent];

		Vector3d pos = Vector3d::Zero();
		Quaternion quat = Quaternion::One();
		node->Anim.Advance(elapsed_ms, &pos, &quat);

		node->X = parent->X + parent->Q * pos;
		node->Q = parent->Q * quat;

		if (node->Entity)
		{
			node->Entity->SetTransform(node->X, node->Q);
		}
	}
}

void AnimationTree::Pause(bool pause)
{
	m_Pause = pause;
}

bool AnimationTree::IsPause() const
{
	return m_Pause;
}

void AnimationTree::SetAnimationPlayRate(float play_rate)
{
	m_PlayRate = play_rate;
}

bool AnimationTree::Bind(const std::string& node_name, RigidBodyStatic* body)
{
	for (size_t i = 0; i < m_FlatTree.size(); ++i)
	{
		if (m_FlatTree[i].Name == node_name)
		{
			m_FlatTree[i].Entity = body;
			m_FlatTree[i].X = body->X;
			m_FlatTree[i].Q = body->Q;
			return true;
		}
	}

	return false;
}

void AnimationTree::UnBind(RigidBodyStatic* body)
{
	for (size_t i = 0; i < m_FlatTree.size(); ++i)
	{
		if (m_FlatTree[i].Entity == body)
		{
			m_FlatTree[i].Entity = nullptr;
		}
	}
}

bool AnimationTree::Deserialize(const std::string& filepath)
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

void AnimationTree::SetRootTransform(const Vector3d& pos, const Quaternion& rot)
{
	if (m_FlatTree.size() > 0)
	{
		m_FlatTree[0].X = pos;
		m_FlatTree[0].Q = rot;
	}
}
