
#include "AnimationTree.h"
#include "../Tools/AnimBinaryParser.h"
#include "RigidBody.h"

static int CalcNumNodes(AnimTreeNode* node)
{
	int num = 1;
	for (size_t i = 0; i < node->Childrens.size(); ++i)
	{
		num += CalcNumNodes(node->Childrens[i]);
	}
	return num;
}

bool AnimationTree::Build(AnimTreeData* data)
{
	m_Roots.clear();
	m_Nodes.clear();

	if (data->channels.empty())
	{
		return false;
	}

	std::vector<KeyframePos> frames_pos;
	std::vector<KeyframeQuat> frames_quat;

	int nBones = (int)data->channels.size();
	m_Nodes.resize(nBones);
	for (int i = 0; i < nBones; ++i)
	{
		AnimTreeNode* node = &m_Nodes[i];
		const BoneChannel& channel = data->channels[i];
		bool is_loop = true;

		frames_pos.clear();
		for (size_t i = 0; i < channel.positionKeys.size(); ++i)
		{
			frames_pos.emplace_back(channel.positionKeys[i].first, channel.positionKeys[i].second);
		}

		frames_quat.clear();
		for (size_t i = 0; i < channel.rotationKeys.size(); ++i)
		{
			frames_quat.emplace_back(channel.rotationKeys[i].first, channel.rotationKeys[i].second);
		}

		node->Name = channel.boneName;
		node->Anim.LoadKeyframes(frames_pos, frames_quat, is_loop);

		if (!node->Anim.CheckAnimData())
		{
			return false;
		}
	}

	for (int i = 0; i < nBones; ++i)
	{
		const BoneChannel& c = data->channels[i];
		if (c.parentPos < 0)
		{
			m_Roots.push_back(&m_Nodes[i]);
		}
		else if (c.parentPos == i || c.parentPos >= nBones)
		{
			return false;
		}
		else
		{
			AnimTreeNode& parent = m_Nodes[c.parentPos];
			parent.Childrens.push_back(&m_Nodes[i]);
		}
	}

	int nTreeNodes = 0;
	for (size_t i = 0; i < m_Roots.size(); ++i)
	{
		nTreeNodes += CalcNumNodes(m_Roots[i]);
	}

	if (nTreeNodes != (int)m_Nodes.size())
	{
		return false;
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
	if (m_Pause || m_Roots.empty())
	{
		return;
	}

	for (size_t i = 0; i < m_Roots.size(); ++i)
	{
		SimulateNode(elapsed * m_PlayRate, nullptr, m_Roots[i]);
	}
}

void AnimationTree::SimulateNode(float elapsed, AnimTreeNode* parent, AnimTreeNode* node)
{
	Vector3d pos = Vector3d::Zero();
	Quaternion quat = Quaternion::One();
	node->Anim.Advance(elapsed, &pos, &quat);

	if (parent)
	{
		node->X = parent->X + parent->Q * pos;
		node->Q = parent->Q * quat;
	}
	else
	{
		node->X = pos;
		node->Q = quat;
	}

	if (node->Entity)
	{
		node->Entity->SetTransform(node->X, node->Q);
	}

	for (size_t i = 0; i < node->Childrens.size(); ++i)
	{
		SimulateNode(elapsed, node, node->Childrens[i]);
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
	for (size_t i = 0; i < m_Nodes.size(); ++i)
	{
		if (m_Nodes[i].Name == node_name)
		{
			m_Nodes[i].Entity = body;
			m_Nodes[i].X = body->X;
			m_Nodes[i].Q = body->Q;
			return true;
		}
	}

	return false;
}

void AnimationTree::UnBind(RigidBodyStatic* actor)
{
	for (size_t i = 0; i < m_Nodes.size(); ++i)
	{
		if (m_Nodes[i].Entity == actor)
		{
			m_Nodes[i].Entity = nullptr;
		}
	}
}

bool AnimationTree::Deserialize(const std::string& filepath)
{
	m_ResName = filepath;

	AnimTreeData* data = AnimTreeData::Deserialize(filepath.c_str());
	if (data == nullptr)
	{
		return false;
	}

	if (!Build(data))
	{
		delete data;

		m_Nodes.clear();
		m_Roots.clear();
		return false;
	}

	delete data;

	return true;
}