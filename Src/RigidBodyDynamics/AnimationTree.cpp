
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
	m_Root = nullptr;
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
			if (m_Root != nullptr)
			{
				return false;
			}
			m_Root = &m_Nodes[i];
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

	if (m_Root == nullptr)
	{
		return false;
	}

	if (CalcNumNodes(m_Root) != (int)m_Nodes.size())
	{
		return false;
	}

	return 0;
}

void AnimationTree::Tick(float elapsed)
{
	if (m_Root == nullptr)
	{
		return;
	}

	float elapsed_ms = elapsed * 1000.0f;
	TickDfs(elapsed_ms, nullptr, m_Root);
}

void AnimationTree::TickDfs(float elapsed_ms, AnimTreeNode* parent, AnimTreeNode* node)
{
	Vector3d pos;
	Quaternion quat;
	if (node->Anim.Advance(elapsed_ms, &pos, &quat))
	{
		if (parent)
		{
			node->X = parent->X + pos;
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
	}

	for (size_t i = 0; i < node->Childrens.size(); ++i)
	{
		TickDfs(elapsed_ms, node, node->Childrens[i]);
	}
}

bool AnimationTree::Bind(const std::string& node_name, RigidBodyStatic* actor)
{
	if (m_Root == nullptr)
	{
		return false;
	}

	for (size_t i = 0; i < m_Nodes.size(); ++i)
	{
		if (m_Nodes[i].Name == node_name)
		{
			m_Nodes[i].Entity = actor;
			m_Nodes[i].X = actor->GetPosition();
			m_Nodes[i].Q = actor->GetRotation();
			return true;
		}
	}

	return false;
}

void AnimationTree::UnBind(RigidBodyStatic* actor)
{
	if (m_Root == nullptr)
	{
		return;
	}

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
		m_Root = nullptr;
		return false;
	}

	delete data;

	return true;
}