
#include "KinematicsTree.h"
#include "../Tools/AnimBinaryParser.h"
#include "RigidBody.h"

KinematicsTree::KinematicsTree()
{
	m_Pause = true;
	m_PlayRate = 1.0f;
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
