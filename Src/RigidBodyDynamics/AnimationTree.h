#pragma once

#include <string>
#include <vector>

#include "../Maths/Vector3d.h"
#include "../Maths/Quaternion.h"

#include "Animation.h"

class AnimTreeData;
class RigidBodyStatic;

struct AnimTreeNode
{
	AnimTreeNode()
	{
		Entity = nullptr;
		X = Vector3d::Zero();
		Q = Quaternion::One();
	}

	std::string						Name;
	Vector3d						X;
	Quaternion						Q;
	Animation						Anim;
	RigidBodyStatic*				Entity;
	std::vector<AnimTreeNode*>		Childrens;
};

class AnimationTree
{
public:
	AnimationTree();

	void						Simulate(float elapsed);
	bool						Deserialize(const std::string& filepath);
	const std::string&			GetName() const { return m_ResName; }
	void						SetAnimationPlayRate(float play_rate);
	void						Pause(bool pause);
	bool						IsPause() const;

	bool						Bind(const std::string& node_name, RigidBodyStatic* body);
	void						UnBind(RigidBodyStatic* actor);

private:
	bool						Build(AnimTreeData* data);
	void						SimulateNode(float elapsed, AnimTreeNode* parent, AnimTreeNode* node);

private:
	AnimTreeNode*				m_Root;
	float						m_PlayRate;
	bool						m_Pause;
	std::string					m_ResName;
	std::vector<AnimTreeNode>	m_Nodes;
};
