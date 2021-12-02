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
	AnimationTree() {}

	void						Tick(float elapsed);
	bool						Deserialize(const std::string& filepath);
	const std::string&			GetName() const { return m_ResName; }

	bool						Bind(const std::string& node_name, RigidBodyStatic* actor);
	void						UnBind(RigidBodyStatic* actor);

private:
	bool						Build(AnimTreeData* data);
	void						TickDfs(float elapsed_ms, AnimTreeNode* parent, AnimTreeNode* node);

private:
	AnimTreeNode*				m_Root;
	std::string					m_ResName;
	std::vector<AnimTreeNode>	m_Nodes;
};
