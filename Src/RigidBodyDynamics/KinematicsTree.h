#pragma once

#include <string>
#include <vector>

#include "../Maths/Vector3.h"
#include "../Maths/Quaternion.h"

#include "KeyFrameAnimation.h"

class AnimTreeData;
class RigidBodyStatic;

struct AnimTreeNode
{
	AnimTreeNode()
	{
		Entity = nullptr;
		X = Vector3::Zero();
		Q = Quaternion::One();
		Parent = -1;
	}

	std::string						Name;
	Vector3						X;
	Quaternion						Q;
	KeyFrameAnimation						Anim;
	RigidBodyStatic*				Entity;
	int								Parent;
};

class KinematicsTree
{
public:
	KinematicsTree();

	void						Simulate(float elapsed);
	bool						Deserialize(const std::string& filepath);

	void						SetRootTransform(const Vector3& pos, const Quaternion& rot);
	const std::string&			GetName() const { return m_ResName; }
	void						SetName(const std::string& Name) { m_ResName = Name; }
	void						SetAnimationPlayRate(float play_rate);
	void						Pause(bool pause);
	bool						IsPause() const;

	bool						Bind(const std::string& node_name, RigidBodyStatic* body);
	void						UnBind(RigidBodyStatic* body);

private:
	bool						BuildFlatTree(AnimTreeData* data);

private:
	float						m_PlayRate;
	bool						m_Pause;
	std::string					m_ResName;
	std::string					m_ResPath;
	std::vector<AnimTreeNode>	m_FlatTree;
};
