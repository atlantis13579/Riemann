#pragma once

#include <string>
#include <vector>

#include "../Maths/Vector3.h"
#include "../Maths/Quaternion.h"

#include "RigidBody.h"
#include "KeyFrameAnimation.h"

class AnimTreeData;
class RigidBodyStatic;

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

class KinematicsDriver
{
public:
	virtual ~KinematicsDriver() {}
	virtual void		Simulate(float elapsed) = 0;

	const std::string&	GetName() const { return m_ResName; }
	void				SetName(const std::string& Name) { m_ResName = Name; }

private:
	std::string			m_ResName;
};

class KinematicsTree : public KinematicsDriver
{
public:
	KinematicsTree();
	virtual ~KinematicsTree();

	virtual void		Simulate(float elapsed) override final;
	bool				Deserialize(const std::string& filepath);

	void				SetRootTransform(const Vector3& pos, const Quaternion& rot);

	void				SetAnimationPlayRate(float play_rate);
	void				Pause(bool pause);
	bool				IsPause() const;

	bool				BindGeometry(const std::string& node_name, Geometry* geom);

private:
	bool				BuildFlatTree(AnimTreeData* data);

private:
	float				m_PlayRate;
	bool				m_Pause;
	std::string			m_ResPath;
	std::vector<KeyframeKinematics*>	m_FlatTree;
};
