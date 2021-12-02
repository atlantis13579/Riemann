#pragma once

#include <string>
#include <vector>

#include "RigidBody.h"

class Geometry;
class GeometryQuery;
class BroadPhase;
class NarrowPhase;
class ForceField;
class AnimationTree;

struct RigidBodySimulationParam
{
	Vector3d Gravity;
};

class RigidBodySimulation
{
public:
	RigidBodySimulation(const RigidBodySimulationParam&param);
	~RigidBodySimulation();

public:
	void		Simulate(float dt);
	void		ApplyGravity();
	void		ApplyWind();

	RigidBody*	CreateRigidBody(Geometry *Geom, const RigidBodyParam &param);
	bool		LoadAnimation(const std::string &filepath, float play_rate);

	bool		BindAnimationNode(const std::string& anim, const std::string& node, RigidBodyStatic* body);

private:
	AnimationTree* FindAnimation(const std::string& anim);

private:
	std::vector<RigidBodyStatic*>	m_RigidStatics;
	std::vector<RigidBodyDynamic*>	m_RigidDynamics;
	std::vector<AnimationTree*>		m_Animations;

	GeometryQuery*	m_GeometryQuery;
	BroadPhase*		m_BPhase;
	NarrowPhase*	m_NPhase;
	ForceField*		m_GravityField;
	ForceField*		m_WindField;
};