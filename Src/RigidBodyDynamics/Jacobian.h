#pragma once

#include "../Maths/Vector3.h"

// http://allenchou.net/2013/12/game-physics-resolution-contact-constraints/

struct Contact;
class RigidBody;
class ContactManifold;

// Jacobian is a 1x12 Matrix which satisfy JV + b = 0
// Where V is 12x1 generalized velocity [va, wa, vb, wb]^T
// b is the bias term
struct  Jacobian
{
	enum
	{
		Normal,
		Tangent,
		Binormal,
	};

	Jacobian(int jt)
	{
		m_Type = jt;
		m_jva = Vector3d::Zero();
		m_jwa = Vector3d::Zero();
		m_jvb = Vector3d::Zero();
		m_jwa = Vector3d::Zero();
		m_bodyA = m_bodyB = nullptr;
	}

	void	Setup(const Contact *contact, RigidBody* bodyA, RigidBody* bodyB, const Vector3d& dir, float dt);
	void	Solve(float clampmin, float clampmax);

	int m_Type;
	Vector3d m_jva;
	Vector3d m_jwa;
	Vector3d m_jvb;
	Vector3d m_jwb;
	float	m_bias;
	float	m_effectiveMass;
	float	m_totalLambda;
	RigidBody* m_bodyA;
	RigidBody* m_bodyB;
};

// Velocity Constraint consists of three Jacobian constraint
// JN * V + b == 0, JT * V + b == 0, JB * V + b == 0
// Where V is 12x1 generalized velocity [va, wa, vb, wb]^T
// b is the bias term
// to model the equation : (vb + CrossProduct(wb, rb) - va - CrossProduct(wa, ra)) * n == 0
struct ContactVelocityConstraintSolver
{
	ContactVelocityConstraintSolver() :
		m_jN(Jacobian::Normal),
		m_jT(Jacobian::Tangent),
		m_jB(Jacobian::Binormal)
	{
	}

	void Setup(Contact* contact, RigidBody* bodyA, RigidBody* bodyB, float dt);
	void Solve();
	void Finalize();

	Contact*	m_contact;
	Jacobian	m_jN;
	Jacobian	m_jT;
	Jacobian	m_jB;
};