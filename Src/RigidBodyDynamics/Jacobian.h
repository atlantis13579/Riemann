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
	Jacobian()
	{
		m_bodyA = m_bodyB = nullptr;
	}

	void	Setup(Contact* contact, RigidBody* bodyA, RigidBody* bodyB, const Vector3& dir, float bias);
	void	Solve(float lambdamin, float lambdamax);

	Vector3 m_jva;
	Vector3 m_jwa;
	Vector3 m_jvb;
	Vector3 m_jwb;
	float	m_error;
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
	ContactVelocityConstraintSolver()
	{
	}

	void	Setup(Contact* contact, RigidBody* bodyA, RigidBody* bodyB, float dt);
	void	Solve();
	float	GetSquaredError() const;
	void	Finalize();

	Contact*	m_contact;
	Jacobian	m_jN;
	Jacobian	m_jT;
	Jacobian	m_jB;
};