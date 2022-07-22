#pragma once

#include "../Maths/Vector3d.h"

// http://allenchou.net/2013/12/game-physics-resolution-contact-constraints/

class Contact;
class RigidBody;

enum JacobianType
{
	Normal,
	Tangent,
	Binormal,
};

// Jacobian is a 1x12 Matrix which satisfy JV + b = 0
// Where V is 12x1 generalized velocity [va, wa, vb, wb]^T
// b is the bias term
struct  Jacobian
{
	Jacobian(JacobianType jt)
	{
		jacobinType = jt;
		m_jva = Vector3d::Zero();
		m_jwa = Vector3d::Zero();
		m_jvb = Vector3d::Zero();
		m_jwa = Vector3d::Zero();
		m_rigidA = m_rigidB = nullptr;
	}

	void	Setup(const Contact *contact, RigidBody* rigidA, RigidBody* rigidB, const Vector3d& dir, float dt);
	void	Solve(float clampFactor);

	JacobianType jacobinType;
	Vector3d m_jva;
	Vector3d m_jwa;
	Vector3d m_jvb;
	Vector3d m_jwb;
	float	m_bias;
	float	m_effectiveMass;
	float	m_totalLambda;
	RigidBody* m_rigidA;
	RigidBody* m_rigidB;
};
