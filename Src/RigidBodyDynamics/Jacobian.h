#pragma once

#include "../Maths/Vector3d.h"

class ContactManifold;

enum JacobianType
{
	Normal,
	Tangent
};

struct  Jacobian
{
	Jacobian()
	{
		m_jva = Vector3d::Zero();
		m_jwa = Vector3d::Zero();
		m_jvb = Vector3d::Zero();
		m_jwa = Vector3d::Zero();
	}

	void Setup(ContactManifold* manifold, int idx, JacobianType jt, const Vector3d& dir, float dt);
	void Solve(ContactManifold* manifold, Jacobian& jN, float dt);

	JacobianType jacobinType;
	Vector3d m_jva;
	Vector3d m_jwa;
	Vector3d m_jvb;
	Vector3d m_jwb;
	float m_bias;
	float m_effectiveMass;
	float m_totalLambda;

};
