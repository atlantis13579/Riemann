#pragma once

#include "../Maths/Vector3d.h"


// http://allenchou.net/2013/12/game-physics-resolution-contact-constraints/
// http://allenchou.net/2013/12/game-physics-constraints-sequential-impulse/

class ContactManifold;
class ContactResult;

enum JacobianType
{
	Normal,
	Tangent
};

// Jacobian Matrix is a 1x12 Matrix which satisfy JV + b = 0
// Where V is 12x1 generalized velocity [va, wa, vb, wb]^T
// b is the bias term
struct  Jacobian
{
	Jacobian()
	{
		m_jva = Vector3d::Zero();
		m_jwa = Vector3d::Zero();
		m_jvb = Vector3d::Zero();
		m_jwa = Vector3d::Zero();
	}

	void Setup(ContactResult *contact, Geometry* GeomA, Geometry* GeomB, JacobianType jt, const Vector3d& dir, float dt);
	void Solve(Geometry* GeomA, Geometry* GeomB, float totalLambda);

	JacobianType jacobinType;
	Vector3d m_jva;
	Vector3d m_jwa;
	Vector3d m_jvb;
	Vector3d m_jwb;
	float m_bias;
	float m_effectiveMass;
	float m_totalLambda;
};
