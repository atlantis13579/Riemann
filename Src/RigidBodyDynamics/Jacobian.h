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
	void Solve(Geometry* GeomA, Geometry* GeomB, Jacobian& jN, float dt);

	JacobianType jacobinType;
	Vector3d m_jva;
	Vector3d m_jwa;
	Vector3d m_jvb;
	Vector3d m_jwb;
	float m_bias;
	float m_effectiveMass;
	float m_totalLambda;
};
