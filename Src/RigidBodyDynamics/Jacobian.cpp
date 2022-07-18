
#include "Jacobian.h"
#include "RigidBody.h"
#include "../Maths/Maths.h"
#include "../Collision/Contact.h"
#include "../Collision/GeometryObject.h"

RigidBody* GetRigidBody(Geometry *Geom)
{
	return static_cast<RigidBody*>(Geom->GetEntity());
}

void Jacobian::Init(ContactManifold* manifold, int idx, JacobianType jt, Vector3d dir, float dt)
{
	jacobinType = jt;

	m_jva = dir * -1;
	m_jwa = manifold->ContactPoints[idx].R1.Cross(dir) * -1.0f;
	m_jvb = dir;
	m_jwb = manifold->ContactPoints[idx].R2.Cross(dir);

	m_bias = 0.0f;
	
	RigidBody* rigidA = GetRigidBody(manifold->Geom1);
	RigidBody* rigidB = GetRigidBody(manifold->Geom2);

	if (jacobinType == JacobianType::Normal)
	{
		float betaA = rigidA->GetContactBeta();
		float betaB = rigidB->GetContactBeta();
		float beta = betaA * betaB;

		float restitutionA = rigidA->GetRestitution();
		float restitutionB = rigidB->GetRestitution();
		float restitution = restitutionA * restitutionB;

		Vector3d va = rigidA->GetLinearVelocity();
		Vector3d wa = rigidA->GetAngularVelocity();
		Vector3d ra = manifold->ContactPoints[idx].R1;

		Vector3d vb = rigidB->GetLinearVelocity();
		Vector3d wb = rigidB->GetAngularVelocity();
		Vector3d rb = manifold->ContactPoints[idx].R2;

		m_bias = 0;
		if (jacobinType == JacobianType::Normal)
		{
			// http://allenchou.net/2013/12/game-physics-resolution-contact-constraints/
			Vector3d relativeVelocity = vb + CrossProduct(wb, rb) - va - CrossProduct(wa, ra);
			float closingVelocity = relativeVelocity.Dot(dir);
			m_bias = -(beta / dt) * manifold->ContactPoints[idx].PenetrationDepth + restitution * closingVelocity;
		}
	}
		
	// http://allenchou.net/2013/12/game-physics-constraints-sequential-impulse/
	// https://www.youtube.com/watch?v=pmdYzNF9x34
	Vector3d rva = rigidA->GetInverseInertia() * m_jwa;
	Vector3d rvb = rigidB->GetInverseInertia() * m_jwb;

	float k =
		rigidA->GetInverseMass() + rigidB->GetInverseMass()
		+ DotProduct(m_jwa, rva)
		+ DotProduct(m_jwb, rvb);

	m_effectiveMass = 1.0f / k;
	m_totalLambda = 0.0f;
}

void Jacobian::Solve(ContactManifold* manifold, Jacobian& jN, float dt)
{
	RigidBody* rigidA = GetRigidBody(manifold->Geom1);
	RigidBody* rigidB = GetRigidBody(manifold->Geom2);
	
	float jv = DotProduct(m_jva, rigidA->GetLinearVelocity())
		+ DotProduct(m_jwa, rigidA->GetAngularVelocity())
		+ DotProduct(m_jvb, rigidB->GetLinearVelocity())
		+ DotProduct(m_jwb, rigidB->GetAngularVelocity());

	float lambda = m_effectiveMass * (-(jv + m_bias));
	float oldTotalLambda = m_totalLambda;
	switch (jacobinType)
	{
	case JacobianType::Normal:
		m_totalLambda = std::max(m_totalLambda + lambda, 0.0f);
		break;

	case JacobianType::Tangent:
		float friction = rigidA->GetFriction() * rigidB->GetFriction();
		float maxFriction = friction * jN.m_totalLambda;
		m_totalLambda = Clamp(m_totalLambda + lambda, -maxFriction, maxFriction);
		break;
	}
	lambda = m_totalLambda - oldTotalLambda;
	
	Vector3d va = rigidA->GetLinearVelocity();
	Vector3d vadelta = m_jva * rigidA->GetInverseMass() * lambda;
	rigidA->SetLinearVelocity(va + vadelta);

	Vector3d wa = rigidA->GetAngularVelocity();
	Vector3d wadelta = (rigidA->GetInverseInertia_WorldSpace() * m_jwa) * lambda;
	rigidA->SetAngularVelocity(wa + wadelta);

	Vector3d vb = rigidB->GetLinearVelocity();
	Vector3d vbdelta = m_jvb * rigidB->GetInverseMass() * lambda;
	rigidB->SetLinearVelocity(vb + vbdelta);

	Vector3d wb = rigidB->GetAngularVelocity();
	Vector3d wbdelta = (rigidB->GetInverseInertia_WorldSpace() * m_jwb) * lambda;
	rigidB->SetAngularVelocity(wb + wbdelta);

	return;
}

