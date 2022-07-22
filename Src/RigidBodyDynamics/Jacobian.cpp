
#include "../Maths/Maths.h"
#include "../Collision/GeometryObject.h"
#include "Jacobian.h"
#include "RigidBody.h"
#include "Contact.h"

RigidBody* GetRigidBody(Geometry *Geom)
{
	return static_cast<RigidBody*>(Geom->GetEntity());
}

// Beta is typically a value between 0 and 1 (usually close to 0)
// Unfortunately, there is no one correct value for beta; 
// you will have to tweak this value based on the specific scenarios.
// Erin Catto proposed that you start increasing the value from 0 
// until your physics system starts to become unstable, and then use half that value.
static float BaumgarteContactBeta()
{
	return 0.1f;
}

// Baumgarte Stabilization
static float BaumgarteStabilizationTerm(float dt, float PenetrationDepth)
{
	const float beta = BaumgarteContactBeta();
	return -(beta / dt) * PenetrationDepth;
}

void Jacobian::Setup(ContactResult* contact, Geometry *GeomA, Geometry *GeomB, JacobianType jt, const Vector3d& dir, float dt)
{
	jacobinType = jt;

	Vector3d ra = contact->PositionWorldA - GeomA->GetPosition();
	Vector3d rb = contact->PositionWorldB - GeomB->GetPosition();

	m_jva = -dir;
	m_jwa = -ra.Cross(dir);
	m_jvb = dir;
	m_jwb = rb.Cross(dir);

	m_bias = 0.0f;
	
	RigidBody* rigidA = GetRigidBody(GeomA);
	RigidBody* rigidB = GetRigidBody(GeomB);

	if (jacobinType == JacobianType::Normal)
	{
		float restitutionA = rigidA->GetRestitution();
		float restitutionB = rigidB->GetRestitution();
		float restitution = restitutionA * restitutionB;

		Vector3d va = rigidA->GetLinearVelocity();
		Vector3d wa = rigidA->GetAngularVelocity();

		Vector3d vb = rigidB->GetLinearVelocity();
		Vector3d wb = rigidB->GetAngularVelocity();

		Vector3d relativeVelocity = vb + CrossProduct(wb, rb) - va - CrossProduct(wa, ra);
		float closingVelocity = relativeVelocity.Dot(dir);
		m_bias = BaumgarteStabilizationTerm(dt, contact->PenetrationDepth) + restitution * closingVelocity;
	}

	Vector3d rva = rigidA->GetInverseInertia() * m_jwa;
	Vector3d rvb = rigidB->GetInverseInertia() * m_jwb;

	float k = rigidA->GetInverseMass() + rigidB->GetInverseMass() + DotProduct(m_jwa, rva) + DotProduct(m_jwb, rvb);

	m_effectiveMass = 1.0f / k;
	m_totalLambda = 0.0f;
	return;
}

void Jacobian::Solve(Geometry* GeomA, Geometry* GeomB, float totalLambda)
{
	RigidBody* rigidA = GetRigidBody(GeomA);
	RigidBody* rigidB = GetRigidBody(GeomB);
	
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
		float friction = rigidA->GetFrictionDynamic() * rigidB->GetFrictionDynamic();
		float maxFriction = friction * totalLambda;
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

