
#include <float.h>
#include "../Maths/Maths.h"
#include "Jacobian.h"
#include "RigidBody.h"
#include "Contact.h"

// Beta is typically a value between 0 and 1 (usually close to 0)
// Unfortunately, there is no one correct value for beta; 
// you will have to tweak this value based on the specific scenarios.
// Erin Catto proposed that you start increasing the value from 0 
// until your physics system starts to become unstable, and then use half that value.
static inline float BaumgarteContactBeta()
{
	return 0.1f;
}

// Baumgarte Stabilization
static float BaumgarteStabilizationTerm(float dt, float PenetrationDepth)
{
	const float beta = BaumgarteContactBeta();
	return -(beta / dt) * PenetrationDepth;
}

// http://allenchou.net/2013/12/game-physics-constraints-sequential-impulse/
void Jacobian::Setup(const Contact* contact, RigidBody* rigidA, RigidBody* rigidB, const Vector3d& dir, float dt)
{
	Vector3d ra = contact->PositionWorldA - rigidA->X;
	Vector3d rb = contact->PositionWorldB - rigidB->X;

	m_jva = -dir;
	m_jwa = -ra.Cross(dir);
	m_jvb = dir;
	m_jwb = rb.Cross(dir);

	m_rigidA = rigidA;
	m_rigidB = rigidB;

	m_bias = 0.0f;

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

void Jacobian::Solve()
{
	float jv = DotProduct(m_jva, m_rigidA->GetLinearVelocity())
			 + DotProduct(m_jwa, m_rigidA->GetAngularVelocity())
			 + DotProduct(m_jvb, m_rigidB->GetLinearVelocity())
			 + DotProduct(m_jwb, m_rigidB->GetAngularVelocity());

	float lambda = m_effectiveMass * (-jv - m_bias);
	float oldTotalLambda = m_totalLambda;
	if (jacobinType == JacobianType::Normal)
	{
		m_totalLambda = Clamp(m_totalLambda + lambda, 0.0f, FLT_MAX);
	}
	else
	{
		float friction = m_rigidA->GetFrictionDynamic() * m_rigidB->GetFrictionDynamic();
		float maxFriction = friction * m_totalLambda;
		m_totalLambda = Clamp(m_totalLambda + lambda, -maxFriction, maxFriction);
	}
	lambda = m_totalLambda - oldTotalLambda;
	
	Vector3d dpa = m_jva * lambda;
	Vector3d dwa = (m_rigidA->GetInverseInertia_WorldSpace() * m_jwa) * lambda;
	Vector3d dpb = m_jvb * lambda;
	Vector3d dwb = (m_rigidB->GetInverseInertia_WorldSpace() * m_jwb) * lambda;

	m_rigidA->AddLinearMomentum(dpa);
	m_rigidA->AddAngularVelocity(dwa);
	m_rigidB->AddLinearMomentum(dpb);
	m_rigidB->AddAngularVelocity(dwb);
	return;
}

