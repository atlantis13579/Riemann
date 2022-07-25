
#include <float.h>
#include <algorithm>
#include "../Maths/Maths.h"
#include "Jacobian.h"
#include "RigidBody.h"
#include "Contact.h"

// With restitution slop, we take away just a little bit of energy every time a collision
// occur, so that the bouncing ball would eventually settle on the floor, completely at rest.
static const float kRestitutionSlop = 1.0f;

// Beta is typically a value between 0 and 1 (usually close to 0)
// Unfortunately, there is no one correct value for beta; 
// you will have to tweak this value based on the specific scenarios.
// Erin Catto proposed that you start increasing the value from 0 
// until your physics system starts to become unstable, and then use half that value.
static const float kBaumgarteContactBeta = 0.1f;

// Allow objects to penetrate a bit before actually applying Baumgarte Stabiliation
// Without slop, if two colliders are just penetrating by a teeny bit, extra impulse is applied. 
// This would result in unnecessary jitter and objects will hardly sit tight when they are supposed to be at rest
static const float kPenetrationSlop = 0.05f;

// Baumgarte Stabilization
static float BaumgarteStabilizationTerm(float dt, float PenetrationDepth)
{
	const float beta = kBaumgarteContactBeta;
	return -(beta / dt) * std::max(PenetrationDepth - kPenetrationSlop, 0.0f);
}

// http://allenchou.net/2013/12/game-physics-constraints-sequential-impulse/
void Jacobian::Setup(const Contact* contact, RigidBody* bodyA, RigidBody* bodyB, const Vector3& dir, float dt)
{
	Vector3 ra = contact->PositionWorldA - bodyA->X;
	Vector3 rb = contact->PositionWorldB - bodyB->X;

	m_jva = -dir;
	m_jwa = -ra.Cross(dir);
	m_jvb = dir;
	m_jwb = rb.Cross(dir);

	m_bodyA = bodyA;
	m_bodyB = bodyB;

	m_bias = 0.0f;

	if (m_Type == Jacobian::Normal)
	{
		float restitutionA = bodyA->GetRestitution();
		float restitutionB = bodyB->GetRestitution();
		float restitution = restitutionA * restitutionB;

		Vector3 va = bodyA->GetLinearVelocity();
		Vector3 wa = bodyA->GetAngularVelocity();
		Vector3 vb = bodyB->GetLinearVelocity();
		Vector3 wb = bodyB->GetAngularVelocity();

		Vector3 relativeVelocity = vb + CrossProduct(wb, rb) - va - CrossProduct(wa, ra);
		float closingSpeed = relativeVelocity.Dot(dir);
		closingSpeed = std::min(closingSpeed + kRestitutionSlop, 0.0f);
		closingSpeed = closingSpeed < -kRestitutionSlop ? closingSpeed : 0.0f;
		m_bias = BaumgarteStabilizationTerm(dt, contact->PenetrationDepth) + restitution * closingSpeed;
	}

	Vector3 rva = bodyA->GetInverseInertia() * m_jwa;
	Vector3 rvb = bodyB->GetInverseInertia() * m_jwb;

	float k = bodyA->GetInverseMass() + bodyB->GetInverseMass() + DotProduct(m_jwa, rva) + DotProduct(m_jwb, rvb);

	m_effectiveMass = 1.0f / k;
	m_totalLambda = 0.0f;
	return;
}

void Jacobian::Solve(float clampmin, float clampmax)
{
	float jv = DotProduct(m_jva, m_bodyA->GetLinearVelocity())
			 + DotProduct(m_jwa, m_bodyA->GetAngularVelocity())
			 + DotProduct(m_jvb, m_bodyB->GetLinearVelocity())
			 + DotProduct(m_jwb, m_bodyB->GetAngularVelocity());

	float lambda = m_effectiveMass * (-jv - m_bias);
	float oldTotalLambda = m_totalLambda;
	m_totalLambda = Clamp(m_totalLambda + lambda, clampmin, clampmax);
	lambda = m_totalLambda - oldTotalLambda;
	
	Vector3 dpa = m_jva * lambda;
	Vector3 dwa = (m_bodyA->GetInverseInertia_WorldSpace() * m_jwa) * lambda;
	Vector3 dpb = m_jvb * lambda;
	Vector3 dwb = (m_bodyB->GetInverseInertia_WorldSpace() * m_jwb) * lambda;

	m_bodyA->AddLinearMomentum(dpa);
	m_bodyA->AddAngularVelocity(dwa);
	m_bodyB->AddLinearMomentum(dpb);
	m_bodyB->AddAngularVelocity(dwb);
	return;
}

void ContactVelocityConstraintSolver::Setup(Contact* contact, RigidBody* bodyA, RigidBody* bodyB, float dt)
{
	m_contact = contact;
	m_jN.Setup(m_contact, bodyA, bodyB, m_contact->Normal, dt);
	m_jT.Setup(m_contact, bodyA, bodyB, m_contact->Tangent, dt);
	m_jB.Setup(m_contact, bodyA, bodyB, m_contact->Binormal, dt);
}

void ContactVelocityConstraintSolver::Solve()
{
	m_jN.Solve(0.0f, FLT_MAX);

	float friction = m_jN.m_bodyA->GetFrictionDynamic() * m_jN.m_bodyB->GetFrictionDynamic();
	float maxFriction = friction * m_jN.m_totalLambda;
	m_jT.Solve(-maxFriction, maxFriction);
	m_jB.Solve(-maxFriction, maxFriction);
}

void ContactVelocityConstraintSolver::Finalize()
{
	m_contact->totalImpulseNormal = m_jN.m_totalLambda;
	m_contact->totalImpulseTangent = m_jT.m_totalLambda;
	m_contact->totalImpulseBinormal = m_jB.m_totalLambda;
}
