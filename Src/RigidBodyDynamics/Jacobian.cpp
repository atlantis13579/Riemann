#include <assert.h>
#include <float.h>
#include <algorithm>
#include "../Maths/Maths.h"
#include "Jacobian.h"
#include "RigidBody.h"
#include "CollidingContact.h"

namespace Riemann
{
	// http://allenchou.net/2013/12/game-physics-constraints-sequential-impulse/
	// http://allenchou.net/2013/12/game-physics-resolution-contact-constraints/
	void Jacobian::Setup(Contact* contact, RigidBody* bodyA, RigidBody* bodyB, GeneralizedVelocity* ga, GeneralizedVelocity* gb, const Vector3& dir, float bias, RagdollVelocityContext* Context)
	{
		m_jva = -dir;
		m_jwa = -contact->PositionLocalA.Cross(dir);
		m_jvb = dir;
		m_jwb = contact->PositionLocalB.Cross(dir);

		m_bias = bias;
		m_error = 0.0f;

		m_invmjva = bodyA->GetInverseMass() * m_jva;
		m_invmjwa = bodyA->GetInverseInertia_WorldSpace() * m_jwa;
		m_invmjvb = bodyB->GetInverseMass() * m_jvb;
		m_invmjwb = bodyB->GetInverseInertia_WorldSpace() * m_jwb;

		// k = J * inv(Mass Matrix) * J^T
		float k = DotProduct(m_jva, m_invmjva) + DotProduct(m_jwa, m_invmjwa) + DotProduct(m_jvb, m_invmjvb) + DotProduct(m_jwb, m_invmjwb);

		m_ReducedRagdoll = Context && (bodyA->ReducedRagdoll || bodyB->ReducedRagdoll);
		m_Response.Changes.clear();
		if (m_ReducedRagdoll) k = (float)Context->Response(bodyA, m_jva, m_jwa, bodyB, m_jvb, m_jwb, m_Response);
		m_effectiveMass = k > 0 ? 1.0f / k : 0.0f;
		m_totalLambda = 0.0f;

		m_ga = ga;
		m_gb = gb;
		return;
	}

	void Jacobian::Solve(float lambdamin, float lambdamax)
	{
		float jv = DotProduct(m_jva, m_ga->v)
			+ DotProduct(m_jwa, m_ga->w)
			+ DotProduct(m_jvb, m_gb->v)
			+ DotProduct(m_jwb, m_gb->w);

		// Given v, find dv such that J * (v + dv) + b == 0
		// assume dv = (inv(Mass Matrix) * J^T) * lambda (linearity)
		// then lambda =  -(jv + b) / (J * inv(Mass Matrix) * J^T)
		m_error = jv + m_bias;
		float lambda = m_effectiveMass * (-m_error);
		float oldTotalLambda = m_totalLambda;
		m_totalLambda = Maths::Clamp(m_totalLambda + lambda, lambdamin, lambdamax);
		lambda = m_totalLambda - oldTotalLambda;

		if (m_ReducedRagdoll) { m_Response.Apply(lambda); return; }
		m_ga->v += m_invmjva * lambda;
		m_ga->w += m_invmjwa * lambda;
		m_gb->v += m_invmjvb * lambda;
		m_gb->w += m_invmjwb * lambda;
		return;
	}

	void Jacobian::ApplyImpulse(float Impulse)
	{
		m_totalLambda += Impulse;
		if (m_ReducedRagdoll) { m_Response.Apply(Impulse); return; }
		m_ga->v += m_invmjva * Impulse;
		m_ga->w += m_invmjwa * Impulse;
		m_gb->v += m_invmjvb * Impulse;
		m_gb->w += m_invmjwb * Impulse;
	}

	// With restitution slop, we take away just a little bit of energy every time a collision
	// occur, so that the bouncing ball would eventually settle on the floor, completely at rest.
	static const float kRestitutionSlop = 1.0f;

	static float CombineRestitution(const RigidBody* bodyA, const RigidBody* bodyB)
	{
		return std::max(bodyA->GetRestitution(), bodyB->GetRestitution());
	}

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
	static float BaumgarteStabilizationTerm(float dt, float PenetrationDepth, float Slop)
	{
		const float beta = kBaumgarteContactBeta;
		return -(beta / dt) * std::max(PenetrationDepth - Slop, 0.0f);
	}

	void ContactJacobianSolver::SetupPositionPass(Contact* contact, float dt)
	{
		m_contact = contact;

		const float restitution = CombineRestitution(bodyA, bodyB);
		Vector3 va = phase[indexA].v;
		Vector3 wa = phase[indexA].w;
		Vector3 vb = phase[indexB].v;
		Vector3 wb = phase[indexB].w;
		Vector3 relativeVelocity = vb + wb.Cross(contact->PositionLocalB) - va - wa.Cross(contact->PositionLocalA);
		float closingSpeed = relativeVelocity.Dot(m_contact->Normal);
		closingSpeed = std::min(closingSpeed + kRestitutionSlop, 0.0f);
		closingSpeed = closingSpeed < -kRestitutionSlop ? closingSpeed : 0.0f;
		float bias = BaumgarteStabilizationTerm(dt, contact->PenetrationDepth, contact->SolverSlop >= 0.0f ? contact->SolverSlop : kPenetrationSlop) + restitution * closingSpeed;
		if (contact->SolverSlop >= 0.0f && contact->PenetrationDepth < 0.0f) bias = -contact->PenetrationDepth / dt;
		m_jN.Setup(contact, bodyA, bodyB, phase + indexA, phase + indexB, m_contact->Normal, bias, ReducedRagdolls);
		m_jT.Setup(contact, bodyA, bodyB, phase + indexA, phase + indexB, m_contact->Tangent, 0.0f, ReducedRagdolls);
		m_jB.Setup(contact, bodyA, bodyB, phase + indexA, phase + indexB, m_contact->Binormal, 0.0f, ReducedRagdolls);
	}

	void ContactJacobianSolver::SetupVelocityPass(Contact* contact, float dt, int n)
	{
		const float restitution = CombineRestitution(bodyA, bodyB);
		Vector3 va = phase[n + indexA].v;
		Vector3 wa = phase[n + indexA].w;
		Vector3 vb = phase[n + indexB].v;
		Vector3 wb = phase[n + indexB].w;
		Vector3 relativeVelocity = vb + wb.Cross(contact->PositionLocalB) - va - wa.Cross(contact->PositionLocalA);
		float closingSpeed = relativeVelocity.Dot(m_contact->Normal);
		closingSpeed = std::min(closingSpeed + kRestitutionSlop, 0.0f);
		closingSpeed = closingSpeed < -kRestitutionSlop ? closingSpeed : 0.0f;
		float bias = contact->SolverSlop >= 0.0f && contact->PenetrationDepth < 0.0f
			? -contact->PenetrationDepth / dt : restitution * closingSpeed;
		m_jN.Setup(contact, bodyA, bodyB, phase + n + indexA, phase + n + indexB, m_contact->Normal, bias, ReducedRagdolls);
		m_jT.Setup(contact, bodyA, bodyB, phase + n + indexA, phase + n + indexB, m_contact->Tangent, 0.0f, ReducedRagdolls);
		m_jB.Setup(contact, bodyA, bodyB, phase + n + indexA, phase + n + indexB, m_contact->Binormal, 0.0f, ReducedRagdolls);
	}

	void ContactJacobianSolver::Solve()
	{
		m_jN.Solve(0.0f, FLT_MAX);
		if (UsePatchFriction) return;

		float friction = bodyA->GetFrictionDynamic() * bodyB->GetFrictionDynamic();
		float maxFriction = m_contact->SolverSlop >= 0.0f && m_contact->PenetrationDepth < 0.0f ? 0.0f : friction * m_jN.m_totalLambda;
		m_jT.Solve(-maxFriction, maxFriction);
		m_jB.Solve(-maxFriction, maxFriction);
	}

	float ContactJacobianSolver::GetSquaredError() const
	{
		return m_jN.m_error * m_jN.m_error + m_jT.m_error * m_jT.m_error + m_jB.m_error * m_jB.m_error;
	}

	void ContactJacobianSolver::Finalize(bool Accumulate)
	{
		if (Accumulate)
		{
			m_contact->totalImpulseNormal += m_jN.m_totalLambda;
			m_contact->totalImpulseTangent += m_jT.m_totalLambda;
			m_contact->totalImpulseBinormal += m_jB.m_totalLambda;
		}
		else
		{
			m_contact->totalImpulseNormal = m_jN.m_totalLambda;
			m_contact->totalImpulseTangent = m_jT.m_totalLambda;
			m_contact->totalImpulseBinormal = m_jB.m_totalLambda;
		}
	}
}
