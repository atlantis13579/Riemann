#include "CollidingContact.h"
#include "WarmStart.h"
#include "RigidBody.h"
#include "Jacobian.h"
#include "../Collision/GeometryObject.h"
#include <algorithm>
#include <cmath>

namespace Riemann
{
	namespace
	{
		Vector3 CachedImpulse(const Contact& ContactPoint, float Dt, const ContactPersistenceSettings& Settings)
		{
			if (!std::isfinite(Dt) || Dt <= 0.0f || !std::isfinite(ContactPoint.CachedDt) || ContactPoint.CachedDt <= 0.0f)
				return Vector3::Zero();
			const float Ratio = std::max(Settings.MinTimeStepRatio, std::min(Settings.MaxTimeStepRatio, Dt / ContactPoint.CachedDt));
			const Vector3 Impulse = ContactPoint.Normal * ContactPoint.totalImpulseNormal
				+ ContactPoint.Tangent * ContactPoint.totalImpulseTangent + ContactPoint.Binormal * ContactPoint.totalImpulseBinormal;
			if (!std::isfinite(Impulse.x) || !std::isfinite(Impulse.y) || !std::isfinite(Impulse.z)) return Vector3::Zero();
			return Impulse * (Settings.WarmStartFactor * Ratio);
		}
	}

	// Warm starting seeds both velocities AND accumulated constraint multipliers.
	// Otherwise the next PGS iteration cannot retract an excessive cached impulse.
	// Bias/restitution targets must be computed from the pre-warm-start velocities.
	// See Erin Catto, "Iterative Dynamics with Temporal Coherence", GDC 2005.
	void WarmStart::Apply(ContactJacobianSolver& Constraint, float Dt, const ContactPersistenceSettings& Settings)
	{
		const Contact& Point = *Constraint.m_contact;
		const Vector3 Impulse = CachedImpulse(Point, Dt, Settings);
		const float Normal = std::max(0.0f, Impulse.Dot(Point.Normal));
		const float FrictionLimit = Point.SolverSlop >= 0.0f && Point.PenetrationDepth < 0.0f ? 0.0f : Constraint.bodyA->GetFrictionDynamic() * Constraint.bodyB->GetFrictionDynamic() * Normal;
		Constraint.m_jN.ApplyImpulse(Normal);
		if (Constraint.UsePatchFriction) return;
		Constraint.m_jT.ApplyImpulse(std::max(-FrictionLimit, std::min(FrictionLimit, Impulse.Dot(Point.Tangent))));
		Constraint.m_jB.ApplyImpulse(std::max(-FrictionLimit, std::min(FrictionLimit, Impulse.Dot(Point.Binormal))));
	}

	void WarmStart::ApplyVelocityConstraint(const std::vector<Geometry*>& Geometries, std::vector<ContactManifold*>& Manifolds, float Dt)
	{
		for (ContactManifold* Manifold : Manifolds)
		{
			if (!Manifold || Manifold->indexA < 0 || Manifold->indexB < 0
				|| (size_t)Manifold->indexA >= Geometries.size() || (size_t)Manifold->indexB >= Geometries.size()) continue;
			RigidBody* A = Geometries[(size_t)Manifold->indexA]->GetParent<RigidBody>();
			RigidBody* B = Geometries[(size_t)Manifold->indexB]->GetParent<RigidBody>();
			for (int Index = 0; Index < Manifold->NumContactPointCount; ++Index) Apply(A, B, Manifold->ContactPoints[Index], Dt);
		}
	}

	void WarmStart::Apply(RigidBody* BodyA, RigidBody* BodyB, Contact& Point, float Dt)
	{
		const Vector3 Impulse = CachedImpulse(Point, Dt, ContactPersistenceSettings());
		RigidBodyDynamic* A = BodyA ? BodyA->CastDynamic() : nullptr;
		RigidBodyDynamic* B = BodyB ? BodyB->CastDynamic() : nullptr;
		if (A && !A->Freezing)
		{
			A->V -= Impulse * A->InvMass;
			A->W -= A->GetInverseInertia_WorldSpace() * Point.PositionLocalA.Cross(Impulse);
			if (!Impulse.IsZero()) A->Wakeup();
		}
		if (B && !B->Freezing)
		{
			B->V += Impulse * B->InvMass;
			B->W += B->GetInverseInertia_WorldSpace() * Point.PositionLocalB.Cross(Impulse);
			if (!Impulse.IsZero()) B->Wakeup();
		}
	}
}
