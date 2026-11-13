#include "SurfaceContactConstraint.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace Riemann
{
	/* A surface contact is a unilateral XPBD constraint on barycentric points:
	     C = n dot sum(beta_i*x_i) - thickness >= 0,
	     w = sum(inverseMass_i * beta_i^2), alpha = compliance / dt^2.
	   Side A has positive weights and side B negative weights. Accumulating the
	   normal multiplier and projecting it onto [0,infinity) prevents attraction.
	   Shared particle addresses are combined BEFORE computing w; treating the same
	   degree of freedom twice would produce an incorrect effective mass.

	   Without an explicit Refresh, normal and barycentric coordinates stay fixed.
	   This preserves the pre-impact side ordering even when predicted surfaces
	   have already crossed. The reference is the relative witness at impact, so
	   tangential travel before that time does not consume friction. Kinematic
	   vertices have zero inverse mass but their current positions still supply
	   motion, allowing a moving obstacle to carry a surface through static friction.

	   Tangential displacement is a two-dimensional vector in the contact plane.
	   Its accumulated multiplier sticks inside the static Coulomb disk; outside
	   that disk it is limited to dynamicFriction*normalMultiplier. We apply only
	   the CHANGE, including release when normal load falls. Reapplying the full
	   friction correction on every iteration would make drag iteration-dependent.

	   Both sides receive equal/opposite generalized corrections, preserving the
	   free system's center of mass. A closest-feature normal also acts along the
	   witness separation; a frozen CCD row is a local linearization, not an exact
	   finite-rotation angular-momentum constraint. The caller refreshes/replaces
	   contact features as needed, and reconstructs velocities after projection.
	   Multipliers belong to ONE physical substep. A new row starts at zero.

	   Particle positions are borrowed and must retain their address until the row
	   is discarded. A conservative *distant* sweep result is deliberately rejected:
	   its motion must first be refined or clipped, not projected as penetration.
	   This solver has no rigid-body ownership or collision discovery policy.
	   References: Macklin et al., "XPBD" (2016); Bridson, Fedkiw and Anderson,
	   "Robust Treatment of Collisions, Contact and Friction for Cloth Animation"
	   (2002), barycentric response and Coulomb friction. */
	namespace
	{
		using DVector = Maths::TVector3<double>;
		constexpr double WeightSumTolerance = 1e-5;
		DVector ToDouble(const Vector3& V) { return DVector(V.x, V.y, V.z); }
		bool Finite(const DVector& V) { return std::isfinite(V.x) && std::isfinite(V.y) && std::isfinite(V.z); }
		bool Nonnegative(double Value) { return std::isfinite(Value) && Value >= 0; }
	}
	bool SurfaceContactMaterial::IsValid() const
	{
		return Nonnegative(Compliance) && Nonnegative(StaticFriction) && Nonnegative(DynamicFriction) && DynamicFriction <= StaticFriction;
	}
	void SurfaceContactConstraint::Add(const SurfaceContactParticle* Particles, int Count, const Vector3& Weights, double Sign)
	{
		if (!Particles || Count < 1 || Count > 3) throw std::invalid_argument("Invalid surface contact particles");
		double Sum = 0;
		for (int I = 0; I < 3; ++I)
		{
			if (!Nonnegative(Weights[I]) || (I >= Count && Weights[I] != 0)) throw std::invalid_argument("Invalid surface contact weights");
			Sum += Weights[I];
		}
		if (std::abs(Sum - 1) > WeightSumTolerance) throw std::invalid_argument("Surface contact weights must sum to one");
		for (int I = 0; I < Count; ++I)
		{
			const auto& P = Particles[I];
			if (!P.Position || !Finite(ToDouble(*P.Position)) || !Nonnegative(P.InverseMass)) throw std::invalid_argument("Invalid surface contact particle");
			int Index = 0; while (Index < m_Count && m_Particles[Index].Position != P.Position) ++Index;
			if (Index == m_Count) { ++m_Count; m_Particles[Index].Position = P.Position; m_Particles[Index].InverseMass = P.InverseMass; }
			else if (m_Particles[Index].InverseMass != P.InverseMass) throw std::invalid_argument("Conflicting surface particle inverse masses");
			m_Particles[Index].Weight += Sign * Weights[I] / Sum;
		}
	}
	SurfaceContactConstraint::SurfaceContactConstraint(const SurfaceContactParticle* A, int CountA, const SurfaceContactParticle* B, int CountB,
		const SurfaceContact& Contact, double Thickness, const SurfaceContactMaterial& Material)
		: m_Thickness(Thickness), m_Material(Material)
	{
		m_Normal = ToDouble(Contact.Normal); const double Length = std::sqrt(m_Normal.SquareLength());
		if (!Material.IsValid() || !Nonnegative(Thickness) || !Finite(m_Normal) || !(Length > 0)
			|| !Finite(ToDouble(Contact.PointA)) || !Finite(ToDouble(Contact.PointB)) || Contact.Conservative)
			throw std::invalid_argument("Invalid surface contact constraint");
		m_Normal = m_Normal / Length; m_Reference = ToDouble(Contact.PointA) - ToDouble(Contact.PointB);
		Add(A, CountA, Contact.WeightsA, 1); Add(B, CountB, Contact.WeightsB, -1);
		for (int I = 0; I < m_Count; ++I) { const auto& P = m_Particles[I]; m_InverseMass += P.InverseMass * P.Weight * P.Weight; }
	}
	void SurfaceContactConstraint::Refresh(const SurfaceContactParticle* A, int CountA, const SurfaceContactParticle* B, int CountB,
		const SurfaceContact& Contact, double Thickness)
	{
		/* Nonlinear discrete contacts re-evaluate their closest feature after other
		   constraints bend the surface. Preserve the scalar XPBD multiplier; resetting
		   it here would make compliant contacts stiffer on every iteration. The
		   friction vector is parallel-transported by the shortest rotation from the
		   old normal to the new one. For a tangent t this is
		     t' = t - (n_old+n_new) * dot(t,n_new)/(1+dot(n_old,n_new)).
		   An opposed normal represents a different side and drops the old cache.
		   The caller supplies the new witness's reference position at substep start
		   (or impact). Refresh is for the same feature within one physical substep;
		   frozen CCD constraints need not call it. Validate before replacing state. */
		SurfaceContactConstraint Next(A, CountA, B, CountB, Contact, Thickness, m_Material);
		const double Alignment = m_Normal.Dot(Next.m_Normal);
		if (Alignment > 0)
		{
			Next.m_NormalMultiplier = m_NormalMultiplier;
			Next.m_TangentMultiplier = m_TangentMultiplier - (m_Normal + Next.m_Normal)
				* (m_TangentMultiplier.Dot(Next.m_Normal) / (1 + Alignment));
			Next.m_Sliding = m_Sliding;
		}
		*this = Next;
	}
	SurfaceContactConstraint::DVector SurfaceContactConstraint::Separation() const
	{
		DVector Separation(0);
		for (int I = 0; I < m_Count; ++I)
		{
			const auto& P = m_Particles[I]; const DVector Position = ToDouble(*P.Position);
			if (!Finite(Position)) throw std::invalid_argument("Non-finite surface contact position");
			Separation += Position * P.Weight;
		}
		return Separation;
	}
	double SurfaceContactConstraint::GetNormalGap() const { return m_Normal.Dot(Separation()) - m_Thickness; }
	bool SurfaceContactConstraint::ProjectSeparation()
	{
		/* Hard-contact feasibility pass after the coupled friction solve. Only
		   remove negative gap; do not release a previous correction or replay the
		   tangential reference. That reference can conflict with another contact's
		   normal when surfaces curve. Keep the friction already applied, and add
		   the normal multiplier so its Coulomb budget cannot shrink. Equal/opposite
		   barycentric corrections preserve the free system's center of mass.
		   This explicitly overrides compliance and is only used for hard barriers. */
		const double Error = GetNormalGap();
		if (!(m_InverseMass > 0) || Error >= 0) return false;
		const double Correction = -Error / m_InverseMass;
		if (!std::isfinite(Correction + m_NormalMultiplier)) throw std::overflow_error("Surface separation multiplier overflow");
		std::array<Vector3, 6> Positions;
		for (int I = 0; I < m_Count; ++I)
		{
			const auto& P = m_Particles[I];
			const DVector Position = ToDouble(*P.Position) + m_Normal * (Correction * P.InverseMass * P.Weight);
			Positions[I] = Vector3((float)Position.x, (float)Position.y, (float)Position.z);
			if (!Finite(ToDouble(Positions[I]))) throw std::overflow_error("Surface separation projection overflow");
		}
		for (int I = 0; I < m_Count; ++I) *m_Particles[I].Position = Positions[I];
		m_NormalMultiplier += Correction;
		return true;
	}
	bool SurfaceContactConstraint::Solve(double Dt)
	{
		if (!std::isfinite(Dt) || Dt <= 0) throw std::invalid_argument("Invalid surface contact time step");
		const DVector Separation = this->Separation();
		if (!(m_InverseMass > 0)) return false;
		const double Error = m_Normal.Dot(Separation) - m_Thickness, DtSquared = (double)Dt * Dt;
		// Multiply by dt^2, then scale. Neither compliance/dt^2 nor alpha*lambda
		// needs to be representable for a valid finite compliant response.
		const double Weight = m_InverseMass * DtSquared, Scale = std::max(Weight, m_Material.Compliance);
		const double Compliance = m_Material.Compliance / Scale, Denominator = Weight / Scale + Compliance;
		const double Normal = std::max(0.0, m_NormalMultiplier - (Error * DtSquared / Scale + Compliance * m_NormalMultiplier) / Denominator);
		const DVector Displacement = Separation - m_Reference;
		const DVector Slip = Displacement - m_Normal * m_Normal.Dot(Displacement);
		DVector Tangent = m_TangentMultiplier - Slip / m_InverseMass;
		const double TangentLength = std::hypot(std::hypot(Tangent.x, Tangent.y), Tangent.z);
		const bool Sliding = TangentLength > m_Material.StaticFriction * Normal;
		if (Sliding && TangentLength > 0) Tangent = Tangent * (m_Material.DynamicFriction * Normal / TangentLength);
		const DVector Delta = m_Normal * (Normal - m_NormalMultiplier) + (Tangent - m_TangentMultiplier);
		if (!std::isfinite(Normal) || !Finite(Tangent) || !Finite(Delta)) throw std::overflow_error("Surface contact multiplier overflow");
		std::array<Vector3, 6> Positions;
		for (int I = 0; I < m_Count; ++I)
		{
			const auto& P = m_Particles[I]; const DVector Position = ToDouble(*P.Position) + Delta * (P.InverseMass * P.Weight);
			Positions[I] = Vector3((float)Position.x, (float)Position.y, (float)Position.z);
			if (!Finite(ToDouble(Positions[I]))) throw std::overflow_error("Surface contact correction overflow");
		}
		for (int I = 0; I < m_Count; ++I) *m_Particles[I].Position = Positions[I];
		m_NormalMultiplier = Normal; m_TangentMultiplier = Tangent; m_Sliding = Sliding;
		return Delta.SquareLength() > 0;
	}
}
