#include "BarycentricAttachment.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace Riemann
{
	/* A material point is x = sum(w_i x_i), with nonnegative barycentric weights
	   summing to one. Joining two such points gives the vector constraint C=xA-xB.
	   Its scalar particle response is W=sum(invMass_i beta_i^2), where beta is the
	   signed, COMBINED weight of a unique particle. Combining shared particles
	   before squaring is essential for attachments within the same mesh.

	   XPBD solves (W + compliance/dt^2) deltaLambda = -C-alpha*lambda.
	   Lambda is a vector and its force on endpoint A is lambda/dt^2. The weights
	   distribute equal/opposite forces to both meshes without selecting an
	   arbitrary nearest vertex. A zero-mass point supplies a prescribed anchor.
	   Construct once per physical substep, then retain lambda across iterations.
	   Indices live in the asset/runtime adapter; this kernel borrows position
	   pointers only during solving, so adding particles between steps is safe.

	   This service is shared by soft bodies and future cloth/strand attachments.
	   Independent attachment components can run concurrently; rows sharing a
	   particle must be ordered or colored. Rigid endpoints use the full point
	   response matrix from RigidPointResponse. Angular corrections are limited
	   by scaling the ENTIRE multiplier increment, including particle and rigid
	   translation, so relaxation cannot break equal/opposite linear response.
	   The accumulated rigid angular multiplier uses each iteration's current
	   lever arm; crossing only the final arm with the final lambda is incorrect.
	   This kernel projects predicted states; the world adapter still owns rigid
	   velocity feedback and must not integrate a corrected pose a second time.
	   Reference: Macklin et al., "XPBD: Position-Based Simulation of Compliant
	   Constrained Dynamics" (2016), https://matthias-research.github.io/pages/
	   publications/XPBD.pdf. */
	namespace
	{
		using DVector = BarycentricAttachmentConstraint::DVector;
		constexpr double WeightSumTolerance = 1e-5;
		DVector ToDouble(const Vector3& V) { return DVector(V.x, V.y, V.z); }
		bool Finite(const DVector& V) { return std::isfinite(V.x) && std::isfinite(V.y) && std::isfinite(V.z); }
		bool Nonnegative(double V) { return std::isfinite(V) && V >= 0; }
		double WeightSum(const double* Weights, int Count)
		{
			if (!Weights || Count < 1 || Count > BarycentricPoint::MaximumVertices) return 0;
			double Sum = 0;
			for (int I = 0; I < Count; ++I) { if (!Nonnegative(Weights[I])) return 0; Sum += Weights[I]; }
			return std::abs(Sum - 1) <= WeightSumTolerance ? Sum : 0;
		}
		double StepSquared(float Dt)
		{
			if (!std::isfinite(Dt) || Dt <= 0) throw std::invalid_argument("Invalid attachment time step");
			return (double)Dt * Dt;
		}
	}
	BarycentricPoint BarycentricPoint::Vertex(int Index)
	{
		BarycentricPoint Point; Point.Count = 1; Point.Indices[0] = Index; Point.Weights[0] = 1; return Point;
	}
	BarycentricPoint BarycentricPoint::Triangle(const Index3& Indices, const Vector3& Weights)
	{
		BarycentricPoint Point; Point.Count = 3; Point.Indices = { Indices.a, Indices.b, Indices.c, -1 };
		Point.Weights = { Weights.x, Weights.y, Weights.z, 0 }; return Point;
	}
	BarycentricPoint BarycentricPoint::Tetrahedron(const std::array<int, MaximumVertices>& Indices,
		const std::array<double, MaximumVertices>& Weights)
	{
		BarycentricPoint Point; Point.Count = MaximumVertices; Point.Indices = Indices; Point.Weights = Weights; return Point;
	}
	bool BarycentricPoint::IsValid(size_t ParticleCount) const
	{
		if (!(WeightSum(Weights.data(), Count) > 0)) return false;
		for (int I = 0; I < Count; ++I) if (Indices[I] < 0 || (size_t)Indices[I] >= ParticleCount) return false;
		for (int I = Count; I < MaximumVertices; ++I) if (Weights[I] != 0) return false;
		return true;
	}
	void BarycentricAttachmentConstraint::Add(const SurfaceContactParticle* Particles, const double* Weights, int Count, double Sign)
	{
		const double Sum = WeightSum(Weights, Count);
		if (!Particles || !(Sum > 0)) throw std::invalid_argument("Invalid attachment particles or barycentric weights");
		for (int I = 0; I < Count; ++I)
		{
			const auto& P = Particles[I];
			if (!P.Position || !Finite(ToDouble(*P.Position)) || !Nonnegative(P.InverseMass))
				throw std::invalid_argument("Invalid attachment particle state");
			int Index = 0; while (Index < m_Count && m_Particles[Index].Position != P.Position) ++Index;
			if (Index == m_Count) { ++m_Count; m_Particles[Index].Position = P.Position; m_Particles[Index].InverseMass = P.InverseMass; }
			else if (m_Particles[Index].InverseMass != P.InverseMass) throw std::invalid_argument("Conflicting attachment inverse masses");
			m_Particles[Index].Weight += Sign * Weights[I] / Sum;
		}
	}
	BarycentricAttachmentConstraint::BarycentricAttachmentConstraint(const SurfaceContactParticle* A, const double* WeightsA, int CountA,
		const SurfaceContactParticle* B, const double* WeightsB, int CountB, double Compliance) : m_Compliance(Compliance)
	{
		if (!Nonnegative(Compliance)) throw std::invalid_argument("Invalid attachment compliance");
		Add(A, WeightsA, CountA, 1); Add(B, WeightsB, CountB, -1);
		for (int I = 0; I < m_Count; ++I) { const auto& P = m_Particles[I]; m_InverseMass += P.InverseMass * P.Weight * P.Weight; }
	}
	BarycentricAttachmentConstraint::BarycentricAttachmentConstraint(const SurfaceContactParticle* A, const double* WeightsA, int CountA,
		const RigidConstraintBody& Body, const Vector3& LocalArm, double Compliance, const RigidResponseSettings& Settings)
		: m_Compliance(Compliance), m_HasRigid(true), m_Rigid(Body, LocalArm, Settings)
	{
		if (!Nonnegative(Compliance)) throw std::invalid_argument("Invalid attachment compliance");
		Add(A, WeightsA, CountA, 1);
		for (int I = 0; I < m_Count; ++I)
		{
			const auto& P = m_Particles[I];
			if (P.Position == m_Rigid.GetCenterAddress()) throw std::invalid_argument("Attachment particle aliases the rigid center");
			m_InverseMass += P.InverseMass * P.Weight * P.Weight;
		}
	}
	double BarycentricAttachmentConstraint::GetEffectiveInverseMass() const
	{
		if (m_HasRigid) throw std::logic_error("Rigid attachment response is a matrix");
		return m_InverseMass;
	}
	ConstraintMassMatrix BarycentricAttachmentConstraint::GetEffectiveInverseMassMatrix() const
	{
		ConstraintMassMatrix Result;
		if (m_HasRigid) Result = m_Rigid.Evaluate().InversePointMass;
		for (int I = 0; I < 3; ++I) Result[I][I] += m_InverseMass;
		return Result;
	}
	BarycentricAttachmentConstraint::DVector BarycentricAttachmentConstraint::GetError() const
	{
		DVector Error(0);
		for (int I = 0; I < m_Count; ++I)
		{
			const auto& P = m_Particles[I]; const auto Position = ToDouble(*P.Position);
			if (!Finite(Position)) throw std::invalid_argument("Non-finite attachment position");
			Error += Position * P.Weight;
		}
		return m_HasRigid ? Error - m_Rigid.Evaluate().Point : Error;
	}
	BarycentricAttachmentConstraint::DVector BarycentricAttachmentConstraint::GetForce(float Dt) const
	{
		const auto Force = m_Multiplier / StepSquared(Dt);
		if (!Finite(Force)) throw std::overflow_error("Attachment force overflow");
		return Force;
	}
	BarycentricAttachmentConstraint::DVector BarycentricAttachmentConstraint::GetRigidTorque(float Dt) const
	{
		const auto Torque = m_RigidAngularMultiplier / StepSquared(Dt);
		if (!Finite(Torque)) throw std::overflow_error("Attachment torque overflow");
		return Torque;
	}
	bool BarycentricAttachmentConstraint::Solve(float Dt)
	{
		const double DtSquared = StepSquared(Dt); const auto Error = GetError();
		DVector Delta(0); RigidPointResponse::Evaluation RigidState;
		if (m_HasRigid)
		{
			RigidState = m_Rigid.Evaluate(); auto Mass = RigidState.InversePointMass;
			for (int I = 0; I < 3; ++I) Mass[I][I] += m_InverseMass;
			const double Scale = std::max(Mass.MaximumMagnitude() * DtSquared, m_Compliance);
			if (!(Scale > 0)) return false;
			for (int I = 0; I < 3; ++I) for (int J = 0; J < 3; ++J) Mass[I][J] *= DtSquared / Scale;
			const double Alpha = m_Compliance / Scale;
			for (int I = 0; I < 3; ++I) Mass[I][I] += Alpha;
			Delta = m_Rigid.SolveMass(Mass, -(Error * (DtSquared / Scale) + m_Multiplier * Alpha));
			Delta = Delta * m_Rigid.StepScale(RigidState, -Delta);
		}
		else
		{
			// A compliant link between prescribed points still has a known spring
			// reaction. A hard link with no movable degree of freedom is unsolvable.
			if (!(m_InverseMass > 0) && !(m_Compliance > 0)) return false;
			// Scale the equation instead of forming compliance/dt^2, which can overflow.
			const double Scale = std::max(m_InverseMass * DtSquared, m_Compliance);
			const double Alpha = m_Compliance / Scale, Denominator = m_InverseMass * DtSquared / Scale + Alpha;
			Delta = -(Error * (DtSquared / Scale) + m_Multiplier * Alpha) / Denominator;
		}
		const auto Multiplier = m_Multiplier + Delta;
		if (!Finite(Multiplier)) throw std::overflow_error("Attachment multiplier overflow");
		std::array<Vector3, BarycentricPoint::MaximumVertices * 2> Next;
		for (int I = 0; I < m_Count; ++I)
		{
			const auto& P = m_Particles[I]; const auto Position = ToDouble(*P.Position) + Delta * (P.InverseMass * P.Weight);
			Next[I] = Vector3((float)Position.x, (float)Position.y, (float)Position.z);
			if (!Finite(ToDouble(Next[I]))) throw std::overflow_error("Attachment position overflow");
		}
		RigidPointResponse::Correction RigidNext; DVector AngularMultiplier = m_RigidAngularMultiplier;
		if (m_HasRigid)
		{
			RigidNext = m_Rigid.Prepare(RigidState, -Delta); AngularMultiplier += RigidNext.AngularMultiplier;
			if (!Finite(AngularMultiplier)) throw std::overflow_error("Attachment angular multiplier overflow");
		}
		for (int I = 0; I < m_Count; ++I) *m_Particles[I].Position = Next[I];
		if (m_HasRigid) { m_Rigid.Commit(RigidNext); m_RigidAngularMultiplier = AngularMultiplier; }
		m_Multiplier = Multiplier; return Delta.SquareLength() > 0;
	}
}
