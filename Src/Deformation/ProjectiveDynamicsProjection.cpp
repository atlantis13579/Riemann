#include "ProjectiveDynamicsProjection.h"
#include <algorithm>
#include <cmath>
#include <set>
#include <stdexcept>

namespace Riemann
{
	/* Projective material stage
	   -------------------------
	   This is the rotation local/global part of fixed-corotated elasticity:
	       Psi(F) = mu ||F - R(F)||^2 + lambda/2 (det(F)-1)^2.
	   The nearest proper rotation is the local projection. With rotations fixed,
	   the global Hessian is M/h^2 + sum_e 2 mu_e V_e G_e^T G_e. Matrix-free Jacobi
	   preconditioned CG needs O(nodes + elements) storage, including exact zero
	   increments at pinned nodes. Rest gradients, not unscaled tetrahedron edges,
	   make stiffness independent of element shape and mesh refinement. The bulk
	   term is solved by the shared XPBD material stage; the linearized law has
	   exactly the supplied Young's modulus and Poisson ratio.

	   The world owns prediction and velocity reconstruction. This stage stores its
	   OWN accepted displacement p. At each iteration the external predictor is
	   b = x-p, so the global residual is -M p/h^2 - grad(E_rotation). Thus contact,
	   ground, bulk and attachment corrections stay in b. Re-solving from the
	   original free-flight predictor would erase their reaction and cause drift.
	   This is block stationarity iteration, with each material/contact block
	   retaining its own primal displacement. Float rounding is charged to p only
	   after staged positions are committed. Shared line search preserves feasible
	   element determinants and bounds relative corrections.

	   Reference: Bouaziz et al., "Projective Dynamics: Fusing Constraint Projections
	   for Fast Simulation" (SIGGRAPH 2014), local/global optimization:
	   https://www.projectivedynamics.org/Projective_Dynamics/index.html
	   The older ProjectiveDynamicsSolver retains its standalone fixed-clock API;
	   coupled world stepping uses this projection-only implementation instead. */
	namespace
	{
		using DVector = DeformationGradient::DVector;
		DVector Double(const Vector3& P) { return DVector(P.x,P.y,P.z); }
		bool Finite(const DVector& P) { return std::isfinite(P.x) && std::isfinite(P.y) && std::isfinite(P.z); }
		double Dot(const std::vector<DVector>& A, const std::vector<DVector>& B)
		{ double Value = 0; for (size_t I = 0; I < A.size(); ++I) Value += A[I].Dot(B[I]); return Value; }
	}
	bool ProjectiveDynamicsSettings::IsValid() const
	{
		return MaximumLinearIterations > 0 && std::isfinite(RelativeLinearTolerance) && RelativeLinearTolerance > 0 && RelativeLinearTolerance < 1;
	}
	void ProjectiveDynamicsProjection::Begin(const std::vector<ProjectiveDynamicsParticle>& Particles,
		const std::vector<ProjectiveDynamicsElement>& Elements, double Dt, const ProjectiveDynamicsSettings& Settings, const TetrahedralSolverSettings& Safety)
	{
		End();
		if (!Settings.IsValid() || !Safety.IsValid() || !std::isfinite(Dt) || Dt <= 0) throw std::invalid_argument("Invalid projective material step");
		std::set<const Vector3*> Seen;
		for (const auto& P : Particles)
			if (!P.Position || !Finite(Double(*P.Position)) || !std::isfinite(P.InverseMass) || P.InverseMass < 0 || !Seen.insert(P.Position).second)
				throw std::invalid_argument("Invalid projective particle");
		for (const auto& E : Elements)
		{
			auto Sorted = E.Particles; std::sort(Sorted.begin(), Sorted.end());
			if (Sorted[0] < 0 || (size_t)Sorted[3] >= Particles.size() || std::adjacent_find(Sorted.begin(),Sorted.end()) != Sorted.end()
				|| !std::isfinite(E.ShearModulus) || E.ShearModulus <= 0 || !std::isfinite(E.Rest.Volume) || E.Rest.Volume <= 0
				|| !std::isfinite(E.Rest.CharacteristicLength) || E.Rest.CharacteristicLength <= 0)
				throw std::invalid_argument("Invalid projective element");
			for (const auto& G : E.Rest.Gradients) if (!Finite(G)) throw std::invalid_argument("Invalid projective rest gradients");
		}
		m_Settings = Settings; m_Safety = Safety; m_Stats = {}; m_Particles = Particles; m_Elements = Elements;
		const size_t Count = Particles.size(); m_InertialWeights.resize(Count); m_Diagonal.resize(Count);
		m_Corrections.assign(Count,DVector(0)); m_Residual.resize(Count); m_Direction.resize(Count); m_Preconditioned.resize(Count);
		m_Product.resize(Count); m_Delta.resize(Count); m_Next.resize(Count);
		for (size_t I = 0; I < Count; ++I)
		{
			m_Diagonal[I] = m_InertialWeights[I] = Particles[I].InverseMass > 0 ? 1 / ((double)Particles[I].InverseMass * Dt * Dt) : 0;
			if (!std::isfinite(m_InertialWeights[I]) || (Particles[I].InverseMass > 0 && m_InertialWeights[I] <= 0)) throw std::overflow_error("Projective inertial weight overflow");
		}
		for (const auto& E : Elements) for (int I = 0; I < 4; ++I)
			m_Diagonal[E.Particles[I]] += 2 * E.ShearModulus * E.Rest.Volume * E.Rest.Gradients[I].SquareLength();
		for (double D : m_Diagonal) if (!std::isfinite(D)) throw std::overflow_error("Projective diagonal overflow");
		m_Active = true;
	}
	void ProjectiveDynamicsProjection::Multiply(const std::vector<DVector>& Input, std::vector<DVector>& Output) const
	{
		for (size_t I = 0; I < Input.size(); ++I) Output[I] = Input[I] * m_InertialWeights[I];
		for (const auto& E : m_Elements)
		{
			DeformationGradient F;
			for (int I = 0; I < 4; ++I) for (int Row = 0; Row < 3; ++Row) for (int Col = 0; Col < 3; ++Col)
				F[Row][Col] += Input[E.Particles[I]][Row] * E.Rest.Gradients[I][Col];
			const double Weight = 2 * E.ShearModulus * E.Rest.Volume;
			for (int I = 0; I < 4; ++I) if (m_InertialWeights[E.Particles[I]] > 0) Output[E.Particles[I]] += (F * E.Rest.Gradients[I]) * Weight;
		}
	}
	bool ProjectiveDynamicsProjection::Project()
	{
		if (!m_Active) throw std::logic_error("Begin a projective step before projection");
		m_Stats = {};
		for (size_t I = 0; I < m_Particles.size(); ++I)
		{
			if (!Finite(Double(*m_Particles[I].Position))) throw std::invalid_argument("Non-finite projective position");
			m_Residual[I] = m_Corrections[I] * -m_InertialWeights[I]; m_Delta[I] = DVector(0);
		}
		for (const auto& E : m_Elements)
		{
			Vector3 Positions[4]; for (int I = 0; I < 4; ++I) Positions[I] = *m_Particles[E.Particles[I]].Position;
			auto Difference = E.Rest.Evaluate(Positions); const auto R = TetrahedralElasticity::ClosestRotation(Difference);
			for (int I = 0; I < 3; ++I) for (int J = 0; J < 3; ++J) Difference[I][J] -= R[I][J];
			const double Weight = 2 * E.ShearModulus * E.Rest.Volume;
			for (int I = 0; I < 4; ++I) if (m_InertialWeights[E.Particles[I]] > 0) m_Residual[E.Particles[I]] -= (Difference * E.Rest.Gradients[I]) * Weight;
		}
		const double Initial = Dot(m_Residual,m_Residual);
		if (!std::isfinite(Initial)) throw std::overflow_error("Projective residual overflow");
		if (!(Initial > 0)) return false;
		for (size_t I = 0; I < m_Particles.size(); ++I) m_Direction[I] = m_Preconditioned[I] = m_InertialWeights[I] > 0 ? m_Residual[I] / m_Diagonal[I] : DVector(0);
		double Product = Dot(m_Residual,m_Preconditioned), Residual = Initial;
		const double Target = Initial * m_Settings.RelativeLinearTolerance * m_Settings.RelativeLinearTolerance;
		while (m_Stats.LinearIterations < m_Settings.MaximumLinearIterations && Residual > Target)
		{
			Multiply(m_Direction,m_Product); const double Curvature = Dot(m_Direction,m_Product);
			if (!std::isfinite(Curvature) || Curvature <= 0 || !std::isfinite(Product) || Product <= 0) throw std::overflow_error("Projective CG lost positive definiteness");
			const double Alpha = Product / Curvature;
			for (size_t I = 0; I < m_Particles.size(); ++I) { m_Delta[I] += m_Direction[I] * Alpha; m_Residual[I] -= m_Product[I] * Alpha; }
			Residual = Dot(m_Residual,m_Residual); ++m_Stats.LinearIterations;
			if (!std::isfinite(Residual)) throw std::overflow_error("Projective CG residual overflow");
			if (Residual <= Target) break;
			for (size_t I = 0; I < m_Particles.size(); ++I) m_Preconditioned[I] = m_InertialWeights[I] > 0 ? m_Residual[I] / m_Diagonal[I] : DVector(0);
			const double Next = Dot(m_Residual,m_Preconditioned), Beta = Next / Product; Product = Next;
			for (size_t I = 0; I < m_Particles.size(); ++I) m_Direction[I] = m_Preconditioned[I] + m_Direction[I] * Beta;
		}
		m_Stats.RelativeLinearResidual = std::sqrt(Residual / Initial); m_Stats.Converged = Residual <= Target;
		double Step = 1;
		for (const auto& E : m_Elements) for (int I : E.Particles)
		{
			const auto& D = m_Delta[I]; const double Length = std::hypot(std::hypot(D.x,D.y),D.z);
			if (Length > 0) Step = std::min(Step,m_Safety.MaximumCorrectionRatio * E.Rest.CharacteristicLength / Length);
		}
		bool Accepted = false;
		for (int Trial = 0; Trial < m_Safety.MaximumLineSearchIterations; ++Trial, Step /= 2)
		{
			for (size_t I = 0; I < m_Particles.size(); ++I)
			{
				const auto P = Double(*m_Particles[I].Position) + m_Delta[I] * Step; m_Next[I] = Vector3((float)P.x,(float)P.y,(float)P.z);
				if (!Finite(Double(m_Next[I]))) throw std::overflow_error("Projective position overflow");
			}
			Accepted = true;
			for (const auto& E : m_Elements)
			{
				Vector3 Before[4], After[4]; for (int I = 0; I < 4; ++I) { Before[I] = *m_Particles[E.Particles[I]].Position; After[I] = m_Next[E.Particles[I]]; }
				if (E.Rest.Evaluate(Before).Determinant() >= m_Safety.MinimumJacobian && E.Rest.Evaluate(After).Determinant() < m_Safety.MinimumJacobian) { Accepted = false; break; }
			}
			if (Accepted) break;
		}
		if (!Accepted) return false;
		bool Changed = false;
		for (size_t I = 0; I < m_Particles.size(); ++I)
		{
			const auto Change = Double(m_Next[I]) - Double(*m_Particles[I].Position); m_Corrections[I] += Change;
			Changed = Changed || Change.SquareLength() > 0; *m_Particles[I].Position = m_Next[I];
		}
		m_Stats.AcceptedStep = Step; return Changed;
	}
	void ProjectiveDynamicsProjection::End() { m_Active = false; m_Particles.clear(); }
}
