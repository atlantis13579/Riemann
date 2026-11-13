#include "RigidPointResponse.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace Riemann
{
	/* A rigid anchor is p=x+R*r, with r measured from the center of mass. Its
	   point response is K=invMass*I-[Rr]x*invInertiaWorld*[Rr]x. The rotational
	   term is a full symmetric matrix: replacing it with one scalar loses the
	   coupling between an off-center force and the resulting angular motion.

	   This layer consumes predicted poses and returns position-level impulses.
	   It does not integrate free motion or modify velocities. A world adapter
	   must account for each linear/angular multiplier once, divided by dt for
	   momentum or by dt^2 for force/torque. Multiple anchors share the same pose
	   pointers and refresh their lever arms on every projection. The adapter
	   decides when to commit these poses relative to rigid contacts and CCD.

	   Symmetric Jacobi diagonalization supplies a scale-relative pseudoinverse
	   for locked directions. Unreachable hard-constraint error remains visible;
	   no diagonal bias fabricates motion along a zero-response direction.
	   Local inertia is validated once and tiny negative roundoff eigenvalues
	   are clamped within the explicit inertia tolerance. All arithmetic before
	   committing a float pose uses double precision.
	   Reference: Muller et al., "Detailed Rigid Body Simulation with Extended
	   Position Based Dynamics" (2020), https://matthias-research.github.io/pages/
	   publications/PBDBodies.pdf. */
	namespace
	{
		using DVector = RigidPointResponse::DVector;
		using Matrix = ConstraintMassMatrix;
		constexpr int Dimensions = 3;
		constexpr double Pi = 3.14159265358979323846;
		DVector ToDouble(const Vector3& P) { return DVector(P.x, P.y, P.z); }
		bool Finite(const DVector& P) { return std::isfinite(P.x) && std::isfinite(P.y) && std::isfinite(P.z); }
		double Length(const DVector& P) { return std::hypot(P.x, std::hypot(P.y, P.z)); }
		bool Positive(double V) { return std::isfinite(V) && V > 0; }
		Matrix Identity()
		{
			Matrix M; for (int I = 0; I < Dimensions; ++I) M[I][I] = 1; return M;
		}
		Matrix Rotation(const Quaternion& Q, const RigidResponseSettings& Settings)
		{
			const double LengthSquared = (double)Q.x * Q.x + (double)Q.y * Q.y + (double)Q.z * Q.z + (double)Q.w * Q.w;
			if (!std::isfinite(LengthSquared) || std::abs(LengthSquared - 1) > Settings.QuaternionTolerance)
				throw std::invalid_argument("Rigid constraint requires a finite unit quaternion");
			const double InverseLength = 1 / std::sqrt(LengthSquared);
			const DVector V(Q.x * InverseLength, Q.y * InverseLength, Q.z * InverseLength); const double W = Q.w * InverseLength;
			Matrix R;
			for (int Column = 0; Column < Dimensions; ++Column)
			{
				DVector Axis(0); Axis[Column] = 1;
				const auto Cross = V.Cross(Axis) * 2; const auto Rotated = Axis + Cross * W + V.Cross(Cross);
				for (int Row = 0; Row < Dimensions; ++Row) R[Row][Column] = Rotated[Row];
			}
			return R;
		}
		void Diagonalize(const Matrix& Input, const RigidResponseSettings& Settings, DVector& Values, Matrix& Vectors)
		{
			const double Scale = Input.MaximumMagnitude(); Vectors = Identity(); Values = DVector(0);
			if (Scale == 0) return;
			Matrix A;
			for (int I = 0; I < Dimensions; ++I) for (int J = 0; J < Dimensions; ++J) A[I][J] = Input[I][J] / Scale;
			for (int I = 0; I < Dimensions; ++I) for (int J = I + 1; J < Dimensions; ++J)
			{
				if (std::abs(A[I][J] - A[J][I]) > Settings.InertiaTolerance) throw std::invalid_argument("Asymmetric constraint response");
				A[I][J] = A[J][I] = (A[I][J] + A[J][I]) / 2;
			}
			for (int Iteration = 0; ; ++Iteration)
			{
				int P = 0, Q = 1; double Largest = std::abs(A[P][Q]);
				for (int I = 0; I < Dimensions; ++I) for (int J = I + 1; J < Dimensions; ++J)
					if (std::abs(A[I][J]) > Largest) { Largest = std::abs(A[I][J]); P = I; Q = J; }
				if (Largest <= Settings.RelativeTolerance) break;
				if (Iteration == Settings.MaximumDiagonalIterations) throw std::runtime_error("Rigid constraint diagonalization did not converge");
				const double Angle = std::atan2(2 * A[P][Q], A[Q][Q] - A[P][P]) / 2;
				const double C = std::cos(Angle), S = std::sin(Angle), APP = A[P][P], AQQ = A[Q][Q], APQ = A[P][Q];
				for (int K = 0; K < Dimensions; ++K) if (K != P && K != Q)
				{
					const double AKP = A[K][P], AKQ = A[K][Q];
					A[K][P] = A[P][K] = C * AKP - S * AKQ; A[K][Q] = A[Q][K] = S * AKP + C * AKQ;
				}
				A[P][P] = C * C * APP - 2 * S * C * APQ + S * S * AQQ;
				A[Q][Q] = S * S * APP + 2 * S * C * APQ + C * C * AQQ; A[P][Q] = A[Q][P] = 0;
				for (int K = 0; K < Dimensions; ++K)
				{
					const double VKP = Vectors[K][P], VKQ = Vectors[K][Q];
					Vectors[K][P] = C * VKP - S * VKQ; Vectors[K][Q] = S * VKP + C * VKQ;
				}
			}
			for (int I = 0; I < Dimensions; ++I) Values[I] = A[I][I] * Scale;
		}
	}
	ConstraintMassMatrix::DVector ConstraintMassMatrix::operator*(const DVector& Vector) const
	{
		DVector Result(0);
		for (int I = 0; I < Dimensions; ++I) for (int J = 0; J < Dimensions; ++J) Result[I] += Values[I][J] * Vector[J];
		return Result;
	}
	double ConstraintMassMatrix::MaximumMagnitude() const
	{
		double Scale = 0;
		for (int I = 0; I < Dimensions; ++I) for (int J = 0; J < Dimensions; ++J)
		{
			if (!std::isfinite(Values[I][J])) throw std::invalid_argument("Non-finite constraint mass matrix");
			Scale = std::max(Scale, std::abs(Values[I][J]));
		}
		return Scale;
	}
	bool RigidResponseSettings::IsValid() const
	{
		return Positive(RelativeTolerance) && RelativeTolerance < 1 && Positive(InertiaTolerance) && InertiaTolerance < 1
			&& Positive(QuaternionTolerance) && QuaternionTolerance < 1 && Positive(MaximumRotation) && MaximumRotation <= Pi
			&& MaximumDiagonalIterations > 0;
	}
	RigidPointResponse::RigidPointResponse(const RigidConstraintBody& Body, const Vector3& LocalArm, const RigidResponseSettings& Settings)
		: m_Body(Body), m_LocalArm(LocalArm), m_Settings(Settings)
	{
		if (!Settings.IsValid() || !Body.Center || !Body.Rotation || !Finite(ToDouble(*Body.Center)) || !Finite(ToDouble(LocalArm))
			|| !std::isfinite(Body.InverseMass) || Body.InverseMass < 0) throw std::invalid_argument("Invalid rigid constraint body");
		Rotation(*Body.Rotation, Settings);
		Matrix Local;
		for (int I = 0; I < Dimensions; ++I) for (int J = 0; J < Dimensions; ++J) Local[I][J] = Body.LocalInverseInertia[I][J];
		const double Scale = Local.MaximumMagnitude();
		for (int I = 0; I < Dimensions; ++I) for (int J = I + 1; J < Dimensions; ++J)
		{
			if (std::abs(Local[I][J] - Local[J][I]) > Settings.InertiaTolerance * Scale) throw std::invalid_argument("Asymmetric rigid inverse inertia");
			Local[I][J] = Local[J][I] = (Local[I][J] + Local[J][I]) / 2;
		}
		DVector Values; Matrix Vectors; Diagonalize(Local, Settings, Values, Vectors);
		for (int K = 0; K < Dimensions; ++K)
		{
			if (Values[K] < -Settings.InertiaTolerance * Scale) throw std::invalid_argument("Indefinite rigid inverse inertia");
			for (int I = 0; I < Dimensions; ++I) for (int J = 0; J < Dimensions; ++J)
				m_LocalInverseInertia[I][J] += Vectors[I][K] * std::max(0.0, Values[K]) * Vectors[J][K];
		}
	}
	RigidPointResponse::Evaluation RigidPointResponse::Evaluate() const
	{
		if (!m_Body.Center || !m_Body.Rotation || !Finite(ToDouble(*m_Body.Center))) throw std::invalid_argument("Invalid rigid constraint pose");
		const auto R = Rotation(*m_Body.Rotation, m_Settings); Evaluation State;
		State.Arm = R * ToDouble(m_LocalArm); State.Point = ToDouble(*m_Body.Center) + State.Arm;
		for (int I = 0; I < Dimensions; ++I) for (int J = 0; J < Dimensions; ++J)
			for (int K = 0; K < Dimensions; ++K) for (int L = 0; L < Dimensions; ++L)
				State.InverseInertia[I][J] += R[I][K] * m_LocalInverseInertia[K][L] * R[J][L];
		for (int J = 0; J < Dimensions; ++J)
		{
			DVector Axis(0); Axis[J] = 1;
			const auto Response = Axis * m_Body.InverseMass + (State.InverseInertia * State.Arm.Cross(Axis)).Cross(State.Arm);
			for (int I = 0; I < Dimensions; ++I) State.InversePointMass[I][J] = Response[I];
		}
		return State;
	}
	double RigidPointResponse::StepScale(const Evaluation& State, const DVector& PositionImpulse) const
	{
		const double Angle = Length(State.InverseInertia * State.Arm.Cross(PositionImpulse));
		if (!std::isfinite(Angle)) throw std::overflow_error("Rigid constraint angular correction overflow");
		return Angle > m_Settings.MaximumRotation ? m_Settings.MaximumRotation / Angle : 1;
	}
	RigidPointResponse::Correction RigidPointResponse::Prepare(const Evaluation& State, const DVector& PositionImpulse) const
	{
		if (!Finite(PositionImpulse)) throw std::overflow_error("Non-finite rigid constraint correction");
		Correction Next; Next.AngularMultiplier = State.Arm.Cross(PositionImpulse);
		const auto Angular = State.InverseInertia * Next.AngularMultiplier; const double Angle = Length(Angular);
		const auto Center = ToDouble(*m_Body.Center) + PositionImpulse * m_Body.InverseMass;
		Next.Center = Vector3((float)Center.x, (float)Center.y, (float)Center.z); Next.Rotation = *m_Body.Rotation;
		if (!Finite(ToDouble(Next.Center)) || !Finite(Next.AngularMultiplier) || !std::isfinite(Angle)) throw std::overflow_error("Rigid constraint correction overflow");
		if (Angle > m_Settings.MaximumRotation * (1 + m_Settings.RelativeTolerance)) throw std::invalid_argument("Scale the complete constraint correction before applying it");
		if (Angle > 0)
		{
			const double Scale = std::sin(Angle / 2) / Angle;
			const Quaternion Delta((float)(Angular.x * Scale), (float)(Angular.y * Scale), (float)(Angular.z * Scale), (float)std::cos(Angle / 2));
			Next.Rotation = (Delta * Next.Rotation).Unit();
		}
		Rotation(Next.Rotation, m_Settings); return Next;
	}
	void RigidPointResponse::Commit(const Correction& Next) const { *m_Body.Center = Next.Center; *m_Body.Rotation = Next.Rotation; }
	RigidPointResponse::DVector RigidPointResponse::SolveMass(const ConstraintMassMatrix& Matrix, const DVector& Right) const
	{
		if (!Finite(Right)) throw std::invalid_argument("Non-finite rigid constraint right-hand side");
		DVector Values; ConstraintMassMatrix Vectors; Diagonalize(Matrix, m_Settings, Values, Vectors);
		const double Scale = std::max(Values.x, std::max(Values.y, Values.z)); DVector Result(0);
		for (int K = 0; K < Dimensions; ++K)
		{
			if (Values[K] < -m_Settings.InertiaTolerance * Scale) throw std::invalid_argument("Indefinite constraint response");
			if (Values[K] <= Scale * m_Settings.RelativeTolerance) continue;
			const DVector Axis(Vectors[0][K], Vectors[1][K], Vectors[2][K]); Result += Axis * (Axis.Dot(Right) / Values[K]);
		}
		if (!Finite(Result)) throw std::overflow_error("Rigid constraint solve overflow");
		return Result;
	}
}
