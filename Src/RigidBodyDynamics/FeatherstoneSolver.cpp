#include "FeatherstoneSolver.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace Riemann
{
	/* Reduced-coordinate articulated dynamics
	   --------------------------------------
	   Link frames lie at their incoming joints. ParentToJoint places the joint at
	   zero configuration in the parent frame; joint motion follows that transform.
	   Parent indices precede their children. The single root can be fixed, moving
	   on a joint, or floating. Spherical/floating rotations use quaternions, while
	   their velocity coordinates are body-frame angular velocity and, for a floating
	   joint, body-frame linear velocity of its origin. They are not Euler derivatives.

	   Spatial motion is [angular; linear]. X maps parent motion into child coordinates;
	   X transpose maps child wrench to its parent and preserves power. Spatial inertia
	   includes COM offset using the parallel-axis theorem. External wrenches use WORLD
	   AXES ABOUT EACH LINK ORIGIN: force F at point P contributes (P-origin) cross F.
	   GetLinkVelocity uses world axes at that origin; GetMomentum is about world zero.

	   ABA eliminates each joint locally: U = IA*S, D = S^T*U + armature, and the inertia
	   transmitted to the parent is IA - U*inverse(D)*U^T. A reverse pass eliminates
	   joint forces and a forward pass recovers accelerations. D is at most 6 by 6;
	   time/storage are linear in link count. Double precision limits cancellation in
	   long chains and large mass ratios. Cholesky pivots use their own diagonal scale;
	   singular input is reported rather than hidden by artificial diagonal padding.

	   Impulse queries reuse this configuration-dependent factorization with gravity,
	   damping and velocity bias removed. The world contact/limit/drive adapter must
	   use this whole-tree response instead of independent link inverse masses, and
	   must not integrate link poses twice. Existing maximal-coordinate joints remain
	   a selectable route. Robot Jacobians, tendons and mimic constraints can consume
	   the same generalized coordinates. The standalone kernel owns no world bodies.

	   InverseDynamics is a separate recursive Newton-Euler pass for feedforward and
	   validation. Armature is motor inertia in joint coordinates; damping is viscous.
	   Momentum reports the rigid links; lumped joint armature has no spatial rotor
	   representation. It contributes to generalized inertia and kinetic energy only.
	   Velocity stepping plus quaternion-exponential pose stepping is semi-implicit
	   Euler, not an energy-conserving integrator. Fast motion/stiff control needs
	   physical substeps. Inputs use SI units. Joint quaternion increments act locally.

	   References: Featherstone, Rigid Body Dynamics Algorithms (2008), ABA/RNEA;
	   https://royfeatherstone.org/spatial/v2/ (spatial conventions and algorithms).
	*/
	namespace
	{
		constexpr int AngularDimension = 3;
		constexpr int SpatialDimension = SpatialVector::Dimension;
		constexpr double Pi = 3.1415926535897932384626433832795;
		RagdollMatrix QueryMatrix(size_t Rows, size_t Columns)
		{
			const size_t Maximum = (size_t)std::numeric_limits<int>::max();
			if (Rows > Maximum || Columns > Maximum || (Rows && Columns > Maximum / Rows))
				throw std::length_error("Ragdoll query matrix exceeds matrix index range");
			return RagdollMatrix((int)Rows, (int)Columns);
		}
		bool Finite(const Vector3& V) { return std::isfinite(V.x) && std::isfinite(V.y) && std::isfinite(V.z); }
		bool RotationValid(const Quaternion& Q) { return std::isfinite(Q.SquareLength()) && Q.SquareLength() > 0; }
		int Dofs(RagdollReducedJointType Type)
		{
			switch (Type)
			{
			case RagdollReducedJointType::Fixed: return 0;
			case RagdollReducedJointType::Revolute: case RagdollReducedJointType::Prismatic: return 1;
			case RagdollReducedJointType::Spherical: return AngularDimension;
			case RagdollReducedJointType::Floating: return SpatialDimension;
			default: throw std::invalid_argument("Unknown reduced ragdoll joint type");
			}
		}
		SpatialVector MotionCross(const SpatialVector& V, const SpatialVector& W)
		{
			SpatialVector Out;
			for (int I = 0; I < AngularDimension; ++I)
			{
				const int J = (I + 1) % AngularDimension, K = (I + 2) % AngularDimension;
				Out[I] = V[J] * W[K] - V[K] * W[J];
				Out[I + AngularDimension] = V[J] * W[K + AngularDimension] - V[K] * W[J + AngularDimension]
					+ V[J + AngularDimension] * W[K] - V[K + AngularDimension] * W[J];
			}
			return Out;
		}
		SpatialVector ForceCross(const SpatialVector& V, const SpatialVector& F)
		{
			SpatialVector Out;
			for (int I = 0; I < AngularDimension; ++I)
			{
				const int J = (I + 1) % AngularDimension, K = (I + 2) % AngularDimension;
				Out[I] = V[J] * F[K] - V[K] * F[J] + V[J + AngularDimension] * F[K + AngularDimension] - V[K + AngularDimension] * F[J + AngularDimension];
				Out[I + AngularDimension] = V[J] * F[K + AngularDimension] - V[K] * F[J + AngularDimension];
			}
			return Out;
		}
		SpatialVector Rotate(const Quaternion& Q, const SpatialVector& V)
		{
			const Matrix3 R = Q.ToRotationMatrix3(); SpatialVector Out;
			for (int I = 0; I < AngularDimension; ++I) for (int J = 0; J < AngularDimension; ++J)
			{
				Out[I] += R[I][J] * V[J];
				Out[I + AngularDimension] += R[I][J] * V[J + AngularDimension];
			}
			return Out;
		}
		SpatialMatrix MotionTransform(const Transform& ParentToChild)
		{
			SpatialMatrix X;
			for (int Column = 0; Column < SpatialDimension; ++Column)
			{
				SpatialVector Basis; Basis[Column] = 1;
				const Vector3 OffsetVelocity = Basis.GetAngular().Cross(ParentToChild.pos);
				for (int I = 0; I < AngularDimension; ++I) Basis[I + AngularDimension] += OffsetVelocity[I];
				const auto Transformed = Rotate(ParentToChild.quat.Conjugate(), Basis);
				for (int Row = 0; Row < SpatialDimension; ++Row) X(Row, Column) = Transformed[Row];
			}
			return X;
		}
		SpatialMatrix BodyInertia(const RagdollLinkSettings& Link)
		{
			SpatialMatrix I;
			// Move joint-origin motion to the COM, apply central inertia, then move
			// momentum back. This determines both off-diagonal block signs.
			const auto X = MotionTransform(Transform(Link.CenterOfMass));
			for (int R = 0; R < AngularDimension; ++R) for (int C = 0; C < AngularDimension; ++C) I(R, C) = Link.Inertia[R][C];
			for (int R = AngularDimension; R < SpatialDimension; ++R) I(R, R) = Link.Mass;
			return X.Transpose() * I * X;
		}
		SpatialMatrix InversePositiveDefinite(const SpatialMatrix& A, int Size, double Tolerance)
		{
			SpatialMatrix L, Inverse;
			for (int I = 0; I < Size; ++I) for (int J = 0; J <= I; ++J)
			{
				double Value = A(I, J);
				for (int K = 0; K < J; ++K) Value -= L(I, K) * L(J, K);
				if (I == J)
				{
					if (!std::isfinite(Value) || A(I, I) <= 0 || Value <= Tolerance * A(I, I))
						throw std::invalid_argument("Ragdoll inertia is singular or not positive definite");
					L(I, J) = std::sqrt(Value);
				}
				else L(I, J) = Value / L(J, J);
			}
			for (int Column = 0; Column < Size; ++Column)
			{
				SpatialVector Y, X;
				for (int I = 0; I < Size; ++I)
				{
					double Value = I == Column ? 1.0 : 0.0;
					for (int K = 0; K < I; ++K) Value -= L(I, K) * Y[K];
					Y[I] = Value / L(I, I);
				}
				for (int I = Size - 1; I >= 0; --I)
				{
					double Value = Y[I];
					for (int K = I + 1; K < Size; ++K) Value -= L(K, I) * X[K];
					X[I] = Value / L(I, I); Inverse(I, Column) = X[I];
				}
			}
			return Inverse;
		}
		Quaternion RotationStep(const Vector3& Omega, double Dt)
		{
			const double Speed = std::hypot(std::hypot((double)Omega.x, (double)Omega.y), (double)Omega.z); Quaternion Q = Quaternion::One();
			if (!std::isfinite(Speed * Dt)) throw std::invalid_argument("Ragdoll rotation step overflow");
			if (Speed > 0)
				Q.FromRotationAxis(Vector3((float)(Omega.x / Speed), (float)(Omega.y / Speed), (float)(Omega.z / Speed)), (float)std::remainder(Speed * Dt, 2 * Pi));
			return Q;
		}
	}

	SpatialVector::SpatialVector(const Vector3& Angular, const Vector3& Linear)
	{ for (int I = 0; I < AngularDimension; ++I) { Value[I] = Angular[I]; Value[I + AngularDimension] = Linear[I]; } }
	Vector3 SpatialVector::GetAngular() const { return Vector3((float)Value[0], (float)Value[1], (float)Value[2]); }
	Vector3 SpatialVector::GetLinear() const { return Vector3((float)Value[3], (float)Value[4], (float)Value[5]); }
	SpatialVector SpatialVector::operator+(const SpatialVector& Other) const
	{ SpatialVector Result; for (int I = 0; I < Dimension; ++I) Result[I] = Value[I] + Other[I]; return Result; }
	SpatialVector SpatialVector::operator-(const SpatialVector& Other) const
	{ SpatialVector Result; for (int I = 0; I < Dimension; ++I) Result[I] = Value[I] - Other[I]; return Result; }
	SpatialVector SpatialVector::operator*(double Scale) const
	{ SpatialVector Result; for (int I = 0; I < Dimension; ++I) Result[I] = Value[I] * Scale; return Result; }
	SpatialVector& SpatialVector::operator+=(const SpatialVector& Other)
	{ for (int I = 0; I < Dimension; ++I) Value[I] += Other[I]; return *this; }
	double SpatialVector::Dot(const SpatialVector& Other) const
	{ double Result = 0; for (int I = 0; I < Dimension; ++I) Result += Value[I] * Other[I]; return Result; }
	bool SpatialVector::IsFinite() const
	{ for (double V : Value) if (!std::isfinite(V)) return false; return true; }
	SpatialVector SpatialMatrix::operator*(const SpatialVector& Vector) const
	{
		SpatialVector Out;
		for (int I = 0; I < SpatialDimension; ++I) for (int J = 0; J < SpatialDimension; ++J) Out[I] += (*this)(I, J) * Vector[J];
		return Out;
	}
	SpatialMatrix SpatialMatrix::operator*(const SpatialMatrix& Other) const
	{
		SpatialMatrix Out;
		for (int I = 0; I < SpatialDimension; ++I) for (int K = 0; K < SpatialDimension; ++K) for (int J = 0; J < SpatialDimension; ++J)
			Out(I, J) += (*this)(I, K) * Other(K, J);
		return Out;
	}
	SpatialMatrix SpatialMatrix::Transpose() const
	{
		SpatialMatrix Out;
		for (int I = 0; I < SpatialDimension; ++I) for (int J = 0; J < SpatialDimension; ++J) Out(I, J) = (*this)(J, I);
		return Out;
	}
	SpatialMatrix& SpatialMatrix::operator+=(const SpatialMatrix& Other)
	{ for (size_t I = 0; I < Value.size(); ++I) Value[I] += Other.Value[I]; return *this; }

	FeatherstoneSolver::FeatherstoneSolver(const std::vector<RagdollLinkSettings>& Links, const FeatherstoneSettings& Settings)
		: m_Links(Links), m_Settings(Settings)
	{
		if (Links.empty() || !std::isfinite(Settings.RelativePivotTolerance) || Settings.RelativePivotTolerance <= 0 || Settings.RelativePivotTolerance >= 1
			|| !std::isfinite(Settings.SymmetryTolerance) || Settings.SymmetryTolerance <= 0 || Settings.SymmetryTolerance >= 1)
			throw std::invalid_argument("Invalid reduced ragdoll model or numerical settings");
		int Count = 0;
		for (size_t I = 0; I < Links.size(); ++I)
		{
			auto& L = m_Links[I];
			if ((I == 0 ? L.Parent != -1 : L.Parent < 0 || (size_t)L.Parent >= I)
				|| (L.Joint == RagdollReducedJointType::Floating && I != 0)
				|| !Finite(L.ParentToJoint.pos) || !RotationValid(L.ParentToJoint.quat) || !Finite(L.CenterOfMass)
				|| !Finite(L.Axis) || !std::isfinite(L.Axis.SquareLength()) || L.Axis.SquareLength() <= 0
				|| !Finite(L.ConeAxisParent) || L.ConeAxisParent.SquareLength() <= 0
				|| !Finite(L.ConeAxisChild) || L.ConeAxisChild.SquareLength() <= 0
				|| !std::isfinite(L.ConeLimit) || L.ConeLimit < 0 || L.ConeLimit > Pi
				|| (L.ConeLimit < Pi && L.Joint != RagdollReducedJointType::Spherical)
				|| !std::isfinite(L.Mass) || L.Mass <= 0 || !std::isfinite(L.Armature) || L.Armature < 0 || !std::isfinite(L.Damping) || L.Damping < 0)
				throw std::invalid_argument("Invalid reduced ragdoll link or topology");
			SpatialMatrix Central;
			for (int R = 0; R < AngularDimension; ++R) for (int C = 0; C < AngularDimension; ++C)
			{
				const double Scale = std::max(std::abs(L.Inertia[R][R]), std::abs(L.Inertia[C][C]));
				if (!std::isfinite(L.Inertia[R][C]) || std::abs(L.Inertia[R][C] - L.Inertia[C][R]) > Settings.SymmetryTolerance * Scale)
					throw std::invalid_argument("Ragdoll central inertia must be finite and symmetric");
				Central(R, C) = (L.Inertia[R][C] + (double)L.Inertia[C][R]) / 2;
			}
			InversePositiveDefinite(Central, AngularDimension, Settings.RelativePivotTolerance);
			for (int R = 0; R < AngularDimension; ++R) for (int C = 0; C < AngularDimension; ++C) L.Inertia[R][C] = (float)Central(R, C);
			L.Axis.Normalize(); L.ConeAxisParent.Normalize(); L.ConeAxisChild.Normalize(); L.ParentToJoint.quat = L.ParentToJoint.quat.Unit();
			m_Offsets.push_back(Count); Count += Dofs(L.Joint);
		}
		m_Velocity.resize(Count); m_Configuration.resize(Links.size()); m_Cache.resize(Links.size());
		m_WorkForce.resize(Links.size()); m_WorkAcceleration.resize(Links.size()); m_WorkJoint.resize(Links.size());
		for (size_t I = 0; I < Links.size(); ++I)
		{
			auto& C = m_Cache[I]; const auto& L = m_Links[I]; C.Inertia = BodyInertia(L);
			if (L.Joint == RagdollReducedJointType::Revolute) C.Subspace[0] = SpatialVector(L.Axis, Vector3::Zero());
			else if (L.Joint == RagdollReducedJointType::Prismatic) C.Subspace[0] = SpatialVector(Vector3::Zero(), L.Axis);
			else for (int J = 0; J < Dofs(L.Joint); ++J) C.Subspace[J][J] = 1;
		}
		UpdateKinematics();
	}
	int FeatherstoneSolver::GetLinkDofCount(size_t Index) const { return Dofs(m_Links.at(Index).Joint); }
	void FeatherstoneSolver::ValidateVector(const std::vector<double>& Value) const
	{
		if (Value.size() != m_Velocity.size()) throw std::invalid_argument("Ragdoll generalized vector has the wrong size");
		for (double V : Value) if (!std::isfinite(V)) throw std::invalid_argument("Non-finite reduced ragdoll generalized vector");
	}
	void FeatherstoneSolver::ValidateWrenches(const std::vector<SpatialVector>& Value) const
	{
		if (!Value.empty() && Value.size() != m_Links.size()) throw std::invalid_argument("Ragdoll wrench vector has the wrong size");
		for (const auto& V : Value) if (!V.IsFinite()) throw std::invalid_argument("Non-finite reduced ragdoll wrench");
	}
	void FeatherstoneSolver::SetConfiguration(const std::vector<RagdollConfiguration>& Configuration)
	{
		if (Configuration.size() != m_Links.size()) throw std::invalid_argument("Ragdoll configuration has the wrong size");
		for (const auto& Q : Configuration)
			if (!std::isfinite(Q.Position) || !RotationValid(Q.Rotation) || !Finite(Q.Translation)) throw std::invalid_argument("Invalid reduced ragdoll configuration");
		auto Next = Configuration;
		for (size_t I = 0; I < Next.size(); ++I)
		{
			if (m_Links[I].Joint == RagdollReducedJointType::Prismatic && !std::isfinite((float)Next[I].Position))
				throw std::invalid_argument("Ragdoll translation exceeds geometry precision range");
			Next[I].Rotation = Next[I].Rotation.Unit();
		}
		m_Configuration.swap(Next);
		try { UpdateKinematics(); }
		catch (...) { m_Configuration.swap(Next); UpdateKinematics(); throw; }
		m_FactorizationValid = false;
	}
	void FeatherstoneSolver::SetVelocity(const std::vector<double>& Velocity)
	{
		ValidateVector(Velocity); auto Next = Velocity; m_Velocity.swap(Next);
		try { UpdateVelocities(); }
		catch (...) { m_Velocity.swap(Next); UpdateVelocities(); throw; }
	}
	void FeatherstoneSolver::UpdateKinematics()
	{
		for (size_t I = 0; I < m_Links.size(); ++I)
		{
			const auto& L = m_Links[I]; const auto& Q = m_Configuration[I]; auto& C = m_Cache[I];
			Transform Joint;
			if (L.Joint == RagdollReducedJointType::Revolute) Joint.quat.FromRotationAxis(L.Axis, (float)std::remainder(Q.Position, 2 * Pi));
			if (L.Joint == RagdollReducedJointType::Prismatic) Joint.pos = L.Axis * (float)Q.Position;
			if (L.Joint == RagdollReducedJointType::Spherical || L.Joint == RagdollReducedJointType::Floating) Joint.quat = Q.Rotation;
			if (L.Joint == RagdollReducedJointType::Floating) Joint.pos = Q.Translation;
			const Transform Relative = L.ParentToJoint * Joint;
			C.MotionTransform = MotionTransform(Relative);
			C.World = L.Parent < 0 ? Relative : m_Cache[L.Parent].World * Relative;
			C.World.quat = C.World.quat.Unit();
			if (!Finite(C.World.pos) || !RotationValid(C.World.quat)) throw std::invalid_argument("Ragdoll pose exceeds representable range");
		}
		UpdateVelocities();
	}
	void FeatherstoneSolver::UpdateVelocities()
	{
		for (size_t I = 0; I < m_Links.size(); ++I)
		{
			const auto& L = m_Links[I]; auto& C = m_Cache[I];
			SpatialVector JointVelocity;
			for (int J = 0; J < Dofs(L.Joint); ++J) JointVelocity += C.Subspace[J] * m_Velocity[m_Offsets[I] + J];
			C.Velocity = JointVelocity;
			if (L.Parent >= 0) C.Velocity += C.MotionTransform * m_Cache[L.Parent].Velocity;
			C.BiasAcceleration = MotionCross(C.Velocity, JointVelocity);
			if (!C.Velocity.IsFinite() || !C.BiasAcceleration.IsFinite()
				|| !Finite(C.Velocity.GetAngular()) || !Finite(C.Velocity.GetLinear()))
				throw std::invalid_argument("Ragdoll kinematic state exceeds representable range");
		}
	}
	Vector3 FeatherstoneSolver::GetCenterOfMass(size_t Index) const { return m_Cache.at(Index).World.LocalToWorld(m_Links.at(Index).CenterOfMass); }
	SpatialVector FeatherstoneSolver::GetLinkVelocity(size_t Index) const { const auto& C = m_Cache.at(Index); return Rotate(C.World.quat, C.Velocity); }
	std::vector<SpatialVector> FeatherstoneSolver::ComputeLinkVelocities(const std::vector<double>& Velocity) const
	{
		ValidateVector(Velocity); std::vector<SpatialVector> Local(m_Links.size()), World(m_Links.size());
		for (size_t I = 0; I < m_Links.size(); ++I)
		{
			const auto& L = m_Links[I]; const auto& C = m_Cache[I];
			if (L.Parent >= 0) Local[I] = C.MotionTransform * Local[L.Parent];
			for (int J = 0; J < Dofs(L.Joint); ++J) Local[I] += C.Subspace[J] * Velocity[m_Offsets[I] + J];
			World[I] = Rotate(C.World.quat, Local[I]);
		}
		return World;
	}
	/* Robotics queries share the integrator's coordinate convention and do not
	   modify configuration, velocity, or the ABA factorization. Columns follow
	   GetLinkDofOffset, including a floating root's BODY-FRAME twist coordinates.
	   Jacobian rows use WORLD axes, [angular; linear], at a point supplied in the
	   link's joint frame. Dense queries stack six rows per link, including a fixed
	   root's zero rows; by default the point is each link's COM. J*qdot is physical
	   point velocity, not a derivative of quaternion components or Euler angles.

	   Only an ancestor's motion can affect a link: compose the cached local motion
	   transforms, rotate into world axes, then shift to the requested local point.
	   Composing local transforms avoids subtracting large world positions to find
	   lever arms. The point shift is omega cross (point-origin).
	   The transpose operation maps world wrenches at joint origins to generalized
	   forces in one reverse tree pass, preserving power: qdot dot Q = sum(v dot F).
	   Tendon gradients and mechanism constraints can use these maps without owning
	   another kinematic state or approximating tree response by independent masses.

	   The composite rigid body algorithm (CRBA) sums each subtree's locked inertia.
	   A joint column F=Ic*S is propagated to ancestors with X transpose; projection
	   on each ancestor subspace fills the symmetric blocks of H. Disjoint branches
	   have zero cross blocks. Work is quadratic for a chain, linear in ancestor
	   depth for a branch, plus the requested dense output storage. Motor armature
	   adds only to the diagonal. This avoids one inverse-dynamics solve per column.
	   H*qddot + bias - generalizedExternalForce equals InverseDynamics; bias includes
	   Coriolis, gravity and the model's viscous joint damping. Solvers should still
	   use ABA's inverse-mass action instead of explicitly inverting this matrix.
	   Reference: Featherstone, Rigid Body Dynamics Algorithms (2008), CRBA and RNEA. */
	void FeatherstoneSolver::FillLinkJacobian(size_t Index, const Vector3& LinkPoint, RagdollMatrix& Matrix, int RowOffset) const
	{
		const Vector3 Point = m_Cache.at(Index).World.LocalToWorld(LinkPoint);
		if (!Finite(Point)) throw std::invalid_argument("Invalid reduced ragdoll Jacobian point");
		const Vector3 Offset = m_Cache[Index].World.quat * LinkPoint;
		SpatialMatrix MotionToLink; for (int I = 0; I < SpatialDimension; ++I) MotionToLink(I, I) = 1;
		for (int Ancestor = (int)Index; Ancestor >= 0; Ancestor = m_Links[Ancestor].Parent)
		{
			const auto& Cache = m_Cache[Ancestor];
			for (int Column = 0; Column < Dofs(m_Links[Ancestor].Joint); ++Column)
			{
				auto Motion = Rotate(m_Cache[Index].World.quat, MotionToLink * Cache.Subspace[Column]);
				for (int I = 0; I < AngularDimension; ++I)
				{
					const int J = (I + 1) % AngularDimension, K = (I + 2) % AngularDimension;
					Motion[I + AngularDimension] += Motion[J] * Offset[K] - Motion[K] * Offset[J];
				}
				for (int Row = 0; Row < SpatialDimension; ++Row) Matrix[RowOffset + Row][m_Offsets[Ancestor] + Column] = Motion[Row];
			}
			MotionToLink = MotionToLink * Cache.MotionTransform;
		}
	}
	RagdollMatrix FeatherstoneSolver::ComputeLinkJacobian(size_t Index, const Vector3& LinkPoint) const
	{
		auto Matrix = QueryMatrix(SpatialDimension, m_Velocity.size()); FillLinkJacobian(Index, LinkPoint, Matrix, 0); return Matrix;
	}
	RagdollMatrix FeatherstoneSolver::ComputeDenseJacobian(bool AtCenterOfMass) const
	{
		auto Matrix = QueryMatrix(m_Links.size() * SpatialDimension, m_Velocity.size());
		for (size_t I = 0; I < m_Links.size(); ++I)
			FillLinkJacobian(I, AtCenterOfMass ? m_Links[I].CenterOfMass : Vector3::Zero(), Matrix, (int)I * SpatialDimension);
		return Matrix;
	}
	RagdollMatrix FeatherstoneSolver::ComputeMassMatrix() const
	{
		auto Matrix = QueryMatrix(m_Velocity.size(), m_Velocity.size());
		std::vector<SpatialMatrix> Composite; Composite.reserve(m_Links.size());
		for (const auto& Cache : m_Cache) Composite.push_back(Cache.Inertia);
		for (size_t Reverse = m_Links.size(); Reverse > 0; --Reverse)
		{
			const size_t I = Reverse - 1; const int Parent = m_Links[I].Parent;
			if (Parent >= 0) Composite[Parent] += m_Cache[I].MotionTransform.Transpose() * Composite[I] * m_Cache[I].MotionTransform;
		}
		for (size_t I = 0; I < m_Links.size(); ++I)
		{
			for (int Column = 0; Column < Dofs(m_Links[I].Joint); ++Column)
			{
				const int C = m_Offsets[I] + Column; auto Force = Composite[I] * m_Cache[I].Subspace[Column];
				for (int Ancestor = (int)I; Ancestor >= 0;)
				{
					for (int Row = 0; Row < Dofs(m_Links[Ancestor].Joint); ++Row)
					{
						const int R = m_Offsets[Ancestor] + Row;
						Matrix[R][C] = Matrix[C][R] = m_Cache[Ancestor].Subspace[Row].Dot(Force);
					}
					Force = m_Cache[Ancestor].MotionTransform.Transpose() * Force; Ancestor = m_Links[Ancestor].Parent;
				}
				Matrix[C][C] += m_Links[I].Armature;
			}
		}
		return Matrix;
	}
	std::vector<double> FeatherstoneSolver::ComputeGeneralizedForce(const std::vector<SpatialVector>& WorldWrenches) const
	{
		ValidateWrenches(WorldWrenches); std::vector<double> Result(m_Velocity.size());
		if (WorldWrenches.empty()) return Result;
		std::vector<SpatialVector> Force(m_Links.size());
		for (size_t I = 0; I < m_Links.size(); ++I) Force[I] = Rotate(m_Cache[I].World.quat.Conjugate(), WorldWrenches[I]);
		for (size_t Reverse = m_Links.size(); Reverse > 0; --Reverse)
		{
			const size_t I = Reverse - 1; const auto& Cache = m_Cache[I];
			for (int Dof = 0; Dof < Dofs(m_Links[I].Joint); ++Dof) Result[m_Offsets[I] + Dof] = Cache.Subspace[Dof].Dot(Force[I]);
			if (m_Links[I].Parent >= 0) Force[m_Links[I].Parent] += Cache.MotionTransform.Transpose() * Force[I];
		}
		ValidateVector(Result); return Result;
	}
	std::vector<double> FeatherstoneSolver::ComputeBiasForce(const Vector3& Gravity) const
	{
		return InverseDynamics(std::vector<double>(m_Velocity.size()), Gravity);
	}
	void FeatherstoneSolver::Factorize()
	{
		if (m_FactorizationValid) return;
		for (auto& C : m_Cache) C.ReducedRagdollInertia = C.Inertia;
		for (size_t Reverse = m_Links.size(); Reverse > 0; --Reverse)
		{
			const size_t I = Reverse - 1; auto& C = m_Cache[I]; const auto& L = m_Links[I]; const int Count = Dofs(L.Joint);
			SpatialMatrix JointInertia;
			for (int J = 0; J < Count; ++J) C.InertiaSubspace[J] = C.ReducedRagdollInertia * C.Subspace[J];
			for (int R = 0; R < Count; ++R) for (int Col = 0; Col < Count; ++Col)
				JointInertia(R, Col) = C.Subspace[R].Dot(C.InertiaSubspace[Col]) + (R == Col ? L.Armature : 0);
			C.JointInverseInertia = InversePositiveDefinite(JointInertia, Count, m_Settings.RelativePivotTolerance);
			C.ReducedInertia = C.ReducedRagdollInertia;
			for (int Row = 0; Row < SpatialDimension; ++Row) for (int Column = 0; Column < SpatialDimension; ++Column)
				for (int A = 0; A < Count; ++A) for (int B = 0; B < Count; ++B)
					C.ReducedInertia(Row, Column) -= C.InertiaSubspace[A][Row] * C.JointInverseInertia(A, B) * C.InertiaSubspace[B][Column];
			if (L.Parent >= 0) m_Cache[L.Parent].ReducedRagdollInertia += C.MotionTransform.Transpose() * C.ReducedInertia * C.MotionTransform;
		}
		m_FactorizationValid = true; ++m_FactorizationCount;
	}
	std::vector<double> FeatherstoneSolver::Solve(const std::vector<double>& GeneralizedForce, const Vector3& Gravity,
		const std::vector<SpatialVector>& ExternalWrenches, bool IncludeBias)
	{
		ValidateVector(GeneralizedForce); ValidateWrenches(ExternalWrenches);
		if (!Finite(Gravity)) throw std::invalid_argument("Invalid reduced ragdoll gravity");
		Factorize();
		for (size_t I = 0; I < m_Links.size(); ++I)
		{
			const auto& C = m_Cache[I];
			m_WorkForce[I] = IncludeBias ? ForceCross(C.Velocity, C.Inertia * C.Velocity) : SpatialVector();
			if (!ExternalWrenches.empty()) m_WorkForce[I] = m_WorkForce[I] - Rotate(C.World.quat.Conjugate(), ExternalWrenches[I]);
		}
		for (size_t Reverse = m_Links.size(); Reverse > 0; --Reverse)
		{
			const size_t I = Reverse - 1; const auto& L = m_Links[I]; const auto& C = m_Cache[I]; const int Count = Dofs(L.Joint);
			SpatialVector U;
			for (int J = 0; J < Count; ++J) U[J] = GeneralizedForce[m_Offsets[I] + J] - C.Subspace[J].Dot(m_WorkForce[I])
				- (IncludeBias ? L.Damping * m_Velocity[m_Offsets[I] + J] : 0);
			m_WorkJoint[I] = U;
			SpatialVector Transmitted = m_WorkForce[I];
			if (IncludeBias) Transmitted += C.ReducedInertia * C.BiasAcceleration;
			const SpatialVector JointAcceleration = C.JointInverseInertia * U;
			for (int J = 0; J < Count; ++J) Transmitted += C.InertiaSubspace[J] * JointAcceleration[J];
			if (L.Parent >= 0) m_WorkForce[L.Parent] += C.MotionTransform.Transpose() * Transmitted;
		}
		std::vector<double> Result(m_Velocity.size()); const SpatialVector BaseAcceleration(Vector3::Zero(), -Gravity);
		for (size_t I = 0; I < m_Links.size(); ++I)
		{
			const auto& L = m_Links[I]; const auto& C = m_Cache[I];
			SpatialVector A = C.MotionTransform * (L.Parent < 0 ? BaseAcceleration : m_WorkAcceleration[L.Parent]);
			if (IncludeBias) A += C.BiasAcceleration;
			SpatialVector U = m_WorkJoint[I];
			for (int J = 0; J < Dofs(L.Joint); ++J) U[J] -= C.InertiaSubspace[J].Dot(A);
			const SpatialVector Qdd = C.JointInverseInertia * U;
			for (int J = 0; J < Dofs(L.Joint); ++J) { Result[m_Offsets[I] + J] = Qdd[J]; A += C.Subspace[J] * Qdd[J]; }
			m_WorkAcceleration[I] = A;
		}
		ValidateVector(Result); return Result;
	}
	std::vector<double> FeatherstoneSolver::ComputeAcceleration(const std::vector<double>& Torque, const Vector3& Gravity,
		const std::vector<SpatialVector>& ExternalWrenches)
	{ return Solve(Torque, Gravity, ExternalWrenches, true); }
	std::vector<double> FeatherstoneSolver::ComputeImpulseResponse(const std::vector<SpatialVector>& LinkImpulses, const std::vector<double>& JointImpulses)
	{ return Solve(JointImpulses.empty() ? std::vector<double>(m_Velocity.size()) : JointImpulses, Vector3::Zero(), LinkImpulses, false); }
	std::vector<double> FeatherstoneSolver::InverseDynamics(const std::vector<double>& Acceleration, const Vector3& Gravity,
		const std::vector<SpatialVector>& ExternalWrenches) const
	{
		ValidateVector(Acceleration); ValidateWrenches(ExternalWrenches);
		if (!Finite(Gravity)) throw std::invalid_argument("Invalid reduced ragdoll gravity");
		std::vector<SpatialVector> A(m_Links.size()), Force(m_Links.size());
		const SpatialVector BaseAcceleration(Vector3::Zero(), -Gravity);
		for (size_t I = 0; I < m_Links.size(); ++I)
		{
			const auto& L = m_Links[I]; const auto& C = m_Cache[I];
			A[I] = C.MotionTransform * (L.Parent < 0 ? BaseAcceleration : A[L.Parent]) + C.BiasAcceleration;
			for (int J = 0; J < Dofs(L.Joint); ++J) A[I] += C.Subspace[J] * Acceleration[m_Offsets[I] + J];
			Force[I] = C.Inertia * A[I] + ForceCross(C.Velocity, C.Inertia * C.Velocity);
			if (!ExternalWrenches.empty()) Force[I] = Force[I] - Rotate(C.World.quat.Conjugate(), ExternalWrenches[I]);
		}
		std::vector<double> Torque(m_Velocity.size());
		for (size_t Reverse = m_Links.size(); Reverse > 0; --Reverse)
		{
			const size_t I = Reverse - 1; const auto& L = m_Links[I]; const auto& C = m_Cache[I];
			for (int J = 0; J < Dofs(L.Joint); ++J) Torque[m_Offsets[I] + J] = C.Subspace[J].Dot(Force[I])
				+ L.Armature * Acceleration[m_Offsets[I] + J] + L.Damping * m_Velocity[m_Offsets[I] + J];
			if (L.Parent >= 0) Force[L.Parent] += C.MotionTransform.Transpose() * Force[I];
		}
		ValidateVector(Torque); return Torque;
	}
	void FeatherstoneSolver::IntegrateVelocity(const std::vector<double>& Acceleration, double Dt)
	{
		ValidateVector(Acceleration);
		if (!std::isfinite(Dt) || Dt < 0) throw std::invalid_argument("Invalid reduced ragdoll time step");
		auto Next = m_Velocity;
		for (size_t I = 0; I < Next.size(); ++I) Next[I] += Dt * Acceleration[I];
		SetVelocity(Next);
	}
	void FeatherstoneSolver::IntegrateConfiguration(double Dt)
	{
		if (!std::isfinite(Dt) || Dt < 0) throw std::invalid_argument("Invalid reduced ragdoll time step");
		if (Dt == 0) return;
		auto Next = m_Configuration;
		for (size_t I = 0; I < m_Links.size(); ++I)
		{
			const auto Type = m_Links[I].Joint; const int Offset = m_Offsets[I]; auto& Q = Next[I];
			if (Type == RagdollReducedJointType::Revolute || Type == RagdollReducedJointType::Prismatic) Q.Position += m_Velocity[Offset] * Dt;
			else if (Type == RagdollReducedJointType::Spherical || Type == RagdollReducedJointType::Floating)
			{
				const Vector3 Omega((float)m_Velocity[Offset], (float)m_Velocity[Offset + 1], (float)m_Velocity[Offset + 2]);
				if (Type == RagdollReducedJointType::Floating)
					Q.Translation += Q.Rotation * Vector3((float)(m_Velocity[Offset + 3] * Dt), (float)(m_Velocity[Offset + 4] * Dt), (float)(m_Velocity[Offset + 5] * Dt));
				Q.Rotation = (Q.Rotation * RotationStep(Omega, Dt)).Unit();
			}
		}
		SetConfiguration(Next);
	}
	double FeatherstoneSolver::GetKineticEnergy() const
	{
		double Energy = 0;
		for (size_t I = 0; I < m_Links.size(); ++I)
		{
			const auto& C = m_Cache[I]; Energy += C.Velocity.Dot(C.Inertia * C.Velocity) / 2;
			for (int J = 0; J < Dofs(m_Links[I].Joint); ++J) { const double V = m_Velocity[m_Offsets[I] + J]; Energy += m_Links[I].Armature * V * V / 2; }
		}
		return Energy;
	}
	double FeatherstoneSolver::GetPotentialEnergy(const Vector3& Gravity) const
	{
		if (!Finite(Gravity)) throw std::invalid_argument("Invalid reduced ragdoll gravity");
		double Energy = 0;
		for (size_t I = 0; I < m_Links.size(); ++I)
		{
			const Vector3 Position = GetCenterOfMass(I);
			for (int Axis = 0; Axis < AngularDimension; ++Axis) Energy -= (double)m_Links[I].Mass * Gravity[Axis] * Position[Axis];
		}
		return Energy;
	}
	SpatialVector FeatherstoneSolver::GetMomentum() const
	{
		SpatialVector Total;
		for (size_t I = 0; I < m_Links.size(); ++I)
		{
			const auto& C = m_Cache[I]; auto Momentum = Rotate(C.World.quat, C.Inertia * C.Velocity);
			for (int Axis = 0; Axis < AngularDimension; ++Axis)
			{
				const int J = (Axis + 1) % AngularDimension, K = (Axis + 2) % AngularDimension;
				Momentum[Axis] += C.World.pos[J] * Momentum[K + AngularDimension] - C.World.pos[K] * Momentum[J + AngularDimension];
			}
			Total += Momentum;
		}
		return Total;
	}
}
