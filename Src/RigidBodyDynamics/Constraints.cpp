#include "Constraints.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "RigidBody.h"
#include "../LinearSystem/EigenValueDecomposition.h"

namespace Riemann
{
	/* Joint reaction loads are signed sums of solver increments, not sums of their
	   magnitudes. Velocity impulses give F = sum(J^T*deltaLambda)/dt. In PBD/XPBD,
	   pose multipliers also contribute sum(J^T*deltaLambdaPosition)/dt^2 because
	   joint pose corrections enter reconstructed velocities. SI/LCP geometric
	   projection does not change momentum and is excluded from physical reports.
	   See Macklin et al., "XPBD: Position-Based Simulation of Compliant Constrained
	   Dynamics" (2016).

	   Force and torque act on body B, in world coordinates at its joint attachment.
	   A point force contributes force but no intrinsic joint torque: its r x F
	   moment about the body's COM is not a torsional load on a ball joint. Row
	   Jacobians retain that moment for dynamics while CoupleB records actual torque.
	   Limits, damping and motors all contribute; opposite row increments can cancel.

	   Break thresholds compare converged average force/torque at the outer-frame
	   boundary (EndStep, or EndFrame when temporal substeps are enabled). Testing
	   inside an iteration would mistake temporary block loads for failure.
	   A broken joint stops solving on subsequent frames; the completed frame's impulse
	   is retained. Repair is explicit and keeps the original attachment frames.
	   This reports impulses, not substep peak stresses or a fracture material model.
	*/
	/* Joint drives share the contact/joint velocity iterations on every solver route.
	   An implicit damped spring has F = -k*C - d*(v - targetVelocity). With
	   C_next = C + dt*v_next, its row solves
	       deltaLambda = -(v + beta*C - targetWeight*targetVelocity + gamma*lambda)
	                     / (J*M^-1*J^T + gamma),
	   where beta=k/(d+dt*k), targetWeight=d/(d+dt*k), gamma=1/(dt*(d+dt*k)).
	   Acceleration gains multiply gamma by the row's inverse effective mass.
	   The physical impulse is still limited to MaxForce*dt in either mode.
	   Multipliers reset once per substep, not per iteration or contact pass.

	   Scalar drives reuse the limit Jacobians, including rotating linear frames.
	   Six-DOF axes follow this project's existing Y-X-Z angular coordinates; they
	   use Y-X-Z angles rather than swing/twist coordinates. The alternate SLERP drive uses the
	   shortest quaternion logarithm and solves a coupled angular block in world
	   space. Its torque cap is spherical, so changing the reference frame cannot
	   change the allowed torque. Singular inertia modes receive no impulse.
	   Limits run after drives, and contact iterations share the resulting velocity.
	   Springs therefore yield to contacts instead of teleporting constrained poses.
	   Reference: Erin Catto, "Soft Constraints" (GDC 2011). This is a locally linearized implicit spring, not
	   an exact nonlinear trajectory integrator. Driven joints stay awake until
	   disabled; island sleep policies can later use drive error as an additional test.
	*/
	namespace
	{
		constexpr float DriveMinimumWeight = 1.0e-9f;
		constexpr float DriveRelativeRankTolerance = 1.0e-6f;
		constexpr float DriveQuaternionNormTolerance = 1.0e-3f;
		bool FiniteDriveVector(const Vector3& V)
		{
			return std::isfinite(V.x) && std::isfinite(V.y) && std::isfinite(V.z);
		}
		struct DriveCoefficients
		{
			double Beta, TargetWeight, Gamma, Limit;
			DriveCoefficients(const JointDriveSettings& Settings, float Dt)
			{
				const double Spring = Settings.PositionEnabled ? Settings.Stiffness : 0.0;
				const double Denominator = Settings.Damping + Dt * Spring;
				Beta = Spring / Denominator;
				TargetWeight = Settings.VelocityEnabled ? Settings.Damping / Denominator : 0.0;
				Gamma = 1.0 / (Dt * Denominator);
				Limit = std::min((double)std::numeric_limits<float>::max(), (double)Settings.MaxForce * Dt);
			}
		};
		float LinearWeight(const JointSolverBody& body)
		{
			RigidBodyDynamic* dynamicBody = body ? body->CastDynamic() : nullptr;
			return dynamicBody && !dynamicBody->Freezing ? dynamicBody->InvMass * body.InverseMassScale : 0.0f;
		}

		void WakeBody(const JointSolverBody& body)
		{
			RigidBodyDynamic* dynamicBody = body ? body->CastDynamic() : nullptr;
			if (dynamicBody)
			{
				dynamicBody->Wakeup();
			}
		}

		void RotateBody(const JointSolverBody& body, const Vector3& axis, float angle)
		{
			if (body == nullptr || LinearWeight(body) <= 0.0f || std::abs(angle) <= 1e-7f || axis.SquareLength() <= 1e-10f)
			{
				return;
			}
			Quaternion rotation;
			rotation.FromRotationAxis(axis.SafeUnit(), angle);
			body->Q = (rotation * body->Q).Unit();
			WakeBody(body);
		}

		Matrix3 GetInverseInertia(const JointSolverBody& body)
		{
			RigidBodyDynamic* dynamicBody = body ? body->CastDynamic() : nullptr;
			return dynamicBody && !dynamicBody->Freezing ? dynamicBody->GetInverseInertia_WorldSpace() * body.InverseMassScale : Matrix3::Zero();
		}

		Vector3 ClampCorrection(const Vector3& correction, float maximumLength)
		{
			const float length = correction.Length();
			return length > maximumLength ? correction * (maximumLength / length) : correction;
		}

		Matrix3 BuildPointEffectiveMass(const JointSolverBody& bodyA, const JointSolverBody& bodyB, const Vector3& rA, const Vector3& rB)
		{
			const float inverseMass = LinearWeight(bodyA) + LinearWeight(bodyB);
			Matrix3 effectiveMass(inverseMass, inverseMass, inverseMass);
			const Matrix3 inverseInertiaA = GetInverseInertia(bodyA);
			const Matrix3 inverseInertiaB = GetInverseInertia(bodyB);
			Vector3 mutableRA = rA;
			Vector3 mutableRB = rB;
			const Matrix3 crossA = Matrix3::Skew(mutableRA);
			const Matrix3 crossB = Matrix3::Skew(mutableRB);
			effectiveMass -= crossA * inverseInertiaA * crossA;
			effectiveMass -= crossB * inverseInertiaB * crossB;
			return effectiveMass;
		}

		void ApplyPositionImpulse(const JointSolverBody& body, const Vector3& impulse, const Vector3& r)
		{
			RigidBodyDynamic* dynamicBody = body ? body->CastDynamic() : nullptr;
			if (dynamicBody == nullptr || dynamicBody->Freezing || body.InverseMassScale == 0)
			{
				return;
			}
			dynamicBody->X += impulse * LinearWeight(body);
			const Vector3 angularCorrection = GetInverseInertia(body) * r.Cross(impulse);
			const float angle = angularCorrection.Length();
			if (angle > 1e-7f)
			{
				RotateBody(dynamicBody, angularCorrection / angle, angle);
			}
			WakeBody(dynamicBody);
		}

		void ApplyVelocityImpulse(const JointSolverBody& body, const Vector3& impulse, const Vector3& r)
		{
			RigidBodyDynamic* dynamicBody = body ? body->CastDynamic() : nullptr;
			if (dynamicBody == nullptr || dynamicBody->Freezing || body.InverseMassScale == 0)
			{
				return;
			}
			dynamicBody->V += impulse * LinearWeight(body);
			dynamicBody->W += GetInverseInertia(body) * r.Cross(impulse);
			if (!impulse.IsZero()) dynamicBody->Wakeup();
		}

		Vector3 GetAngularVelocity(const JointSolverBody& body)
		{
			if (!body || body->mRigidType == RigidType::Static)
			{
				return Vector3::Zero();
			}
			RigidBodyDynamic* dynamicBody = body->CastDynamic();
			return dynamicBody && dynamicBody->Freezing ? Vector3::Zero() : body->W;
		}

		Vector3 GetPointVelocity(const JointSolverBody& body, const Vector3& worldPoint)
		{
			if (!body || body->mRigidType == RigidType::Static)
			{
				return Vector3::Zero();
			}
			RigidBodyDynamic* dynamicBody = body ? body->CastDynamic() : nullptr;
			return dynamicBody && dynamicBody->Freezing ? Vector3::Zero() : body->V + body->W.Cross(worldPoint - body->X);
		}

		void ApplyPointPositionCorrection(JointImpulseAccumulator& Reactions, const JointSolverBody& bodyA, const JointSolverBody& bodyB, const Vector3& anchorA, const Vector3& anchorB,
			const Vector3& error, float alpha = 0.0f, Vector3* lambda = nullptr)
		{
			const Vector3 rA = anchorA - bodyA->X;
			const Vector3 rB = anchorB - bodyB->X;
			Matrix3 effectiveMass = BuildPointEffectiveMass(bodyA, bodyB, rA, rB);
			const bool soft = alpha > 0.0f && lambda;
			Vector3 rhs = -ClampCorrection(error, 0.5f);
			if (soft)
			{
				effectiveMass += Matrix3(alpha, alpha, alpha);
				rhs = -error - *lambda * alpha;
			}
			Vector3 impulse;
			if (!effectiveMass.Solve(rhs, impulse)) return;
			if (soft)
			{
				const float translation = std::max(LinearWeight(bodyA), LinearWeight(bodyB)) * impulse.Length();
				const float rotation = std::max((GetInverseInertia(bodyA) * rA.Cross(impulse)).Length(),
					(GetInverseInertia(bodyB) * rB.Cross(impulse)).Length());
				float scale = 1.0f;
				if (translation > 0.5f) scale = std::min(scale, 0.5f / translation);
				if (rotation > 0.2f) scale = std::min(scale, 0.2f / rotation);
				impulse *= scale;
				*lambda += impulse;
			}
			ApplyPositionImpulse(bodyA, -impulse, rA);
			ApplyPositionImpulse(bodyB, impulse, rB);
			Reactions.Add(impulse, Vector3::Zero());
		}

		void RemoveRelativePointVelocity(JointImpulseAccumulator& Reactions, const JointSolverBody& bodyA, const JointSolverBody& bodyB, const Vector3& anchorA, const Vector3& anchorB)
		{
			const Vector3 rA = anchorA - bodyA->X;
			const Vector3 rB = anchorB - bodyB->X;
			Matrix3 effectiveMass = BuildPointEffectiveMass(bodyA, bodyB, rA, rB);
			Vector3 impulse;
			if (!effectiveMass.Solve(GetPointVelocity(bodyA, anchorA) - GetPointVelocity(bodyB, anchorB), impulse))
			{
				return;
			}
			ApplyVelocityImpulse(bodyA, -impulse, rA);
			ApplyVelocityImpulse(bodyB, impulse, rB);
			Reactions.Add(impulse, Vector3::Zero());
		}

		void ApplyAngularVelocityImpulse(const JointSolverBody& body, const Vector3& impulse)
		{
			RigidBodyDynamic* dynamicBody = body ? body->CastDynamic() : nullptr;
			if (dynamicBody && !dynamicBody->Freezing && body.InverseMassScale > 0)
			{
				dynamicBody->W += GetInverseInertia(body) * impulse;
				if (!impulse.IsZero()) dynamicBody->Wakeup();
			}
		}

		void RemoveRelativeAngularVelocityAlongAxis(JointImpulseAccumulator& Reactions, const JointSolverBody& bodyA, const JointSolverBody& bodyB, const Vector3& axis)
		{
			const Vector3 unitAxis = axis.SafeUnit();
			if (unitAxis.IsZero())
			{
				return;
			}
			const Matrix3 inverseInertia = GetInverseInertia(bodyA) + GetInverseInertia(bodyB);
			const float denominator = unitAxis.Dot(inverseInertia * unitAxis);
			if (denominator <= 1e-7f)
			{
				return;
			}
			const Vector3 relativeAngularVelocity = GetAngularVelocity(bodyB) - GetAngularVelocity(bodyA);
			const Vector3 impulse = unitAxis * (-relativeAngularVelocity.Dot(unitAxis) / denominator);
			Reactions.Add(Vector3::Zero(), impulse);
			ApplyAngularVelocityImpulse(bodyA, -impulse);
			ApplyAngularVelocityImpulse(bodyB, impulse);
		}

		void ApplyAngularCorrection(JointImpulseAccumulator& Reactions, const JointSolverBody& bodyA, const JointSolverBody& bodyB, const Vector3& axisAngle,
			float alpha = 0.0f, Vector3* lambda = nullptr)
		{
			const bool soft = alpha > 0.0f && lambda;
			const Vector3 limitedCorrection = soft ? axisAngle : ClampCorrection(axisAngle, 0.2f);
			if (!soft && limitedCorrection.Length() <= 1e-7f) return;
			Matrix3 inverseInertia = GetInverseInertia(bodyA) + GetInverseInertia(bodyB);
			Vector3 rhs = -limitedCorrection;
			if (soft)
			{
				inverseInertia += Matrix3(alpha, alpha, alpha);
				rhs -= *lambda * alpha;
			}
			Vector3 impulse;
			if (!inverseInertia.Solve(rhs, impulse)) return;
			Vector3 correctionA = GetInverseInertia(bodyA) * -impulse;
			Vector3 correctionB = GetInverseInertia(bodyB) * impulse;
			if (soft)
			{
				const float maximum = std::max(correctionA.Length(), correctionB.Length());
				if (maximum > 0.2f)
				{
					const float scale = 0.2f / maximum;
					impulse *= scale; correctionA *= scale; correctionB *= scale;
				}
				*lambda += impulse;
			}
			const float angleA = correctionA.Length();
			const float angleB = correctionB.Length();
			Reactions.Add(Vector3::Zero(), impulse);
			if (angleA > 1e-7f) RotateBody(bodyA, correctionA / angleA, angleA);
			if (angleB > 1e-7f) RotateBody(bodyB, correctionB / angleB, angleB);
		}

		Vector3 QuaternionAxisAngle(Quaternion rotation)
		{
			rotation = rotation.Unit();
			if (rotation.w < 0.0f)
			{
				rotation = -rotation;
			}
			const float sinHalf = std::sqrt(std::max(0.0f, 1.0f - rotation.w * rotation.w));
			if (sinHalf <= 1e-6f)
			{
				return Vector3::Zero();
			}
			const float angle = 2.0f * std::atan2(sinHalf, rotation.w);
			return Vector3(rotation.x, rotation.y, rotation.z) * (angle / sinHalf);
		}

		void SolveRelativeRotation(JointImpulseAccumulator& Reactions, const JointSolverBody& bodyA, const JointSolverBody& bodyB, const Quaternion& targetRelativeRotation,
			float alpha = 0.0f, Vector3* localLambda = nullptr)
		{
			const Quaternion currentRelative = bodyA->Q.Conjugate() * bodyB->Q;
			const Quaternion localCorrection = targetRelativeRotation * currentRelative.Conjugate();
			const Quaternion frame = bodyA->Q;
			Vector3 worldLambda = localLambda ? frame * *localLambda : Vector3::Zero();
			ApplyAngularCorrection(Reactions, bodyA, bodyB, -(bodyA->Q * QuaternionAxisAngle(localCorrection)),
				alpha, localLambda ? &worldLambda : nullptr);
			if (localLambda) *localLambda = frame.Conjugate() * worldLambda;
		}

		float SignedAngle(const Vector3& from, const Vector3& to, const Vector3& axis)
		{
			return std::atan2(axis.Dot(from.Cross(to)), std::max(-1.0f, std::min(1.0f, from.Dot(to))));
		}

		float Clamp(float value, float minValue, float maxValue)
		{
			return std::max(minValue, std::min(maxValue, value));
		}

		Vector3 ValidAxis(const Vector3& axis)
		{
			return axis.SquareLength() > 1e-12f ? axis.SafeUnit() : Vector3::UnitX();
		}

		bool IsLimitActive(float position, float speed, float minimum, float maximum)
		{
			return maximum - minimum <= 1e-6f
				|| (position <= minimum + 1e-5f && speed < 0.0f)
				|| (position >= maximum - 1e-5f && speed > 0.0f);
		}

		struct ConstraintRow
		{
			Vector3 LinearAxis;
			Vector3 AngularA;
			Vector3 AngularB;
			Vector3 CoupleB = Vector3::Zero();

			float EffectiveMass(const JointSolverBody& bodyA, const JointSolverBody& bodyB) const
			{
				return (LinearWeight(bodyA) + LinearWeight(bodyB)) * LinearAxis.SquareLength()
					+ AngularA.Dot(GetInverseInertia(bodyA) * AngularA)
					+ AngularB.Dot(GetInverseInertia(bodyB) * AngularB);
			}

			float Velocity(const JointSolverBody& bodyA, const JointSolverBody& bodyB) const
			{
				return LinearAxis.Dot(GetPointVelocity(bodyB, bodyB->X) - GetPointVelocity(bodyA, bodyA->X))
					+ AngularA.Dot(GetAngularVelocity(bodyA)) + AngularB.Dot(GetAngularVelocity(bodyB));
			}

			void SolvePosition(JointImpulseAccumulator& Reactions, const JointSolverBody& bodyA, const JointSolverBody& bodyB, float error,
				float alpha = 0.0f, float* lambda = nullptr, bool unilateral = false) const
			{
				const float denominator = EffectiveMass(bodyA, bodyB);
				const bool soft = alpha > 0.0f && lambda;
				if (denominator <= 1e-9f || (!soft && std::abs(error) <= 1e-7f)) return;
				float impulse = soft ? (-error - alpha * *lambda) / (denominator + alpha) : -error / denominator;
				if (unilateral && lambda) impulse = std::max(-*lambda, impulse);
				if (!std::isfinite(impulse) || std::abs(impulse) <= 1e-12f) return;
				const Vector3 rotationA = GetInverseInertia(bodyA) * AngularA;
				const Vector3 rotationB = GetInverseInertia(bodyB) * AngularB;
				// Scale the entire row so the angular trust region preserves its Jacobian.
				const float maxRotation = std::max(rotationA.Length(), rotationB.Length()) * std::abs(impulse);
				if (maxRotation > 0.2f) impulse *= 0.2f / maxRotation;
				if (soft)
				{
					const float translation = std::max(LinearWeight(bodyA), LinearWeight(bodyB)) * LinearAxis.Length() * std::abs(impulse);
					if (translation > 0.5f) impulse *= 0.5f / translation;
					*lambda += impulse;
				}
				Reactions.Add(LinearAxis * impulse, CoupleB * impulse);
				if (LinearWeight(bodyA) > 0.0f)
				{
					bodyA->X -= LinearAxis * (LinearWeight(bodyA) * impulse);
					WakeBody(bodyA);
				}
				if (LinearWeight(bodyB) > 0.0f)
				{
					bodyB->X += LinearAxis * (LinearWeight(bodyB) * impulse);
					WakeBody(bodyB);
				}
				RotateBody(bodyA, rotationA, rotationA.Length() * impulse);
				RotateBody(bodyB, rotationB, rotationB.Length() * impulse);
			}

			void SolveVelocity(JointImpulseAccumulator& Reactions, const JointSolverBody& bodyA, const JointSolverBody& bodyB) const
			{
				const float denominator = EffectiveMass(bodyA, bodyB);
				if (denominator <= 1e-9f) return;
				const float impulse = -Velocity(bodyA, bodyB) / denominator;
				Reactions.Add(LinearAxis * impulse, CoupleB * impulse);
				ApplyVelocityImpulse(bodyA, LinearAxis * -impulse, Vector3::Zero());
				ApplyVelocityImpulse(bodyB, LinearAxis * impulse, Vector3::Zero());
				ApplyAngularVelocityImpulse(bodyA, AngularA * impulse);
				ApplyAngularVelocityImpulse(bodyB, AngularB * impulse);
			}

			void SolveDrive(JointImpulseAccumulator& Reactions, const JointSolverBody& A, const JointSolverBody& B, const ScalarJointDrive& Drive,
				float Position, float Dt, float& Lambda) const
			{
				if (!Drive.Settings.IsActive() || !std::isfinite(Dt) || Dt <= 0.0f) return;
				const double Weight = EffectiveMass(A, B);
				if (Weight <= DriveMinimumWeight) return;
				const DriveCoefficients C(Drive.Settings, Dt);
				const double Gamma = C.Gamma * (Drive.Settings.Acceleration ? Weight : 1.0);
				const double Error = (double)Position - Drive.TargetPosition;
				const double Residual = Velocity(A, B) + C.Beta * Error - C.TargetWeight * Drive.TargetVelocity + Gamma * Lambda;
				const float Next = (float)std::max(-C.Limit, std::min(C.Limit, Lambda - Residual / (Weight + Gamma)));
				const float Delta = Next - Lambda;
				Lambda = Next;
				Reactions.Add(LinearAxis * Delta, CoupleB * Delta);
				ApplyVelocityImpulse(A, LinearAxis * -Delta, Vector3::Zero());
				ApplyVelocityImpulse(B, LinearAxis * Delta, Vector3::Zero());
				ApplyAngularVelocityImpulse(A, AngularA * Delta);
				ApplyAngularVelocityImpulse(B, AngularB * Delta);
			}
		};

		ConstraintRow MakePointRow(const JointSolverBody& bodyA, const JointSolverBody& bodyB,
			const Vector3& anchorA, const Vector3& anchorB, const Vector3& axis, bool rotatingFrame)
		{
			ConstraintRow row;
			row.LinearAxis = axis;
			// A local axis rotates with body A. Its derivative contributes
			// (axis x (anchorB - anchorA)) to body A's angular Jacobian.
			row.AngularA = -(rotatingFrame ? anchorB - bodyA->X : anchorA - bodyA->X).Cross(axis);
			row.AngularB = (anchorB - bodyB->X).Cross(axis);
			return row;
		}

		ConstraintRow MakeAngularRow(const Vector3& axis)
		{
			ConstraintRow row;
			row.LinearAxis = Vector3::Zero();
			row.AngularA = -axis;
			row.AngularB = axis;
			row.CoupleB = axis;
			return row;
		}

		template<typename Measure>
		void SolvePositionRange(JointImpulseAccumulator& Reactions, const JointSolverBody& bodyA, const JointSolverBody& bodyB, float minimum, float maximum,
			float alpha, float& equalityLambda, float& lowerLambda, float& upperLambda, float maximumError, const Measure& measure)
		{
			ConstraintRow row;
			float position;
			measure(row, position);
			if (alpha <= 0.0f)
			{
				row.SolvePosition(Reactions, bodyA, bodyB, Clamp(position - Clamp(position, minimum, maximum), -maximumError, maximumError));
				return;
			}
			if (maximum - minimum <= 1e-6f)
			{
				row.SolvePosition(Reactions, bodyA, bodyB, position - minimum, alpha, &equalityLambda);
				return;
			}
			// Each side has its own nonnegative multiplier. An inactive side still
			// solves when necessary to release an impulse accumulated earlier this step.
			row.SolvePosition(Reactions, bodyA, bodyB, position - minimum, alpha, &lowerLambda, true);
			measure(row, position);
			row.LinearAxis = -row.LinearAxis;
			row.AngularA = -row.AngularA;
			row.AngularB = -row.AngularB;
			row.CoupleB = -row.CoupleB;
			row.SolvePosition(Reactions, bodyA, bodyB, maximum - position, alpha, &upperLambda, true);
		}

		void SolveLocalLinearPosition(JointImpulseAccumulator& Reactions, const JointSolverBody& bodyA, const JointSolverBody& bodyB, const Vector3& localAnchorA,
			const Vector3& localAnchorB, const Vector3& localAxisA, float minimum, float maximum,
			float alpha, float& equalityLambda, float& lowerLambda, float& upperLambda)
		{
			SolvePositionRange(Reactions, bodyA, bodyB, minimum, maximum, alpha, equalityLambda, lowerLambda, upperLambda, 0.5f,
				[&](ConstraintRow& row, float& position)
				{
					const Vector3 anchorA = bodyA->BodyLocalToWorld(localAnchorA);
					const Vector3 anchorB = bodyB->BodyLocalToWorld(localAnchorB);
					const Vector3 axis = bodyA->BodyLocalToWorldDirection(localAxisA);
					position = (anchorB - anchorA).Dot(axis);
					row = MakePointRow(bodyA, bodyB, anchorA, anchorB, axis, true);
				});
		}

		void SolveLocalLinearVelocity(JointImpulseAccumulator& Reactions, const JointSolverBody& bodyA, const JointSolverBody& bodyB, const Vector3& localAnchorA,
			const Vector3& localAnchorB, const Vector3& localAxisA, float minimum, float maximum)
		{
			const Vector3 anchorA = bodyA->BodyLocalToWorld(localAnchorA);
			const Vector3 anchorB = bodyB->BodyLocalToWorld(localAnchorB);
			const Vector3 axis = bodyA->BodyLocalToWorldDirection(localAxisA);
			const ConstraintRow row = MakePointRow(bodyA, bodyB, anchorA, anchorB, axis, true);
			if (IsLimitActive((anchorB - anchorA).Dot(axis), row.Velocity(bodyA, bodyB), minimum, maximum))
			{
				row.SolveVelocity(Reactions, bodyA, bodyB);
			}
		}

		void RemoveRelativeAngularVelocity(JointImpulseAccumulator& Reactions, const JointSolverBody& bodyA, const JointSolverBody& bodyB)
		{
			Matrix3 effectiveMass = GetInverseInertia(bodyA) + GetInverseInertia(bodyB);
			Vector3 impulse;
			if (effectiveMass.Solve(GetAngularVelocity(bodyA) - GetAngularVelocity(bodyB), impulse))
			{
				Reactions.Add(Vector3::Zero(), impulse);
				ApplyAngularVelocityImpulse(bodyA, -impulse);
				ApplyAngularVelocityImpulse(bodyB, impulse);
			}
		}

		Vector3 AxisAlignmentAxis(const Vector3& axisA, const Vector3& axisB)
		{
			const Vector3 cross = axisA.Cross(axisB);
			if (cross.SquareLength() > 1e-12f) return cross.SafeUnit();
			Vector3 perpendicular;
			Vector3 unused;
			axisA.GetPerpVectors(perpendicular, unused);
			return perpendicular;
		}

		void SolveSoftAxisAlignment(JointImpulseAccumulator& Reactions, const JointSolverBody& bodyA, const JointSolverBody& bodyB, const Vector3& localAxisA,
			const Vector3& localAxisB, float alpha, Vector3& lambda)
		{
			Vector3 basis[2];
			localAxisA.GetPerpVectors(basis[0], basis[1]);
			for (int i = 0; i < 2; ++i)
			{
				const Vector3 axisB = bodyA->WorldToBodyLocalDirection(bodyB->BodyLocalToWorldDirection(localAxisB)).SafeUnit();
				const Vector3 cross = localAxisA.Cross(axisB);
				const float sine = cross.Length();
				const float cosine = Clamp(localAxisA.Dot(axisB), -1.0f, 1.0f);
				const float angle = std::atan2(sine, cosine);
				const Vector3 axis = sine > 1e-6f ? cross / sine : basis[0];
				const float error = axis.Dot(basis[i]) * angle;
				Vector3 gradient;
				if (sine < 1e-4f)
				{
					// At the antipode the shortest swing is ambiguous. The fixed tangent
					// basis selects a deterministic path until the logarithm is regular.
					gradient = basis[i];
				}
				else
				{
					const float factor = angle / sine;
					const float derivative = (sine - angle * cosine) / (sine * sine);
					gradient = (basis[i] * cosine - localAxisA * axisB.Dot(basis[i])) * factor
						+ axis * (derivative * cross.Dot(basis[i]));
				}
				MakeAngularRow(bodyA->BodyLocalToWorldDirection(gradient)).SolvePosition(Reactions, bodyA, bodyB, error, alpha, &lambda[i]);
			}
		}

		void GetAngularLimitState(const JointSolverBody& bodyA, const JointSolverBody& bodyB, const Quaternion& reference,
			Vector3& angles, Vector3 (&axes)[3])
		{
			const Quaternion frame = (bodyA->Q * reference).Unit();
			const Quaternion relative = frame.Conjugate() * bodyB->Q;
			relative.ToEuler(angles.y, angles.z, angles.x);
			const float cosineYaw = std::cos(angles.y);
			const float sineYaw = std::sin(angles.y);
			const float cosinePitch = std::max(1e-4f, std::cos(angles.x));
			const float sinePitch = std::sin(angles.x);
			// Dual axes convert relative angular velocity into Y-X-Z Euler rates.
			// A fixed reference orientation rotates this basis with the joint frame.
			axes[0] = frame * Vector3(cosineYaw, 0.0f, -sineYaw);
			axes[1] = frame * Vector3(sineYaw * sinePitch / cosinePitch, 1.0f, cosineYaw * sinePitch / cosinePitch);
			axes[2] = frame * Vector3(sineYaw / cosinePitch, 0.0f, cosineYaw / cosinePitch);
		}

		void SolveSlerpDrive(JointImpulseAccumulator& Reactions, const JointSolverBody& A, const JointSolverBody& B, const Quaternion& Frame,
			const SixDOFDriveSettings& Drive, float Dt, Vector3& Lambda)
		{
			if (!Drive.Slerp.IsActive() || !std::isfinite(Dt) || Dt <= 0.0f) return;
			Quaternion Difference = (B->Q * (Frame * Drive.OrientationTarget).Conjugate()).Unit();
			if (Difference.w < 0.0f) Difference = -Difference;
			const Vector3 Imaginary(Difference.x, Difference.y, Difference.z);
			const float Sine = Imaginary.Length();
			const Vector3 Error = Sine > 0.0f ? Imaginary * (2.0f * std::atan2(Sine, Difference.w) / Sine) : Vector3::Zero();
			const Vector3 Velocity = GetAngularVelocity(B) - GetAngularVelocity(A);
			const DriveCoefficients C(Drive.Slerp, Dt);
			const Vector3 Target = Frame * Drive.AngularVelocityTarget;
			const Matrix3 Weight = GetInverseInertia(A) + GetInverseInertia(B);
			const double Scale = std::max(Weight(0, 0), std::max(Weight(1, 1), Weight(2, 2)));
			if (Scale <= DriveMinimumWeight) return;
			double Matrix[9], Values[3], Vectors[9];
			for (int I = 0; I < 3; ++I)
				for (int J = 0; J < 3; ++J) Matrix[I * 3 + J] = Weight(I, J) / Scale;
			Maths::LinearAlgebra::SymmetricEigenSolver<double>()(Matrix, 3, Values, Vectors);
			double Components[3] = {};
			for (int Mode = 0; Mode < 3; ++Mode)
			{
				if (Values[Mode] <= DriveRelativeRankTolerance * Values[0]) continue;
				const Vector3 Axis((float)Vectors[Mode], (float)Vectors[3 + Mode], (float)Vectors[6 + Mode]);
				const double InverseMass = Values[Mode] * Scale;
				const double Gamma = C.Gamma * (Drive.Slerp.Acceleration ? InverseMass : 1.0);
				const double Old = Lambda.Dot(Axis);
				const double Residual = Velocity.Dot(Axis) + C.Beta * Error.Dot(Axis) - C.TargetWeight * Target.Dot(Axis) + Gamma * Old;
				const double Impulse = Old - Residual / (InverseMass + Gamma);
				for (int I = 0; I < 3; ++I) Components[I] += Axis[I] * Impulse;
			}
			const double Length = std::sqrt(Components[0] * Components[0] + Components[1] * Components[1] + Components[2] * Components[2]);
			const double ClampScale = Length > C.Limit ? C.Limit / Length : 1.0;
			const Vector3 Next((float)(Components[0] * ClampScale), (float)(Components[1] * ClampScale), (float)(Components[2] * ClampScale));
			const Vector3 Delta = Next - Lambda;
			Lambda = Next;
			Reactions.Add(Vector3::Zero(), Delta);
			ApplyAngularVelocityImpulse(A, -Delta); ApplyAngularVelocityImpulse(B, Delta);
		}
	}

	void JointImpulseAccumulator::Clear(bool Enabled)
	{
		m_Enabled = Enabled;
		for (int Axis = 0; Axis < 3; ++Axis) m_Linear[Axis] = m_Angular[Axis] = 0.0;
	}
	void JointImpulseAccumulator::Add(const Vector3& Linear, const Vector3& Angular)
	{
		if (!m_Enabled) return;
		for (int Axis = 0; Axis < 3; ++Axis)
		{
			m_Linear[Axis] += Linear[Axis]; m_Angular[Axis] += Angular[Axis];
		}
	}
	void JointImpulseAccumulator::Accumulate(const JointImpulseAccumulator& Other, double Scale)
	{
		if (!m_Enabled || !Other.m_Enabled) return;
		for (int Axis = 0; Axis < 3; ++Axis)
		{
			m_Linear[Axis] += Other.m_Linear[Axis] * Scale;
			m_Angular[Axis] += Other.m_Angular[Axis] * Scale;
		}
	}
	JointReaction JointImpulseAccumulator::ToReaction(double Scale) const
	{
		JointReaction Result;
		for (int Axis = 0; Axis < 3; ++Axis)
		{
			Result.Force[Axis] = (float)(m_Linear[Axis] * Scale);
			Result.Torque[Axis] = (float)(m_Angular[Axis] * Scale);
		}
		return Result;
	}
	bool JointBreakSettings::IsValid() const
	{
		return std::isfinite(Force) && Force >= 0.0f && std::isfinite(Torque) && Torque >= 0.0f;
	}
	bool PositionConstraint::IsSolverActive() const
	{
		if (!IsEnabled()) return false;
		bool Managed = false;
		for (RigidBody* Body : { GetBodyA(), GetBodyB() })
		{
			auto* Dynamic = Body ? Body->CastDynamic() : nullptr;
			if (!Dynamic) continue;
			Managed = Managed || Dynamic->IslandSleepManaged;
			if (!Dynamic->Freezing && (!Dynamic->IslandSleepManaged || !Dynamic->Sleeping)) return true;
		}
		return !Managed;
	}
	void PositionConstraint::SetEnabled(bool Enabled)
	{
		if (m_Enabled == Enabled) return;
		m_Enabled = Enabled; ResetMultipliers();
		m_StepOpen = false; m_Reaction = JointReaction();
		WakeBody(m_BodyA); WakeBody(m_BodyB);
	}
	void PositionConstraint::SetBreakSettings(const JointBreakSettings& Settings)
	{
		if (!Settings.IsValid()) throw std::invalid_argument("Invalid joint break settings");
		m_BreakSettings = Settings;
		WakeBody(m_BodyA); WakeBody(m_BodyB);
	}
	void PositionConstraint::Repair()
	{
		m_Broken = false; m_Enabled = true; m_StepOpen = m_FrameOpen = false;
		m_Reaction = JointReaction(); ResetMultipliers();
		m_PositionImpulses.Clear(false); m_VelocityImpulses.Clear(false);
		WakeBody(m_BodyA); WakeBody(m_BodyB);
	}
	// Substeps reset their XPBD multipliers and drive force budgets independently.
	// Reports sum physical impulse over the outer frame: velocity lambda contributes
	// directly, while a positional multiplier contributes lambda / substepDt.
	// Compare break thresholds only after this sum is divided by the full frame Dt.
	void PositionConstraint::BeginFrame(float Dt)
	{
		m_FrameDt = std::isfinite(Dt) && Dt > 0 ? Dt : 0;
		m_FrameOpen = m_FrameDt > 0;
		m_FrameImpulses.Clear(m_FrameOpen);
	}
	bool PositionConstraint::EndFrame(bool EnableBreaking)
	{
		if (!m_FrameOpen) return false;
		m_FrameOpen = false;
		m_Reaction = m_FrameImpulses.ToReaction(1.0 / m_FrameDt);
		return CheckBreak(EnableBreaking);
	}
	bool PositionConstraint::EndStep(bool EnableBreaking)
	{
		if (!m_StepOpen) return false;
		m_StepOpen = false;
		m_Reaction = JointReaction();
		if (m_StepDt <= 0.0f || !IsEnabled()) return false;
		const double InverseDt = 1.0 / m_StepDt;
		if (m_FrameOpen)
		{
			m_FrameImpulses.Accumulate(m_VelocityImpulses, 1);
			m_FrameImpulses.Accumulate(m_PositionImpulses, InverseDt);
			m_Reaction = m_FrameImpulses.ToReaction(1.0 / m_FrameDt);
			return false;
		}
		m_Reaction = m_VelocityImpulses.ToReaction(InverseDt);
		const JointReaction Position = m_PositionImpulses.ToReaction(InverseDt * InverseDt);
		m_Reaction.Force += Position.Force; m_Reaction.Torque += Position.Torque;
		return CheckBreak(EnableBreaking);
	}
	bool PositionConstraint::CheckBreak(bool EnableBreaking)
	{
		if (!IsEnabled()) return false;
		auto Exceeds = [](const Vector3& Load, float Limit)
		{
			const double Squared = (double)Load.x * Load.x + (double)Load.y * Load.y + (double)Load.z * Load.z;
			return Squared > (double)Limit * Limit;
		};
		if (EnableBreaking && m_BreakSettings.Enabled
			&& (Exceeds(m_Reaction.Force, m_BreakSettings.Force) || Exceeds(m_Reaction.Torque, m_BreakSettings.Torque)))
		{
			m_Broken = true;
			WakeBody(m_BodyA); WakeBody(m_BodyB);
			return true;
		}
		return false;
	}

	bool JointDriveSettings::IsValid() const
	{
		return std::isfinite(Stiffness) && Stiffness >= 0.0f && std::isfinite(Damping) && Damping >= 0.0f
			&& std::isfinite(MaxForce) && MaxForce >= 0.0f;
	}
	bool ScalarJointDrive::IsValid() const
	{
		return Settings.IsValid() && std::isfinite(TargetPosition) && std::isfinite(TargetVelocity);
	}
	bool SixDOFDriveSettings::IsValid() const
	{
		for (int Axis = 0; Axis < 3; ++Axis) if (!Linear[Axis].IsValid() || !Angular[Axis].IsValid()) return false;
		return (AngularMode == AngularDriveMode::Axes || AngularMode == AngularDriveMode::Slerp)
			&& Slerp.IsValid() && FiniteDriveVector(AngularVelocityTarget)
			&& std::isfinite(OrientationTarget.SquareLength())
			&& std::abs(OrientationTarget.SquareLength() - 1.0f) <= DriveQuaternionNormTolerance;
	}
	bool SixDOFDriveSettings::IsActive() const
	{
		for (int Axis = 0; Axis < 3; ++Axis)
			if (Linear[Axis].Settings.IsActive() || (AngularMode == AngularDriveMode::Axes && Angular[Axis].Settings.IsActive())) return true;
		return AngularMode == AngularDriveMode::Slerp && Slerp.IsActive();
	}

	PositionConstraint::PositionConstraint(PositionConstraintType type, RigidBody* bodyA, RigidBody* bodyB)
		: m_Type(type)
		, m_BodyA(bodyA)
		, m_BodyB(bodyB)
		, m_Enabled(true)
	{
	}

	void PositionConstraint::ResetMultipliers()
	{
		m_LinearDriveImpulse = m_AngularDriveImpulse = Vector3::Zero();
		m_LinearLambda = m_AngularLambda = Vector3::Zero();
		m_LinearLowerLambda = m_LinearUpperLambda = Vector3::Zero();
		m_AngularLowerLambda = m_AngularUpperLambda = Vector3::Zero();
	}

	void PositionConstraint::SetCompliance(float linear, float angular)
	{
		DrivesChanged();
		m_LinearCompliance = std::isfinite(linear) ? std::max(0.0f, linear) : 0.0f;
		m_AngularCompliance = std::isfinite(angular) ? std::max(0.0f, angular) : 0.0f;
		m_LinearAlpha = m_AngularAlpha = 0.0f;
		ResetMultipliers();
	}

	void PositionConstraint::DrivesChanged()
	{
		m_LinearDriveImpulse = m_AngularDriveImpulse = Vector3::Zero();
		WakeBody(m_BodyA); WakeBody(m_BodyB);
	}

	void PositionConstraint::BeginStep(float dt, bool enableXPBD, bool enableDrives, bool positionReactions)
	{
		ResetMultipliers();
		m_StepDt = std::isfinite(dt) && dt > 0.0f ? dt : 0.0f;
		m_StepOpen = true;
		m_PositionImpulses.Clear(m_StepDt > 0.0f && IsEnabled() && (enableXPBD || positionReactions));
		m_VelocityImpulses.Clear(m_StepDt > 0.0f && IsEnabled());
		m_DrivesEnabled = enableDrives;
		m_LinearAlpha = m_AngularAlpha = 0.0f;
		if (!enableXPBD || !std::isfinite(dt) || dt <= 0.0f) return;
		const double inverseDtSquared = 1.0 / ((double)dt * dt);
		const double maximum = std::numeric_limits<float>::max();
		m_LinearAlpha = (float)std::min(maximum, m_LinearCompliance * inverseDtSquared);
		m_AngularAlpha = (float)std::min(maximum, m_AngularCompliance * inverseDtSquared);
	}

	void PositionConstraint::Stabilize(float Dt, float ScaleA, float ScaleB, bool Positions)
	{
		if (!IsSolverActive() || m_LinearCompliance > 0 || m_AngularCompliance > 0) return;
		if (!std::isfinite(ScaleA) || !std::isfinite(ScaleB) || ScaleA < 0 || ScaleA > 1 || ScaleB < 0 || ScaleB > 1)
			throw std::invalid_argument("Invalid joint stabilization mass scale");
		// This is an explicitly nonphysical projection. Do not spend motor impulse
		// budgets or report these conditioned row multipliers as physical reactions.
		struct Restore
		{
			PositionConstraint& Joint;
			JointImpulseAccumulator Position, Velocity;
			bool Drives; float A, B;
			~Restore()
			{
				Joint.m_PositionImpulses = Position; Joint.m_VelocityImpulses = Velocity;
				Joint.m_DrivesEnabled = Drives;
				Joint.m_BodyA.InverseMassScale = A; Joint.m_BodyB.InverseMassScale = B;
			}
		} Guard{ *this, m_PositionImpulses, m_VelocityImpulses, m_DrivesEnabled, m_BodyA.InverseMassScale, m_BodyB.InverseMassScale };
		m_BodyA.InverseMassScale = ScaleA; m_BodyB.InverseMassScale = ScaleB;
		m_DrivesEnabled = false; m_PositionImpulses.Clear(false); m_VelocityImpulses.Clear(false);
		if (Positions) SolvePosition(Dt); else SolveVelocity(Dt);
	}

	void PositionConstraint::SolveVelocity(float dt)
	{
		(void)dt;
	}

	PointConstraint::PointConstraint(RigidBody* bodyA, RigidBody* bodyB, const Vector3& worldAnchor)
		: PositionConstraint(PositionConstraintType::Point, bodyA, bodyB)
		, m_LocalAnchorA(bodyA->WorldToBodyLocal(worldAnchor))
		, m_LocalAnchorB(bodyB->WorldToBodyLocal(worldAnchor))
	{
	}

	Vector3 PointConstraint::GetWorldAnchorA() const { return m_BodyA->BodyLocalToWorld(m_LocalAnchorA); }
	Vector3 PointConstraint::GetWorldAnchorB() const { return m_BodyB->BodyLocalToWorld(m_LocalAnchorB); }

	void PointConstraint::SolvePosition(float dt)
	{
		if (!IsSolverActive()) return;
		(void)dt;
		const Vector3 anchorA = GetWorldAnchorA();
		const Vector3 anchorB = GetWorldAnchorB();
		ApplyPointPositionCorrection(m_PositionImpulses, m_BodyA, m_BodyB, anchorA, anchorB, anchorB - anchorA, m_LinearAlpha, &m_LinearLambda);
	}

	void PointConstraint::SolveVelocity(float dt)
	{
		if (!IsSolverActive()) return;
		(void)dt;
		if (HasSoftLinearRows()) return;
		RemoveRelativePointVelocity(m_VelocityImpulses, m_BodyA, m_BodyB, GetWorldAnchorA(), GetWorldAnchorB());
	}

	FixedConstraint::FixedConstraint(RigidBody* bodyA, RigidBody* bodyB)
		: PositionConstraint(PositionConstraintType::Fixed, bodyA, bodyB)
		, m_RelativePosition(bodyA->Q.Conjugate() * (bodyB->X - bodyA->X))
		, m_RelativeRotation(bodyA->Q.Conjugate() * bodyB->Q)
	{
	}

	void FixedConstraint::SolvePosition(float dt)
	{
		if (!IsSolverActive()) return;
		(void)dt;
		const Vector3 targetPositionB = m_BodyA->X + m_BodyA->Q * m_RelativePosition;
		ApplyPointPositionCorrection(m_PositionImpulses, m_BodyA, m_BodyB, targetPositionB, m_BodyB->X, m_BodyB->X - targetPositionB, m_LinearAlpha, &m_LinearLambda);
		SolveRelativeRotation(m_PositionImpulses, m_BodyA, m_BodyB, m_RelativeRotation, m_AngularAlpha, &m_AngularLambda);
	}

	void FixedConstraint::SolveVelocity(float dt)
	{
		if (!IsSolverActive()) return;
		(void)dt;
		const Vector3 targetPositionB = m_BodyA->X + m_BodyA->Q * m_RelativePosition;
		if (!HasSoftLinearRows()) RemoveRelativePointVelocity(m_VelocityImpulses, m_BodyA, m_BodyB, targetPositionB, m_BodyB->X);
		if (!HasSoftAngularRows()) RemoveRelativeAngularVelocity(m_VelocityImpulses, m_BodyA, m_BodyB);
	}

	DistanceConstraint::DistanceConstraint(RigidBody* bodyA, RigidBody* bodyB, const Vector3& worldAnchorA, const Vector3& worldAnchorB, float minDistance, float maxDistance)
		: PositionConstraint(PositionConstraintType::Distance, bodyA, bodyB)
		, m_LocalAnchorA(bodyA->WorldToBodyLocal(worldAnchorA))
		, m_LocalAnchorB(bodyB->WorldToBodyLocal(worldAnchorB))
		, m_MinDistance(std::max(0.0f, minDistance))
		, m_MaxDistance(std::max(m_MinDistance, maxDistance))
	{
	}

	float DistanceConstraint::GetCurrentDistance() const
	{
		return (m_BodyB->BodyLocalToWorld(m_LocalAnchorB) - m_BodyA->BodyLocalToWorld(m_LocalAnchorA)).Length();
	}

	void DistanceConstraint::SetDistanceLimits(float minDistance, float maxDistance)
	{
		DrivesChanged();
		ResetMultipliers();
		m_MinDistance = std::max(0.0f, minDistance);
		m_MaxDistance = std::max(m_MinDistance, maxDistance);
	}

	void DistanceConstraint::SolvePosition(float dt)
	{
		if (!IsSolverActive()) return;
		(void)dt;
		SolvePositionRange(m_PositionImpulses, m_BodyA, m_BodyB, m_MinDistance, m_MaxDistance, m_LinearAlpha,
			m_LinearLambda.x, m_LinearLowerLambda.x, m_LinearUpperLambda.x, 0.5f,
			[&](ConstraintRow& row, float& position)
			{
				const Vector3 anchorA = m_BodyA->BodyLocalToWorld(m_LocalAnchorA);
				const Vector3 anchorB = m_BodyB->BodyLocalToWorld(m_LocalAnchorB);
				const Vector3 delta = anchorB - anchorA;
				position = delta.Length();
				const Vector3 axis = position > 1e-7f ? delta / position : m_BodyA->BodyLocalToWorldDirection(Vector3::UnitX());
				row = MakePointRow(m_BodyA, m_BodyB, anchorA, anchorB, axis, false);
			});
	}

	void DistanceConstraint::SolveVelocity(float dt)
	{
		if (!IsSolverActive()) return;
		(void)dt;
		if (HasSoftLinearRows()) return;
		const Vector3 anchorA = m_BodyA->BodyLocalToWorld(m_LocalAnchorA);
		const Vector3 anchorB = m_BodyB->BodyLocalToWorld(m_LocalAnchorB);
		const Vector3 delta = anchorB - anchorA;
		const float distance = delta.Length();
		const Vector3 axis = distance > 1e-7f ? delta / distance : m_BodyA->BodyLocalToWorldDirection(Vector3::UnitX());
		const ConstraintRow row = MakePointRow(m_BodyA, m_BodyB, anchorA, anchorB, axis, false);
		if (m_MaxDistance <= 1e-7f)
		{
			RemoveRelativePointVelocity(m_VelocityImpulses, m_BodyA, m_BodyB, anchorA, anchorB);
		}
		else if (IsLimitActive(distance, row.Velocity(m_BodyA, m_BodyB), m_MinDistance, m_MaxDistance))
		{
			row.SolveVelocity(m_VelocityImpulses, m_BodyA, m_BodyB);
		}
	}

	HingeConstraint::HingeConstraint(RigidBody* bodyA, RigidBody* bodyB, const Vector3& worldAnchor, const Vector3& worldAxis, float minAngle, float maxAngle)
		: PointConstraint(bodyA, bodyB, worldAnchor)
		, m_LocalAxisA(bodyA->WorldToBodyLocalDirection(ValidAxis(worldAxis)))
		, m_LocalAxisB(bodyB->WorldToBodyLocalDirection(ValidAxis(worldAxis)))
		, m_MinAngle(minAngle)
		, m_MaxAngle(std::max(minAngle, maxAngle))
	{
		m_Type = PositionConstraintType::Hinge;
		SetAngleLimits(minAngle, maxAngle);
		Vector3 reference;
		Vector3 unused;
		ValidAxis(worldAxis).GetPerpVectors(reference, unused);
		m_LocalReferenceA = bodyA->WorldToBodyLocalDirection(reference);
		m_LocalReferenceB = bodyB->WorldToBodyLocalDirection(reference);
	}

	float HingeConstraint::GetCurrentAngle() const
	{
		const Vector3 axis = m_BodyA->BodyLocalToWorldDirection(m_LocalAxisA).SafeUnit();
		const Vector3 referenceA = m_BodyA->BodyLocalToWorldDirection(m_LocalReferenceA).SafeUnit();
		const Vector3 referenceB = m_BodyB->BodyLocalToWorldDirection(m_LocalReferenceB).SafeUnit();
		return SignedAngle(referenceA, referenceB, axis);
	}

	void HingeConstraint::SetAngleLimits(float minAngle, float maxAngle)
	{
		DrivesChanged();
		ResetMultipliers();
		m_MinAngle = std::max(-PI, std::min(PI, minAngle));
		m_MaxAngle = std::max(m_MinAngle, std::max(-PI, std::min(PI, maxAngle)));
	}

	void HingeConstraint::SolvePosition(float dt)
	{
		if (!IsSolverActive()) return;
		PointConstraint::SolvePosition(dt);
		if (HasSoftAngularRows())
		{
			SolveSoftAxisAlignment(m_PositionImpulses, m_BodyA, m_BodyB, m_LocalAxisA, m_LocalAxisB, m_AngularAlpha, m_AngularLambda);
		}
		else
		{
			const Vector3 axisA = m_BodyA->BodyLocalToWorldDirection(m_LocalAxisA).SafeUnit();
			const Vector3 axisB = m_BodyB->BodyLocalToWorldDirection(m_LocalAxisB).SafeUnit();
			const float alignmentAngle = std::atan2(axisA.Cross(axisB).Length(), Clamp(axisA.Dot(axisB), -1.0f, 1.0f));
			MakeAngularRow(AxisAlignmentAxis(axisA, axisB)).SolvePosition(m_PositionImpulses, m_BodyA, m_BodyB, alignmentAngle);
		}
		SolvePositionRange(m_PositionImpulses, m_BodyA, m_BodyB, m_MinAngle, m_MaxAngle, m_AngularAlpha,
			m_AngularLambda.z, m_AngularLowerLambda.z, m_AngularUpperLambda.z, std::numeric_limits<float>::max(),
			[&](ConstraintRow& row, float& position)
			{
				position = GetCurrentAngle();
				row = MakeAngularRow(m_BodyA->BodyLocalToWorldDirection(m_LocalAxisA));
			});
	}

	void HingeConstraint::SolveVelocity(float dt)
	{
		if (!IsSolverActive()) return;
		if (HasActiveDrive()) MakeAngularRow(m_BodyA->BodyLocalToWorldDirection(m_LocalAxisA)).SolveDrive(m_VelocityImpulses,
			m_BodyA, m_BodyB, m_Drive, GetCurrentAngle(), dt, m_AngularDriveImpulse.x);
		PointConstraint::SolveVelocity(dt);
		if (HasSoftAngularRows()) return;
		const Vector3 axisA = m_BodyA->BodyLocalToWorldDirection(m_LocalAxisA).SafeUnit();
		const Vector3 axisB = m_BodyB->BodyLocalToWorldDirection(m_LocalAxisB).SafeUnit();
		Vector3 axis = (axisA + axisB).SafeUnit();
		if (axis.IsZero())
		{
			axis = axisA;
		}
		Vector3 perpendicularA;
		Vector3 perpendicularB;
		axis.GetPerpVectors(perpendicularA, perpendicularB);
		RemoveRelativeAngularVelocityAlongAxis(m_VelocityImpulses, m_BodyA, m_BodyB, perpendicularA);
		RemoveRelativeAngularVelocityAlongAxis(m_VelocityImpulses, m_BodyA, m_BodyB, perpendicularB);

		const Vector3 relativeAngularVelocity = GetAngularVelocity(m_BodyB) - GetAngularVelocity(m_BodyA);
		const float angle = GetCurrentAngle();
		const float hingeSpeed = relativeAngularVelocity.Dot(axis);
		if (IsLimitActive(angle, hingeSpeed, m_MinAngle, m_MaxAngle))
		{
			RemoveRelativeAngularVelocityAlongAxis(m_VelocityImpulses, m_BodyA, m_BodyB, axis);
		}
	}

	SliderConstraint::SliderConstraint(RigidBody* bodyA, RigidBody* bodyB, const Vector3& worldAnchor, const Vector3& worldAxis, float minDistance, float maxDistance)
		: PositionConstraint(PositionConstraintType::Slider, bodyA, bodyB)
		, m_LocalAnchorA(bodyA->WorldToBodyLocal(worldAnchor))
		, m_LocalAnchorB(bodyB->WorldToBodyLocal(worldAnchor))
		, m_LocalAxisA(bodyA->WorldToBodyLocalDirection(ValidAxis(worldAxis)))
		, m_RelativeRotation(bodyA->Q.Conjugate() * bodyB->Q)
		, m_MinDistance(minDistance)
		, m_MaxDistance(std::max(minDistance, maxDistance))
	{
	}

	float SliderConstraint::GetCurrentPosition() const
	{
		const Vector3 axis = m_BodyA->BodyLocalToWorldDirection(m_LocalAxisA).SafeUnit();
		return (m_BodyB->BodyLocalToWorld(m_LocalAnchorB) - m_BodyA->BodyLocalToWorld(m_LocalAnchorA)).Dot(axis);
	}

	void SliderConstraint::SetDistanceLimits(float minDistance, float maxDistance)
	{
		DrivesChanged();
		ResetMultipliers();
		m_MinDistance = minDistance;
		m_MaxDistance = std::max(minDistance, maxDistance);
	}

	void SliderConstraint::SolvePosition(float dt)
	{
		if (!IsSolverActive()) return;
		(void)dt;
		Vector3 perpendicularA;
		Vector3 perpendicularB;
		m_LocalAxisA.GetPerpVectors(perpendicularA, perpendicularB);
		SolveLocalLinearPosition(m_PositionImpulses, m_BodyA, m_BodyB, m_LocalAnchorA, m_LocalAnchorB, perpendicularA, 0.0f, 0.0f,
			m_LinearAlpha, m_LinearLambda.x, m_LinearLowerLambda.x, m_LinearUpperLambda.x);
		SolveLocalLinearPosition(m_PositionImpulses, m_BodyA, m_BodyB, m_LocalAnchorA, m_LocalAnchorB, perpendicularB, 0.0f, 0.0f,
			m_LinearAlpha, m_LinearLambda.y, m_LinearLowerLambda.y, m_LinearUpperLambda.y);
		SolveLocalLinearPosition(m_PositionImpulses, m_BodyA, m_BodyB, m_LocalAnchorA, m_LocalAnchorB, m_LocalAxisA, m_MinDistance, m_MaxDistance,
			m_LinearAlpha, m_LinearLambda.z, m_LinearLowerLambda.z, m_LinearUpperLambda.z);
		SolveRelativeRotation(m_PositionImpulses, m_BodyA, m_BodyB, m_RelativeRotation, m_AngularAlpha, &m_AngularLambda);
	}

	void SliderConstraint::SolveVelocity(float dt)
	{
		if (!IsSolverActive()) return;
		if (HasActiveDrive())
		{
			const Vector3 A = m_BodyA->BodyLocalToWorld(m_LocalAnchorA), B = m_BodyB->BodyLocalToWorld(m_LocalAnchorB);
			const Vector3 Axis = m_BodyA->BodyLocalToWorldDirection(m_LocalAxisA);
			MakePointRow(m_BodyA, m_BodyB, A, B, Axis, true).SolveDrive(m_VelocityImpulses,
				m_BodyA, m_BodyB, m_Drive, (B - A).Dot(Axis), dt, m_LinearDriveImpulse.x);
		}
		Vector3 perpendicularA;
		Vector3 perpendicularB;
		m_LocalAxisA.GetPerpVectors(perpendicularA, perpendicularB);
		if (!HasSoftLinearRows())
		{
			SolveLocalLinearVelocity(m_VelocityImpulses, m_BodyA, m_BodyB, m_LocalAnchorA, m_LocalAnchorB, perpendicularA, 0.0f, 0.0f);
			SolveLocalLinearVelocity(m_VelocityImpulses, m_BodyA, m_BodyB, m_LocalAnchorA, m_LocalAnchorB, perpendicularB, 0.0f, 0.0f);
			SolveLocalLinearVelocity(m_VelocityImpulses, m_BodyA, m_BodyB, m_LocalAnchorA, m_LocalAnchorB, m_LocalAxisA, m_MinDistance, m_MaxDistance);
		}
		if (!HasSoftAngularRows()) RemoveRelativeAngularVelocity(m_VelocityImpulses, m_BodyA, m_BodyB);
	}

	ConeConstraint::ConeConstraint(RigidBody* bodyA, RigidBody* bodyB, const Vector3& worldAnchor, const Vector3& worldAxisA, const Vector3& worldAxisB, float maxAngle)
		: PointConstraint(bodyA, bodyB, worldAnchor)
		, m_LocalAxisA(bodyA->WorldToBodyLocalDirection(ValidAxis(worldAxisA)))
		, m_LocalAxisB(bodyB->WorldToBodyLocalDirection(ValidAxis(worldAxisB)))
		, m_MaxAngle(Clamp(maxAngle, 0.0f, PI))
	{
		m_Type = PositionConstraintType::Cone;
	}

	float ConeConstraint::GetCurrentAngle() const
	{
		const Vector3 axisA = m_BodyA->BodyLocalToWorldDirection(m_LocalAxisA).SafeUnit();
		const Vector3 axisB = m_BodyB->BodyLocalToWorldDirection(m_LocalAxisB).SafeUnit();
		return std::acos(Clamp(axisA.Dot(axisB), -1.0f, 1.0f));
	}

	void ConeConstraint::SetMaxAngle(float maxAngle)
	{
		DrivesChanged();
		ResetMultipliers();
		m_MaxAngle = Clamp(maxAngle, 0.0f, PI);
	}

	void ConeConstraint::SolvePosition(float dt)
	{
		if (!IsSolverActive()) return;
		PointConstraint::SolvePosition(dt);
		if (HasSoftAngularRows() && m_MaxAngle <= 1e-6f)
		{
			SolveSoftAxisAlignment(m_PositionImpulses, m_BodyA, m_BodyB, m_LocalAxisA, m_LocalAxisB, m_AngularAlpha, m_AngularLambda);
			return;
		}
		const Vector3 axisA = m_BodyA->BodyLocalToWorldDirection(m_LocalAxisA).SafeUnit();
		const Vector3 axisB = m_BodyB->BodyLocalToWorldDirection(m_LocalAxisB).SafeUnit();
		const float angle = GetCurrentAngle();
		if (HasSoftAngularRows())
		{
			MakeAngularRow(-AxisAlignmentAxis(axisA, axisB)).SolvePosition(m_PositionImpulses, m_BodyA, m_BodyB,
				m_MaxAngle - angle, m_AngularAlpha, &m_AngularUpperLambda.x, true);
		}
		else if (angle > m_MaxAngle)
		{
			MakeAngularRow(AxisAlignmentAxis(axisA, axisB)).SolvePosition(m_PositionImpulses, m_BodyA, m_BodyB, angle - m_MaxAngle);
		}
	}

	void ConeConstraint::SolveVelocity(float dt)
	{
		if (!IsSolverActive()) return;
		PointConstraint::SolveVelocity(dt);
		if (HasSoftAngularRows()) return;
		const Vector3 axisA = m_BodyA->BodyLocalToWorldDirection(m_LocalAxisA).SafeUnit();
		const Vector3 axisB = m_BodyB->BodyLocalToWorldDirection(m_LocalAxisB).SafeUnit();
		if (m_MaxAngle <= 1e-6f)
		{
			Vector3 perpendicularA;
			Vector3 perpendicularB;
			axisA.GetPerpVectors(perpendicularA, perpendicularB);
			RemoveRelativeAngularVelocityAlongAxis(m_VelocityImpulses, m_BodyA, m_BodyB, perpendicularA);
			RemoveRelativeAngularVelocityAlongAxis(m_VelocityImpulses, m_BodyA, m_BodyB, perpendicularB);
		}
		else if (GetCurrentAngle() >= m_MaxAngle - 1e-5f)
		{
			const Vector3 limitAxis = AxisAlignmentAxis(axisA, axisB);
			if ((GetAngularVelocity(m_BodyB) - GetAngularVelocity(m_BodyA)).Dot(limitAxis) > 0.0f)
			{
				RemoveRelativeAngularVelocityAlongAxis(m_VelocityImpulses, m_BodyA, m_BodyB, limitAxis);
			}
		}
	}

	SixDOFConstraintSettings::SixDOFConstraintSettings()
		: LinearMin(Vector3::Zero())
		, LinearMax(Vector3::Zero())
		, AngularMin(Vector3::Zero())
		, AngularMax(Vector3::Zero())
	{
	}

	SixDOFConstraint::SixDOFConstraint(RigidBody* bodyA, RigidBody* bodyB, const Vector3& worldAnchor, const SixDOFConstraintSettings& settings)
		: PositionConstraint(PositionConstraintType::SixDOF, bodyA, bodyB)
		, m_LocalAnchorA(bodyA->WorldToBodyLocal(worldAnchor))
		, m_LocalAnchorB(bodyB->WorldToBodyLocal(worldAnchor))
		, m_ReferenceRotation(bodyA->Q.Conjugate() * bodyB->Q)
		, m_Settings(settings)
	{
		SetSettings(settings);
	}

	void SixDOFConstraint::SetSettings(const SixDOFConstraintSettings& settings)
	{
		DrivesChanged();
		ResetMultipliers();
		m_Settings = settings;
		for (int axis = 0; axis < 3; ++axis)
		{
			if (m_Settings.LinearMax[axis] < m_Settings.LinearMin[axis])
			{
				m_Settings.LinearMax[axis] = m_Settings.LinearMin[axis];
			}
			m_Settings.AngularMin[axis] = Clamp(m_Settings.AngularMin[axis], -PI, PI);
			m_Settings.AngularMax[axis] = Clamp(m_Settings.AngularMax[axis], m_Settings.AngularMin[axis], PI);
		}
	}

	void SixDOFConstraint::SolvePosition(float dt)
	{
		if (!IsSolverActive()) return;
		(void)dt;
		for (int axis = 0; axis < 3; ++axis)
		{
			Vector3 localAxis = Vector3::Zero();
			localAxis[axis] = 1.0f;
			SolveLocalLinearPosition(m_PositionImpulses, m_BodyA, m_BodyB, m_LocalAnchorA, m_LocalAnchorB, localAxis,
				m_Settings.LinearMin[axis], m_Settings.LinearMax[axis], m_LinearAlpha,
				m_LinearLambda[axis], m_LinearLowerLambda[axis], m_LinearUpperLambda[axis]);
		}
		if ((m_Settings.AngularMax - m_Settings.AngularMin).SquareLength() <= 1e-12f)
		{
			Quaternion target;
			target.FromEuler(m_Settings.AngularMin.y, m_Settings.AngularMin.z, m_Settings.AngularMin.x);
			SolveRelativeRotation(m_PositionImpulses, m_BodyA, m_BodyB, m_ReferenceRotation * target, m_AngularAlpha, &m_AngularLambda);
			return;
		}
		for (int axis = 0; axis < 3; ++axis)
		{
			SolvePositionRange(m_PositionImpulses, m_BodyA, m_BodyB, m_Settings.AngularMin[axis], m_Settings.AngularMax[axis], m_AngularAlpha,
				m_AngularLambda[axis], m_AngularLowerLambda[axis], m_AngularUpperLambda[axis], std::numeric_limits<float>::max(),
				[&](ConstraintRow& row, float& position)
				{
					Vector3 angles;
					Vector3 axes[3];
					GetAngularLimitState(m_BodyA, m_BodyB, m_ReferenceRotation, angles, axes);
					position = angles[axis];
					row = MakeAngularRow(axes[axis]);
				});
		}
	}

	void SixDOFConstraint::SolveVelocity(float dt)
	{
		if (!IsSolverActive()) return;
		if (HasActiveDrive())
		{
			const Vector3 A = m_BodyA->BodyLocalToWorld(m_LocalAnchorA), B = m_BodyB->BodyLocalToWorld(m_LocalAnchorB);
			for (int Axis = 0; Axis < 3; ++Axis)
			{
				Vector3 Local = Vector3::Zero(); Local[Axis] = 1.0f;
				const Vector3 Direction = m_BodyA->BodyLocalToWorldDirection(Local);
				MakePointRow(m_BodyA, m_BodyB, A, B, Direction, true).SolveDrive(m_VelocityImpulses, m_BodyA, m_BodyB,
					m_Drives.Linear[Axis], (B - A).Dot(Direction), dt, m_LinearDriveImpulse[Axis]);
			}
			if (m_Drives.AngularMode == AngularDriveMode::Slerp)
				SolveSlerpDrive(m_VelocityImpulses, m_BodyA, m_BodyB, (m_BodyA->Q * m_ReferenceRotation).Unit(), m_Drives, dt, m_AngularDriveImpulse);
			else
			{
				Vector3 Angles, Axes[3];
				GetAngularLimitState(m_BodyA, m_BodyB, m_ReferenceRotation, Angles, Axes);
				for (int Axis = 0; Axis < 3; ++Axis) MakeAngularRow(Axes[Axis]).SolveDrive(m_VelocityImpulses,
					m_BodyA, m_BodyB, m_Drives.Angular[Axis], Angles[Axis], dt, m_AngularDriveImpulse[Axis]);
			}
		}
		for (int axis = 0; axis < 3 && !HasSoftLinearRows(); ++axis)
		{
			Vector3 localAxis = Vector3::Zero();
			localAxis[axis] = 1.0f;
			SolveLocalLinearVelocity(m_VelocityImpulses, m_BodyA, m_BodyB, m_LocalAnchorA, m_LocalAnchorB, localAxis,
				m_Settings.LinearMin[axis], m_Settings.LinearMax[axis]);
		}
		if (HasSoftAngularRows()) return;
		if ((m_Settings.AngularMax - m_Settings.AngularMin).SquareLength() <= 1e-12f)
		{
			RemoveRelativeAngularVelocity(m_VelocityImpulses, m_BodyA, m_BodyB);
			return;
		}
		Vector3 angles;
		Vector3 axes[3];
		GetAngularLimitState(m_BodyA, m_BodyB, m_ReferenceRotation, angles, axes);
		for (int axis = 0; axis < 3; ++axis)
		{
			const ConstraintRow row = MakeAngularRow(axes[axis]);
			if (IsLimitActive(angles[axis], row.Velocity(m_BodyA, m_BodyB),
				m_Settings.AngularMin[axis], m_Settings.AngularMax[axis]))
			{
				row.SolveVelocity(m_VelocityImpulses, m_BodyA, m_BodyB);
			}
		}
	}

	void HingeConstraint::SetDrive(const ScalarJointDrive& Drive)
	{
		if (!Drive.IsValid()) throw std::invalid_argument("Invalid hinge drive");
		m_Drive = Drive; DrivesChanged();
	}
	void SliderConstraint::SetDrive(const ScalarJointDrive& Drive)
	{
		if (!Drive.IsValid()) throw std::invalid_argument("Invalid slider drive");
		m_Drive = Drive; DrivesChanged();
	}
	void SixDOFConstraint::SetDrives(const SixDOFDriveSettings& Drives)
	{
		if (!Drives.IsValid()) throw std::invalid_argument("Invalid six-DOF drives");
		m_Drives = Drives; m_Drives.OrientationTarget = Drives.OrientationTarget.Unit(); DrivesChanged();
	}
	/* A pose drive has no geometric limit or position projection. It adds bounded
	   muscle/servo forces to the same velocity graph as contacts and passive joints.
	   Targets are body COM poses relative to A's COM frame, with their derivatives
	   in that frame. The rotating-frame linear Jacobian includes A's lever arm, so
	   parent-relative drives preserve internal linear/angular impulse balance.
	   A stationary reference instead gives a world-space servo. Orientation uses
	   the existing shortest-arc SLERP block and a torque-norm cap, without imposing
	   artificial Euler limits at +/-pi. Force caps are shared across solver sweeps. */
	bool PoseDriveSettings::IsValid() const
	{
		return Linear.IsValid() && Angular.IsValid() && FiniteDriveVector(Target.pos)
			&& std::isfinite(Target.quat.SquareLength()) && Target.quat.SquareLength() > 0
			&& FiniteDriveVector(LinearVelocityTarget) && FiniteDriveVector(AngularVelocityTarget);
	}
	PoseDriveConstraint::PoseDriveConstraint(RigidBody* Reference, RigidBody* Body)
		: PositionConstraint(PositionConstraintType::PoseDrive, Reference, Body)
	{
		m_Drive.Target = Transform(Reference->X, Reference->Q).TransformInv(Transform(Body->X, Body->Q));
	}
	void PoseDriveConstraint::SetDrive(const PoseDriveSettings& Settings)
	{
		if (!Settings.IsValid()) throw std::invalid_argument("Invalid pose drive settings");
		m_Drive = Settings; m_Drive.Target.quat = Settings.Target.quat.Unit(); DrivesChanged();
	}
	void PoseDriveConstraint::SetReference(RigidBody* Reference)
	{
		if (Reference == GetBodyA()) return;
		WakeBody(m_BodyA); m_BodyA.Body = Reference; DrivesChanged();
	}
	void PoseDriveConstraint::SolveVelocity(float Dt)
	{
		if (!IsSolverActive() || !HasActiveDrive()) return;
		const Quaternion Frame = m_BodyA->Q;
		for (int Axis = 0; Axis < 3; ++Axis)
		{
			Vector3 Local = Vector3::Zero(); Local[Axis] = 1;
			const Vector3 Direction = Frame * Local;
			ScalarJointDrive Drive; Drive.Settings = m_Drive.Linear;
			Drive.TargetPosition = m_Drive.Target.pos[Axis]; Drive.TargetVelocity = m_Drive.LinearVelocityTarget[Axis];
			MakePointRow(m_BodyA, m_BodyB, m_BodyA->X, m_BodyB->X, Direction, true).SolveDrive(m_VelocityImpulses,
				m_BodyA, m_BodyB, Drive, (m_BodyB->X - m_BodyA->X).Dot(Direction), Dt, m_LinearDriveImpulse[Axis]);
		}
		SixDOFDriveSettings Angular; Angular.Slerp = m_Drive.Angular;
		Angular.OrientationTarget = m_Drive.Target.quat; Angular.AngularVelocityTarget = m_Drive.AngularVelocityTarget;
		SolveSlerpDrive(m_VelocityImpulses, m_BodyA, m_BodyB, Frame, Angular, Dt, m_AngularDriveImpulse);
	}
}
