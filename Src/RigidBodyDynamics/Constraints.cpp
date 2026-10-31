#include "Constraints.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "RigidBody.h"

namespace Riemann
{
	namespace
	{
		float LinearWeight(RigidBody* body)
		{
			RigidBodyDynamic* dynamicBody = body ? body->CastDynamic() : nullptr;
			return dynamicBody && !dynamicBody->Freezing ? dynamicBody->InvMass : 0.0f;
		}

		void WakeBody(RigidBody* body)
		{
			RigidBodyDynamic* dynamicBody = body ? body->CastDynamic() : nullptr;
			if (dynamicBody)
			{
				dynamicBody->Wakeup();
			}
		}

		void RotateBody(RigidBody* body, const Vector3& axis, float angle)
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

		Matrix3 GetInverseInertia(RigidBody* body)
		{
			RigidBodyDynamic* dynamicBody = body ? body->CastDynamic() : nullptr;
			return dynamicBody && !dynamicBody->Freezing ? dynamicBody->GetInverseInertia_WorldSpace() : Matrix3::Zero();
		}

		Vector3 ClampCorrection(const Vector3& correction, float maximumLength)
		{
			const float length = correction.Length();
			return length > maximumLength ? correction * (maximumLength / length) : correction;
		}

		Matrix3 BuildPointEffectiveMass(RigidBody* bodyA, RigidBody* bodyB, const Vector3& rA, const Vector3& rB)
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

		void ApplyPositionImpulse(RigidBody* body, const Vector3& impulse, const Vector3& r)
		{
			RigidBodyDynamic* dynamicBody = body ? body->CastDynamic() : nullptr;
			if (dynamicBody == nullptr || dynamicBody->Freezing)
			{
				return;
			}
			dynamicBody->X += impulse * dynamicBody->InvMass;
			const Vector3 angularCorrection = dynamicBody->GetInverseInertia_WorldSpace() * r.Cross(impulse);
			const float angle = angularCorrection.Length();
			if (angle > 1e-7f)
			{
				RotateBody(dynamicBody, angularCorrection / angle, angle);
			}
			WakeBody(dynamicBody);
		}

		void ApplyVelocityImpulse(RigidBody* body, const Vector3& impulse, const Vector3& r)
		{
			RigidBodyDynamic* dynamicBody = body ? body->CastDynamic() : nullptr;
			if (dynamicBody == nullptr || dynamicBody->Freezing)
			{
				return;
			}
			dynamicBody->V += impulse * dynamicBody->InvMass;
			dynamicBody->W += dynamicBody->GetInverseInertia_WorldSpace() * r.Cross(impulse);
			if (!impulse.IsZero()) dynamicBody->Wakeup();
		}

		Vector3 GetAngularVelocity(RigidBody* body)
		{
			if (!body || body->mRigidType == RigidType::Static)
			{
				return Vector3::Zero();
			}
			RigidBodyDynamic* dynamicBody = body->CastDynamic();
			return dynamicBody && dynamicBody->Freezing ? Vector3::Zero() : body->W;
		}

		Vector3 GetPointVelocity(RigidBody* body, const Vector3& worldPoint)
		{
			if (!body || body->mRigidType == RigidType::Static)
			{
				return Vector3::Zero();
			}
			RigidBodyDynamic* dynamicBody = body ? body->CastDynamic() : nullptr;
			return dynamicBody && dynamicBody->Freezing ? Vector3::Zero() : body->V + body->W.Cross(worldPoint - body->X);
		}

		void ApplyPointPositionCorrection(RigidBody* bodyA, RigidBody* bodyB, const Vector3& anchorA, const Vector3& anchorB,
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
		}

		void RemoveRelativePointVelocity(RigidBody* bodyA, RigidBody* bodyB, const Vector3& anchorA, const Vector3& anchorB)
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
		}

		void ApplyAngularVelocityImpulse(RigidBody* body, const Vector3& impulse)
		{
			RigidBodyDynamic* dynamicBody = body ? body->CastDynamic() : nullptr;
			if (dynamicBody && !dynamicBody->Freezing)
			{
				dynamicBody->W += dynamicBody->GetInverseInertia_WorldSpace() * impulse;
				if (!impulse.IsZero()) dynamicBody->Wakeup();
			}
		}

		void RemoveRelativeAngularVelocityAlongAxis(RigidBody* bodyA, RigidBody* bodyB, const Vector3& axis)
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
			ApplyAngularVelocityImpulse(bodyA, -impulse);
			ApplyAngularVelocityImpulse(bodyB, impulse);
		}

		void ApplyAngularCorrection(RigidBody* bodyA, RigidBody* bodyB, const Vector3& axisAngle,
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

		void SolveRelativeRotation(RigidBody* bodyA, RigidBody* bodyB, const Quaternion& targetRelativeRotation,
			float alpha = 0.0f, Vector3* localLambda = nullptr)
		{
			const Quaternion currentRelative = bodyA->Q.Conjugate() * bodyB->Q;
			const Quaternion localCorrection = targetRelativeRotation * currentRelative.Conjugate();
			const Quaternion frame = bodyA->Q;
			Vector3 worldLambda = localLambda ? frame * *localLambda : Vector3::Zero();
			ApplyAngularCorrection(bodyA, bodyB, -(bodyA->Q * QuaternionAxisAngle(localCorrection)),
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

			float EffectiveMass(RigidBody* bodyA, RigidBody* bodyB) const
			{
				return (LinearWeight(bodyA) + LinearWeight(bodyB)) * LinearAxis.SquareLength()
					+ AngularA.Dot(GetInverseInertia(bodyA) * AngularA)
					+ AngularB.Dot(GetInverseInertia(bodyB) * AngularB);
			}

			float Velocity(RigidBody* bodyA, RigidBody* bodyB) const
			{
				return LinearAxis.Dot(GetPointVelocity(bodyB, bodyB->X) - GetPointVelocity(bodyA, bodyA->X))
					+ AngularA.Dot(GetAngularVelocity(bodyA)) + AngularB.Dot(GetAngularVelocity(bodyB));
			}

			void SolvePosition(RigidBody* bodyA, RigidBody* bodyB, float error,
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

			void SolveVelocity(RigidBody* bodyA, RigidBody* bodyB) const
			{
				const float denominator = EffectiveMass(bodyA, bodyB);
				if (denominator <= 1e-9f) return;
				const float impulse = -Velocity(bodyA, bodyB) / denominator;
				ApplyVelocityImpulse(bodyA, LinearAxis * -impulse, Vector3::Zero());
				ApplyVelocityImpulse(bodyB, LinearAxis * impulse, Vector3::Zero());
				ApplyAngularVelocityImpulse(bodyA, AngularA * impulse);
				ApplyAngularVelocityImpulse(bodyB, AngularB * impulse);
			}
		};

		ConstraintRow MakePointRow(RigidBody* bodyA, RigidBody* bodyB,
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
			return row;
		}

		template<typename Measure>
		void SolvePositionRange(RigidBody* bodyA, RigidBody* bodyB, float minimum, float maximum,
			float alpha, float& equalityLambda, float& lowerLambda, float& upperLambda, float maximumError, const Measure& measure)
		{
			ConstraintRow row;
			float position;
			measure(row, position);
			if (alpha <= 0.0f)
			{
				row.SolvePosition(bodyA, bodyB, Clamp(position - Clamp(position, minimum, maximum), -maximumError, maximumError));
				return;
			}
			if (maximum - minimum <= 1e-6f)
			{
				row.SolvePosition(bodyA, bodyB, position - minimum, alpha, &equalityLambda);
				return;
			}
			// Each side has its own nonnegative multiplier. An inactive side still
			// solves when necessary to release an impulse accumulated earlier this step.
			row.SolvePosition(bodyA, bodyB, position - minimum, alpha, &lowerLambda, true);
			measure(row, position);
			row.LinearAxis = -row.LinearAxis;
			row.AngularA = -row.AngularA;
			row.AngularB = -row.AngularB;
			row.SolvePosition(bodyA, bodyB, maximum - position, alpha, &upperLambda, true);
		}

		void SolveLocalLinearPosition(RigidBody* bodyA, RigidBody* bodyB, const Vector3& localAnchorA,
			const Vector3& localAnchorB, const Vector3& localAxisA, float minimum, float maximum,
			float alpha, float& equalityLambda, float& lowerLambda, float& upperLambda)
		{
			SolvePositionRange(bodyA, bodyB, minimum, maximum, alpha, equalityLambda, lowerLambda, upperLambda, 0.5f,
				[&](ConstraintRow& row, float& position)
				{
					const Vector3 anchorA = bodyA->BodyLocalToWorld(localAnchorA);
					const Vector3 anchorB = bodyB->BodyLocalToWorld(localAnchorB);
					const Vector3 axis = bodyA->BodyLocalToWorldDirection(localAxisA);
					position = (anchorB - anchorA).Dot(axis);
					row = MakePointRow(bodyA, bodyB, anchorA, anchorB, axis, true);
				});
		}

		void SolveLocalLinearVelocity(RigidBody* bodyA, RigidBody* bodyB, const Vector3& localAnchorA,
			const Vector3& localAnchorB, const Vector3& localAxisA, float minimum, float maximum)
		{
			const Vector3 anchorA = bodyA->BodyLocalToWorld(localAnchorA);
			const Vector3 anchorB = bodyB->BodyLocalToWorld(localAnchorB);
			const Vector3 axis = bodyA->BodyLocalToWorldDirection(localAxisA);
			const ConstraintRow row = MakePointRow(bodyA, bodyB, anchorA, anchorB, axis, true);
			if (IsLimitActive((anchorB - anchorA).Dot(axis), row.Velocity(bodyA, bodyB), minimum, maximum))
			{
				row.SolveVelocity(bodyA, bodyB);
			}
		}

		void RemoveRelativeAngularVelocity(RigidBody* bodyA, RigidBody* bodyB)
		{
			Matrix3 effectiveMass = GetInverseInertia(bodyA) + GetInverseInertia(bodyB);
			Vector3 impulse;
			if (effectiveMass.Solve(GetAngularVelocity(bodyA) - GetAngularVelocity(bodyB), impulse))
			{
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

		void SolveSoftAxisAlignment(RigidBody* bodyA, RigidBody* bodyB, const Vector3& localAxisA,
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
				MakeAngularRow(bodyA->BodyLocalToWorldDirection(gradient)).SolvePosition(bodyA, bodyB, error, alpha, &lambda[i]);
			}
		}

		void GetAngularLimitState(RigidBody* bodyA, RigidBody* bodyB, const Quaternion& reference,
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
		m_LinearLambda = m_AngularLambda = Vector3::Zero();
		m_LinearLowerLambda = m_LinearUpperLambda = Vector3::Zero();
		m_AngularLowerLambda = m_AngularUpperLambda = Vector3::Zero();
	}

	void PositionConstraint::SetCompliance(float linear, float angular)
	{
		m_LinearCompliance = std::isfinite(linear) ? std::max(0.0f, linear) : 0.0f;
		m_AngularCompliance = std::isfinite(angular) ? std::max(0.0f, angular) : 0.0f;
		m_LinearAlpha = m_AngularAlpha = 0.0f;
		ResetMultipliers();
	}

	void PositionConstraint::BeginStep(float dt, bool enableXPBD)
	{
		ResetMultipliers();
		m_LinearAlpha = m_AngularAlpha = 0.0f;
		if (!enableXPBD || !std::isfinite(dt) || dt <= 0.0f) return;
		const double inverseDtSquared = 1.0 / ((double)dt * dt);
		const double maximum = std::numeric_limits<float>::max();
		m_LinearAlpha = (float)std::min(maximum, m_LinearCompliance * inverseDtSquared);
		m_AngularAlpha = (float)std::min(maximum, m_AngularCompliance * inverseDtSquared);
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
		(void)dt;
		const Vector3 anchorA = GetWorldAnchorA();
		const Vector3 anchorB = GetWorldAnchorB();
		ApplyPointPositionCorrection(m_BodyA, m_BodyB, anchorA, anchorB, anchorB - anchorA, m_LinearAlpha, &m_LinearLambda);
	}

	void PointConstraint::SolveVelocity(float dt)
	{
		(void)dt;
		if (HasSoftLinearRows()) return;
		RemoveRelativePointVelocity(m_BodyA, m_BodyB, GetWorldAnchorA(), GetWorldAnchorB());
	}

	FixedConstraint::FixedConstraint(RigidBody* bodyA, RigidBody* bodyB)
		: PositionConstraint(PositionConstraintType::Fixed, bodyA, bodyB)
		, m_RelativePosition(bodyA->Q.Conjugate() * (bodyB->X - bodyA->X))
		, m_RelativeRotation(bodyA->Q.Conjugate() * bodyB->Q)
	{
	}

	void FixedConstraint::SolvePosition(float dt)
	{
		(void)dt;
		const Vector3 targetPositionB = m_BodyA->X + m_BodyA->Q * m_RelativePosition;
		ApplyPointPositionCorrection(m_BodyA, m_BodyB, targetPositionB, m_BodyB->X, m_BodyB->X - targetPositionB, m_LinearAlpha, &m_LinearLambda);
		SolveRelativeRotation(m_BodyA, m_BodyB, m_RelativeRotation, m_AngularAlpha, &m_AngularLambda);
	}

	void FixedConstraint::SolveVelocity(float dt)
	{
		(void)dt;
		const Vector3 targetPositionB = m_BodyA->X + m_BodyA->Q * m_RelativePosition;
		if (!HasSoftLinearRows()) RemoveRelativePointVelocity(m_BodyA, m_BodyB, targetPositionB, m_BodyB->X);
		if (!HasSoftAngularRows()) RemoveRelativeAngularVelocity(m_BodyA, m_BodyB);
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
		ResetMultipliers();
		m_MinDistance = std::max(0.0f, minDistance);
		m_MaxDistance = std::max(m_MinDistance, maxDistance);
	}

	void DistanceConstraint::SolvePosition(float dt)
	{
		(void)dt;
		SolvePositionRange(m_BodyA, m_BodyB, m_MinDistance, m_MaxDistance, m_LinearAlpha,
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
			RemoveRelativePointVelocity(m_BodyA, m_BodyB, anchorA, anchorB);
		}
		else if (IsLimitActive(distance, row.Velocity(m_BodyA, m_BodyB), m_MinDistance, m_MaxDistance))
		{
			row.SolveVelocity(m_BodyA, m_BodyB);
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
		ResetMultipliers();
		m_MinAngle = std::max(-PI, std::min(PI, minAngle));
		m_MaxAngle = std::max(m_MinAngle, std::max(-PI, std::min(PI, maxAngle)));
	}

	void HingeConstraint::SolvePosition(float dt)
	{
		PointConstraint::SolvePosition(dt);
		if (HasSoftAngularRows())
		{
			SolveSoftAxisAlignment(m_BodyA, m_BodyB, m_LocalAxisA, m_LocalAxisB, m_AngularAlpha, m_AngularLambda);
		}
		else
		{
			const Vector3 axisA = m_BodyA->BodyLocalToWorldDirection(m_LocalAxisA).SafeUnit();
			const Vector3 axisB = m_BodyB->BodyLocalToWorldDirection(m_LocalAxisB).SafeUnit();
			const float alignmentAngle = std::atan2(axisA.Cross(axisB).Length(), Clamp(axisA.Dot(axisB), -1.0f, 1.0f));
			MakeAngularRow(AxisAlignmentAxis(axisA, axisB)).SolvePosition(m_BodyA, m_BodyB, alignmentAngle);
		}
		SolvePositionRange(m_BodyA, m_BodyB, m_MinAngle, m_MaxAngle, m_AngularAlpha,
			m_AngularLambda.z, m_AngularLowerLambda.z, m_AngularUpperLambda.z, std::numeric_limits<float>::max(),
			[&](ConstraintRow& row, float& position)
			{
				position = GetCurrentAngle();
				row = MakeAngularRow(m_BodyA->BodyLocalToWorldDirection(m_LocalAxisA));
			});
	}

	void HingeConstraint::SolveVelocity(float dt)
	{
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
		RemoveRelativeAngularVelocityAlongAxis(m_BodyA, m_BodyB, perpendicularA);
		RemoveRelativeAngularVelocityAlongAxis(m_BodyA, m_BodyB, perpendicularB);

		const Vector3 relativeAngularVelocity = GetAngularVelocity(m_BodyB) - GetAngularVelocity(m_BodyA);
		const float angle = GetCurrentAngle();
		const float hingeSpeed = relativeAngularVelocity.Dot(axis);
		if (IsLimitActive(angle, hingeSpeed, m_MinAngle, m_MaxAngle))
		{
			RemoveRelativeAngularVelocityAlongAxis(m_BodyA, m_BodyB, axis);
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
		ResetMultipliers();
		m_MinDistance = minDistance;
		m_MaxDistance = std::max(minDistance, maxDistance);
	}

	void SliderConstraint::SolvePosition(float dt)
	{
		(void)dt;
		Vector3 perpendicularA;
		Vector3 perpendicularB;
		m_LocalAxisA.GetPerpVectors(perpendicularA, perpendicularB);
		SolveLocalLinearPosition(m_BodyA, m_BodyB, m_LocalAnchorA, m_LocalAnchorB, perpendicularA, 0.0f, 0.0f,
			m_LinearAlpha, m_LinearLambda.x, m_LinearLowerLambda.x, m_LinearUpperLambda.x);
		SolveLocalLinearPosition(m_BodyA, m_BodyB, m_LocalAnchorA, m_LocalAnchorB, perpendicularB, 0.0f, 0.0f,
			m_LinearAlpha, m_LinearLambda.y, m_LinearLowerLambda.y, m_LinearUpperLambda.y);
		SolveLocalLinearPosition(m_BodyA, m_BodyB, m_LocalAnchorA, m_LocalAnchorB, m_LocalAxisA, m_MinDistance, m_MaxDistance,
			m_LinearAlpha, m_LinearLambda.z, m_LinearLowerLambda.z, m_LinearUpperLambda.z);
		SolveRelativeRotation(m_BodyA, m_BodyB, m_RelativeRotation, m_AngularAlpha, &m_AngularLambda);
	}

	void SliderConstraint::SolveVelocity(float dt)
	{
		(void)dt;
		Vector3 perpendicularA;
		Vector3 perpendicularB;
		m_LocalAxisA.GetPerpVectors(perpendicularA, perpendicularB);
		if (!HasSoftLinearRows())
		{
			SolveLocalLinearVelocity(m_BodyA, m_BodyB, m_LocalAnchorA, m_LocalAnchorB, perpendicularA, 0.0f, 0.0f);
			SolveLocalLinearVelocity(m_BodyA, m_BodyB, m_LocalAnchorA, m_LocalAnchorB, perpendicularB, 0.0f, 0.0f);
			SolveLocalLinearVelocity(m_BodyA, m_BodyB, m_LocalAnchorA, m_LocalAnchorB, m_LocalAxisA, m_MinDistance, m_MaxDistance);
		}
		if (!HasSoftAngularRows()) RemoveRelativeAngularVelocity(m_BodyA, m_BodyB);
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
		ResetMultipliers();
		m_MaxAngle = Clamp(maxAngle, 0.0f, PI);
	}

	void ConeConstraint::SolvePosition(float dt)
	{
		PointConstraint::SolvePosition(dt);
		if (HasSoftAngularRows() && m_MaxAngle <= 1e-6f)
		{
			SolveSoftAxisAlignment(m_BodyA, m_BodyB, m_LocalAxisA, m_LocalAxisB, m_AngularAlpha, m_AngularLambda);
			return;
		}
		const Vector3 axisA = m_BodyA->BodyLocalToWorldDirection(m_LocalAxisA).SafeUnit();
		const Vector3 axisB = m_BodyB->BodyLocalToWorldDirection(m_LocalAxisB).SafeUnit();
		const float angle = GetCurrentAngle();
		if (HasSoftAngularRows())
		{
			MakeAngularRow(-AxisAlignmentAxis(axisA, axisB)).SolvePosition(m_BodyA, m_BodyB,
				m_MaxAngle - angle, m_AngularAlpha, &m_AngularUpperLambda.x, true);
		}
		else if (angle > m_MaxAngle)
		{
			MakeAngularRow(AxisAlignmentAxis(axisA, axisB)).SolvePosition(m_BodyA, m_BodyB, angle - m_MaxAngle);
		}
	}

	void ConeConstraint::SolveVelocity(float dt)
	{
		PointConstraint::SolveVelocity(dt);
		if (HasSoftAngularRows()) return;
		const Vector3 axisA = m_BodyA->BodyLocalToWorldDirection(m_LocalAxisA).SafeUnit();
		const Vector3 axisB = m_BodyB->BodyLocalToWorldDirection(m_LocalAxisB).SafeUnit();
		if (m_MaxAngle <= 1e-6f)
		{
			Vector3 perpendicularA;
			Vector3 perpendicularB;
			axisA.GetPerpVectors(perpendicularA, perpendicularB);
			RemoveRelativeAngularVelocityAlongAxis(m_BodyA, m_BodyB, perpendicularA);
			RemoveRelativeAngularVelocityAlongAxis(m_BodyA, m_BodyB, perpendicularB);
		}
		else if (GetCurrentAngle() >= m_MaxAngle - 1e-5f)
		{
			const Vector3 limitAxis = AxisAlignmentAxis(axisA, axisB);
			if ((GetAngularVelocity(m_BodyB) - GetAngularVelocity(m_BodyA)).Dot(limitAxis) > 0.0f)
			{
				RemoveRelativeAngularVelocityAlongAxis(m_BodyA, m_BodyB, limitAxis);
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
		(void)dt;
		for (int axis = 0; axis < 3; ++axis)
		{
			Vector3 localAxis = Vector3::Zero();
			localAxis[axis] = 1.0f;
			SolveLocalLinearPosition(m_BodyA, m_BodyB, m_LocalAnchorA, m_LocalAnchorB, localAxis,
				m_Settings.LinearMin[axis], m_Settings.LinearMax[axis], m_LinearAlpha,
				m_LinearLambda[axis], m_LinearLowerLambda[axis], m_LinearUpperLambda[axis]);
		}
		if ((m_Settings.AngularMax - m_Settings.AngularMin).SquareLength() <= 1e-12f)
		{
			Quaternion target;
			target.FromEuler(m_Settings.AngularMin.y, m_Settings.AngularMin.z, m_Settings.AngularMin.x);
			SolveRelativeRotation(m_BodyA, m_BodyB, m_ReferenceRotation * target, m_AngularAlpha, &m_AngularLambda);
			return;
		}
		for (int axis = 0; axis < 3; ++axis)
		{
			SolvePositionRange(m_BodyA, m_BodyB, m_Settings.AngularMin[axis], m_Settings.AngularMax[axis], m_AngularAlpha,
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
		(void)dt;
		for (int axis = 0; axis < 3 && !HasSoftLinearRows(); ++axis)
		{
			Vector3 localAxis = Vector3::Zero();
			localAxis[axis] = 1.0f;
			SolveLocalLinearVelocity(m_BodyA, m_BodyB, m_LocalAnchorA, m_LocalAnchorB, localAxis,
				m_Settings.LinearMin[axis], m_Settings.LinearMax[axis]);
		}
		if (HasSoftAngularRows()) return;
		if ((m_Settings.AngularMax - m_Settings.AngularMin).SquareLength() <= 1e-12f)
		{
			RemoveRelativeAngularVelocity(m_BodyA, m_BodyB);
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
				row.SolveVelocity(m_BodyA, m_BodyB);
			}
		}
	}
}
