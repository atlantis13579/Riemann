#include "Constraints.h"

#include <algorithm>
#include <cmath>

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

		void ApplyLinearCorrection(RigidBody* bodyA, RigidBody* bodyB, const Vector3& correction)
		{
			const float weightA = LinearWeight(bodyA);
			const float weightB = LinearWeight(bodyB);
			const float weightSum = weightA + weightB;
			if (weightSum <= kMinimumInvMass)
			{
				return;
			}
			if (weightA > 0.0f)
			{
				bodyA->X += correction * (weightA / weightSum);
				WakeBody(bodyA);
			}
			if (weightB > 0.0f)
			{
				bodyB->X -= correction * (weightB / weightSum);
				WakeBody(bodyB);
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
		}

		Vector3 GetPointVelocity(RigidBody* body, const Vector3& worldPoint)
		{
			RigidBodyDynamic* dynamicBody = body ? body->CastDynamic() : nullptr;
			return dynamicBody ? dynamicBody->V + dynamicBody->W.Cross(worldPoint - dynamicBody->X) : Vector3::Zero();
		}

		void ApplyPointPositionCorrection(RigidBody* bodyA, RigidBody* bodyB, const Vector3& anchorA, const Vector3& anchorB, const Vector3& error)
		{
			const Vector3 rA = anchorA - bodyA->X;
			const Vector3 rB = anchorB - bodyB->X;
			Matrix3 effectiveMass = BuildPointEffectiveMass(bodyA, bodyB, rA, rB);
			Vector3 impulse;
			if (!effectiveMass.Solve(-ClampCorrection(error, 0.5f), impulse))
			{
				return;
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

		void RemoveRelativePointVelocityAlongAxis(RigidBody* bodyA, RigidBody* bodyB, const Vector3& anchorA, const Vector3& anchorB, const Vector3& axis)
		{
			const Vector3 unitAxis = axis.SafeUnit();
			if (unitAxis.IsZero())
			{
				return;
			}
			const Vector3 rA = anchorA - bodyA->X;
			const Vector3 rB = anchorB - bodyB->X;
			const Matrix3 effectiveMass = BuildPointEffectiveMass(bodyA, bodyB, rA, rB);
			const float denominator = unitAxis.Dot(effectiveMass * unitAxis);
			if (denominator <= kMinimumInvMass)
			{
				return;
			}
			const float relativeSpeed = (GetPointVelocity(bodyB, anchorB) - GetPointVelocity(bodyA, anchorA)).Dot(unitAxis);
			const Vector3 impulse = unitAxis * (-relativeSpeed / denominator);
			ApplyVelocityImpulse(bodyA, -impulse, rA);
			ApplyVelocityImpulse(bodyB, impulse, rB);
		}

		void ApplyAngularVelocityImpulse(RigidBody* body, const Vector3& impulse)
		{
			RigidBodyDynamic* dynamicBody = body ? body->CastDynamic() : nullptr;
			if (dynamicBody && !dynamicBody->Freezing)
			{
				dynamicBody->W += dynamicBody->GetInverseInertia_WorldSpace() * impulse;
			}
		}

		void RemoveRelativeAngularVelocityAlongAxis(RigidBody* bodyA, RigidBody* bodyB, const Vector3& axis)
		{
			const Vector3 unitAxis = axis.SafeUnit();
			if (unitAxis.IsZero())
			{
				return;
			}
			RigidBodyDynamic* dynamicA = bodyA ? bodyA->CastDynamic() : nullptr;
			RigidBodyDynamic* dynamicB = bodyB ? bodyB->CastDynamic() : nullptr;
			const Matrix3 inverseInertia = GetInverseInertia(bodyA) + GetInverseInertia(bodyB);
			const float denominator = unitAxis.Dot(inverseInertia * unitAxis);
			if (denominator <= 1e-7f)
			{
				return;
			}
			const Vector3 relativeAngularVelocity = (dynamicB ? dynamicB->W : Vector3::Zero()) - (dynamicA ? dynamicA->W : Vector3::Zero());
			const Vector3 impulse = unitAxis * (-relativeAngularVelocity.Dot(unitAxis) / denominator);
			ApplyAngularVelocityImpulse(bodyA, -impulse);
			ApplyAngularVelocityImpulse(bodyB, impulse);
		}

		void ApplyAngularCorrection(RigidBody* bodyA, RigidBody* bodyB, const Vector3& axisAngle)
		{
			const Vector3 limitedCorrection = ClampCorrection(axisAngle, 0.2f);
			const float angle = limitedCorrection.Length();
			if (angle <= 1e-7f)
			{
				return;
			}
			Matrix3 inverseInertia = GetInverseInertia(bodyA) + GetInverseInertia(bodyB);
			Vector3 impulse;
			if (!inverseInertia.Solve(-limitedCorrection, impulse))
			{
				return;
			}
			const Vector3 correctionA = GetInverseInertia(bodyA) * -impulse;
			const Vector3 correctionB = GetInverseInertia(bodyB) * impulse;
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

		void SolveRelativeRotation(RigidBody* bodyA, RigidBody* bodyB, const Quaternion& targetRelativeRotation)
		{
			const Quaternion currentRelative = bodyA->Q.Conjugate() * bodyB->Q;
			const Quaternion localCorrection = targetRelativeRotation * currentRelative.Conjugate();
			ApplyAngularCorrection(bodyA, bodyB, -(bodyA->Q * QuaternionAxisAngle(localCorrection)));
		}

		void RemoveRelativeLinearVelocity(RigidBody* bodyA, RigidBody* bodyB, const Vector3& axes)
		{
			RigidBodyDynamic* dynamicA = bodyA ? bodyA->CastDynamic() : nullptr;
			RigidBodyDynamic* dynamicB = bodyB ? bodyB->CastDynamic() : nullptr;
			const float weightA = LinearWeight(bodyA);
			const float weightB = LinearWeight(bodyB);
			const float weightSum = weightA + weightB;
			if (weightSum <= kMinimumInvMass)
			{
				return;
			}
			const Vector3 velocityA = dynamicA ? dynamicA->V : Vector3::Zero();
			const Vector3 velocityB = dynamicB ? dynamicB->V : Vector3::Zero();
			const Vector3 correction = (velocityB - velocityA) * axes;
			if (dynamicA)
			{
				dynamicA->V += correction * (weightA / weightSum);
			}
			if (dynamicB)
			{
				dynamicB->V -= correction * (weightB / weightSum);
			}
		}

		void RemoveRelativeLinearVelocityAlongAxis(RigidBody* bodyA, RigidBody* bodyB, const Vector3& axis)
		{
			RigidBodyDynamic* dynamicA = bodyA ? bodyA->CastDynamic() : nullptr;
			RigidBodyDynamic* dynamicB = bodyB ? bodyB->CastDynamic() : nullptr;
			const float weightA = LinearWeight(bodyA);
			const float weightB = LinearWeight(bodyB);
			const float weightSum = weightA + weightB;
			if (weightSum <= kMinimumInvMass)
			{
				return;
			}
			const Vector3 velocityA = dynamicA ? dynamicA->V : Vector3::Zero();
			const Vector3 velocityB = dynamicB ? dynamicB->V : Vector3::Zero();
			const Vector3 unitAxis = axis.SafeUnit();
			const Vector3 correction = unitAxis * (velocityB - velocityA).Dot(unitAxis);
			if (dynamicA) dynamicA->V += correction * (weightA / weightSum);
			if (dynamicB) dynamicB->V -= correction * (weightB / weightSum);
		}

		void RemoveRelativeLinearVelocityPerpendicularToAxis(RigidBody* bodyA, RigidBody* bodyB, const Vector3& axis)
		{
			RigidBodyDynamic* dynamicA = bodyA ? bodyA->CastDynamic() : nullptr;
			RigidBodyDynamic* dynamicB = bodyB ? bodyB->CastDynamic() : nullptr;
			const float weightA = LinearWeight(bodyA);
			const float weightB = LinearWeight(bodyB);
			const float weightSum = weightA + weightB;
			if (weightSum <= kMinimumInvMass)
			{
				return;
			}
			const Vector3 velocityA = dynamicA ? dynamicA->V : Vector3::Zero();
			const Vector3 velocityB = dynamicB ? dynamicB->V : Vector3::Zero();
			const Vector3 relativeVelocity = velocityB - velocityA;
			const Vector3 unitAxis = axis.SafeUnit();
			const Vector3 correction = relativeVelocity - unitAxis * relativeVelocity.Dot(unitAxis);
			if (dynamicA) dynamicA->V += correction * (weightA / weightSum);
			if (dynamicB) dynamicB->V -= correction * (weightB / weightSum);
		}

		float SignedAngle(const Vector3& from, const Vector3& to, const Vector3& axis)
		{
			return std::atan2(axis.Dot(from.Cross(to)), std::max(-1.0f, std::min(1.0f, from.Dot(to))));
		}

		float Clamp(float value, float minValue, float maxValue)
		{
			return std::max(minValue, std::min(maxValue, value));
		}
	}

	PositionConstraint::PositionConstraint(PositionConstraintType type, RigidBody* bodyA, RigidBody* bodyB)
		: m_Type(type)
		, m_BodyA(bodyA)
		, m_BodyB(bodyB)
		, m_Enabled(true)
	{
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
		ApplyPointPositionCorrection(m_BodyA, m_BodyB, anchorA, anchorB, anchorB - anchorA);
	}

	void PointConstraint::SolveVelocity(float dt)
	{
		(void)dt;
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
		ApplyPointPositionCorrection(m_BodyA, m_BodyB, targetPositionB, m_BodyB->X, m_BodyB->X - targetPositionB);
		SolveRelativeRotation(m_BodyA, m_BodyB, m_RelativeRotation);
	}

	void FixedConstraint::SolveVelocity(float dt)
	{
		(void)dt;
		const Vector3 targetPositionB = m_BodyA->X + m_BodyA->Q * m_RelativePosition;
		RemoveRelativePointVelocity(m_BodyA, m_BodyB, targetPositionB, m_BodyB->X);
		RemoveRelativeAngularVelocityAlongAxis(m_BodyA, m_BodyB, Vector3::UnitX());
		RemoveRelativeAngularVelocityAlongAxis(m_BodyA, m_BodyB, Vector3::UnitY());
		RemoveRelativeAngularVelocityAlongAxis(m_BodyA, m_BodyB, Vector3::UnitZ());
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
		m_MinDistance = std::max(0.0f, minDistance);
		m_MaxDistance = std::max(m_MinDistance, maxDistance);
	}

	void DistanceConstraint::SolvePosition(float dt)
	{
		(void)dt;
		const Vector3 anchorA = m_BodyA->BodyLocalToWorld(m_LocalAnchorA);
		const Vector3 anchorB = m_BodyB->BodyLocalToWorld(m_LocalAnchorB);
		const Vector3 delta = anchorB - anchorA;
		const float distance = delta.Length();
		if (distance <= 1e-7f)
		{
			return;
		}
		const float targetDistance = Clamp(distance, m_MinDistance, m_MaxDistance);
		ApplyPointPositionCorrection(m_BodyA, m_BodyB, anchorA, anchorB, delta * ((distance - targetDistance) / distance));
	}

	void DistanceConstraint::SolveVelocity(float dt)
	{
		(void)dt;
		const Vector3 anchorA = m_BodyA->BodyLocalToWorld(m_LocalAnchorA);
		const Vector3 anchorB = m_BodyB->BodyLocalToWorld(m_LocalAnchorB);
		const Vector3 delta = anchorB - anchorA;
		const float distance = delta.Length();
		const Vector3 axis = delta.SafeUnit();
		const float relativeSpeed = (GetPointVelocity(m_BodyB, anchorB) - GetPointVelocity(m_BodyA, anchorA)).Dot(axis);
		const bool movingPastMinimum = distance <= m_MinDistance + 1e-5f && relativeSpeed < 0.0f;
		const bool movingPastMaximum = distance >= m_MaxDistance - 1e-5f && relativeSpeed > 0.0f;
		if (movingPastMinimum || movingPastMaximum)
		{
			RemoveRelativePointVelocityAlongAxis(m_BodyA, m_BodyB, anchorA, anchorB, axis);
		}
	}

	HingeConstraint::HingeConstraint(RigidBody* bodyA, RigidBody* bodyB, const Vector3& worldAnchor, const Vector3& worldAxis, float minAngle, float maxAngle)
		: PointConstraint(bodyA, bodyB, worldAnchor)
		, m_LocalAxisA(bodyA->WorldToBodyLocalDirection(worldAxis.SafeUnit()))
		, m_LocalAxisB(bodyB->WorldToBodyLocalDirection(worldAxis.SafeUnit()))
		, m_MinAngle(minAngle)
		, m_MaxAngle(std::max(minAngle, maxAngle))
	{
		m_Type = PositionConstraintType::Hinge;
		Vector3 reference;
		Vector3 unused;
		worldAxis.SafeUnit().GetPerpVectors(reference, unused);
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
		m_MinAngle = std::max(-PI, std::min(PI, minAngle));
		m_MaxAngle = std::max(m_MinAngle, std::max(-PI, std::min(PI, maxAngle)));
	}

	void HingeConstraint::SolvePosition(float dt)
	{
		PointConstraint::SolvePosition(dt);
		const Vector3 axisA = m_BodyA->BodyLocalToWorldDirection(m_LocalAxisA).SafeUnit();
		const Vector3 axisB = m_BodyB->BodyLocalToWorldDirection(m_LocalAxisB).SafeUnit();
		ApplyAngularCorrection(m_BodyA, m_BodyB, axisA.Cross(axisB));
		const float angle = GetCurrentAngle();
		const float targetAngle = Clamp(angle, m_MinAngle, m_MaxAngle);
		ApplyAngularCorrection(m_BodyA, m_BodyB, axisA * (angle - targetAngle));
	}

	void HingeConstraint::SolveVelocity(float dt)
	{
		PointConstraint::SolveVelocity(dt);
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

		RigidBodyDynamic* dynamicA = m_BodyA->CastDynamic();
		RigidBodyDynamic* dynamicB = m_BodyB->CastDynamic();
		const Vector3 relativeAngularVelocity = (dynamicB ? dynamicB->W : Vector3::Zero()) - (dynamicA ? dynamicA->W : Vector3::Zero());
		const float angle = GetCurrentAngle();
		const float hingeSpeed = relativeAngularVelocity.Dot(axis);
		if ((angle <= m_MinAngle + 1e-5f && hingeSpeed < 0.0f)
			|| (angle >= m_MaxAngle - 1e-5f && hingeSpeed > 0.0f))
		{
			RemoveRelativeAngularVelocityAlongAxis(m_BodyA, m_BodyB, axis);
		}
	}

	SliderConstraint::SliderConstraint(RigidBody* bodyA, RigidBody* bodyB, const Vector3& worldAnchor, const Vector3& worldAxis, float minDistance, float maxDistance)
		: PositionConstraint(PositionConstraintType::Slider, bodyA, bodyB)
		, m_LocalAnchorA(bodyA->WorldToBodyLocal(worldAnchor))
		, m_LocalAnchorB(bodyB->WorldToBodyLocal(worldAnchor))
		, m_LocalAxisA(bodyA->WorldToBodyLocalDirection(worldAxis.SafeUnit()))
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
		m_MinDistance = minDistance;
		m_MaxDistance = std::max(minDistance, maxDistance);
	}

	void SliderConstraint::SolvePosition(float dt)
	{
		(void)dt;
		const Vector3 anchorA = m_BodyA->BodyLocalToWorld(m_LocalAnchorA);
		const Vector3 anchorB = m_BodyB->BodyLocalToWorld(m_LocalAnchorB);
		const Vector3 axis = m_BodyA->BodyLocalToWorldDirection(m_LocalAxisA).SafeUnit();
		const Vector3 delta = anchorB - anchorA;
		const float position = delta.Dot(axis);
		const float targetPosition = Clamp(position, m_MinDistance, m_MaxDistance);
		ApplyPointPositionCorrection(m_BodyA, m_BodyB, anchorA, anchorB, delta - axis * targetPosition);
		SolveRelativeRotation(m_BodyA, m_BodyB, m_RelativeRotation);
	}

	void SliderConstraint::SolveVelocity(float dt)
	{
		(void)dt;
		const Vector3 axis = m_BodyA->BodyLocalToWorldDirection(m_LocalAxisA).SafeUnit();
		RemoveRelativeLinearVelocityPerpendicularToAxis(m_BodyA, m_BodyB, axis);
		RigidBodyDynamic* dynamicA = m_BodyA->CastDynamic();
		RigidBodyDynamic* dynamicB = m_BodyB->CastDynamic();
		const Vector3 velocityA = dynamicA ? dynamicA->V : Vector3::Zero();
		const Vector3 velocityB = dynamicB ? dynamicB->V : Vector3::Zero();
		const float relativeSpeed = (velocityB - velocityA).Dot(axis);
		const float position = GetCurrentPosition();
		if ((position <= m_MinDistance + 1.0e-5f && relativeSpeed < 0.0f)
			|| (position >= m_MaxDistance - 1.0e-5f && relativeSpeed > 0.0f))
		{
			RemoveRelativeLinearVelocityAlongAxis(m_BodyA, m_BodyB, axis);
		}
	}

	ConeConstraint::ConeConstraint(RigidBody* bodyA, RigidBody* bodyB, const Vector3& worldAnchor, const Vector3& worldAxisA, const Vector3& worldAxisB, float maxAngle)
		: PointConstraint(bodyA, bodyB, worldAnchor)
		, m_LocalAxisA(bodyA->WorldToBodyLocalDirection(worldAxisA.SafeUnit()))
		, m_LocalAxisB(bodyB->WorldToBodyLocalDirection(worldAxisB.SafeUnit()))
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
		m_MaxAngle = Clamp(maxAngle, 0.0f, PI);
	}

	void ConeConstraint::SolvePosition(float dt)
	{
		PointConstraint::SolvePosition(dt);
		const Vector3 axisA = m_BodyA->BodyLocalToWorldDirection(m_LocalAxisA).SafeUnit();
		const Vector3 axisB = m_BodyB->BodyLocalToWorldDirection(m_LocalAxisB).SafeUnit();
		const float angle = GetCurrentAngle();
		if (angle > m_MaxAngle)
		{
			ApplyAngularCorrection(m_BodyA, m_BodyB, axisA.Cross(axisB).SafeUnit() * (angle - m_MaxAngle));
		}
	}

	void ConeConstraint::SolveVelocity(float dt)
	{
		PointConstraint::SolveVelocity(dt);
		const Vector3 axisA = m_BodyA->BodyLocalToWorldDirection(m_LocalAxisA).SafeUnit();
		const Vector3 axisB = m_BodyB->BodyLocalToWorldDirection(m_LocalAxisB).SafeUnit();
		const Vector3 limitAxis = axisA.Cross(axisB).SafeUnit();
		if (!limitAxis.IsZero() && GetCurrentAngle() >= m_MaxAngle - 1e-5f)
		{
			RigidBodyDynamic* dynamicA = m_BodyA->CastDynamic();
			RigidBodyDynamic* dynamicB = m_BodyB->CastDynamic();
			const Vector3 relativeAngularVelocity = (dynamicB ? dynamicB->W : Vector3::Zero()) - (dynamicA ? dynamicA->W : Vector3::Zero());
			if (relativeAngularVelocity.Dot(limitAxis) > 0.0f)
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
	}

	void SixDOFConstraint::SetSettings(const SixDOFConstraintSettings& settings)
	{
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
		const Vector3 anchorA = m_BodyA->BodyLocalToWorld(m_LocalAnchorA);
		const Vector3 anchorB = m_BodyB->BodyLocalToWorld(m_LocalAnchorB);
		const Vector3 localDelta = m_BodyA->WorldToBodyLocalDirection(anchorB - anchorA);
		const Vector3 target(
			Clamp(localDelta.x, m_Settings.LinearMin.x, m_Settings.LinearMax.x),
			Clamp(localDelta.y, m_Settings.LinearMin.y, m_Settings.LinearMax.y),
			Clamp(localDelta.z, m_Settings.LinearMin.z, m_Settings.LinearMax.z));
		ApplyLinearCorrection(m_BodyA, m_BodyB, m_BodyA->BodyLocalToWorldDirection(localDelta - target));

		const Quaternion current = m_ReferenceRotation.Conjugate() * (m_BodyA->Q.Conjugate() * m_BodyB->Q);
		float yaw;
		float roll;
		float pitch;
		current.ToEuler(yaw, roll, pitch);
		const Vector3 angles(pitch, yaw, roll);
		const Vector3 targetAngles(
			Clamp(angles.x, m_Settings.AngularMin.x, m_Settings.AngularMax.x),
			Clamp(angles.y, m_Settings.AngularMin.y, m_Settings.AngularMax.y),
			Clamp(angles.z, m_Settings.AngularMin.z, m_Settings.AngularMax.z));
		ApplyAngularCorrection(m_BodyA, m_BodyB, m_BodyA->Q * (angles - targetAngles));
	}

	void SixDOFConstraint::SolveVelocity(float dt)
	{
		(void)dt;
		RigidBodyDynamic* dynamicA = m_BodyA->CastDynamic();
		RigidBodyDynamic* dynamicB = m_BodyB->CastDynamic();
		const float weightA = LinearWeight(m_BodyA);
		const float weightB = LinearWeight(m_BodyB);
		const float weightSum = weightA + weightB;
		if (weightSum <= kMinimumInvMass)
		{
			return;
		}
		const Vector3 velocityA = dynamicA ? dynamicA->V : Vector3::Zero();
		const Vector3 velocityB = dynamicB ? dynamicB->V : Vector3::Zero();
		const Vector3 localVelocity = m_BodyA->WorldToBodyLocalDirection(velocityB - velocityA);
		const Vector3 localPosition = m_BodyA->WorldToBodyLocalDirection(
			m_BodyB->BodyLocalToWorld(m_LocalAnchorB) - m_BodyA->BodyLocalToWorld(m_LocalAnchorA));
		Vector3 localCorrection = Vector3::Zero();
		for (int axis = 0; axis < 3; ++axis)
		{
			const bool locked = std::abs(m_Settings.LinearMax[axis] - m_Settings.LinearMin[axis]) <= 1e-6f;
			const bool movingPastMin = localPosition[axis] <= m_Settings.LinearMin[axis] + 1e-5f && localVelocity[axis] < 0.0f;
			const bool movingPastMax = localPosition[axis] >= m_Settings.LinearMax[axis] - 1e-5f && localVelocity[axis] > 0.0f;
			if (locked || movingPastMin || movingPastMax)
			{
				localCorrection[axis] = localVelocity[axis];
			}
		}
		const Vector3 correction = m_BodyA->BodyLocalToWorldDirection(localCorrection);
		if (dynamicA) dynamicA->V += correction * (weightA / weightSum);
		if (dynamicB) dynamicB->V -= correction * (weightB / weightSum);
	}
}
