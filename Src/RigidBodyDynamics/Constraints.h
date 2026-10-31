#pragma once

#include "../Maths/Quaternion.h"
#include "../Maths/Vector3.h"

namespace Riemann
{
	class RigidBody;

	enum class PositionConstraintType
	{
		Fixed,
		Point,
		Distance,
		Hinge,
		Slider,
		Cone,
		SixDOF,
	};

	class PositionConstraint
	{
	public:
		PositionConstraint(PositionConstraintType type, RigidBody* bodyA, RigidBody* bodyB);
		virtual ~PositionConstraint() {}

		PositionConstraintType GetType() const { return m_Type; }
		RigidBody* GetBodyA() const { return m_BodyA; }
		RigidBody* GetBodyB() const { return m_BodyB; }
		bool IsEnabled() const { return m_Enabled; }
		void SetEnabled(bool enabled) { m_Enabled = enabled; ResetMultipliers(); }

		void SetCompliance(float linear, float angular);
		float GetLinearCompliance() const { return m_LinearCompliance; }
		float GetAngularCompliance() const { return m_AngularCompliance; }
		// Reset multipliers once per simulation substep, before any joint solve.
		void BeginStep(float dt, bool enableXPBD);

		virtual void Setup() {}
		virtual void SolvePosition(float dt) = 0;
		virtual void SolveVelocity(float dt);
		void Solve() { SolvePosition(1.0f / 60.0f); }

	protected:
		void ResetMultipliers();
		bool HasSoftLinearRows() const { return m_LinearAlpha > 0.0f; }
		bool HasSoftAngularRows() const { return m_AngularAlpha > 0.0f; }

		float m_LinearCompliance = 0.0f;
		float m_AngularCompliance = 0.0f;
		float m_LinearAlpha = 0.0f;
		float m_AngularAlpha = 0.0f;
		Vector3 m_LinearLambda = Vector3::Zero();
		Vector3 m_AngularLambda = Vector3::Zero();
		Vector3 m_LinearLowerLambda = Vector3::Zero();
		Vector3 m_LinearUpperLambda = Vector3::Zero();
		Vector3 m_AngularLowerLambda = Vector3::Zero();
		Vector3 m_AngularUpperLambda = Vector3::Zero();
		PositionConstraintType m_Type;
		RigidBody* m_BodyA;
		RigidBody* m_BodyB;
		bool m_Enabled;
	};

	class PointConstraint : public PositionConstraint
	{
	public:
		PointConstraint(RigidBody* bodyA, RigidBody* bodyB, const Vector3& worldAnchor);
		virtual void SolvePosition(float dt) override;
		virtual void SolveVelocity(float dt) override;

		Vector3 GetWorldAnchorA() const;
		Vector3 GetWorldAnchorB() const;

	protected:
		Vector3 m_LocalAnchorA;
		Vector3 m_LocalAnchorB;
	};

	class FixedConstraint : public PositionConstraint
	{
	public:
		FixedConstraint(RigidBody* bodyA, RigidBody* bodyB);
		virtual void SolvePosition(float dt) override;
		virtual void SolveVelocity(float dt) override;

	private:
		Vector3 m_RelativePosition;
		Quaternion m_RelativeRotation;
	};

	class DistanceConstraint : public PositionConstraint
	{
	public:
		DistanceConstraint(RigidBody* bodyA, RigidBody* bodyB, const Vector3& worldAnchorA, const Vector3& worldAnchorB, float minDistance, float maxDistance);
		virtual void SolvePosition(float dt) override;
		virtual void SolveVelocity(float dt) override;

		float GetCurrentDistance() const;
		float GetMinDistance() const { return m_MinDistance; }
		float GetMaxDistance() const { return m_MaxDistance; }
		void SetDistanceLimits(float minDistance, float maxDistance);

	private:
		Vector3 m_LocalAnchorA;
		Vector3 m_LocalAnchorB;
		float m_MinDistance;
		float m_MaxDistance;
	};

	class HingeConstraint : public PointConstraint
	{
	public:
		HingeConstraint(RigidBody* bodyA, RigidBody* bodyB, const Vector3& worldAnchor, const Vector3& worldAxis, float minAngle, float maxAngle);
		virtual void SolvePosition(float dt) override;
		virtual void SolveVelocity(float dt) override;

		float GetCurrentAngle() const;
		float GetMinAngle() const { return m_MinAngle; }
		float GetMaxAngle() const { return m_MaxAngle; }
		void SetAngleLimits(float minAngle, float maxAngle);

	private:
		Vector3 m_LocalAxisA;
		Vector3 m_LocalAxisB;
		Vector3 m_LocalReferenceA;
		Vector3 m_LocalReferenceB;
		float m_MinAngle;
		float m_MaxAngle;
	};

	class SliderConstraint : public PositionConstraint
	{
	public:
		SliderConstraint(RigidBody* bodyA, RigidBody* bodyB, const Vector3& worldAnchor, const Vector3& worldAxis, float minDistance, float maxDistance);
		virtual void SolvePosition(float dt) override;
		virtual void SolveVelocity(float dt) override;

		float GetCurrentPosition() const;
		float GetMinDistance() const { return m_MinDistance; }
		float GetMaxDistance() const { return m_MaxDistance; }
		void SetDistanceLimits(float minDistance, float maxDistance);

	private:
		Vector3 m_LocalAnchorA;
		Vector3 m_LocalAnchorB;
		Vector3 m_LocalAxisA;
		Quaternion m_RelativeRotation;
		float m_MinDistance;
		float m_MaxDistance;
	};

	class ConeConstraint : public PointConstraint
	{
	public:
		ConeConstraint(RigidBody* bodyA, RigidBody* bodyB, const Vector3& worldAnchor, const Vector3& worldAxisA, const Vector3& worldAxisB, float maxAngle);
		virtual void SolvePosition(float dt) override;
		virtual void SolveVelocity(float dt) override;

		float GetCurrentAngle() const;
		float GetMaxAngle() const { return m_MaxAngle; }
		void SetMaxAngle(float maxAngle);

	private:
		Vector3 m_LocalAxisA;
		Vector3 m_LocalAxisB;
		float m_MaxAngle;
	};

	struct SixDOFConstraintSettings
	{
		SixDOFConstraintSettings();

		Vector3 LinearMin;
		Vector3 LinearMax;
		Vector3 AngularMin;
		Vector3 AngularMax;
	};

	class SixDOFConstraint : public PositionConstraint
	{
	public:
		SixDOFConstraint(RigidBody* bodyA, RigidBody* bodyB, const Vector3& worldAnchor, const SixDOFConstraintSettings& settings);
		virtual void SolvePosition(float dt) override;
		virtual void SolveVelocity(float dt) override;
		const SixDOFConstraintSettings& GetSettings() const { return m_Settings; }
		void SetSettings(const SixDOFConstraintSettings& settings);

	private:
		Vector3 m_LocalAnchorA;
		Vector3 m_LocalAnchorB;
		Quaternion m_ReferenceRotation;
		SixDOFConstraintSettings m_Settings;
	};
}
