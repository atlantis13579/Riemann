#pragma once

#include <limits>
#include <stdint.h>
#include "../Maths/Quaternion.h"
#include "../Maths/Vector3.h"
#include "../Maths/Transform.h"

namespace Riemann
{
	class RigidBody;

	struct JointReaction
	{
		// World-space load on body B at its joint attachment, averaged over the last step.
		Vector3 Force = Vector3::Zero(); // N; the force on A is its negative.
		Vector3 Torque = Vector3::Zero(); // Nm; excludes the moment of Force about B's COM.
	};

	struct JointBreakSettings
	{
		bool Enabled = false;
		float Force = std::numeric_limits<float>::max();
		float Torque = std::numeric_limits<float>::max();
		bool IsValid() const;
	};

	// Shared by the local row and block solvers; signed sums preserve unloading.
	class JointImpulseAccumulator
	{
	public:
		void Clear(bool Enabled);
		void Add(const Vector3& Linear, const Vector3& Angular);
		void Accumulate(const JointImpulseAccumulator& Other, double Scale);
		JointReaction ToReaction(double Scale) const;
	private:
		bool m_Enabled = false;
		double m_Linear[3] = {}, m_Angular[3] = {};
	};

	// Configure drives between simulation steps; setters reject invalid input and wake both endpoints.
	struct JointDriveSettings
	{
		bool PositionEnabled = false;
		bool VelocityEnabled = false;
		bool Acceleration = false;
		float Stiffness = 0.0f;
		float Damping = 0.0f;
		float MaxForce = std::numeric_limits<float>::max(); // Generalized N or Nm per coordinate; SLERP bounds the torque norm.
		bool IsValid() const;
		bool IsActive() const { return MaxForce > 0.0f && ((PositionEnabled && Stiffness > 0.0f) || ((PositionEnabled || VelocityEnabled) && Damping > 0.0f)); }
	};

	struct ScalarJointDrive
	{
		JointDriveSettings Settings;
		float TargetPosition = 0.0f; // Meters or radians in the joint coordinate.
		float TargetVelocity = 0.0f;
		bool IsValid() const;
	};

	enum class AngularDriveMode { Axes, Slerp };
	struct SixDOFDriveSettings
	{
		ScalarJointDrive Linear[3];
		ScalarJointDrive Angular[3]; // Y-X-Z Euler coordinates, matching the existing limits.
		AngularDriveMode AngularMode = AngularDriveMode::Axes;
		JointDriveSettings Slerp;
		Quaternion OrientationTarget = Quaternion::One(); // Relative to the initial joint orientation.
		Vector3 AngularVelocityTarget = Vector3::Zero(); // In the joint reference frame.
		bool IsValid() const;
		bool IsActive() const;
	};

	enum class PositionConstraintType
	{
		Fixed,
		Point,
		Distance,
		Hinge,
		Slider,
		Cone,
		SixDOF,
		PoseDrive,
	};

	// Constraint-local response weights; physical body mass and inertia stay unchanged.
	struct JointSolverBody
	{
		JointSolverBody(RigidBody* Value = nullptr) : Body(Value) {}
		RigidBody* operator->() const { return Body; }
		operator RigidBody*() const { return Body; }
		RigidBody* Body;
		float InverseMassScale = 1.0f;
	};

	class PositionConstraint
	{
	public:
		PositionConstraint(PositionConstraintType type, RigidBody* bodyA, RigidBody* bodyB);
		virtual ~PositionConstraint() {}

		PositionConstraintType GetType() const { return m_Type; }
		RigidBody* GetBodyA() const { return m_BodyA; }
		RigidBody* GetBodyB() const { return m_BodyB; }
		bool IsSolverActive() const;
		bool IsEnabled() const { return m_Enabled && !m_Broken; }
		void SetEnabled(bool enabled);
		bool IsBroken() const { return m_Broken; }
		void Repair(); // Reuses the existing joint frames; call between simulation steps.
		void SetBreakSettings(const JointBreakSettings& Settings);
		const JointBreakSettings& GetBreakSettings() const { return m_BreakSettings; }
		const JointReaction& GetReaction() const { return m_Reaction; }
		uint64_t GetId() const { return m_Id; } // Assigned by the owning world; zero for standalone constraints.

		void SetCompliance(float linear, float angular);
		float GetLinearCompliance() const { return m_LinearCompliance; }
		float GetAngularCompliance() const { return m_AngularCompliance; }
		// Reset multipliers once per simulation substep, before any joint solve.
		void BeginFrame(float Dt);
		bool EndFrame(bool EnableBreaking = true);
		void BeginStep(float dt, bool enableXPBD, bool enableDrives = true, bool positionReactions = false);
		bool EndStep(bool EnableBreaking = true); // Finalize reports; true only for a new break transition.
		virtual bool HasActiveDrive() const { return false; }

		virtual void Setup() {}
		virtual void SolvePosition(float dt) = 0;
		virtual void SolveVelocity(float dt);
		void Stabilize(float Dt, float ScaleA, float ScaleB, bool Positions);
		void Solve() { SolvePosition(1.0f / 60.0f); }

	protected:
		void ResetMultipliers();
		bool HasSoftLinearRows() const { return m_LinearAlpha > 0.0f; }
		bool HasSoftAngularRows() const { return m_AngularAlpha > 0.0f; }
		void DrivesChanged();
		bool m_DrivesEnabled = true;
		Vector3 m_LinearDriveImpulse = Vector3::Zero();
		Vector3 m_AngularDriveImpulse = Vector3::Zero();
		JointImpulseAccumulator m_PositionImpulses, m_VelocityImpulses;

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
		JointSolverBody m_BodyA, m_BodyB;
		bool m_Enabled;
	private:
		friend class PhysicsWorld;
		uint64_t m_Id = 0;
		bool m_Broken = false, m_StepOpen = false;
		float m_StepDt = 0.0f;
		float m_FrameDt = 0.0f;
		bool m_FrameOpen = false;
		JointImpulseAccumulator m_FrameImpulses;
		bool CheckBreak(bool EnableBreaking);
		JointBreakSettings m_BreakSettings;
		JointReaction m_Reaction;
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
		const Vector3& GetRelativePosition() const { return m_RelativePosition; }
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
		void SetDrive(const ScalarJointDrive& Drive);
		const ScalarJointDrive& GetDrive() const { return m_Drive; }
		bool HasActiveDrive() const override { return m_DrivesEnabled && m_Drive.Settings.IsActive(); }
		float GetDriveImpulse() const { return m_AngularDriveImpulse.x; }

	private:
		Vector3 m_LocalAxisA;
		Vector3 m_LocalAxisB;
		Vector3 m_LocalReferenceA;
		Vector3 m_LocalReferenceB;
		float m_MinAngle;
		float m_MaxAngle;
		ScalarJointDrive m_Drive;
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
		void SetDrive(const ScalarJointDrive& Drive);
		const ScalarJointDrive& GetDrive() const { return m_Drive; }
		bool HasActiveDrive() const override { return m_DrivesEnabled && m_Drive.Settings.IsActive(); }
		float GetDriveImpulse() const { return m_LinearDriveImpulse.x; }

	private:
		Vector3 m_LocalAnchorA;
		Vector3 m_LocalAnchorB;
		Vector3 m_LocalAxisA;
		Quaternion m_RelativeRotation;
		float m_MinDistance;
		float m_MaxDistance;
		ScalarJointDrive m_Drive;
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
		void SetDrives(const SixDOFDriveSettings& Drives);
		const SixDOFDriveSettings& GetDrives() const { return m_Drives; }
		bool HasActiveDrive() const override { return m_DrivesEnabled && m_Drives.IsActive(); }

	private:
		Vector3 m_LocalAnchorA;
		Vector3 m_LocalAnchorB;
		Quaternion m_ReferenceRotation;
		SixDOFConstraintSettings m_Settings;
		SixDOFDriveSettings m_Drives;
	};
	struct PoseDriveSettings
	{
		JointDriveSettings Linear, Angular;
		Transform Target;
		Vector3 LinearVelocityTarget = Vector3::Zero(), AngularVelocityTarget = Vector3::Zero();
		bool IsValid() const;
	};
	class PoseDriveConstraint : public PositionConstraint
	{
	public:
		PoseDriveConstraint(RigidBody* Reference, RigidBody* Body);
		void SetDrive(const PoseDriveSettings& Settings);
		const PoseDriveSettings& GetDrive() const { return m_Drive; }
		bool HasActiveDrive() const override { return m_DrivesEnabled && (m_Drive.Linear.IsActive() || m_Drive.Angular.IsActive()); }
		void SolvePosition(float Dt) override { (void)Dt; }
		void SolveVelocity(float Dt) override;
	private:
		friend class PhysicalAnimation;
		void SetReference(RigidBody* Reference);
		PoseDriveSettings m_Drive;
	};
}
