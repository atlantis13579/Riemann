#pragma once

#include <memory>
#include "../../Collision/GeometryQuery.h"

namespace Riemann
{
	class PhysicsWorld;
	class NarrowPhase;

	enum class RobotShape { Capsule, Box };

	struct RobotControllerSettings
	{
		RobotShape Shape = RobotShape::Capsule;
		float Radius = 0.35f;
		float HalfHeight = 0.55f;
		Vector3 HalfExtent = Vector3(0.35f, 0.9f, 0.35f);
		Vector3 Up = Vector3::UnitY();
		float SkinWidth = 0.02f;
		float StepHeight = 0.3f;
		float SlopeLimitCosine = 0.70710678f;
		float GroundSnapDistance = 0.2f;
		float Gravity = 9.8f;
		float MinimumMoveDistance = 1.0e-5f;
		float SweepTolerance = 1.0e-3f;
		float RecoveryTolerance = 1.0e-4f;
		float MaximumRecoveryDistance = 0.5f;
		float NormalTolerance = 1.0e-5f;
		int SlideIterations = 8;
		int RecoveryIterations = 8;
		bool AutomaticUpdate = true;
		bool FollowPlatforms = true;
		CollisionData FilterData;
		bool IsValid() const;
	};

	struct RobotGround
	{
		GeometryHandle Handle;
		Vector3 Normal = Vector3::Zero();
		Vector3 Position = Vector3::Zero();
		float Distance = 0.0f;
		bool Walkable = false;
	};

	struct RobotMoveResult
	{
		Vector3 Displacement = Vector3::Zero();
		Vector3 PlatformDisplacement = Vector3::Zero();
		float RecoveryDistance = 0.0f;
		bool HitSide = false;
		bool HitAbove = false;
		bool HitBelow = false;
		bool Stepped = false;
		bool RecoveryIncomplete = false;
		bool IterationLimitReached = false;
	};

	class RobotController
	{
	public:
		virtual ~RobotController() = default;
		virtual Vector3 GetPosition() const = 0;
		virtual void SetMoveVelocity(const Vector3& WorldVelocity) = 0;
		virtual bool IsGrounded() const = 0;
		virtual Geometry* GetGeometry() const { return nullptr; }
	protected:
		friend class PhysicsWorld;
		virtual void Simulate(float Dt) = 0;
	};

	class SweepRobotController final : public RobotController
	{
	public:
		~SweepRobotController() override;
		SweepRobotController(const SweepRobotController&) = delete;
		SweepRobotController& operator=(const SweepRobotController&) = delete;
		const RobotControllerSettings& GetSettings() const { return m_Settings; }
		Vector3 GetPosition() const override;
		Geometry* GetGeometry() const override { return m_Geometry.get(); }
		const RobotGround& GetGround() const { return m_Ground; }
		const RobotMoveResult& GetLastMove() const { return m_LastMove; }
		bool IsGrounded() const override { return m_Ground.Walkable; }
		void SetPosition(const Vector3& Position);
		void SetMoveVelocity(const Vector3& Velocity) override;
		// Launch along Up at Speed units per second. Only a supported character can jump.
		bool Jump(float Speed);
		RobotMoveResult Move(const Vector3& Displacement, float Dt);
		void SetFilterData(const CollisionData& Data);

	private:
		friend class PhysicsWorld;
		SweepRobotController(PhysicsWorld& World, const RobotControllerSettings& Settings, const Vector3& Position);
		void Simulate(float Dt) override;
		RobotMoveResult MoveInternal(const Vector3& Displacement, float Dt, bool IntegrateGravity);
		float AdvanceVertical(float Dt);
		void SetPositionInternal(const Vector3& Position);
		bool Blocks(const Geometry* Shape) const;
		bool Cast(const Vector3& Displacement, SweepResult& Hit) const;
		float TravelDistance(const Vector3& Direction, const SweepResult& Hit) const;
		void RecoverOverlaps();
		void Slide(const Vector3& Displacement, bool Horizontal, bool AllowStep);
		bool TryStep(const Vector3& Displacement);
		bool StepSurface(const SweepResult& Hit, Vector3& Normal, Vector3& Point) const;
		void FindGround(float ProbeDistance, bool Snap);
		void FollowPlatform();
		void RememberPlatform();

		PhysicsWorld& m_World;
		RobotControllerSettings m_Settings;
		std::unique_ptr<Geometry> m_Geometry;
		std::unique_ptr<Geometry> m_SweepGeometry;
		std::unique_ptr<NarrowPhase> m_NarrowPhase;
		RobotGround m_Ground;
		RobotMoveResult m_LastMove;
		Vector3 m_MoveVelocity = Vector3::Zero();
		Vector3 m_PlatformLocalPosition = Vector3::Zero();
		Transform m_PlatformTransform;
		GeometryHandle m_IgnoredPlatform;
		GeometryHandle m_StepSupport;
		float m_VerticalVelocity = 0.0f;
	};
}
