#pragma once

#include <memory>
#include "../../Collision/GeometryQuery.h"

namespace Riemann
{
	class PhysicsWorld;
	class NarrowPhase;

	enum class CharacterShape { Capsule, Box };

	struct CharacterControllerSettings
	{
		CharacterShape Shape = CharacterShape::Capsule;
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

	struct CharacterGround
	{
		GeometryHandle Handle;
		Vector3 Normal = Vector3::Zero();
		Vector3 Position = Vector3::Zero();
		float Distance = 0.0f;
		bool Walkable = false;
	};

	struct CharacterMoveResult
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

	class CharacterController
	{
	public:
		~CharacterController();
		CharacterController(const CharacterController&) = delete;
		CharacterController& operator=(const CharacterController&) = delete;
		const CharacterControllerSettings& GetSettings() const { return m_Settings; }
		const Vector3& GetPosition() const;
		Geometry* GetGeometry() const { return m_Geometry.get(); }
		const CharacterGround& GetGround() const { return m_Ground; }
		const CharacterMoveResult& GetLastMove() const { return m_LastMove; }
		bool IsGrounded() const { return m_Ground.Walkable; }
		void SetPosition(const Vector3& Position);
		void SetMoveVelocity(const Vector3& Velocity);
		// Launch along Up at Speed units per second. Only a supported character can jump.
		bool Jump(float Speed);
		CharacterMoveResult Move(const Vector3& Displacement, float Dt);
		void SetFilterData(const CollisionData& Data);

	private:
		friend class PhysicsWorld;
		CharacterController(PhysicsWorld& World, const CharacterControllerSettings& Settings, const Vector3& Position);
		void Simulate(float Dt);
		CharacterMoveResult MoveInternal(const Vector3& Displacement, float Dt, bool IntegrateGravity);
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
		CharacterControllerSettings m_Settings;
		std::unique_ptr<Geometry> m_Geometry;
		std::unique_ptr<Geometry> m_SweepGeometry;
		std::unique_ptr<NarrowPhase> m_NarrowPhase;
		CharacterGround m_Ground;
		CharacterMoveResult m_LastMove;
		Vector3 m_MoveVelocity = Vector3::Zero();
		Vector3 m_PlatformLocalPosition = Vector3::Zero();
		Transform m_PlatformTransform;
		GeometryHandle m_IgnoredPlatform;
		GeometryHandle m_StepSupport;
		float m_VerticalVelocity = 0.0f;
	};
}
