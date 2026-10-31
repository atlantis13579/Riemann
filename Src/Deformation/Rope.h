#pragma once

#include <vector>
#include "../Maths/Transform.h"

namespace Riemann
{
	class Geometry;
	class RigidBody;

	struct RopeSettings
	{
		RopeSettings();

		int SegmentCount;
		int SolverIterations;
		float ParticleRadius;
		float ParticleInverseMass;
		// Rest length per segment; zero uses the initial endpoint distance.
		float RestLength;
		// Velocity retention at 60 Hz, converted to the actual simulation step.
		float Damping;
		float StretchCompliance;
		float CollisionFriction;
		bool TensionOnly;
		bool ContinuousCollision;
		Vector3 Gravity;
	};

	struct RopeParticle
	{
		Vector3 Position;
		Vector3 PreviousPosition;
		Vector3 PinPosition;
		float InverseMass;
		bool Pinned;
		Geometry* RenderGeometry;
	};

	class RopeSystem
	{
	public:
		RopeSystem(const RopeSettings& settings, const Vector3& start, const Vector3& end);
		~RopeSystem();

		void Simulate(float dt);
		bool PinParticle(int index, const Vector3& worldPosition);
		bool UnpinParticle(int index);
		bool AttachRigidBody(int index, RigidBody* body, const Vector3& worldAnchor);
		bool DetachRigidBody(int index);
		bool DetachRigidBody(RigidBody* body);
		bool SetParticleVelocity(int index, const Vector3& velocity);
		Vector3 GetParticleVelocity(int index) const;
		// Colliders are borrowed. Remove them before destroying their rigid body.
		// Explicit collider poses are supplied in world space; a null body is static.
		// Collider contacts affect the rope; rigid-body feedback uses attachments.
		bool AddSphereCollider(RigidBody* body, float radius, const Vector3& worldCenter);
		bool AddBoxCollider(RigidBody* body, const Vector3& halfExtent, const Transform& worldTransform);
		// Capture shape data and follow its body. Re-register to change shape data.
		bool AddCollider(Geometry* geometry);
		bool RemoveCollider(const Geometry* geometry);
		bool RemoveColliders(RigidBody* body);
		void ClearColliders();
		size_t GetColliderCount() const { return m_Colliders.size(); }

		int GetParticleCount() const { return (int)m_Particles.size(); }
		int GetSegmentCount() const { return m_Settings.SegmentCount; }
		float GetRestLength() const { return m_SegmentRestLength; }
		const RopeParticle& GetParticle(int index) const { return m_Particles[index]; }
		RopeParticle& GetParticle(int index) { return m_Particles[index]; }
		const std::vector<Geometry*>& GetRenderGeometries() const { return m_RenderGeometries; }

	private:
		struct BodyAttachment
		{
			int ParticleIndex;
			RigidBody* Body;
			Vector3 LocalAnchor;
		};

		struct BodyState
		{
			RigidBody* Body;
			Vector3 Position;
			Quaternion Rotation;
		};

		struct Collider
		{
			RigidBody* Body;
			const Geometry* Source;
			Transform LocalTransform;
			Transform PreviousTransform;
			Transform WorldTransform;
			Vector3 HalfExtent;
			float Radius;
			bool Sphere;
		};

		void SolveDistanceConstraints(float dt);
		void SolvePins();
		void SolveBodyAttachments();
		void SolveCollisions();
		void SolveSegmentCollision(int indexA, int indexB, const Collider& collider);
		void ApplyCollision(int indexA, int indexB, float fraction, const Vector3& normal,
			float depth, const Vector3& colliderDisplacement);
		void UpdateRenderGeometries();

	private:
		RopeSettings m_Settings;
		float m_SegmentRestLength;
		float m_PreviousDt;
		std::vector<RopeParticle> m_Particles;
		std::vector<Vector3> m_CollisionStartPositions;
		std::vector<float> m_StretchMultipliers;
		std::vector<BodyAttachment> m_BodyAttachments;
		std::vector<Geometry*> m_RenderGeometries;
		std::vector<BodyState> m_BodyStartStates;
		std::vector<Collider> m_Colliders;
	};
}
