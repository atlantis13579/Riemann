#pragma once

#include <vector>
#include <utility>

#include "../Maths/Vector3.h"

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
		float RestLength;
		float Damping;
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

		void SolveDistanceConstraints();
		void SolvePins();
		void SolveBodyAttachments();
		void UpdateRenderGeometries();

	private:
		RopeSettings m_Settings;
		float m_SegmentRestLength;
		std::vector<RopeParticle> m_Particles;
		std::vector<BodyAttachment> m_BodyAttachments;
		std::vector<Geometry*> m_RenderGeometries;
		std::vector<std::pair<RigidBody*, Vector3> > m_BodyStartPositions;
	};
}
