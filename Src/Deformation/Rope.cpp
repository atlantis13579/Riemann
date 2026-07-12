#include "Rope.h"

#include <algorithm>

#include "../RigidBodyDynamics/RigidBody.h"
#include "../Collision/GeometryObject.h"

namespace Riemann
{
	RopeSettings::RopeSettings()
		: SegmentCount(16)
		, SolverIterations(12)
		, ParticleRadius(0.08f)
		, ParticleInverseMass(1.0f)
		, RestLength(0.0f)
		, Damping(0.995f)
		, Gravity(0.0f, -9.8f, 0.0f)
	{
	}

	RopeSystem::RopeSystem(const RopeSettings& settings, const Vector3& start, const Vector3& end)
		: m_Settings(settings)
		, m_SegmentRestLength(0.0f)
	{
		m_Settings.SegmentCount = std::max(1, m_Settings.SegmentCount);
		m_Settings.SolverIterations = std::max(1, m_Settings.SolverIterations);
		m_Settings.ParticleRadius = std::max(0.001f, m_Settings.ParticleRadius);
		m_Settings.ParticleInverseMass = std::max(0.0f, m_Settings.ParticleInverseMass);
		m_Settings.Damping = std::max(0.0f, std::min(1.0f, m_Settings.Damping));
		m_SegmentRestLength = m_Settings.RestLength > 0.0f
			? m_Settings.RestLength
			: (end - start).Length() / (float)m_Settings.SegmentCount;

		m_Particles.resize((size_t)m_Settings.SegmentCount + 1);
		m_RenderGeometries.reserve(m_Particles.size());
		for (int i = 0; i <= m_Settings.SegmentCount; ++i)
		{
			const float fraction = (float)i / (float)m_Settings.SegmentCount;
			RopeParticle& particle = m_Particles[(size_t)i];
			particle.Position = start + (end - start) * fraction;
			particle.PreviousPosition = particle.Position;
			particle.PinPosition = particle.Position;
			particle.InverseMass = m_Settings.ParticleInverseMass;
			particle.Pinned = false;
			particle.RenderGeometry = GeometryFactory::CreateSphere(particle.Position, m_Settings.ParticleRadius);
			particle.RenderGeometry->SetQueryEnabled(false);
			particle.RenderGeometry->SetSimulationEnabled(false);
			m_RenderGeometries.push_back(particle.RenderGeometry);
		}
	}

	RopeSystem::~RopeSystem()
	{
		for (Geometry* geometry : m_RenderGeometries)
		{
			delete geometry;
		}
		m_RenderGeometries.clear();
	}

	bool RopeSystem::PinParticle(int index, const Vector3& worldPosition)
	{
		if (index < 0 || index >= GetParticleCount())
		{
			return false;
		}
		RopeParticle& particle = m_Particles[(size_t)index];
		particle.Pinned = true;
		particle.PinPosition = worldPosition;
		particle.Position = worldPosition;
		particle.PreviousPosition = worldPosition;
		UpdateRenderGeometries();
		return true;
	}

	bool RopeSystem::UnpinParticle(int index)
	{
		if (index < 0 || index >= GetParticleCount())
		{
			return false;
		}
		m_Particles[(size_t)index].Pinned = false;
		return true;
	}

	bool RopeSystem::AttachRigidBody(int index, RigidBody* body, const Vector3& worldAnchor)
	{
		if (index < 0 || index >= GetParticleCount() || body == nullptr)
		{
			return false;
		}
		DetachRigidBody(index);
		BodyAttachment attachment;
		attachment.ParticleIndex = index;
		attachment.Body = body;
		attachment.LocalAnchor = body->WorldToBodyLocal(worldAnchor);
		m_BodyAttachments.push_back(attachment);
		return true;
	}

	bool RopeSystem::DetachRigidBody(int index)
	{
		const size_t oldSize = m_BodyAttachments.size();
		m_BodyAttachments.erase(std::remove_if(m_BodyAttachments.begin(), m_BodyAttachments.end(),
			[index](const BodyAttachment& attachment) { return attachment.ParticleIndex == index; }), m_BodyAttachments.end());
		return m_BodyAttachments.size() != oldSize;
	}

	bool RopeSystem::DetachRigidBody(RigidBody* body)
	{
		const size_t oldSize = m_BodyAttachments.size();
		m_BodyAttachments.erase(std::remove_if(m_BodyAttachments.begin(), m_BodyAttachments.end(),
			[body](const BodyAttachment& attachment) { return attachment.Body == body; }), m_BodyAttachments.end());
		return m_BodyAttachments.size() != oldSize;
	}

	void RopeSystem::Simulate(float dt)
	{
		if (dt <= 0.0f)
		{
			return;
		}

		m_BodyStartPositions.clear();
		for (const BodyAttachment& attachment : m_BodyAttachments)
		{
			if (attachment.Body && attachment.Body->CastDynamic())
			{
				const auto found = std::find_if(m_BodyStartPositions.begin(), m_BodyStartPositions.end(),
					[&attachment](const std::pair<RigidBody*, Vector3>& entry) { return entry.first == attachment.Body; });
				if (found == m_BodyStartPositions.end())
				{
					m_BodyStartPositions.push_back(std::make_pair(attachment.Body, attachment.Body->X));
					attachment.Body->X += attachment.Body->V * dt;
				}
			}
		}

		const float dtSquared = dt * dt;
		for (RopeParticle& particle : m_Particles)
		{
			if (particle.Pinned || particle.InverseMass <= 0.0f)
			{
				continue;
			}
			const Vector3 velocity = (particle.Position - particle.PreviousPosition) * m_Settings.Damping;
			particle.PreviousPosition = particle.Position;
			particle.Position += velocity + m_Settings.Gravity * dtSquared;
		}

		for (int iteration = 0; iteration < m_Settings.SolverIterations; ++iteration)
		{
			SolveDistanceConstraints();
			SolvePins();
			SolveBodyAttachments();
		}

		for (const std::pair<RigidBody*, Vector3>& entry : m_BodyStartPositions)
		{
			RigidBodyDynamic* body = entry.first->CastDynamic();
			if (body)
			{
				body->V = (body->X - entry.second) / dt;
				body->X = entry.second;
				body->Wakeup();
			}
		}
		UpdateRenderGeometries();
	}

	void RopeSystem::SolveDistanceConstraints()
	{
		for (int i = 0; i < m_Settings.SegmentCount; ++i)
		{
			RopeParticle& particleA = m_Particles[(size_t)i];
			RopeParticle& particleB = m_Particles[(size_t)i + 1];
			const Vector3 delta = particleB.Position - particleA.Position;
			const float distance = delta.Length();
			if (distance <= 1e-7f)
			{
				continue;
			}
			const float weightA = particleA.Pinned ? 0.0f : particleA.InverseMass;
			const float weightB = particleB.Pinned ? 0.0f : particleB.InverseMass;
			const float weightSum = weightA + weightB;
			if (weightSum <= 1e-7f)
			{
				continue;
			}
			const Vector3 correction = delta * ((distance - m_SegmentRestLength) / distance);
			particleA.Position += correction * (weightA / weightSum);
			particleB.Position -= correction * (weightB / weightSum);
		}
	}

	void RopeSystem::SolvePins()
	{
		for (RopeParticle& particle : m_Particles)
		{
			if (particle.Pinned)
			{
				particle.Position = particle.PinPosition;
			}
		}
	}

	void RopeSystem::SolveBodyAttachments()
	{
		for (const BodyAttachment& attachment : m_BodyAttachments)
		{
			if (attachment.Body == nullptr || attachment.ParticleIndex < 0 || attachment.ParticleIndex >= GetParticleCount())
			{
				continue;
			}
			RopeParticle& particle = m_Particles[(size_t)attachment.ParticleIndex];
			RigidBodyDynamic* dynamicBody = attachment.Body->CastDynamic();
			const float particleWeight = particle.Pinned ? 0.0f : particle.InverseMass;
			const float bodyWeight = dynamicBody ? dynamicBody->InvMass : 0.0f;
			const float weightSum = particleWeight + bodyWeight;
			if (weightSum <= 1e-7f)
			{
				continue;
			}
			const Vector3 bodyAnchor = attachment.Body->BodyLocalToWorld(attachment.LocalAnchor);
			const Vector3 error = particle.Position - bodyAnchor;
			particle.Position -= error * (particleWeight / weightSum);
			if (dynamicBody)
			{
				dynamicBody->X += error * (bodyWeight / weightSum);
			}
		}
	}

	void RopeSystem::UpdateRenderGeometries()
	{
		for (RopeParticle& particle : m_Particles)
		{
			if (particle.RenderGeometry)
			{
				particle.RenderGeometry->SetPosition(particle.Position);
			}
		}
	}
}
