#include "DeformableBody.h"

#include <algorithm>
#include <cmath>

namespace Riemann
{
	namespace
	{
		bool IsFiniteVector(const Vector3& value)
		{
			return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
		}

		float NonNegative(float value, float fallback)
		{
			return std::isfinite(value) ? std::max(value, 0.0f) : fallback;
		}

		float SignedVolume(const Vector3& a, const Vector3& b, const Vector3& c, const Vector3& d)
		{
			return (b - a).Dot((c - a).Cross(d - a)) / 6.0f;
		}
	}

	DeformableBodySettings::DeformableBodySettings()
		: Substeps(4)
		, SolverIterations(12)
		, EdgeCompliance(1.0e-6f)
		, VolumeCompliance(0.0f)
		, Damping(0.5f)
		, Gravity(0.0f, -9.8f, 0.0f)
		, YieldStrain(0.1f)
		, PlasticCreep(0.0f)
		, MaxPlasticStrain(0.5f)
		, GroundEnabled(false)
		, GroundHeight(0.0f)
		, ParticleRadius(0.025f)
		, Restitution(0.0f)
		, Friction(0.5f)
	{
	}

	DeformableBody::DeformableBody(const DeformableBodySettings& settings)
		: m_Settings(settings)
	{
		m_Settings.Substeps = std::max(1, std::min(settings.Substeps, 128));
		m_Settings.SolverIterations = std::max(1, std::min(settings.SolverIterations, 128));
		m_Settings.EdgeCompliance = NonNegative(settings.EdgeCompliance, 1.0e-6f);
		m_Settings.VolumeCompliance = NonNegative(settings.VolumeCompliance, 0.0f);
		m_Settings.Damping = NonNegative(settings.Damping, 0.5f);
		m_Settings.Gravity = IsFiniteVector(settings.Gravity) ? settings.Gravity : Vector3::Zero();
		m_Settings.YieldStrain = NonNegative(settings.YieldStrain, 0.1f);
		m_Settings.PlasticCreep = NonNegative(settings.PlasticCreep, 0.0f);
		m_Settings.MaxPlasticStrain = NonNegative(settings.MaxPlasticStrain, 0.5f);
		m_Settings.GroundHeight = std::isfinite(settings.GroundHeight) ? settings.GroundHeight : 0.0f;
		m_Settings.ParticleRadius = NonNegative(settings.ParticleRadius, 0.025f);
		m_Settings.Restitution = std::min(NonNegative(settings.Restitution, 0.0f), 1.0f);
		m_Settings.Friction = NonNegative(settings.Friction, 0.5f);
	}

	bool DeformableBody::IsValidParticle(int index) const
	{
		return index >= 0 && (size_t)index < m_Particles.size();
	}

	float DeformableBody::GetInverseMass(int index) const
	{
		const DeformableParticle& particle = m_Particles[(size_t)index];
		return particle.Pinned ? 0.0f : particle.InverseMass;
	}

	int DeformableBody::AddParticle(const Vector3& position, float inverseMass)
	{
		if (!IsFiniteVector(position) || !std::isfinite(inverseMass) || inverseMass < 0.0f)
		{
			return -1;
		}
		DeformableParticle particle;
		particle.Position = position;
		particle.PreviousPosition = position;
		particle.InitialPosition = position;
		particle.PinPosition = position;
		particle.Velocity = Vector3::Zero();
		particle.Force = Vector3::Zero();
		particle.InverseMass = inverseMass;
		particle.Pinned = false;
		m_Particles.push_back(particle);
		m_GroundContacts.push_back(0);
		m_GroundCorrections.push_back(0.0f);
		m_RenderTopologyDirty = true;
		return (int)m_Particles.size() - 1;
	}

	int DeformableBody::AddEdge(int particleA, int particleB, float compliance)
	{
		if (!IsValidParticle(particleA) || !IsValidParticle(particleB) || particleA == particleB
			|| !std::isfinite(compliance))
		{
			return -1;
		}
		for (size_t i = 0; i < m_Edges.size(); ++i)
		{
			const DeformableEdge& edge = m_Edges[i];
			if ((edge.ParticleA == particleA && edge.ParticleB == particleB)
				|| (edge.ParticleA == particleB && edge.ParticleB == particleA))
			{
				return (int)i;
			}
		}
		const float length = (m_Particles[(size_t)particleB].Position - m_Particles[(size_t)particleA].Position).Length();
		if (!std::isfinite(length) || length <= 1.0e-7f)
		{
			return -1;
		}
		DeformableEdge edge;
		edge.ParticleA = particleA;
		edge.ParticleB = particleB;
		edge.RestLength = edge.InitialRestLength = length;
		edge.Compliance = compliance < 0.0f ? m_Settings.EdgeCompliance : compliance;
		edge.Lambda = 0.0f;
		m_Edges.push_back(edge);
		return (int)m_Edges.size() - 1;
	}

	int DeformableBody::AddTetrahedron(int a, int b, int c, int d, float compliance)
	{
		int indices[4] = { a, b, c, d };
		for (int i = 0; i < 4; ++i)
		{
			if (!IsValidParticle(indices[i])) return -1;
			for (int j = 0; j < i; ++j)
			{
				if (indices[i] == indices[j]) return -1;
			}
		}
		if (!std::isfinite(compliance)) return -1;
		int sortedIndices[4] = { a, b, c, d };
		std::sort(sortedIndices, sortedIndices + 4);
		for (size_t i = 0; i < m_Tetrahedra.size(); ++i)
		{
			int existing[4];
			std::copy(m_Tetrahedra[i].Particles, m_Tetrahedra[i].Particles + 4, existing);
			std::sort(existing, existing + 4);
			if (std::equal(existing, existing + 4, sortedIndices)) return (int)i;
		}
		const float volume = SignedVolume(m_Particles[(size_t)a].Position, m_Particles[(size_t)b].Position,
			m_Particles[(size_t)c].Position, m_Particles[(size_t)d].Position);
		float maxLength = 0.0f;
		for (int i = 0; i < 4; ++i)
		{
			for (int j = i + 1; j < 4; ++j)
			{
				const float length = (m_Particles[(size_t)indices[i]].Position - m_Particles[(size_t)indices[j]].Position).Length();
				if (!std::isfinite(length) || length <= 1.0e-7f) return -1;
				maxLength = std::max(maxLength, length);
			}
		}
		if (!std::isfinite(volume) || std::fabs(volume) <= 1.0e-7f * maxLength * maxLength * maxLength)
		{
			return -1;
		}
		DeformableTetrahedron tetrahedron;
		std::copy(indices, indices + 4, tetrahedron.Particles);
		tetrahedron.RestVolume = tetrahedron.InitialRestVolume = volume;
		tetrahedron.Compliance = compliance < 0.0f ? m_Settings.VolumeCompliance : compliance;
		tetrahedron.Lambda = 0.0f;
		m_Tetrahedra.push_back(tetrahedron);
		for (int i = 0; i < 4; ++i)
		{
			for (int j = i + 1; j < 4; ++j) AddEdge(indices[i], indices[j]);
		}
		return (int)m_Tetrahedra.size() - 1;
	}

	bool DeformableBody::SetSurfaceTriangles(const std::vector<Index3>& triangles)
	{
		for (const Index3& triangle : triangles)
		{
			if (!IsValidParticle(triangle.a) || !IsValidParticle(triangle.b) || !IsValidParticle(triangle.c)
				|| triangle.a == triangle.b || triangle.a == triangle.c || triangle.b == triangle.c)
			{
				return false;
			}
		}
		m_SurfaceTriangles = triangles;
		m_RenderTopologyDirty = true;
		UpdateRenderGeometry();
		return true;
	}

	bool DeformableBody::PinParticle(int index, const Vector3& position)
	{
		if (!IsValidParticle(index) || !IsFiniteVector(position)) return false;
		DeformableParticle& particle = m_Particles[(size_t)index];
		particle.Pinned = true;
		particle.PinPosition = particle.Position = particle.PreviousPosition = position;
		particle.Velocity = particle.Force = Vector3::Zero();
		UpdateRenderGeometry();
		return true;
	}

	bool DeformableBody::UnpinParticle(int index)
	{
		if (!IsValidParticle(index)) return false;
		m_Particles[(size_t)index].Pinned = false;
		return true;
	}

	bool DeformableBody::SetParticlePosition(int index, const Vector3& position, bool resetVelocity)
	{
		if (!IsValidParticle(index) || !IsFiniteVector(position)) return false;
		DeformableParticle& particle = m_Particles[(size_t)index];
		particle.Position = particle.PreviousPosition = position;
		if (particle.Pinned) particle.PinPosition = position;
		if (resetVelocity) particle.Velocity = Vector3::Zero();
		UpdateRenderGeometry();
		return true;
	}

	bool DeformableBody::SetParticleVelocity(int index, const Vector3& velocity)
	{
		if (!IsValidParticle(index) || !IsFiniteVector(velocity) || GetInverseMass(index) <= 0.0f) return false;
		m_Particles[(size_t)index].Velocity = velocity;
		return true;
	}

	bool DeformableBody::AddForce(int index, const Vector3& force)
	{
		if (!IsValidParticle(index) || !IsFiniteVector(force) || GetInverseMass(index) <= 0.0f) return false;
		DeformableParticle& particle = m_Particles[(size_t)index];
		if (!IsFiniteVector(particle.Force + force)) return false;
		particle.Force += force;
		return true;
	}

	bool DeformableBody::AddImpulse(int index, const Vector3& impulse)
	{
		if (!IsValidParticle(index) || !IsFiniteVector(impulse) || GetInverseMass(index) <= 0.0f) return false;
		DeformableParticle& particle = m_Particles[(size_t)index];
		const Vector3 velocity = particle.Velocity + impulse * particle.InverseMass;
		if (!IsFiniteVector(velocity)) return false;
		particle.Velocity = velocity;
		return true;
	}

	int DeformableBody::AddRadialImpulse(const Vector3& center, float radius, const Vector3& impulse)
	{
		if (!IsFiniteVector(center) || !IsFiniteVector(impulse) || !std::isfinite(radius) || radius <= 0.0f) return 0;
		std::vector<float> weights(m_Particles.size(), 0.0f);
		float weightSum = 0.0f;
		for (size_t i = 0; i < m_Particles.size(); ++i)
		{
			if (GetInverseMass((int)i) <= 0.0f) continue;
			weights[i] = std::max(0.0f, 1.0f - (m_Particles[i].Position - center).Length() / radius);
			weightSum += weights[i];
		}
		if (weightSum <= 0.0f) return 0;
		int count = 0;
		for (size_t i = 0; i < m_Particles.size(); ++i)
		{
			if (weights[i] > 0.0f && AddImpulse((int)i, impulse * (weights[i] / weightSum))) ++count;
		}
		return count;
	}

	void DeformableBody::UpdatePlasticity(float dt)
	{
		if (m_Settings.PlasticCreep <= 0.0f || m_Settings.MaxPlasticStrain <= 0.0f) return;
		const float fraction = 1.0f - std::exp(-m_Settings.PlasticCreep * dt);
		for (DeformableEdge& edge : m_Edges)
		{
			if (edge.Compliance <= 0.0f) continue;
			const float length = (m_Particles[(size_t)edge.ParticleB].Position - m_Particles[(size_t)edge.ParticleA].Position).Length();
			const float difference = length - edge.RestLength;
			const float yieldLength = m_Settings.YieldStrain * edge.RestLength;
			if (std::fabs(difference) <= yieldLength) continue;
			const float excess = difference - std::copysign(yieldLength, difference);
			const float minimum = edge.InitialRestLength * std::max(0.001f, 1.0f - m_Settings.MaxPlasticStrain);
			const float maximum = edge.InitialRestLength * (1.0f + m_Settings.MaxPlasticStrain);
			edge.RestLength = std::max(minimum, std::min(edge.RestLength + excess * fraction, maximum));
		}
	}

	void DeformableBody::SolveEdges(float dt)
	{
		for (DeformableEdge& edge : m_Edges)
		{
			const float inverseMassA = GetInverseMass(edge.ParticleA);
			const float inverseMassB = GetInverseMass(edge.ParticleB);
			if (inverseMassA + inverseMassB <= 0.0f) continue;
			DeformableParticle& a = m_Particles[(size_t)edge.ParticleA];
			DeformableParticle& b = m_Particles[(size_t)edge.ParticleB];
			const Vector3 delta = b.Position - a.Position;
			const float length = delta.Length();
			const Vector3 direction = length > 1.0e-8f ? delta / length : (b.InitialPosition - a.InitialPosition).SafeUnit();
			const float alpha = edge.Compliance / (dt * dt);
			const float deltaLambda = (-(length - edge.RestLength) - alpha * edge.Lambda) / (inverseMassA + inverseMassB + alpha);
			edge.Lambda += deltaLambda;
			a.Position -= direction * (inverseMassA * deltaLambda);
			b.Position += direction * (inverseMassB * deltaLambda);
		}
	}

	void DeformableBody::SolveVolumes(float dt)
	{
		for (DeformableTetrahedron& tetrahedron : m_Tetrahedra)
		{
			Vector3 positions[4];
			float inverseMass[4];
			for (int i = 0; i < 4; ++i)
			{
				positions[i] = m_Particles[(size_t)tetrahedron.Particles[i]].Position;
				inverseMass[i] = GetInverseMass(tetrahedron.Particles[i]);
			}
			Vector3 gradients[4];
			gradients[1] = (positions[2] - positions[0]).Cross(positions[3] - positions[0]) / 6.0f;
			gradients[2] = (positions[3] - positions[0]).Cross(positions[1] - positions[0]) / 6.0f;
			gradients[3] = (positions[1] - positions[0]).Cross(positions[2] - positions[0]) / 6.0f;
			gradients[0] = -gradients[1] - gradients[2] - gradients[3];
			float weight = 0.0f;
			for (int i = 0; i < 4; ++i) weight += inverseMass[i] * gradients[i].SquareLength();
			if (weight <= 1.0e-20f) continue;
			const float alpha = tetrahedron.Compliance / (dt * dt);
			const float error = SignedVolume(positions[0], positions[1], positions[2], positions[3]) - tetrahedron.RestVolume;
			const float deltaLambda = (-error - alpha * tetrahedron.Lambda) / (weight + alpha);
			tetrahedron.Lambda += deltaLambda;
			for (int i = 0; i < 4; ++i)
			{
				m_Particles[(size_t)tetrahedron.Particles[i]].Position += gradients[i] * (inverseMass[i] * deltaLambda);
			}
		}
	}

	void DeformableBody::SolveGround()
	{
		if (!m_Settings.GroundEnabled) return;
		const float height = m_Settings.GroundHeight + m_Settings.ParticleRadius;
		for (size_t i = 0; i < m_Particles.size(); ++i)
		{
			if (GetInverseMass((int)i) <= 0.0f) continue;
			if (m_Particles[i].Position.y < height)
			{
				m_GroundCorrections[i] += height - m_Particles[i].Position.y;
				m_Particles[i].Position.y = height;
				m_GroundContacts[i] = 1;
			}
		}
	}

	void DeformableBody::Simulate(float dt)
	{
		if (!std::isfinite(dt) || dt <= 0.0f || m_Particles.empty()) return;
		const float step = dt / m_Settings.Substeps;
		if (step < 1.0e-8f) return;
		const float damping = std::exp(-m_Settings.Damping * step);
		for (int substep = 0; substep < m_Settings.Substeps; ++substep)
		{
			std::fill(m_GroundContacts.begin(), m_GroundContacts.end(), 0);
			std::fill(m_GroundCorrections.begin(), m_GroundCorrections.end(), 0.0f);
			for (size_t i = 0; i < m_Particles.size(); ++i)
			{
				DeformableParticle& particle = m_Particles[i];
				particle.PreviousPosition = particle.Position;
				if (GetInverseMass((int)i) <= 0.0f)
				{
					if (particle.Pinned) particle.Position = particle.PinPosition;
					particle.Velocity = Vector3::Zero();
					continue;
				}
				particle.Velocity += (m_Settings.Gravity + particle.Force * particle.InverseMass) * step;
				particle.Velocity *= damping;
				particle.Position += particle.Velocity * step;
			}
			for (DeformableEdge& edge : m_Edges) edge.Lambda = 0.0f;
			for (DeformableTetrahedron& tetrahedron : m_Tetrahedra) tetrahedron.Lambda = 0.0f;
			for (int iteration = 0; iteration < m_Settings.SolverIterations; ++iteration)
			{
				SolveEdges(step);
				SolveVolumes(step);
				SolveGround();
			}
			// Plastic flow uses elastic strain after projection, not timestep-dependent
			// unconstrained motion. The updated rest lengths apply to the next substep.
			UpdatePlasticity(step);
			for (size_t i = 0; i < m_Particles.size(); ++i)
			{
				DeformableParticle& particle = m_Particles[i];
				if (GetInverseMass((int)i) <= 0.0f) continue;
				const Vector3 predictedVelocity = particle.Velocity;
				particle.Velocity = (particle.Position - particle.PreviousPosition) / step;
				if (m_GroundContacts[i])
				{
					// Depenetration is a positional correction, not an outgoing impulse.
					particle.Velocity.y -= m_GroundCorrections[i] / step;
					const float bounce = predictedVelocity.y < -0.2f ? -predictedVelocity.y * m_Settings.Restitution : 0.0f;
					particle.Velocity.y = std::max(particle.Velocity.y, bounce);
					const float normalChange = std::max(0.0f, particle.Velocity.y - predictedVelocity.y);
					const float speed = std::sqrt(particle.Velocity.x * particle.Velocity.x + particle.Velocity.z * particle.Velocity.z);
					const float friction = speed > 1.0e-8f ? std::max(0.0f, 1.0f - m_Settings.Friction * normalChange / speed) : 0.0f;
					particle.Velocity.x *= friction;
					particle.Velocity.z *= friction;
				}
			}
		}
		for (DeformableParticle& particle : m_Particles) particle.Force = Vector3::Zero();
		UpdateRenderGeometry();
	}

	void DeformableBody::Reset()
	{
		for (DeformableParticle& particle : m_Particles)
		{
			particle.Position = particle.PreviousPosition = particle.PinPosition = particle.InitialPosition;
			particle.Velocity = particle.Force = Vector3::Zero();
		}
		for (DeformableEdge& edge : m_Edges)
		{
			edge.RestLength = edge.InitialRestLength;
			edge.Lambda = 0.0f;
		}
		for (DeformableTetrahedron& tetrahedron : m_Tetrahedra)
		{
			tetrahedron.RestVolume = tetrahedron.InitialRestVolume;
			tetrahedron.Lambda = 0.0f;
		}
		std::fill(m_GroundContacts.begin(), m_GroundContacts.end(), 0);
		std::fill(m_GroundCorrections.begin(), m_GroundCorrections.end(), 0.0f);
		UpdateRenderGeometry();
	}

	float DeformableBody::GetTetrahedronVolume(int index) const
	{
		if (index < 0 || (size_t)index >= m_Tetrahedra.size()) return 0.0f;
		const DeformableTetrahedron& tetrahedron = m_Tetrahedra[(size_t)index];
		return SignedVolume(m_Particles[(size_t)tetrahedron.Particles[0]].Position,
			m_Particles[(size_t)tetrahedron.Particles[1]].Position,
			m_Particles[(size_t)tetrahedron.Particles[2]].Position,
			m_Particles[(size_t)tetrahedron.Particles[3]].Position);
	}
}
