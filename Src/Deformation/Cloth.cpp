#include "Cloth.h"

#include <algorithm>

#include "../RigidBodyDynamics/RigidBody.h"
#include "../Collision/GeometryObject.h"
#include "../CollisionPrimitive/StaticMesh.h"
#include "../CollisionPrimitive/TriangleMesh.h"

namespace Riemann
{
	ClothSettings::ClothSettings()
		: Rows(16)
		, Columns(16)
		, SolverIterations(12)
		, Width(8.0f)
		, Height(8.0f)
		, Thickness(0.08f)
		, ParticleInverseMass(1.0f)
		, Damping(0.995f)
		, Gravity(0.0f, -9.8f, 0.0f)
	{
	}

	ClothSystem::ClothSystem(const ClothSettings& settings, const Vector3& center, const Vector3& horizontalAxis, const Vector3& verticalAxis)
		: m_Settings(settings)
		, m_RenderGeometry(nullptr)
	{
		m_Settings.Rows = std::max(2, m_Settings.Rows);
		m_Settings.Columns = std::max(2, m_Settings.Columns);
		m_Settings.SolverIterations = std::max(1, m_Settings.SolverIterations);
		m_Settings.Width = std::max(0.01f, m_Settings.Width);
		m_Settings.Height = std::max(0.01f, m_Settings.Height);
		m_Settings.Thickness = std::max(0.001f, m_Settings.Thickness);
		m_Settings.ParticleInverseMass = std::max(0.0f, m_Settings.ParticleInverseMass);
		m_Settings.Damping = std::max(0.0f, std::min(1.0f, m_Settings.Damping));

		Vector3 horizontal = horizontalAxis.SafeUnit();
		Vector3 vertical = verticalAxis.SafeUnit();
		if (horizontal.IsZero()) horizontal = Vector3::UnitX();
		if (vertical.IsZero()) vertical = Vector3::UnitY();
		vertical = (vertical - horizontal * vertical.Dot(horizontal)).SafeUnit();
		if (vertical.IsZero())
		{
			Vector3 unused;
			horizontal.GetPerpVectors(vertical, unused);
		}

		m_Particles.resize((size_t)m_Settings.Rows * (size_t)m_Settings.Columns);
		for (int row = 0; row < m_Settings.Rows; ++row)
		{
			for (int column = 0; column < m_Settings.Columns; ++column)
			{
				const float u = (float)column / (float)(m_Settings.Columns - 1) - 0.5f;
				const float v = 0.5f - (float)row / (float)(m_Settings.Rows - 1);
				ClothParticle& particle = m_Particles[(size_t)ParticleIndex(row, column)];
				particle.Position = center + horizontal * (u * m_Settings.Width) + vertical * (v * m_Settings.Height);
				particle.PreviousPosition = particle.Position;
				particle.PinPosition = particle.Position;
				particle.InverseMass = m_Settings.ParticleInverseMass;
				particle.Pinned = false;
			}
		}

		for (int row = 0; row < m_Settings.Rows; ++row)
		{
			for (int column = 0; column < m_Settings.Columns; ++column)
			{
				const int particle = ParticleIndex(row, column);
				if (column + 1 < m_Settings.Columns) AddLink(particle, ParticleIndex(row, column + 1));
				if (row + 1 < m_Settings.Rows) AddLink(particle, ParticleIndex(row + 1, column));
				if (row + 1 < m_Settings.Rows && column + 1 < m_Settings.Columns)
				{
					AddLink(particle, ParticleIndex(row + 1, column + 1));
					AddLink(ParticleIndex(row, column + 1), ParticleIndex(row + 1, column));
				}
				if (column + 2 < m_Settings.Columns) AddLink(particle, ParticleIndex(row, column + 2));
				if (row + 2 < m_Settings.Rows) AddLink(particle, ParticleIndex(row + 2, column));
			}
		}

		BuildRenderMesh();
	}

	ClothSystem::~ClothSystem()
	{
		delete m_RenderGeometry;
		m_RenderGeometry = nullptr;
	}

	void ClothSystem::AddLink(int particleA, int particleB)
	{
		DistanceLink link;
		link.ParticleA = particleA;
		link.ParticleB = particleB;
		link.RestLength = (m_Particles[(size_t)particleB].Position - m_Particles[(size_t)particleA].Position).Length();
		m_Links.push_back(link);
	}

	bool ClothSystem::PinParticle(int row, int column, const Vector3& worldPosition)
	{
		if (row < 0 || row >= m_Settings.Rows || column < 0 || column >= m_Settings.Columns)
		{
			return false;
		}
		ClothParticle& particle = m_Particles[(size_t)ParticleIndex(row, column)];
		particle.Pinned = true;
		particle.PinPosition = worldPosition;
		particle.Position = worldPosition;
		particle.PreviousPosition = worldPosition;
		UpdateRenderMesh();
		return true;
	}

	bool ClothSystem::UnpinParticle(int row, int column)
	{
		if (row < 0 || row >= m_Settings.Rows || column < 0 || column >= m_Settings.Columns)
		{
			return false;
		}
		m_Particles[(size_t)ParticleIndex(row, column)].Pinned = false;
		return true;
	}

	bool ClothSystem::AddSphereCollider(RigidBody* body, float radius, const Vector3& worldCenter)
	{
		if (body == nullptr || radius <= 0.0f)
		{
			return false;
		}
		RemoveSphereCollider(body);
		SphereCollider collider;
		collider.Body = body;
		collider.LocalCenter = body->WorldToBodyLocal(worldCenter);
		collider.Radius = radius;
		m_SphereColliders.push_back(collider);
		return true;
	}

	bool ClothSystem::RemoveSphereCollider(RigidBody* body)
	{
		const size_t oldSize = m_SphereColliders.size();
		m_SphereColliders.erase(std::remove_if(m_SphereColliders.begin(), m_SphereColliders.end(),
			[body](const SphereCollider& collider) { return collider.Body == body; }), m_SphereColliders.end());
		return oldSize != m_SphereColliders.size();
	}

	void ClothSystem::Simulate(float dt)
	{
		if (dt <= 0.0f)
		{
			return;
		}
		const float dtSquared = dt * dt;
		for (ClothParticle& particle : m_Particles)
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
			SolveSphereCollisions();
		}
		UpdateRenderMesh();
	}

	void ClothSystem::SolveDistanceConstraints()
	{
		for (const DistanceLink& link : m_Links)
		{
			ClothParticle& particleA = m_Particles[(size_t)link.ParticleA];
			ClothParticle& particleB = m_Particles[(size_t)link.ParticleB];
			const Vector3 delta = particleB.Position - particleA.Position;
			const float distance = delta.Length();
			if (distance <= 1e-7f) continue;
			const float weightA = particleA.Pinned ? 0.0f : particleA.InverseMass;
			const float weightB = particleB.Pinned ? 0.0f : particleB.InverseMass;
			const float weightSum = weightA + weightB;
			if (weightSum <= 1e-7f) continue;
			const Vector3 correction = delta * ((distance - link.RestLength) / distance);
			particleA.Position += correction * (weightA / weightSum);
			particleB.Position -= correction * (weightB / weightSum);
		}
	}

	void ClothSystem::SolvePins()
	{
		for (ClothParticle& particle : m_Particles)
		{
			if (particle.Pinned) particle.Position = particle.PinPosition;
		}
	}

	void ClothSystem::SolveSphereCollisions()
	{
		for (const SphereCollider& collider : m_SphereColliders)
		{
			if (collider.Body == nullptr) continue;
			const Vector3 center = collider.Body->BodyLocalToWorld(collider.LocalCenter);
			const float minDistance = collider.Radius + m_Settings.Thickness;
			for (ClothParticle& particle : m_Particles)
			{
				if (particle.Pinned || particle.InverseMass <= 0.0f) continue;
				Vector3 delta = particle.Position - center;
				float distance = delta.Length();
				if (distance >= minDistance) continue;
				if (distance <= 1e-7f)
				{
					delta = Vector3::UnitZ();
					distance = 1.0f;
				}
				particle.Position = center + delta * (minDistance / distance);
			}
		}
	}

	void ClothSystem::BuildRenderMesh()
	{
		m_RenderGeometry = GeometryFactory::CreateTriangleMesh();
		m_RenderGeometry->SetQueryEnabled(false);
		m_RenderGeometry->SetSimulationEnabled(false);
		TriangleMesh* mesh = m_RenderGeometry->GetShapeObj<TriangleMesh>();
		mesh->Flags |= INDICES_16_BIT;
		for (const ClothParticle& particle : m_Particles) mesh->AddVertex(particle.Position);
		for (int row = 0; row + 1 < m_Settings.Rows; ++row)
		{
			for (int column = 0; column + 1 < m_Settings.Columns; ++column)
			{
				const uint32_t i0 = (uint32_t)ParticleIndex(row, column);
				const uint32_t i1 = (uint32_t)ParticleIndex(row, column + 1);
				const uint32_t i2 = (uint32_t)ParticleIndex(row + 1, column);
				const uint32_t i3 = (uint32_t)ParticleIndex(row + 1, column + 1);
				mesh->AddTriangle(i0, i2, i1);
				mesh->AddTriangle(i1, i2, i3);
			}
		}
		mesh->Compact();
		m_RenderGeometry->SetShapeBounds(mesh->BoundingVolume);
	}

	void ClothSystem::UpdateRenderMesh()
	{
		TriangleMesh* mesh = m_RenderGeometry ? m_RenderGeometry->GetShapeObj<TriangleMesh>() : nullptr;
		if (mesh == nullptr || mesh->NumVertices != m_Particles.size()) return;
		mesh->BoundingVolume.SetEmpty();
		for (size_t i = 0; i < m_Particles.size(); ++i)
		{
			mesh->Vertices[i] = m_Particles[i].Position;
			mesh->BoundingVolume.Encapsulate(m_Particles[i].Position);
		}
		mesh->mNormals.clear();
		m_RenderGeometry->SetShapeBounds(mesh->BoundingVolume);
	}
}
