#pragma once

#include <vector>

#include "../Maths/Vector3.h"

namespace Riemann
{
	class Geometry;
	class RigidBody;

	struct ClothSettings
	{
		ClothSettings();

		int Rows;
		int Columns;
		int SolverIterations;
		float Width;
		float Height;
		float Thickness;
		float ParticleInverseMass;
		float Damping;
		Vector3 Gravity;
	};

	struct ClothParticle
	{
		Vector3 Position;
		Vector3 PreviousPosition;
		Vector3 PinPosition;
		float InverseMass;
		bool Pinned;
	};

	class ClothSystem
	{
	public:
		ClothSystem(const ClothSettings& settings, const Vector3& center, const Vector3& horizontalAxis, const Vector3& verticalAxis);
		~ClothSystem();

		void Simulate(float dt);
		bool PinParticle(int row, int column, const Vector3& worldPosition);
		bool UnpinParticle(int row, int column);
		bool AddSphereCollider(RigidBody* body, float radius, const Vector3& worldCenter);
		bool RemoveSphereCollider(RigidBody* body);

		int GetRows() const { return m_Settings.Rows; }
		int GetColumns() const { return m_Settings.Columns; }
		int GetParticleCount() const { return (int)m_Particles.size(); }
		const ClothParticle& GetParticle(int row, int column) const { return m_Particles[(size_t)ParticleIndex(row, column)]; }
		ClothParticle& GetParticle(int row, int column) { return m_Particles[(size_t)ParticleIndex(row, column)]; }
		Geometry* GetRenderGeometry() const { return m_RenderGeometry; }

	private:
		struct DistanceLink
		{
			int ParticleA;
			int ParticleB;
			float RestLength;
		};

		struct SphereCollider
		{
			RigidBody* Body;
			Vector3 LocalCenter;
			float Radius;
		};

		int ParticleIndex(int row, int column) const { return row * m_Settings.Columns + column; }
		void AddLink(int particleA, int particleB);
		void SolveDistanceConstraints();
		void SolvePins();
		void SolveSphereCollisions();
		void BuildRenderMesh();
		void UpdateRenderMesh();

	private:
		ClothSettings m_Settings;
		std::vector<ClothParticle> m_Particles;
		std::vector<DistanceLink> m_Links;
		std::vector<SphereCollider> m_SphereColliders;
		Geometry* m_RenderGeometry;
	};
}
