#pragma once

#include <vector>

#include "../Maths/Index3.h"
#include "../Maths/Vector3.h"

namespace Riemann
{
	class Geometry;

	struct DeformableBodySettings
	{
		DeformableBodySettings();

		int Substeps;
		int SolverIterations;
		float EdgeCompliance;
		float VolumeCompliance;
		float Damping; // Velocity decay rate per second.
		Vector3 Gravity;
		float YieldStrain; // Elastic strain threshold; rigid edges do not yield.
		float PlasticCreep;
		float MaxPlasticStrain;
		bool GroundEnabled;
		float GroundHeight;
		float ParticleRadius;
		float Restitution;
		float Friction;
	};

	struct DeformableParticle
	{
		Vector3 Position;
		Vector3 PreviousPosition;
		Vector3 InitialPosition;
		Vector3 PinPosition;
		Vector3 Velocity;
		Vector3 Force;
		float InverseMass;
		bool Pinned;
	};

	struct DeformableEdge
	{
		int ParticleA;
		int ParticleB;
		float RestLength;
		float InitialRestLength;
		float Compliance;
		float Lambda;
	};

	struct DeformableTetrahedron
	{
		int Particles[4];
		float RestVolume;
		float InitialRestVolume;
		float Compliance;
		float Lambda;
	};

	class DeformableBody
	{
	public:
		explicit DeformableBody(const DeformableBodySettings& settings = DeformableBodySettings());
		~DeformableBody();
		DeformableBody(const DeformableBody&) = delete;
		DeformableBody& operator=(const DeformableBody&) = delete;

		int AddParticle(const Vector3& position, float inverseMass = 1.0f);
		int AddEdge(int particleA, int particleB, float compliance = -1.0f);
		// Tetrahedra also create their six elastic edges, sharing existing edges.
		int AddTetrahedron(int a, int b, int c, int d, float compliance = -1.0f);
		bool SetSurfaceTriangles(const std::vector<Index3>& triangles);
		bool PinParticle(int index, const Vector3& position);
		bool UnpinParticle(int index);
		bool SetParticlePosition(int index, const Vector3& position, bool resetVelocity = true);
		bool SetParticleVelocity(int index, const Vector3& velocity);
		bool AddForce(int index, const Vector3& force);
		bool AddImpulse(int index, const Vector3& impulse);
		// The total impulse is distributed by normalized radial weights.
		int AddRadialImpulse(const Vector3& center, float radius, const Vector3& impulse);
		void Simulate(float dt);
		// Restore the original shape and material state while retaining pin flags.
		void Reset();

		int GetParticleCount() const { return (int)m_Particles.size(); }
		const DeformableParticle& GetParticle(int index) const { return m_Particles[(size_t)index]; }
		const std::vector<DeformableParticle>& GetParticles() const { return m_Particles; }
		const std::vector<DeformableEdge>& GetEdges() const { return m_Edges; }
		const std::vector<DeformableTetrahedron>& GetTetrahedra() const { return m_Tetrahedra; }
		const std::vector<Index3>& GetSurfaceTriangles() const { return m_SurfaceTriangles; }
		const DeformableBodySettings& GetSettings() const { return m_Settings; }
		Geometry* GetRenderGeometry() const { return m_RenderGeometry; }
		float GetTetrahedronVolume(int index) const;

	private:
		bool IsValidParticle(int index) const;
		float GetInverseMass(int index) const;
		void UpdatePlasticity(float dt);
		void SolveEdges(float dt);
		void SolveVolumes(float dt);
		void SolveGround();
		void UpdateRenderGeometry();

	private:
		DeformableBodySettings m_Settings;
		std::vector<DeformableParticle> m_Particles;
		std::vector<DeformableEdge> m_Edges;
		std::vector<DeformableTetrahedron> m_Tetrahedra;
		std::vector<Index3> m_SurfaceTriangles;
		std::vector<unsigned char> m_GroundContacts;
		std::vector<float> m_GroundCorrections;
		Geometry* m_RenderGeometry = nullptr;
		bool m_RenderTopologyDirty = true;
	};
}
