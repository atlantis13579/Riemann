#pragma once

#include <array>
#include <cstddef>
#include "../Maths/Index3.h"
#include "SurfaceContactConstraint.h"
#include "RigidPointResponse.h"

namespace Riemann
{
	struct BarycentricPoint
	{
		static constexpr int MaximumVertices = 4;
		int Count = 0;
		std::array<int, MaximumVertices> Indices = { -1, -1, -1, -1 };
		std::array<double, MaximumVertices> Weights = {};
		static BarycentricPoint Vertex(int Index);
		static BarycentricPoint Triangle(const Index3& Indices, const Vector3& Weights);
		static BarycentricPoint Tetrahedron(const std::array<int, MaximumVertices>& Indices,
			const std::array<double, MaximumVertices>& Weights);
		bool IsValid(size_t ParticleCount) const;
	};

	class BarycentricAttachmentConstraint
	{
	public:
		using DVector = Maths::TVector3<double>;
		BarycentricAttachmentConstraint(const SurfaceContactParticle* A, const double* WeightsA, int CountA,
			const SurfaceContactParticle* B, const double* WeightsB, int CountB, double Compliance = 0);
		BarycentricAttachmentConstraint(const SurfaceContactParticle* A, const double* WeightsA, int CountA,
			const RigidConstraintBody& Body, const Vector3& LocalArm, double Compliance = 0,
			const RigidResponseSettings& Settings = RigidResponseSettings());
		bool Solve(float Dt);
		DVector GetError() const;
		DVector GetMultiplier() const { return m_Multiplier; }
		DVector GetForce(float Dt) const;
		DVector GetRigidAngularMultiplier() const { return m_RigidAngularMultiplier; }
		DVector GetRigidTorque(float Dt) const;
		double GetEffectiveInverseMass() const;
		ConstraintMassMatrix GetEffectiveInverseMassMatrix() const;
	private:
		struct Particle { Vector3* Position = nullptr; double InverseMass = 0, Weight = 0; };
		void Add(const SurfaceContactParticle* Particles, const double* Weights, int Count, double Sign);
		std::array<Particle, BarycentricPoint::MaximumVertices * 2> m_Particles;
		int m_Count = 0;
		double m_InverseMass = 0, m_Compliance = 0;
		DVector m_Multiplier = DVector(0);
		bool m_HasRigid = false;
		RigidPointResponse m_Rigid;
		DVector m_RigidAngularMultiplier = DVector(0);
	};
}
