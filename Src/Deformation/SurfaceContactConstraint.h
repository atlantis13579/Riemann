#pragma once

#include <array>
#include "../Collision/SurfaceCollision.h"

namespace Riemann
{
	struct SurfaceContactMaterial
	{
		double Compliance = 0;
		double StaticFriction = 0.5, DynamicFriction = 0.3;
		bool IsValid() const;
	};
	struct SurfaceContactParticle
	{
		Vector3* Position = nullptr;
		double InverseMass = 0;
	};
	class SurfaceContactConstraint
	{
	public:
		SurfaceContactConstraint(const SurfaceContactParticle* A, int CountA, const SurfaceContactParticle* B, int CountB,
			const SurfaceContact& Contact, double Thickness, const SurfaceContactMaterial& Material = SurfaceContactMaterial());
		void Refresh(const SurfaceContactParticle* A, int CountA, const SurfaceContactParticle* B, int CountB,
			const SurfaceContact& Contact, double Thickness);
		bool Solve(double Dt);
		bool ProjectSeparation();
		double GetNormalGap() const;
		double GetNormalMultiplier() const { return m_NormalMultiplier; }
		Maths::TVector3<double> GetTangentMultiplier() const { return m_TangentMultiplier; }
		double GetEffectiveInverseMass() const { return m_InverseMass; }
		bool IsSliding() const { return m_Sliding; }
	private:
		using DVector = Maths::TVector3<double>;
		struct Particle { Vector3* Position = nullptr; double InverseMass = 0, Weight = 0; };
		void Add(const SurfaceContactParticle* Particles, int Count, const Vector3& Weights, double Sign);
		DVector Separation() const;
		std::array<Particle, 6> m_Particles;
		int m_Count = 0;
		DVector m_Normal = DVector(0), m_Reference = DVector(0), m_TangentMultiplier = DVector(0);
		double m_Thickness = 0, m_InverseMass = 0, m_NormalMultiplier = 0;
		SurfaceContactMaterial m_Material;
		bool m_Sliding = false;
	};
}
