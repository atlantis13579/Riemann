#pragma once

#include "SurfaceContactConstraint.h"
#include "RigidPointResponse.h"

namespace Riemann
{
	class RigidSurfaceContact
	{
	public:
		using DVector = RigidPointResponse::DVector;
		RigidSurfaceContact(const SurfaceContactParticle* Particles, int Count, const RigidConstraintBody& Rigid,
			const Vector3& LocalArm, const SurfaceContact& Contact, double Thickness,
			const SurfaceContactMaterial& Material = SurfaceContactMaterial(), const RigidResponseSettings& Settings = RigidResponseSettings());
		void Refresh(const SurfaceContactParticle* Particles, int Count, const RigidConstraintBody& Rigid,
			const Vector3& LocalArm, const SurfaceContact& Contact, double Thickness);
		bool Solve(float Dt);
		double GetNormalGap() const;
		double GetNormalMultiplier() const { return m_NormalMultiplier; }
		DVector GetTangentMultiplier() const { return m_TangentMultiplier; }
		DVector GetRigidAngularMultiplier() const { return m_AngularMultiplier; }
		bool IsSliding() const { return m_Sliding; }
	private:
		struct Particle { Vector3* Position = nullptr; double InverseMass = 0, Weight = 0; };
		DVector SoftPoint() const;
		std::array<Particle, 3> m_Particles;
		int m_Count = 0;
		double m_SoftMass = 0, m_Thickness = 0, m_NormalMultiplier = 0;
		DVector m_Normal = DVector(0), m_Reference = DVector(0), m_TangentMultiplier = DVector(0), m_AngularMultiplier = DVector(0);
		SurfaceContactMaterial m_Material;
		RigidResponseSettings m_Settings;
		RigidPointResponse m_Rigid;
		bool m_Sliding = false;
	};
}
