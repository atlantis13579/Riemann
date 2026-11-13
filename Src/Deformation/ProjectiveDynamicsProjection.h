#pragma once

#include <vector>
#include "TetrahedralMaterial.h"

namespace Riemann
{
	struct ProjectiveDynamicsSettings
	{
		int MaximumLinearIterations = 128;
		double RelativeLinearTolerance = 1e-8;
		bool IsValid() const;
	};
	struct ProjectiveDynamicsParticle
	{
		Vector3* Position = nullptr;
		float InverseMass = 0;
	};
	struct ProjectiveDynamicsElement
	{
		std::array<int,4> Particles = {};
		TetrahedralRestShape Rest;
		double ShearModulus = 0;
	};
	struct ProjectiveDynamicsStats
	{
		int LinearIterations = 0;
		double RelativeLinearResidual = 0;
		double AcceptedStep = 0;
		bool Converged = true;
	};
	class ProjectiveDynamicsProjection
	{
	public:
		void Begin(const std::vector<ProjectiveDynamicsParticle>& Particles, const std::vector<ProjectiveDynamicsElement>& Elements,
			double Dt, const ProjectiveDynamicsSettings& Settings = {}, const TetrahedralSolverSettings& Safety = {});
		bool Project();
		void End();
		const ProjectiveDynamicsStats& GetStats() const { return m_Stats; }
	private:
		using DVector = DeformationGradient::DVector;
		void Multiply(const std::vector<DVector>& Input, std::vector<DVector>& Output) const;
		std::vector<ProjectiveDynamicsParticle> m_Particles;
		std::vector<ProjectiveDynamicsElement> m_Elements;
		std::vector<double> m_InertialWeights, m_Diagonal;
		std::vector<DVector> m_Corrections, m_Residual, m_Direction, m_Preconditioned, m_Product, m_Delta;
		std::vector<Vector3> m_Next;
		ProjectiveDynamicsSettings m_Settings;
		TetrahedralSolverSettings m_Safety;
		ProjectiveDynamicsStats m_Stats;
		bool m_Active = false;
	};
}
