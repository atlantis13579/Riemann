#pragma once

#include <array>
#include "../Maths/Vector3.h"

namespace Riemann
{
	struct DeformationGradient
	{
		using DVector = Maths::TVector3<double>;
		double Values[3][3] = {};
		double* operator[](int I) { return Values[I]; }
		const double* operator[](int I) const { return Values[I]; }
		DVector operator*(const DVector& V) const;
		double Determinant() const;
		DeformationGradient Cofactor() const;
	};
	enum class TetrahedralMaterialModel { StableNeoHookean, SaintVenantKirchhoff, Corotated };
	enum class TetrahedralSolveChannels { All, Volume };
	struct TetrahedralMaterial
	{
		TetrahedralMaterialModel Model = TetrahedralMaterialModel::StableNeoHookean;
		double YoungModulus = 10000, PoissonRatio = 0.3, Density = 1000;
		bool IsValid() const;
		double ShearModulus() const;
		double LameLambda() const;
	};
	struct TetrahedralSolverSettings
	{
		double RelativeVolumeTolerance = 1e-8;
		double MinimumJacobian = 0.05;
		double MaximumCorrectionRatio = 0.25;
		int MaximumLineSearchIterations = 24;
		bool IsValid() const;
	};
	struct TetrahedralRestShape
	{
		using DVector = DeformationGradient::DVector;
		std::array<DVector, 4> Gradients = { DVector(0), DVector(0), DVector(0), DVector(0) };
		double Volume = 0, CharacteristicLength = 0;
		static TetrahedralRestShape Create(const Vector3 Positions[4], const TetrahedralSolverSettings& Settings = TetrahedralSolverSettings());
		DeformationGradient Evaluate(const Vector3 Positions[4]) const;
	};
	struct TetrahedralMaterialResponse
	{
		double EnergyDensity = 0, Jacobian = 0;
		DeformationGradient FirstPiola;
		std::array<double, 2> Constraints = {}, Stiffness = {};
		std::array<DeformationGradient, 2> Gradients;
	};
	struct TetrahedralMaterialState
	{
		std::array<double, 2> Multipliers = {};
		std::array<DeformationGradient::DVector, 4> Corrections = { DeformationGradient::DVector(0), DeformationGradient::DVector(0), DeformationGradient::DVector(0), DeformationGradient::DVector(0) };
	};
	class TetrahedralElasticity
	{
	public:
		static DeformationGradient ClosestRotation(const DeformationGradient& F);
		static TetrahedralMaterialResponse Evaluate(const DeformationGradient& F, const TetrahedralMaterial& Material);
		static bool Solve(Vector3* const Positions[4], const float InverseMasses[4], const TetrahedralRestShape& Rest,
			const TetrahedralMaterial& Material, TetrahedralMaterialState& State, float Dt,
			const TetrahedralSolverSettings& Settings = TetrahedralSolverSettings(), TetrahedralSolveChannels Channels = TetrahedralSolveChannels::All);
	};
}
