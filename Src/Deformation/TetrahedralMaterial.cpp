#include "TetrahedralMaterial.h"
#include "../LinearSystem/SingularValueDecomposition.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace Riemann
{
	/* Material pipeline
	   -----------------
	   Rest-shape gradients belong to prepared geometry. Constitutive parameters
	   belong to a material table; XPBD multipliers belong to one physical substep.
	   A future mesh cooker/embedding adapter can consume these rest shapes without
	   owning integration. Contact and attachment rows join the same position solve.
	   The existing edge/volume model remains an independent per-element option;
	   its automatically generated springs must be disabled for FEM-only edges.

	   Units are meters, kilograms, seconds and Pascals. With shear modulus mu and
	   Lame lambda, the polynomial stable Neo-Hookean energy density used here is
	     psi = mu/2 (F:F-3) - mu (J-1) + (lambda+mu)/2 (J-1)^2.
	   Its small-strain tangent matches the requested Young modulus and Poisson
	   ratio. The shifted determinant coefficient is lambda+mu, not lambda: omitting
	   that calibration changes the measured bulk modulus. Its two force-equivalent
	   constraints are ||F|| and J-1-mu/(lambda+mu), with stiffness mu and lambda+mu.
	   Their nonzero rest values cancel in stress. Solve their 2x2 XPBD block jointly
	   so that this cancellation also holds at rest with a finite iteration budget.
	   Track this element's actual accumulated position correction p as well. When
	   its Jacobian changes, e=W J^T lambda-p is a stationarity residual. Include
	   J e in the block right-hand side and apply e+W J^T deltaLambda. Ignoring e
	   leaves the old gradients in the accumulated response and biases calibration,
	   especially for the nonzero Neo-Hookean rest multipliers. Other elements and
	   contacts retain their own corrections; this is not a second integration.

	   The comparison model is Saint Venant-Kirchhoff: E=(F^T F-I)/2 and
	     psi = mu (E:E) + lambda/2 tr(E)^2.
	   Its two constraints are sqrt(2)||E|| and tr(E). Both models expose their
	   first Piola stress and energy for calibration. This implementation accepts
	   non-auxetic materials, 0 <= nu < 1/2. It does not model plastic flow or fracture.

	   Constant-strain tetrahedra integrate energy by multiplying by rest volume.
	   Thus compliance is 1/(stiffness*volume), not a mesh-independent spring value.
	   Gradients use double precision and rest-relative edges, avoiding translation
	   cancellation. A determinant barrier and a bounded complete correction protect
	   orientation; line search preserves an already feasible element. A fully
	   collapsed element has no determinant gradient and needs remeshing/recovery
	   outside this local solve. No velocity or external load is integrated here.

	   References: Macklin and Mueller, "A Constraint-based Formulation of Stable
	   Neo-Hookean Materials" (MIG 2021), sections 3-4:
	   https://matthias-research.github.io/pages/publications/neohookean.pdf
	   Fixed-corotated elasticity adds mu ||F-R||^2 + lambda/2 (J-1)^2. The polar
	   rotation is the nearest SO(3) matrix. Rotation derivatives vanish in the
	   energy gradient by local optimality, giving 2 mu (F-R). This same energy can
	   use both XPBD modes, or the sparse projective rotation stage plus only this
	   adapter's bulk mode. Each block keeps its own displacement; world prediction
	   and velocity reconstruction still happen once. */
	namespace
	{
		using DVector = DeformationGradient::DVector;
		constexpr double RestEnergyTolerance = 1e-12;
		DVector Double(const Vector3& P) { return DVector(P.x, P.y, P.z); }
		bool Finite(const DVector& P) { return std::isfinite(P.x) && std::isfinite(P.y) && std::isfinite(P.z); }
		double Length(const DVector& P) { return std::hypot(std::hypot(P.x, P.y), P.z); }
		bool Positive(double X) { return std::isfinite(X) && X > 0; }
		void Validate(const DeformationGradient& F)
		{
			for (int I = 0; I < 3; ++I) for (int J = 0; J < 3; ++J)
				if (!std::isfinite(F[I][J])) throw std::invalid_argument("Non-finite deformation gradient");
		}
		bool Apply(Vector3* const Positions[4], const std::array<DVector, 4>& Delta, double& Step,
			const TetrahedralRestShape& Rest, const TetrahedralSolverSettings& Settings)
		{
			double Maximum = 0; Vector3 Start[4];
			for (int I = 0; I < 4; ++I) { Maximum = std::max(Maximum, Length(Delta[I])); Start[I] = *Positions[I]; }
			if (!std::isfinite(Maximum)) throw std::overflow_error("Tetrahedral correction overflow");
			Step = Maximum > Rest.CharacteristicLength * Settings.MaximumCorrectionRatio ? Rest.CharacteristicLength * Settings.MaximumCorrectionRatio / Maximum : 1;
			const bool Feasible = Rest.Evaluate(Start).Determinant() >= Settings.MinimumJacobian;
			Vector3 Next[4];
			for (int Trial = 0; Trial < Settings.MaximumLineSearchIterations; ++Trial)
			{
				for (int I = 0; I < 4; ++I)
				{
					const auto P = Double(Start[I]) + Delta[I] * Step; Next[I] = Vector3((float)P.x, (float)P.y, (float)P.z);
					if (!Finite(Double(Next[I]))) throw std::overflow_error("Tetrahedral position overflow");
				}
				if (!Feasible || Rest.Evaluate(Next).Determinant() >= Settings.MinimumJacobian)
				{
					for (int I = 0; I < 4; ++I) *Positions[I] = Next[I]; return Maximum > 0;
				}
				Step /= 2;
			}
			Step = 0; return false;
		}
	}
	DeformationGradient::DVector DeformationGradient::operator*(const DVector& V) const
	{
		return DVector(Values[0][0] * V.x + Values[0][1] * V.y + Values[0][2] * V.z,
			Values[1][0] * V.x + Values[1][1] * V.y + Values[1][2] * V.z,
			Values[2][0] * V.x + Values[2][1] * V.y + Values[2][2] * V.z);
	}
	DeformationGradient DeformationGradient::Cofactor() const
	{
		DeformationGradient C;
		for (int I = 0; I < 3; ++I) for (int J = 0; J < 3; ++J)
			C[I][J] = Values[(I + 1) % 3][(J + 1) % 3] * Values[(I + 2) % 3][(J + 2) % 3]
				- Values[(I + 1) % 3][(J + 2) % 3] * Values[(I + 2) % 3][(J + 1) % 3];
		return C;
	}
	double DeformationGradient::Determinant() const
	{
		const auto C = Cofactor(); return Values[0][0] * C[0][0] + Values[0][1] * C[0][1] + Values[0][2] * C[0][2];
	}
	double TetrahedralMaterial::ShearModulus() const { return YoungModulus / (2 * (1 + PoissonRatio)); }
	double TetrahedralMaterial::LameLambda() const { return ShearModulus() * (2 * PoissonRatio / (1 - 2 * PoissonRatio)); }
	bool TetrahedralMaterial::IsValid() const
	{
		return (Model == TetrahedralMaterialModel::StableNeoHookean || Model == TetrahedralMaterialModel::SaintVenantKirchhoff || Model == TetrahedralMaterialModel::Corotated)
			&& Positive(YoungModulus) && Positive(Density) && std::isfinite(PoissonRatio) && PoissonRatio >= 0 && PoissonRatio < 0.5
			&& Positive(ShearModulus()) && std::isfinite(LameLambda() + ShearModulus());
	}
	bool TetrahedralSolverSettings::IsValid() const
	{
		return Positive(RelativeVolumeTolerance) && RelativeVolumeTolerance < 1 && Positive(MinimumJacobian) && MinimumJacobian < 1
			&& Positive(MaximumCorrectionRatio) && MaximumLineSearchIterations > 0;
	}
	TetrahedralRestShape TetrahedralRestShape::Create(const Vector3 Positions[4], const TetrahedralSolverSettings& Settings)
	{
		if (!Positions || !Settings.IsValid()) throw std::invalid_argument("Invalid tetrahedral rest shape settings");
		for (int I = 0; I < 4; ++I) if (!Finite(Double(Positions[I]))) throw std::invalid_argument("Non-finite tetrahedral rest position");
		const DVector A = Double(Positions[1]) - Double(Positions[0]), B = Double(Positions[2]) - Double(Positions[0]), C = Double(Positions[3]) - Double(Positions[0]);
		const double Determinant = A.Dot(B.Cross(C)); TetrahedralRestShape Rest;
		Rest.CharacteristicLength = std::max(Length(A), std::max(Length(B), Length(C)));
		if (!Positive(Rest.CharacteristicLength) || !std::isfinite(Determinant) || std::abs(Determinant) <= Settings.RelativeVolumeTolerance * Rest.CharacteristicLength * Rest.CharacteristicLength * Rest.CharacteristicLength)
			throw std::invalid_argument("Degenerate tetrahedral rest shape");
		Rest.Volume = std::abs(Determinant) / 6;
		Rest.Gradients[1] = B.Cross(C) / Determinant; Rest.Gradients[2] = C.Cross(A) / Determinant; Rest.Gradients[3] = A.Cross(B) / Determinant;
		Rest.Gradients[0] = -Rest.Gradients[1] - Rest.Gradients[2] - Rest.Gradients[3]; return Rest;
	}
	DeformationGradient TetrahedralRestShape::Evaluate(const Vector3 Positions[4]) const
	{
		if (!Positions || !Positive(Volume) || !Positive(CharacteristicLength)) throw std::invalid_argument("Invalid prepared tetrahedral rest shape");
		for (const auto& G : Gradients) if (!Finite(G)) throw std::invalid_argument("Non-finite tetrahedral rest gradient");
		DeformationGradient F;
		for (int K = 1; K < 4; ++K)
		{
			const auto Edge = Double(Positions[K]) - Double(Positions[0]);
			if (!Finite(Edge)) throw std::invalid_argument("Non-finite tetrahedral position");
			for (int I = 0; I < 3; ++I) for (int J = 0; J < 3; ++J) F[I][J] += Edge[I] * Gradients[K][J];
		}
		Validate(F); return F;
	}
	DeformationGradient TetrahedralElasticity::ClosestRotation(const DeformationGradient& F)
	{
		Validate(F); double Scale = 0;
		for (int I = 0; I < 3; ++I) for (int J = 0; J < 3; ++J) Scale = std::max(Scale, std::abs(F[I][J]));
		DeformationGradient R;
		if (!(Scale > 0)) { for (int I = 0; I < 3; ++I) R[I][I] = 1; return R; }
		double A[9], U[9], V[9], Singular[3];
		for (int I = 0; I < 3; ++I) for (int J = 0; J < 3; ++J) A[I * 3 + J] = F[I][J] / Scale;
		if (!Maths::LinearAlgebra::SingularValueDecomposition<double>()(A, 3, 3, U, Singular, V)) throw std::runtime_error("Corotated material SVD did not converge");
		auto Multiply = [&]()
		{
			R = DeformationGradient();
			for (int I = 0; I < 3; ++I) for (int J = 0; J < 3; ++J) for (int K = 0; K < 3; ++K) R[I][J] += U[I * 3 + K] * V[J * 3 + K];
		};
		Multiply();
		// The nearest SO(3) matrix corrects a reflection along the least singular
		// direction, rather than negating all three axes of an improper polar factor.
		if (R.Determinant() < 0)
		{
			int Smallest = 0; for (int I = 1; I < 3; ++I) if (Singular[I] < Singular[Smallest]) Smallest = I;
			for (int I = 0; I < 3; ++I) U[I * 3 + Smallest] = -U[I * 3 + Smallest]; Multiply();
		}
		Validate(R); return R;
	}
	TetrahedralMaterialResponse TetrahedralElasticity::Evaluate(const DeformationGradient& F, const TetrahedralMaterial& Material)
	{
		if (!Material.IsValid()) throw std::invalid_argument("Invalid tetrahedral material"); Validate(F);
		TetrahedralMaterialResponse R; R.Jacobian = F.Determinant();
		const double Mu = Material.ShearModulus(), Lambda = Material.LameLambda();
		R.Stiffness = { Mu, Lambda };
		if (Material.Model == TetrahedralMaterialModel::StableNeoHookean)
		{
			double Norm = 0; for (int I = 0; I < 3; ++I) for (int J = 0; J < 3; ++J) Norm = std::hypot(Norm, F[I][J]);
			R.Stiffness[1] += Mu; R.Constraints = { Norm, R.Jacobian - 1 - Mu / R.Stiffness[1] };
			R.Gradients[1] = F.Cofactor();
			if (Norm > 0) for (int I = 0; I < 3; ++I) for (int J = 0; J < 3; ++J) R.Gradients[0][I][J] = F[I][J] / Norm;
			const double VolumeChange = R.Jacobian - 1;
			R.EnergyDensity = Mu / 2 * (Norm * Norm - 3) - Mu * VolumeChange + R.Stiffness[1] / 2 * VolumeChange * VolumeChange;
		}
		else if (Material.Model == TetrahedralMaterialModel::Corotated)
		{
			const auto Rotation = ClosestRotation(F); DeformationGradient Difference; double Norm = 0;
			for (int I = 0; I < 3; ++I) for (int J = 0; J < 3; ++J)
			{ Difference[I][J] = F[I][J] - Rotation[I][J]; Norm = std::hypot(Norm, Difference[I][J]); }
			R.Constraints = { std::sqrt(2.0) * Norm, R.Jacobian - 1 }; R.Gradients[1] = F.Cofactor();
			if (Norm > 0) for (int I = 0; I < 3; ++I) for (int J = 0; J < 3; ++J) R.Gradients[0][I][J] = std::sqrt(2.0) * Difference[I][J] / Norm;
			R.EnergyDensity = Mu * Norm * Norm + Lambda / 2 * R.Constraints[1] * R.Constraints[1];
		}
		else
		{
			DeformationGradient Strain; double Norm = 0, Trace = 0;
			for (int I = 0; I < 3; ++I) for (int J = 0; J < 3; ++J)
			{
				for (int K = 0; K < 3; ++K) Strain[I][J] += F[K][I] * F[K][J] / 2;
				if (I == J) { Strain[I][J] -= 0.5; Trace += Strain[I][J]; }
				Norm = std::hypot(Norm, Strain[I][J]);
			}
			R.Constraints = { std::sqrt(2.0) * Norm, Trace }; R.Gradients[1] = F;
			if (Norm > 0) for (int I = 0; I < 3; ++I) for (int J = 0; J < 3; ++J)
				for (int K = 0; K < 3; ++K) R.Gradients[0][I][J] += std::sqrt(2.0) * F[I][K] * Strain[K][J] / Norm;
			R.EnergyDensity = Mu * Norm * Norm + Lambda / 2 * Trace * Trace;
		}
		for (int Mode = 0; Mode < 2; ++Mode) for (int I = 0; I < 3; ++I) for (int J = 0; J < 3; ++J)
			R.FirstPiola[I][J] += R.Stiffness[Mode] * R.Constraints[Mode] * R.Gradients[Mode][I][J];
		if (R.EnergyDensity < 0 && R.EnergyDensity > -RestEnergyTolerance * (Mu + Lambda)) R.EnergyDensity = 0;
		if (!std::isfinite(R.EnergyDensity) || !std::isfinite(R.Jacobian)) throw std::overflow_error("Tetrahedral material energy overflow");
		Validate(R.FirstPiola); return R;
	}
	bool TetrahedralElasticity::Solve(Vector3* const Positions[4], const float InverseMasses[4], const TetrahedralRestShape& Rest,
		const TetrahedralMaterial& Material, TetrahedralMaterialState& State, float Dt, const TetrahedralSolverSettings& Settings, TetrahedralSolveChannels Channels)
	{
		if (!Positions || !InverseMasses || !Material.IsValid() || !Settings.IsValid() || !Positive(Dt) || !Positive(Rest.Volume) || !Positive(Rest.CharacteristicLength))
			throw std::invalid_argument("Invalid tetrahedral material solve");
		if (Channels != TetrahedralSolveChannels::All && Channels != TetrahedralSolveChannels::Volume) throw std::invalid_argument("Invalid tetrahedral solve channels");
		Vector3 Current[4], Working[4]; Vector3* Mutable[4]; double Mass = 0;
		for (int I = 0; I < 4; ++I)
		{
			if (!Positions[I] || !Finite(Double(*Positions[I])) || !std::isfinite(InverseMasses[I]) || InverseMasses[I] < 0) throw std::invalid_argument("Invalid tetrahedral material particle");
			for (int J = 0; J < I; ++J) if (Positions[I] == Positions[J]) throw std::invalid_argument("Duplicate tetrahedral material particle");
			Current[I] = *Positions[I]; Mass += InverseMasses[I];
			Working[I] = Current[I]; Mutable[I] = &Working[I];
		}
		for (double L : State.Multipliers) if (!std::isfinite(L)) throw std::invalid_argument("Non-finite tetrahedral multiplier");
		for (const auto& P : State.Corrections) if (!Finite(P)) throw std::invalid_argument("Non-finite tetrahedral correction state");
		if (!(Mass > 0)) return false;
		bool Changed = false; auto F = Rest.Evaluate(Current); const double J = F.Determinant();
		if (J < Settings.MinimumJacobian)
		{
			const auto Cofactor = F.Cofactor(); std::array<DVector, 4> Delta; double Weight = 0;
			for (int I = 0; I < 4; ++I) { Delta[I] = Cofactor * Rest.Gradients[I]; Weight += InverseMasses[I] * Delta[I].SquareLength(); }
			if (Weight > 0)
			{
				for (int I = 0; I < 4; ++I) Delta[I] *= (Settings.MinimumJacobian - J) * InverseMasses[I] / Weight;
				double Step; Changed = Apply(Mutable, Delta, Step, Rest, Settings);
				for (int I = 0; I < 4; ++I) Current[I] = Working[I]; F = Rest.Evaluate(Current);
			}
		}
		TetrahedralMaterialResponse R;
		if (Channels == TetrahedralSolveChannels::Volume && Material.Model == TetrahedralMaterialModel::Corotated)
		{
			R.Stiffness[1] = Material.LameLambda(); R.Constraints[1] = F.Determinant() - 1; R.Gradients[1] = F.Cofactor();
		}
		else R = Evaluate(F, Material);
		std::array<std::array<DVector, 4>, 2> Gradients;
		double G[2][2] = {}, Compliance[2], Values[2]; const double H2 = (double)Dt * Dt;
		for (int Mode = 0; Mode < 2; ++Mode)
		{
			const bool Active = R.Stiffness[Mode] > 0 && (Mode == 1 || Channels == TetrahedralSolveChannels::All); Compliance[Mode] = Active ? 1 / (R.Stiffness[Mode] * Rest.Volume) : 1;
			if (!Positive(Compliance[Mode])) throw std::overflow_error("Tetrahedral compliance is not representable");
			Values[Mode] = Active ? R.Constraints[Mode] : 0;
			for (int I = 0; I < 4; ++I) Gradients[Mode][I] = Active ? R.Gradients[Mode] * Rest.Gradients[I] : DVector(0);
		}
		for (int A = 0; A < 2; ++A) for (int B = 0; B < 2; ++B)
			for (int I = 0; I < 4; ++I) G[A][B] += InverseMasses[I] * Gradients[A][I].Dot(Gradients[B][I]);
		std::array<DVector, 4> Residual; double Drift[2] = {};
		for (int I = 0; I < 4; ++I)
		{
			Residual[I] = (Gradients[0][I] * State.Multipliers[0] + Gradients[1][I] * State.Multipliers[1]) * InverseMasses[I] - State.Corrections[I];
			for (int Mode = 0; Mode < 2; ++Mode) Drift[Mode] += Gradients[Mode][I].Dot(Residual[I]);
		}
		const double Scale = std::max(H2 * G[0][0] + Compliance[0], H2 * G[1][1] + Compliance[1]);
		if (!Positive(Scale)) throw std::overflow_error("Tetrahedral response overflow");
		const double A = H2 * G[0][0] / Scale, B = H2 * G[0][1] / Scale, C = H2 * G[1][1] / Scale;
		const double U = Compliance[0] / Scale, V = Compliance[1] / Scale;
		// Keep positive compliance terms separate from the Gram determinant. The
		// two rest gradients can be parallel even when the compliant block is SPD.
		const double Determinant = std::max(0.0, A * C - B * B) + A * V + C * U + U * V;
		const double RHS0 = -((Values[0] + Drift[0]) * H2 / Scale + U * State.Multipliers[0]), RHS1 = -((Values[1] + Drift[1]) * H2 / Scale + V * State.Multipliers[1]);
		const double L0 = ((C + V) * RHS0 - B * RHS1) / Determinant, L1 = ((A + U) * RHS1 - B * RHS0) / Determinant;
		if (!Positive(Determinant) || !std::isfinite(L0) || !std::isfinite(L1)) throw std::overflow_error("Singular tetrahedral material response");
		std::array<DVector, 4> Delta; Vector3 Before[4];
		for (int I = 0; I < 4; ++I) { Delta[I] = Residual[I] + (Gradients[0][I] * L0 + Gradients[1][I] * L1) * InverseMasses[I]; Before[I] = Working[I]; }
		double Step; Changed = Apply(Mutable, Delta, Step, Rest, Settings) || Changed;
		auto Next = State; Next.Multipliers[0] += L0 * Step; Next.Multipliers[1] += L1 * Step;
		for (int I = 0; I < 4; ++I) Next.Corrections[I] += Double(Working[I]) - Double(Before[I]);
		for (double L : Next.Multipliers) if (!std::isfinite(L)) throw std::overflow_error("Tetrahedral multiplier overflow");
		for (int I = 0; I < 4; ++I) *Positions[I] = Working[I]; State = Next; return Changed;
	}
}
