#include "Test.h"
#include "../Src/Deformation/DeformableBody.h"
#include "../Src/RigidBodyDynamics/PhysicsWorld.h"
#include "../Viewer/SceneWorld.h"
#include <algorithm>
#include <limits>
#include <stdexcept>

namespace
{
	using namespace Riemann;
	using DVector = DeformationGradient::DVector;
	DVector Double(const Vector3& P) { return DVector(P.x, P.y, P.z); }
	double Length(const DVector& P) { return std::hypot(std::hypot(P.x, P.y), P.z); }
	template<class F> void Reject(F Function) { bool Threw = false; try { Function(); } catch (const std::exception&) { Threw = true; } EXPECT(Threw); }
	DeformationGradient Identity() { DeformationGradient F; for (int I = 0; I < 3; ++I) F[I][I] = 1; return F; }
	void UnitTet(DeformableBody& Body)
	{
		for (const auto& P : { Vector3::Zero(), Vector3::UnitX(), Vector3::UnitY(), Vector3::UnitZ() }) Body.AddParticle(P);
		EXPECT(Body.AddTetrahedron(0, 1, 2, 3) == 0);
	}
	void StressAndCalibration()
	{
		for (auto Model : { TetrahedralMaterialModel::StableNeoHookean, TetrahedralMaterialModel::SaintVenantKirchhoff })
		{
			TetrahedralMaterial M; M.Model = Model; auto F = Identity(); auto R = TetrahedralElasticity::Evaluate(F, M);
			EXPECT_NEAR(R.EnergyDensity, 0, 1e-10); for (int I = 0; I < 3; ++I) for (int J = 0; J < 3; ++J) EXPECT_NEAR(R.FirstPiola[I][J], 0, 1e-10);
			F[0][0] = 1.12; F[1][1] = 0.93; F[2][2] = 1.04; F[0][1] = 0.1; F[2][0] = -0.07;
			R = TetrahedralElasticity::Evaluate(F, M); constexpr double H = 1e-6;
			for (int I = 0; I < 3; ++I) for (int J = 0; J < 3; ++J)
			{
				auto Plus = F, Minus = F; Plus[I][J] += H; Minus[I][J] -= H;
				const double Derivative = (TetrahedralElasticity::Evaluate(Plus, M).EnergyDensity - TetrahedralElasticity::Evaluate(Minus, M).EnergyDensity) / (2 * H);
				EXPECT_NEAR(Derivative, R.FirstPiola[I][J], 2e-5 * std::max(1.0, std::abs(Derivative)));
			}
			for (double Nu : { 0.0, 0.25, 0.49 })
			{
				M.PoissonRatio = Nu; auto Plus = Identity(), Minus = Identity(); Plus[0][0] += H; Minus[0][0] -= H;
				const auto P = TetrahedralElasticity::Evaluate(Plus, M), N = TetrahedralElasticity::Evaluate(Minus, M);
				EXPECT_NEAR((P.FirstPiola[0][0] - N.FirstPiola[0][0]) / (2 * H), M.LameLambda() + 2 * M.ShearModulus(), 1e-5 * M.YoungModulus);
				EXPECT_NEAR((P.FirstPiola[1][1] - N.FirstPiola[1][1]) / (2 * H), M.LameLambda(), 1e-5 * M.YoungModulus);
			}
			M.PoissonRatio = 0.3; const double C = std::cos(0.7), S = std::sin(0.7); auto Rotated = F;
			for (int J = 0; J < 3; ++J) { Rotated[0][J] = C * F[0][J] - S * F[1][J]; Rotated[1][J] = S * F[0][J] + C * F[1][J]; }
			const auto Q = TetrahedralElasticity::Evaluate(Rotated, M); EXPECT_NEAR(Q.EnergyDensity, R.EnergyDensity, 1e-8);
			for (int J = 0; J < 3; ++J)
			{
				EXPECT_NEAR(Q.FirstPiola[0][J], C * R.FirstPiola[0][J] - S * R.FirstPiola[1][J], 1e-8);
				EXPECT_NEAR(Q.FirstPiola[1][J], S * R.FirstPiola[0][J] + C * R.FirstPiola[1][J], 1e-8);
			}
		}
	}
	void RestShapesAndKernel()
	{
		for (auto Model : { TetrahedralMaterialModel::StableNeoHookean, TetrahedralMaterialModel::SaintVenantKirchhoff })
			for (float Scale : { 0.001f, 1.0f, 1000.0f }) for (bool Reverse : { false, true })
		{
			Vector3 Rest[] = { Vector3(0), Vector3(Scale, 0, 0), Vector3(0, Scale, 0), Vector3(0, 0, Scale) };
			if (Reverse) std::swap(Rest[1], Rest[2]); const auto Shape = TetrahedralRestShape::Create(Rest);
			const auto F = Shape.Evaluate(Rest); for (int I = 0; I < 3; ++I) for (int J = 0; J < 3; ++J) EXPECT_NEAR(F[I][J], I == J ? 1 : 0, 1e-12);
			Vector3 P[4]; Vector3* Positions[4]; float Masses[] = { 1, 0.5f, 0.25f, 2 }; double TotalMass = 0;
			for (float& W : Masses) { W = (float)(W / ((double)Scale * Scale * Scale)); TotalMass += 1 / (double)W; }
			for (int I = 0; I < 4; ++I) { P[I] = Rest[I]; Positions[I] = &P[I]; }
			TetrahedralMaterial M; M.Model = Model; TetrahedralMaterialState State;
			for (int I = 0; I < 16; ++I) TetrahedralElasticity::Solve(Positions, Masses, Shape, M, State, 0.01f * Scale);
			for (int I = 0; I < 4; ++I) EXPECT_NEAR((P[I] - Rest[I]).Length() / Scale, 0, 1e-6);
			P[1] += Vector3(0.05f, 0.01f, 0) * Scale; State = TetrahedralMaterialState();
			Vector3 Predicted[4]; for (int I = 0; I < 4; ++I) Predicted[I] = P[I];
			for (int Iteration = 0; Iteration < 64; ++Iteration)
			{
				DVector Center(0); for (int I = 0; I < 4; ++I) Center += Double(P[I]) / Masses[I];
				TetrahedralElasticity::Solve(Positions, Masses, Shape, M, State, 0.01f * Scale);
				DVector Next(0), Force(0), Torque(0); const auto R = TetrahedralElasticity::Evaluate(Shape.Evaluate(P), M);
				for (int I = 0; I < 4; ++I)
				{
					Next += Double(P[I]) / Masses[I]; const auto NodalForce = (R.FirstPiola * Shape.Gradients[I]) * Shape.Volume;
					Force += NodalForce; Torque += Double(P[I]).Cross(NodalForce);
				}
				EXPECT(Length(Next - Center) / (Scale * TotalMass) < 2e-6);
				EXPECT(Length(Force) / (M.YoungModulus * Shape.Volume / Scale) < 1e-10);
				EXPECT(Length(Torque) / (M.YoungModulus * Shape.Volume) < 1e-10);
				EXPECT(Shape.Evaluate(P).Determinant() >= 0.05);
			}
			// Newton iterates are not independent physical impulses. Verify the final
			// inertia/elastic force balance and angular change about the predicted pose.
			DVector Angular(0); const auto Response = TetrahedralElasticity::Evaluate(Shape.Evaluate(P), M);
			for (int I = 0; I < 4; ++I)
			{
				const auto Correction = Double(P[I]) - Double(Predicted[I]); Angular += Double(Predicted[I]).Cross(Correction) / Masses[I];
				const auto Balance = Correction / (Masses[I] * (double)(0.01f * Scale) * (0.01f * Scale)) + (Response.FirstPiola * Shape.Gradients[I]) * Shape.Volume;
				EXPECT(Length(Balance) / (M.YoungModulus * Shape.Volume / Scale) < 2e-5);
			}
			EXPECT(Length(Angular) / (Scale * Scale * TotalMass) < 2e-6);
		}
	}
	DeformableBodySettings Settings()
	{
		DeformableBodySettings S; S.Gravity = Vector3::Zero(); S.GroundEnabled = false; S.Damping = 0; S.Substeps = 1; S.SolverIterations = 32; S.EdgeCompliance = 0; return S;
	}
	void MaterialAssignment()
	{
		DeformableBody B(Settings()); UnitTet(B); TetrahedralMaterial M; M.Density = 6;
		const int A = B.AddMaterial(M); B.SetTetrahedronMaterial(0, A);
		for (const auto& E : B.GetEdges()) EXPECT(!E.Enabled);
		B.AddParticle(Vector3(0, 0, -1)); EXPECT(B.AddTetrahedron(0, 1, 2, 4) == 1);
		for (const auto& E : B.GetEdges()) if (E.ParticleA != 3 && E.ParticleB != 3) EXPECT(E.Enabled);
		M.Density = 12; const int C = B.AddMaterial(M); B.SetTetrahedronMaterial(1, C);
		for (const auto& E : B.GetEdges()) EXPECT(!E.Enabled);
		B.AddEdge(0, 1); EXPECT(B.GetEdges()[0].Explicit && B.GetEdges()[0].Enabled);
		B.PinParticle(3, B.GetParticle(3).Position); B.ApplyMaterialDensities();
		EXPECT_NEAR(B.GetParticle(0).InverseMass, 4.0 / 3, 1e-6); EXPECT_NEAR(B.GetParticle(3).InverseMass, 4, 1e-6);
		EXPECT_NEAR(B.GetParticle(4).InverseMass, 2, 1e-6); EXPECT(B.GetParticle(3).Pinned);
		EXPECT_NEAR(B.GetMaterialEnergy(), 0, 1e-10); B.SetTetrahedronMaterial(0, -1);
		for (int I : B.GetTetrahedra()[0].Edges) EXPECT(B.GetEdges()[I].Enabled);
		Reject([&] { B.SetTetrahedronMaterial(99, A); }); Reject([&] { B.SetTetrahedronMaterial(0, 99); });
		M.PoissonRatio = 0.5; Reject([&] { B.SetMaterial(A, M); }); EXPECT(B.GetMaterials()[A].PoissonRatio == 0.3);
		Reject([&] { B.GetTetrahedronMaterialResponse(0); });
	}
	void StaticCalibration()
	{
		for (auto Model : { TetrahedralMaterialModel::StableNeoHookean, TetrahedralMaterialModel::SaintVenantKirchhoff })
			for (double Young : { 1000.0, 10000.0 }) for (int Substeps : { 1, 2 })
		{
			auto S = Settings(); S.Substeps = Substeps; S.SolverIterations = 64; S.Damping = 12;
			DeformableBody B(S); UnitTet(B); B.PinParticle(0, Vector3::Zero()); TetrahedralMaterial M;
			M.Model = Model; M.YoungModulus = Young; M.PoissonRatio = 0.25; M.Density = 6;
			B.SetTetrahedronMaterial(0, B.AddMaterial(M)); B.ApplyMaterialDensities();
			constexpr double Stress = 1; for (int I = 0; I < 300; ++I) { B.AddForce(2, Vector3(0, (float)(Stress / 6), 0)); B.Simulate(0.01f); }
			const double Longitudinal = B.GetParticle(2).Position.y - 1, Transverse = (B.GetParticle(1).Position.x + B.GetParticle(3).Position.z) / 2 - 1;
			printf("FEM calibration model=%d E=%g substeps=%d strain=%g expected=%g lateral=%g expected=%g\n", (int)Model, Young, Substeps, Longitudinal, Stress / Young, Transverse, -M.PoissonRatio * Stress / Young);
			const auto Response = B.GetTetrahedronMaterialResponse(0);
			printf("FEM stress: xx=%g yy=%g velocityY=%g\n", Response.FirstPiola[0][0], Response.FirstPiola[1][1], B.GetParticle(2).Velocity.y);
			EXPECT_NEAR(Longitudinal, Stress / Young, Stress / Young * 0.02);
			EXPECT_NEAR(Transverse, -M.PoissonRatio * Stress / Young, M.PoissonRatio * Stress / Young * 0.02);
			EXPECT(B.GetMaterialEnergy() > 0); B.Reset(); EXPECT_NEAR(B.GetMaterialEnergy(), 0, 1e-10);
		}
	}
	void ValidationAndRecovery()
	{
		Vector3 R[] = { Vector3(0), Vector3::UnitX(), Vector3::UnitY(), Vector3::UnitZ() }; const auto Shape = TetrahedralRestShape::Create(R);
		Vector3 P[4]; Vector3* Positions[4]; float Masses[] = { 1, 1, 1, 1 };
		for (int I = 0; I < 4; ++I) { P[I] = R[I]; Positions[I] = &P[I]; }
		TetrahedralMaterial M; TetrahedralMaterialState State;
		P[3].z = -0.1f;
		for (int I = 0; I < 64; ++I) TetrahedralElasticity::Solve(Positions, Masses, Shape, M, State, 0.01f);
		EXPECT(Shape.Evaluate(P).Determinant() >= 0.05 - 1e-5);
		P[1].x = std::numeric_limits<float>::quiet_NaN(); const auto Before = P[0];
		Reject([&] { TetrahedralElasticity::Solve(Positions, Masses, Shape, M, State, 0.01f); }); EXPECT(P[0] == Before);
		R[3] = R[2]; Reject([&] { TetrahedralRestShape::Create(R); });
		M.PoissonRatio = -0.1; EXPECT(!M.IsValid());
	}
	void ExistingDemo()
	{
		SceneWorld Scene; Scene.SetClothModelDemo(true); std::string Error;
		EXPECT(Scene.LoadFromFile(TestContentsPath() + "Scenes/08_cloth_sphere.json", &Error));
		for (bool Rigid : { false, true }) for (bool StVK : { false, true })
		{
			EXPECT(Scene.ApplyDeformableContactDemo(false));
			EXPECT(Scene.GetSimulation()->SetSolverConfiguration(Rigid ? ResolutionPhaseSolver::XPBD : ResolutionPhaseSolver::SequentialImpulse, SolverBackend::CPU, false));
			EXPECT(Scene.ApplyDeformableContactDemo(true, Rigid)); EXPECT(Scene.ApplyDeformableMaterialDemo(true, StVK));
			EXPECT(Scene.GetCalibratedSoftMaterialDemo()); EXPECT(Scene.GetStVKSoftMaterialDemo() == StVK);
			size_t Contacts = 0;
			for (size_t I = 0; I < 2; ++I)
			{
				auto* B = Scene.GetDemoContactBody(I); double Mass = 0;
				for (const auto& P : B->GetParticles()) Mass += 1 / P.InverseMass;
				EXPECT_NEAR(Mass, 4, 1e-6); for (const auto& E : B->GetEdges()) EXPECT(!E.Enabled);
			}
			for (int Frame = 0; Frame < 120; ++Frame)
			{
				Scene.Step(1.0f / 60);
				for (size_t I = 0; I < 2; ++I)
				{
					auto* B = Scene.GetDemoContactBody(I); const auto Response = B->GetTetrahedronMaterialResponse(0);
					EXPECT(std::isfinite(Response.EnergyDensity) && Response.Jacobian > 0);
					for (const auto& P : B->GetParticles()) EXPECT(std::isfinite(P.Position.SquareLength()) && P.Position.Length() < 100);
					Contacts += B->GetCollisionStats().InterContacts + B->GetCollisionStats().RigidContacts;
				}
			}
			printf("FEM existing demo StVK=%d rigid=%d contacts=%zu energies=%g/%g\n", StVK, Rigid, Contacts,
				Scene.GetDemoContactBody(0)->GetMaterialEnergy(), Scene.GetDemoContactBody(1)->GetMaterialEnergy());
			EXPECT(Contacts > 0); if (Rigid) EXPECT(Scene.GetDemoContactRigid()->X.y > 1.7f);
			EXPECT(Scene.ApplyDeformableMaterialDemo(false)); EXPECT(!Scene.GetCalibratedSoftMaterialDemo());
			for (size_t I = 0; I < 2; ++I) for (const auto& E : Scene.GetDemoContactBody(I)->GetEdges()) EXPECT(E.Enabled);
			Scene.Step(1.0f / 60);
		}
		EXPECT(Scene.ApplyDeformableContactDemo(false)); EXPECT(!Scene.GetCalibratedSoftMaterialDemo());
	}
}
void TestTetrahedralMaterial()
{
	StressAndCalibration(); RestShapesAndKernel(); MaterialAssignment(); StaticCalibration(); ValidationAndRecovery(); ExistingDemo();
}
