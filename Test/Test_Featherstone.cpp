#include "Test.h"
#include "../Src/RigidBodyDynamics/FeatherstoneSolver.h"
#include <algorithm>
#include <chrono>
#include <limits>
#include <stdexcept>

namespace
{
	using namespace Riemann;
	const Vector3 Gravity(0, -9.8f, 0);
	void Near(double Actual, double Expected, double Tolerance = 1e-6)
	{ EXPECT_NEAR(Actual, Expected, Tolerance * std::max(1.0, std::abs(Expected))); }
	void VectorNear(const std::vector<double>& A, const std::vector<double>& B, double Tolerance = 1e-6)
	{ EXPECT(A.size() == B.size()); for (size_t I = 0; I < std::min(A.size(), B.size()); ++I) Near(A[I], B[I], Tolerance); }
	void Step(FeatherstoneSolver& Solver, double Dt, int Count, const Vector3& G = Gravity)
	{
		const std::vector<double> Torque(Solver.GetDofCount());
		for (int I = 0; I < Count; ++I) { Solver.IntegrateVelocity(Solver.ComputeAcceleration(Torque, G), Dt); Solver.IntegrateConfiguration(Dt); }
	}
	void AnalyticJoints()
	{
		RagdollLinkSettings L; L.Joint = RagdollReducedJointType::Revolute; L.Mass = 2;
		L.CenterOfMass = Vector3(1, 0, 0); L.Inertia = Matrix3(0.2f, 0.3f, 0.4f);
		FeatherstoneSolver Pendulum({ L });
		Near(Pendulum.ComputeAcceleration({ 0 }, Gravity)[0], -19.6 / 2.4);
		auto Q = Pendulum.GetConfiguration(); Q[0].Position = 0.7; Pendulum.SetConfiguration(Q);
		Near(Pendulum.ComputeAcceleration({ 1.2 }, Gravity)[0], (1.2 - 19.6 * std::cos(0.7)) / 2.4);
		const auto Holding = Pendulum.InverseDynamics({ 0 }, Gravity);
		Near(Holding[0], 19.6 * std::cos(0.7)); Near(Pendulum.ComputeAcceleration(Holding, Gravity)[0], 0);
		L.Joint = RagdollReducedJointType::Prismatic; L.Axis = Vector3::UnitY(); L.Mass = 3; L.Armature = 2; L.Damping = 4;
		FeatherstoneSolver Slider({ L }); Slider.SetVelocity({ 0.5 });
		Near(Slider.ComputeAcceleration({ 10 }, Gravity)[0], (10 - 3 * 9.8 - 4 * 0.5) / 5);
		Near(Slider.ComputeImpulseResponse({}, { 10 })[0], 2);
		L.Joint = RagdollReducedJointType::Spherical; L.CenterOfMass = Vector3::Zero(); L.Inertia = Matrix3(2, 3, 4); L.Armature = L.Damping = 0;
		FeatherstoneSolver Ball({ L }); Ball.SetVelocity({ 1, 2, 3 });
		VectorNear(Ball.ComputeAcceleration({ 0, 0, 0 }, Gravity), { -3, 2, -0.5 });
		L.Joint = RagdollReducedJointType::Fixed; FeatherstoneSolver Fixed({ L });
		EXPECT(Fixed.ComputeAcceleration({}, Gravity).empty()); EXPECT(Fixed.ComputeImpulseResponse({}).empty());
	}
	void DoublePendulum()
	{
		constexpr double M1 = 2, M2 = 3, A1 = 0.4, A2 = 0.6, Length = 1.1, I1 = 0.3, I2 = 0.5;
		std::vector<RagdollLinkSettings> Links(2);
		Links[0].Joint = Links[1].Joint = RagdollReducedJointType::Revolute;
		Links[0].Mass = (float)M1; Links[1].Mass = (float)M2; Links[1].Parent = 0;
		Links[0].CenterOfMass = Vector3((float)A1, 0, 0); Links[1].CenterOfMass = Vector3((float)A2, 0, 0);
		Links[0].Inertia = Matrix3(0.2f, 0.2f, (float)I1); Links[1].Inertia = Matrix3(0.2f, 0.2f, (float)I2);
		Links[1].ParentToJoint.pos = Vector3((float)Length, 0, 0);
		FeatherstoneSolver Solver(Links);
		for (double Q1 : { -1.0, 0.0, 0.7 }) for (double Q2 : { -0.8, 0.1, 1.4 })
		{
			auto Q = Solver.GetConfiguration(); Q[0].Position = Q1; Q[1].Position = Q2; Solver.SetConfiguration(Q);
			const double V1 = 0.8, V2 = -0.4; Solver.SetVelocity({ V1, V2 });
			const double H11 = I1 + I2 + M1 * A1 * A1 + M2 * (Length * Length + A2 * A2 + 2 * Length * A2 * std::cos(Q2));
			const double H12 = I2 + M2 * (A2 * A2 + Length * A2 * std::cos(Q2)), H22 = I2 + M2 * A2 * A2;
			const double C1 = -M2 * Length * A2 * std::sin(Q2) * (2 * V1 * V2 + V2 * V2);
			const double C2 = M2 * Length * A2 * std::sin(Q2) * V1 * V1;
			const double G1 = 9.8 * ((M1 * A1 + M2 * Length) * std::cos(Q1) + M2 * A2 * std::cos(Q1 + Q2));
			const double G2 = 9.8 * M2 * A2 * std::cos(Q1 + Q2);
			const double F1 = 1.3 - C1 - G1, F2 = -0.7 - C2 - G2, Det = H11 * H22 - H12 * H12;
			VectorNear(Solver.ComputeAcceleration({ 1.3, -0.7 }, Gravity), { (H22 * F1 - H12 * F2) / Det, (H11 * F2 - H12 * F1) / Det }, 4e-6);
		}
	}
	std::vector<RagdollLinkSettings> Tree(int Count, bool Floating, bool Chain = false)
	{
		std::vector<RagdollLinkSettings> Links(Count);
		for (int I = 0; I < Count; ++I)
		{
			auto& L = Links[I]; L.Parent = I == 0 ? -1 : Chain ? I - 1 : (I - 1) / 2;
			L.Joint = I == 0 ? (Floating ? RagdollReducedJointType::Floating : RagdollReducedJointType::Fixed)
				: (RagdollReducedJointType)(I % 4);
			L.ParentToJoint.pos = Vector3(0.4f, -0.1f, 0.2f);
			L.ParentToJoint.quat.FromRotationAxis(Vector3(1, 2, 3).Unit(), I * 0.07f);
			L.Axis = Vector3(1, (float)(I % 3), 0.5f).Unit();
			L.CenterOfMass = Vector3(0.2f, -0.1f, 0.05f); L.Mass = (float)(1 + I % 5);
			L.Inertia = Matrix3(0.3f * L.Mass, 0.5f * L.Mass, 0.6f * L.Mass);
			L.Armature = 0.02; L.Damping = 0.03;
		}
		return Links;
	}
	void SetVariedState(FeatherstoneSolver& Solver)
	{
		auto Q = Solver.GetConfiguration();
		for (size_t I = 0; I < Q.size(); ++I)
		{
			Q[I].Position = 0.4 * std::sin((double)I); Q[I].Rotation.FromRotationAxis(Vector3(2, 1, 3).Unit(), (float)(0.1 * I));
			Q[I].Translation = Vector3(1, 2, -3);
		}
		Solver.SetConfiguration(Q);
		auto V = Solver.GetVelocity(); for (size_t I = 0; I < V.size(); ++I) V[I] = 0.7 * std::cos((double)I);
		Solver.SetVelocity(V);
	}
	void InverseAndImpulse()
	{
		for (bool Floating : { false, true }) for (int Count : { 2, 16, 64 })
		{
			FeatherstoneSolver Solver(Tree(Count, Floating)); SetVariedState(Solver);
			std::vector<double> Target(Solver.GetDofCount()); for (size_t I = 0; I < Target.size(); ++I) Target[I] = std::sin(I * 0.3);
			std::vector<SpatialVector> Forces(Count);
			for (int I = 0; I < Count; ++I) Forces[I] = SpatialVector(Vector3(0.1f, 0.2f, -0.3f), Vector3(2, -1, 3));
			const auto Torque = Solver.InverseDynamics(Target, Gravity, Forces);
			const auto Actual = Solver.ComputeAcceleration(Torque, Gravity, Forces);
			VectorNear(Actual, Target, 2e-7);
			const size_t Factorizations = Solver.GetFactorizationCount();
			const auto Response = Solver.ComputeImpulseResponse(Forces, Target);
			const auto OldV = Solver.GetVelocity(); Solver.SetVelocity(std::vector<double>(Solver.GetDofCount()));
			const auto ForceFromResponse = Solver.InverseDynamics(Response, Vector3::Zero(), Forces);
			VectorNear(ForceFromResponse, Target, 1e-6);
			Solver.SetVelocity(OldV); Solver.ComputeAcceleration(Torque, Gravity, Forces);
			EXPECT(Solver.GetFactorizationCount() == Factorizations);
			auto Q = Solver.GetConfiguration(); Q.back().Position += 0.1; Solver.SetConfiguration(Q);
			Solver.ComputeImpulseResponse(Forces); EXPECT(Solver.GetFactorizationCount() == Factorizations + 1);
			printf("Ragdoll ABA/RNEA tree links=%d floating=%d dofs=%d\n", Count, Floating, Solver.GetDofCount());
		}
	}
	void KinematicDerivativeAndMass()
	{
		FeatherstoneSolver Solver(Tree(9, true)); SetVariedState(Solver);
		const auto Q = Solver.GetConfiguration(); const auto V = Solver.GetVelocity();
		std::vector<Vector3> Position; std::vector<SpatialVector> Velocity;
		for (size_t I = 0; I < Solver.GetLinkCount(); ++I) { Position.push_back(Solver.GetLinkTransform(I).pos); Velocity.push_back(Solver.GetLinkVelocity(I)); }
		constexpr double StepSize = 0.0005; Solver.IntegrateConfiguration(StepSize);
		for (size_t I = 0; I < Position.size(); ++I)
		{
			const Vector3 Difference = (Solver.GetLinkTransform(I).pos - Position[I]) / (float)StepSize;
			EXPECT((Difference - Velocity[I].GetLinear()).Length() < 0.006f);
		}
		Solver.SetConfiguration(Q);
		const int N = Solver.GetDofCount(); std::vector<std::vector<SpatialVector>> Columns(N);
		for (int J = 0; J < N; ++J)
		{
			std::vector<double> Unit(N); Unit[J] = 1; Solver.SetVelocity(Unit);
			for (size_t I = 0; I < Solver.GetLinkCount(); ++I) Columns[J].push_back(Solver.GetLinkVelocity(I));
		}
		std::vector<double> H(N * N);
		for (int R = 0; R < N; ++R) for (int C = 0; C < N; ++C)
			for (size_t I = 0; I < Solver.GetLinkCount(); ++I)
			{
				const auto& L = Solver.GetLinkSettings(I); const auto Pose = Solver.GetLinkTransform(I);
				const Vector3 Offset = Pose.quat * L.CenterOfMass;
				const Vector3 WA = Columns[R][I].GetAngular(), WB = Columns[C][I].GetAngular();
				const Vector3 VA = Columns[R][I].GetLinear() + WA.Cross(Offset), VB = Columns[C][I].GetLinear() + WB.Cross(Offset);
				const auto Rot = Pose.quat.ToRotationMatrix3();
				H[R * N + C] += L.Mass * VA.Dot(VB) + WA.Dot(Rot * L.Inertia * Rot.Transpose() * WB);
				if (R == C && R >= Solver.GetLinkDofOffset(I) && R < Solver.GetLinkDofOffset(I) + Solver.GetLinkDofCount(I)) H[R * N + C] += L.Armature;
			}
		Solver.SetVelocity(V);
		std::vector<double> Torque(N); for (int I = 0; I < N; ++I) Torque[I] = std::cos(I * 0.7);
		const auto Response = Solver.ComputeImpulseResponse({}, Torque);
		for (int R = 0; R < N; ++R)
		{
			double Force = 0; for (int C = 0; C < N; ++C) Force += H[R * N + C] * Response[C];
			Near(Force, Torque[R], 3e-5);
		}
	}
	void FreeBodyAndMomentum()
	{
		RagdollLinkSettings L; L.Joint = RagdollReducedJointType::Floating; L.Mass = 2; L.Inertia = Matrix3(2, 3, 4);
		FeatherstoneSolver Body({ L }); Body.SetVelocity({ 1, 2, 3, 4, 5, 6 });
		VectorNear(Body.ComputeAcceleration(std::vector<double>(6), Gravity), { -3, 2, -0.5, 3, -15.8, 3 });
		L.CenterOfMass = Vector3(0.4f, -0.2f, 0.3f); FeatherstoneSolver Offset({ L }); Offset.SetVelocity(Body.GetVelocity());
		const auto Acceleration = Offset.ComputeAcceleration(std::vector<double>(6), Gravity);
		const Vector3 Alpha((float)Acceleration[0], (float)Acceleration[1], (float)Acceleration[2]);
		const Vector3 Linear((float)Acceleration[3], (float)Acceleration[4], (float)Acceleration[5]);
		const Vector3 Omega(1, 2, 3), V(4, 5, 6);
		const Vector3 COMAcceleration = Linear + Omega.Cross(V) + Alpha.Cross(L.CenterOfMass) + Omega.Cross(Omega.Cross(L.CenterOfMass));
		EXPECT((COMAcceleration - Gravity).Length() < 1e-5f);
		auto Links = Tree(8, true); for (auto& Link : Links) Link.Armature = Link.Damping = 0;
		FeatherstoneSolver TreeSolver(Links); SetVariedState(TreeSolver);
		const auto Before = TreeSolver.GetMomentum(); std::vector<SpatialVector> Impulses(Links.size());
		const Vector3 Force(1.2f, -0.7f, 0.2f); Impulses.back() = SpatialVector(Vector3(0.1f, 0.3f, -0.2f), Force);
		const auto Response = TreeSolver.ComputeImpulseResponse(Impulses); auto Vel = TreeSolver.GetVelocity();
		for (size_t I = 0; I < Vel.size(); ++I) Vel[I] += Response[I]; TreeSolver.SetVelocity(Vel);
		const auto Delta = TreeSolver.GetMomentum() - Before;
		const Vector3 ExpectedTorque = Impulses.back().GetAngular() + TreeSolver.GetLinkTransform(Links.size() - 1).pos.Cross(Force);
		EXPECT((Delta.GetLinear() - Force).Length() < 2e-5f); EXPECT((Delta.GetAngular() - ExpectedTorque).Length() < 4e-5f);
		const auto Momentum = TreeSolver.GetMomentum(); const double Energy = TreeSolver.GetKineticEnergy();
		Step(TreeSolver, 0.0001, 1000, Vector3::Zero());
		EXPECT((TreeSolver.GetMomentum().GetLinear() - Momentum.GetLinear()).Length() < 0.01f);
		EXPECT((TreeSolver.GetMomentum().GetAngular() - Momentum.GetAngular()).Length() < 0.02f);
		Near(TreeSolver.GetKineticEnergy(), Energy, 0.002);
	}
	void LongChainAndValidation()
	{
		for (int Count : { 32, 128, 512 })
		{
			auto Links = Tree(Count, false, true);
			for (int I = 1; I < Count; ++I)
			{
				Links[I].Joint = RagdollReducedJointType::Revolute;
				Links[I].Mass = I % 2 ? 1000.0f : 1.0f;
				Links[I].Inertia = Matrix3(Vector3(Links[I].Mass * 0.2f)); Links[I].Damping = 0;
			}
			FeatherstoneSolver Solver(Links); SetVariedState(Solver);
			std::vector<double> Expected(Solver.GetDofCount(), 0.1);
			const auto Torque = Solver.InverseDynamics(Expected, Gravity);
			const auto Start = std::chrono::steady_clock::now(); const auto Acceleration = Solver.ComputeAcceleration(Torque, Gravity);
			const double Milliseconds = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - Start).count();
			VectorNear(Acceleration, Expected, 0.003);
			printf("Ragdoll chain links=%d ABA=%.3f ms\n", Count, Milliseconds);
		}
		auto Valid = Tree(4, true);
		auto Reject = [](const std::vector<RagdollLinkSettings>& Links)
		{ bool Failed = false; try { FeatherstoneSolver Bad(Links); } catch (const std::invalid_argument&) { Failed = true; } EXPECT(Failed); };
		Reject({}); auto Bad = Valid; Bad[2].Parent = 3; Reject(Bad); Bad = Valid; Bad[2].Parent = -1; Reject(Bad);
		Bad = Valid; Bad[1].Joint = RagdollReducedJointType::Floating; Reject(Bad);
		Bad = Valid; Bad[1].Mass = 0; Reject(Bad); Bad = Valid; Bad[1].Inertia[2][2] = -1; Reject(Bad);
		Bad = Valid; Bad[1].Inertia[0][1] = 1; Reject(Bad); Bad = Valid; Bad[1].Axis = Vector3::Zero(); Reject(Bad);
		Bad = Valid; Bad[1].Armature = -1; Reject(Bad);
		FeatherstoneSolver Solver(Valid); const auto Before = Solver.GetConfiguration(); auto Q = Before;
		Q[1].Position = std::numeric_limits<double>::infinity(); bool Failed = false;
		try { Solver.SetConfiguration(Q); } catch (const std::invalid_argument&) { Failed = true; } EXPECT(Failed);
		Near(Solver.GetConfiguration()[1].Position, Before[1].Position);
		Failed = false; try { Solver.SetVelocity({ 1 }); } catch (const std::invalid_argument&) { Failed = true; } EXPECT(Failed);
		Failed = false; try { Solver.IntegrateConfiguration(-1); } catch (const std::invalid_argument&) { Failed = true; } EXPECT(Failed);
		Q = Before; Q[2].Position = 1e100; Failed = false;
		try { Solver.SetConfiguration(Q); } catch (const std::invalid_argument&) { Failed = true; } EXPECT(Failed);
		Near(Solver.GetConfiguration()[2].Position, Before[2].Position);
		const auto OldV = Solver.GetVelocity(); auto Huge = OldV; Huge[0] = 1e100; Failed = false;
		try { Solver.SetVelocity(Huge); } catch (const std::invalid_argument&) { Failed = true; } EXPECT(Failed);
		VectorNear(Solver.GetVelocity(), OldV);
	}
	void CoordinatesAndIntegration()
	{
		auto Links = Tree(12, true); FeatherstoneSolver Original(Links); SetVariedState(Original);
		Transform Change(Vector3(10, -4, 7)); Change.quat.FromRotationAxis(Vector3(1, 2, -3).Unit(), 1.1f);
		Links[0].ParentToJoint = Change * Links[0].ParentToJoint;
		FeatherstoneSolver Transformed(Links); Transformed.SetConfiguration(Original.GetConfiguration()); Transformed.SetVelocity(Original.GetVelocity());
		std::vector<SpatialVector> Forces(Links.size()), Rotated(Links.size());
		for (size_t I = 0; I < Links.size(); ++I)
		{
			const Vector3 Torque(0.1f, -0.2f, 0.3f), Force(2, -1, 4);
			Forces[I] = SpatialVector(Torque, Force); Rotated[I] = SpatialVector(Change.quat * Torque, Change.quat * Force);
			EXPECT((Transformed.GetLinkTransform(I).pos - Change.LocalToWorld(Original.GetLinkTransform(I).pos)).Length() < 1e-5f);
		}
		const std::vector<double> Torque(Original.GetDofCount(), 0.3);
		VectorNear(Original.ComputeAcceleration(Torque, Gravity, Forces), Transformed.ComputeAcceleration(Torque, Change.quat * Gravity, Rotated), 1e-5);
		RagdollLinkSettings L; L.Joint = RagdollReducedJointType::Revolute; L.CenterOfMass = Vector3(1, 0, 0);
		L.Inertia = Matrix3(0.1f, 0.1f, 0.1f);
		double Error[2]{};
		for (int Resolution = 0; Resolution < 2; ++Resolution)
		{
			FeatherstoneSolver Pendulum({ L }); auto Q = Pendulum.GetConfiguration(); Q[0].Position = 0.6; Pendulum.SetConfiguration(Q);
			const double Initial = Pendulum.GetPotentialEnergy(Gravity); const int Count = Resolution == 0 ? 400 : 1600;
			for (int I = 0; I < Count; ++I)
			{
				Step(Pendulum, 2.0 / Count, 1);
				Error[Resolution] = std::max(Error[Resolution], std::abs(Pendulum.GetKineticEnergy() + Pendulum.GetPotentialEnergy(Gravity) - Initial));
			}
			printf("Ragdoll pendulum steps=%d maximum energy error=%.6f\n", Count, Error[Resolution]);
		}
		EXPECT(Error[1] < Error[0] / 2); EXPECT(Error[1] < 0.1);
	}
	void MovingLongChain()
	{
		constexpr int Count = 32;
		std::vector<RagdollLinkSettings> Links(Count);
		Links[0].ParentToJoint.pos = Vector3(0, 20, 0);
		for (int I = 1; I < Count; ++I)
		{
			auto& L = Links[I]; L.Parent = I - 1; L.Joint = RagdollReducedJointType::Revolute;
			L.ParentToJoint.pos = Vector3(0, -0.5f, 0); L.CenterOfMass = Vector3(0, -0.25f, 0);
			L.Mass = I % 2 ? 100.0f : 1.0f; L.Inertia = Matrix3(Vector3(L.Mass / 48));
		}
		FeatherstoneSolver Solver(Links); auto Q = Solver.GetConfiguration(); Q[1].Position = 0.4; Solver.SetConfiguration(Q);
		const Vector3 Start = Solver.GetCenterOfMass(Count - 1); const double Energy = Solver.GetPotentialEnergy(Gravity);
		Step(Solver, 0.001, 1000);
		EXPECT((Solver.GetCenterOfMass(Count - 1) - Start).Length() > 0.1f);
		for (int I = 1; I < Count; ++I)
		{
			const auto Pose = Solver.GetLinkTransform(I); EXPECT(std::isfinite(Pose.pos.Length())); Near(Pose.quat.SquareLength(), 1);
			Near((Pose.pos - Solver.GetLinkTransform(I - 1).pos).Length(), 0.5, 1e-5);
		}
		const double FinalEnergy = Solver.GetKineticEnergy() + Solver.GetPotentialEnergy(Gravity);
		printf("Ragdoll moving chain energy drift=%.6f%% tip displacement=%.4f\n", 100 * std::abs(FinalEnergy / Energy - 1), (Solver.GetCenterOfMass(Count - 1) - Start).Length());
		Near(FinalEnergy, Energy, 0.01);
	}
	void DynamicsQueryIdentities()
	{
		for (bool Floating : { false, true }) for (bool Chain : { false, true }) for (int Count : { 1, 4, 12, 32 })
		{
			FeatherstoneSolver Solver(Tree(Count, Floating, Chain)); SetVariedState(Solver);
			const int N = Solver.GetDofCount(); const auto Velocity = Solver.GetVelocity();
			Solver.ComputeImpulseResponse({}); const size_t Factorizations = Solver.GetFactorizationCount();
			const auto H = Solver.ComputeMassMatrix(); const auto J = Solver.ComputeDenseJacobian(false), COM = Solver.ComputeDenseJacobian();
			const auto Bias = Solver.ComputeBiasForce(Gravity);
			EXPECT(H.GetRows() == N && H.GetCols() == N); EXPECT(J.GetRows() == Count * 6 && J.GetCols() == N);
			double Energy = 0;
			for (int R = 0; R < N; ++R) for (int C = 0; C < N; ++C)
			{
				EXPECT(H[R][C] == H[C][R]); Energy += Velocity[R] * H[R][C] * Velocity[C] / 2;
			}
			Near(Energy, Solver.GetKineticEnergy(), 2e-7);
			std::vector<SpatialVector> Wrenches(Count); std::vector<double> ExpectedForce(N), Acceleration(N);
			for (int I = 0; I < Count; ++I)
			{
				const Vector3 Torque(0.2f, -0.1f, 0.3f), Force(2, 1, -3);
				Wrenches[I] = SpatialVector(Torque, Force);
				const auto Motion = Solver.GetLinkVelocity(I); const Vector3 Offset = Solver.GetLinkTransform(I).quat * Solver.GetLinkSettings(I).CenterOfMass;
				const Vector3 COMVelocity = Motion.GetLinear() + Motion.GetAngular().Cross(Offset);
				for (int Row = 0; Row < 6; ++Row)
				{
					double OriginSpeed = 0, COMSpeed = 0;
					for (int Column = 0; Column < N; ++Column)
					{
						OriginSpeed += J[I * 6 + Row][Column] * Velocity[Column]; COMSpeed += COM[I * 6 + Row][Column] * Velocity[Column];
						ExpectedForce[Column] += J[I * 6 + Row][Column] * Wrenches[I][Row];
					}
					Near(OriginSpeed, Motion[Row], 3e-6); Near(COMSpeed, Row < 3 ? Motion[Row] : COMVelocity[Row - 3], 3e-6);
				}
			}
			const auto Force = Solver.ComputeGeneralizedForce(Wrenches); VectorNear(Force, ExpectedForce, 3e-6);
			for (int I = 0; I < N; ++I) Acceleration[I] = std::cos(I * 0.3);
			const auto Torque = Solver.InverseDynamics(Acceleration, Gravity, Wrenches);
			for (int R = 0; R < N; ++R)
			{
				double Expected = Bias[R] - Force[R]; for (int C = 0; C < N; ++C) Expected += H[R][C] * Acceleration[C];
				Near(Torque[R], Expected, 2e-6);
			}
			VectorNear(Solver.ComputeAcceleration(Torque, Gravity, Wrenches), Acceleration, 2e-6);
			EXPECT(Solver.GetFactorizationCount() == Factorizations); VectorNear(Solver.GetVelocity(), Velocity);
			for (double Q : Solver.ComputeGeneralizedForce({})) Near(Q, 0);
		}
	}
	void PointJacobianDerivative()
	{
		FeatherstoneSolver Solver(Tree(10, true)); SetVariedState(Solver);
		const auto Initial = Solver.GetConfiguration(); const auto InitialVelocity = Solver.GetVelocity();
		const size_t Link = 7; const Vector3 Point(0.3f, -0.2f, 0.4f); const auto J = Solver.ComputeLinkJacobian(Link, Point);
		constexpr double Interval = 0.001;
		for (int Column = 0; Column < Solver.GetDofCount(); ++Column)
		{
			std::vector<double> Unit(Solver.GetDofCount()); Unit[Column] = 1;
			Solver.SetConfiguration(Initial); Solver.SetVelocity(Unit); Solver.IntegrateConfiguration(Interval);
			const auto Plus = Solver.GetLinkTransform(Link);
			Solver.SetConfiguration(Initial); Unit[Column] = -1; Solver.SetVelocity(Unit); Solver.IntegrateConfiguration(Interval);
			const auto Minus = Solver.GetLinkTransform(Link);
			const Vector3 Linear = (Plus.LocalToWorld(Point) - Minus.LocalToWorld(Point)) / (float)(2 * Interval);
			const Quaternion Rotation = Plus.quat * Minus.quat.Conjugate();
			const Vector3 Angular = Vector3(Rotation.x, Rotation.y, Rotation.z) * (float)((Rotation.w < 0 ? -1 : 1) / Interval);
			for (int Axis = 0; Axis < 3; ++Axis) { Near(Linear[Axis], J[Axis + 3][Column], 0.001); Near(Angular[Axis], J[Axis][Column], 0.001); }
		}
		Solver.SetConfiguration(Initial); Solver.SetVelocity(InitialVelocity);
		const SpatialVector Wrench(Vector3(0.1f, -0.4f, 0.2f), Vector3(2, -3, 1));
		std::vector<SpatialVector> Wrenches(Solver.GetLinkCount());
		const Vector3 Offset = Solver.GetLinkTransform(Link).quat * Point;
		Wrenches[Link] = SpatialVector(Wrench.GetAngular() + Offset.Cross(Wrench.GetLinear()), Wrench.GetLinear());
		const auto Force = Solver.ComputeGeneralizedForce(Wrenches);
		for (int Column = 0; Column < Solver.GetDofCount(); ++Column)
		{
			double Expected = 0; for (int Row = 0; Row < 6; ++Row) Expected += J[Row][Column] * Wrench[Row];
			Near(Force[Column], Expected, 3e-6);
		}
		bool Rejected = false;
		try { Solver.ComputeLinkJacobian(Link, Vector3(std::numeric_limits<float>::quiet_NaN())); } catch (const std::invalid_argument&) { Rejected = true; }
		EXPECT(Rejected); Rejected = false;
		try { Solver.ComputeLinkJacobian(Solver.GetLinkCount()); } catch (const std::out_of_range&) { Rejected = true; }
		EXPECT(Rejected); Rejected = false;
		try { Solver.ComputeGeneralizedForce({ Wrench }); } catch (const std::invalid_argument&) { Rejected = true; }
		EXPECT(Rejected);
	}
	void QueryCoordinatesAndScale()
	{
		auto Links = Tree(32, true, true); FeatherstoneSolver Origin(Links); SetVariedState(Origin);
		Links[0].ParentToJoint.pos += Vector3(1000000, -2000000, 3000000); FeatherstoneSolver Shifted(Links);
		Shifted.SetConfiguration(Origin.GetConfiguration()); Shifted.SetVelocity(Origin.GetVelocity());
		const auto H = Origin.ComputeMassMatrix(), ShiftedH = Shifted.ComputeMassMatrix();
		const auto J = Origin.ComputeDenseJacobian(), ShiftedJ = Shifted.ComputeDenseJacobian();
		for (int R = 0; R < H.GetRows(); ++R) for (int C = 0; C < H.GetCols(); ++C) Near(H[R][C], ShiftedH[R][C], 1e-12);
		for (int R = 0; R < J.GetRows(); ++R) for (int C = 0; C < J.GetCols(); ++C) Near(J[R][C], ShiftedJ[R][C], 1e-12);
		for (int Count : { 32, 128, 512 })
		{
			FeatherstoneSolver Solver(Tree(Count, true, true)); SetVariedState(Solver);
			const auto Start = std::chrono::steady_clock::now(); const auto Mass = Solver.ComputeMassMatrix();
			const auto Middle = std::chrono::steady_clock::now(); const auto Dense = Solver.ComputeDenseJacobian();
			const auto End = std::chrono::steady_clock::now();
			printf("Ragdoll queries links=%d CRBA=%.3f ms Jacobian=%.3f ms\n", Count,
				std::chrono::duration<double, std::milli>(Middle - Start).count(), std::chrono::duration<double, std::milli>(End - Middle).count());
			std::vector<double> Force(Solver.GetDofCount()); for (size_t I = 0; I < Force.size(); ++I) Force[I] = std::sin(I * 0.3);
			const auto Response = Solver.ComputeImpulseResponse({}, Force);
			for (int R = 0; R < Mass.GetRows(); ++R)
			{
				double Recovered = 0; for (int C = 0; C < Mass.GetCols(); ++C) Recovered += Mass[R][C] * Response[C];
				Near(Recovered, Force[R], 0.0001);
			}
			EXPECT(Dense.GetRows() == Count * 6 && Dense.GetCols() == Solver.GetDofCount());
		}
	}
}

void TestFeatherstone()
{
	printf("Running TestFeatherstone\n");
	AnalyticJoints(); DoublePendulum(); InverseAndImpulse(); KinematicDerivativeAndMass(); FreeBodyAndMomentum(); LongChainAndValidation();
	CoordinatesAndIntegration(); MovingLongChain();
}

void TestReducedRagdollQueries()
{
	DynamicsQueryIdentities(); PointJacobianDerivative(); QueryCoordinatesAndScale();
}
