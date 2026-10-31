#include "Test.h"

#include <limits>

#include "../Src/Destruction/StrainSolver.h"

using namespace Riemann;

namespace
{
	StrainSolverSettings StressSettings()
	{
		StrainSolverSettings Settings;
		Settings.Hardness = 1.0f;
		Settings.StressLinearFactor = 1.0f;
		Settings.StressAngularFactor = 1.0f;
		Settings.MaxIterations = 256;
		Settings.RelativeTolerance = 1.0e-7f;
		Settings.AbsoluteTolerance = 1.0e-8f;
		return Settings;
	}

	ConnectionGraph StressChain(int Count)
	{
		ConnectionGraph Graph;
		Graph.Resize((size_t)Count);
		for (int i = 1; i < Count; ++i) Graph.AddBond(i - 1, i, DestructionBondType::Connect, 100000.0f, 10 + i);
		return Graph;
	}

	void SetupStressChain(StrainSolver& Solver, int Count, bool StaticRoot = true)
	{
		for (int i = 0; i < Count; ++i)
		{
			Solver.SetNodeInfo(i, 1.0f + i, 1.0f, Vector3((float)i, 0.0f, 0.0f), StaticRoot && i == 0);
			EXPECT(Solver.SetNodeInertia(i, Matrix3(2.0f, 3.0f, 4.0f)));
		}
	}

	void ExpectStressEquilibrium(const StrainSolver& Solver)
	{
		EXPECT(Solver.HasConverged());
		EXPECT(Solver.GetStressErrorLinear() < 1.0e-4f);
		EXPECT(Solver.GetStressErrorAngular() < 1.0e-4f);
	}

	void TestStressAxialLoadsAndUnits()
	{
		printf("Running TestStressAxialLoadsAndUnits\n");
		const ConnectionGraph Graph = StressChain(2);
		StrainSolver Solver(Graph, StressSettings());
		SetupStressChain(Solver, 2);
		EXPECT(Solver.SetBondGeometry(11, Vector3(0.5f, 0.0f, 0.0f), Vector3::UnitX(), 2.0f));
		for (float Dt : { 0.1f, 0.02f })
		{
			Solver.AddForce(1, Vector3(12.0f, 0.0f, 0.0f), StrainForceMode::Force);
			Solver.Update(Dt);
			ExpectStressEquilibrium(Solver);
			const StrainBondInfo* Bond = Solver.GetBondInfo(11);
			EXPECT(Bond != nullptr);
			if (!Bond) return;
			EXPECT_NEAR(Bond->ImpulseLinear.x, 12.0f * Dt, 1.0e-5f);
			EXPECT_NEAR(Bond->ForceLinear.x, 12.0f, 1.0e-5f);
			EXPECT_NEAR(Bond->Tension, 6.0f, 1.0e-5f);
			EXPECT_NEAR(Bond->Compression, 0.0f, 1.0e-5f);
			EXPECT_NEAR(Bond->Shear, 0.0f, 1.0e-5f);
			EXPECT_NEAR(Bond->Stress, 6.0f, 1.0e-5f);
		}
		Solver.AddForce(1, Vector3(-8.0f, 0.0f, 0.0f), StrainForceMode::Force);
		Solver.Update(0.1f);
		EXPECT_NEAR(Solver.GetBondInfo(11)->Compression, 4.0f, 1.0e-5f);
		EXPECT_NEAR(Solver.GetBondInfo(11)->Tension, 0.0f, 1.0e-5f);
		Solver.Update(0.1f);
		EXPECT_NEAR(Solver.GetBondStress(11), 0.0f, 1.0e-5f);
		EXPECT(Solver.GetLastIterationCount() == 0);
		Solver.AddForce(1, Vector3(2.0f, 0.0f, 0.0f), StrainForceMode::VelocityChange);
		Solver.Update(0.5f);
		EXPECT_NEAR(Solver.GetBondInfo(11)->ImpulseLinear.x, 4.0f, 1.0e-5f);
		EXPECT_NEAR(Solver.GetBondInfo(11)->ForceLinear.x, 8.0f, 1.0e-5f);
	}

	void TestStressEccentricLoadsAndTorque()
	{
		printf("Running TestStressEccentricLoadsAndTorque\n");
		StrainSolver Solver(StressChain(2), StressSettings());
		Solver.SetNodeInfo(0, 1.0f, 1.0f, Vector3::Zero(), true);
		Solver.SetNodeInfo(1, 3.0f, 1.0f, Vector3(2.0f, 0.0f, 0.0f), false);
		EXPECT(Solver.SetNodeInertia(1, Matrix3(4.0f, 1.0f, 0.5f, 1.0f, 3.0f, 0.2f, 0.5f, 0.2f, 2.0f)));
		EXPECT(Solver.SetBondGeometry(11, Vector3::UnitX(), Vector3::UnitX(), 2.0f));
		EXPECT(Solver.AddForceAtPosition(1, Vector3(2.0f, 0.0f, 1.0f), Vector3(0.0f, 10.0f, 0.0f), StrainForceMode::Force));
		Solver.Update(0.1f);
		ExpectStressEquilibrium(Solver);
		const StrainBondInfo* Bond = Solver.GetBondInfo(11);
		EXPECT_SAME(Bond->ForceLinear, Vector3(0.0f, 10.0f, 0.0f));
		EXPECT_SAME(Bond->Torque, Vector3(-10.0f, 0.0f, 10.0f));
		const float Radius = std::sqrt(2.0f / 3.14159265358979323846f);
		EXPECT_NEAR(Bond->Bending, 40.0f / (2.0f * Radius), 1.0e-4f);
		EXPECT_NEAR(Bond->Torsion, 20.0f / (2.0f * Radius), 1.0e-4f);
		EXPECT_NEAR(Bond->Shear, 5.0f + Bond->Torsion, 1.0e-4f);
		EXPECT_NEAR(Bond->Tension, Bond->Bending, 1.0e-4f);
		EXPECT_NEAR(Bond->Compression, Bond->Bending, 1.0e-4f);
		EXPECT(Solver.AddTorque(1, Vector3(6.0f, 0.0f, 0.0f), StrainForceMode::Force));
		Solver.Update(0.2f);
		ExpectStressEquilibrium(Solver);
		EXPECT_SAME(Solver.GetBondInfo(11)->ForceLinear, Vector3::Zero());
		EXPECT_SAME(Solver.GetBondInfo(11)->Torque, Vector3(6.0f, 0.0f, 0.0f));
	}

	void TestStressFreeRigidMotionAndMomentum()
	{
		printf("Running TestStressFreeRigidMotionAndMomentum\n");
		StrainSolver Solver(StressChain(2), StressSettings());
		const Vector3 A(-1.0f, 0.0f, 0.0f), B(1.0f, 0.0f, 0.0f);
		const Matrix3 IA(2.0f, 3.0f, 4.0f);
		const Matrix3 IB(4.0f, 0.5f, 0.0f, 0.5f, 2.0f, 0.25f, 0.0f, 0.25f, 3.0f);
		Solver.SetNodeInfo(0, 1.0f, 1.0f, A, false);
		Solver.SetNodeInfo(1, 3.0f, 1.0f, B, false);
		Solver.SetNodeInertia(0, IA);
		Solver.SetNodeInertia(1, IB);
		Solver.AddForce(1, Vector3(0.0f, 4.0f, 0.0f));
		Solver.AddTorque(1, Vector3(1.0f, 2.0f, 3.0f));
		Solver.Update();
		ExpectStressEquilibrium(Solver);
		const Vector3 PA = Solver.GetNodeLinearVelocity(0);
		const Vector3 PB = Solver.GetNodeLinearVelocity(1) * 3.0f;
		const Vector3 WA = Solver.GetNodeAngularVelocity(0), WB = Solver.GetNodeAngularVelocity(1);
		EXPECT_SAME(PA + PB, Vector3(0.0f, 4.0f, 0.0f));
		EXPECT_SAME(A.Cross(PA) + B.Cross(PB) + IA * WA + IB * WB, Vector3(1.0f, 2.0f, 7.0f));
		EXPECT_SAME(WA, WB);

		Solver.AddForce(0, Vector3(2.0f, -3.0f, 1.0f), StrainForceMode::VelocityChange);
		Solver.AddForce(1, Vector3(2.0f, -3.0f, 1.0f), StrainForceMode::VelocityChange);
		Solver.Update();
		EXPECT_NEAR(Solver.GetBondStress(11), 0.0f, 1.0e-5f);
		EXPECT_SAME(Solver.GetNodeLinearVelocity(0), Vector3(2.0f, -3.0f, 1.0f));
		EXPECT_SAME(Solver.GetNodeLinearVelocity(1), Vector3(2.0f, -3.0f, 1.0f));
		Solver.Update();
		EXPECT_SAME(Solver.GetNodeLinearVelocity(0), Vector3::Zero());
	}

	void TestStressWarmStartAndCyclicGraph()
	{
		printf("Running TestStressWarmStartAndCyclicGraph\n");
		ConnectionGraph Graph = StressChain(4);
		Graph.AddBond(0, 2, DestructionBondType::Connect, 100000.0f, 30);
		Graph.AddBond(1, 3, DestructionBondType::Connect, 100000.0f, 31);
		StrainSolverSettings Settings = StressSettings();
		StrainSolver Warm(Graph, Settings);
		Settings.WarmStart = false;
		StrainSolver Cold(Graph, Settings);
		SetupStressChain(Warm, 4);
		SetupStressChain(Cold, 4);
		for (int Frame = 0; Frame < 8; ++Frame)
		{
			const Vector3 Force((float)Frame, 3.0f - Frame, 0.5f * Frame);
			for (StrainSolver* Solver : { &Warm, &Cold })
			{
				Solver->AddForce(3, Force, StrainForceMode::Force);
				Solver->AddTorque(2, Vector3(1.0f, -0.3f * Frame, 0.2f), StrainForceMode::Force);
				Solver->Update(1.0f / 60.0f);
				ExpectStressEquilibrium(*Solver);
			}
			for (const StrainBondInfo& Bond : Cold.GetBonds())
			{
				const StrainBondInfo* Other = Warm.GetBondInfo(Bond.BondIndex);
				EXPECT((Bond.ForceLinear - Other->ForceLinear).Length() < 2.0e-4f);
				EXPECT((Bond.Torque - Other->Torque).Length() < 2.0e-4f);
			}
		}
		Warm.Update();
		for (const StrainBondInfo& Bond : Warm.GetBonds()) EXPECT_NEAR(Bond.Stress, 0.0f, 1.0e-5f);
	}

	void TestStressTopologyAndReset()
	{
		printf("Running TestStressTopologyAndReset\n");
		ConnectionGraph Graph = StressChain(3);
		StrainSolver Solver(Graph, StressSettings());
		SetupStressChain(Solver, 3);
		Solver.SetBondGeometry(11, Vector3(0.25f, 0.0f, 0.0f), Vector3::UnitX(), 3.0f);
		Solver.AddForce(2, Vector3(12.0f, 0.0f, 0.0f), StrainForceMode::Force);
		Solver.Update(0.1f);
		EXPECT_NEAR(Solver.GetBondInfo(11)->ForceLinear.x, 12.0f, 1.0e-4f);
		EXPECT_NEAR(Solver.GetBondInfo(12)->ForceLinear.x, 12.0f, 1.0e-4f);
		Solver.AddForce(2, Vector3(9.0f, 0.0f, 0.0f));
		Graph.BreakBond(1, 2);
		EXPECT(Solver.SyncGraph(Graph) == 1);
		EXPECT(Solver.GetBondInfo(12) == nullptr);
		EXPECT_NEAR(Solver.GetBondInfo(11)->Area, 3.0f, 1.0e-6f);
		Solver.Update();
		EXPECT_NEAR(Solver.GetBondStress(11), 0.0f, 1.0e-5f);
		EXPECT_NEAR(Solver.GetNodeLinearVelocity(2).x, 3.0f, 1.0e-5f);
		Graph.AddBond(1, 2, DestructionBondType::Connect, 100000.0f, 19);
		Solver.SyncGraph(Graph);
		EXPECT(Solver.GetBondInfo(19) != nullptr);
		EXPECT_NEAR(Solver.GetBondInfo(11)->Centroid.x, 0.25f, 1.0e-6f);
		Solver.AddForce(2, Vector3(12.0f, 0.0f, 0.0f), StrainForceMode::Force);
		Solver.Update(0.1f);
		EXPECT_NEAR(Solver.GetBondInfo(19)->ForceLinear.x, 12.0f, 1.0e-4f);
		EXPECT(Solver.SetBondHealth(19, 0.01f));
		EXPECT(Solver.GetOverstressedBondCount() == 1);
		DestructionFractureBuffer Commands;
		Solver.GenerateFractureCommands(Commands);
		EXPECT(Commands.BondFractures.size() == 1 && Commands.BondFractures[0] == 19);
		Solver.Reset();
		EXPECT(Solver.GetFrameCount() == 0);
		EXPECT(Solver.GetOverstressedBondCount() == 0);
		for (const StrainBondInfo& Bond : Solver.GetBonds())
		{
			EXPECT_NEAR(Bond.Stress, 0.0f, 1.0e-6f);
			EXPECT_SAME(Bond.ForceLinear, Vector3::Zero());
		}
		EXPECT(Solver.RemoveBond(19));
		EXPECT_FALSE(Solver.RemoveBond(19));
		EXPECT(Solver.GetBondCount() == 1);
		EXPECT(Solver.SetBondHealth(11, 0.0f));
		EXPECT(Solver.GetBondCount() == 0);
		Graph.Resize(1);
		Solver.ResetGraph(Graph);
		EXPECT(Solver.GetNodeCount() == 1);
		EXPECT(Solver.GetBondCount() == 0);
		Solver.AddForce(2, Vector3::UnitX());
		Solver.Update();
		EXPECT(Solver.HasConverged());
	}

	void TestStressInertiaAndValidation()
	{
		printf("Running TestStressInertiaAndValidation\n");
		DestructionSet Set;
		Set.AddChunk(Box3(Vector3(-1.0f, -1.0f, -1.0f), Vector3(1.0f, 1.0f, 1.0f)), 2.0f, false);
		StrainSolver Solver(Set, StressSettings());
		EXPECT(Solver.SetNodeInertia(0, Matrix3(2.0f, 4.0f, 8.0f)));
		Solver.SetAllNodesInfo(Set);
		EXPECT(Solver.AddTorque(0, Vector3(2.0f, 4.0f, 8.0f)));
		Solver.Update();
		EXPECT_SAME(Solver.GetNodeAngularVelocity(0), Vector3(1.0f, 1.0f, 1.0f));
		EXPECT_FALSE(Solver.SetNodeInertia(0, Matrix3(-1.0f, 1.0f, 1.0f)));
		EXPECT_FALSE(Solver.SetNodeInertia(0, Matrix3(1.0f, 2.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f)));
		EXPECT_FALSE(Solver.SetNodeInertia(4, Matrix3::Identity()));
		EXPECT_FALSE(Solver.SetNodeInertia(0, Matrix3(1.0e-40f, 1.0e-40f, 1.0e-40f)));
		EXPECT_FALSE(Solver.AddTorque(-1, Vector3::UnitX()));
		EXPECT_FALSE(Solver.SetBondGeometry(99, Vector3::Zero(), Vector3::UnitX(), 1.0f));
		Solver.AddForce(0, Vector3(2.0f, 0.0f, 0.0f));
		const uint32_t Frame = Solver.GetFrameCount();
		Solver.Update(std::numeric_limits<float>::quiet_NaN());
		Solver.Update(0.0f);
		EXPECT(Solver.GetFrameCount() == Frame);
		Solver.Update();
		EXPECT_NEAR(Solver.GetNodeLinearVelocity(0).x, 1.0f, 1.0e-5f);

		StrainSolverSettings Settings = StressSettings();
		Settings.MaxIterations = 1;
		StrainSolver Limited(StressChain(12), Settings);
		SetupStressChain(Limited, 12);
		Limited.AddForce(11, Vector3(1.0f, 5.0f, 0.0f));
		Limited.Update();
		EXPECT_FALSE(Limited.HasConverged());
		EXPECT(Limited.GetLastIterationCount() == 1);
		EXPECT(Limited.GetRelativeResidual() > Settings.RelativeTolerance);

		Vector3 AngularVelocity[2];
		for (int Translated = 0; Translated < 2; ++Translated)
		{
			StrainSolver Free(StressChain(2), StressSettings());
			const Vector3 Offset(10.0f * Translated, 0.0f, 0.0f);
			Free.SetNodeInfo(0, 1.0f, 1.0f, Offset, false);
			Free.SetNodeInfo(1, 1.0f, 1.0f, Offset + Vector3::UnitX(), false);
			Free.AddTorque(1, Vector3::UnitX());
			Free.Update();
			ExpectStressEquilibrium(Free);
			AngularVelocity[Translated] = Free.GetNodeAngularVelocity(0);
		}
		EXPECT_SAME(AngularVelocity[0], AngularVelocity[1]);
	}
}

void TestStressSolver()
{
	TestStressAxialLoadsAndUnits();
	TestStressEccentricLoadsAndTorque();
	TestStressFreeRigidMotionAndMomentum();
	TestStressWarmStartAndCyclicGraph();
	TestStressTopologyAndReset();
	TestStressInertiaAndValidation();
}
