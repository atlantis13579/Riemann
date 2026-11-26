#include "Test.h"
#include "../Src/RigidBodyDynamics/PhysicsWorld.h"
#include "../Src/RigidBodyDynamics/ForceField.h"
#include "../Src/RigidBodyDynamics/CollidingContact.h"
#include "../Src/Collision/GeometryObject.h"
#include "../Src/RigidBodyDynamics/NarrowPhase.h"
#include <memory>
#include <chrono>
#include <stdexcept>

namespace
{
	using namespace Riemann;
	const ResolutionPhaseSolver Solvers[] = { ResolutionPhaseSolver::SequentialImpulse, ResolutionPhaseSolver::LCPGlobal,
		ResolutionPhaseSolver::PBD, ResolutionPhaseSolver::XPBD };
	const float FrameDt = 0.1f;
	PhysicsWorldParam Parameters(ResolutionPhaseSolver Solver, int Count, SolverBackend Backend = SolverBackend::CPU)
	{
		PhysicsWorldParam P; P.resolutionSolver = Solver; P.solverBackend = Backend; P.allowBackendFallback = false;
		P.pbdSettings.Backend = Backend; P.xpbdSettings.Backend = Backend;
		P.temporalSubsteps.Enabled = true; P.temporalSubsteps.Substeps = Count;
		return P;
	}
	RigidBody* Body(PhysicsWorld& World, const Vector3& Position, bool Dynamic = true, bool Collision = false)
	{
		RigidBodyParam P; P.rigidType = Dynamic ? RigidType::Dynamic : RigidType::Static;
		P.linearDamping = P.angularDamping = 1; P.sleepThreshold = 0;
		auto* Shape = GeometryFactory::CreateSphere(Position, 0.5f); Shape->SetSimulationEnabled(Collision);
		auto* B = World.CreateRigidBody(Shape, P); B->SetRestitution(0); B->SetFriction(0, 0);
		if (Dynamic) { B->InvMass = 1; B->InvInertia = Matrix3(1, 1, 1); }
		return B;
	}
	class ConstantField : public ForceField
	{
	public:
		Vector3 Force = Vector3(10, 0, 0), Torque = Vector3(0, 0, 4);
		int Updates = 0; float Time = 0;
		void Update(float Dt) override { ++Updates; Time += Dt; }
		bool ApplyForce(RigidBodyDynamic* B) override { B->ApplyForce(Force); B->ApplyTorgue(Torque); return true; }
	};
	void TestTimeAndLoads()
	{
		for (auto Solver : Solvers)
		for (int Count : { 1, 2, 4, 8 })
		{
			PhysicsWorld World(Parameters(Solver, Count));
			auto* Field = new ConstantField; World.AddForceField(Field);
			auto* B = Body(World, Vector3::Zero())->CastDynamic(); B->ApplyForce(Vector3(4, 0, 0));
			World.Simulate(FrameDt);
			EXPECT(Field->Updates == Count); EXPECT_NEAR(Field->Time, FrameDt, 1e-6f);
			EXPECT_NEAR(B->V.x, 3, 1e-5f); EXPECT_NEAR(B->W.z, 0.4f, 1e-5f);
			const float ExpectedX = 2 * FrameDt + 10 * FrameDt * FrameDt * (Count + 1) / (2 * Count);
			EXPECT_NEAR(B->X.x, ExpectedX, 1e-5f); EXPECT(B->ExtForce == Vector3::Zero());
			EXPECT(World.GetStepInfo().SubstepCount == Count && World.GetStepInfo().SubstepIndex == Count - 1);
			EXPECT_NEAR(World.GetStepInfo().DeltaTime, FrameDt / Count, 1e-6f);
		}
	}
	void TestCallbackLoads()
	{
		for (auto Solver : Solvers)
		{
			auto P = Parameters(Solver, 4); RigidBodyDynamic* Free = nullptr; bool Queued = false;
			std::vector<PhysicsStepInfo> Steps;
			P.onContact = [&](const PhysicsContactInfo& Info)
			{
				Steps.push_back(Info.Step);
				if (!Queued) { Queued = true; Free->ApplyForce(Vector3(4, 0, 0)); }
			};
			PhysicsWorld World(P); auto* Floor = GeometryFactory::CreatePlane(Vector3::Zero(), Vector3::UnitY());
			World.CreateRigidBody(Floor, RigidBodyParam())->SetRestitution(0);
			Body(World, Vector3(0, 0.49f, 0), true, true);
			Free = Body(World, Vector3(10, 10, 0))->CastDynamic(); World.Simulate(FrameDt);
			EXPECT(Queued && !Steps.empty()); EXPECT_NEAR(Free->V.x, 0, 1e-6f); EXPECT_NEAR(Free->ExtForce.x, 4, 1e-6f);
			for (const auto& Step : Steps)
			{
				EXPECT(Step.SubstepCount == 4 && Step.SubstepIndex >= 0 && Step.SubstepIndex < 4);
				EXPECT_NEAR(Step.DeltaTime, FrameDt / 4, 1e-6f); EXPECT_NEAR(Step.FrameDeltaTime, FrameDt, 1e-6f);
			}
			World.Simulate(FrameDt); EXPECT_NEAR(Free->V.x, 2, 1e-5f); EXPECT_NEAR(Free->X.x, 10 + 2 * FrameDt, 1e-5f);
		}
	}
	void TestFailureBookkeeping()
	{
		class FailingField : public ForceField
		{
		public:
			int Updates = 0;
			void Update(float) override { if (++Updates == 2) throw std::runtime_error("deliberate substep failure"); }
			bool ApplyForce(RigidBodyDynamic*) override { return false; }
		};
		auto P = Parameters(ResolutionPhaseSolver::SequentialImpulse, 4);
		RigidBodyDynamic* Free = nullptr; bool Queued = false;
		P.onContact = [&](const PhysicsContactInfo&) { if (!Queued) { Queued = true; Free->ApplyForce(Vector3(4, 0, 0)); } };
		PhysicsWorld World(P); World.AddForceField(new FailingField());
		World.CreateRigidBody(GeometryFactory::CreatePlane(Vector3::Zero(), Vector3::UnitY()), RigidBodyParam());
		Body(World, Vector3(0, 0.49f, 0), true, true); Free = Body(World, Vector3(10, 0, 0))->CastDynamic();
		bool Failed = false; try { World.Simulate(FrameDt); } catch (const std::runtime_error&) { Failed = true; }
		EXPECT(Failed && Queued); EXPECT_NEAR(Free->ExtForce.x, 4, 1e-6f);
		auto Settings = World.GetTemporalSubstepSettings(); Settings.Enabled = false; World.SetTemporalSubstepSettings(Settings);
		World.Simulate(FrameDt); EXPECT_NEAR(Free->V.x, 2, 1e-5f); EXPECT(Free->ExtForce == Vector3::Zero());
	}
	void TestFrameReactions()
	{
		for (auto Solver : Solvers)
		for (auto Backend : { SolverBackend::CPU, SolverBackend::SIMD, SolverBackend::ComputeShader, SolverBackend::CUDA })
		{
			if (!PhysicsWorld::IsSolverBackendAvailable(Solver, Backend)) continue;
			for (int Count : { 1, 4 })
			{
				auto P = Parameters(Solver, Count, Backend); int Breaks = 0; PhysicsJointBreakInfo Report;
				P.onJointBreak = [&](const PhysicsJointBreakInfo& Event) { ++Breaks; Report = Event; };
				PhysicsWorld World(P);
				auto* A = Body(World, Vector3::Zero(), false); auto* B = Body(World, Vector3::Zero())->CastDynamic();
				auto* Joint = World.CreateFixedConstraint(A, B);
				JointBreakSettings Break; Break.Enabled = true; Break.Force = 15; Joint->SetBreakSettings(Break);
				B->ApplyForce(Vector3(0, -2, 0)); World.Simulate(FrameDt);
				EXPECT(Breaks == 0 && !Joint->IsBroken()); EXPECT_NEAR(Joint->GetReaction().Force.y, 10, 0.002f);
				Break.Force = 5; Joint->SetBreakSettings(Break); B->ApplyForce(Vector3(0, -2, 0)); World.Simulate(FrameDt);
				EXPECT(Breaks == 1 && Joint->IsBroken()); EXPECT_NEAR(Report.Reaction.Force.y, 10, 0.002f);
				EXPECT_NEAR(Report.DeltaTime, FrameDt, 1e-6f); EXPECT(Report.Tick == 2);
				World.Simulate(FrameDt); EXPECT(Breaks == 1);
			}
		}
		for (auto Solver : Solvers)
		{
			auto P = Parameters(Solver, 4); P.gravityAcc = Vector3(0, -10, 0); PhysicsWorld World(P);
			auto* A = Body(World, Vector3::Zero(), false); auto* B = Body(World, Vector3::Zero());
			auto* Joint = World.CreateFixedConstraint(A, B);
			auto* Field = new ConstantField; Field->Force.SetZero(); World.AddForceField(Field);
			World.Simulate(FrameDt); EXPECT_NEAR(Joint->GetReaction().Force.y, 10, 0.002f);
			EXPECT_NEAR(Joint->GetReaction().Torque.z, -4, 0.002f);
		}
	}
	void TestDriveBudget()
	{
		for (auto Solver : Solvers)
		for (int Count : { 1, 4, 8 })
		{
			PhysicsWorld World(Parameters(Solver, Count));
			auto* A = Body(World, Vector3::Zero(), false); auto* B = Body(World, Vector3::Zero());
			auto* Joint = World.CreateSliderConstraint(A, B, Vector3::Zero(), Vector3::UnitX(), -100, 100);
			ScalarJointDrive Drive; Drive.Settings.VelocityEnabled = true; Drive.Settings.Damping = 100;
			Drive.Settings.MaxForce = 6; Drive.TargetVelocity = 1000; Joint->SetDrive(Drive);
			World.Simulate(FrameDt);
			EXPECT_NEAR(B->V.x, 6 * FrameDt, 0.0001f); EXPECT_NEAR(Joint->GetReaction().Force.x, 6, 0.001f);
			EXPECT_NEAR(Joint->GetDriveImpulse(), 6 * FrameDt / Count, 0.0001f);
		}
	}
	void TestStepEquivalence()
	{
		for (auto Solver : Solvers)
		{
			auto P = Parameters(Solver, 4); P.gravityAcc = Vector3(0, -10, 0); P.narrowphase = NarrowPhaseSolver::PCM;
			PhysicsWorld Refined(P); P.temporalSubsteps.Enabled = false; PhysicsWorld Reference(P);
			RigidBody* B[2]; int Index = 0;
			for (auto* World : { &Refined, &Reference })
			{
				World->CreateRigidBody(GeometryFactory::CreatePlane(Vector3::Zero(), Vector3::UnitY()), RigidBodyParam())->SetRestitution(0);
				B[Index++] = Body(*World, Vector3(0, 0.6f, 0), true, true);
			}
			for (int Frame = 0; Frame < 30; ++Frame)
			{
				Refined.Simulate(1.0f / 60); for (int Step = 0; Step < 4; ++Step) Reference.Simulate(1.0f / 240);
				EXPECT((B[0]->X - B[1]->X).Length() < 1e-5f); EXPECT((B[0]->V - B[1]->V).Length() < 1e-5f);
			}
			EXPECT(Refined.GetCachedContactPairCount() > 0);
		}
	}
	void TestCCDTime()
	{
		auto P = Parameters(ResolutionPhaseSolver::SequentialImpulse, 4); P.ccdContinuation.Enabled = true;
		std::vector<PhysicsTriggerInfo> Events; P.onTrigger = [&](const PhysicsTriggerInfo& Info) { if (Info.IsCCD) Events.push_back(Info); };
		PhysicsWorld World(P); auto* Wall = GeometryFactory::CreatePlane(Vector3(2, 0, 0), -Vector3::UnitX());
		World.CreateRigidBody(Wall, RigidBodyParam())->SetRestitution(0);
		auto* Trigger = GeometryFactory::CreateOBB(Vector3::Zero(), Vector3(0.1f, 5, 5));
		Trigger->SetTriggerEnabled(true); Trigger->SetTriggerCCDEnabled(true); World.CreateRigidBody(Trigger, RigidBodyParam());
		auto* B = Body(World, Vector3(-5, 0, 0), true, true)->CastDynamic();
		B->mMotionType = MorionType::Continuous; B->V = Vector3(100, 0, 0); B->SetRestitution(1);
		World.Simulate(FrameDt); EXPECT(World.GetLastCCDHitCount() == 1); EXPECT(Events.size() == 2);
		if (Events.size() == 2)
		{
			EXPECT_NEAR(Events[0].Fraction, 0.44f, 0.001f); EXPECT_NEAR(Events[1].Fraction, 0.74f, 0.001f);
			EXPECT(Events[0].Step.SubstepIndex == 1 && Events[1].Step.SubstepIndex == 2);
		}
		EXPECT_NEAR(B->X.x, -2, 0.01f); EXPECT_NEAR(B->V.x, -100, 0.01f);
	}
	struct StabilityResult { float Error = 0, Speed = 0, HeightError = 0, Drift = 0, AngularSpeed = 0; int BlockFallbacks = 0; double Milliseconds = 0; };
	StabilityResult MeasureStability(ResolutionPhaseSolver Solver, int Count, bool Chain, float MassRatio, bool Support = false, SolverBackend Backend = SolverBackend::CPU, int Budget = 4, int Size = 8, int Frames = 120)
	{
		// Uncap physical contacts to measure conditioning rather than an intentional impulse budget.
		auto P = Parameters(Solver, Count, Backend); P.gravityAcc = Vector3(0, -10, 0); P.narrowphase = NarrowPhaseSolver::PCM;
		P.supportStabilization.Enabled = Support;
		const int Iterations = Budget / Count;
		P.constraintIterations = P.contactSolverIterations = Iterations;
		P.sequentialImpulseSettings.PositionIterations = P.sequentialImpulseSettings.VelocityIterations = Iterations;
		P.pbdSettings.PositionIterations = P.pbdSettings.VelocityIterations = Iterations;
		P.xpbdSettings.PositionIterations = P.xpbdSettings.VelocityIterations = Iterations;
		PhysicsWorld World(P); std::vector<RigidBody*> Bodies;
		RigidBody* Previous = nullptr;
		if (Chain) Previous = Body(World, Vector3(0, float(Size), 0), false);
		else World.CreateRigidBody(GeometryFactory::CreatePlane(Vector3::Zero(), Vector3::UnitY()), RigidBodyParam())->SetRestitution(0);
		for (int Index = 0; Index < Size; ++Index)
		{
			RigidBody* B;
			if (Chain) B = Body(World, Vector3(0, float(Size - Index - 1), 0));
			else
			{
				RigidBodyParam Params; Params.maxContactImpulse = std::numeric_limits<float>::max(); Params.rigidType = RigidType::Dynamic; Params.linearDamping = Params.angularDamping = 1; Params.sleepThreshold = 0;
				B = World.CreateRigidBody(GeometryFactory::CreateOBB(Vector3(0, Index + 0.5f, 0), Vector3(0.5f)), Params);
				B->InvMass = 1; B->InvInertia = Matrix3(6, 6, 6); B->SetRestitution(0);
			}
			if (Index + 1 == Size) { B->InvMass /= MassRatio; B->InvInertia = B->InvInertia * (1 / MassRatio); }
			if (Chain) World.CreateDistanceConstraint(Previous, B, Previous->X, B->X, 1, 1);
			Previous = B; Bodies.push_back(B);
		}
		if (Chain) Bodies.back()->V = Vector3(5, 0, 0);
		StabilityResult Result; const auto Start = std::chrono::steady_clock::now();
		for (int Frame = 0; Frame < Frames; ++Frame)
		{
			World.Simulate(1.0f / 60);
			Result.BlockFallbacks += World.GetLastSupportStabilizationStats().ContactBlockFallbacks;
			for (int Index = 0; Index < Size; ++Index)
			{
				auto* B = Bodies[Index]; EXPECT(std::isfinite(B->X.Length()) && std::isfinite(B->V.Length()));
				if (Frame < Frames * 3 / 4) continue;
				const Vector3 Anchor = Index == 0 ? Vector3(0, float(Size), 0) : Bodies[Index - 1]->X;
				const float Error = Chain ? std::abs((B->X - Anchor).Length() - 1) : std::max(0.0f, Index + 0.5f - B->X.y);
				Result.Error = std::max(Result.Error, Error); Result.Speed = std::max(Result.Speed, B->V.Length());
				if (!Chain)
				{
					Result.HeightError = std::max(Result.HeightError, std::abs(Index + 0.5f - B->X.y));
					Result.Drift = std::max(Result.Drift, Vector3(B->X.x, 0, B->X.z).Length());
					Result.AngularSpeed = std::max(Result.AngularSpeed, B->W.Length());
				}
			}
		}
		Result.Milliseconds = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - Start).count();
		return Result;
	}
	void TestStabilityEvidence()
	{
		for (auto Solver : Solvers)
		for (bool Chain : { false, true })
		for (float Ratio : { 1.0f, 100.0f })
		{
			const auto One = MeasureStability(Solver, 1, Chain, Ratio), Four = MeasureStability(Solver, 4, Chain, Ratio);
			printf("Temporal equal-budget solver=%d chain=%d ratio=%g error=%g -> %g speed=%g -> %g time_ms=%g -> %g\n",
				(int)Solver, Chain, Ratio, One.Error, Four.Error, One.Speed, Four.Speed, One.Milliseconds, Four.Milliseconds);
			if (Solver == ResolutionPhaseSolver::LCPGlobal && !Chain && Ratio == 1) EXPECT(Four.Error < One.Error / 2);
			if ((Solver == ResolutionPhaseSolver::PBD || Solver == ResolutionPhaseSolver::XPBD) && Chain && Ratio == 100)
				EXPECT(Four.Error < One.Error / 2);
		}
	}
}
void TestTemporalSubsteps()
{
	printf("Running TestTemporalSubsteps\n");
	TestTimeAndLoads(); TestCallbackLoads(); TestFailureBookkeeping(); TestFrameReactions(); TestDriveBudget(); TestStepEquivalence(); TestCCDTime();   TestStabilityEvidence();
}

void TestSupportStabilization()
{
	// Plane witnesses must stay on the actual half-space in both pair orders.
	for (const auto& Normal : { Vector3::UnitY(), Vector3(1, 2, 3).Unit() })
	for (float Scale : { 0.1f, 1.0f, 100.0f })
	for (bool Reverse : { false, true })
	{
		const Vector3 Origin = Vector3(7, -3, 4) * Scale;
		std::unique_ptr<Geometry> Floor(GeometryFactory::CreatePlane(Origin, Normal));
		std::unique_ptr<Geometry> Box(GeometryFactory::CreateOBB(Origin + Normal * (0.4f * Scale), Vector3(0.5f * Scale)));
		std::unique_ptr<NarrowPhase> Narrow(NarrowPhase::Create_PCM());
		ContactGenerationSettings Settings; Settings.UseShapeOffsets = false; Narrow->SetContactGeneration(Settings, FrameDt);
		for (int Frame = 0; Frame < 8; ++Frame)
		{
			Box->SetPosition(Origin + Normal * ((0.4f + Frame * 0.001f) * Scale));
			GeometryWorldState States[2]; States[0].Geom = Reverse ? Box.get() : Floor.get(); States[1].Geom = Reverse ? Floor.get() : Box.get();
			for (auto& State : States) { State.ShapeToWorld = *State.Geom->GetTransform(); State.WorldBounds = State.Geom->GetBounds(); }
			const GeometryWorldState* Pointers[] = { &States[0], &States[1] }; std::vector<ContactManifold*> Contacts;
			Narrow->BeginFrame(Frame + 1); Narrow->CollisionDetection(GeometryWorldStateSpan(Pointers, 2), { OverlapPair(0, 1) }, &Contacts);
			EXPECT(!Contacts.empty());
			for (auto* M : Contacts) for (int I = 0; I < M->NumContactPointCount; ++I)
			{
				const auto& C = M->ContactPoints[I]; const Vector3 Point = Reverse ? C.PositionWorldB : C.PositionWorldA;
				EXPECT(std::abs((Point - Origin).Dot(Normal)) < Scale * 1e-5f);
				EXPECT_NEAR((C.PositionWorldA - C.PositionWorldB).Dot(C.Normal), C.PenetrationDepth, Scale * 1e-5f);
				EXPECT(C.SolverSlop < 0);
			}
		}
	}
	// Static friction stops small slip; sliding uses a circular, not axis-wise cap.
	for (float Speed : { 0.2f, 10.0f })
	for (const auto& Direction : { Vector3::UnitX(), Vector3(1, 0, 1).Unit() })
	{
		PhysicsWorld W{ PhysicsWorldParam() };
		auto* Floor = W.CreateRigidBody(GeometryFactory::CreatePlane(Vector3::Zero(), Vector3::UnitY()), RigidBodyParam());
		auto* Box = Body(W, Vector3(0, 0.4f, 0), true, true); Box->InvInertia = Matrix3(6, 6, 6);
		Floor->SetFriction(0.25f, 0.6f); Box->SetFriction(0.25f, 0.6f); Box->V = Direction * Speed - Vector3(0, 3, 0);
		ContactManifold Contact; Contact.indexA = 0; Contact.indexB = 1; Contact.NumContactPointCount = 4;
		for (int I = 0; I < 4; ++I)
		{
			auto& C = Contact.ContactPoints[I]; C.Normal = Vector3::UnitY(); C.PenetrationDepth = 0.1f;
			C.PositionWorldA = Vector3((I & 1) ? 0.5f : -0.5f, 0, (I & 2) ? 0.5f : -0.5f);
			C.PositionWorldB = C.PositionWorldA - Vector3(0, 0.1f, 0);
		}
		SupportGraph G; G.Build({ Floor->mGeometries[0], Box->mGeometries[0] }, { &Contact }, {}, 0.001f);
		SupportStabilizationSettings S; S.Enabled = true; S.PositionIterations = S.VelocityIterations = 1;
		SupportStabilizationStats Stats; StabilizeSupports(G, S, FrameDt, Stats);
		EXPECT_NEAR(Box->V.Dot(Direction), Speed < 1 ? 0 : Speed - 3 * 0.25f, 1e-5f);
		EXPECT(std::abs(Box->V.y) < 1e-5f && Box->W.Length() < 1e-5f); EXPECT(Stats.ContactBlockFallbacks == 0);
	}
	// A redundant flat four-point patch must resolve in one block without torque.
	for (bool Reverse : { false, true })
	for (bool Rebound : { false, true })
	{
		PhysicsWorld W{ PhysicsWorldParam() };
		auto* Floor = W.CreateRigidBody(GeometryFactory::CreatePlane(Vector3::Zero(), Vector3::UnitY()), RigidBodyParam());
		auto* Box = Body(W, Vector3(0, 0.4f, 0), true, true); Box->InvInertia = Matrix3(6, 6, 6);
		Box->V = Vector3(0, Rebound ? 3.0f : -3.0f, 0);
		Box->W = Rebound ? Vector3::Zero() : Vector3(0, 0, 1);
		ContactManifold Contact; Contact.indexA = 0; Contact.indexB = 1; Contact.NumContactPointCount = 4;
		for (int I = 0; I < 4; ++I)
		{
			const int Corner = Reverse ? 3 - I : I; auto& C = Contact.ContactPoints[I];
			C.Normal = Vector3::UnitY(); C.PositionWorldA = Vector3((Corner & 1) ? 0.5f : -0.5f, 0, (Corner & 2) ? 0.5f : -0.5f);
			C.PositionWorldB = C.PositionWorldA - Vector3(0, 0.1f, 0); C.PenetrationDepth = 0.1f;
		}
		SupportGraph G; G.Build({ Floor->mGeometries[0], Box->mGeometries[0] }, { &Contact }, {}, 0.001f);
		SupportStabilizationSettings S; S.Enabled = true; S.PositionIterations = S.VelocityIterations = 1;
		SupportStabilizationStats Stats; StabilizeSupports(G, S, FrameDt, Stats);
		EXPECT_NEAR(Box->X.y, 0.499f, 1e-5f); EXPECT_NEAR(Box->Q.w, 1, 1e-6f);
		EXPECT_NEAR(Box->V.y, Rebound ? 3 : 0, 1e-5f); EXPECT(Box->W.Length() < 1e-5f);
		EXPECT(Stats.ContactBlockFallbacks == 0 && Stats.MaximumLevel == 1);
		EXPECT_NEAR(Stats.MaximumLinearVelocityChange, Rebound ? 0 : 3, 1e-5f);
	}
	for (auto Solver : Solvers)
	for (bool Chain : { false, true })
	for (float Ratio : { 1.0f, 100.0f })
	{
		const auto Base = MeasureStability(Solver, 1, Chain, Ratio);
		const auto Support = MeasureStability(Solver, 1, Chain, Ratio, true);
		printf("Support solver=%d chain=%d ratio=%g error=%g -> %g speed=%g -> %g time_ms=%g -> %g\n",
			(int)Solver, Chain, Ratio, Base.Error, Support.Error, Base.Speed, Support.Speed, Base.Milliseconds, Support.Milliseconds);
		EXPECT(Support.Error < 0.02f); EXPECT(Support.Speed < 6);
		if (Ratio == 100) EXPECT(Support.Error < Base.Error / 2);
	}
	// Body order and insertion order do not encode the support hierarchy.
	PhysicsWorld World{ PhysicsWorldParam() };
	auto* Root = Body(World, Vector3::Zero(), false);
	auto* A = Body(World, Vector3(0, 1, 0)); auto* B = Body(World, Vector3(0, 2, 0));
	auto* C = Body(World, Vector3(10, 0, 0)); auto* D = Body(World, Vector3(10, 1, 0));
	auto* AB = World.CreateDistanceConstraint(B, A, B->X, A->X, 1, 1);
	auto* RA = World.CreateDistanceConstraint(A, Root, A->X, Root->X, 1, 1);
	auto* CD = World.CreateDistanceConstraint(C, D, C->X, D->X, 1, 1);
	SupportGraph Graph; Graph.Build({}, {}, { AB, CD, RA }, 0.001f);
	EXPECT(Graph.GetLevel(Root) == 0 && Graph.GetLevel(A) == 1 && Graph.GetLevel(B) == 2);
	EXPECT(Graph.GetLevel(C) == -1 && Graph.GetLevel(D) == -1 && Graph.GetEdges().size() == 2);
	C->CastDynamic()->Freezing = true; Graph.Build({}, {}, { CD }, 0.001f);
	EXPECT(Graph.GetLevel(C) == 0 && Graph.GetLevel(D) == 1); C->CastDynamic()->Freezing = false;
	RigidBodyKinematics Platform; Platform.X = Vector3(20, 0, 0); Platform.Q = Quaternion::One(); Platform.V = Vector3(2, 0, 0); Platform.W.SetZero();
	FixedConstraint PlatformJoint(&Platform, D); PlatformJoint.BeginStep(FrameDt, false);
	Graph.Build({}, {}, { &PlatformJoint }, 0.001f);
	SupportStabilizationSettings PlatformSettings; PlatformSettings.Enabled = true; SupportStabilizationStats PlatformStats;
	StabilizeSupports(Graph, PlatformSettings, FrameDt, PlatformStats);
	EXPECT(Graph.GetLevel(&Platform) == 0 && Graph.GetLevel(D) == 1);
	EXPECT((D->V - Platform.V).Length() < 1e-5f); EXPECT(Platform.V == Vector3(2, 0, 0));
	RA->SetCompliance(0.01f, 0); Graph.Build({}, {}, { AB, RA }, 0.001f); EXPECT(Graph.GetEdges().empty());
	RA->SetCompliance(0, 0); RA->SetEnabled(false); Graph.Build({}, {}, { AB, RA }, 0.001f); EXPECT(Graph.GetEdges().empty());

	// A conditioned hard row must leave mass, motor budgets and physical reports alone.
	B->InvMass = 0.01f; B->InvInertia = Matrix3(0.01f, 0.01f, 0.01f);
	PointConstraint Joint(A, B, Vector3(0, 1.5f, 0));
	JointBreakSettings Threshold; Threshold.Enabled = true; Threshold.Force = 150; Joint.SetBreakSettings(Threshold);
	Joint.BeginStep(FrameDt, false);
	A->V = Vector3::Zero(); B->V = Vector3(0, 10, 0); Joint.SolveVelocity(FrameDt);
	const float PhysicalForce = -10 / (1.01f * FrameDt);
	A->CastDynamic()->Sleep(); B->X.y += 0.1f;
	Joint.Stabilize(FrameDt, 0, 1, true); EXPECT(A->CastDynamic()->Sleeping);
	B->V = Vector3(0, 20, 0); Joint.Stabilize(FrameDt, 0, 1, false); EXPECT(A->CastDynamic()->Sleeping);
	EXPECT(!Joint.EndStep() && !Joint.IsBroken());
	EXPECT_NEAR(B->V.y, A->V.y, 1e-5f); EXPECT_NEAR(Joint.GetReaction().Force.y, PhysicalForce, 0.001f);
	EXPECT_NEAR(A->InvMass, 1, 1e-6f); EXPECT_NEAR(B->InvMass, 0.01f, 1e-6f);
	EXPECT_NEAR(B->InvInertia[0][0], 0.01f, 1e-6f);

	for (auto Solver : Solvers)
	for (auto Backend : { SolverBackend::CPU, SolverBackend::SIMD, SolverBackend::ComputeShader, SolverBackend::CUDA })
	for (int Count : { 1, 4 })
	{
		if (!PhysicsWorld::IsSolverBackendAvailable(Solver, Backend)) continue;
		auto P = Parameters(Solver, Count, Backend); P.supportStabilization.Enabled = true;
		PhysicsWorld Driven(P); auto* Static = Body(Driven, Vector3::Zero(), false); auto* Moving = Body(Driven, Vector3::Zero());
		auto* Slider = Driven.CreateSliderConstraint(Static, Moving, Vector3::Zero(), Vector3::UnitX(), -100, 100);
		ScalarJointDrive Drive; Drive.Settings.VelocityEnabled = true; Drive.Settings.Damping = 100;
		Drive.Settings.MaxForce = 6; Drive.TargetVelocity = 1000; Slider->SetDrive(Drive);
		Driven.Simulate(FrameDt);
		EXPECT_NEAR(Moving->V.x, 6 * FrameDt, 0.0001f); EXPECT_NEAR(Slider->GetReaction().Force.x, 6, 0.001f);
		EXPECT_NEAR(Slider->GetDriveImpulse(), 6 * FrameDt / Count, 0.0001f);
		EXPECT(Driven.GetLastSupportStabilizationStats().JointPasses > 0);
	}

	for (auto Solver : Solvers)
	for (auto Backend : { SolverBackend::CPU, SolverBackend::SIMD, SolverBackend::ComputeShader, SolverBackend::CUDA })
	{
		if (!PhysicsWorld::IsSolverBackendAvailable(Solver, Backend)) continue;
		for (int Count : { 1, 4 })
		for (bool Chain : { false, true })
		{
			const int Size = Chain ? 32 : 16;
			const auto Result = MeasureStability(Solver, Count, Chain, 1000, true, Backend, 4, Size);
			printf("Support backend solver=%d backend=%d substeps=%d chain=%d size=%d ratio=1000 error=%g speed=%g ms=%g\n",
				(int)Solver, (int)Backend, Count, Chain, Size, Result.Error, Result.Speed, Result.Milliseconds);
			printf("  heightError=%g drift=%g angularSpeed=%g blockFallbacks=%d\n", Result.HeightError, Result.Drift, Result.AngularSpeed, Result.BlockFallbacks);
			EXPECT(Result.Error < 0.05f); EXPECT(Result.Speed < 6);
			if (!Chain) { EXPECT(Result.HeightError < 0.1f); EXPECT(Result.Drift < 0.1f); }
		}
	}
	for (auto Solver : Solvers)
	{
		const auto ExtraIterations = MeasureStability(Solver, 1, false, 100, false, SolverBackend::CPU, 8);
		printf("Ordinary eight-sweep stack solver=%d ratio=100 error=%g speed=%g ms=%g\n",
			(int)Solver, ExtraIterations.Error, ExtraIterations.Speed, ExtraIterations.Milliseconds);
	}
	for (auto Solver : Solvers)
	for (int Count : { 1, 4 })
	{
		const auto Result = MeasureStability(Solver, Count, false, 1000, true, SolverBackend::CPU, 4, 16, 600);
		printf("Support long run solver=%d substeps=%d heightError=%g drift=%g speed=%g angularSpeed=%g\n",
			(int)Solver, Count, Result.HeightError, Result.Drift, Result.Speed, Result.AngularSpeed);
		EXPECT(Result.HeightError < 0.05f && Result.Drift < 0.1f && Result.Speed < 1);
	}
	// With no boundary, changing this option must not change the physical trajectory.
	// Compliance must also retain the ordinary XPBD spring response when rooted.
	for (bool Compliant : { false, true })
	{
		auto P = Parameters(ResolutionPhaseSolver::XPBD, 4); P.gravityAcc = Vector3(0, -10, 0);
		PhysicsWorld Reference(P); P.supportStabilization.Enabled = true; PhysicsWorld Stabilized(P);
		RigidBody* Ends[2]; int Index = 0;
		for (auto* W : { &Reference, &Stabilized })
		{
			auto* R = Body(*W, Vector3::Zero(), !Compliant); auto* E = Body(*W, Vector3(0, -1, 0));
			auto* J = W->CreateDistanceConstraint(R, E, R->X, E->X, 1, 1);
			if (Compliant) J->SetCompliance(0.005f, 0);
			E->V = Vector3(2, 0, 0); Ends[Index++] = E;
		}
		for (int Frame = 0; Frame < 120; ++Frame)
		{
			Reference.Simulate(1.0f / 60); Stabilized.Simulate(1.0f / 60);
			EXPECT((Ends[0]->X - Ends[1]->X).Length() < 1e-6f); EXPECT((Ends[0]->V - Ends[1]->V).Length() < 1e-6f);
			EXPECT(Stabilized.GetLastSupportStabilizationStats().JointPasses == 0);
		}
	}
	// Support can pass through a joint and then a contact, in either body order.
	for (bool Reverse : { false, true })
	{
		auto P = Parameters(ResolutionPhaseSolver::SequentialImpulse, 1); P.supportStabilization.Enabled = true;
		P.gravityAcc = Vector3(0, -10, 0); P.narrowphase = NarrowPhaseSolver::PCM; PhysicsWorld Mixed(P);
		auto* Fixed = Body(Mixed, Vector3(0, -1, 0), false);
		RigidBodyParam Dynamic; Dynamic.maxContactImpulse = std::numeric_limits<float>::max(); Dynamic.rigidType = RigidType::Dynamic; Dynamic.linearDamping = Dynamic.angularDamping = 1; Dynamic.sleepThreshold = 0;
		auto* Lower = Mixed.CreateRigidBody(GeometryFactory::CreateOBB(Vector3::Zero(), Vector3(0.5f)), Dynamic);
		auto* Upper = Mixed.CreateRigidBody(GeometryFactory::CreateOBB(Vector3(0, 1, 0), Vector3(0.5f)), Dynamic);
		Lower->InvMass = 1; Lower->InvInertia = Matrix3(6, 6, 6); Upper->InvMass = 0.001f; Upper->InvInertia = Matrix3(0.006f, 0.006f, 0.006f);
		Lower->SetRestitution(0); Upper->SetRestitution(0);
		Mixed.CreateFixedConstraint(Reverse ? Lower : Fixed, Reverse ? Fixed : Lower);
		for (int Frame = 0; Frame < 120; ++Frame) Mixed.Simulate(1.0f / 60);
		printf("Mixed reverse=%d lowerY=%g upper=(%g,%g,%g) speed=%g level=%d\n", Reverse, Lower->X.y, Upper->X.x, Upper->X.y, Upper->X.z, Upper->V.Length(), Mixed.GetLastSupportStabilizationStats().MaximumLevel);
		EXPECT_NEAR(Lower->X.y, 0, 0.01f); EXPECT_NEAR(Upper->X.y, 1, 0.01f);
		EXPECT(Mixed.GetLastSupportStabilizationStats().MaximumLevel == 2);
	}
}
