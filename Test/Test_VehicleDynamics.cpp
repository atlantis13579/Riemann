#include "Test.h"
#include "../Src/Modules/Vehicle/VehicleDynamics.h"
#include "../Src/RigidBodyDynamics/PhysicsWorld.h"
#include "../Src/Collision/GeometryQuery.h"
#include "../Src/CollisionPrimitive/TriangleMesh.h"
#include "../Src/CollisionPrimitive/HeightField3.h"
#include "../Viewer/SceneWorld.h"
#include <limits>
#include <stdexcept>

namespace
{
	using namespace Riemann;
	constexpr float Dt = 1.0f / 120.0f;
	constexpr float Mass = 1200, Gravity = 9.8f;
	PhysicsWorldParam Parameters(ResolutionPhaseSolver Solver = ResolutionPhaseSolver::SequentialImpulse, int Substeps = 1)
	{
		PhysicsWorldParam P; P.gravityAcc = Vector3(0, -Gravity, 0); P.resolutionSolver = Solver;
		P.temporalSubsteps.Enabled = true; P.temporalSubsteps.Substeps = Substeps;
		return P;
	}
	RigidBody* Road(PhysicsWorld& World, bool Dynamic = false)
	{
		RigidBodyParam P; P.rigidType = Dynamic ? RigidType::Dynamic : RigidType::Static;
		P.linearDamping = P.angularDamping = 1; P.sleepThreshold = 0;
		auto* B = World.CreateRigidBody(GeometryFactory::CreateOBB(Vector3(0, -0.5f, 0), Vector3(100, 0.5f, 100)), P);
		B->SetFriction(1, 1); B->SetRestitution(0);
		if (Dynamic) { B->InvMass = 1 / (Mass * 2); B->InvInertia = Matrix3(0, 0, 0); }
		return B;
	}
	RigidBodyDynamic* Chassis(PhysicsWorld& World, const Vector3& Position = Vector3(0, 0.55f, 0))
	{
		RigidBodyParam P; P.rigidType = RigidType::Dynamic; P.linearDamping = P.angularDamping = 1;
		P.sleepThreshold = 0; P.maxContactImpulse = std::numeric_limits<float>::max();
		auto* B = World.CreateRigidBody(GeometryFactory::CreateOBB(Position, Vector3(0.8f, 0.2f, 1.8f)), P)->CastDynamic();
		B->InvMass = 1 / Mass;
		B->InvInertia = Matrix3(1 / 1500.0f, 1 / 1800.0f, 1 / 600.0f);
		B->SetRestitution(0); return B;
	}
	VehicleDynamicsSettings Settings()
	{
		VehicleDynamicsSettings S; S.IdleGovernor = 0; S.SolverIterations = 32;
		for (int Side : { -1, 1 }) for (int End : { -1, 1 })
		{
			VehicleWheelSettings W; W.Attachment = Vector3((float)Side, 0, End * S.Wheelbase / 2);
			W.SteeringFactor = End > 0 ? 1.0f : 0.0f; W.Axle = End > 0 ? 1 : 0;
			W.HandbrakeTorque = End < 0 ? W.BrakeTorque : 0;
			S.Wheels.push_back(W);
		}
		return S;
	}
	void Step(PhysicsWorld& World, int Count, float StepDt = Dt) { for (int I = 0; I < Count; ++I) World.Simulate(StepDt); }
	VehicleControls Park() { VehicleControls C; C.Gear = 0; C.Brake = 1; return C; }
	void CheckFinite(VehicleDynamics* V)
	{
		const auto* B = V->GetChassis();
		EXPECT(std::isfinite(B->X.Length()) && std::isfinite(B->V.Length()) && std::isfinite(B->W.Length()));
		EXPECT_NEAR(B->Q.SquareLength(), 1, 0.001f);
		EXPECT(std::isfinite(V->GetEngineSpeed()));
		for (size_t I = 0; I < V->GetWheels().size(); ++I)
		{
			const auto& W = V->GetWheels()[I]; const auto& S = V->GetSettings().Wheels[I];
			EXPECT(std::isfinite(W.AngularSpeed) && std::isfinite(W.LongitudinalSlip) && std::isfinite(W.LateralSlip));
			EXPECT(W.SuspensionForce >= 0 && W.SuspensionForce <= S.MaximumSuspensionForce + 1);
			EXPECT(W.SuspensionLength >= 0 && W.SuspensionLength <= S.Travel);
			EXPECT_NEAR(V->GetWheelTransform(I).quat.SquareLength(), 1, 0.001f);
		}
	}
	void SuspensionAndDriving()
	{
		for (auto Solver : { ResolutionPhaseSolver::SequentialImpulse, ResolutionPhaseSolver::LCPGlobal, ResolutionPhaseSolver::PBD, ResolutionPhaseSolver::XPBD })
		for (int Substeps : { 1, 4 })
		{
			PhysicsWorld World(Parameters(Solver, Substeps)); Road(World);
			auto* B = Chassis(World); auto* V = World.CreateVehicle(B, Settings()); V->SetControls(Park());
			Step(World, 360); CheckFinite(V);
			float Load = 0;
			for (const auto& W : V->GetWheels()) { EXPECT(W.Grounded); Load += W.SuspensionForce; }
			const auto& S = V->GetSettings().Wheels[0];
			const float Equilibrium = S.Radius + S.RestLength - Mass * Gravity / (4 * S.SpringStiffness);
			printf("Vehicle solver=%d substeps=%d ride=%.5f load=%.3f\n", (int)Solver, Substeps, B->X.y, Load);
			EXPECT_NEAR(B->X.y, Equilibrium, 0.008f); EXPECT_NEAR(Load, Mass * Gravity, 100);
			VehicleControls C; C.Throttle = 0.8f; V->SetControls(C); Step(World, 240); CheckFinite(V);
			printf("  drive speed=%.4f distance=%.4f engine=%.4f\n", B->V.z, B->X.z, V->GetEngineSpeed());
			EXPECT(B->V.z > 2); EXPECT(B->X.z > 1);
			C.Throttle = 0; C.Gear = 0; C.Brake = 1; V->SetControls(C); Step(World, 240);
			EXPECT(B->V.Length() < 0.15f); CheckFinite(V);
			C.Gear = -1; C.Brake = 0; C.Throttle = 0.8f; V->SetControls(C); Step(World, 240);
			EXPECT(B->V.z < -1); CheckFinite(V);
		}
	}
	void SteeringAndFriction()
	{
		for (float Mu : { 0.0f, 1.0f })
		{
			PhysicsWorld World(Parameters()); auto* Ground = Road(World); Ground->SetFriction(Mu, Mu);
			auto* B = Chassis(World); auto* V = World.CreateVehicle(B, Settings()); V->SetControls(Park()); Step(World, 240);
			VehicleControls C; C.Throttle = 0.7f; C.Steering = 0.5f; V->SetControls(C); Step(World, 240);
			printf("Vehicle friction=%.2f speed=%.4f yaw=%.4f x=%.4f\n", Mu, B->V.Length(), B->W.y, B->X.x);
			EXPECT(V->GetWheels()[3].SteeringAngle > V->GetWheels()[1].SteeringAngle);
			if (Mu == 0) { EXPECT(B->V.Length() < 0.05f); EXPECT(B->X.Length() < 1); }
			else { EXPECT(B->X.x > 0.5f); EXPECT(B->W.y > 0.05f); }
			for (size_t I = 0; I < 4; ++I)
			{
				const auto& W = V->GetWheels()[I];
				EXPECT(std::hypot(W.LongitudinalForce, W.LateralForce) <= Mu * W.SuspensionForce + 1);
			}
			CheckFinite(V);
		}
	}
	void DrivetrainAndDifferentials()
	{
		float Differences[3]{};
		for (int Mode = 0; Mode < 3; ++Mode)
		{
			PhysicsWorldParam P = Parameters(); P.gravityAcc = Vector3::Zero(); PhysicsWorld World(P);
			auto S = Settings(); S.Differential = (VehicleDifferential)Mode;
			S.DifferentialStrength = 100; S.MaximumDifferentialTorque = 1500;
			// Unequal rotor inertias expose torque splitting without road/contact noise.
			S.Wheels[0].Inertia *= 4; S.Wheels[1].Inertia *= 4;
			auto* V = World.CreateVehicle(Chassis(World, Vector3(0, 20, 0)), S);
			VehicleControls C; C.Throttle = 1; V->SetControls(C); Step(World, 120);
			Differences[Mode] = std::abs(V->GetWheels()[0].AngularSpeed - V->GetWheels()[2].AngularSpeed);
			printf("Vehicle differential=%d rotor difference=%.6f\n", Mode, Differences[Mode]);
			CheckFinite(V); EXPECT(!V->GetWheels()[0].Grounded);
			C.Gear = 2; V->SetControls(C); World.Simulate(Dt); EXPECT_NEAR(V->GetClutchTorque(), 0, 1e-5f);
			Step(World, 60); EXPECT(std::abs(V->GetClutchTorque()) > 0);
			C.Clutch = 0; V->SetControls(C); World.Simulate(Dt); EXPECT_NEAR(V->GetClutchTorque(), 0, 1e-5f);
			C.Throttle = 0; C.Brake = 1; V->SetControls(C); Step(World, 120);
			for (const auto& W : V->GetWheels()) EXPECT_NEAR(W.AngularSpeed, 0, 1e-5f);
		}
		EXPECT(Differences[0] > 1); EXPECT(Differences[1] < Differences[0]); EXPECT(Differences[2] < 1e-4f);
	}
	void DynamicRoadAndSleep()
	{
		PhysicsWorldParam P = Parameters(); P.gravityAcc = Vector3::Zero(); PhysicsWorld World(P);
		auto* Ground = Road(World, true); auto* B = Chassis(World);
		auto* V = World.CreateVehicle(B, Settings()); V->SetControls(Park()); World.Simulate(Dt);
		const Vector3 Momentum = B->V * Mass + Ground->V * (Mass * 2);
		EXPECT(Momentum.Length() < 0.1f); EXPECT(B->V.y > 0); EXPECT(Ground->V.y < 0);
		Ground->V = Ground->W = Vector3::Zero(); Ground->CastDynamic()->Freeze();
		World.Simulate(Dt); EXPECT_SAME(Ground->V, Vector3::Zero()); EXPECT_SAME(Ground->W, Vector3::Zero());
		B->Freeze(); const Vector3 FrozenPosition = B->X, FrozenVelocity = B->V;
		VehicleControls Drive; Drive.Throttle = 1; V->SetControls(Drive); Step(World, 5);
		EXPECT_SAME(B->X, FrozenPosition); EXPECT_SAME(B->V, FrozenVelocity);
		for (bool Island : { false, true })
		{
			P = Parameters(); P.islandSleeping.Enabled = Island; PhysicsWorld Sleeping(P);
			Ground = Road(Sleeping); B = Chassis(Sleeping); B->SleepThreshold = 0.01f;
			V = Sleeping.CreateVehicle(B, Settings()); V->SetControls(Park()); Step(Sleeping, 600);
			printf("Vehicle sleeping mode=%d asleep=%d y=%.4f\n", Island, B->Sleeping, B->X.y);
			EXPECT(B->Sleeping);
			Ground->X.x += 0.1f; Sleeping.Simulate(Dt); EXPECT(!B->Sleeping);
			VehicleControls C; C.Throttle = 1; V->SetControls(C); Step(Sleeping, 60); EXPECT(B->V.z > 0.2f);
			EXPECT(Sleeping.RemoveRigidBody(Ground)); Ground->ReleaseGeometries(); delete Ground;
			Step(Sleeping, 10); for (const auto& W : V->GetWheels()) EXPECT(!W.Grounded);
			CheckFinite(V);
		}
	}
	void OwnershipAndValidation()
	{
		PhysicsWorld World(Parameters()); auto* B = Chassis(World); auto S = Settings();
		auto Reject = [&](const VehicleDynamicsSettings& Bad) { bool Threw = false; try { World.CreateVehicle(B, Bad); } catch (const std::invalid_argument&) { Threw = true; } EXPECT(Threw); };
		auto Bad = S; Bad.Wheels.clear(); Reject(Bad); Bad = S; Bad.Wheels[0].Radius = 0; Reject(Bad);
		Bad = S; Bad.Up = Bad.Forward; Reject(Bad); Bad = S; Bad.EngineInertia = 0; Reject(Bad);
		Bad = S; Bad.ForwardGears[0] = std::numeric_limits<float>::infinity(); Reject(Bad);
		Bad = S; Bad.Wheels[0].Attachment.x = std::numeric_limits<float>::quiet_NaN(); Reject(Bad);
		auto* V = World.CreateVehicle(B, S); EXPECT(World.GetVehicleCount() == 1); Reject(S);
		VehicleControls C; C.Throttle = 2; bool Threw = false; try { V->SetControls(C); } catch (const std::invalid_argument&) { Threw = true; } EXPECT(Threw);
		EXPECT(World.RemoveRigidBody(B)); EXPECT(World.GetVehicleCount() == 0); B->ReleaseGeometries(); delete B;
		B = Chassis(World); V = World.CreateVehicle(B, S); EXPECT(World.RemoveVehicle(V)); EXPECT(!World.RemoveVehicle(V));
		World.CreateVehicle(B, S); World.Reset(); EXPECT(World.GetVehicleCount() == 0);
		S.Up = Vector3::UnitZ(); S.Forward = Vector3::UnitX();
		V = World.CreateVehicle(Chassis(World), S);
		const auto Frame = V->GetWheelTransform(0); EXPECT_SAME(Frame.quat * Vector3::UnitX(), Vector3::UnitY());
		EXPECT_SAME(Frame.quat * Vector3::UnitZ(), Vector3::UnitX());
	}
	void SplitTraction()
	{
		float Speeds[3]{};
		for (int Mode = 0; Mode < 3; ++Mode)
		{
			PhysicsWorld World(Parameters());
			for (float Side : { -1.0f, 1.0f })
			{
				auto* Ground = World.CreateRigidBody(GeometryFactory::CreateOBB(Vector3(Side * 50, -0.5f, 0), Vector3(50, 0.5f, 100)), RigidBodyParam());
				Ground->SetFriction(Side < 0 ? 1.0f : 0.0f, Side < 0 ? 1.0f : 0.0f);
			}
			auto S = Settings(); S.Differential = (VehicleDifferential)Mode; S.DifferentialStrength = 100;
			auto* B = Chassis(World); B->InvInertia = Matrix3(0, 0, 0);
			auto* V = World.CreateVehicle(B, S); V->SetControls(Park()); Step(World, 240);
			VehicleControls C; C.Throttle = 1; V->SetControls(C); Step(World, 240);
			Speeds[Mode] = B->V.z;
			printf("Vehicle split traction mode=%d speed=%.4f rotor delta=%.4f\n", Mode, B->V.z, V->GetWheels()[0].AngularSpeed - V->GetWheels()[2].AngularSpeed);
			if (Mode == 2) EXPECT_NEAR(V->GetWheels()[0].AngularSpeed, V->GetWheels()[2].AngularSpeed, 1e-4f);
			CheckFinite(V);
		}
		EXPECT(Speeds[1] > Speeds[0] * 2); EXPECT(Speeds[2] > Speeds[0] * 2);
	}
	void SurfacesAndFilters()
	{
		for (int Kind = 0; Kind < 3; ++Kind)
		{
			PhysicsWorld World(Parameters()); Geometry* Shape;
			if (Kind == 0) Shape = GeometryFactory::CreatePlane(Vector3::Zero(), Vector3::UnitY());
			else if (Kind == 1)
			{
				Shape = GeometryFactory::CreateTriangleMesh(); auto* M = Shape->GetShapeObj<TriangleMesh>();
				for (auto P : { Vector3(-4, 0, -4), Vector3(-4, 0, 4), Vector3(4, 0, 4), Vector3(4, 0, -4) }) M->AddVertex(P);
				M->AddTriangle(0, 1, 2); M->AddTriangle(0, 2, 3); M->Compact(); M->BuildBVH(); Shape->UpdateVolumeProperties();
			}
			else
			{
				Shape = GeometryFactory::CreateHeightField(Box3(Vector3(-4, -1, -4), Vector3(4, 1, 4)), 8, 8);
				auto* H = Shape->GetShapeObj<HeightField3>(); H->HeightScale = 0.001f; H->AllocMemory();
				for (int Z = 0; Z < 8; ++Z) for (int X = 0; X < 8; ++X) EXPECT(H->SetHeight(X, Z, 0));
				Shape->UpdateVolumeProperties();
			}
			World.CreateRigidBody(Shape, RigidBodyParam());
			auto* V = World.CreateVehicle(Chassis(World), Settings()); V->SetControls(Park()); Step(World, 240);
			for (const auto& W : V->GetWheels()) EXPECT(W.Grounded);
			EXPECT_NEAR(V->GetChassis()->X.y, 0.552f, 0.01f); CheckFinite(V);
		}
		for (auto Tree : { SceneQueryType::StaticAABB, SceneQueryType::DynamicAABB })
		{
			auto P = Parameters(); P.sceneQueryType = Tree; P.collisionFilter.reset(CollisionFilter::CreateLayerMask()); PhysicsWorld World(P);
			auto* Ground = Road(World); auto* B = Chassis(World);
			// Trigger, query-only and rejected-layer surfaces lie above the true road.
			for (int Kind = 0; Kind < 3; ++Kind)
			{
				auto* Fake = World.CreateRigidBody(GeometryFactory::CreateOBB(Vector3(0, 0.05f * (Kind + 1), 0), Vector3(3, 0.02f, 3)), RigidBodyParam());
				auto* G = Fake->mGeometries[0];
				if (Kind == 0) G->SetTriggerEnabled(true); else if (Kind == 1) G->SetSimulationEnabled(false); else G->SetFilterData(CollisionData::ForLayer(2));
			}
			auto S = Settings(); S.FilterData = CollisionData::ForLayer(1, 1u);
			auto* V = World.CreateVehicle(B, S); V->SetControls(Park()); World.Simulate(Dt);
			for (const auto& W : V->GetWheels()) { EXPECT(W.Grounded); EXPECT_NEAR(W.ContactPoint.y, 0, 0.002f); }
			Ground->mGeometries[0]->SetQueryEnabled(false); World.Simulate(Dt);
			for (const auto& W : V->GetWheels()) EXPECT(!W.Grounded);
			V->SetEnabled(false); EXPECT(!V->IsEnabled()); Step(World, 5); CheckFinite(V);
			Ground->mGeometries[0]->SetQueryEnabled(true); V->SetEnabled(true); World.Simulate(Dt);
			for (const auto& W : V->GetWheels()) EXPECT(W.Grounded);
		}
	}
	void DemoIntegration()
	{
		SceneWorld Scene; Scene.SetVehicleDynamicsDemo(true); std::string Error;
		const auto File = TestContentsPath() + "Scenes/19_vehicle_deformation.json";
		for (int Reload = 0; Reload < 2; ++Reload)
		{
			EXPECT(Scene.LoadFromFile(File, &Error)); auto* V = Scene.GetVehicleDynamics(0);
			EXPECT(V != nullptr); if (!V) return;
			EXPECT(Scene.GetSimulation()->GetVehicleCount() == 1);
			for (int I = 0; I < 180; ++I) Scene.Step(Dt);
			for (const auto& W : V->GetWheels()) EXPECT(W.Grounded);
			const float Start = V->GetChassis()->X.z;
			VehicleControls C; C.Throttle = 0.7f; V->SetControls(C);
			for (int I = 0; I < 120; ++I) Scene.Step(Dt);
			printf("Vehicle demo distance=%.3f speed=%.3f height=%.3f\n", V->GetChassis()->X.z - Start, V->GetChassis()->V.z, V->GetChassis()->X.y);
			EXPECT(V->GetChassis()->X.z > Start + 0.3f); CheckFinite(V);
			for (size_t I = 0; I < 4; ++I)
				for (const auto& Object : Scene.GetObjects()) if (Object.Id == "crash_car_wheel_" + std::to_string(I))
					EXPECT_SAME(Object.GeometryPtr->GetTransform()->pos, V->GetWheelTransform(I).pos);
		}
		Scene.SetVehicleDynamicsDemo(false); EXPECT(Scene.LoadFromFile(File, &Error)); EXPECT(Scene.GetVehicleDynamics(0) == nullptr);
		EXPECT(Scene.GetSimulation()->GetVehicleCount() == 0);
	}
	void SubstepsAndPassiveRotors()
	{
		PhysicsWorld Automatic(Parameters(ResolutionPhaseSolver::SequentialImpulse, 4)), Manual(Parameters());
		Road(Automatic); Road(Manual);
		auto* A = Automatic.CreateVehicle(Chassis(Automatic), Settings()); auto* B = Manual.CreateVehicle(Chassis(Manual), Settings());
		VehicleControls C; C.Throttle = 0.7f; A->SetControls(C); B->SetControls(C);
		for (int I = 0; I < 60; ++I) { Automatic.Simulate(Dt * 4); Step(Manual, 4); }
		EXPECT_NEAR((A->GetChassis()->X - B->GetChassis()->X).Length(), 0, 1e-4f);
		EXPECT_NEAR((A->GetChassis()->V - B->GetChassis()->V).Length(), 0, 1e-4f);
		EXPECT_NEAR(A->GetEngineSpeed(), B->GetEngineSpeed(), 1e-4f);
		// A disconnected trailer must not silently dissipate engine energy through
		// an imaginary shaft. A connected passive clutch may only lose energy.
		for (bool Driven : { false, true })
		{
			auto P = Parameters(); P.gravityAcc = Vector3::Zero(); PhysicsWorld World(P);
			auto S = Settings(); S.EngineDamping = 0; S.IdleGovernor = 0;
			for (auto& W : S.Wheels) { W.DriveWeight = Driven ? 1.0f : 0.0f; W.AngularDamping = 0; }
			auto* V = World.CreateVehicle(Chassis(World, Vector3(0, 20, 0)), S);
			float PreviousEnergy = S.EngineInertia * S.IdleEngineSpeed * S.IdleEngineSpeed / 2;
			for (int I = 0; I < 120; ++I)
			{
				World.Simulate(Dt); float Energy = S.EngineInertia * V->GetEngineSpeed() * V->GetEngineSpeed() / 2;
				for (size_t Wheel = 0; Wheel < 4; ++Wheel) Energy += S.Wheels[Wheel].Inertia * V->GetWheels()[Wheel].AngularSpeed * V->GetWheels()[Wheel].AngularSpeed / 2;
				EXPECT(Energy <= PreviousEnergy + 0.001f); PreviousEnergy = Energy;
			}
			if (!Driven) { EXPECT_NEAR(V->GetEngineSpeed(), S.IdleEngineSpeed, 1e-5f); EXPECT_NEAR(V->GetClutchTorque(), 0, 1e-5f); }
		}
	}
}

void TestVehicleDynamics()
{
	printf("Running TestVehicleDynamics\n");
	OwnershipAndValidation(); SuspensionAndDriving(); SteeringAndFriction(); DrivetrainAndDifferentials(); DynamicRoadAndSleep();
	SplitTraction(); SurfacesAndFilters(); DemoIntegration();
	SubstepsAndPassiveRotors();
}
