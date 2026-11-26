#include "Test.h"
#include "../Src/Modules/Vehicle/VehicleDynamics.h"
#include "../Src/RigidBodyDynamics/PhysicsWorld.h"
#include "../Src/Collision/GeometryQuery.h"
#include "../Src/CollisionPrimitive/TriangleMesh.h"
#include "../Src/CollisionPrimitive/HeightField3.h"
#include <limits>
#include <stdexcept>
#include <algorithm>
#include <map>
#include <memory>
#include "../Src/Collision/GeometryObject.h"
#include "../Src/Modules/Vehicle/VehicleDeformation.h"

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
		auto S = VehicleDynamicsSettings::Roadster();
		S.IdleGovernor = 0;
		S.SolverIterations = 32;
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
			EXPECT(V->GetWheels()[1].SteeringAngle > V->GetWheels()[0].SteeringAngle);
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
			S.Wheels[2].Inertia *= 4;
			auto* V = World.CreateVehicle(Chassis(World, Vector3(0, 20, 0)), S);
			VehicleControls C; C.Throttle = 1; V->SetControls(C); Step(World, 120);
			Differences[Mode] = std::abs(V->GetWheels()[2].AngularSpeed - V->GetWheels()[3].AngularSpeed);
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
	void RoadsterLoadTransfer()
	{
		auto Car = VehicleDynamicsSettings::Roadster();
		EXPECT(Car.IsValid());
		PhysicsWorld World(Parameters());
		const Vector3 Normal = Vector3(0.06f, 1.0f, -0.08f).Unit();
		auto* Pavement = World.CreateRigidBody(GeometryFactory::CreatePlane(Vector3::Zero(), Normal), RigidBodyParam());
		Pavement->SetFriction(1, 1);
		auto* B = Chassis(World);
		auto* V = World.CreateVehicle(B, Car);
		V->SetControls(Park());
		Step(World, 180);
		VehicleControls C;
		C.Throttle = 0.85f;
		V->SetControls(C);
		Step(World, 240);
		EXPECT(B->V.z > 3 && B->V.y > 0);
		const auto& Wheels = V->GetWheels();
		auto FrontLoad = [&]() { return Wheels[0].SuspensionForce + Wheels[1].SuspensionForce; };
		const float AcceleratingLoad = FrontLoad();
		const float InitialCrossLoad = Wheels[0].SuspensionForce - Wheels[1].SuspensionForce;
		C.Throttle = 0; C.Gear = 0; C.Brake = 0.8f; C.Steering = 0.3f;
		V->SetControls(C);
		Step(World, 30);
		EXPECT(FrontLoad() > AcceleratingLoad + 200);
		const float CorneringCrossLoad = Wheels[0].SuspensionForce - Wheels[1].SuspensionForce;
		printf("Roadster front load %.1f -> %.1f N, cross load %.1f -> %.1f N\n",
			AcceleratingLoad, FrontLoad(), InitialCrossLoad, CorneringCrossLoad);
		EXPECT(CorneringCrossLoad > InitialCrossLoad + 50);
		for (const auto& Wheel : Wheels)
		{
			EXPECT(Wheel.Grounded && Wheel.ContactNormal.Dot(Normal) > 0.99f);
			EXPECT(std::hypot(Wheel.LongitudinalForce, Wheel.LateralForce) <= Wheel.SuspensionForce + 1);
		}
		CheckFinite(V);
	}

	void RoadsterAeroAndAntiRoll()
	{
		auto Params = Parameters(); Params.gravityAcc = Vector3::Zero();
		auto Car = VehicleDynamicsSettings::Roadster();
		PhysicsWorld Air(Params);
		auto* B = Chassis(Air, Vector3(0, 10, 0));
		auto* V = Air.CreateVehicle(B, Car);
		B->V = Vector3(0, 0, 40);
		Air.Simulate(Dt);
		const float Pressure = 0.5f * Car.AirDensity * 40 * 40;
		EXPECT_NEAR(B->V.z, 40 - Pressure * Car.DragArea * Dt / Mass, 1e-5f);
		EXPECT_NEAR(B->V.y, -Pressure * Car.DownforceArea * Dt / Mass, 1e-5f);
		for (const auto& Wheel : V->GetWheels()) EXPECT(!Wheel.Grounded && Wheel.SuspensionForce == 0);

		// Raise the road under the left wheels with the same initial chassis pose.
		float LoadDifference[2];
		for (int WithBar = 0; WithBar <= 1; ++WithBar)
		{
			PhysicsWorld World(Params);
			Road(World);
			World.CreateRigidBody(GeometryFactory::CreateOBB(Vector3(-1, -0.45f, 0), Vector3(0.6f, 0.5f, 4)), RigidBodyParam());
			Car.AntiRollStiffness = WithBar * 9000.0f;
			auto* OnRoad = World.CreateVehicle(Chassis(World), Car);
			OnRoad->SetControls(Park());
			World.Simulate(Dt);
			const auto& Wheels = OnRoad->GetWheels();
			EXPECT(Wheels[0].SuspensionLength < Wheels[1].SuspensionLength);
			LoadDifference[WithBar] = Wheels[0].SuspensionForce - Wheels[1].SuspensionForce;
		}
		EXPECT(LoadDifference[1] > LoadDifference[0] + 100);
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
			printf("Vehicle split traction mode=%d speed=%.4f rotor delta=%.4f\n", Mode, B->V.z, V->GetWheels()[2].AngularSpeed - V->GetWheels()[3].AngularSpeed);
			if (Mode == 2) EXPECT_NEAR(V->GetWheels()[2].AngularSpeed, V->GetWheels()[3].AngularSpeed, 1e-4f);
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
	SplitTraction(); SurfacesAndFilters();
	RoadsterLoadTransfer(); RoadsterAeroAndAntiRoll();
	SubstepsAndPassiveRotors();
}

namespace Test_VehicleDeformation_Cases
{
using namespace Riemann;

namespace
{
	constexpr float VehicleStep = 1.0f / 60.0f;

	bool VehicleFinite(const Vector3& value)
	{
		return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
	}

	struct VehicleFixture
	{
		PhysicsWorld World{ PhysicsWorldParam() };
		RigidBody* Body;
		VehicleDeformation Vehicle;

		explicit VehicleFixture(const VehicleDeformationSettings& settings = VehicleDeformationSettings())
			: Vehicle(settings)
		{
			RigidBodyParam bodySettings;
			bodySettings.rigidType = RigidType::Dynamic;
			bodySettings.disableGravity = true;
			Body = World.CreateRigidBody(GeometryFactory::CreateOBB(Vector3::Zero(), settings.HalfExtent), bodySettings);
			EXPECT(Vehicle.Attach(Body));
		}
	};

	Vector3 FrontPoint(const VehicleDeformation& vehicle)
	{
		return Vector3(0.0f, 0.2f, vehicle.GetSettings().HalfExtent.z);
	}

	void ImpactFront(VehicleDeformation& vehicle, float impulse)
	{
		const Transform world = vehicle.GetWorldTransform();
		EXPECT(vehicle.QueueImpact(world.LocalToWorld(FrontPoint(vehicle)),
			world.LocalToWorldDirection(Vector3(0, 0, -impulse))));
	}

	float FrontDent(const VehicleDeformation& vehicle)
	{
		float dent = 0.0f;
		for (const DeformableParticle& particle : vehicle.GetCage().GetParticles())
		{
			if (particle.InitialPosition.z > vehicle.GetSettings().HalfExtent.z - 1.0e-4f)
				dent = std::max(dent, particle.InitialPosition.z - particle.Position.z);
		}
		return dent;
	}

	void CompareCages(const VehicleDeformation& a, const VehicleDeformation& b, float tolerance)
	{
		EXPECT(a.GetCage().GetParticleCount() == b.GetCage().GetParticleCount());
		if (a.GetCage().GetParticleCount() != b.GetCage().GetParticleCount()) return;
		for (int i = 0; i < a.GetCage().GetParticleCount(); ++i)
		{
			EXPECT((a.GetCage().GetParticle(i).Position - b.GetCage().GetParticle(i).Position).Length() < tolerance);
			EXPECT((a.GetCage().GetParticle(i).Velocity - b.GetCage().GetParticle(i).Velocity).Length() < tolerance * 100.0f);
		}
	}

	void CheckVehicleFinite(const VehicleDeformation& vehicle)
	{
		for (const DeformableParticle& particle : vehicle.GetCage().GetParticles())
		{
			EXPECT(VehicleFinite(particle.Position));
			EXPECT(VehicleFinite(particle.Velocity));
			if (particle.Pinned)
			{
				EXPECT_SAME(particle.Position, particle.InitialPosition);
				EXPECT_SAME(particle.Velocity, Vector3::Zero());
			}
		}
		for (int i = 0; i < (int)vehicle.GetCage().GetTetrahedra().size(); ++i)
			EXPECT(std::isfinite(vehicle.GetCage().GetTetrahedronVolume(i)));
	}

	void TestVehicleCageTopology()
	{
		for (float cabin : { 0.0f, 0.45f })
		{
			VehicleDeformationSettings settings;
			settings.CabinHeight = cabin;
			VehicleFixture fixture(settings);
			const DeformableBody& cage = fixture.Vehicle.GetCage();
			EXPECT(cage.GetParticleCount() == 135);
			EXPECT(cage.GetTetrahedra().size() == 384);
			EXPECT(cage.GetSurfaceTriangles().size() == 224);
			int pinned = 0;
			for (const DeformableParticle& particle : cage.GetParticles()) pinned += particle.Pinned ? 1 : 0;
			EXPECT(pinned == 9);
			double volume = 0.0, surfaceVolume = 0.0;
			for (int i = 0; i < (int)cage.GetTetrahedra().size(); ++i)
			{
				EXPECT(cage.GetTetrahedronVolume(i) > 0.0f);
				volume += cage.GetTetrahedronVolume(i);
			}
			std::map<std::pair<int, int>, std::pair<int, int>> edges;
			for (const Index3& triangle : cage.GetSurfaceTriangles())
			{
				const Vector3 a = cage.GetParticle(triangle.a).Position;
				const Vector3 b = cage.GetParticle(triangle.b).Position;
				const Vector3 c = cage.GetParticle(triangle.c).Position;
				EXPECT((b - a).Cross(c - a).SquareLength() > 1.0e-10f);
				surfaceVolume += double(a.Dot(b.Cross(c))) / 6.0;
				for (int side = 0; side < 3; ++side)
				{
					const int from = triangle[side], to = triangle[(side + 1) % 3];
					auto& entry = edges[{ std::min(from, to), std::max(from, to) }];
					++entry.first;
					entry.second += from < to ? 1 : -1;
				}
			}
			for (const auto& edge : edges) { EXPECT(edge.second.first == 2); EXPECT(edge.second.second == 0); }
			EXPECT_NEAR(volume, surfaceVolume, 1.0e-4);
			if (cabin == 0.0f) EXPECT_NEAR(volume, 8.0f * settings.HalfExtent.x * settings.HalfExtent.y * settings.HalfExtent.z, 1.0e-4);
			Geometry* surface = fixture.Vehicle.GetRenderGeometry();
			EXPECT(surface != nullptr);
			if (!surface) continue;
			EXPECT(surface->GetParent<RigidBody>() == nullptr);
			EXPECT_FALSE(surface->IsSimulationEnabled());
			EXPECT_FALSE(surface->IsQueryEnabled());
			const TriangleMesh* mesh = surface->GetShapeObj<TriangleMesh>();
			EXPECT(mesh->NumVertices == 135 && mesh->NumTriangles == 224);
			EXPECT_NEAR(mesh->BoundingVolume.Min.y, -settings.HalfExtent.y, 1.0e-6f);
			EXPECT_NEAR(mesh->BoundingVolume.Max.y, settings.HalfExtent.y + cabin, 1.0e-6f);
		}
	}

	void TestVehicleFrameAndContactSigns()
	{
		VehicleFixture direct, contactA, contactB;
		const Vector3 centerOfMass(0.2f, -0.1f, 0.3f);
		Quaternion localRotation, worldRotation;
		localRotation.FromRotationAxis(Vector3::UnitZ(), 0.2f);
		worldRotation.FromRotationAxis(Vector3::UnitY(), 1.57079632679f);
		const Transform localFrame(Vector3(0.3f, 0.1f, -0.2f), localRotation);
		for (VehicleFixture* fixture : { &direct, &contactA, &contactB })
		{
			fixture->Body->CM = centerOfMass;
			EXPECT(fixture->Vehicle.Attach(fixture->Body, localFrame));
		}
		contactB.Body->X = Vector3(17, 3, -9);
		contactB.Body->Q = worldRotation;
		ImpactFront(direct.Vehicle, 400.0f);
		for (VehicleFixture* fixture : { &contactA, &contactB })
		{
			const bool isA = fixture == &contactA;
			const Transform world = fixture->Vehicle.GetWorldTransform();
			PhysicsContactPoint point{};
			point.PositionWorldA = isA ? world.LocalToWorld(FrontPoint(fixture->Vehicle)) : Vector3(1000.0f);
			point.PositionWorldB = !isA ? world.LocalToWorld(FrontPoint(fixture->Vehicle)) : Vector3(1000.0f);
			point.PositionLocalA = point.PositionLocalB = Vector3(-1000.0f);
			point.Impulse = world.LocalToWorldDirection(Vector3(0, 0, isA ? 400.0f : -400.0f));
			PhysicsContactInfo info{};
			info.BodyA = isA ? fixture->Body : nullptr;
			info.BodyB = isA ? nullptr : fixture->Body;
			info.ContactPoints = &point;
			info.NumContactPoints = 1;
			fixture->Vehicle.OnContact(info);
			EXPECT(fixture->Vehicle.GetPendingImpactCount() == 1);
		}
		// Move the chassis after callback delivery. The queued impact must retain its local location.
		contactB.Body->X += Vector3(2, 1, -3);
		contactB.Body->Q = (worldRotation * worldRotation).Unit();
		for (int step = 0; step < 30; ++step)
		{
			direct.Vehicle.Simulate(VehicleStep);
			contactA.Vehicle.Simulate(VehicleStep);
			contactB.Vehicle.Simulate(VehicleStep);
			CompareCages(direct.Vehicle, contactA.Vehicle, 2.0e-4f);
			CompareCages(direct.Vehicle, contactB.Vehicle, 2.0e-4f);
		}
		EXPECT(FrontDent(direct.Vehicle) > 0.005f);
		const Transform actual = contactB.Vehicle.GetWorldTransform();
		const Vector3 expectedPosition = contactB.Body->X + contactB.Body->Q * (localFrame.pos - centerOfMass);
		EXPECT_SAME(actual.pos, expectedPosition);
		EXPECT_NEAR(std::abs(actual.quat.Dot(contactB.Body->Q * localRotation)), 1.0f, 1.0e-5f);
		EXPECT_SAME(contactB.Vehicle.GetRenderGeometry()->GetPosition(), actual.pos);
		EXPECT_NEAR(std::abs(contactB.Vehicle.GetRenderGeometry()->GetRotation().Dot(actual.quat)), 1.0f, 1.0e-5f);
		EXPECT_SAME(contactB.Body->V, Vector3::Zero());
		EXPECT_SAME(contactB.Body->W, Vector3::Zero());

		VehicleFixture moving;
		const uint64_t revision = moving.Vehicle.GetRevision();
		for (int frame = 0; frame < 60; ++frame)
		{
			moving.Body->X = Vector3(0.1f * frame, 3.0f, -2.0f);
			moving.Body->Q.FromRotationAxis(Vector3::UnitY(), frame * 0.03f);
			moving.Vehicle.Simulate(VehicleStep);
		}
		EXPECT(moving.Vehicle.GetMaxDisplacement() < 1.0e-5f);
		EXPECT(moving.Vehicle.GetMaxPlasticStrain() == 0.0f);
		EXPECT(moving.Vehicle.GetRevision() > revision);
	}

	void TestVehicleElasticPlasticAndReset()
	{
		VehicleFixture fixture;
		VehicleDeformation& vehicle = fixture.Vehicle;
		Geometry* surface = vehicle.GetRenderGeometry();
		ImpactFront(vehicle, 30.0f);
		float weakPeak = 0.0f;
		for (int step = 0; step < 180; ++step)
		{
			vehicle.Simulate(VehicleStep);
			weakPeak = std::max(weakPeak, vehicle.GetMaxDisplacement());
		}
		printf("  Vehicle elastic: peak=%.6f remaining=%.6f plastic=%.6f\n", weakPeak, vehicle.GetMaxDisplacement(), vehicle.GetMaxPlasticStrain());
		EXPECT(weakPeak > 0.001f);
		EXPECT(vehicle.GetMaxDisplacement() < 0.001f);
		EXPECT(vehicle.GetMaxPlasticStrain() < 1.0e-5f);
		vehicle.Reset();
		ImpactFront(vehicle, 400.0f);
		float strongPeak = 0.0f;
		for (int step = 0; step < 180; ++step)
		{
			vehicle.Simulate(VehicleStep);
			strongPeak = std::max(strongPeak, FrontDent(vehicle));
		}
		printf("  Vehicle plastic: peak dent=%.6f remaining=%.6f plastic=%.6f\n", strongPeak, FrontDent(vehicle), vehicle.GetMaxPlasticStrain());
		EXPECT(strongPeak > 0.02f);
		EXPECT(FrontDent(vehicle) > 0.01f);
		EXPECT(vehicle.GetMaxPlasticStrain() > 1.0e-4f);
		for (int hit = 0; hit < 6; ++hit)
		{
			ImpactFront(vehicle, 400.0f);
			for (int step = 0; step < 60; ++step) vehicle.Simulate(VehicleStep);
			CheckVehicleFinite(vehicle);
		}
		EXPECT(vehicle.GetMaxPlasticStrain() <= vehicle.GetSettings().Material.MaxPlasticStrain + 1.0e-5f);
		EXPECT(vehicle.GetMaxDisplacement() < 1.0f);
		for (int i = 0; i < (int)vehicle.GetCage().GetTetrahedra().size(); ++i) EXPECT(vehicle.GetCage().GetTetrahedronVolume(i) > 0.0f);
		EXPECT(surface == vehicle.GetRenderGeometry());
		ImpactFront(vehicle, 400.0f);
		EXPECT(vehicle.GetPendingImpactCount() > 0);
		fixture.Body->X = Vector3(5, 2, -3);
		const Transform world = vehicle.GetWorldTransform();
		vehicle.Reset();
		EXPECT(vehicle.GetPendingImpactCount() == 0);
		EXPECT(vehicle.GetMaxDisplacement() == 0.0f);
		EXPECT(vehicle.GetMaxPlasticStrain() == 0.0f);
		EXPECT(fixture.Body->Parent == &vehicle && vehicle.GetBody() == fixture.Body);
		EXPECT_SAME(vehicle.GetRenderGeometry()->GetPosition(), world.pos);
		for (const DeformableParticle& particle : vehicle.GetCage().GetParticles())
		{
			EXPECT_SAME(particle.Position, particle.InitialPosition);
			EXPECT_SAME(particle.PreviousPosition, particle.InitialPosition);
			EXPECT_SAME(particle.Velocity, Vector3::Zero());
			EXPECT_SAME(particle.Force, Vector3::Zero());
		}
		for (const DeformableEdge& edge : vehicle.GetCage().GetEdges())
		{
			EXPECT(edge.RestLength == edge.InitialRestLength);
			EXPECT(edge.Lambda == 0.0f);
		}
		for (const DeformableTetrahedron& tetrahedron : vehicle.GetCage().GetTetrahedra())
		{
			EXPECT(tetrahedron.RestVolume == tetrahedron.InitialRestVolume);
			EXPECT(tetrahedron.Lambda == 0.0f);
		}
		vehicle.Simulate(VehicleStep);
		EXPECT(vehicle.GetMaxDisplacement() < 1.0e-5f);
	}

	void TestVehicleBudgetAndLifetime()
	{
		VehicleFixture one, many;
		ImpactFront(one.Vehicle, 400.0f);
		for (int i = 0; i < 4; ++i) ImpactFront(many.Vehicle, 400.0f);
		one.Vehicle.Simulate(VehicleStep);
		many.Vehicle.Simulate(VehicleStep);
		CompareCages(one.Vehicle, many.Vehicle, 1.0e-4f);
		EXPECT(one.Vehicle.GetPendingImpactCount() == 0 && many.Vehicle.GetPendingImpactCount() == 0);
		VehicleFixture fixture;
		VehicleDeformation& vehicle = fixture.Vehicle;
		const Vector3 point = vehicle.GetWorldTransform().LocalToWorld(FrontPoint(vehicle));
		EXPECT_FALSE(vehicle.QueueImpact(point, Vector3(0, 0, -19.0f)));
		EXPECT_FALSE(vehicle.QueueImpact(point, Vector3::Zero()));
		EXPECT_FALSE(vehicle.QueueImpact(Vector3(1000.0f), Vector3(0, 0, -100.0f)));
		EXPECT_FALSE(vehicle.QueueImpact(point, Vector3(std::numeric_limits<float>::infinity(), 0, 0)));
		ImpactFront(vehicle, 400.0f);
		vehicle.Simulate(0.0f);
		vehicle.Simulate(-1.0f);
		vehicle.Simulate(std::numeric_limits<float>::quiet_NaN());
		EXPECT(vehicle.GetPendingImpactCount() == 1);
		EXPECT(vehicle.GetMaxDisplacement() == 0.0f);
		GeometryAggregate otherOwner;
		RigidBodyParam fixed;
		RigidBody* staticBody = fixture.World.CreateRigidBody(GeometryFactory::CreateSphere(Vector3(10, 0, 0), 0.2f), fixed);
		EXPECT_FALSE(vehicle.Attach(nullptr));
		EXPECT_FALSE(vehicle.Attach(staticBody));
		{
			VehicleDeformation second;
			EXPECT_FALSE(second.Attach(fixture.Body));
			EXPECT(fixture.Body->Parent == &vehicle);
		}
		fixture.Body->X = Vector3(2, 3, 4);
		vehicle.Detach();
		EXPECT(fixture.Body->Parent == nullptr);
		EXPECT(vehicle.GetBody() == nullptr && vehicle.GetPendingImpactCount() == 0);
		EXPECT_SAME(vehicle.GetRenderGeometry()->GetPosition(), fixture.Body->X);
		fixture.Body->Parent = &otherOwner;
		EXPECT_FALSE(vehicle.Attach(fixture.Body));
		EXPECT(fixture.Body->Parent == &otherOwner);
		fixture.Body->Parent = nullptr;
		{
			VehicleDeformation temporary;
			EXPECT(temporary.Attach(fixture.Body));
		}
		EXPECT(fixture.Body->Parent == nullptr);
		EXPECT(vehicle.Attach(fixture.Body));
		fixture.Body->Parent = &otherOwner;
		vehicle.Detach();
		EXPECT(fixture.Body->Parent == &otherOwner);
		fixture.Body->Parent = nullptr;
	}
}

void TestVehicleDeformation()
{
	printf("Running TestVehicleDeformation\n");
	TestVehicleCageTopology();
	TestVehicleFrameAndContactSigns();
	TestVehicleElasticPlasticAndReset();
	TestVehicleBudgetAndLifetime();
}
}

void TestVehicleDeformation() { Test_VehicleDeformation_Cases::TestVehicleDeformation(); }
