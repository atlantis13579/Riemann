#include "Test.h"
#include "../Src/RigidBodyDynamics/Ragdoll.h"
#include "../Src/RigidBodyDynamics/PhysicsWorld.h"
#include "../Src/Collision/GeometryObject.h"
#include <limits>
#include <stdexcept>

namespace
{
	using namespace Riemann;
	constexpr float Dt = 0.01f;
	PhysicsWorldParam Parameters()
	{
		PhysicsWorldParam P; P.gravityAcc = Vector3::Zero(); P.narrowphase = NarrowPhaseSolver::PCM;
		P.sequentialImpulseSettings.PositionIterations = 32; P.sequentialImpulseSettings.VelocityIterations = 16; return P;
	}
	RigidBodyDynamic* Body(PhysicsWorld& World, bool Collider = false, const Vector3& Position = Vector3::Zero())
	{
		RigidBodyParam P; P.rigidType = RigidType::Dynamic; P.linearDamping = P.angularDamping = 1; P.sleepThreshold = 0;
		auto* B = (Collider ? World.CreateRigidBody(GeometryFactory::CreateSphere(Position, 0.25f), P)
			: World.CreateRigidBody(P, Transform(Position)))->CastDynamic();
		B->SetRestitution(0); B->SetFriction(0.8f, 0.8f); return B;
	}
	Ragdoll* Tree(PhysicsWorld& World, const RagdollSettings& Settings)
	{
		std::vector<RigidBodyDynamic*> Bodies; for (const auto& L : Settings.Links) { (void)L; Bodies.push_back(Body(World)); }
		return World.CreateRagdoll(Settings, Bodies);
	}
	void Step(PhysicsWorld& World, int Count) { for (int I = 0; I < Count; ++I) World.Simulate(Dt); }
	RagdollSettings Branches()
	{
		RagdollSettings S = RagdollSettings::Reduced(); S.Links.resize(4);
		for (size_t I = 1; I < S.Links.size(); ++I)
		{
			auto& L = S.Links[I]; L.Parent = 0; L.Joint = I == 3 ? RagdollReducedJointType::Prismatic : RagdollReducedJointType::Revolute;
			L.Mass = (float)(I + 1); L.Inertia = Matrix3(Vector3(L.Mass)); L.Axis = I == 3 ? Vector3::UnitX() : Vector3::UnitZ();
		}
		return S;
	}
	void GearProjectionAndDrive()
	{
		PhysicsWorld World(Parameters()); auto* A = Tree(World, Branches());
		auto* Gear = A->CreateMechanism(RagdollMechanismSettings::Gear(1, 2, 2));
		A->SetVelocity({ 2, -0.5, 0 }); const double BeforeEnergy = A->GetDynamics().GetKineticEnergy();
		Step(World, 1);
		const double Weight = 1.0 / 2 + 4.0 / 3, Impulse = -1 / Weight;
		EXPECT_NEAR(Gear->GetImpulse(), Impulse, 1e-7);
		EXPECT_NEAR(A->GetDynamics().GetVelocity()[0], 2 + Impulse / 2, 1e-7);
		EXPECT_NEAR(A->GetDynamics().GetVelocity()[1], -0.5 + 2 * Impulse / 3, 1e-7);
		EXPECT_NEAR(Gear->GetRate(), 0, 1e-7); EXPECT_NEAR(Gear->GetCoordinate(), 0, 1e-7);
		EXPECT(A->GetDynamics().GetKineticEnergy() < BeforeEnergy);
		auto* Rack = A->CreateMechanism(RagdollMechanismSettings::Rack(2, 3, 0.4));
		RagdollControl C; C.Drive.Settings.VelocityEnabled = true; C.Drive.Settings.Damping = 100;
		C.Drive.Settings.MaxForce = 100; C.Drive.TargetVelocity = 1; A->SetControl(0, C);
		Step(World, 1000);
		printf("Mechanism gear=%.9f rack=%.9f speed=%.6f\n", Gear->GetCoordinate(), Rack->GetCoordinate(), A->GetDynamics().GetVelocity()[0]);
		EXPECT_NEAR(Gear->GetCoordinate(), 0, 1e-6); EXPECT_NEAR(Rack->GetCoordinate(), 0, 1e-6);
		EXPECT_NEAR(A->GetDynamics().GetVelocity()[0], 1, 1e-5);
		EXPECT(A->GetDynamics().GetConfiguration()[1].Position > 6);
		EXPECT(A->RemoveMechanism(Gear)); EXPECT(A->GetMechanismCount() == 1);
		auto* Mimic = A->CreateMechanism(RagdollMechanismSettings::Mimic(1, 2, 1.5, 0.3));
		Step(World, 500); EXPECT_NEAR(Mimic->GetCoordinate(), 0.3, 1e-6); EXPECT_NEAR(Rack->GetCoordinate(), 0, 1e-6);
	}
	void CoupledTreeResponse()
	{
		PhysicsWorld World(Parameters()); auto S = Branches(); S.Links[0].Joint = RagdollReducedJointType::Floating;
		S.Links[1].CenterOfMass = Vector3(1, 0.3f, 0); S.Links[2].CenterOfMass = Vector3(-0.5f, 0.7f, 0);
		auto* A = Tree(World, S); RagdollMechanismSettings C; C.Target = 0.2;
		C.Joints = { { 1, 0.5 }, { 1, 0.5 }, { 2, 2 }, { 3, -0.3 } };
		auto* Row = A->CreateMechanism(C); const auto Gradient = Row->GetGradient();
		FeatherstoneSolver Model(S.Links); const auto Response = Model.ComputeImpulseResponse({}, Gradient);
		double Weight = 0; for (size_t I = 0; I < Gradient.size(); ++I) Weight += Gradient[I] * Response[I];
		const double Impulse = C.Target * C.ErrorReduction / Dt / Weight;
		Step(World, 1); EXPECT_NEAR(Row->GetImpulse(), Impulse, 1e-6);
		for (size_t I = 0; I < Gradient.size(); ++I) EXPECT_NEAR(A->GetDynamics().GetVelocity()[I], Response[I] * Impulse, 2e-6);
		EXPECT(std::abs(A->GetDynamics().GetVelocity()[2]) > 0.01);
	}
	void ForceCapsAndContacts()
	{
		for (int Substeps : { 1, 4 }) for (bool Compliant : { false, true }) for (bool Patches : { false, true })
		{
			auto P = Parameters(); P.temporalSubsteps.Enabled = true; P.temporalSubsteps.Substeps = Substeps;
			P.frictionPatches.Enabled = Patches; P.unifiedConstraints = Patches;
			PhysicsWorld World(P); RagdollSettings S = RagdollSettings::Reduced(); S.Links.resize(1);
			S.Links[0].Joint = RagdollReducedJointType::Prismatic; S.Links[0].Axis = Vector3::UnitX();
			auto* A = Body(World, true); auto* B = Body(World, true, Vector3(0.5f, 0, 0)); B->InvMass = 1;
			auto* Owner = World.CreateRagdoll(S, { A }); RagdollMechanismSettings C;
			C.Joints = { { 0, 1 } }; C.Target = 10; C.MaxForce = 1; C.Compliant = Compliant;
			auto* Row = Owner->CreateMechanism(C); Step(World, 20);
			EXPECT_NEAR(A->V.x + B->V.x, 20 * Dt, 1e-5f); EXPECT(B->V.x > 0.05f);
			EXPECT_NEAR(Row->GetForce(), 1, 1e-7);
		}
	}
	void SpatialGradient()
	{
		PhysicsWorld World(Parameters()); auto S = Branches(); S.Links[0].Joint = RagdollReducedJointType::Floating;
		S.Links[1].ParentToJoint.pos = Vector3(1, 0, 0); S.Links[2].Parent = 1; S.Links[2].ParentToJoint.pos = Vector3(0, 1, 0);
		auto* A = Tree(World, S); RagdollMechanismSettings C; C.Type = RagdollMechanismType::SpatialTendon;
		C.Path = { { -1, Vector3(2, 3, 1) }, { 0, Vector3(0.1f, -0.2f, 0.3f) }, { 1, Vector3(0.5f, 0.2f, 0.1f) },
			{ 1, Vector3(0.5f, 0.2f, 0.1f) }, { 2, Vector3(0.4f, 0.8f, 0.2f) }, { 3, Vector3(-0.5f, 0, 0) } };
		auto* Row = A->CreateMechanism(C); auto Q = A->GetDynamics().GetConfiguration();
		Q[0].Rotation.FromRotationAxis(Vector3::UnitY(), 0.3f); Q[1].Position = 0.2; Q[2].Position = -0.3; Q[3].Position = 0.4;
		A->SetConfiguration(Q); const auto Gradient = Row->GetGradient();
		FeatherstoneSolver Model(S.Links); constexpr double Interval = 0.001;
		for (size_t I = 0; I < Gradient.size(); ++I)
		{
			std::vector<double> Unit(Gradient.size()); Unit[I] = 1; Model.SetConfiguration(Q); Model.SetVelocity(Unit);
			Model.IntegrateConfiguration(Interval); A->SetConfiguration(Model.GetConfiguration()); const double Plus = Row->GetCoordinate();
			Unit[I] = -1; Model.SetConfiguration(Q); Model.SetVelocity(Unit); Model.IntegrateConfiguration(Interval);
			A->SetConfiguration(Model.GetConfiguration()); const double Minus = Row->GetCoordinate();
			EXPECT_NEAR(Gradient[I], (Plus - Minus) / (2 * Interval), 0.001);
		}
		A->SetConfiguration(Q); std::vector<double> Velocity(Gradient.size(), 0.2); A->SetVelocity(Velocity);
		double Rate = 0; for (double G : Gradient) Rate += 0.2 * G; EXPECT_NEAR(Row->GetRate(), Rate, 1e-7);
	}
	void TendonSlackAndElasticity()
	{
		for (bool Compliant : { false, true })
		{
			auto P = Parameters(); P.gravityAcc = Vector3(0, -9.8f, 0); PhysicsWorld World(P);
			RagdollSettings S = RagdollSettings::Reduced(); S.Links.resize(1); S.Links[0].Joint = RagdollReducedJointType::Floating;
			S.Links[0].Mass = 2; S.Links[0].ParentToJoint.pos = Vector3(0, -1, 0);
			auto* A = Tree(World, S); RagdollMechanismSettings C; C.Type = RagdollMechanismType::SpatialTendon;
			C.Path = { { -1, Vector3::Zero() }, { 0, Vector3::Zero() } }; C.Target = 1; C.TensionOnly = true;
			C.Compliant = Compliant; C.Stiffness = 200; C.Damping = 40; auto* Row = A->CreateMechanism(C);
			Step(World, 600); const double ExpectedLength = 1 + (Compliant ? 19.6 / C.Stiffness : 0);
			printf("Tendon compliant=%d length=%.6f force=%.6f\n", Compliant, Row->GetCoordinate(), Row->GetForce());
			EXPECT_NEAR(Row->GetCoordinate(), ExpectedLength, 2e-5); EXPECT_NEAR(Row->GetForce(), -19.6, 1e-4);
		}
		PhysicsWorld World(Parameters()); RagdollSettings S = RagdollSettings::Reduced(); S.Links.resize(1); S.Links[0].Joint = RagdollReducedJointType::Floating;
		S.Links[0].ParentToJoint.pos = Vector3(0, -0.5f, 0); auto* A = Tree(World, S);
		RagdollMechanismSettings C; C.Type = RagdollMechanismType::SpatialTendon;
		C.Path = { { -1, Vector3::Zero() }, { 0, Vector3::Zero() } }; C.Target = 1; C.TensionOnly = true;
		auto* Row = A->CreateMechanism(C);
		for (bool Compliant : { false, true })
		{
			C.Compliant = Compliant; Row->SetSettings(C); auto Q = A->GetDynamics().GetConfiguration(); Q[0].Translation = Vector3::Zero(); A->SetConfiguration(Q);
			A->SetVelocity({ 0, 0, 0, 0, -2, 0 }); Step(World, 10);
			EXPECT_NEAR(Row->GetCoordinate(), 0.7, 1e-6); EXPECT_NEAR(Row->GetForce(), 0, 1e-8);
		}
		C.Compliant = false; Row->SetSettings(C); Step(World, 30); EXPECT_NEAR(Row->GetCoordinate(), 1, 1e-6);
		A->SetVelocity({ 0, 0, 0, 0, 1, 0 }); Step(World, 10); EXPECT_NEAR(Row->GetCoordinate(), 0.9, 1e-6); EXPECT_NEAR(Row->GetForce(), 0, 1e-8);
		C.Path[0] = C.Path[1]; C.Target = 0; Row->SetSettings(C); Step(World, 10);
		EXPECT_NEAR(Row->GetCoordinate(), 0, 1e-8); EXPECT_NEAR(Row->GetForce(), 0, 1e-8);
	}
	void InternalTendonMomentum()
	{
		PhysicsWorld World(Parameters()); RagdollSettings S = RagdollSettings::Reduced(); S.Links.resize(2);
		S.Links[0].Joint = RagdollReducedJointType::Floating; S.Links[0].ParentToJoint.pos = Vector3(-1, 0, 0);
		S.Links[1].Parent = 0; S.Links[1].Joint = RagdollReducedJointType::Prismatic; S.Links[1].Axis = Vector3::UnitX();
		S.Links[1].ParentToJoint.pos = Vector3(2, 0, 0); auto* A = Tree(World, S);
		RagdollMechanismSettings C; C.Type = RagdollMechanismType::SpatialTendon;
		C.Path = { { 0, Vector3::Zero() }, { 1, Vector3::Zero() } }; C.Target = 2; C.TensionOnly = true;
		auto* Row = A->CreateMechanism(C); A->SetVelocity({ 0, 0, 0, -1, 0, 0, 2 }); Step(World, 100);
		EXPECT_NEAR(Row->GetCoordinate(), 2, 1e-5); EXPECT_NEAR(A->GetDynamics().GetMomentum().GetLinear().Length(), 0, 1e-5f);
		EXPECT_NEAR((A->GetBodies()[0]->X + A->GetBodies()[1]->X).Length(), 0, 1e-5f);
	}
	void ValidationAndLifetime()
	{
		auto P = Parameters(); P.islandSleeping.Enabled = true; P.islandSleeping.QuietTime = 0.03f;
		PhysicsWorld World(P); auto* A = Tree(World, Branches());
		for (auto* B : A->GetBodies()) B->SleepThreshold = RigidBodyParam().sleepThreshold;
		auto C = RagdollMechanismSettings::Gear(1, 2, 2); auto* Row = A->CreateMechanism(C);
		auto Rejects = [](auto Function) { bool Rejected = false; try { Function(); } catch (const std::invalid_argument&) { Rejected = true; } EXPECT(Rejected); };
		auto Bad = C; Bad.Joints[0].Link = 0; Rejects([&] { A->CreateMechanism(Bad); });
		Bad = C; Bad.Joints[0].Coefficient = std::numeric_limits<double>::infinity(); Rejects([&] { Row->SetSettings(Bad); });
		Bad = C; Bad.Joints = { { 1, 1 }, { 1, -1 } }; Rejects([&] { A->CreateMechanism(Bad); });
		Bad = C; Bad.ErrorReduction = -1; Rejects([&] { Row->SetSettings(Bad); });
		Bad = C; Bad.Type = RagdollMechanismType::SpatialTendon; Bad.Joints.clear();
		Bad.Path = { { -1, Vector3::Zero() }, { 4, Vector3::Zero() } }; Rejects([&] { Row->SetSettings(Bad); });
		EXPECT(Row->GetSettings().Joints.size() == 2); EXPECT(A->GetMechanismCount() == 1);
		Step(World, 10); EXPECT(A->IsSleeping()); C.Target = 0.2; Row->SetSettings(C); EXPECT(!A->IsSleeping());
		// Sleeping permits small residual drift under the configured kinetic-energy
		// threshold. Verify wake propagation and motion, not a tighter position rule.
		Step(World, 50); EXPECT(Row->GetCoordinate() > 0.1 && Row->GetCoordinate() <= 0.2);
		A->SetFrozen(true); C.Target = 1; Row->SetSettings(C); const double Frozen = Row->GetCoordinate(); Step(World, 10);
		EXPECT_NEAR(Row->GetCoordinate(), Frozen, 1e-8); EXPECT_NEAR(Row->GetForce(), 0, 1e-8);
		A->SetFrozen(false); Step(World, 100); EXPECT(Row->GetCoordinate() > 0.9 && Row->GetCoordinate() <= 1);
		C.Enabled = false; Row->SetSettings(C); A->SetVelocity({ 1, 0, 0 }); Step(World, 10); EXPECT_NEAR(Row->GetForce(), 0, 1e-8);
		EXPECT(A->RemoveMechanism(Row)); EXPECT(!A->RemoveMechanism(nullptr)); EXPECT(A->GetMechanismCount() == 0);
		A->CreateMechanism(C); EXPECT(World.RemoveRigidBody(A->GetBodies().back())); EXPECT(World.GetReducedRagdollCount() == 0);
		World.Reset();
	}
}
void TestRagdollMechanism()
{
	GearProjectionAndDrive(); CoupledTreeResponse(); ForceCapsAndContacts(); SpatialGradient();
	TendonSlackAndElasticity(); InternalTendonMomentum(); ValidationAndLifetime();
}
