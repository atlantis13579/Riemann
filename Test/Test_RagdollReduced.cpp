#include "Test.h"
#include "../Src/RigidBodyDynamics/Ragdoll.h"
#include "../Src/RigidBodyDynamics/Island.h"
#include "../Src/RigidBodyDynamics/Jacobian.h"
#include "../Src/RigidBodyDynamics/PhysicsWorld.h"
#include "../Src/Collision/GeometryObject.h"
#include "../Src/CollisionPrimitive/TriangleMesh.h"
#include "../Src/Deformation/Rope.h"
#include "../Viewer/SceneWorld.h"
#include <cmath>
#include <stdexcept>

namespace
{
	using namespace Riemann;
	constexpr float Dt = 0.01f, Gravity = 9.8f;
	PhysicsWorldParam Parameters(bool GravityEnabled = false, bool Friction = false)
	{
		PhysicsWorldParam P; P.gravityAcc = Vector3(0, GravityEnabled ? -Gravity : 0, 0);
		P.frictionPatches.Enabled = Friction; P.narrowphase = NarrowPhaseSolver::PCM;
		P.sequentialImpulseSettings.PositionIterations = 32; P.sequentialImpulseSettings.VelocityIterations = 8;
		return P;
	}
	RigidBodyDynamic* Body(PhysicsWorld& World, const Vector3& Position = Vector3::Zero(), bool Shape = true)
	{
		RigidBodyParam P; P.rigidType = RigidType::Dynamic; P.linearDamping = P.angularDamping = 1; P.sleepThreshold = 0;
		auto* B = (Shape ? World.CreateRigidBody(GeometryFactory::CreateSphere(Position, 0.25f), P)
			: World.CreateRigidBody(P, Transform(Position)))->CastDynamic();
		B->SetRestitution(0); B->SetFriction(0.8f, 0.8f); return B;
	}
	void Step(PhysicsWorld& World, int Count, float Interval = Dt) { for (int I = 0; I < Count; ++I) World.Simulate(Interval); }
	RagdollSettings Pair()
	{
		RagdollSettings S = RagdollSettings::Reduced(); S.Links.resize(2);
		S.Links[0].Joint = RagdollReducedJointType::Floating; S.Links[0].ParentToJoint.pos = Vector3(-1, 2, 0);
		S.Links[1].Parent = 0; S.Links[1].ParentToJoint.pos = Vector3(2, 0, 0);
		return S;
	}
	void BindingAndForceIntegration()
	{
		PhysicsWorld World(Parameters(true)); auto* B = Body(World);
		const float OriginalMass = B->InvMass; const auto OriginalCM = B->CM;
		RagdollSettings S = RagdollSettings::Reduced(); S.Links.resize(1); auto& L = S.Links[0];
		L.Joint = RagdollReducedJointType::Revolute; L.CenterOfMass = Vector3::UnitX();
		auto* A = World.CreateRagdoll(S, { B });
		EXPECT(A->GetAlgorithm() == RagdollAlgorithm::ReducedCoordinates);
		EXPECT(World.GetRagdollCount() == 1 && World.GetRagdoll(0) == A);
		EXPECT(World.GetReducedRagdollCount() == 1 && A->GetBone(RagdollBone::Pelvis) == nullptr);
		EXPECT_NEAR(B->X.x, 1, 1e-6f); EXPECT_NEAR(B->GetGeometryTransform(B->Geometries()[0]).pos.x, 1, 1e-6f);
		Step(World, 1);
		EXPECT_NEAR(A->GetDynamics().GetVelocity()[0], -Gravity * Dt / 2, 1e-6);
		EXPECT_NEAR(A->GetDynamics().GetConfiguration()[0].Position, -Gravity * Dt * Dt / 2, 1e-7);
		EXPECT(B->ExtForce == Vector3::Zero()); EXPECT_NEAR(B->X.Length(), 1, 1e-6f);
		const auto ShapePose = B->GetGeometryTransform(B->Geometries()[0]);
		EXPECT(World.RemoveRagdoll(A)); EXPECT(World.GetRagdollCount() == 0 && World.GetReducedRagdollCount() == 0);
		EXPECT(!B->ReducedRagdoll); EXPECT(B->CM == OriginalCM);
		EXPECT_NEAR(B->InvMass, OriginalMass, 1e-6f);
		EXPECT_NEAR((B->GetGeometryTransform(B->Geometries()[0]).pos - ShapePose.pos).Length(), 0, 1e-6f);
	}
	void CoupledResponses()
	{
		PhysicsWorld World(Parameters()); auto* A = Body(World); auto* B = Body(World, Vector3::Zero(), false);
		auto* Tree = World.CreateRagdoll(Pair(), { A, B });
		Tree->ApplyImpulse(1, B->X, Vector3::UnitY());
		EXPECT_NEAR(A->V.y, 0.25f, 1e-6f); EXPECT_NEAR(B->V.y, 0.75f, 1e-6f);
		EXPECT_NEAR(A->W.z, 0.25f, 1e-6f); EXPECT_NEAR(B->W.z, 0.25f, 1e-6f);
		Tree->SetVelocity(std::vector<double>(6));
		GeneralizedVelocity Buffer[2] = {}; RagdollVelocityContext Context; Context.Begin({ A, B }, Buffer);
		RagdollImpulseResponse Response;
		const double K = Context.Response(A, -Vector3::UnitY(), Vector3::Zero(), B, Vector3::UnitY(), Vector3::Zero(), Response);
		EXPECT_NEAR(K, 1, 1e-6); EXPECT_NEAR(Context.Project(Response, A, -Vector3::UnitY(), Vector3::Zero(), B, Vector3::UnitY(), Vector3::Zero()), K, 1e-6);
		Response.Apply(1); Context.Commit();
		EXPECT_NEAR(A->V.y, -0.5f, 1e-6f); EXPECT_NEAR(B->V.y, 0.5f, 1e-6f);
		EXPECT_NEAR(Tree->GetDynamics().GetMomentum().GetLinear().Length(), 0, 1e-6f);
		ConstraintGraph Graph; Graph.Begin({ A, B }); Graph.Finish(); EXPECT(Graph.GetIslands().size() == 1);
		A->Sleep(); EXPECT(B->Sleeping); B->Wakeup(); EXPECT(!A->Sleeping);
		B->Freeze(); EXPECT(A->Freezing); Tree->ApplyImpulse(1, B->X, Vector3::UnitY()); EXPECT(B->V == Vector3::Zero());
		A->Defreeze(); EXPECT(!B->Freezing && !B->Sleeping);
	}
	void ExplicitReducedDestroy()
	{
		PhysicsWorld World(Parameters()); auto* BodyInstance = Body(World);
		RagdollSettings Settings = RagdollSettings::Reduced(); Settings.Links.resize(1);
		auto* Instance = World.CreateRagdoll(Settings, { BodyInstance });
		Instance->Destroy();
		EXPECT(World.GetRagdollCount() == 0 && World.GetReducedRagdollCount() == 0);
		EXPECT(BodyInstance->ReducedRagdoll == nullptr);
	}
	void AuthoredAlgorithmSwitch()
	{
		PhysicsWorld World(Parameters(true));
		RagdollSettings Settings;
		Settings.DefinitionFile = TestContentsPath() + "Config/default_ragdoll.json";
		const Transform Pose(Vector3(0, 4, 0));
		auto* Jointed = World.CreateRagdoll(Settings, Pose);
		EXPECT(Jointed && Jointed->GetAlgorithm() == RagdollAlgorithm::JointConstraints);
		if (!Jointed) return;
		EXPECT(Jointed->GetConstraintCount() == 10 && World.GetReducedRagdollCount() == 0);
		EXPECT(World.RemoveRagdoll(Jointed) && World.GetRagdollCount() == 0);

		Settings.Algorithm = RagdollAlgorithm::ReducedCoordinates;
		auto* Reduced = World.CreateRagdoll(Settings, Pose);
		EXPECT(Reduced && Reduced->GetAlgorithm() == RagdollAlgorithm::ReducedCoordinates);
		if (!Reduced) return;
		EXPECT(Reduced->GetBodyCount() == 11 && Reduced->GetConstraintCount() == 0);
		EXPECT_NEAR(Reduced->GetDynamics().GetLinkSettings((size_t)RagdollBone::Torso).ConeLimit,
			35.0 * 3.14159265358979323846 / 180.0, 1e-6);
		EXPECT(World.GetRagdollCount() == 1 && World.GetReducedRagdollCount() == 1);
		const size_t ElbowDof = (size_t)Reduced->GetDynamics().GetLinkDofOffset((size_t)RagdollBone::LeftLowerArm);
		const auto& ElbowControl = Reduced->GetControl(ElbowDof);
		EXPECT(ElbowControl.LimitEnabled);
		EXPECT_NEAR(ElbowControl.Lower, -135.0 * 3.14159265358979323846 / 180.0, 1e-6);
		auto* Pelvis = Reduced->GetBone(RagdollBone::Pelvis);
		EXPECT(Pelvis != nullptr);
		if (Pelvis)
		{
			const float StartY = Pelvis->X.y;
			Step(World, 20);
			EXPECT(Pelvis->X.y < StartY && std::isfinite(Pelvis->X.y));
		}
		EXPECT(World.RemoveRagdoll(Reduced));
		EXPECT(World.GetRagdollCount() == 0 && World.GetReducedRagdollCount() == 0);

		SceneWorld Scene; std::string Error;
		EXPECT(Scene.LoadFromText(R"({"name":"Ragdoll algorithm switch","objects":[],"ragdolls":[{"algorithm":"reducedCoordinates","position":[0,4,0]}]})", "switch", &Error));
		EXPECT(Scene.GetSimulation()->GetRagdollCount() == 1 && Scene.GetSimulation()->GetReducedRagdollCount() == 1);
		Scene.Step(Dt);

		SceneWorld Stairs; Stairs.SetRagdollAlgorithmOverride(RagdollAlgorithm::ReducedCoordinates);
		EXPECT(Stairs.LoadFromFile(TestContentsPath() + "Scenes/09_ragdoll_stairs.json", &Error));
		EXPECT(Stairs.GetSimulation()->GetRagdollCount() == 1 && Stairs.GetSimulation()->GetReducedRagdollCount() == 1);
		Stairs.Step(Dt);
	}
	void SphericalConeLimit()
	{
		PhysicsWorld World(Parameters()); RagdollSettings S=RagdollSettings::Reduced(); S.Links.resize(2);
		S.Links[0].Joint=RagdollReducedJointType::Fixed;
		auto& Joint=S.Links[1]; Joint.Parent=0; Joint.Joint=RagdollReducedJointType::Spherical;
		Joint.ConeAxisParent=Joint.ConeAxisChild=Vector3::UnitZ(); Joint.ConeLimit=0.4;
		auto* R=World.CreateRagdoll(S,{Body(World,Vector3::Zero(),false),Body(World,Vector3::Zero(),false)});
		auto Configuration=R->GetDynamics().GetConfiguration();
		Configuration[1].Rotation.FromRotationAxis(Vector3::UnitX(),0.35f); R->SetConfiguration(Configuration);
		R->SetVelocity({2,0,0}); Step(World,30);
		const auto& Model=R->GetDynamics();
		const Vector3 A=Model.GetLinkTransform(0).quat*Joint.ConeAxisParent;
		const Vector3 B=Model.GetLinkTransform(1).quat*Joint.ConeAxisChild;
		const double Angle=std::acos(std::max(-1.0,std::min(1.0,(double)A.Dot(B))));
		EXPECT(Angle<=Joint.ConeLimit+0.005); EXPECT(Angle>Joint.ConeLimit-0.005);
		R->SetVelocity({-1,0,0}); Step(World,10);
		const Vector3 Inner=R->GetDynamics().GetLinkTransform(1).quat*Joint.ConeAxisChild;
		EXPECT(std::acos(std::max(-1.0,std::min(1.0,(double)A.Dot(Inner))))<Angle-0.05);
		PhysicsWorld LockedWorld(Parameters()); S.Links[1].ConeLimit=0;
		auto* Locked=LockedWorld.CreateRagdoll(S,{Body(LockedWorld,Vector3::Zero(),false),Body(LockedWorld,Vector3::Zero(),false)});
		auto LockedConfiguration=Locked->GetDynamics().GetConfiguration();
		LockedConfiguration[1].Rotation.FromRotationAxis(Vector3(1,1,0).SafeUnit(),0.2f);
		Locked->SetConfiguration(LockedConfiguration); Step(LockedWorld,5);
		const Vector3 LockedAxis=Locked->GetDynamics().GetLinkTransform(1).quat*Joint.ConeAxisChild;
		EXPECT(std::acos(std::max(-1.0,std::min(1.0,(double)A.Dot(LockedAxis))))<0.002);
	}
	void ScalarControls()
	{
		for (int Substeps : { 1, 4 })
		{
			auto P = Parameters(); P.temporalSubsteps.Enabled = true; P.temporalSubsteps.Substeps = Substeps;
			PhysicsWorld World(P); auto* B = Body(World); RagdollSettings S = RagdollSettings::Reduced(); S.Links.resize(1);
			S.Links[0].Joint = RagdollReducedJointType::Prismatic; S.Links[0].Axis = Vector3::UnitX(); S.Links[0].Mass = 2;
			auto* A = World.CreateRagdoll(S, { B }); RagdollControl C;
			C.Drive.Settings.PositionEnabled = true; C.Drive.Settings.Stiffness = 200; C.Drive.Settings.Damping = 40;
			C.Drive.Settings.MaxForce = 1; C.Drive.TargetPosition = 1; A->SetControl(0, C);
			Step(World, 1); EXPECT_NEAR(A->GetDynamics().GetVelocity()[0], Dt / 2, 1e-6);
			C.Drive.Settings.MaxForce = 100; A->SetControl(0, C); Step(World, 500);
			EXPECT_NEAR(A->GetDynamics().GetConfiguration()[0].Position, 1, 1e-4);
			C.LimitEnabled = true; C.Lower = -0.2; C.Upper = 0.2; A->SetControl(0, C); Step(World, 300);
			EXPECT_NEAR(A->GetDynamics().GetConfiguration()[0].Position, 0.2, 1e-5);
			EXPECT_NEAR(A->GetDynamics().GetVelocity()[0], 0, 1e-4);
		}
	}
	void ContactAndFriction()
	{
		for (bool Patch : { false, true }) for (bool Colliderless : { false, true })
		{
			PhysicsWorld World(Parameters(true, Patch)); RigidBodyParam Floor;
			auto* Ground = World.CreateRigidBody(GeometryFactory::CreateOBB(Vector3(0, -0.5f, 0), Vector3(20, 0.5f, 20)), Floor);
			Ground->SetFriction(0.8f, 0.8f); Ground->SetRestitution(0);
			auto* A = Body(World); auto* B = Body(World, Vector3::Zero(), !Colliderless);
			auto* Tree = World.CreateRagdoll(Pair(), { A, B });
			auto V = Tree->GetDynamics().GetVelocity(); V[5] = 1; Tree->SetVelocity(V);
			Step(World, 500, 0.005f);
			printf("Ragdoll friction=%d colliderless=%d heights=%.5f/%.5f speed=%.5f\n", Patch, Colliderless, A->X.y, B->X.y, A->V.Length());
			EXPECT(std::isfinite(A->X.Length()) && std::isfinite(B->X.Length()));
			EXPECT(A->X.y > 0.14f); if (!Colliderless) { EXPECT(B->X.y > 0.14f); EXPECT(std::abs(A->V.z) < 0.15f); }
			EXPECT_NEAR((B->X - A->X).Length(), 2, 2e-5f);
			EXPECT_NEAR((B->V - A->V - A->W.Cross(B->X - A->X)).Length(), 0, 2e-5f);
		}
	}
	void MomentumAndLifetime()
	{
		PhysicsWorld World(Parameters(false, true)); auto* A = Body(World); auto* B = Body(World);
		auto* Tree = World.CreateRagdoll(Pair(), { A, B });
		auto* Projectile = Body(World, Vector3(-3, 2, 0)); Projectile->InvMass = 1; Projectile->V = Vector3(2, 0, 0);
		Step(World, 300, 0.005f);
		EXPECT_NEAR((Tree->GetDynamics().GetMomentum().GetLinear() + Projectile->V).x, 2, 1e-4f);
		EXPECT(A->V.x > 0.1f); EXPECT_NEAR((B->X - A->X).Length(), 2, 2e-5f);
		std::string Error; EXPECT(!World.SetSolverConfiguration(ResolutionPhaseSolver::XPBD, SolverBackend::CPU, true, &Error));
		EXPECT(!Error.empty()); EXPECT(World.GetReducedRagdollCount() == 1);
		EXPECT(World.CreateFixedConstraint(A, Projectile) == nullptr);
		EXPECT(World.RemoveRigidBody(B)); EXPECT(!A->ReducedRagdoll); EXPECT(World.GetReducedRagdollCount() == 0);
		World.Reset(); EXPECT(World.GetReducedRagdollCount() == 0);
	}
	void TreeIntegrationAndSleeping()
	{
		PhysicsWorld World(Parameters(true)); RagdollSettings S = RagdollSettings::Reduced(); S.Links.resize(12);
		std::vector<RigidBodyDynamic*> Bodies;
		for (size_t I = 0; I < S.Links.size(); ++I)
		{
			auto& L = S.Links[I]; L.Parent = I == 0 ? -1 : (int)(I - 1) / 2;
			L.Joint = I == 0 ? RagdollReducedJointType::Floating : (RagdollReducedJointType)(I % 4);
			L.ParentToJoint.pos = Vector3(0.5f, 0.1f, 0.2f); L.CenterOfMass = Vector3(0.2f, 0.1f, 0);
			L.Axis = Vector3(1, 2, 3).Unit(); L.Mass = 1 + (float)I; L.Inertia = Matrix3(Vector3(L.Mass));
			L.Damping = 0.1; Bodies.push_back(Body(World, Vector3::Zero(), I % 2 == 0));
		}
		auto* A = World.CreateRagdoll(S, Bodies); FeatherstoneSolver Reference(S.Links);
		auto Configuration = Reference.GetConfiguration();
		for (size_t I = 0; I < Configuration.size(); ++I)
		{
			Configuration[I].Position = 0.2; Configuration[I].Rotation.FromRotationAxis(Vector3::UnitY(), 0.2f);
		}
		A->SetConfiguration(Configuration); Reference.SetConfiguration(Configuration);
		std::vector<double> Velocity(Reference.GetDofCount(), 0.1), Force(Reference.GetDofCount());
		A->SetVelocity(Velocity); Reference.SetVelocity(Velocity);
		for (int StepIndex = 0; StepIndex < 100; ++StepIndex)
		{
			constexpr float Interval = 0.002f;
			Reference.IntegrateVelocity(Reference.ComputeAcceleration(Force, Vector3(0, -Gravity, 0)), Interval);
			Reference.IntegrateConfiguration(Interval); World.Simulate(Interval);
		}
		for (size_t I = 0; I < Bodies.size(); ++I)
			EXPECT_NEAR((Bodies[I]->X - Reference.GetCenterOfMass(I)).Length(), 0, 1e-4f);
		for (size_t I = 0; I < Velocity.size(); ++I) EXPECT_NEAR(A->GetDynamics().GetVelocity()[I], Reference.GetVelocity()[I], 1e-4);

		auto P = Parameters(); P.islandSleeping.Enabled = true; P.islandSleeping.QuietTime = 0.03f;
		PhysicsWorld SleepWorld(P); auto* Root = Body(SleepWorld); auto* Tip = Body(SleepWorld);
		Root->SleepThreshold = Tip->SleepThreshold = RigidBodyParam().sleepThreshold;
		auto* SleepingTree = SleepWorld.CreateRagdoll(Pair(), { Root, Tip }); Step(SleepWorld, 10);
		EXPECT(Root->Sleeping && Tip->Sleeping);
		SleepingTree->ApplyImpulse(1, Tip->X, Vector3::UnitX()); EXPECT(!Root->Sleeping && !Tip->Sleeping);
		Step(SleepWorld, 1); EXPECT(Root->V.x > 0 && Tip->V.x > 0);
		SleepingTree->SetVelocity(std::vector<double>(6));
		RagdollControl C; C.Drive.Settings.VelocityEnabled = true; C.Drive.Settings.Damping = 10;
		SleepingTree->SetControl(3, C); Step(SleepWorld, 10); EXPECT(!Root->Sleeping && !Tip->Sleeping);
	}
	void InvalidConfigurations()
	{
		PhysicsWorld World(Parameters()); auto* A = Body(World); auto* B = Body(World); auto* Tree = World.CreateRagdoll(Pair(), { A, B });
		auto Rejects = [](const std::function<void()>& Action) { bool Rejected = false; try { Action(); } catch (const std::invalid_argument&) { Rejected = true; } EXPECT(Rejected); };
		RagdollControl C; C.LimitEnabled = true;
		Rejects([&] { Tree->SetControl(0, C); });
		Rejects([&] { World.CreateRagdoll(RagdollSettings(), { A, B }); });
		Rejects([&] { World.CreateRagdoll(Pair(), { A, B }); });
		GeneralizedVelocity Buffer{}; RagdollVelocityContext Context;
		Rejects([&] { Context.Begin({ A }, &Buffer); });
		RopeSettings RopeParameters; auto* Rope = World.CreateRope(RopeParameters, Vector3::Zero(), Vector3::UnitY());
		EXPECT(!Rope->AttachRigidBody(0, A, A->X));
		World.RemoveRagdoll(Tree); EXPECT(Rope->AttachRigidBody(0, A, A->X));
		Rejects([&] { World.CreateRagdoll(Pair(), { A, B }); });
		Rope->DetachRigidBody(A);
		A->mMotionType = MorionType::Continuous; Rejects([&] { World.CreateRagdoll(Pair(), { A, B }); });
		A->mMotionType = MorionType::Discrete;
		EXPECT(!A->ReducedRagdoll && !B->ReducedRagdoll);
	}
	void SelfCollisionAndContactDrive()
	{
		for (bool SelfCollision : { false, true })
		{
			PhysicsWorld World(Parameters()); auto S = Pair(); S.SelfCollision = SelfCollision;
			S.Links[1].Joint = RagdollReducedJointType::Prismatic; S.Links[1].Axis = Vector3::UnitX();
			S.Links[1].ParentToJoint.pos = Vector3(0.6f, 0, 0);
			auto* A = Body(World); auto* B = Body(World); auto* Tree = World.CreateRagdoll(S, { A, B });
			auto Velocity = Tree->GetDynamics().GetVelocity(); Velocity[3] = 0.5; Velocity[6] = -1; Tree->SetVelocity(Velocity);
			Step(World, 40); const float Separation = (B->X - A->X).Length();
			printf("Ragdoll self collision=%d gap=%.7f momentum=(%.8f,%.8f,%.8f) velocity=(%.7f,%.7f)\n", SelfCollision, Separation,
				Tree->GetDynamics().GetMomentum().GetLinear().x, Tree->GetDynamics().GetMomentum().GetLinear().y, Tree->GetDynamics().GetMomentum().GetLinear().z, A->V.x, B->V.x);
			if (SelfCollision) EXPECT(Separation > 0.45f); else EXPECT_NEAR(Separation, 0.2f, 2e-6f);
			EXPECT_NEAR(Tree->GetDynamics().GetMomentum().GetLinear().Length(), 0, 1e-5f);
		}
		for (int Substeps : { 1, 4 })
		{
			auto P = Parameters(); P.temporalSubsteps.Enabled = true; P.temporalSubsteps.Substeps = Substeps;
			PhysicsWorld World(P); RagdollSettings S = RagdollSettings::Reduced(); S.Links.resize(1);
			S.Links[0].Joint = RagdollReducedJointType::Prismatic; S.Links[0].Axis = Vector3::UnitX();
			auto* A = Body(World); auto* B = Body(World, Vector3(0.5f, 0, 0)); B->InvMass = 1;
			auto* Tree = World.CreateRagdoll(S, { A }); RagdollControl C;
			C.Drive.Settings.VelocityEnabled = true; C.Drive.Settings.Damping = 100; C.Drive.Settings.MaxForce = 1;
			C.Drive.TargetVelocity = 10; Tree->SetControl(0, C);
			Step(World, 20);
			EXPECT_NEAR(A->V.x + B->V.x, 20 * Dt * C.Drive.Settings.MaxForce, 1e-5f);
			EXPECT(B->V.x > 0.05f);
		}
	}
}
void TestReducedRagdoll()
{
	BindingAndForceIntegration(); CoupledResponses(); ExplicitReducedDestroy(); AuthoredAlgorithmSwitch(); SphericalConeLimit(); ScalarControls(); ContactAndFriction(); MomentumAndLifetime();
	TreeIntegrationAndSleeping(); InvalidConfigurations();
	SelfCollisionAndContactDrive();
}

void TestReducedRagdollDemo()
{
	using namespace Riemann;
	SceneWorld Scene; std::string Error; const std::string Path = TestContentsPath() + "Scenes/06_constraints.json";
	EXPECT(Scene.LoadFromFile(Path, &Error)); const size_t OriginalObjects = Scene.GetObjects().size();
	EXPECT(Scene.GetSimulation()->GetReducedRagdollCount() == 0);
	auto Find = [&](const std::string& Id) -> RigidBody*
	{
		for (const auto& Object : Scene.GetObjects()) if (Object.Id == Id) return Object.GeometryPtr->GetParent<RigidBody>();
		return nullptr;
	};
	for (bool Patches : { false, true })
	{
		Scene.SetReducedRagdollDemo(true); EXPECT(Scene.LoadFromFile(Path, &Error));
		auto* World = Scene.GetSimulation(); EXPECT(World->GetReducedRagdollCount() == 6);
		EXPECT(Scene.GetObjects().size() == OriginalObjects + 38);
		EXPECT(World->GetTemporalSubstepSettings().Enabled);
		World->SetUnifiedConstraintSolving(Patches);
		auto Friction = World->GetFrictionPatchSettings(); Friction.Enabled = Patches; World->SetFrictionPatchSettings(Friction);
		auto* ChainRoot = Find("ragdoll_chain_0"); auto* ArmRoot = Find("ragdoll_arm_0"); auto* FreeRoot = Find("ragdoll_floating_0");
		EXPECT(ChainRoot && ArmRoot && FreeRoot); if (!ChainRoot || !ArmRoot || !FreeRoot) return;
		auto* Arm = ArmRoot->ReducedRagdoll; auto* Chain = ChainRoot->ReducedRagdoll; auto* Free = FreeRoot->ReducedRagdoll;
		auto* Gears = Find("mechanism_transmission_0")->ReducedRagdoll;
		auto* Hard = Find("mechanism_cable_0_0")->ReducedRagdoll; auto* Elastic = Find("mechanism_cable_1_0")->ReducedRagdoll;
		EXPECT(Gears->GetMechanismCount() == 3 && Hard->GetMechanismCount() == 1 && Elastic->GetMechanismCount() == 1);
		double MaximumCouplingError = 0, MaximumCableError = 0, MaximumExtension = 0, MinimumInput = 0, MaximumInput = 0;
		const Vector3 Anchor = ChainRoot->X; float MaximumJointError = 0;
		constexpr float FrameTime = 1.0f / 60;
		for (int Frame = 0; Frame < 240; ++Frame)
		{
			Scene.Step(FrameTime);
			for (auto* Tree : { Chain, Arm, Free, Gears, Hard, Elastic }) for (size_t I = 0; I < Tree->GetBodies().size(); ++I)
			{
				auto* Link = Tree->GetBodies()[I]; const auto& S = Tree->GetDynamics().GetLinkSettings(I);
				EXPECT(std::isfinite(Link->X.Length()) && std::isfinite(Link->V.Length()) && std::isfinite(Link->W.Length()));
				EXPECT_NEAR(Link->Q.SquareLength(), 1, 1e-5f);
				if (S.Parent < 0) continue;
				Vector3 Translation = Vector3::Zero();
				if (S.Joint == RagdollReducedJointType::Prismatic) Translation = S.Axis * (float)Tree->GetDynamics().GetConfiguration()[I].Position;
				const Vector3 ParentPoint = Tree->GetBodies()[S.Parent]->BodyLocalToWorld(S.ParentToJoint.LocalToWorld(Translation));
				MaximumJointError = std::max(MaximumJointError, (Link->BodyLocalToWorld(Vector3::Zero()) - ParentPoint).Length());
			}
			for (size_t I = 0; I < Gears->GetMechanismCount(); ++I) MaximumCouplingError = std::max(MaximumCouplingError, std::abs(Gears->GetMechanism(I)->GetCoordinate()));
			MaximumCableError = std::max(MaximumCableError, std::abs(Hard->GetMechanism(0)->GetCoordinate() - Hard->GetMechanism(0)->GetSettings().Target));
			MaximumExtension = std::max(MaximumExtension, Elastic->GetMechanism(0)->GetCoordinate() - Elastic->GetMechanism(0)->GetSettings().Target);
			EXPECT(Hard->GetMechanism(0)->GetForce() <= 0 && Elastic->GetMechanism(0)->GetForce() <= 0);
			const double Input = Gears->GetDynamics().GetConfiguration()[1].Position;
			MinimumInput = std::min(MinimumInput, Input); MaximumInput = std::max(MaximumInput, Input);
		}
		printf("Mechanism demo gear error=%.8f cable error=%.8f elastic stretch=%.6f input=[%.4f, %.4f]\n",
			MaximumCouplingError, MaximumCableError, MaximumExtension, MinimumInput, MaximumInput);
		EXPECT(MaximumCouplingError < 1e-5); EXPECT(MaximumCableError < 1e-4); EXPECT(MaximumExtension > 0.01 && MaximumExtension < 0.1);
		EXPECT(MinimumInput < -0.5 && MaximumInput > 0.5);
		for (int Mode = 0; Mode < 2; ++Mode)
			for (const auto& Object : Scene.GetObjects()) if (Object.Id == "mechanism_cable_" + std::to_string(Mode) + "_surface")
			{
				const auto* Mesh = Object.GeometryPtr->GetShapeObj<TriangleMesh>(); EXPECT(Mesh->NumVertices == 50 && Mesh->NumTriangles == 64);
				EXPECT(!Object.GeometryPtr->IsSimulationEnabled() && !Object.GeometryPtr->IsQueryEnabled());
				for (uint32_t I = 0; I < Mesh->NumVertices; ++I) { EXPECT(std::isfinite(Mesh->mVertices[I].Length())); EXPECT_NEAR(Mesh->mNormals[I].Length(), 1, 1e-5f); }
				const auto* Owner = Mode == 0 ? Hard : Elastic;
				EXPECT_NEAR((Mesh->mVertices[32] - Owner->GetBodies()[1]->X).Length(), 0, 1e-5f);
				EXPECT_NEAR((Mesh->mVertices[41] - Owner->GetBodies()[2]->X).Length(), 0, 1e-5f);
			}
		printf("Ragdoll demo patches=%d joint error=%.7f arm=(%.4f, %.4f, %.4f) freeY=%.4f\n", Patches, MaximumJointError,
			Arm->GetDynamics().GetConfiguration()[0].Position, Arm->GetDynamics().GetConfiguration()[1].Position,
			Arm->GetDynamics().GetConfiguration()[2].Position, FreeRoot->X.y);
		EXPECT_NEAR((ChainRoot->X - Anchor).Length(), 0, 1e-6f); EXPECT(MaximumJointError < 1e-4f);
		EXPECT(Arm->GetDynamics().GetConfiguration()[0].Position > 0.2);
		EXPECT(Arm->GetDynamics().GetConfiguration()[1].Position < -0.4);
		EXPECT(Arm->GetDynamics().GetConfiguration()[2].Position >= 0 && Arm->GetDynamics().GetConfiguration()[2].Position <= 0.601);
		for (auto* Link : Free->GetBodies()) EXPECT(Link->X.y > -2.1f && Link->X.y < -1.0f);
		const double DrivenAngle = Arm->GetDynamics().GetConfiguration()[0].Position;
		World->SetJointDrivesEnabled(false); for (int Frame = 0; Frame < 90; ++Frame) Scene.Step(FrameTime);
		EXPECT(std::abs(Arm->GetDynamics().GetConfiguration()[0].Position - DrivenAngle) > 0.05);
		World->SetJointDrivesEnabled(true);
		// A projectile uses the ordinary scene interaction and transfers its impulse
		// into the entire floating tree through the same discrete contact pipeline.
		const Vector3 Position = FreeRoot->X; const float PreviousZ = Position.z;
		constexpr float ProjectileRadius = 0.1f, ProjectileMass = 1;
		auto* Projectile = Scene.AddProjectileSphere("ragdoll_projectile", Position - Vector3(0, 0, 2), ProjectileRadius, Vector3(0, 0, 8), Vector4(1))->GetParent<RigidBody>();
		// Give the small projectile a specified mass while scaling its inertia by
		// the same density ratio. Its footprint clears the floor before impact.
		Projectile->InvInertia = Projectile->InvInertia * (1 / (ProjectileMass * Projectile->InvMass));
		Projectile->InvMass = 1 / ProjectileMass;
		for (int Frame = 0; Frame < 60; ++Frame) Scene.Step(FrameTime);
		printf("Ragdoll projectile displacement=%.5f speed=%.5f\n", FreeRoot->X.z - PreviousZ, Projectile->V.z);
		EXPECT(FreeRoot->X.z > PreviousZ + 0.05f);
	}
	Scene.SetReducedRagdollDemo(false); EXPECT(Scene.LoadFromFile(Path, &Error));
	EXPECT(Scene.GetSimulation()->GetReducedRagdollCount() == 0); EXPECT(Scene.GetObjects().size() == OriginalObjects);
	Scene.SetReducedRagdollDemo(true);
	EXPECT(Scene.LoadFromText("{\"name\":\"Other\",\"objects\":[]}", "other", &Error)); EXPECT(Scene.GetSimulation()->GetReducedRagdollCount() == 0);
	EXPECT(!Scene.LoadFromText("{\"name\":\"Constraints\",\"physics\":{\"resolutionSolver\":\"xpbd\"},\"objects\":[]}", "unsupported", &Error));
	EXPECT(!Error.empty());
	EXPECT(Scene.LoadFromFile(Path, &Error)); EXPECT(Scene.GetSimulation()->GetReducedRagdollCount() == 6);
	// Removing a tendon or its owner must not leave a dangling render/drive source.
	auto* Cable = Find("mechanism_cable_1_0")->ReducedRagdoll; EXPECT(Cable->RemoveMechanism(Cable->GetMechanism(0)));
	Scene.Step(1.0f / 60);
	for (const auto& Object : Scene.GetObjects()) if (Object.Id == "mechanism_cable_1_surface") EXPECT(Object.GeometryPtr->GetShapeObj<TriangleMesh>()->NumVertices == 0);
	EXPECT(Scene.GetSimulation()->RemoveRagdoll(Cable)); Scene.Step(1.0f / 60);
	Scene.Reset(); EXPECT(Scene.GetSimulation()->GetReducedRagdollCount() == 0);
}
