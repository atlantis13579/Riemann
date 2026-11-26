#include "Test.h"
#include "../Src/RigidBodyDynamics/Ragdoll.h"
#include "../Src/RigidBodyDynamics/Island.h"
#include "../Src/RigidBodyDynamics/Jacobian.h"
#include "../Src/RigidBodyDynamics/PhysicsWorld.h"
#include "../Src/Collision/GeometryObject.h"
#include "../Src/CollisionPrimitive/TriangleMesh.h"
#include "../Src/Deformation/Rope.h"
#include <cmath>
#include <stdexcept>
#include <limits>
#include "../Src/RigidBodyDynamics/PhysicalAnimation.h"
#include <algorithm>
#if !defined(_WIN32)
#include <unistd.h>
#endif

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

namespace Test_RagdollMechanism_Cases
{
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
}

void TestRagdollMechanism() { Test_RagdollMechanism_Cases::TestRagdollMechanism(); }

namespace Test_PhysicalAnimation_Cases
{
namespace
{
	using namespace Riemann;
	constexpr float Dt = 0.01f, Pi = 3.14159265358979323846f;
	PhysicsWorldParam Parameters()
	{
		PhysicsWorldParam P; P.gravityAcc = Vector3::Zero(); P.narrowphase = NarrowPhaseSolver::PCM;
		P.constraintIterations = 24; P.sequentialImpulseSettings.PositionIterations = 32; P.sequentialImpulseSettings.VelocityIterations = 16; return P;
	}
	RigidBodyDynamic* Body(PhysicsWorld& World, const Vector3& Position = Vector3::Zero(), bool Collider = false)
	{
		RigidBodyParam P; P.rigidType = RigidType::Dynamic; P.linearDamping = P.angularDamping = 1; P.sleepThreshold = 0;
		auto* B = (Collider ? World.CreateRigidBody(GeometryFactory::CreateSphere(Position, 0.25f), P)
			: World.CreateRigidBody(P, Transform(Position)))->CastDynamic();
		B->InvMass = 1; B->InvInertia = Matrix3(1, 1, 1); B->SetRestitution(0); return B;
	}
	void Step(PhysicsWorld& World, int Count) { for (int I = 0; I < Count; ++I) World.Simulate(Dt); }
	float Angle(const Quaternion& A, const Quaternion& B)
	{
		Quaternion Q = (A * B.Conjugate()).Unit(); if (Q.w < 0) Q = -Q;
		return 2 * std::atan2(Vector3(Q.x, Q.y, Q.z).Length(), Q.w);
	}
	void PoseDriveCaps()
	{
		for (auto Solver : { ResolutionPhaseSolver::SequentialImpulse, ResolutionPhaseSolver::LCPGlobal, ResolutionPhaseSolver::XPBD, ResolutionPhaseSolver::PBD })
			for (int Substeps : { 1, 4 })
		{
			auto P = Parameters(); P.temporalSubsteps.Enabled = true; P.temporalSubsteps.Substeps = Substeps; PhysicsWorld World(P);
			std::string Error; EXPECT(World.SetSolverConfiguration(Solver, SolverBackend::CPU, false, &Error));
			auto* Reference = World.CreateRigidBody(RigidBodyParam(), Transform::Identity()); auto* B = Body(World);
			auto* Drive = World.CreatePoseDriveConstraint(Reference, B); PoseDriveSettings S;
			S.Linear.PositionEnabled = true; S.Linear.Stiffness = 100; S.Linear.Damping = 10; S.Linear.MaxForce = 2; S.Target.pos.x = 10;
			S.Angular = S.Linear; S.Angular.MaxForce = 3; S.Target.quat.FromRotateY(Pi / 2); Drive->SetDrive(S);
			Step(World, 20); EXPECT_NEAR(B->V.x, 0.4f, 1e-5f); EXPECT_NEAR(B->W.y, 0.6f, 1e-5f);
			EXPECT_NEAR(Drive->GetReaction().Force.x, 2, 1e-4f); EXPECT_NEAR(Drive->GetReaction().Torque.Length(), 3, 1e-4f);
			const auto Before = Transform(B->X, B->Q); Drive->SolvePosition(Dt); EXPECT(B->X == Before.pos && B->Q == Before.quat);
		}
	}
	void ParentResponseAndProfiles()
	{
		PhysicsWorld World(Parameters()); auto* A = Body(World); auto* B = Body(World, Vector3(2, 0, 0)); auto* C = Body(World, Vector3(3, 0, 0));
		auto* Animation = World.CreatePhysicalAnimation({ { A, -1 }, { B, 0 }, { C, 1 } });
		auto Pose = Animation->GetPhysicsPose(); Pose[1].quat.FromRotateZ(1); Pose[2].quat = Pose[1].quat;
		Animation->SetAnimationPose(Pose); Step(World, 300);
		EXPECT(Angle(A->Q.Conjugate() * B->Q, Pose[1].quat) < 1e-4f);
		EXPECT_NEAR((A->W + B->W + C->W).Length(), 0, 1e-5f);
		EXPECT(Angle(A->Q, Quaternion::One()) > 0.1f); EXPECT(A->X == Vector3::Zero());
		auto Control = Animation->GetControl(1); Control.Enabled = false; Control.PhysicsBlend = 0.25f;
		Animation->SetControlBelow(1, Control, false); EXPECT(Animation->GetControl(1).Enabled); EXPECT(!Animation->GetControl(2).Enabled);
		Animation->SetControlBelow(1, Control); EXPECT(!Animation->GetControl(1).Enabled); EXPECT(Animation->GetControl(0).Enabled);
		Step(World, 1); EXPECT_NEAR(Animation->GetReaction(1).Torque.Length(), 0, 1e-8f);
		Control = PhysicalAnimationControl(); Control.Space = PhysicalAnimationSpace::World; Animation->SetControl(1, Control);
		Step(World, 300); EXPECT(Angle(B->Q, Pose[1].quat) < 1e-4f);
		EXPECT(World.RemovePhysicalAnimation(Animation)); EXPECT(World.GetPhysicalAnimationCount() == 0); EXPECT(World.GetConstraintCount() == 0);
	}
	void HierarchicalSamplingAndBlending()
	{
		PhysicsWorld World(Parameters()); auto* A = Body(World); auto* B = Body(World, Vector3(2, 0, 0));
		auto* Animation = World.CreatePhysicalAnimation({ { A, -1 }, { B, 0 } }); Animation->SetStrength(0);
		auto Pose = Animation->GetPhysicsPose(); Pose[0].quat.FromRotateZ(Pi / 2); Pose[1] = Pose[0] * Transform(Vector3(2, 0, 0));
		Animation->SetAnimationPose(Pose, 1); Step(World, 50);
		EXPECT_NEAR(Angle(Animation->GetAnimationPose()[0].quat, Quaternion::One()), Pi / 4, 1e-5f);
		EXPECT_NEAR((Animation->GetAnimationPose()[1].pos - Animation->GetAnimationPose()[0].pos).Length(), 2, 1e-5f);
		EXPECT_NEAR(Animation->GetAnimationPose()[1].pos.x, std::sqrt(2.0f), 1e-5f);
		Animation->SetAnimationPose(Pose);
		auto Control = Animation->GetControl(0); Control.PhysicsBlend = 0.5f; Animation->SetControlBelow(0, Control);
		const auto Blended = Animation->GetBlendedPose();
		EXPECT_NEAR(Angle(Blended[0].quat, Quaternion::One()), Pi / 4, 1e-5f);
		EXPECT_NEAR(Blended[1].pos.x, std::sqrt(2.0f), 1e-5f); EXPECT_NEAR((Blended[1].pos - Blended[0].pos).Length(), 2, 1e-5f);
		EXPECT(A->X == Vector3::Zero() && A->Q == Quaternion::One()); EXPECT(B->X == Vector3(2, 0, 0) && B->V == Vector3::Zero());
		Animation->SetActive(false); EXPECT(Animation->GetBlendedPose()[1].pos == B->X);
		Control.PhysicsBlend = 0; Animation->SetControlBelow(0, Control); Animation->BeginRecovery(1);
		const auto Initial = Animation->GetBlendedPose(); EXPECT(Initial[0].quat == A->Q && Initial[1].pos == B->X);
		Step(World, 50); EXPECT_NEAR(Animation->GetRecoveryProgress(), 0.5f, 1e-5f);
		EXPECT_NEAR(Angle(Animation->GetTargetPose()[0].quat, Quaternion::One()), Pi / 4, 1e-5f);
		Animation->BeginRecovery(0.2f); EXPECT_NEAR(Animation->GetRecoveryProgress(), 0, 1e-8f);
		EXPECT(Animation->GetTargetPose()[0].quat == A->Q); Step(World, 21); EXPECT(Animation->GetState() == PhysicalAnimationState::Active);
		EXPECT(Angle(Animation->GetBlendedPose()[0].quat, Pose[0].quat) < 1e-5f);
	}
	void PassiveJointLimits()
	{
		PhysicsWorld World(Parameters()); auto* Reference = World.CreateRigidBody(RigidBodyParam(), Transform::Identity()); auto* B = Body(World);
		auto* Hinge = World.CreateHingeConstraint(Reference, B, Vector3::Zero(), Vector3::UnitZ(), -0.2f, 0.2f);
		auto* Animation = World.CreatePhysicalAnimation({ { B, -1 } }); auto Control = Animation->GetControl(0);
		Control.Space = PhysicalAnimationSpace::World; Control.Angular.Acceleration = false; Control.Angular.Stiffness = 200;
		Control.Angular.Damping = 20; Control.Angular.MaxForce = 1; Animation->SetControl(0, Control);
		auto Pose = Animation->GetPhysicsPose(); Pose[0].quat.FromRotateZ(1); Animation->SetAnimationPose(Pose); Step(World, 200);
		EXPECT_NEAR(Hinge->GetCurrentAngle(), Hinge->GetMaxAngle(), 1e-4f); EXPECT(B->W.Length() < 1e-4f);
		EXPECT(Animation->GetReaction(0).Torque.Length() <= Control.Angular.MaxForce + 1e-4f);
		Animation->SetActive(false); Step(World, 1); EXPECT_NEAR(Animation->GetReaction(0).Torque.Length(), 0, 1e-8f);
		EXPECT(Hinge->IsEnabled());
	}
	void FeedForwardAndContacts()
	{
		PhysicsWorld World(Parameters()); auto* B = Body(World); auto* Animation = World.CreatePhysicalAnimation({ { B, -1 } });
		auto Control = Animation->GetControl(0); Control.Space = PhysicalAnimationSpace::World;
		Control.Angular.PositionEnabled = Control.Angular.VelocityEnabled = false;
		Control.Linear.VelocityEnabled = true; Control.Linear.Damping = 100; Animation->SetControl(0, Control);
		auto Pose = Animation->GetPhysicsPose(); Pose[0].pos.x = 1; Animation->SetAnimationPose(Pose, 1); Step(World, 50);
		EXPECT_NEAR(B->V.x, 1, 1e-4f); Step(World, 100); EXPECT(std::abs(B->V.x) < 1e-5f);
		Pose[0].pos.x = 100; Animation->SetAnimationPose(Pose); Step(World, 1); EXPECT(std::abs(B->V.x) < 1e-5f);
		Animation->BeginRecovery(2); Step(World, 20); const float MovingSpeed = B->V.x; EXPECT(MovingSpeed > 0);
		Pose[0].pos.x = 10000; Animation->SetAnimationPose(Pose); Step(World, 1); EXPECT(B->V.x <= MovingSpeed);
		for (bool Unified : { false, true })
		{
			auto P = Parameters(); P.unifiedConstraints = Unified; P.frictionPatches.Enabled = Unified; PhysicsWorld ContactWorld(P);
			ContactWorld.CreateRigidBody(GeometryFactory::CreateOBB(Vector3(0, -0.5f, 0), Vector3(5, 0.5f, 5)), RigidBodyParam());
			auto* Ball = Body(ContactWorld, Vector3(0, 0.25f, 0), true); auto* Driver = ContactWorld.CreatePhysicalAnimation({ { Ball, -1 } });
			auto Settings = Driver->GetControl(0); Settings.Space = PhysicalAnimationSpace::World;
			Settings.Linear.PositionEnabled = true; Settings.Linear.MaxForce = 10; Driver->SetControl(0, Settings);
			auto Target = Driver->GetPhysicsPose(); Target[0].pos.y = -1; Driver->SetAnimationPose(Target); Step(ContactWorld, 100);
			EXPECT(Ball->X.y > 0.23f); EXPECT(std::abs(Ball->V.y) < 0.01f); EXPECT(std::abs(Driver->GetReaction(0).Force.y) <= 10.001f);
			Driver->SetActive(false); Step(ContactWorld, 1); EXPECT_NEAR(Driver->GetReaction(0).Force.Length(), 0, 1e-8f);
		}
	}
	void RagdollRecoveryAndLifetime()
	{
		auto P = Parameters(); P.gravityAcc = Vector3(0, -9.8f, 0); P.temporalSubsteps.Enabled = true; P.temporalSubsteps.Substeps = 4;
		PhysicsWorld World(P); RagdollSettings S; S.DisableSelfCollision = true;
		auto* Ragdoll = World.CreateRagdoll(S, Transform(Vector3(0, 4, 0))); EXPECT(Ragdoll); if (!Ragdoll) return;
		auto* Animation = World.CreatePhysicalAnimation(Ragdoll); const auto Rest = Animation->GetPhysicsPose();
		EXPECT(Animation->GetBones().size() == (size_t)RagdollBone::Count); EXPECT(World.GetConstraintCount() == 21);
		// World-root support carries the complete character. Use physical gains;
		// acceleration gains normalized by the pelvis alone would undersize it.
		auto Muscles = Animation->GetControl(0); Muscles.Angular.Acceleration = false;
		Muscles.Angular.Stiffness = 200; Muscles.Angular.Damping = 20;
		Animation->SetControlBelow(0, Muscles);
		auto Root = Animation->GetControl(0); Root.Space = PhysicalAnimationSpace::World;
		Root.Linear.PositionEnabled = Root.Linear.VelocityEnabled = true; Root.Linear.Acceleration = false;
		Root.Linear.Stiffness = 400; Root.Linear.Damping = 40; Root.Linear.MaxForce = 500;
		Root.Angular.Stiffness = 200; Root.Angular.Damping = 40; Root.Angular.MaxForce = 500; Animation->SetControl(0, Root);
		Step(World, 300); auto* Pelvis = Ragdoll->GetBone(RagdollBone::Pelvis);
		printf("Physical animation held pelvis=%.5f\n", Pelvis->X.y); EXPECT(Pelvis->X.y > 3.8f);
		Animation->SetActive(false); Step(World, 40); const float Fallen = Pelvis->X.y; EXPECT(Fallen < 3.5f);
		const auto BeforeRecovery = Animation->GetPhysicsPose(); Animation->BeginRecovery(1.5f);
		EXPECT(Pelvis->X == BeforeRecovery[0].pos); EXPECT(Animation->GetBlendedPose()[0].pos == Pelvis->X);
		Step(World, 500); EXPECT(Animation->GetState() == PhysicalAnimationState::Active);
		printf("Physical animation recovered pelvis=%.5f after fall=%.5f\n", Pelvis->X.y, Fallen); EXPECT(Pelvis->X.y > 3.8f);
		for (size_t I = 0; I < Rest.size(); ++I)
		{
			auto* Bone = Ragdoll->GetBodies()[I]; const float Error = Angle(Bone->Q, Rest[I].quat);
			printf("Physical animation bone=%zu orientation error=%.6f\n", I, Error);
			EXPECT(std::isfinite(Bone->X.Length())); EXPECT(Error < 0.1f);
		}
		for (auto* Joint : Ragdoll->GetConstraints())
		{
			auto* Point = dynamic_cast<PointConstraint*>(Joint); if (Point) EXPECT((Point->GetWorldAnchorA() - Point->GetWorldAnchorB()).Length() < 0.03f);
		}
		auto* Removed = Ragdoll->GetBone(RagdollBone::LeftLowerArm); EXPECT(World.RemoveRigidBody(Removed));
		EXPECT(World.GetPhysicalAnimationCount() == 0); EXPECT(!Ragdoll->IsValid()); EXPECT(Ragdoll->GetBone(RagdollBone::LeftLowerArm) == nullptr);
		Removed->ReleaseGeometries(); delete Removed; EXPECT(World.RemoveRagdoll(Ragdoll)); EXPECT(World.GetConstraintCount() == 0);
		auto* Other = World.CreateRagdoll(S, Transform::Identity()); World.CreatePhysicalAnimation(Other);
		EXPECT(World.RemoveConstraint(Other->GetConstraints().back())); EXPECT(!Other->IsValid()); EXPECT(World.RemoveRagdoll(Other));
		EXPECT(World.GetPhysicalAnimationCount() == 0 && World.GetConstraintCount() == 0);
		Other = World.CreateRagdoll(S, Transform::Identity()); World.CreatePhysicalAnimation(Other); World.Reset();
		EXPECT(World.GetPhysicalAnimationCount() == 0 && World.GetConstraintCount() == 0 && World.GetRagdollCount() == 0);
	}
	void ValidationAndSleeping()
	{
		auto P = Parameters(); P.islandSleeping.Enabled = true; P.islandSleeping.QuietTime = 0.03f; PhysicsWorld World(P);
		auto* B = Body(World); B->SleepThreshold = RigidBodyParam().sleepThreshold;
		auto Rejects = [](auto Function) { bool Rejected = false; try { Function(); } catch (const std::invalid_argument&) { Rejected = true; } EXPECT(Rejected); };
		Rejects([&] { World.CreatePhysicalAnimation({ { B, 0 } }); }); Rejects([&] { World.CreatePhysicalAnimation({ { B, -1 }, { B, 0 } }); });
		auto* Animation = World.CreatePhysicalAnimation({ { B, -1 } });
		Rejects([&] { World.CreatePhysicalAnimation({ { B, -1 } }); }); Rejects([&] { Animation->SetAnimationPose({}); });
		auto Pose = Animation->GetPhysicsPose(); Pose[0].pos.x = std::numeric_limits<float>::quiet_NaN();
		Rejects([&] { Animation->SetAnimationPose(Pose); }); Rejects([&] { Animation->BeginRecovery(-1); });
		auto Control = Animation->GetControl(0); Control.PhysicsBlend = 2; Rejects([&] { Animation->SetControl(0, Control); });
		Animation->SetActive(false); Step(World, 10); EXPECT(B->Sleeping);
		for (int I = 0; I < 10; ++I) { Animation->SetAnimationPose(Animation->GetPhysicsPose(), Dt); Step(World, 1); } EXPECT(B->Sleeping);
		Control = PhysicalAnimationControl(); Control.Space = PhysicalAnimationSpace::World; Animation->SetControl(0, Control);
		Animation->BeginRecovery(0.1f); EXPECT(!B->Sleeping); Step(World, 12); EXPECT(Animation->GetState() == PhysicalAnimationState::Active);
		Animation->SetStrength(0); Step(World, 10); EXPECT(B->Sleeping);
		for (int I = 0; I < 10; ++I) { Animation->SetAnimationPose(Animation->GetPhysicsPose(), Dt); Step(World, 1); } EXPECT(B->Sleeping);
		World.SetJointDrivesEnabled(false); Animation->SetStrength(1); Step(World, 10); EXPECT(B->Sleeping);
		for (int I = 0; I < 10; ++I) { Animation->SetAnimationPose(Animation->GetPhysicsPose(), Dt); Step(World, 1); } EXPECT(B->Sleeping);
		EXPECT(World.RemovePhysicalAnimation(Animation)); EXPECT(!World.RemovePhysicalAnimation(nullptr));
	}
}
void TestPhysicalAnimation()
{
	PoseDriveCaps(); ParentResponseAndProfiles(); HierarchicalSamplingAndBlending(); PassiveJointLimits(); FeedForwardAndContacts();
	RagdollRecoveryAndLifetime(); ValidationAndSleeping();
}
}

void TestPhysicalAnimation() { Test_PhysicalAnimation_Cases::TestPhysicalAnimation(); }
