#include "Test.h"
#include "../Src/RigidBodyDynamics/PhysicalAnimation.h"
#include "../Viewer/SceneWorld.h"
#include "../Src/RigidBodyDynamics/PhysicsWorld.h"
#include "../Src/RigidBodyDynamics/Ragdoll.h"
#include "../Src/Collision/GeometryObject.h"
#include <algorithm>
#include <limits>
#include <stdexcept>
#if !defined(_WIN32)
#include <unistd.h>
#endif

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

void TestPhysicalAnimationDemo()
{
	using namespace Riemann;
	constexpr float FrameTime = 1.0f / 60;
	const Vector3 ComparisonOffset(3.6f, 0, 0);
	SceneWorld Scene; std::string Error; const std::string Path = TestRepositoryPath() + "Contents/Scenes/09_ragdoll_stairs.json";
	// The scene file and built-in ragdoll definition must resolve together even
	// when a Viewer executable is launched from a nested build directory.
	{
		char WorkingDirectory[4096];
#if defined(_WIN32)
		const bool Located = _getcwd(WorkingDirectory, sizeof(WorkingDirectory)) != nullptr;
		auto ChangeDirectory = [](const std::string& Directory) { return _chdir(Directory.c_str()) == 0; };
#else
		const bool Located = getcwd(WorkingDirectory, sizeof(WorkingDirectory)) != nullptr;
		auto ChangeDirectory = [](const std::string& Directory) { return chdir(Directory.c_str()) == 0; };
#endif
		EXPECT(Located);
		if (Located)
		{
			const std::string Original(WorkingDirectory);
			const std::string AbsoluteScene = Original + "/" + Path;
			std::string Nested = Original + "/" + TestOutputPath("");
			bool Created = true;
			for (const char* Part : { "RagdollPath", "A", "B", "C" })
			{
				Nested += Part;
				Created &= TestCreateDirectory(Nested);
				Nested += "/";
			}
			EXPECT(Created);
			if (Created)
			{
				const bool Changed = ChangeDirectory(Nested);
				EXPECT(Changed);
				if (Changed)
				{
					{
						SceneWorld NestedScene; std::string NestedError;
						const bool Loaded = NestedScene.LoadFromFile(AbsoluteScene, &NestedError);
						if (!Loaded) printf("Ragdoll 09 nested load error: %s\n", NestedError.c_str());
						EXPECT(Loaded);
						if (Loaded)
						{
							EXPECT(NestedScene.GetSimulation()->GetRagdollCount() == 1);
							NestedScene.SetPhysicalAnimationDemo(true);
							EXPECT(NestedScene.LoadFromFile(AbsoluteScene, &NestedError));
							EXPECT(NestedScene.GetSimulation()->GetRagdollCount() == 2);
							EXPECT(NestedScene.GetDemoPhysicalAnimation());
						}
					}
					EXPECT(ChangeDirectory(Original));
				}
			}
		}
	}
	const bool Loaded = Scene.LoadFromFile(Path, &Error);
	if (!Loaded) printf("Ragdoll 09 load error: %s\n", Error.c_str());
	EXPECT(Loaded); if (!Loaded) return;
	const size_t OriginalObjects = Scene.GetObjects().size();
	EXPECT(!Scene.GetDemoPhysicalAnimation()); EXPECT(Scene.GetSimulation()->GetRagdollCount() == 1);
	size_t RenderedBones = 0;
	for (const auto& Object : Scene.GetObjects()) if (Object.Id.find("falling_ragdoll_bone_") == 0)
	{
		EXPECT(Object.RenderEnabled && Object.GeometryPtr && Object.Color.w > 0);
		++RenderedBones;
	}
	EXPECT(RenderedBones == (size_t)RagdollBone::Count);
	Scene.SetPhysicalAnimationDemo(true); EXPECT(Scene.LoadFromFile(Path, &Error));
	auto* Animation = Scene.GetDemoPhysicalAnimation(); EXPECT(Animation); if (!Animation) return;
	auto* Falling = Scene.GetSimulation()->GetRagdoll(0)->GetBone(RagdollBone::Pelvis);
	const float FallingStartHeight = Falling->X.y; EXPECT(FallingStartHeight > 10);
	std::vector<PointConstraint*> Joints;
	for (auto* Joint : Scene.GetSimulation()->GetRagdoll(1)->GetConstraints()) Joints.push_back(static_cast<PointConstraint*>(Joint));
	EXPECT(Scene.GetSimulation()->GetRagdollCount() == 2);
	EXPECT(Scene.GetObjects().size() == OriginalObjects + 2 * (size_t)RagdollBone::Count);
	EXPECT(Scene.GetSimulation()->GetTemporalSubstepSettings().Enabled);
	struct AnchorPair { size_t A, B; Vector3 LocalA, LocalB; };
	std::vector<AnchorPair> Anchors;
	const auto Initial = Animation->GetPhysicsPose(); const auto& Bones = Animation->GetBones();
	for (auto* Joint : Joints)
	{
		auto* Point = static_cast<PointConstraint*>(Joint); size_t A = 0, B = 0;
		for (size_t I = 0; I < Bones.size(); ++I) { if (Bones[I].Body == Joint->GetBodyA()) A = I; if (Bones[I].Body == Joint->GetBodyB()) B = I; }
		Anchors.push_back({ A, B, Initial[A].WorldToLocal(Point->GetWorldAnchorA()), Initial[B].WorldToLocal(Point->GetWorldAnchorB()) });
	}
	float MaximumJointError = 0, MaximumTargetError = 0;
	for (int Frame = 0; Frame < 240; ++Frame)
	{
		Scene.Step(FrameTime); const auto& Target = Animation->GetAnimationPose();
		for (const auto& Anchor : Anchors)
			MaximumTargetError = std::max(MaximumTargetError, (Target[Anchor.A] * Anchor.LocalA - Target[Anchor.B] * Anchor.LocalB).Length());
		for (auto* Joint : Joints)
		{
			auto* Point = static_cast<PointConstraint*>(Joint);
			MaximumJointError = std::max(MaximumJointError, (Point->GetWorldAnchorA() - Point->GetWorldAnchorB()).Length());
		}
		for (const auto& Bone : Bones)
		{
			EXPECT(std::isfinite(Bone.Body->X.Length()) && std::isfinite(Bone.Body->V.Length()) && std::isfinite(Bone.Body->W.Length()));
			EXPECT(Bone.Body->X.y > -1.6f); EXPECT_NEAR(Bone.Body->Q.SquareLength(), 1, 1e-5f);
		}
	}
	EXPECT(Falling->X.y < FallingStartHeight - 1);
	printf("Physical animation demo joint error=%.6f sample anchor error=%.6f pelvis=%.5f\n", MaximumJointError, MaximumTargetError, Bones[0].Body->X.y);
	EXPECT(MaximumTargetError < 1e-4f); EXPECT(MaximumJointError < 0.05f); EXPECT(Bones[0].Body->X.y > 0.3f);
	EXPECT(Angle(Initial[(size_t)RagdollBone::RightLowerArm].quat, Animation->GetAnimationPose()[(size_t)RagdollBone::RightLowerArm].quat) > 0.2f);
	auto CheckPresentation = [&](float Blend)
	{
		const auto Before = Animation->GetPhysicsPose(); EXPECT(Scene.ApplyPhysicalAnimationDemo(false, 1, Blend));
		const auto Pose = Animation->GetBlendedPose();
		for (size_t I = 0; I < Bones.size(); ++I)
		{
			EXPECT(Bones[I].Body->X == Before[I].pos && Bones[I].Body->Q == Before[I].quat);
			const auto& Expected = Blend == 1 ? Before[I] : Animation->GetTargetPose()[I];
			EXPECT_NEAR((Pose[I].pos - Expected.pos).Length(), 0, 1e-5f);
			EXPECT(Angle(Pose[I].quat, Expected.quat) < 1e-3f);
			const std::string Name = "physical_animation_blend_" + std::to_string(I) + "_0"; bool Found = false;
			for (const auto& Object : Scene.GetObjects()) if (Object.Id == Name)
			{
				Found = true; auto* Copy = Object.GeometryPtr;
				EXPECT(!Copy->IsQueryEnabled() && !Copy->IsSimulationEnabled() && !Copy->GetParent<RigidBody>());
				const Transform Local = Before[I].TransformInv(Bones[I].Body->GetGeometryTransform(Bones[I].Body->Geometries()[0]));
				const Transform World = Pose[I] * Local;
				EXPECT_NEAR((Copy->GetPosition() - World.pos - ComparisonOffset).Length(), 0, 1e-5f);
				EXPECT(Angle(Copy->GetRotation(), World.quat) < 1e-3f);
			}
			EXPECT(Found);
		}
	};
	CheckPresentation(0); CheckPresentation(1);
	EXPECT(Scene.ApplyPhysicalAnimationDemo(false, 0, 0.5f));
	EXPECT(Animation->GetControl((size_t)RagdollBone::LeftUpperArm).Strength == 0);
	EXPECT(Animation->GetControl((size_t)RagdollBone::LeftLowerArm).Strength == 0);
	EXPECT(Animation->GetControl((size_t)RagdollBone::RightUpperArm).Strength == 1);
	EXPECT(!Scene.ApplyPhysicalAnimationDemo(false, -1, 0.5f));
	EXPECT(!Scene.ApplyPhysicalAnimationDemo(false, 1, std::numeric_limits<float>::quiet_NaN()));
	EXPECT(Scene.ApplyPhysicalAnimationDemo(true, 1, 0.5f));
	for (int Frame = 0; Frame < 150; ++Frame) Scene.Step(FrameTime);
	const float FallenHeight = Bones[0].Body->X.y; EXPECT(FallenHeight < Initial[0].pos.y - 0.3f);
	const auto FallenPose = Animation->GetPhysicsPose(); EXPECT(Scene.ApplyPhysicalAnimationDemo(false, 1, 0.5f));
	EXPECT(Animation->GetState() == PhysicalAnimationState::Recovering); EXPECT(Bones[0].Body->X == FallenPose[0].pos);
	for (int Frame = 0; Frame < 360; ++Frame) Scene.Step(FrameTime);
	printf("Physical animation demo fall=%.5f recovered=%.5f\n", FallenHeight, Bones[0].Body->X.y);
	EXPECT(Animation->GetState() == PhysicalAnimationState::Active); EXPECT(Bones[0].Body->X.y > 0.3f);
	Scene.GetSimulation()->SetJointDrivesEnabled(false); Scene.Step(FrameTime);
	EXPECT_NEAR(Animation->GetReaction(0).Force.Length(), 0, 1e-8f);
	Scene.GetSimulation()->SetJointDrivesEnabled(true);
	auto* Removed = Animation->GetBones()[(size_t)RagdollBone::LeftLowerArm].Body;
	EXPECT(Scene.GetSimulation()->RemoveRigidBody(Removed)); Removed->ReleaseGeometries(); delete Removed; Scene.Step(FrameTime);
	EXPECT(!Scene.GetDemoPhysicalAnimation()); EXPECT(Scene.GetObjects().size() == OriginalObjects);
	EXPECT(Scene.LoadFromFile(Path, &Error)); EXPECT(Scene.GetDemoPhysicalAnimation());
	Scene.SetPhysicalAnimationDemo(false); EXPECT(Scene.LoadFromFile(Path, &Error));
	EXPECT(!Scene.GetDemoPhysicalAnimation()); EXPECT(Scene.GetObjects().size() == OriginalObjects);
	Scene.SetPhysicalAnimationDemo(true); EXPECT(Scene.LoadFromText("{\"name\":\"Other\",\"objects\":[]}", "other", &Error));
	EXPECT(!Scene.GetDemoPhysicalAnimation()); Scene.Reset();

	// Rendering weights cannot alter the physical trajectory, even while sampled
	// targets and contacts evolve. Both worlds receive exactly the same controls.
	SceneWorld A, B; A.SetPhysicalAnimationDemo(true); B.SetPhysicalAnimationDemo(true);
	EXPECT(A.LoadFromFile(Path, &Error)); EXPECT(B.LoadFromFile(Path, &Error));
	EXPECT(A.ApplyPhysicalAnimationDemo(false, 1, 0)); EXPECT(B.ApplyPhysicalAnimationDemo(false, 1, 1));
	for (int Frame = 0; Frame < 120; ++Frame) { A.Step(FrameTime); B.Step(FrameTime); }
	const auto PA = A.GetDemoPhysicalAnimation()->GetPhysicsPose(), PB = B.GetDemoPhysicalAnimation()->GetPhysicsPose();
	for (size_t I = 0; I < PA.size(); ++I)
	{
		EXPECT_NEAR((PA[I].pos - PB[I].pos).Length(), 0, 1e-6f); EXPECT(Angle(PA[I].quat, PB[I].quat) < 1e-3f);
	}
}
