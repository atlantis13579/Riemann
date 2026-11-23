#include "Test.h"
#include "../Src/Deformation/Cloth.h"
#include "../Src/Deformation/DeformableBody.h"
#include "../Src/Fluid/AerodynamicsSimulation.h"
#include "../Src/Fluid/AerodynamicsRigidCoupling.h"
#include "../Src/RigidBodyDynamics/PhysicsWorld.h"
#include "../Src/Collision/GeometryObject.h"
#include "../Src/Collision/GeometryQuery.h"
#include <stdexcept>

namespace
{
	using namespace Riemann;
	void StepFrame(PhysicsWorld& World, float Dt)
	{
		const auto Before = World.GetBroadPhasePassCount();
		World.Simulate(Dt);
		EXPECT(World.GetBroadPhasePassCount() == Before + 1);
	}
	PhysicsWorldParam WorldSettings()
	{
		PhysicsWorldParam P; P.resolutionSolver = ResolutionPhaseSolver::XPBD;
		P.gravityAcc = Vector3::Zero(); P.xpbdSettings.PositionIterations = 24; return P;
	}
	ClothSettings Material()
	{
		ClothSettings S; S.Model = ClothModel::TriangleStrain; S.Gravity = Vector3::Zero(); S.Damping = 1;
		S.Thickness = 0.02f; S.SolverIterations = 24; S.Collision.RigidCoupling = true;
		S.Collision.Material.StaticFriction = S.Collision.Material.DynamicFriction = 0; return S;
	}
	ClothMesh Triangle(float Z)
	{
		ClothMesh M; M.Positions = { Vector3(-1, -1, Z), Vector3(1, -1, Z), Vector3(0, 1, Z) };
		M.Triangles = { Index3(0, 1, 2) }; M.InverseMasses = { 1, 1, 1 }; return M;
	}
	RigidBodyDynamic* Ball(PhysicsWorld& W, const Vector3& P, float Radius = 0.5f)
	{
		RigidBodyParam R; R.rigidType = RigidType::Dynamic; R.linearDamping = R.angularDamping = 1;
		R.sleepThreshold = R.freezeThreshold = 0;
		return W.CreateRigidBody(GeometryFactory::CreateSphere(P, Radius), R)->CastDynamic();
	}
	Vector3 Moment(const ClothSystem& C, const RigidBody& B)
	{
		Vector3 Result = B.X / B.InvMass;
		for (int I = 0; I < C.GetParticleCount(); ++I) Result += C.GetParticle(I).Position / C.GetParticle(I).InverseMass;
		return Result;
	}
	void ContactReaction()
	{
		for (auto Solver : { ResolutionPhaseSolver::PBD, ResolutionPhaseSolver::XPBD })
		{
			auto P = WorldSettings(); P.resolutionSolver = Solver; PhysicsWorld W(P);
			auto S = Material(); S.Material.StretchEnabled = S.Material.ShearEnabled = S.Material.BendingEnabled = false;
			auto* C = W.CreateCloth(S, Triangle(0.48f)); auto* B = Ball(W, Vector3::Zero());
			const Vector3 Before = Moment(*C, *B); StepFrame(W, 1.0f / 120);
			EXPECT(C->GetCollisionStats().RigidContacts > 0); EXPECT(B->X.z < -0.005f);
			EXPECT_NEAR((Moment(*C, *B) - Before).Length(), 0, 2e-5f);
			Vector3 Momentum = B->V / B->InvMass;
			for (int I = 0; I < C->GetParticleCount(); ++I) Momentum += C->GetParticleVelocity(I) / C->GetParticle(I).InverseMass;
			EXPECT_NEAR(Momentum.Length(), 0, 0.003f);
			printf("Cloth rigid contact solver %d: rigid displacement %g, momentum error %g\n", (int)Solver, B->X.z, Momentum.Length());
		}
	}
	void AttachmentPullAndLifetime()
	{
		PhysicsWorld W(WorldSettings()); auto S = Material(); S.Collision.RigidCoupling = false; S.Substeps = 4; S.Damping = 0.98f;
		auto* C = W.CreateCloth(S, Triangle(0)); auto* B = Ball(W, C->GetParticle(2).Position, 0.1f);
		C->PinParticle(0, C->GetParticle(0).Position); C->PinParticle(1, C->GetParticle(1).Position);
		const auto Id = C->AttachToRigid(BarycentricPoint::Vertex(2), B, B->X);
		C->PinParticle(0, C->GetParticle(0).Position + Vector3(0, -0.35f, 0));
		C->PinParticle(1, C->GetParticle(1).Position + Vector3(0, -0.35f, 0));
		for (int I = 0; I < 60; ++I) StepFrame(W, 1.0f / 120);
		EXPECT(B->X.y < 0.9f); EXPECT((C->GetParticle(2).Position - B->X).Length() < 0.002f);
		printf("Cloth tension pulled attached ball to y=%g\n", B->X.y);
		std::string Error; EXPECT(!W.SetSolverConfiguration(ResolutionPhaseSolver::SequentialImpulse, SolverBackend::CPU, false, &Error));
		EXPECT(!Error.empty()); EXPECT(C->RemoveRigidAttachment(Id)); EXPECT(!C->RemoveRigidAttachment(Id));
		C->AttachToRigid(BarycentricPoint::Vertex(2), B, B->X);
		EXPECT(W.RemoveRigidBody(B)); EXPECT(C->GetRigidAttachmentCount() == 0); StepFrame(W, 1.0f / 120);
		W.Reset(); EXPECT(W.GetClothCount() == 0);
	}
	void BroadPhaseMotion()
	{
		PhysicsWorld W(WorldSettings()); auto S = Material();
		S.Material.StretchEnabled = S.Material.ShearEnabled = S.Material.BendingEnabled = false;
		auto* C = W.CreateCloth(S, Triangle(3)); auto* B = Ball(W, Vector3::Zero());
		StepFrame(W, 1.0f / 120); EXPECT(C->GetCollisionStats().RigidContacts == 0);
		for (int I = 0; I < C->GetParticleCount(); ++I) C->SetParticlePosition(I, Triangle(0.48f).Positions[I]);
		StepFrame(W, 1.0f / 120); EXPECT(C->GetCollisionStats().RigidContacts > 0); EXPECT(B->X.z < 0);
	}
	void OffsetAttachmentTorque()
	{
		PhysicsWorld W(WorldSettings()); auto S = Material(); S.Collision.RigidCoupling = false;
		auto* C = W.CreateCloth(S, Triangle(0)); auto* B = Ball(W, Vector3(0, 0, 0));
		const auto Initial = Moment(*C, *B);
		C->AttachToRigid(BarycentricPoint::Vertex(2), B, C->GetParticle(2).Position);
		C->ApplyParticleImpulse(2, Vector3(0, 0, 0.1f)); StepFrame(W, 1.0f / 120);
		EXPECT(B->V.z > 0); EXPECT(std::abs(B->W.x) > 0.001f);
		Vector3 Momentum = B->V / B->InvMass;
		for (int I = 0; I < C->GetParticleCount(); ++I) Momentum += C->GetParticleVelocity(I) / C->GetParticle(I).InverseMass;
		EXPECT_NEAR((Momentum - Vector3(0, 0, 0.1f)).Length(), 0, 0.002f);
		EXPECT_NEAR((Moment(*C, *B) - Initial - Vector3(0, 0, 0.1f / 120)).Length(), 0, 2e-5f);
	}
	void MixedClothColliders()
	{
		PhysicsWorld W(WorldSettings()); auto S = Material(); S.Collision.InterCollision = true;
		W.CreateCloth(S, Triangle(3));
		S.Collision.RigidCoupling = false; S.Collision.SurfaceContacts = true;
		auto* C = W.CreateCloth(S, Triangle(0.48f)); auto* B = Ball(W, Vector3::Zero());
		EXPECT(C->AddSphereCollider(B, 0.5f, B->X));
		StepFrame(W, 1.0f / 120);
		// Joining the shared cloth stage must preserve this cloth's prescribed collider.
		EXPECT(C->GetCollisionStats().Contacts > 0); EXPECT(C->GetCollisionStats().RigidContacts == 0);
		EXPECT(B->X == Vector3::Zero());
		const Vector3 Center = (C->GetParticle(0).Position + C->GetParticle(1).Position + C->GetParticle(2).Position) / 3;
		EXPECT(Center.z > 0.49f);
	}
	void SharedBroadPhase(BroadPhaseSolver Broad, ResolutionPhaseSolver Solver, bool ShapeOffsets)
	{
		auto P = WorldSettings(); P.broadphase = Broad; P.resolutionSolver = Solver;
		P.pbdSettings.PositionIterations = P.xpbdSettings.PositionIterations = 8;
		P.contactGeneration.UseShapeOffsets = ShapeOffsets;
		PhysicsWorld W(P);
		auto* A = Ball(W, Vector3::Zero()); auto* B = Ball(W, Vector3(4, 0, 0));
		StepFrame(W, 1.0f / 120);
		const uint64_t RigidPasses = W.GetBroadPhasePassCount();
		EXPECT(RigidPasses == 1);
		auto S = Material(); S.Substeps = 1; S.SolverIterations = 8;
		S.Material.StretchEnabled = S.Material.ShearEnabled = S.Material.BendingEnabled = false;
		auto* C = W.CreateCloth(S, Triangle(0.51f));
		DeformableBodySettings D; D.Gravity = Vector3::Zero(); D.GroundEnabled = false;
		D.Substeps = 1; D.SolverIterations = 8; D.ParticleRadius = 0.02f; D.Collision.RigidCollision = true;
		auto* Soft = W.CreateDeformableBody(D);
		for (const auto& V : Triangle(0.51f).Positions) Soft->AddParticle(V + Vector3(4, 0, 0));
		EXPECT(Soft->SetSurfaceTriangles({ Index3(0, 1, 2) }));
		StepFrame(W, 1.0f / 120);
		EXPECT(C->GetCollisionStats().RigidContacts > 0); EXPECT(Soft->GetCollisionStats().RigidContacts > 0);
		EXPECT(A->X.z < 0 && B->X.z < 0);
		// Adding both material systems must not add a per-system broad-phase pass.
		EXPECT(W.GetBroadPhasePassCount() - RigidPasses == RigidPasses);
		EXPECT(W.RemoveCloth(C)); EXPECT(W.RemoveDeformableBody(Soft));
		A->X = Vector3::Zero(); A->V = Vector3::Zero(); A->UpdateGeometries();
		C = W.CreateCloth(S, Triangle(3)); StepFrame(W, 1.0f / 120);
		EXPECT(C->GetCollisionStats().RigidContacts == 0);
		for (int I = 0; I < 3; ++I) C->SetParticlePosition(I, Triangle(0.51f).Positions[I]);
		StepFrame(W, 1.0f / 120); EXPECT(C->GetCollisionStats().RigidContacts > 0);
		printf("Shared broad phase=%d solver=%d offsets=%d passes=%llu\n", (int)Broad, (int)Solver,
			(int)ShapeOffsets, (unsigned long long)RigidPasses);
	}
	void SharedSurfaceFiltering(BroadPhaseSolver Broad)
	{
		auto P = WorldSettings(); P.broadphase = Broad;
		P.collisionFilter = std::shared_ptr<CollisionFilter>(CollisionFilter::CreateLayerMask());
		PhysicsWorld W(P); auto S = Material(); S.Collision.RigidCoupling = false; S.Collision.InterCollision = true;
		S.Substeps = 1; S.SolverIterations = 4;
		S.Material.StretchEnabled = S.Material.ShearEnabled = S.Material.BendingEnabled = false;
		auto* A = W.CreateCloth(S, Triangle(0));
		S.Collision.FilterData = CollisionData::ForLayer(1, 0);
		auto* B = W.CreateCloth(S, Triangle(0.03f));
		StepFrame(W, 1.0f / 120); EXPECT(A->GetCollisionStats().InterContacts == 0);
		S.Collision.FilterData = CollisionData::ForLayer(1); B->SetCollisionSettings(S.Collision);
		StepFrame(W, 1.0f / 120); EXPECT(A->GetCollisionStats().InterContacts > 0);
		EXPECT(B->GetCollisionStats().InterContacts > 0);
	}
	void SharedFieldLifetime(BroadPhaseSolver Broad)
	{
		auto P = WorldSettings(); P.broadphase = Broad; PhysicsWorld W(P);
		AerodynamicsSimulationSettings AirSettings(AerodynamicsAlgorithm::Incompressible);
		AirSettings.Backend = FluidBackend::CPU; AirSettings.Resolution = Vector3i(8, 8, 8);
		AirSettings.BoundsMin = Vector3(-1); AirSettings.BoundsMax = Vector3(1); AirSettings.Emitters.clear();
		AirSettings.MaxTimeStep = 1.0f / 120; AirSettings.PressureIterations = 8;
		auto* Air = W.CreateAerodynamics(AirSettings); EXPECT(Air); if (!Air) return;
		auto S = Material(); S.Substeps = 1; S.SolverIterations = 4;
		auto* C = W.CreateCloth(S, Triangle(3)); auto* B = Ball(W, Vector3(3, 0, 0));
		StepFrame(W, 1.0f / 120);
		auto Occupied = [&]() { size_t N = 0; for (auto Solid : Air->GetSolidCells()) N += Solid != 0; return N; };
		EXPECT(Occupied() == 0);
		B->X = Vector3::Zero(); B->UpdateGeometries();
		StepFrame(W, 1.0f / 120); EXPECT(Occupied() > 0);
		EXPECT(W.RemoveRigidBody(B)); B->ReleaseGeometries(); delete B;
		for (int I = 0; I < 3; ++I) C->SetParticlePosition(I, Triangle(0).Positions[I]);
		StepFrame(W, 1.0f / 120); EXPECT(Occupied() > 0);
		EXPECT(W.RemoveCloth(C)); StepFrame(W, 1.0f / 120); EXPECT(Occupied() == 0);
		EXPECT(W.RemoveAerodynamics()); Air = W.CreateAerodynamics(AirSettings);
		EXPECT(Air); StepFrame(W, 1.0f / 120); EXPECT(Occupied() == 0);
	}
	void BarotropicFieldFiltering(BroadPhaseSolver Broad)
	{
		auto P = WorldSettings(); P.broadphase = Broad;
		P.collisionFilter = std::shared_ptr<CollisionFilter>(CollisionFilter::CreateLayerMask());
		PhysicsWorld W(P);
		AerodynamicsSimulationSettings S(AerodynamicsAlgorithm::Barotropic);
		S.Backend = FluidBackend::CPU; S.Resolution = Vector3i(8, 8, 8);
		S.BoundsMin = Vector3(-1); S.BoundsMax = Vector3(1); S.Emitters.clear();
		EXPECT(W.CreateAerodynamics(S)); auto* B = Ball(W, Vector3::Zero());
		B->mGeometries[0]->SetFilterData(CollisionData::ForLayer(1, 0));
		StepFrame(W, 1.0f / 120); AerodynamicsRigidCouplingResult Load;
		W.GetAerodynamicsCoupling()->Evaluate(B, &Load); EXPECT(Load.SampleCount == 0);
		B->mGeometries[0]->SetFilterData(CollisionData::ForLayer(1));
		StepFrame(W, 1.0f / 120);
		EXPECT(W.GetAerodynamicsCoupling()->Evaluate(B, &Load)); EXPECT(Load.SampleCount > 0);
	}
	void FrameMotion(BroadPhaseSolver Broad, ResolutionPhaseSolver Solver)
	{
		for (bool Accelerating : { false, true })
		{
			auto P = WorldSettings(); P.broadphase = Broad; P.resolutionSolver = Solver;
			P.temporalSubsteps.Enabled = true; P.temporalSubsteps.Substeps = 8;
			P.gravityAcc = Accelerating ? Vector3(100, 0, 0) : Vector3::Zero();
			int Contacts = 0; P.onContact = [&](const PhysicsContactInfo&) { ++Contacts; };
			PhysicsWorld W(P);
			W.CreateRigidBody(GeometryFactory::CreateSphere(Vector3(3, 0, 0), 0.5f), RigidBodyParam())->SetRestitution(0);
			auto* B = Ball(W, Vector3::Zero()); B->V = Accelerating ? Vector3::Zero() : Vector3(20, 0, 0); B->SetRestitution(0);
			StepFrame(W, 0.25f);
			printf("Frame motion broad=%d solver=%d acceleration=%d contacts=%d x=%g\n", (int)Broad, (int)Solver, (int)Accelerating, Contacts, B->X.x);
			// SI/LCP retain their gradual penetration correction. This fixture checks
			// that later substeps detect and stop the impact, not their correction rate.
			EXPECT(Contacts > 0); EXPECT(B->X.x < 2.5f);
		}
	}
	void IndependentMaterials(BroadPhaseSolver Broad, ResolutionPhaseSolver Solver)
	{
		auto P = WorldSettings(); P.broadphase = Broad; P.resolutionSolver = Solver;
		P.temporalSubsteps.Enabled = true; P.temporalSubsteps.Substeps = 4;
		PhysicsWorld W(P);
		auto S = Material(); S.Collision.RigidCoupling = false; S.Collision.InterCollision = true;
		S.Substeps = 3; S.SolverIterations = 5;
		S.Material.StretchEnabled = S.Material.ShearEnabled = S.Material.BendingEnabled = false;
		W.CreateCloth(S, Triangle(0)); auto* C = W.CreateCloth(S, Triangle(0.2f));
		for (int I = 0; I < 3; ++I) C->SetParticleVelocity(I, Vector3(0, 0, -1));
		DeformableBodySettings D; D.Gravity = Vector3::Zero(); D.GroundEnabled = false;
		D.Substeps = 2; D.SolverIterations = 5; D.Collision.InterCollision = true;
		for (float Z : { 0.0f, 0.01f })
		{
			auto* B = W.CreateDeformableBody(D);
			for (const auto& V : Triangle(Z).Positions) B->AddParticle(V + Vector3(4, 0, 0));
			EXPECT(B->SetSurfaceTriangles({ Index3(0, 1, 2) }));
		}
		StepFrame(W, 0.2f);
		EXPECT(C->GetCollisionStats().InterContacts > 0);
	}
	void TransferredMotion(BroadPhaseSolver Broad, ResolutionPhaseSolver Solver)
	{
		auto P = WorldSettings(); P.broadphase = Broad; P.resolutionSolver = Solver;
		P.temporalSubsteps.Enabled = true; P.temporalSubsteps.Substeps = 16;
		PhysicsWorld W(P);
		auto* A = Ball(W, Vector3(-2, 0, 0)); auto* B = Ball(W, Vector3::Zero()); auto* C = Ball(W, Vector3(2, 0, 0));
		A->SetRestitution(1); B->SetRestitution(1); C->SetRestitution(1); A->V = Vector3(20, 0, 0);
		StepFrame(W, 0.3f); EXPECT(C->X.x > 2.1f);
	}
	void MovingCoupledSurface(BroadPhaseSolver Broad)
	{
		auto P = WorldSettings(); P.broadphase = Broad;
		P.temporalSubsteps.Enabled = true; P.temporalSubsteps.Substeps = 4;
		PhysicsWorld W(P); auto S = Material(); S.Substeps = 2;
		S.Material.StretchEnabled = S.Material.ShearEnabled = S.Material.BendingEnabled = false;
		auto* C = W.CreateCloth(S, Triangle(1.5f)); auto* B = Ball(W, Vector3::Zero());
		for (int I = 0; I < 3; ++I) C->SetParticleVelocity(I, Vector3(0, 0, -10));
		StepFrame(W, 0.15f); EXPECT(B->X.z < -0.01f);
	}
	void LateRegistration(BroadPhaseSolver Broad)
	{
		auto P = WorldSettings(); P.broadphase = Broad;
		P.temporalSubsteps.Enabled = true; P.temporalSubsteps.Substeps = 4;
		PhysicsWorld* World = nullptr; RigidBodyDynamic* Added = nullptr; bool AddedContact = false;
		P.onContact = [&](const PhysicsContactInfo& Info)
		{
			if (!Added) Added = Ball(*World, Vector3(4, 0.49f, 0));
			else if (Info.BodyA == Added || Info.BodyB == Added) AddedContact = true;
		};
		PhysicsWorld W(P); World = &W;
		W.CreateRigidBody(GeometryFactory::CreatePlane(Vector3::Zero(), Vector3::UnitY()), RigidBodyParam());
		Ball(W, Vector3(0, 0.49f, 0));
		StepFrame(W, 1.0f / 60); EXPECT(Added && AddedContact);
	}
	void ProjectedMotion(BroadPhaseSolver Broad)
	{
		auto P = WorldSettings(); P.broadphase = Broad; int Contacts = 0;
		P.onContact = [&](const PhysicsContactInfo&) { ++Contacts; };
		PhysicsWorld W(P);
		auto* Reference = W.CreateRigidBody(GeometryFactory::CreateSphere(Vector3::Zero(), 0.1f), RigidBodyParam());
		Reference->mGeometries[0]->SetSimulationEnabled(false);
		auto* B = Ball(W, Vector3(-5, 0, 0));
		W.AddConstraint(new FixedConstraint(Reference, B));
		W.CreateRigidBody(GeometryFactory::CreateSphere(Vector3(-5, 0.75f, 0), 0.5f), RigidBodyParam());
		B->X = Vector3::Zero(); B->UpdateGeometries();
		StepFrame(W, 1.0f / 60); EXPECT(Contacts > 0);
	}
	void CandidateViews()
	{
		BroadPhaseResult Result;
		constexpr int Rigids = 128, ClothA = Rigids, ClothB = Rigids + 1;
		constexpr int SoftA = Rigids + 2, SoftB = Rigids + 3, Air = Rigids + 4;
		Result.States.resize(Rigids + 5);
		Result.SetObject(ClothA, BroadPhaseObjectType::Cloth);
		Result.SetObject(ClothB, BroadPhaseObjectType::Cloth);
		Result.SetObject(SoftA, BroadPhaseObjectType::Deformable);
		Result.SetObject(SoftB, BroadPhaseObjectType::Deformable);
		Result.SetObject(Air, BroadPhaseObjectType::Aerodynamics);
		for (int I = 0; I < Rigids; ++I) for (int J = I + 1; J < Rigids; ++J) Result.Pairs.emplace_back(I, J);
		const size_t RigidCount = Result.Pairs.size();
		Result.Pairs.emplace_back(ClothB, ClothA);
		Result.Pairs.emplace_back(0, ClothA);
		Result.Pairs.emplace_back(SoftA, 1);
		Result.Pairs.emplace_back(SoftA, SoftB);
		Result.Pairs.emplace_back(Air, 0);
		Result.Pairs.emplace_back(ClothA, Air);
		// Cloth-soft interaction has no contact implementation and must not be
		// routed to either homogeneous surface detector.
		Result.Pairs.emplace_back(ClothA, SoftA);
		Result.DispatchPairs();
		EXPECT(Result.GetPairs(BroadPhasePairType::Rigid).size() == RigidCount);
		EXPECT(Result.GetPairs(BroadPhasePairType::Cloth).size() == 1);
		EXPECT(&Result.GetPairs(BroadPhasePairType::Cloth)[0] == &Result.Pairs[RigidCount]);
		EXPECT(Result.GetPairs(BroadPhasePairType::ClothRigid).size() == 1);
		EXPECT(Result.GetPairs(BroadPhasePairType::DeformableRigid).size() == 1);
		EXPECT(Result.GetPairs(BroadPhasePairType::Deformable).size() == 1);
		EXPECT(Result.GetPairs(BroadPhasePairType::Aerodynamics).size() == 2);
		BroadPhaseObjectMap<int> Mapping; int Lookups = 0;
		auto Bind = [&](int Index) { ++Lookups; return Index * 2; };
		Mapping.Update(Result, Bind);
		for (int Iteration = 0; Iteration < 24; ++Iteration)
		{
			Mapping.Update(Result, Bind);
			size_t Visits = 0;
			for (const auto& Pair : Result.GetPairs(BroadPhasePairType::Cloth))
			{
				++Visits; EXPECT(Mapping[Pair.index1] == ClothB * 2); EXPECT(Mapping[Pair.index2] == ClothA * 2);
			}
			EXPECT(Visits == 1);
		}
		EXPECT(Lookups == (int)Result.States.size());
		const size_t Begin = Result.Pairs.size(); const int Added = (int)Result.States.size();
		Result.States.emplace_back();
		Result.Pairs.emplace_back(Added, ClothB); Result.Pairs.emplace_back(Air, Added); Result.Pairs.emplace_back(Added, 1);
		Result.DispatchPairs(Begin); Mapping.Update(Result, Bind);
		EXPECT(Lookups == (int)Result.States.size()); EXPECT(Mapping[Added] == Added * 2);
		EXPECT(Result.GetPairs(BroadPhasePairType::ClothRigid).size() == 2);
		EXPECT(&Result.GetPairs(BroadPhasePairType::ClothRigid)[1] == &Result.Pairs[Begin]);
		EXPECT(Result.GetPairs(BroadPhasePairType::Aerodynamics).size() == 3);
		EXPECT(Result.GetPairs(BroadPhasePairType::Rigid).size() == RigidCount + 1);
		Result.Clear(); Result.States.resize(2); Result.Pairs.emplace_back(0, 1); Result.DispatchPairs();
		Mapping.Update(Result, Bind); EXPECT(Lookups == Added + 3);
		const auto PureRigid = Result.GetPairs(BroadPhasePairType::Rigid);
		EXPECT(PureRigid.size() == 1 && PureRigid.Indices == nullptr);
		EXPECT(&PureRigid[0] == &Result.Pairs[0]); EXPECT(Result.GetPairs(BroadPhasePairType::Cloth).empty());
	}
	void ReplacedFieldGeometry(BroadPhaseSolver Broad)
	{
		auto P = WorldSettings(); P.broadphase = Broad;
		P.temporalSubsteps.Enabled = true; P.temporalSubsteps.Substeps = 4;
		PhysicsWorld* World = nullptr; RigidBody* Obstacle = nullptr; bool Replaced = false;
		GeometryHandle Previous;
		P.onContact = [&](const PhysicsContactInfo&)
		{
			if (Replaced) return;
			Replaced = true;
			Previous = Obstacle->GetGeometryHandle(0);
			EXPECT(World->RemoveRigidBody(Obstacle)); Obstacle->ReleaseGeometries(); delete Obstacle;
			Obstacle = World->CreateRigidBody(GeometryFactory::CreateSphere(Vector3(4, 0.5f, 0), 0.3f), RigidBodyParam());
			const auto Current = Obstacle->GetGeometryHandle(0);
			EXPECT(Current.Index == Previous.Index && Current.Generation != Previous.Generation);
		};
		PhysicsWorld W(P); World = &W;
		W.CreateRigidBody(GeometryFactory::CreatePlane(Vector3::Zero(), Vector3::UnitY()), RigidBodyParam());
		Ball(W, Vector3(0, 0.49f, 0));
		Obstacle = W.CreateRigidBody(GeometryFactory::CreateSphere(Vector3(4, -0.5f, 0), 0.3f), RigidBodyParam());
		AerodynamicsSimulationSettings S(AerodynamicsAlgorithm::Incompressible);
		S.Backend = FluidBackend::CPU; S.Resolution = Vector3i(16); S.PressureIterations = 8;
		S.BoundsMin = Vector3(3, -1, -1); S.BoundsMax = Vector3(5, 1, 1); S.Emitters.clear();
		auto* Air = W.CreateAerodynamics(S); EXPECT(Air); if (!Air) return;
		StepFrame(W, 1.0f / 60); EXPECT(Replaced);
		const auto& Solid = Air->GetSolidCells();
		EXPECT(Solid[8 + 16 * (4 + 16 * 8)] == 0);
		EXPECT(Solid[8 + 16 * (12 + 16 * 8)] != 0);
	}
}

void TestClothRigidCoupling()
{
	ContactReaction(); AttachmentPullAndLifetime(); OffsetAttachmentTorque(); MixedClothColliders(); BroadPhaseMotion();
}

void TestUnifiedBroadPhase()
{
	CandidateViews();
	for (auto Broad : { BroadPhaseSolver::SAP, BroadPhaseSolver::ABP, BroadPhaseSolver::MBP,
		BroadPhaseSolver::AllPairs, BroadPhaseSolver::Bruteforce, BroadPhaseSolver::DynamicAABB })
	{
		for (auto Solver : { ResolutionPhaseSolver::PBD, ResolutionPhaseSolver::XPBD })
			for (bool Offsets : { false, true }) SharedBroadPhase(Broad, Solver, Offsets);
		SharedSurfaceFiltering(Broad); SharedFieldLifetime(Broad); BarotropicFieldFiltering(Broad);
		for (auto Solver : { ResolutionPhaseSolver::SequentialImpulse, ResolutionPhaseSolver::LCPGlobal, ResolutionPhaseSolver::PBD, ResolutionPhaseSolver::XPBD })
		{
			FrameMotion(Broad, Solver); IndependentMaterials(Broad, Solver); TransferredMotion(Broad, Solver);
		}
		MovingCoupledSurface(Broad); LateRegistration(Broad); ProjectedMotion(Broad); ReplacedFieldGeometry(Broad);
	}
}
