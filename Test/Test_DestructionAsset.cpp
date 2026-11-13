#include "Test.h"
#include "../Src/Destruction/DestructionSet.h"
#include "../Src/Destruction/StrainSolver.h"
#include "../Src/CollisionPrimitive/ConvexMesh.h"
#include "../Src/Geometry/DynamicMesh.h"
#include "../Src/Geometry/ConvexHull3.h"
#include "../Src/RigidBodyDynamics/PhysicsWorld.h"
#include "../Viewer/SceneWorld.h"
#include <limits>
#include <map>
#include <stdexcept>

using namespace Riemann;
namespace
{
	template<class F> void Reject(F Function)
	{
		bool Rejected = false; try { Function(); } catch (const std::exception&) { Rejected = true; } EXPECT(Rejected);
	}
	void Author(DestructionSet& Set, bool Reverse = false)
	{
		Set.GetDestructionConstants().MinClusterVolume = 0;
		Set.GetDestructionConstants().FilterData = CollisionData::ForLayer(3);
		Set.GetDestructionConstants().ContactOffset = 0.02f; Set.GetDestructionConstants().RestOffset = -0.005f;
		for (int I = 0; I < 3; ++I)
		{
			const Vector3 Center(I * 1.1f,0,0); Set.AddChunk(Box3(Center-Vector3(0.5f),Center+Vector3(0.5f)),(float)(I+2),false,"chunk_"+std::to_string(I));
		}
		if (Reverse) { Set.AddBond(1,2,DestructionBondType::Connect,3,12); Set.AddBond(0,1,DestructionBondType::Connect,2,11); }
		else { Set.AddBond(0,1,DestructionBondType::Connect,2,11); Set.AddBond(1,2,DestructionBondType::Connect,3,12); }
		Set.GetGraph()[0].IsSupport = true; Set.GetGraph()[0].SupportGroupID = 7; Set.RebuildClustersFromGraph();
	}
	void BoxMesh(DynamicMesh& Mesh)
	{
		for (const auto& P : {Vector3(-1,-1,-1),Vector3(1,-1,-1),Vector3(1,1,-1),Vector3(-1,1,-1),
			Vector3(-1,-1,1),Vector3(1,-1,1),Vector3(1,1,1),Vector3(-1,1,1)}) Mesh.AppendVertex(P);
		for (const auto& T : {Index3(0,2,1),Index3(0,3,2),Index3(4,5,6),Index3(4,6,7),Index3(0,4,7),Index3(0,7,3),
			Index3(1,2,6),Index3(1,6,5),Index3(0,1,5),Index3(0,5,4),Index3(3,7,6),Index3(3,6,2)}) EXPECT(Mesh.AppendTriangle(T.a,T.b,T.c) >= 0);
		Mesh.BuildBounds();
	}
	void HullVisibility()
	{
		for (float Scale : {0.001f,1.0f,1000.0f})
		{
			const Vector3 Offset = Vector3(17,-9,4) * Scale; Vector3 Corners[8]; Box3::GetVertices(Vector3(-Scale),Vector3(Scale),Corners);
			std::vector<Vector3> Points; for (const auto& P : Corners) Points.push_back(P+Offset);
			ConvexHull3Options Options; Options.DistanceTolerance = Scale * 1e-4f;
			// The point is strongly outside Y but only shallowly outside X. Both
			// visible patches must leave the hull, even below the acceptance band.
			Points.push_back(Offset+Vector3(Scale+Options.DistanceTolerance/2,Scale+Options.DistanceTolerance*2,Scale/2));
			ConvexHull3d Hull; EXPECT(BuildConvexHull3(Points,Hull,Options));
			std::map<std::pair<int,int>,std::pair<int,int>> Edges;
			for (const auto& F : Hull.faces)
			{
				for (const auto& P : Points) EXPECT(F.norm.Dot(P)+F.w <= Options.DistanceTolerance*5);
				for (size_t I = 0; I < F.verts.size(); ++I)
				{
					const int A = F.verts[I], B = F.verts[(I+1)%F.verts.size()]; auto& E = Edges[{std::min(A,B),std::max(A,B)}];
					++E.first; E.second += A < B ? 1 : -1;
				}
			}
			for (const auto& E : Edges) EXPECT(E.second.first == 2 && E.second.second == 0);
			EXPECT((int)Hull.verts.size()-(int)Edges.size()+(int)Hull.faces.size() == 2);
		}
	}
	void SnapshotsAndInstances()
	{
		DestructionSet Source, Reordered; Author(Source); Author(Reordered,true);
		const auto Asset = DestructionAsset::Capture(Source), Copy = Asset;
		EXPECT(&Copy.GetChunks() == &Asset.GetChunks()); EXPECT(Asset.GetChunks().size() == 3 && Asset.GetStats().BondCount == 2);
		EXPECT_NEAR(Asset.GetStats().TotalMass,9,1e-12); EXPECT_NEAR(Asset.GetStats().TotalVolume,3,1e-12);
		EXPECT(Asset.GetStats().Fingerprint == DestructionAsset::Capture(Reordered).GetStats().Fingerprint);
		Source.GetGraph()[0].Bonds[0].frame.Origin.x = 0.2f;
		EXPECT(Asset.GetStats().Fingerprint != DestructionAsset::Capture(Source).GetStats().Fingerprint); Source.Clear();
		PhysicsWorldParam Param; PhysicsWorld World(Param); DestructionSet A(World), B(World);
		Quaternion Rotation; Rotation.FromRotationAxis(Vector3(0.2f,0.9f,0.3f).Unit(),0.7f); const Transform Pose(Vector3(5,2,-3),Rotation);
		A.InitializeAsset(Asset); B.InitializeAsset(Copy,Pose);
		EXPECT(&A.GetAuthoredAsset()->GetStats() == &B.GetAuthoredAsset()->GetStats());
		EXPECT(A.GetGraph()[0].IsSupport && B.GetGraph()[0].SupportGroupID == 7);
		for (int I = 0; I < 3; ++I)
		{
			const auto* CA = A.FindClusterContaining(I); const auto* CB = B.FindClusterContaining(I); EXPECT(CA && CB);
			Transform Actual; EXPECT(CB->GetWorldTransform(I,Actual));
			EXPECT_NEAR((Actual.pos-Pose.LocalToWorld(Asset.GetChunks()[I].Centroid)).Length(),0,2e-5);
			const auto& HA = *A.GetChunks()[I].CollisionConvex; const auto& HB = *B.GetChunks()[I].CollisionConvex;
			EXPECT(&HA != &HB && HA.Vertices != HB.Vertices);
			for (uint16_t J = 0; J < HA.NumVertices; ++J)
				EXPECT_NEAR((Actual.LocalToWorld(HB.Vertices[J])-Pose.LocalToWorld(HA.Vertices[J]+Asset.GetChunks()[I].Centroid)).Length(),0,2e-5);
			auto* Geometry = CB->GetGeometryByChunkIndex(I); EXPECT(Geometry && Geometry->GetFilterData().v0 == 3);
			EXPECT_NEAR(Geometry->GetContactOffset(),0.02f,0); EXPECT_NEAR(Geometry->GetRestOffset(),-0.005f,0);
		}
		const auto Original = A.GetChunks()[0].CollisionConvex->Vertices[0]; A.GetChunks()[0].CollisionConvex->Vertices[0].x += 0.1f;
		DestructionSet C; C.InitializeAsset(Asset); EXPECT(C.GetChunks()[0].CollisionConvex->Vertices[0] == Original);
		Reject([&] { A.AddChunk(Box3::Unit()); }); Reject([&] { A.InitializeAsset(Asset); });
		A.DoFractures({{11}}); EXPECT(!A.GetGraph().HasBond(0,1) && B.GetGraph().HasBond(0,1)); EXPECT(A.GetClusters().size() == 2 && B.GetClusters().size() == 1);
		for (int Frame = 0; Frame < 10; ++Frame) World.Simulate(1.0f/60);
		World.Reset(); EXPECT(!A.GetPhysicsWorld() && !B.GetPhysicsWorld()); EXPECT(A.GetAuthoredAsset() && B.GetAuthoredAsset());
		World.AddDestructionSet(&B); EXPECT(B.GetClusters().size() == 1); EXPECT(B.GetClusters()[0]->GetRigidBody());
	}
	void StressAuthoring()
	{
		DestructionSet Source; Author(Source); StrainSolverSettings Settings; Settings.Hardness = 1e8f; Source.EnableStressSolver(Settings);
		auto* Original = Source.GetStressSolver(); EXPECT(Original->SetNodeInertia(1,Matrix3(1,2,3)));
		EXPECT(Original->SetBondGeometry(11,Vector3(0.6f,0.1f,0),Vector3::UnitX(),0.25f)); EXPECT(Original->SetBondHealth(11,0.7f)); EXPECT(Original->RemoveBond(12));
		const auto Asset = DestructionAsset::Capture(Source); Quaternion Q; Q.FromRotationAxis(Vector3::UnitY(),0.6f); const Transform Pose(Vector3(3,2,1),Q);
		DestructionSet Placed; Placed.InitializeAsset(Asset,Pose); auto* Solver = Placed.GetStressSolver(); EXPECT(Solver && Solver->GetSettings().Hardness == Settings.Hardness);
		EXPECT(Solver->GetBondInfo(12) == nullptr && Placed.GetGraph().HasBond(1,2)); const auto* Bond = Solver->GetBondInfo(11); EXPECT(Bond);
		EXPECT_NEAR(Bond->Health,0.7f,0); EXPECT_NEAR(Bond->Area,0.25f,0);
		EXPECT_NEAR((Bond->Centroid-Pose.LocalToWorld(Vector3(0.6f,0.1f,0))).Length(),0,1e-6);
		EXPECT_NEAR((Bond->Normal-Q*Vector3::UnitX()).Length(),0,1e-6);
		const Vector3 Impulse(0.2f,0.4f,0.7f); Original->AddTorque(1,Impulse); Solver->AddTorque(1,Q*Impulse); Original->Update(0.01f); Solver->Update(0.01f);
		EXPECT_NEAR((Solver->GetNodeAngularVelocity(1)-Q*Original->GetNodeAngularVelocity(1)).Length(),0,1e-4);
		Reject([&] { DestructionAsset::Capture(Source); });
	}
	void HierarchicalAuthoring()
	{
		DynamicMesh Mesh; BoxMesh(Mesh); DestructionSet Source; DestructionFractureSettings Settings;
		Settings.MaxChunkCount = 8; Settings.CutRetryCount = 8; Settings.MinChunkVolume = 0.001f;
		DestructionFractureLevel Level; Level.MinPieceCount = Level.MaxPieceCount = 2; Settings.Levels = {Level,Level};
		EXPECT(Source.BuildHierarchicalFracture(Mesh,Settings,12));
		const auto Asset = DestructionAsset::Capture(Source); EXPECT(Asset.GetStats().HierarchyNodes == Source.GetHierarchy().size());
		DestructionSet Instance; Instance.InitializeAsset(Asset); EXPECT(Instance.GetHierarchyRoot() == Source.GetHierarchyRoot());
		EXPECT(Instance.GetHierarchy().size() == Source.GetHierarchy().size());
		for (const auto& Node : Instance.GetGraph().GetBondLists()) if (!Node.Bonds.empty()) { Instance.DoFractures({{Node.Bonds[0].bond_index}}); break; }
		EXPECT(Source.GetGraph().GetBrokenBonds().empty());
		EXPECT(Asset.GetStats().Fingerprint == DestructionAsset::Capture(Source).GetStats().Fingerprint);
	}
	void Validation()
	{
		DestructionAsset Empty; Reject([&] { Empty.GetChunks(); }); DestructionSet Source; Reject([&] { DestructionAsset::Capture(Source); }); Author(Source);
		const auto Asset = DestructionAsset::Capture(Source); DestructionSet Target;
		Transform Bad(Vector3(1e20f)); Reject([&] { Target.InitializeAsset(Asset,Bad); }); EXPECT(Target.GetChunks().empty() && Target.GetGraph().Empty());
		Bad = Transform::Identity(); Bad.quat.x = 2; Reject([&] { Target.InitializeAsset(Asset,Bad); });
		Source.GetChunks()[0].Mass = std::numeric_limits<float>::quiet_NaN(); Reject([&] { DestructionAsset::Capture(Source); }); Source.GetChunks()[0].Mass = 2;
		Source.GetGraph().AddBond(0,8); Reject([&] { DestructionAsset::Capture(Source); }); Source.GetGraph().Resize(3); Source.GetGraph()[0].Bonds.pop_back(); Source.GetGraph().RefreshBondCache();
		Source.AddMomentumQueue(1,Vector3::UnitX(),Vector3(1.1f,0,0),Vector3::UnitX()); Reject([&] { DestructionAsset::Capture(Source); });
		Source.Clear(); Author(Source); Source.BreakBondEx(1); Reject([&] { DestructionAsset::Capture(Source); });
		Source.Clear(); Author(Source); Source.EnableStressSolver(StrainSolverSettings()); Source.GetStressSolver()->AddForce(1,Vector3::UnitX()); Reject([&] { DestructionAsset::Capture(Source); });
	}
	void ExistingScenes()
	{
		for (const char* Name : {"10_destruction.json","11_glass_fracture.json"})
		{
			SceneWorld Scene; Scene.SetAuthoredDestructionDemo(true); std::string Error;
			const bool Loaded = Scene.LoadFromFile(TestContentsPath()+"Scenes/"+Name,&Error); EXPECT(Loaded); if (!Loaded) { printf("Authored scene error: %s\n",Error.c_str()); continue; }
			EXPECT(Scene.GetDestructionSetCount() > 0); size_t Chunks = 0;
			for (size_t I = 0; I < Scene.GetDestructionSetCount(); ++I)
			{
				auto* Set = Scene.GetDestructionSet(I); EXPECT(Set->GetAuthoredAsset()); Chunks += Set->GetChunks().size();
				int Bond = -1; for (const auto& Node : Set->GetGraph().GetBondLists()) if (!Node.Bonds.empty()) { Bond = Node.Bonds[0].bond_index; break; }
				if (Bond >= 0) Set->DoFractures({{Bond}});
			}
			for (int Frame = 0; Frame < 20; ++Frame) Scene.Step(1.0f/60);
			for (size_t I = 0; I < Scene.GetDestructionSetCount(); ++I) for (auto* Cluster : Scene.GetDestructionSet(I)->GetClusters())
				if (auto* Body = Cluster->GetRigidBody()) EXPECT(std::isfinite(Body->X.SquareLength()) && std::isfinite(Body->V.SquareLength()));
			printf("Authored destruction scene %s sets=%zu chunks=%zu\n",Name,Scene.GetDestructionSetCount(),Chunks);
			Scene.SetAuthoredDestructionDemo(false); EXPECT(Scene.LoadFromFile(TestContentsPath()+"Scenes/"+Name,&Error));
			for (size_t I = 0; I < Scene.GetDestructionSetCount(); ++I) EXPECT(!Scene.GetDestructionSet(I)->GetAuthoredAsset());
		}
	}
}
void TestDestructionAsset()
{
	try { HullVisibility(); SnapshotsAndInstances(); StressAuthoring(); HierarchicalAuthoring(); Validation(); ExistingScenes(); }
	catch (const std::exception& Error) { printf("Destruction asset test exception: %s\n",Error.what()); EXPECT(false); }
}
