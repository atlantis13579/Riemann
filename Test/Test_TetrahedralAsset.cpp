#include "Test.h"
#include "../Src/Deformation/DeformableBody.h"
#include "../Src/Collision/GeometryObject.h"
#include "../Src/CollisionPrimitive/TriangleMesh.h"
#include "../Src/RigidBodyDynamics/PhysicsWorld.h"
#include "../Viewer/SceneWorld.h"
#include <algorithm>
#include <limits>
#include <stdexcept>

using namespace Riemann;
namespace
{
	template<class F> void Reject(F Function)
	{
		bool Rejected = false; try { Function(); } catch (const std::exception&) { Rejected = true; } EXPECT(Rejected);
	}
	TetrahedralAssetInput Unit(float Scale = 1)
	{
		TetrahedralAssetInput Input;
		Input.Positions = { Vector3(0), Vector3(Scale, 0, 0), Vector3(0, Scale, 0), Vector3(0, 0, Scale) };
		Input.Elements.push_back({ { 0, 1, 2, 3 }, 0 }); Input.Materials[0].Density = 6;
		return Input;
	}
	Vector3 Reconstruct(const TetrahedralAsset& Asset, const TetrahedralRenderBinding& B)
	{
		DeformationGradient::DVector P(0); const auto& E = Asset.GetElements()[B.Element];
		for (int I = 0; I < 4; ++I) { const auto& X = Asset.GetPositions()[E.Vertices[I]]; P += DeformationGradient::DVector(X.x, X.y, X.z) * B.Weights[I]; }
		return Vector3((float)P.x, (float)P.y, (float)P.z);
	}
	void PreparationAndTopology()
	{
		auto Input = Unit(); Input.Positions.push_back(-Vector3::UnitZ());
		Input.Materials.push_back(Input.Materials[0]); Input.Materials[1].Density = 12;
		Input.Elements.push_back({ { 0, 1, 2, 4 }, 1 }); Input.PinnedVertices = { 0 };
		const auto Asset = TetrahedralAsset::Prepare(Input);
		EXPECT(Asset.GetElements().size() == 2 && Asset.GetEdges().size() == 9 && Asset.GetBoundary().size() == 6);
		EXPECT(Asset.GetStats().ReorientedElements == 1 && Asset.GetStats().ConnectedComponents == 1);
		EXPECT_NEAR(Asset.GetStats().TotalVolume, 1.0 / 3, 1e-12); EXPECT_NEAR(Asset.GetStats().TotalMass, 3, 1e-12);
		for (int I = 0; I < 3; ++I) EXPECT_NEAR(1 / Asset.GetInverseMasses()[I], 0.75, 1e-7);
		EXPECT_NEAR(1 / Asset.GetInverseMasses()[3], 0.25, 1e-7); EXPECT_NEAR(1 / Asset.GetInverseMasses()[4], 0.5, 1e-7);
		EXPECT(Asset.GetPins()[0]); double BoundaryVolume = 0;
		for (const auto& F : Asset.GetBoundary()) BoundaryVolume += Asset.GetPositions()[F.a].Dot(Asset.GetPositions()[F.b].Cross(Asset.GetPositions()[F.c])) / 6.0;
		EXPECT_NEAR(BoundaryVolume, Asset.GetStats().TotalVolume, 1e-12);
		for (size_t I = 0; I < Asset.GetPositions().size(); ++I) EXPECT_NEAR((Reconstruct(Asset, Asset.GetRenderBindings()[I]) - Asset.GetPositions()[I]).Length(), 0, 1e-7);
		const auto Face = Asset.BindPoints({ Vector3(0.2f, 0.3f, 0) }); EXPECT(Face[0].Element == 0);
		// Six tetrahedra around a cube diagonal exercise interior face cancellation
		// and connected boundary vertex links beyond a two-element fixture.
		TetrahedralAssetInput Cube;
		Cube.Positions = { Vector3(0,0,0), Vector3(1,0,0), Vector3(1,1,0), Vector3(0,1,0), Vector3(0,0,1), Vector3(1,0,1), Vector3(1,1,1), Vector3(0,1,1) };
		for (const auto& T : { std::array<int,4>{0,1,2,6}, {0,2,3,6}, {0,3,7,6}, {0,7,4,6}, {0,4,5,6}, {0,5,1,6} }) Cube.Elements.push_back({ T, 0 });
		const auto Box = TetrahedralAsset::Prepare(Cube); EXPECT(Box.GetBoundary().size() == 12); EXPECT_NEAR(Box.GetStats().TotalVolume, 1, 1e-12);
		auto Separate = Unit(); for (const auto& P : Unit().Positions) Separate.Positions.push_back(P + Vector3(3,0,0));
		Separate.Elements.push_back({ {4,5,6,7}, 0 }); EXPECT(TetrahedralAsset::Prepare(Separate).GetStats().ConnectedComponents == 2);
	}
	void InvalidAssets()
	{
		auto Bad = Unit(); Bad.Elements[0].Vertices[3] = 8; Reject([&] { TetrahedralAsset::Prepare(Bad); });
		Bad = Unit(); Bad.Elements.push_back(Bad.Elements[0]); Reject([&] { TetrahedralAsset::Prepare(Bad); });
		Bad = Unit(); Bad.Elements[0].Material = 1; Reject([&] { TetrahedralAsset::Prepare(Bad); });
		Bad = Unit(); Bad.Positions.push_back(Vector3(2)); Reject([&] { TetrahedralAsset::Prepare(Bad); });
		Bad = Unit(); Bad.Positions[3].z = 1e-7f; Reject([&] { TetrahedralAsset::Prepare(Bad); });
		Bad = Unit(); Bad.Positions[0].x = std::numeric_limits<float>::infinity(); Reject([&] { TetrahedralAsset::Prepare(Bad); });
		Bad = Unit(); Bad.PinnedVertices = { -1 }; Reject([&] { TetrahedralAsset::Prepare(Bad); });
		// Geometric overlap without any shared connectivity must also fail.
		Bad = Unit(); for (const auto& P : Unit().Positions) Bad.Positions.push_back(P + Vector3(0.1f));
		Bad.Elements.push_back({ {4,5,6,7}, 0 }); Reject([&] { TetrahedralAsset::Prepare(Bad); });
		// Two otherwise closed volumes joined at one vertex form a pinched boundary.
		Bad = Unit(); Bad.Positions.insert(Bad.Positions.end(), { -Vector3::UnitX(), -Vector3::UnitY(), -Vector3::UnitZ() });
		Bad.Elements.push_back({ {0,4,5,6}, 0 }); Reject([&] { TetrahedralAsset::Prepare(Bad); });
		Bad = Unit(); Bad.Positions.push_back(Vector3(0,0,2)); Bad.Elements.push_back({ {0,1,2,4}, 0 }); Reject([&] { TetrahedralAsset::Prepare(Bad); });
		Bad = Unit(); Bad.RenderPositions = { Vector3(0) }; Reject([&] { TetrahedralAsset::Prepare(Bad); });
		Bad = Unit(); Bad.RenderPositions = Bad.Positions; Bad.RenderTriangles = { Index3(0,1,1) }; Reject([&] { TetrahedralAsset::Prepare(Bad); });
		TetrahedralAsset Empty; Reject([&] { Empty.GetPositions(); }); Reject([&] { Empty.BindPoints({ Vector3(0) }); });
		auto Settings = TetrahedralAssetSettings(); Settings.MinimumQuality = 2; EXPECT(!Settings.IsValid());
	}
	void BindingAndAffineRendering()
	{
		for (float Scale : { 0.001f, 1.0f, 1000.0f })
		{
			auto Input = Unit(Scale); Input.RenderPositions = { Vector3(0.2f,0.2f,0.2f)*Scale, Vector3(0.5f,0.1f,0.1f)*Scale, Vector3(0.1f,0.5f,0.1f)*Scale,
				Vector3(0.2f,0.2f,-0.05f)*Scale }; Input.RenderTriangles = { Index3(0,1,2), Index3(0,3,1) };
			Reject([&] { TetrahedralAsset::Prepare(Input); });
			TetrahedralAssetSettings Settings; Settings.AllowExtrapolation = true;
			const auto Asset = TetrahedralAsset::Prepare(Input, Settings); EXPECT(Asset.GetStats().ExtrapolatedVertices == 1);
			for (size_t I = 0; I < Input.RenderPositions.size(); ++I) EXPECT_NEAR((Reconstruct(Asset, Asset.GetRenderBindings()[I]) - Input.RenderPositions[I]).Length() / Scale, 0, 2e-7);
			EXPECT(*std::min_element(Asset.GetRenderBindings()[3].Weights.begin(), Asset.GetRenderBindings()[3].Weights.end()) < 0);
			Reject([&] { Asset.BindPoints({ Vector3(0,0,-2)*Scale }); });
			std::vector<Vector3> Samples; for (int I = 0; I <= 10; ++I) for (int J = 0; J + I <= 10; ++J)
				Samples.push_back(Vector3(I/10.0f, J/10.0f, 0)*Scale);
			const auto Bindings = Asset.BindPoints(Samples); for (size_t I = 0; I < Samples.size(); ++I) EXPECT_NEAR((Reconstruct(Asset, Bindings[I])-Samples[I]).Length()/Scale, 0, 2e-7);
			DeformableBody Body; Body.InitializeAsset(Asset); auto* Geometry = Body.GetRenderGeometry(); auto* Mesh = Geometry->GetShapeObj<TriangleMesh>();
			auto Affine = [&](const Vector3& P) { return Vector3(1.2f*P.x+0.3f*P.y, -0.2f*P.x+0.9f*P.y+0.1f*P.z, 1.1f*P.z) + Vector3(2,-1,3)*Scale; };
			for (int I = 0; I < Body.GetParticleCount(); ++I) EXPECT(Body.SetParticlePosition(I, Affine(Input.Positions[I])));
			for (size_t I = 0; I < Input.RenderPositions.size(); ++I) EXPECT_NEAR((Mesh->Vertices[I]-Affine(Input.RenderPositions[I])).Length()/Scale, 0, 1e-6);
			EXPECT(Body.GetSurfaceTriangles().size() == 4 && Mesh->NumTriangles == 2);
			for (const auto& N : Mesh->mNormals) EXPECT_NEAR(N.Length(), 1, 1e-6);
			EXPECT(!Geometry->IsQueryEnabled() && !Geometry->IsSimulationEnabled());
			Body.SetParticleVelocity(1, Vector3(0.1f)); const auto Velocity = Body.GetParticle(1).Velocity;
			const auto Point = BarycentricPoint::Tetrahedron({0,1,2,3}, {0.25,0.25,0.25,0.25}); Body.AttachToWorld(Point, Body.GetPointPosition(Point));
			Body.UseEmbeddedRenderMesh(false); EXPECT(Body.GetRenderGeometry() == Geometry && Mesh->NumTriangles == 4);
			EXPECT(Body.GetAttachmentCount() == 1 && Body.GetParticle(1).Velocity == Velocity);
			Body.UseEmbeddedRenderMesh(true); EXPECT(Mesh->NumTriangles == 2 && Body.GetRenderGeometry() == Geometry);
			Body.Reset(); for (size_t I = 0; I < Input.RenderPositions.size(); ++I) EXPECT_NEAR((Mesh->Vertices[I]-Input.RenderPositions[I]).Length()/Scale, 0, 2e-7);
		}
		// Distance alone cannot admit large barycentric weights in thin elements.
		auto Input = Unit(); Input.Positions[3].z = 0.01f; auto Settings = TetrahedralAssetSettings(); Settings.AllowExtrapolation = true; Settings.MaximumExtrapolationWeight = 1;
		const auto Thin = TetrahedralAsset::Prepare(Input, Settings); Reject([&] { Thin.BindPoints({ Vector3(0.2f,0.2f,-0.02f) }); });
	}
	void ExistingDemo()
	{
		SceneWorld Scene; Scene.SetClothModelDemo(true); std::string Error; const auto Path = TestContentsPath() + "Scenes/08_cloth_sphere.json";
		EXPECT(Scene.LoadFromFile(Path, &Error));
		for (bool Rigid : {false,true}) for (bool Material : {false,true})
		{
			EXPECT(Scene.ApplyDeformableContactDemo(false)); auto* World = Scene.GetSimulation();
			EXPECT(World->SetSolverConfiguration(Rigid ? ResolutionPhaseSolver::XPBD : ResolutionPhaseSolver::SequentialImpulse, SolverBackend::CPU, false));
			EXPECT(Scene.ApplyDeformableContactDemo(true, Rigid, true)); EXPECT(Scene.GetPreparedSoftAssetDemo() && Scene.GetEmbeddedSoftRenderDemo());
			EXPECT(Scene.ApplyDeformableMaterialDemo(Material));
			auto* Body = Scene.GetDemoContactBody(0); auto* Geometry = Body->GetRenderGeometry(); const auto Revision = Scene.GetRenderRevision();
			auto* Mesh = Geometry->GetShapeObj<TriangleMesh>(); EXPECT(Body->GetParticleCount() == 4 && Body->GetSurfaceTriangles().size() == 4);
			EXPECT(Mesh->NumVertices == 34 && Mesh->NumTriangles == 64);
			const auto Before = Body->GetParticle(1).Position; const size_t Attachments = Body->GetAttachmentCount();
			EXPECT(Scene.ApplyDeformableContactDemo(true, Rigid, true, false)); EXPECT(Scene.GetDemoContactBody(0) == Body && Body->GetRenderGeometry() == Geometry);
			EXPECT(Mesh->NumTriangles == 4 && Body->GetParticle(1).Position == Before && Body->GetAttachmentCount() == Attachments);
			EXPECT(Scene.GetRenderRevision() > Revision); EXPECT(Scene.ApplyDeformableContactDemo(true, Rigid, true, true));
			size_t Contacts = 0;
			for (int Frame = 0; Frame < 120; ++Frame)
			{
				Scene.Step(1.0f/60);
				for (size_t I = 0; I < 2; ++I)
				{
					auto* B = Scene.GetDemoContactBody(I); EXPECT(B && B->GetPreparedAsset()); if (!B) continue;
					for (const auto& P : B->GetParticles()) EXPECT(std::isfinite(P.Position.SquareLength()) && P.Position.Length() < 100);
					Contacts += B->GetCollisionStats().InterContacts + B->GetCollisionStats().RigidContacts;
					if (Material) EXPECT(B->GetTetrahedronMaterialResponse(0).Jacobian > 0);
				}
			}
			EXPECT(Contacts > 0); printf("Prepared asset demo material=%d rigid=%d contacts=%zu render=%u/%u\n", Material, Rigid, Contacts, Mesh->NumVertices, Mesh->NumTriangles);
			World->RemoveDeformableBody(Scene.GetDemoContactBody(1)); Scene.Step(1.0f/60); EXPECT(!Scene.GetDemoContactBody(0));
			EXPECT(Scene.ApplyDeformableContactDemo(true, Rigid, true));
			EXPECT(Scene.ApplyDeformableContactDemo(true, Rigid, false)); EXPECT(!Scene.GetPreparedSoftAssetDemo());
		}
		EXPECT(Scene.LoadFromFile(Path, &Error)); EXPECT(!Scene.GetPreparedSoftAssetDemo());
	}
	void InstancesAndWorld()
	{
		auto Input = Unit(); Input.PinnedVertices = {0}; const auto Asset = TetrahedralAsset::Prepare(Input); const auto Copy = Asset;
		EXPECT(&Copy.GetPositions() == &Asset.GetPositions());
		DeformableBodySettings Settings; Settings.GroundEnabled = false;
		PhysicsWorldParam Param; PhysicsWorld World(Param); auto* A = World.CreateDeformableBody(Asset, Settings);
		Quaternion Rotation; Rotation.FromRotationAxis(Vector3::UnitY(), 0.7f); const Transform Pose(Vector3(3,2,1), Rotation);
		auto* B = World.CreateDeformableBody(Copy, Settings, Pose);
		EXPECT(&A->GetPreparedAsset()->GetPositions() == &B->GetPreparedAsset()->GetPositions());
		for (int I = 0; I < B->GetParticleCount(); ++I) EXPECT_NEAR((B->GetParticle(I).Position-Pose.LocalToWorld(Input.Positions[I])).Length(), 0, 1e-6);
		const auto& Rest = B->GetTetrahedronRestShape(0); const auto& Tet = B->GetTetrahedra()[0]; Vector3 Positions[4];
		for (int I = 0; I < 4; ++I) Positions[I] = B->GetParticle(Tet.Particles[I]).Position;
		const auto F = Rest.Evaluate(Positions); for (int I = 0; I < 3; ++I) for (int J = 0; J < 3; ++J) EXPECT_NEAR(F[I][J], I == J ? 1 : 0, 1e-12);
		EXPECT(std::abs(B->GetMaterialEnergy()) / (Rest.Volume * B->GetMaterials()[0].YoungModulus) < 1e-12);
		A->UnpinParticle(0); auto Material = A->GetMaterials()[0]; Material.YoungModulus *= 2; A->SetMaterial(0, Material);
		EXPECT(B->GetParticle(0).Pinned && Asset.GetPins()[0]); EXPECT_NEAR(B->GetMaterials()[0].YoungModulus, Input.Materials[0].YoungModulus, 0);
		Reject([&] { A->AddParticle(Vector3(2)); }); Reject([&] { A->AddTetrahedron(0,1,2,3); }); Reject([&] { A->SetSurfaceTriangles({}); });
		Reject([&] { A->InitializeAsset(Asset); }); EXPECT(A->AddEdge(0,1) >= 0);
		Transform Bad(Vector3(1e20f)); const size_t Count = World.GetDeformableBodies().size();
		Reject([&] { World.CreateDeformableBody(Asset, Settings, Bad); }); EXPECT(World.GetDeformableBodies().size() == Count);
		Bad = Transform::Identity(); Bad.quat.x = 2; Reject([&] { World.CreateDeformableBody(Asset, Settings, Bad); });
		A->SetTetrahedronMaterial(0, -1); for (const auto& E : A->GetEdges()) EXPECT(E.Enabled);
		A->SetTetrahedronMaterial(0, 0); EXPECT(A->GetEdges()[0].Enabled);
		for (int Frame = 0; Frame < 30; ++Frame) World.Simulate(1.0f/60);
		for (const auto* Body : {A,B}) for (const auto& P : Body->GetParticles()) EXPECT(std::isfinite(P.Position.SquareLength()));
		EXPECT(World.RemoveDeformableBody(A)); EXPECT(World.RemoveDeformableBody(B));
	}
}
void TestTetrahedralAsset()
{
	PreparationAndTopology(); InvalidAssets(); BindingAndAffineRendering(); InstancesAndWorld(); ExistingDemo();
}
