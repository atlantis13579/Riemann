
#include "Test.h"
#include <algorithm>
#include <vector>
#include "../Src/Destruction/Fracture.h"
#include "../Src/Destruction/GraphPartition.h"
#include "../Src/Destruction/DestructionSet.h"
#include "../Src/Geometry/MeshCut.h"
#include "../Src/Destruction/StrainSolver.h"
#include "../Src/Destruction/VoronoiTessellation.h"
#include "../Src/Collision/GeometryObject.h"
#include "../Src/Geometry/DynamicMesh.h"
#include "../Src/RigidBodyDynamics/PhysicsWorld.h"

using namespace Riemann;

static int CountValidTriangles(const DynamicMesh& mesh)
{
	int count = 0;
	for (int tid = 0; tid < mesh.GetTriangleCount(); ++tid)
	{
		if (mesh.IsTriangleFast(tid))
		{
			++count;
		}
	}
	return count;
}

static float GetBoundsMaxSize(const Box3& bounds)
{
	Vector3 size = bounds.Max - bounds.Min;
	return std::max(size.x, std::max(size.y, size.z));
}

static void AppendMeshWithOffset(DynamicMesh& merged, const DynamicMesh& source, const Vector3& offset)
{
	FDynamicMeshEditor editor(&merged);
	FMeshIndexMappings mappings;
	editor.AppendMesh(&source, mappings,
		[offset](int, const Vector3& position)
		{
			return position + offset;
		});
}

static Box3 MakeChunkBounds(float x0, float x1)
{
	return Box3(Vector3(x0, 0.0f, 0.0f), Vector3(x1, 1.0f, 1.0f));
}

static Geometry* MakeChunkGeometry(const Box3& bounds)
{
	return GeometryFactory::CreateOBB(bounds.GetCenter(), bounds.GetExtent());
}

static DynamicMesh MakeChunkMesh(const Box3& bounds)
{
	Vector3 vertices[8];
	Box3::GetVertices(bounds.Min, bounds.Max, vertices);

	DynamicMesh mesh;
	for (int vertexIndex = 0; vertexIndex < 8; ++vertexIndex)
	{
		mesh.AppendVertex(vertices[vertexIndex]);
	}

	mesh.AppendTriangle(0, 2, 1);
	mesh.AppendTriangle(1, 2, 3);
	mesh.AppendTriangle(4, 5, 6);
	mesh.AppendTriangle(5, 7, 6);
	mesh.AppendTriangle(0, 1, 4);
	mesh.AppendTriangle(1, 5, 4);
	mesh.AppendTriangle(2, 6, 3);
	mesh.AppendTriangle(3, 6, 7);
	mesh.AppendTriangle(0, 4, 2);
	mesh.AppendTriangle(2, 4, 6);
	mesh.AppendTriangle(1, 3, 5);
	mesh.AppendTriangle(3, 7, 5);
	mesh.BuildBounds();
	return mesh;
}

void TestMetis()
{
	int nVertices = 8;
	std::vector<int> nodes;
	for (int i = 0; i < nVertices; ++i)
	{
		nodes.push_back(i);
	}
	std::vector<std::vector<int>> part;
	std::vector<int> xadj = { 0,2,4,7,10,13,16,18,20 };
	std::vector<int> adjncy = { 1,2,0,3,0,3,4,1,2,5,2,5,6,3,4,7,4,7,5,6 };
	PartitionCsrGraph(nodes, xadj, adjncy, nullptr, 2, part);

	EXPECT(part.size() == 2);
	EXPECT(part[0].size() == 4);
	EXPECT(part[1].size() == 4);
	return ;
}

void TestVoronoiMesh()
{
	printf("Running TestVoronoiMesh\n");

	std::vector<Vector3> points;
	Voronoi3::GenerateRandomPoints(Box3::Unit(), 100, points);

	VoronoiMesh mesh(points, Box3::Unit(), 1e-3f);
	EXPECT(mesh.GetMeshCount() == 100);
	EXPECT(mesh.GetTotalTriangleCount() > 0);
	for (int index = 0; index < mesh.GetMeshCount(); ++index)
	{
		const DynamicMesh* cellMesh = mesh.GetMesh(index);
		EXPECT(cellMesh != nullptr);
		EXPECT(cellMesh == nullptr || CountValidTriangles(*cellMesh) > 0);
	}

	std::vector<Vector3> twoPoints;
	twoPoints.push_back(Vector3(-0.25f, 0.0f, 0.0f));
	twoPoints.push_back(Vector3( 0.25f, 0.0f, 0.0f));

	VoronoiMesh singlePlane(twoPoints, Box3::Unit(), 1e-3f);
	EXPECT(singlePlane.GetMeshCount() == 2);
	EXPECT(singlePlane.GetTotalTriangleCount() > 0);

	return;
}

void TestConnectionGraphDestruction()
{
	printf("Running TestConnectionGraphDestruction\n");

	ConnectionGraph graph;
	graph.AddBond(0, 1, DestructionBondType::Connect, 0.5f, 10);
	graph.AddBond(1, 2, DestructionBondType::Support, 2.0f, 11);
	graph.AddBond(2, 3, DestructionBondType::Connect, 0.25f, 12);

	EXPECT(graph.Size() == 4);
	EXPECT(graph.HasBond(0, 1));
	EXPECT(graph.HasBond(1, 0));
	EXPECT(graph.GetBondType(1, 2) == DestructionBondType::Support);
	EXPECT(graph.GetBondType(2, 1) == DestructionBondType::BeSupported);

	BitSet connectedOnly = graph.FindSupported(0, DestructionBondMaskConnect);
	EXPECT(connectedOnly.get(0));
	EXPECT(connectedOnly.get(1));
	EXPECT(!connectedOnly.get(2));

	std::vector<std::pair<int, int>> shockBreaks;
	graph.ShockPropagation(0, 10.0f, 0.5f, shockBreaks);
	EXPECT(!shockBreaks.empty());

	graph.BreakBond(1, 2);
	EXPECT(!graph.HasBond(1, 2));
	EXPECT(!graph.HasBond(2, 1));
	EXPECT(!graph.GetBrokenBonds().empty());

	graph.BreakBondsByType(0, DestructionBondMaskConnect);
	EXPECT(!graph.HasBond(0, 1));
}

void TestConnectionGraphPartition()
{
	printf("Running TestConnectionGraphPartition\n");

	ConnectionGraph graph;
	graph.AddBond(0, 1, DestructionBondType::Connect, 1.0f, 0);
	graph.AddBond(1, 2, DestructionBondType::Connect, 1.0f, 1);
	graph.AddBond(2, 3, DestructionBondType::Connect, 1.0f, 2);
	graph.AddBond(3, 4, DestructionBondType::Connect, 1.0f, 3);
	graph.AddBond(4, 5, DestructionBondType::Connect, 1.0f, 4);

	std::vector<int> nodes;
	for (int i = 0; i < 6; ++i)
	{
		nodes.push_back(i);
	}

	DestructionBreakStatus status = graph.BreakGraph(nodes, 2);
	EXPECT(status == DestructionBreakStatus::Success);
	EXPECT(!graph.GetBrokenBonds().empty());
}

void TestDestructionSetAndCluster()
{
	printf("Running TestDestructionSetAndCluster\n");

	DestructionSet destructSet;
	destructSet.AddChunk(MakeChunkBounds(0.0f, 1.0f), 1.0f, true, "ground");
	destructSet.AddChunk(MakeChunkBounds(1.0f, 2.0f), 1.0f, false, "a");
	destructSet.AddChunk(MakeChunkBounds(2.0f, 3.0f), 1.0f, false, "b");
	destructSet.AddChunk(MakeChunkBounds(3.0f, 4.0f), 1.0f, false, "c");
	destructSet.AddBond(0, 1, DestructionBondType::Support, 1.0f, 0);
	destructSet.AddBond(1, 2, DestructionBondType::Connect, 1.0f, 1);
	destructSet.AddBond(2, 3, DestructionBondType::Connect, 1.0f, 2);
	destructSet.RebuildClustersFromGraph();

	std::vector<DestructionCluster*> clusters = destructSet.GetClusters();
	EXPECT(clusters.size() == 1);
	EXPECT(clusters[0]->Size() == 4);
	EXPECT(clusters[0]->IsStatic());
	Maths::Transform localTransform;
	Maths::Transform worldTransform;
	Vector3 worldPosition;
	EXPECT(clusters[0]->GetLocalTransform(2, localTransform));
	EXPECT(clusters[0]->GetWorldTransform(2, worldTransform));
	EXPECT(clusters[0]->GetWorldPosition(2, worldPosition));
	EXPECT_SAME(worldTransform.pos, destructSet.GetChunks()[2].Centroid);
	EXPECT_SAME(worldPosition, destructSet.GetChunks()[2].Centroid);

	PhysicsWorldParam simParam;
	PhysicsWorld simulation(simParam);
	RigidBodyParam rigidParam;
	rigidParam.rigidType = RigidType::Dynamic;
	std::vector<Geometry*> geometries;
	for (const DestructionChunk& chunk : destructSet.GetChunks())
	{
		geometries.push_back(MakeChunkGeometry(chunk.Bounds));
	}

	RigidBodyDynamic* clusterBody = clusters[0]->BuildRigidBody(simulation, rigidParam, geometries);
	EXPECT(clusterBody != nullptr);
	EXPECT(clusters[0]->GetRigidBody() == clusterBody);
	EXPECT(clusterBody->GetNumGeometries() == clusters[0]->Size());
	EXPECT(clusterBody->CastDynamic() == clusterBody);
	EXPECT(clusters[0]->GetGeometryByChunkIndex(2) == geometries[2]);
	EXPECT(clusters[0]->GetChunkIndexByGeometry(geometries[2]) == 2);
	EXPECT(geometries[2]->GetParent<RigidBody>() == clusterBody);
	EXPECT_SAME(geometries[2]->GetWorldPosition(), destructSet.GetChunks()[2].Centroid);
	EXPECT(clusterBody->Freezing);
	clusterBody->ReleaseGeometries();

	BitSet unsupported = destructSet.FindUnsupported();
	EXPECT(unsupported.to_vector().empty());

	destructSet.BreakBondEx(1);
	std::vector<int> pruned = destructSet.PruneUnsupported();
	EXPECT(pruned.size() == 2);
	DestructionSet::Stats stats = destructSet.GetStats();
	EXPECT(stats.StaticCount == 1);
	EXPECT(stats.FreeCount == 3);
}

void TestDestructionChunkConvexCollision()
{
	printf("Running TestDestructionChunkConvexCollision\n");

	PhysicsWorldParam simParam;
	PhysicsWorld simulation(simParam);
	DestructionSet destructSet(simulation);

	const Box3 bounds = MakeChunkBounds(0.0f, 1.0f);
	DynamicMesh mesh = MakeChunkMesh(bounds);
	const int chunkIndex = destructSet.AddChunk(mesh, 1.0f, false, "convex");
	EXPECT(chunkIndex == 0);
	EXPECT(destructSet.GetChunks()[0].CollisionConvex != nullptr);

	destructSet.RebuildClustersFromGraph();
	std::vector<DestructionCluster*> clusters = destructSet.GetClusters();
	EXPECT(clusters.size() == 1);
	if (!clusters.empty())
	{
		Geometry* geom = clusters[0]->GetGeometryByChunkIndex(0);
		EXPECT(geom != nullptr);
		EXPECT(geom == nullptr || geom->GetShapeType() == PrimitiveType::CONVEX_MESH);
		if (geom)
		{
			EXPECT_SAME(geom->GetWorldPosition(), bounds.GetCenter());
		}
	}
}

void TestDestructionEvents()
{
	printf("Running TestDestructionEvents\n");

	DestructionSet destructSet;
	destructSet.AddChunk(MakeChunkBounds(0.0f, 1.0f), 1.0f, false);
	destructSet.AddChunk(MakeChunkBounds(1.0f, 2.0f), 1.0f, false);
	destructSet.AddBond(0, 1, DestructionBondType::Connect, 1.0f, 0);
	destructSet.RebuildClustersFromGraph();

	destructSet.AddDestructionInfo(DestructionEventType::SelfCollapse, { 1 });
	uint32_t flags = destructSet.ProcessDestructionInfo();
	EXPECT((flags & (1u << (int)DestructionEventType::SelfCollapse)) != 0);
	EXPECT(destructSet.GetChunks()[1].IsSmashed);
	EXPECT(destructSet.GetChunks()[1].IsFree);

	DestructionSet::Stats stats = destructSet.GetStats();
	EXPECT(stats.SmashedCount == 1);
}

void TestDestructionMomentum()
{
	printf("Running TestDestructionMomentum\n");

	DestructionSet destructSet;
	for (int i = 0; i < 6; ++i)
	{
		destructSet.AddChunk(MakeChunkBounds((float)i, (float)i + 1.0f), 1.0f, false);
		if (i > 0)
		{
			destructSet.AddBond(i - 1, i, DestructionBondType::Connect, 1.0f, i - 1);
		}
	}
	destructSet.GetDestructionConstants().MinClusterVolume = 0.0f;
	destructSet.GetDestructionConstants().Levels[0].BreakThreshold = 1.0f;
	destructSet.GetDestructionConstants().Levels[0].PartitionSize = 2;
	destructSet.RebuildClustersFromGraph();

	bool changed = destructSet.ApplyMomentum(2, Vector3(5.0f, 0.0f, 0.0f), Vector3(2.5f, 0.5f, 0.5f), Vector3::UnitX());
	EXPECT(changed);
	EXPECT(!destructSet.GetGraph().GetBrokenBonds().empty());
}

void TestDestructionClusterSplitKeepsWorldTransform()
{
	printf("Running TestDestructionClusterSplitKeepsWorldTransform\n");

	PhysicsWorldParam simParam;
	PhysicsWorld simulation(simParam);
	DestructionSet destructSet(simulation);
	destructSet.AddChunk(MakeChunkBounds(0.0f, 1.0f), 1.0f, false);
	destructSet.AddChunk(MakeChunkBounds(1.0f, 2.0f), 1.0f, false);
	destructSet.AddBond(0, 1, DestructionBondType::Connect, 1.0f, 0);
	destructSet.GetDestructionConstants().MinClusterVolume = 0.0f;
	destructSet.GetDestructionConstants().Levels[0].BreakThreshold = 1.0f;
	destructSet.GetDestructionConstants().Levels[0].PartitionSize = 2;
	destructSet.RebuildClustersFromGraph();

	std::vector<DestructionCluster*> clusters = destructSet.GetClusters();
	EXPECT(clusters.size() == 1);
	EXPECT(clusters[0]->GetRigidBody() != nullptr);

	RigidBodyDynamic* body = clusters[0]->GetRigidBody();
	const Vector3 offset(0.0f, -3.0f, 0.0f);
	body->X += offset;
	body->UpdateGeometries();

	const bool changed = destructSet.ApplyMomentum(0, Vector3(5.0f, 0.0f, 0.0f), body->X, Vector3::UnitY());
	EXPECT(changed);
	destructSet.Update(0.0f);
	int bodyCount = 0;
	for (DestructionCluster* cluster : destructSet.GetClusters())
	{
		if (cluster->GetRigidBody())
		{
			++bodyCount;
		}
	}
	EXPECT(bodyCount == 2);

	for (DestructionCluster* cluster : destructSet.GetClusters())
	{
		for (int chunkIndex : cluster->GetSourceIndices())
		{
			Geometry* geom = cluster->GetGeometryByChunkIndex(chunkIndex);
			EXPECT(geom != nullptr);
			const Vector3 expected = destructSet.GetChunks()[(size_t)chunkIndex].Centroid + offset;
			EXPECT_SAME(geom->GetWorldPosition(), expected);
		}
	}

	const bool changedFinal = destructSet.ApplyMomentum(0, Vector3(50.0f, 0.0f, 0.0f), destructSet.GetChunks()[0].Centroid + offset, Vector3::UnitY());
	EXPECT(!changedFinal);
	destructSet.Update(0.0f);
	bodyCount = 0;
	for (DestructionCluster* cluster : destructSet.GetClusters())
	{
		if (cluster->GetRigidBody())
		{
			++bodyCount;
		}
	}
	EXPECT(bodyCount == 2);

	DestructionSet::Stats stats = destructSet.GetStats();
	EXPECT(stats.FreeCount == 0);
	EXPECT(stats.SmashedCount == 0);
}

void TestStrainSolverDestruction()
{
	printf("Running TestStrainSolverDestruction\n");

	DestructionSet destructSet;
	destructSet.AddChunk(MakeChunkBounds(0.0f, 1.0f), 1.0f, true);
	destructSet.AddChunk(MakeChunkBounds(1.0f, 2.0f), 1.0f, false);
	destructSet.AddBond(0, 1, DestructionBondType::Connect, 0.01f, 77);
	destructSet.RebuildClustersFromGraph();

	StrainSolverSettings settings;
	settings.Hardness = 1.0f;
	settings.BondIterationsPerFrame = 64;
	StrainSolver solver(destructSet, settings);
	solver.AddForce(1, Vector3(100.0f, 0.0f, 0.0f), StrainForceMode::Impulse);
	solver.Update();

	EXPECT(solver.GetBondCount() == 1);
	EXPECT(solver.GetOverstressedBondCount() == 1);

	DestructionFractureBuffer commands;
	solver.GenerateFractureCommands(commands);
	EXPECT(commands.BondFractures.size() == 1);
	EXPECT(commands.BondFractures[0] == 77);

	destructSet.DoFractures(commands);
	EXPECT(!destructSet.GetGraph().HasBond(0, 1));
}

void TestPlanarCutBunny()
{
	printf("Running TestPlanarCutBunny\n");

	DynamicMesh bunny;
	bool loaded = bunny.LoadObj(TestDataPath("bunny.obj").c_str());
	EXPECT(loaded);
	if (!loaded)
	{
		return;
	}

	const Box3 bounds = bunny.GetBounds();
	const Vector3 center = bounds.GetCenter();
	const float maxSize = GetBoundsMaxSize(bounds);
	const Vector3 normal = Vector3(0.73f, 1.0f, 0.37f).SafeUnit();

	PlanarCutOptions options;
	options.SnapTolerance = std::max(maxSize * 1e-5f, 1e-6f);
	options.WeldSharedEdges = true;

	DynamicMesh negativeSide;
	DynamicMesh positiveSide;
	bool cut = MeshCut::Cut(bunny, center, normal, &negativeSide, &positiveSide, options);
	EXPECT(cut);
	EXPECT(CountValidTriangles(negativeSide) > 0);
	EXPECT(CountValidTriangles(positiveSide) > 0);

	DynamicMesh merged;
	const float pushDistance = maxSize * 0.55f;
	AppendMeshWithOffset(merged, negativeSide, -normal * pushDistance);
	AppendMeshWithOffset(merged, positiveSide, normal * pushDistance);
	merged.BuildBounds();
	EXPECT(merged.ExportObj(TestOutputPath("planar_cut.obj").c_str()));
}

void TestVoronoiFractureBunny()
{
	printf("Running TestVoronoiFractureBunny\n");

	DynamicMesh bunny;
	bool loaded = bunny.LoadObj(TestDataPath("bunny.obj").c_str());
	EXPECT(loaded);
	if (!loaded)
	{
		return;
	}

	const Box3 bounds = bunny.GetBounds();
	const Vector3 center = bounds.GetCenter();
	const Vector3 extent = bounds.GetExtent();
	const float maxSize = GetBoundsMaxSize(bounds);

	std::vector<Vector3> sites;
	sites.push_back(center + Vector3( extent.x * 0.35f, 0.0f, 0.0f));
	sites.push_back(center + Vector3(-extent.x * 0.35f, 0.0f, 0.0f));
	sites.push_back(center + Vector3(0.0f,  extent.y * 0.25f, 0.0f));
	sites.push_back(center + Vector3(0.0f, -extent.y * 0.25f, 0.0f));
	sites.push_back(center + Vector3(0.0f, 0.0f,  extent.z * 0.35f));
	sites.push_back(center + Vector3(0.0f, 0.0f, -extent.z * 0.35f));

	PlanarCutOptions options;
	options.SnapTolerance = std::max(maxSize * 1e-5f, 1e-6f);
	options.BoundsPaddingScale = 0.2f;
	options.Grout = maxSize * 0.015f;
	options.WeldSharedEdges = true;

	std::vector<FracturePiece> pieces;
	bool fractured = Fracture::VoronoiFracture(bunny, sites, pieces, options);
	EXPECT(fractured);
	EXPECT(pieces.size() >= 2);

	DynamicMesh merged;
	for (const FracturePiece& piece : pieces)
	{
		Vector3 direction = piece.Site - center;
		const float siteDistance = direction.SafeNormalize();
		if (siteDistance == 0.0f)
		{
			direction = Vector3(1.0f, 0.0f, 0.0f);
		}
		const Vector3 offset = direction * (maxSize * 0.65f + siteDistance * 1.5f);
		AppendMeshWithOffset(merged, piece.Mesh, offset);
	}
	merged.BuildBounds();
	EXPECT(merged.ExportObj(TestOutputPath("voronoi_fracture.obj").c_str()));
}

void TestPlanarCellsBunny()
{
	printf("Running TestPlanarCellsBunny\n");

	DynamicMesh bunny;
	bool loaded = bunny.LoadObj(TestDataPath("bunny.obj").c_str());
	EXPECT(loaded);
	if (!loaded)
	{
		return;
	}

	const Box3 bounds = bunny.GetBounds();
	const Vector3 center = bounds.GetCenter();
	const Vector3 extent = bounds.GetExtent();
	const float maxSize = GetBoundsMaxSize(bounds);

	std::vector<Box3> cells;
	cells.emplace_back(
		Vector3(bounds.Min.x, bounds.Min.y, bounds.Min.z),
		Vector3(center.x + extent.x * 0.10f, bounds.Max.y, bounds.Max.z));
	cells.emplace_back(
		Vector3(center.x - extent.x * 0.10f, bounds.Min.y, bounds.Min.z),
		Vector3(bounds.Max.x, bounds.Max.y, bounds.Max.z));

	PlanarCells planarCells(cells);
	EXPECT(planarCells.AssumeConvexCells);
	EXPECT(planarCells.NumCells == 2);
	EXPECT(planarCells.HasValidPlaneBoundaryOrientations());

	PlanarCutOptions options;
	options.SnapTolerance = std::max(maxSize * 1e-5f, 1e-6f);
	options.Grout = maxSize * 0.02f;
	options.WeldSharedEdges = true;

	std::vector<PlanarCutPiece> pieces;
	bool cut = MeshCut::CutWithPlanarCells(bunny, planarCells, pieces, options);
	EXPECT(cut);
	EXPECT(pieces.size() == 2);

	DynamicMesh merged;
	for (const PlanarCutPiece& piece : pieces)
	{
		Vector3 direction = piece.Center - center;
		if (direction.SafeNormalize() == 0.0f)
		{
			direction = Vector3(piece.CellIndex == 0 ? -1.0f : 1.0f, 0.0f, 0.0f);
		}
		AppendMeshWithOffset(merged, piece.Mesh, direction * maxSize * 0.55f);
	}

	DynamicMesh preview;
	bool previewBuilt = MeshCut::CreateCuttingSurfacePreview(planarCells, bounds, preview, options);
	EXPECT(previewBuilt);
	EXPECT(CountValidTriangles(preview) > 0);
	if (previewBuilt)
	{
		AppendMeshWithOffset(merged, preview, Vector3(0.0f, 0.0f, maxSize * 0.85f));
	}

	merged.BuildBounds();
	EXPECT(merged.ExportObj(TestOutputPath("planar_cells.obj").c_str()));
}

void TestDestruction()
{
	TestMetis();
	TestVoronoiMesh();
	TestConnectionGraphDestruction();
	TestConnectionGraphPartition();
	TestDestructionSetAndCluster();
	TestDestructionChunkConvexCollision();
	TestDestructionEvents();
	TestDestructionMomentum();
	TestDestructionClusterSplitKeepsWorldTransform();
	TestStrainSolverDestruction();
	TestPlanarCutBunny();
	TestVoronoiFractureBunny();
	TestPlanarCellsBunny();
	return;
}
