
#include "Test.h"
#include <algorithm>
#include <cfloat>
#include <cmath>
#include <vector>
#include "../Src/Destruction/Fracture.h"
#include "../Src/Destruction/GraphPartition.h"
#include "../Src/Destruction/DestructionSet.h"
#include "../Src/Geometry/MeshCut.h"
#include "../Src/Destruction/StrainSolver.h"
#include "../Src/Destruction/VoronoiTessellation.h"
#include "../Src/Collision/GeometryObject.h"
#include "../Src/Geometry/DynamicMesh.h"
#include "../Src/RigidBodyDynamics/CollidingContact.h"
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

static Vector3 GetGeometryPosition(const Geometry* geom)
{
	const RigidBody* body = geom ? geom->GetParent<RigidBody>() : nullptr;
	return body ? body->GetGeometryTransform(geom).pos : geom->GetPosition();
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

static int GetGridChunkIndex(int x, int y, int z, int countY, int countZ)
{
	return (x * countY + y) * countZ + z;
}

static void BuildChunkGrid(DestructionSet& destructSet, const Vector3& minCorner, int countX, int countY, int countZ, float cellSize, float mass)
{
	for (int x = 0; x < countX; ++x)
	{
		for (int y = 0; y < countY; ++y)
		{
			for (int z = 0; z < countZ; ++z)
			{
				const Vector3 cellMin = minCorner + Vector3((float)x, (float)y, (float)z) * cellSize;
				const Vector3 cellMax = cellMin + Vector3(cellSize, cellSize, cellSize);
				destructSet.AddChunk(Box3(cellMin, cellMax), mass, false);
			}
		}
	}

	int bondIndex = 0;
	for (int x = 0; x < countX; ++x)
	{
		for (int y = 0; y < countY; ++y)
		{
			for (int z = 0; z < countZ; ++z)
			{
				const int chunk = GetGridChunkIndex(x, y, z, countY, countZ);
				if (x + 1 < countX)
				{
					destructSet.AddBond(chunk, GetGridChunkIndex(x + 1, y, z, countY, countZ), DestructionBondType::Connect, 1.0f, bondIndex++);
				}
				if (y + 1 < countY)
				{
					destructSet.AddBond(chunk, GetGridChunkIndex(x, y + 1, z, countY, countZ), DestructionBondType::Connect, 1.0f, bondIndex++);
				}
				if (z + 1 < countZ)
				{
					destructSet.AddBond(chunk, GetGridChunkIndex(x, y, z + 1, countY, countZ), DestructionBondType::Connect, 1.0f, bondIndex++);
				}
			}
		}
	}
}

static void ConfigureRecursiveDestruction(DestructionSet& destructSet, float breakThreshold)
{
	DestructionConstants& constants = destructSet.GetDestructionConstants();
	constants.MinClusterVolume = 0.0f;
	constants.ImpulseProcessPerTick = 8;
	constants.PartitionProcessPerTick = 2;
	constants.PartitionProcessIntervalTicks = 1;
	for (int level = 0; level < 8; ++level)
	{
		constants.Levels[level].BreakThreshold = breakThreshold;
		constants.Levels[level].PartitionSize = 2;
	}
}

static bool BuildVoxelDestructionBox(DestructionSet& destructSet, const Box3& bounds, int countX, int countY, int countZ, float totalMass)
{
	DynamicMesh sourceMesh = MakeChunkMesh(bounds);
	PlanarCutOptions cutOptions;
	cutOptions.SnapTolerance = 1e-5f;
	cutOptions.BoundsPaddingScale = 0.10f;
	cutOptions.WeldSharedEdges = true;
	cutOptions.MinTriangleCount = 4;

	std::vector<FracturePiece> pieces;
	const int targetPieceCount = countX * countY * countZ;
	if (!Fracture::Voxel3D(sourceMesh, pieces, countX, countY, countZ, targetPieceCount, cutOptions) || pieces.empty())
	{
		return false;
	}
	float totalVolume = 0.0f;
	for (const FracturePiece& piece : pieces)
	{
		totalVolume += std::max(piece.Mesh.GetBounds().GetVolume(), 1e-3f);
	}
	if (totalVolume <= 0.0f)
	{
		return false;
	}

	std::vector<int> pieceToChunk(pieces.size(), -1);
	for (size_t pieceIndex = 0; pieceIndex < pieces.size(); ++pieceIndex)
	{
		const float pieceVolume = std::max(pieces[pieceIndex].Mesh.GetBounds().GetVolume(), 1e-3f);
		pieceToChunk[pieceIndex] = destructSet.AddChunk(pieces[pieceIndex].Mesh, totalMass * pieceVolume / totalVolume, false);
	}

	int bondIndex = 0;
	for (size_t pieceIndex = 0; pieceIndex < pieces.size(); ++pieceIndex)
	{
		for (int neighbor : pieces[pieceIndex].NeighborPieceIndices)
		{
			if (neighbor > (int)pieceIndex && neighbor < (int)pieceToChunk.size())
			{
				destructSet.AddBond(pieceToChunk[pieceIndex], pieceToChunk[(size_t)neighbor], DestructionBondType::Connect, 1.0f, bondIndex++);
			}
		}
	}
	return pieceToChunk.size() == (size_t)targetPieceCount;
}

static DestructionFractureSettings MakeHierarchicalFractureSettings()
{
	DestructionFractureSettings settings;
	settings.RandomSeed = 7321;
	settings.MaxChunkCount = 64;
	settings.CutRetryCount = 8;
	settings.MinChunkVolume = 0.001f;
	settings.BondStrength = 1.0f;
	settings.DamageAttenuation = 0.55f;
	settings.NeighborDamageFraction = 0.10f;

	DestructionFractureLevel firstLevel;
	firstLevel.MinPieceCount = 5;
	firstLevel.MaxPieceCount = 5;
	firstLevel.PreserveLargestPieces = 2;
	firstLevel.ChildFractureProbability = 1.0f;
	firstLevel.BreakThreshold = 5.0f;
	firstLevel.StrengthVariation = 0.10f;
	firstLevel.UseClusteredVoronoi = true;
	settings.Levels.push_back(firstLevel);

	DestructionFractureLevel secondLevel;
	secondLevel.MinPieceCount = 3;
	secondLevel.MaxPieceCount = 3;
	secondLevel.ChildFractureProbability = 1.0f;
	secondLevel.BreakThreshold = 40.0f;
	secondLevel.StrengthVariation = 0.15f;
	secondLevel.UseClusteredVoronoi = false;
	settings.Levels.push_back(secondLevel);

	DestructionFractureLevel thirdLevel;
	thirdLevel.MinPieceCount = 2;
	thirdLevel.MaxPieceCount = 2;
	thirdLevel.ChildFractureProbability = 1.0f;
	thirdLevel.BreakThreshold = 250.0f;
	thirdLevel.StrengthVariation = 0.20f;
	thirdLevel.UseClusteredVoronoi = false;
	settings.Levels.push_back(thirdLevel);
	return settings;
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
	EXPECT(clusterBody->GetNumGeometries() == clusters[0]->Size() + 1);
	EXPECT(clusters[0]->GetCollisionProxy() != nullptr);
	EXPECT(clusters[0]->GetCollisionProxy() == nullptr
		|| clusters[0]->GetCollisionProxy()->GetShapeType() == PrimitiveType::CONVEX_MESH);
	int simulationGeometryCount = 0;
	int queryGeometryCount = 0;
	for (Geometry* geometry : clusterBody->Geometries())
	{
		simulationGeometryCount += geometry && geometry->IsSimulationEnabled() ? 1 : 0;
		queryGeometryCount += geometry && geometry->IsQueryEnabled() ? 1 : 0;
	}
	EXPECT(simulationGeometryCount == 1);
	EXPECT(queryGeometryCount == 1);
	EXPECT(clusterBody->CastDynamic() == clusterBody);
	EXPECT(clusters[0]->GetGeometryByChunkIndex(2) == geometries[2]);
	EXPECT(clusters[0]->GetChunkIndexByGeometry(geometries[2]) == 2);
	EXPECT(geometries[2]->GetParent<RigidBody>() == clusterBody);
	EXPECT_SAME(GetGeometryPosition(geometries[2]), destructSet.GetChunks()[2].Centroid);
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
			EXPECT_SAME(GetGeometryPosition(geom), bounds.GetCenter());
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
	body->mMotionType = MorionType::Continuous;
	body->SetRestitution(0.37f);
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
		RigidBodyDynamic* splitBody = cluster->GetRigidBody();
		EXPECT(splitBody != nullptr);
		EXPECT(splitBody == nullptr || splitBody->mMotionType == MorionType::Continuous);
		EXPECT(splitBody == nullptr || std::fabs(splitBody->GetRestitution() - 0.37f) <= 1e-6f);
		for (int chunkIndex : cluster->GetSourceIndices())
		{
			Geometry* geom = cluster->GetGeometryByChunkIndex(chunkIndex);
			EXPECT(geom != nullptr);
			const Vector3 expected = destructSet.GetChunks()[(size_t)chunkIndex].Centroid + offset;
			EXPECT_SAME(GetGeometryPosition(geom), expected);
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

void TestDetachedStaticClusterWakesDynamicBody()
{
	printf("Running TestDetachedStaticClusterWakesDynamicBody\n");

	PhysicsWorldParam simParam;
	simParam.gravityAcc = Vector3(0.0f, -9.8f, 0.0f);
	PhysicsWorld simulation(simParam);
	DestructionSet destructSet(simulation);
	destructSet.AddChunk(Box3(Vector3(-0.5f, 4.0f, -0.5f), Vector3(0.5f, 5.0f, 0.5f)), 1.0f, true);
	destructSet.AddChunk(Box3(Vector3(0.6f, 5.0f, -0.5f), Vector3(1.6f, 6.0f, 0.5f)), 1.0f, false);
	destructSet.AddBond(0, 1, DestructionBondType::Connect, 1.0f, 17);
	destructSet.RebuildClustersFromGraph();

	DestructionCluster* initialCluster = destructSet.FindClusterContaining(1);
	EXPECT(initialCluster != nullptr);
	EXPECT(initialCluster == nullptr || initialCluster->IsStatic());

	DestructionFractureBuffer commands;
	commands.BondFractures.push_back(17);
	destructSet.DoFractures(commands);
	DestructionCluster* detachedCluster = destructSet.FindClusterContaining(1);
	RigidBodyDynamic* detachedBody = detachedCluster ? detachedCluster->GetRigidBody() : nullptr;
	EXPECT(detachedCluster != nullptr);
	EXPECT(detachedCluster == nullptr || !detachedCluster->IsStatic());
	EXPECT(detachedBody != nullptr);
	EXPECT(detachedBody == nullptr || !detachedBody->Sleeping);
	if (detachedBody)
	{
		const float startHeight = detachedBody->X.y;
		for (int frame = 0; frame < 10; ++frame)
		{
			simulation.Simulate();
		}
		EXPECT(detachedBody->X.y < startHeight);
	}
}

void TestDestructionSetReplacementAndWorldMigration()
{
	printf("Running TestDestructionSetReplacementAndWorldMigration\n");

	PhysicsWorldParam simParam;
	PhysicsWorld firstWorld(simParam);
	PhysicsWorld secondWorld(simParam);
	DestructionSet destructSet(firstWorld);
	destructSet.AddChunk(MakeChunkBounds(0.0f, 1.0f), 1.0f, false);
	destructSet.AddChunk(MakeChunkBounds(1.0f, 2.0f), 1.0f, false);
	destructSet.AddBond(0, 1, DestructionBondType::Connect, 1.0f, 0);
	destructSet.RebuildClustersFromGraph();
	EXPECT(destructSet.GetClusters().size() == 1);
	destructSet.GetDestructionConstants().MinClusterVolume = 0.0f;
	destructSet.GetDestructionConstants().Levels[0].BreakThreshold = 1.0f;
	destructSet.GetDestructionConstants().Levels[0].PartitionSize = 2;
	destructSet.AddMomentumQueue(0, Vector3(5.0f, 0.0f, 0.0f), Vector3(0.5f, 0.5f, 0.5f), Vector3::UnitX());
	destructSet.ClearClusters();
	destructSet.RebuildClustersFromGraph();
	destructSet.ProcessMomentumQueue();
	EXPECT(destructSet.GetClusters().size() == 1);

	secondWorld.AddDestructionSet(&destructSet);
	EXPECT(destructSet.GetPhysicsWorld() == &secondWorld);
	EXPECT(destructSet.GetClusters().size() == 1);
	EXPECT(destructSet.GetClusters().empty() || destructSet.GetClusters()[0]->GetRigidBody() != nullptr);

	destructSet.SetChunks(destructSet.GetChunks());
	EXPECT(destructSet.GetChunks().size() == 2);
	EXPECT(destructSet.GetGraph().Size() == 2);
	EXPECT(!destructSet.GetGraph().HasBond(0, 1));
	EXPECT(destructSet.GetClusters().empty());

	secondWorld.RemoveDestructionSet(&destructSet);
	EXPECT(destructSet.GetPhysicsWorld() == nullptr);
	EXPECT(destructSet.GetClusters().empty());
}

void TestHierarchyExternalFractureReconciliation()
{
	printf("Running TestHierarchyExternalFractureReconciliation\n");

	DestructionFractureSettings settings = MakeHierarchicalFractureSettings();
	settings.Levels.resize(1);
	settings.Levels[0].PreserveLargestPieces = 0;
	DestructionSet destructSet;
	EXPECT(destructSet.BuildHierarchicalFracture(
		MakeChunkMesh(Box3(Vector3(-3.0f, -2.0f, -2.0f), Vector3(3.0f, 2.0f, 2.0f))), settings, 12.0f));
	const int rootIndex = destructSet.GetHierarchyRoot();
	const std::vector<DestructionHierarchyNode>& hierarchy = destructSet.GetHierarchy();
	EXPECT(rootIndex >= 0 && (size_t)rootIndex < hierarchy.size());
	if (rootIndex < 0 || (size_t)rootIndex >= hierarchy.size() || hierarchy[(size_t)rootIndex].Children.size() < 2)
	{
		return;
	}

	const DestructionHierarchyNode& root = hierarchy[(size_t)rootIndex];
	int boundaryBondIndex = -1;
	for (size_t childA = 0; childA < root.Children.size() && boundaryBondIndex < 0; ++childA)
	{
		for (size_t childB = childA + 1; childB < root.Children.size() && boundaryBondIndex < 0; ++childB)
		{
			for (int chunkA : hierarchy[(size_t)root.Children[childA]].ChunkIndices)
			{
				for (int chunkB : hierarchy[(size_t)root.Children[childB]].ChunkIndices)
				{
					const Bond* bond = destructSet.GetGraph().FindBond(chunkA, chunkB);
					if (bond)
					{
						boundaryBondIndex = bond->bond_index;
						break;
					}
				}
				if (boundaryBondIndex >= 0)
				{
					break;
				}
			}
		}
	}
	EXPECT(boundaryBondIndex >= 0);
	if (boundaryBondIndex < 0)
	{
		return;
	}

	DestructionFractureBuffer commands;
	commands.BondFractures.push_back(boundaryBondIndex);
	destructSet.DoFractures(commands);
	EXPECT(hierarchy[(size_t)rootIndex].IsBroken);
	EXPECT(destructSet.GetClusters().size() > 1);

	const int extraChunkA = hierarchy[(size_t)root.Children[0]].ChunkIndices.front();
	const int extraChunkB = hierarchy[(size_t)root.Children[1]].ChunkIndices.front();
	destructSet.AddBond(extraChunkA, extraChunkB, DestructionBondType::Connect, 1.0f, 100000);
	destructSet.RebuildClustersFromGraph();
	EXPECT(!destructSet.GetGraph().HasBond(extraChunkA, extraChunkB));

	DestructionSet freedSet;
	EXPECT(freedSet.BuildHierarchicalFracture(
		MakeChunkMesh(Box3(Vector3(-3.0f, -2.0f, -2.0f), Vector3(3.0f, 2.0f, 2.0f))), settings, 12.0f));
	const int freedRootIndex = freedSet.GetHierarchyRoot();
	const std::vector<DestructionHierarchyNode>& freedHierarchy = freedSet.GetHierarchy();
	EXPECT(freedRootIndex >= 0 && (size_t)freedRootIndex < freedHierarchy.size());
	if (freedRootIndex >= 0 && (size_t)freedRootIndex < freedHierarchy.size()
		&& !freedHierarchy[(size_t)freedRootIndex].Children.empty())
	{
		const int freedChildIndex = freedHierarchy[(size_t)freedRootIndex].Children.front();
		const int freedChunkIndex = freedHierarchy[(size_t)freedChildIndex].ChunkIndices.front();
		freedSet.AddDestructionInfo(DestructionEventType::Destruct, { freedChunkIndex });
		freedSet.ProcessDestructionInfo();
		EXPECT(freedSet.GetChunks()[(size_t)freedChunkIndex].IsFree);
		EXPECT(freedHierarchy[(size_t)freedRootIndex].IsBroken);
		EXPECT(freedSet.GetGraph()[(size_t)freedChunkIndex].Bonds.empty());
	}
}

void TestHierarchicalFractureRegression()
{
	printf("Running TestHierarchicalFractureRegression\n");

	const DestructionFractureSettings settings = MakeHierarchicalFractureSettings();
	const Box3 sourceBounds(Vector3(-5.0f, -4.0f, -3.0f), Vector3(5.0f, 4.0f, 3.0f));
	const float totalMass = 37.5f;
	DestructionSet destructSet;
	const bool built = destructSet.BuildHierarchicalFracture(MakeChunkMesh(sourceBounds), settings, totalMass, false, "hierarchy");
	EXPECT(built);
	if (!built)
	{
		return;
	}

	const std::vector<DestructionChunk>& chunks = destructSet.GetChunks();
	const std::vector<DestructionHierarchyNode>& hierarchy = destructSet.GetHierarchy();
	const int rootIndex = destructSet.GetHierarchyRoot();
	EXPECT(rootIndex >= 0 && (size_t)rootIndex < hierarchy.size());
	if (rootIndex < 0 || (size_t)rootIndex >= hierarchy.size())
	{
		return;
	}

	const DestructionHierarchyNode& root = hierarchy[(size_t)rootIndex];
	EXPECT(root.Parent == -1);
	EXPECT(root.Level == 0);
	EXPECT(root.IsBreakable);
	EXPECT(root.Children.size() >= 4 && root.Children.size() <= 5);
	EXPECT(chunks.size() >= 12);
	EXPECT(chunks.size() <= (size_t)settings.MaxChunkCount);
	EXPECT(root.ChunkIndices.size() == chunks.size());

	std::vector<int> rootMembership(chunks.size(), 0);
	for (int chunkIndex : root.ChunkIndices)
	{
		EXPECT(chunkIndex >= 0 && (size_t)chunkIndex < chunks.size());
		if (chunkIndex >= 0 && (size_t)chunkIndex < chunks.size())
		{
			++rootMembership[(size_t)chunkIndex];
		}
	}
	for (int membership : rootMembership)
	{
		EXPECT(membership == 1);
	}

	int leafNodeCount = 0;
	int maximumLevel = 0;
	float minimumBreakThreshold = FLT_MAX;
	float maximumBreakThreshold = 0.0f;
	bool hasBreakableLevel[3] = { false, false, false };
	for (size_t nodeIndex = 0; nodeIndex < hierarchy.size(); ++nodeIndex)
	{
		const DestructionHierarchyNode& node = hierarchy[nodeIndex];
		maximumLevel = std::max(maximumLevel, node.Level);
		if (node.IsBreakable && node.Level >= 0 && node.Level < 3)
		{
			hasBreakableLevel[node.Level] = true;
			minimumBreakThreshold = std::min(minimumBreakThreshold, node.BreakThreshold);
			maximumBreakThreshold = std::max(maximumBreakThreshold, node.BreakThreshold);
		}

		if ((int)nodeIndex != rootIndex)
		{
			EXPECT(node.Parent >= 0 && (size_t)node.Parent < hierarchy.size());
			if (node.Parent >= 0 && (size_t)node.Parent < hierarchy.size())
			{
				const DestructionHierarchyNode& parent = hierarchy[(size_t)node.Parent];
				EXPECT(node.Level == parent.Level + 1);
				EXPECT(std::find(parent.Children.begin(), parent.Children.end(), (int)nodeIndex) != parent.Children.end());
			}
		}

		std::vector<int> childMembership(chunks.size(), 0);
		for (int childIndex : node.Children)
		{
			EXPECT(childIndex >= 0 && (size_t)childIndex < hierarchy.size());
			if (childIndex < 0 || (size_t)childIndex >= hierarchy.size())
			{
				continue;
			}
			EXPECT(hierarchy[(size_t)childIndex].Parent == (int)nodeIndex);
			for (int chunkIndex : hierarchy[(size_t)childIndex].ChunkIndices)
			{
				EXPECT(chunkIndex >= 0 && (size_t)chunkIndex < chunks.size());
				if (chunkIndex >= 0 && (size_t)chunkIndex < chunks.size())
				{
					++childMembership[(size_t)chunkIndex];
				}
			}
		}

		if (node.Children.empty())
		{
			++leafNodeCount;
			EXPECT(!node.IsBreakable);
			EXPECT(node.ChunkIndices.size() == 1);
		}
		else
		{
			EXPECT(node.IsBreakable);
			EXPECT(node.Children.size() >= 2);
			for (int chunkIndex : node.ChunkIndices)
			{
				EXPECT(chunkIndex >= 0 && (size_t)chunkIndex < chunks.size());
				if (chunkIndex >= 0 && (size_t)chunkIndex < chunks.size())
				{
					EXPECT(childMembership[(size_t)chunkIndex] == 1);
				}
			}
		}
	}
	EXPECT(leafNodeCount == (int)chunks.size());
	EXPECT(maximumLevel == 3);
	EXPECT(hasBreakableLevel[0]);
	EXPECT(hasBreakableLevel[1]);
	EXPECT(hasBreakableLevel[2]);
	EXPECT(maximumBreakThreshold > minimumBreakThreshold * 10.0f);

	float massSum = 0.0f;
	float minimumVolume = FLT_MAX;
	float maximumVolume = 0.0f;
	float minimumBoundsVolume = FLT_MAX;
	float maximumBoundsVolume = 0.0f;
	for (const DestructionChunk& chunk : chunks)
	{
		EXPECT(chunk.Mass > 0.0f);
		EXPECT(chunk.Volume > 0.0f);
		massSum += chunk.Mass;
		minimumVolume = std::min(minimumVolume, chunk.Volume);
		maximumVolume = std::max(maximumVolume, chunk.Volume);
		const float boundsVolume = chunk.Bounds.GetVolume();
		minimumBoundsVolume = std::min(minimumBoundsVolume, boundsVolume);
		maximumBoundsVolume = std::max(maximumBoundsVolume, boundsVolume);
	}
	EXPECT(std::fabs(massSum - totalMass) <= totalMass * 1e-4f);
	EXPECT(maximumVolume > minimumVolume * 1.10f);
	EXPECT(maximumBoundsVolume > minimumBoundsVolume * 1.10f);

	int permanentNode = -1;
	int recursiveNode = -1;
	int recursiveGrandchildCount = 0;
	int permanentFirstLevelCount = 0;
	for (int childIndex : root.Children)
	{
		if (childIndex < 0 || (size_t)childIndex >= hierarchy.size())
		{
			continue;
		}
		const DestructionHierarchyNode& child = hierarchy[(size_t)childIndex];
		if (child.Children.empty())
		{
			++permanentFirstLevelCount;
			if (permanentNode < 0)
			{
				permanentNode = childIndex;
			}
			continue;
		}

		int breakableGrandchildren = 0;
		for (int grandchildIndex : child.Children)
		{
			if (grandchildIndex >= 0 && (size_t)grandchildIndex < hierarchy.size()
				&& hierarchy[(size_t)grandchildIndex].IsBreakable)
			{
				++breakableGrandchildren;
			}
		}
		if (breakableGrandchildren > recursiveGrandchildCount)
		{
			recursiveNode = childIndex;
			recursiveGrandchildCount = breakableGrandchildren;
		}
	}
	EXPECT(permanentFirstLevelCount >= settings.Levels[0].PreserveLargestPieces);
	EXPECT(permanentNode >= 0);
	EXPECT(recursiveNode >= 0);
	EXPECT(recursiveGrandchildCount >= 2);
	if (permanentNode < 0 || recursiveNode < 0 || recursiveGrandchildCount < 1
		|| hierarchy[(size_t)permanentNode].ChunkIndices.empty())
	{
		return;
	}

	const int permanentChunk = hierarchy[(size_t)permanentNode].ChunkIndices.front();
	int impactChunk = -1;
	for (int grandchildIndex : hierarchy[(size_t)recursiveNode].Children)
	{
		if (grandchildIndex >= 0 && (size_t)grandchildIndex < hierarchy.size()
			&& hierarchy[(size_t)grandchildIndex].IsBreakable
			&& !hierarchy[(size_t)grandchildIndex].ChunkIndices.empty())
		{
			impactChunk = hierarchy[(size_t)grandchildIndex].ChunkIndices.front();
			break;
		}
	}
	EXPECT(impactChunk >= 0 && (size_t)impactChunk < chunks.size());
	if (impactChunk < 0 || (size_t)impactChunk >= chunks.size())
	{
		return;
	}

	DestructionConstants& constants = destructSet.GetDestructionConstants();
	constants.MinClusterVolume = 0.0f;
	constants.ImpulseBreakStrength = 0.0f;
	constants.PartitionProcessPerTick = 1;
	constants.PartitionProcessIntervalTicks = 1;

	// Open only the root. The attenuated impulse remains far below every level-one threshold.
	const float rootImpulse = root.BreakThreshold * 1.05f + 0.01f;
	const bool rootChanged = destructSet.ApplyMomentum(impactChunk, Vector3::UnitX() * rootImpulse,
		chunks[(size_t)impactChunk].Centroid, Vector3::UnitX());
	EXPECT(rootChanged);
	EXPECT(hierarchy[(size_t)rootIndex].IsBroken);
	for (int childIndex : root.Children)
	{
		if (childIndex >= 0 && (size_t)childIndex < hierarchy.size())
		{
			EXPECT(!hierarchy[(size_t)childIndex].IsBroken);
		}
	}

	DestructionCluster* permanentCluster = destructSet.FindClusterContaining(permanentChunk);
	EXPECT(permanentCluster != nullptr);
	EXPECT(permanentCluster == nullptr || permanentCluster->Size() == 1);
	const size_t clusterCountBeforePermanentImpact = destructSet.GetClusters().size();
	const uint32_t revisionBeforePermanentImpact = destructSet.GetClusterRevision();
	const bool permanentChanged = destructSet.ApplyMomentum(permanentChunk, Vector3(1000000.0f, -250000.0f, 750000.0f),
		chunks[(size_t)permanentChunk].Centroid, Vector3::UnitY());
	EXPECT(!permanentChanged);
	EXPECT(!hierarchy[(size_t)permanentNode].IsBroken);
	EXPECT(destructSet.GetClusters().size() == clusterCountBeforePermanentImpact);
	EXPECT(destructSet.GetClusterRevision() == revisionBeforePermanentImpact);

	const size_t clusterCountBeforeRecursiveImpact = destructSet.GetClusters().size();
	const bool recursiveChanged = destructSet.ApplyMomentum(impactChunk, Vector3(100000.0f, 0.0f, 0.0f),
		chunks[(size_t)impactChunk].Centroid, Vector3::UnitX());
	EXPECT(recursiveChanged);
	EXPECT(hierarchy[(size_t)recursiveNode].IsBroken);
	EXPECT(destructSet.GetClusters().size() > clusterCountBeforeRecursiveImpact);

	int brokenGrandchildren = 0;
	for (int grandchildIndex : hierarchy[(size_t)recursiveNode].Children)
	{
		if (grandchildIndex >= 0 && (size_t)grandchildIndex < hierarchy.size()
			&& hierarchy[(size_t)grandchildIndex].IsBroken)
		{
			++brokenGrandchildren;
		}
	}
	EXPECT(brokenGrandchildren == 0);

	// One update may consume only one queued child, so a deep impact cannot flatten the hierarchy in one frame.
	destructSet.Update(0.0f);
	brokenGrandchildren = 0;
	for (int grandchildIndex : hierarchy[(size_t)recursiveNode].Children)
	{
		if (grandchildIndex >= 0 && (size_t)grandchildIndex < hierarchy.size()
			&& hierarchy[(size_t)grandchildIndex].IsBroken)
		{
			++brokenGrandchildren;
		}
	}
	EXPECT(brokenGrandchildren == 1);
	for (int childIndex : root.Children)
	{
		if (childIndex >= 0 && (size_t)childIndex < hierarchy.size() && childIndex != recursiveNode)
		{
			EXPECT(!hierarchy[(size_t)childIndex].IsBroken);
		}
	}

	int singletonClusterCount = 0;
	for (DestructionCluster* cluster : destructSet.GetClusters())
	{
		EXPECT(cluster != nullptr);
		if (cluster && cluster->Size() == 1)
		{
			++singletonClusterCount;
		}
	}
	EXPECT(singletonClusterCount < (int)chunks.size());
	EXPECT(destructSet.GetClusters().size() < chunks.size());
}

void TestRecursiveDestructionToLeafBodies()
{
	printf("Running TestRecursiveDestructionToLeafBodies\n");

	PhysicsWorldParam simParam;
	PhysicsWorld simulation(simParam);
	DestructionSet destructSet(simulation);
	BuildChunkGrid(destructSet, Vector3(-1.5f, -1.5f, -1.5f), 3, 3, 3, 1.0f, 1.0f);
	ConfigureRecursiveDestruction(destructSet, 1.0f);
	destructSet.GetDestructionConstants().ImpulseBreakStrength = 0.0f;
	destructSet.RebuildClustersFromGraph();

	std::vector<DestructionCluster*> clusters = destructSet.GetClusters();
	EXPECT(clusters.size() == 1);
	RigidBodyDynamic* parentBody = clusters.empty() ? nullptr : clusters[0]->GetRigidBody();
	EXPECT(parentBody != nullptr);
	if (parentBody == nullptr)
	{
		return;
	}

	const Vector3 parentPosition = parentBody->X;
	const Vector3 parentLinearVelocity(1.5f, -2.0f, 0.75f);
	const Vector3 parentAngularVelocity(0.15f, 0.35f, -0.2f);
	parentBody->SetLinearVelocity(parentLinearVelocity);
	parentBody->SetAngularVelocity(parentAngularVelocity);

	const bool changed = destructSet.ApplyMomentum(13, Vector3(0.0f, 100.0f, 0.0f), Vector3(0.0f, -2.0f, 0.0f), Vector3::UnitY());
	EXPECT(changed);
	clusters = destructSet.GetClusters();
	EXPECT(clusters.size() > 1);
	EXPECT(clusters.size() < 27);
	int previousClusterCount = (int)clusters.size();
	for (int iteration = 0; iteration < 64 && clusters.size() < 27; ++iteration)
	{
		destructSet.Update(0.0f);
		clusters = destructSet.GetClusters();
		EXPECT((int)clusters.size() >= previousClusterCount);
		previousClusterCount = (int)clusters.size();
	}
	EXPECT(clusters.size() == 27);
	for (DestructionCluster* cluster : clusters)
	{
		EXPECT(cluster != nullptr);
		EXPECT(cluster == nullptr || cluster->Size() == 1);
		RigidBodyDynamic* body = cluster ? cluster->GetRigidBody() : nullptr;
		EXPECT(body != nullptr);
		if (body)
		{
			const Vector3 expectedLinearVelocity = parentLinearVelocity + parentAngularVelocity.Cross(body->X - parentPosition);
			EXPECT_SAME(body->GetLinearVelocity(), expectedLinearVelocity);
			EXPECT_SAME(body->GetAngularVelocity(), parentAngularVelocity);
		}
	}

	DestructionSet::Stats stats = destructSet.GetStats();
	EXPECT(stats.NormalCount == 27);
	EXPECT(stats.FreeCount == 0);
	EXPECT(stats.SmashedCount == 0);
}

void TestIterativeDestructionThrottlesThousandChunks()
{
	printf("Running TestIterativeDestructionThrottlesThousandChunks\n");

	DestructionSet destructSet;
	BuildChunkGrid(destructSet, Vector3(-5.0f, -5.0f, -5.0f), 10, 10, 10, 1.0f, 1.0f);
	ConfigureRecursiveDestruction(destructSet, 1.0f);
	DestructionConstants& constants = destructSet.GetDestructionConstants();
	constants.ImpulseBreakStrength = 0.0f;
	constants.PartitionProcessPerTick = 16;
	constants.PartitionProcessIntervalTicks = 2;
	destructSet.RebuildClustersFromGraph();

	EXPECT(destructSet.GetClusters().size() == 1);
	EXPECT(destructSet.ApplyMomentum(555, Vector3(0.0f, 100.0f, 0.0f), Vector3::Zero(), Vector3::UnitY()));
	size_t clusterCount = destructSet.GetClusters().size();
	EXPECT(clusterCount > 1);
	EXPECT(clusterCount < 1000);

	destructSet.Update(0.0f);
	const size_t afterFirstBatch = destructSet.GetClusters().size();
	EXPECT(afterFirstBatch > clusterCount);
	EXPECT(afterFirstBatch < 1000);
	destructSet.Update(0.0f);
	EXPECT(destructSet.GetClusters().size() == afterFirstBatch);
	clusterCount = afterFirstBatch;
	int growthTickCount = 1;

	for (int tick = 0; tick < 256 && clusterCount < 1000; ++tick)
	{
		destructSet.Update(0.0f);
		const size_t nextClusterCount = destructSet.GetClusters().size();
		EXPECT(nextClusterCount >= clusterCount);
		if (nextClusterCount > clusterCount)
		{
			++growthTickCount;
		}
		clusterCount = nextClusterCount;
	}

	EXPECT(clusterCount == 1000);
	EXPECT(growthTickCount >= 5);
	for (DestructionCluster* cluster : destructSet.GetClusters())
	{
		EXPECT(cluster != nullptr);
		EXPECT(cluster == nullptr || cluster->Size() == 1);
	}
}

void TestDestructionDropContactCallback()
{
	printf("Running TestDestructionDropContactCallback\n");

	PhysicsWorldParam simParam;
	simParam.gravityAcc = Vector3(0.0f, -9.8f, 0.0f);
	simParam.broadphase = BroadPhaseSolver::SAP;
	PhysicsWorld simulation(simParam);

	Geometry* ground = GeometryFactory::CreateOBB(Vector3(0.0f, -0.5f, 0.0f), Vector3(20.0f, 0.5f, 20.0f));
	RigidBodyParam groundParam;
	groundParam.rigidType = RigidType::Static;
	EXPECT(simulation.CreateRigidBody(ground, groundParam) != nullptr);

	DestructionSet destructSet(simulation);
	BuildChunkGrid(destructSet, Vector3(-1.5f, 4.0f, -1.5f), 3, 3, 3, 1.0f, 0.5f);
	ConfigureRecursiveDestruction(destructSet, 1.0f);
	destructSet.RebuildClustersFromGraph();

	bool shattered = false;
	for (int frame = 0; frame < 240; ++frame)
	{
		simulation.Simulate();
		std::vector<DestructionCluster*> clusters = destructSet.GetClusters();
		if (clusters.size() == 27)
		{
			shattered = true;
			for (DestructionCluster* cluster : clusters)
			{
				if (cluster == nullptr || cluster->Size() != 1 || cluster->GetRigidBody() == nullptr)
				{
					shattered = false;
					break;
				}
			}
		}
		if (shattered)
		{
			break;
		}
	}
	EXPECT(shattered);
}

void TestCompoundRigidBodyContactResolution()
{
	printf("Running TestCompoundRigidBodyContactResolution\n");

	PhysicsWorldParam simParam;
	simParam.gravityAcc = Vector3(0.0f, -9.8f, 0.0f);
	simParam.broadphase = BroadPhaseSolver::SAP;
	PhysicsWorld simulation(simParam);

	Geometry* ground = GeometryFactory::CreateOBB(Vector3(0.0f, -0.5f, 0.0f), Vector3(8.0f, 0.5f, 8.0f));
	RigidBodyParam groundParam;
	groundParam.rigidType = RigidType::Static;
	EXPECT(simulation.CreateRigidBody(ground, groundParam) != nullptr);

	std::vector<Geometry*> geometries;
	geometries.push_back(GeometryFactory::CreateOBB(Vector3(0.0f, 4.0f, 0.0f), Vector3(0.5f, 0.5f, 0.5f)));
	geometries.push_back(GeometryFactory::CreateOBB(Vector3(0.0f, 5.0f, 0.0f), Vector3(0.5f, 0.5f, 0.5f)));
	RigidBodyParam bodyParam;
	bodyParam.rigidType = RigidType::Dynamic;
	RigidBody* createdBody = simulation.CreateRigidBody(geometries, bodyParam, Transform(Vector3(0.0f, 4.5f, 0.0f)));
	RigidBodyDynamic* body = createdBody ? createdBody->CastDynamic() : nullptr;
	EXPECT(body != nullptr);
	if (body == nullptr)
	{
		return;
	}

	ContactManifold groundContact0;
	groundContact0.indexA = 0;
	groundContact0.indexB = 1;
	ContactManifold groundContact1;
	groundContact1.indexA = 0;
	groundContact1.indexB = 2;
	std::vector<Geometry*> islandGeometries = { ground, geometries[0], geometries[1] };
	std::vector<ContactManifold*> islandManifolds = { &groundContact0, &groundContact1 };
	ContactManifoldIslands contactIslands;
	contactIslands.BuildIslands(islandGeometries, islandManifolds);
	EXPECT(contactIslands.islands.size() == 1);
	EXPECT(contactIslands.islands.empty() || contactIslands.islands[0].size() == 2);

	for (int frame = 0; frame < 240; ++frame)
	{
		simulation.Simulate();
	}
	EXPECT(body->X.y > 0.5f);
	EXPECT(body->GetLinearVelocity().y > -1.0f);
}

void TestVoxelDestructionDropContactCallback()
{
	printf("Running TestVoxelDestructionDropContactCallback\n");

	PhysicsWorldParam simParam;
	simParam.gravityAcc = Vector3(0.0f, -9.8f, 0.0f);
	simParam.broadphase = BroadPhaseSolver::SAP;
	PhysicsWorld simulation(simParam);

	Geometry* ground = GeometryFactory::CreateOBB(Vector3(0.0f, -0.5f, 0.0f), Vector3(20.0f, 0.5f, 20.0f));
	RigidBodyParam groundParam;
	groundParam.rigidType = RigidType::Static;
	EXPECT(simulation.CreateRigidBody(ground, groundParam) != nullptr);

	DestructionSet destructSet(simulation);
	EXPECT(BuildVoxelDestructionBox(destructSet, Box3(Vector3(-1.6f, 8.8f, -1.6f), Vector3(1.6f, 11.2f, 1.6f)), 3, 3, 3, 12.5f));
	ConfigureRecursiveDestruction(destructSet, 1.0f);
	destructSet.RebuildClustersFromGraph();

	std::vector<DestructionCluster*> clusters = destructSet.GetClusters();
	EXPECT(clusters.size() == 1);
	if (!clusters.empty())
	{
		for (Geometry* geometry : clusters[0]->GetGeometries())
		{
			EXPECT(geometry != nullptr);
			if (geometry)
			{
				const Vector3 size = geometry->GetShapeBounds().GetSize();
				EXPECT(size.x > 0.1f && size.y > 0.1f && size.z > 0.1f);
			}
		}
	}

	bool shattered = false;
	for (int frame = 0; frame < 300; ++frame)
	{
		simulation.Simulate();
		clusters = destructSet.GetClusters();
		shattered = clusters.size() == 27;
		if (shattered)
		{
			break;
		}
	}
	EXPECT(shattered);
	if (shattered)
	{
		for (int frame = 0; frame < 120; ++frame)
		{
			simulation.Simulate();
		}
	}

	float minimumBodyHeight = FLT_MAX;
	for (DestructionCluster* cluster : destructSet.GetClusters())
	{
		RigidBodyDynamic* body = cluster ? cluster->GetRigidBody() : nullptr;
		EXPECT(body != nullptr);
		if (body)
		{
			minimumBodyHeight = std::min(minimumBodyHeight, body->X.y);
		}
	}
	EXPECT(minimumBodyHeight > -2.0f);
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
	TestDetachedStaticClusterWakesDynamicBody();
	TestDestructionSetReplacementAndWorldMigration();
	TestHierarchyExternalFractureReconciliation();
	TestHierarchicalFractureRegression();
	TestRecursiveDestructionToLeafBodies();
	TestIterativeDestructionThrottlesThousandChunks();
	TestDestructionDropContactCallback();
	TestCompoundRigidBodyContactResolution();
	TestVoxelDestructionDropContactCallback();
	TestStrainSolverDestruction();
	TestPlanarCutBunny();
	TestVoronoiFractureBunny();
	TestPlanarCellsBunny();
	return;
}
