#include "../Src/RigidBodyDynamics/Island.h"
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
#include "../Src/CollisionPrimitive/ConvexMesh.h"
#include "../Src/Geometry/ConvexHull3.h"
#include <limits>
#include <map>
#include <stdexcept>
#include <set>
#include "../Src/Collision/GeometryQuery.h"
#include "../Src/Deformation/DeformableBody.h"

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

namespace Test_DestructionAsset_Cases
{
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
}
void TestDestructionAsset()
{
	try { HullVisibility(); SnapshotsAndInstances(); StressAuthoring(); HierarchicalAuthoring(); Validation();  }
	catch (const std::exception& Error) { printf("Destruction asset test exception: %s\n",Error.what()); EXPECT(false); }
}
}

void TestDestructionAsset() { Test_DestructionAsset_Cases::TestDestructionAsset(); }

namespace Test_DestructionEvents_Cases
{
using namespace Riemann;
namespace
{
	template<class F> void Reject(F Function)
	{ bool Caught = false; try { Function(); } catch (const std::exception&) { Caught = true; } EXPECT(Caught); }
	void Author(DestructionSet& Set, int Count = 3, bool Support = false, float Strength = 1)
	{
		Set.GetDestructionConstants().MinClusterVolume = 0;
		for (int I = 0; I < Count; ++I)
		{
			const Vector3 Center((float)I,0,0); Set.AddChunk(Box3(Center-Vector3(0.5f),Center+Vector3(0.5f)),(float)(I+1),Support && I == 0);
			if (I) EXPECT(Set.AddBond(I-1,I,DestructionBondType::Connect,Strength,10+I));
		}
		Set.RebuildClustersFromGraph();
	}
	void Enable(DestructionSet& Set)
	{ DestructionEventSettings S; S.Enabled = true; Set.ConfigureEvents(S); }
	const DestructionEvent* Find(const std::vector<DestructionEvent>& Events, DestructionEventKind Kind)
	{ for (const auto& E : Events) if (E.Kind == Kind) return &E; return nullptr; }
	void IdentityAndKinematics()
	{
		DestructionSet Source; Author(Source); const auto Asset = DestructionAsset::Capture(Source);
		PhysicsWorld World{PhysicsWorldParam()}; DestructionSet Set(World); Set.InitializeAsset(Asset); Enable(Set);
		const auto Id = Set.GetInstanceId(), Generation = Set.GetGeneration();
		auto* Cluster = Set.FindClusterContaining(0); const auto ClusterId = Cluster->GetID(); auto* Body = Cluster->GetRigidBody();
		Body->X = Vector3(5,3,-2); Body->Q.FromRotationAxis(Vector3(1,2,3).Unit(),0.7f);
		Body->SetLinearVelocity(Vector3(1,2,3)); Body->SetAngularVelocity(Vector3(0.1f,0.2f,0.3f));
		Transform Pose; EXPECT(Cluster->GetWorldTransform(0,Pose)); const Vector3 ExpectedV = Body->V + Body->W.Cross(Pose.pos-Body->X);
		const Vector3 ExpectedPoint = Pose.LocalToWorld(Set.GetGraph().FindBond(0,1)->frame.Origin-Set.GetChunks()[0].Centroid);
		Set.DoFractures({{11,11,-1,999}}); auto Events = Set.TakeEvents(); EXPECT(Events.size() == 2);
		const auto* Broken = Find(Events,DestructionEventKind::BondBroken); const auto* Changed = Find(Events,DestructionEventKind::ClusterChanged);
		EXPECT(Broken && Changed);
		if (Broken)
		{
			EXPECT(Broken->BondIndex == 11 && Broken->NodeA == 0 && Broken->NodeB == 1);
			EXPECT(Broken->Cause == DestructionEventCause::Command && Broken->BondType == DestructionBondType::Connect);
			EXPECT_NEAR(Broken->BondStrength,1,0); EXPECT(Broken->Chunks.size() == 2);
			EXPECT_NEAR((Broken->Position-ExpectedPoint).Length(),0,1e-5);
			if (!Broken->Chunks.empty())
			{
				const auto& C = Broken->Chunks[0]; EXPECT(C.ChunkIndex == 0 && C.ClusterId == ClusterId && C.HasRigidBody);
				EXPECT_NEAR((C.Pose.pos-Pose.pos).Length(),0,1e-5); EXPECT_NEAR((C.LinearVelocity-ExpectedV).Length(),0,1e-5);
				EXPECT_NEAR(C.Mass,1,0);
			}
		}
		if (Changed)
		{
			EXPECT(Changed->PreviousClusterId == ClusterId && Changed->Chunks.size() == 3 && Changed->Fragments.size() == 2);
			std::set<int> Members; for (const auto& F : Changed->Fragments) for (int I : F.ChunkIndices) Members.insert(I);
			EXPECT(Members == std::set<int>({0,1,2}));
		}
		uint64_t Sequence = 0;
		for (const auto& E : Events)
		{
			EXPECT(E.InstanceId == Id && E.Generation == Generation && E.Sequence > Sequence);
			EXPECT(E.AssetFingerprint == Asset.GetStats().Fingerprint); Sequence = E.Sequence;
		}
		Set.RebuildClustersFromGraph(); Set.DoFractures({{11}}); EXPECT(Set.TakeEvents().empty());
		Set.Clear(); EXPECT(Set.GetInstanceId() == Id && Set.GetGeneration() > Generation && Set.TakeEvents().empty());
		World.Reset(); EXPECT(Events.size() == 2 && Events.front().InstanceId == Id);
		EXPECT(Source.GetInstanceId() != Id);
	}
	void CapacityAndFiltering()
	{
		DestructionSet Set; Author(Set); DestructionEventSettings S; S.Enabled = true;
		S.KindMask = 1u << (uint32_t)DestructionEventKind::BondBroken; S.MaximumQueuedEvents = 1; Set.ConfigureEvents(S);
		Set.DoFractures({{11,12}}); const auto Stats = Set.GetEventStats();
		EXPECT(Stats.Generated == 3 && Stats.Filtered == 1 && Stats.Dropped == 1 && Stats.Invalid == 0 && Stats.Pending == 1);
		auto Bad = S; Bad.MaximumQueuedEvents = 0; Reject([&] { Set.ConfigureEvents(Bad); });
		Bad = S; Bad.KindMask = ~DestructionEventSettings::AllKinds; Reject([&] { Set.ConfigureEvents(Bad); });
		Bad = S; Bad.MinimumContactImpulse = std::numeric_limits<float>::quiet_NaN(); Reject([&] { Set.ConfigureEvents(Bad); });
		auto Events = Set.TakeEvents(); EXPECT(Events.size() == 1 && Events[0].BondIndex == 11);
		EXPECT(Set.GetEventStats().Pending == 0); Set.RebuildClustersFromGraph(); EXPECT(Set.TakeEvents().empty());
		Set.Clear(); Author(Set); EXPECT(Set.GetEventStats().Generated == 0);
		Set.DoFractures({{11}}); Events = Set.TakeEvents(); EXPECT(Events.size() == 1 && Events[0].Sequence > 3);
		S.Enabled = false; Set.ConfigureEvents(S); Set.DoFractures({{12}}); EXPECT(Set.TakeEvents().empty());
	}
	void DirectedSupportBond()
	{
		DestructionSet Set; Set.AddChunk(Box3(Vector3(-0.5f),Vector3(0.5f)),1,true);
		Set.AddChunk(Box3(Vector3(0.5f,-0.5f,-0.5f),Vector3(1.5f,0.5f,0.5f)),2);
		EXPECT(Set.GetGraph().AddDirectedBond(1,0,DestructionBondType::Support,3,77)); Set.RebuildClustersFromGraph(); Enable(Set);
		Set.DoFractures({{77}}); const auto Events = Set.TakeEvents(); const auto* B = Find(Events,DestructionEventKind::BondBroken); EXPECT(B);
		if (B)
		{
			EXPECT(B->NodeA == 1 && B->NodeB == 0 && B->BondType == DestructionBondType::Support && B->Chunks.size() == 2);
			EXPECT_NEAR(B->BondStrength,3,0); EXPECT(B->Chunks[0].ChunkIndex == 1 && B->Chunks[1].ChunkIndex == 0);
		}
	}
	void ReleaseAndExternalGraphEdits()
	{
		PhysicsWorld World{PhysicsWorldParam()}; DestructionSet Set(World); Author(Set); Enable(Set);
		Set.GetGraph().BreakBond(0,1); Set.RebuildClustersFromGraph(); auto Events = Set.TakeEvents();
		const auto* Bond = Find(Events,DestructionEventKind::BondBroken); EXPECT(Bond && Bond->Cause == DestructionEventCause::GraphEdit);
		auto* Cluster = Set.FindClusterContaining(1); auto* Body = Cluster->GetRigidBody(); Body->X += Vector3(3,4,5);
		Transform Pose; EXPECT(Cluster->GetWorldTransform(1,Pose));
		Set.AddDestructionInfo(DestructionEventType::Destruct,{1}); Set.ProcessDestructionInfo(); Events = Set.TakeEvents();
		const auto* Released = Find(Events,DestructionEventKind::ChunkReleased); const auto* Smashed = Find(Events,DestructionEventKind::ChunkSmashed);
		EXPECT(Released && Smashed);
		if (Released)
		{
			EXPECT(Released->Cause == DestructionEventCause::LegacyCommand && Released->Chunks.size() == 1);
			EXPECT_NEAR((Released->Position-Pose.pos).Length(),0,1e-5);
			EXPECT(!Released->Chunks[0].IsFree && Released->Chunks[0].HasRigidBody);
		}
		Set.Update(1.0f/60); EXPECT(Set.TakeEvents().empty());
		DestructionSet Unsupported; Author(Unsupported); Enable(Unsupported); EXPECT(Unsupported.PruneUnsupported().size() == 3);
		Events = Unsupported.TakeEvents(); Released = Find(Events,DestructionEventKind::ChunkReleased);
		EXPECT(Released && Released->Cause == DestructionEventCause::Unsupported);
	}
	void StressAndPartitionCauses()
	{
		DestructionSet Set; Author(Set,2,true,0.01f); StrainSolverSettings Stress; Stress.Hardness = 1; Set.EnableStressSolver(Stress); Enable(Set);
		EXPECT(Set.AddStressImpulse(1,Set.GetChunks()[1].Centroid,Vector3(100,0,0))); Set.Update(1.0f/60);
		auto Events = Set.TakeEvents(); const auto* Broken = Find(Events,DestructionEventKind::BondBroken);
		EXPECT(Broken && Broken->Cause == DestructionEventCause::Stress && Broken->UpdateIndex == 1);
		if (Broken) EXPECT_NEAR(Broken->DeltaTime,1.0f/60,0);
		DestructionSet Partition; Author(Partition,8); auto& C = Partition.GetDestructionConstants();
		for (auto& L : C.Levels) { L.BreakThreshold = 0; L.PartitionSize = 2; }
		Enable(Partition); EXPECT(Partition.ApplyMomentum(3,Vector3(100,0,0),Vector3(3,0,0),Vector3::UnitX()));
		Events = Partition.TakeEvents(); Broken = Find(Events,DestructionEventKind::BondBroken);
		EXPECT(Broken && Broken->Cause == DestructionEventCause::Momentum);
		bool Queued = false;
		for (int Frame = 0; Frame < 8; ++Frame)
		{
			Partition.Update(1.0f/60); for (const auto& E : Partition.TakeEvents()) Queued |= E.Cause == DestructionEventCause::Partition;
		}
		EXPECT(Queued);
	}
	void HierarchyNotifications()
	{
		DynamicMesh Mesh;
		for (const auto& P : {Vector3(-1,-1,-1),Vector3(1,-1,-1),Vector3(1,1,-1),Vector3(-1,1,-1),
			Vector3(-1,-1,1),Vector3(1,-1,1),Vector3(1,1,1),Vector3(-1,1,1)}) Mesh.AppendVertex(P);
		for (const auto& T : {Index3(0,2,1),Index3(0,3,2),Index3(4,5,6),Index3(4,6,7),Index3(0,4,7),Index3(0,7,3),
			Index3(1,2,6),Index3(1,6,5),Index3(0,1,5),Index3(0,5,4),Index3(3,7,6),Index3(3,6,2)}) Mesh.AppendTriangle(T.a,T.b,T.c);
		Mesh.BuildBounds(); DestructionSet Set; DestructionFractureSettings S; S.MaxChunkCount = 8; S.CutRetryCount = 8; S.MinChunkVolume = 0.001f;
		DestructionFractureLevel Level; Level.MinPieceCount = Level.MaxPieceCount = 2; S.Levels = {Level,Level};
		EXPECT(Set.BuildHierarchicalFracture(Mesh,S)); Enable(Set); const auto* B = Set.GetGraph().GetBondLists()[0].Bonds.data();
		EXPECT(!Set.GetGraph()[0].Bonds.empty()); if (Set.GetGraph()[0].Bonds.empty()) return;
		Set.DoFractures({{B->bond_index}}); auto Events = Set.TakeEvents();
		EXPECT(Find(Events,DestructionEventKind::HierarchyBroken));
		std::set<int> Hierarchy; std::set<std::pair<int,int>> Bonds;
		for (const auto& E : Events)
		{
			if (E.Kind == DestructionEventKind::HierarchyBroken) EXPECT(Hierarchy.insert(E.HierarchyNode).second);
			if (E.Kind == DestructionEventKind::BondBroken) EXPECT(Bonds.insert({E.NodeA,E.NodeB}).second);
		}
		Set.RebuildClustersFromGraph(); EXPECT(Set.TakeEvents().empty());
	}
	void ContactPayloadAndFilters()
	{
		PhysicsWorld World{PhysicsWorldParam()}; DestructionSet Set(World); Author(Set); DestructionEventSettings S; S.Enabled = true;
		S.KindMask = 1u << (uint32_t)DestructionEventKind::Contact; S.MinimumContactImpulse = 1; Set.ConfigureEvents(S);
		auto* C = Set.FindClusterContaining(1); PhysicsContactPoint Point{}; Point.Position = Vector3(1,0.5f,0); Point.Normal = Vector3::UnitY(); Point.Impulse = Vector3(0,2,0);
		PhysicsContactInfo Info{}; Info.BodyB = C->GetRigidBody(); Info.GeomB = C->GetGeometryByChunkIndex(1); Info.ContactPoints = &Point; Info.NumContactPoints = 1;
		Set.OnContact(Info); Point.Position = Vector3(100); auto Events = Set.TakeEvents(); EXPECT(Events.size() == 1);
		if (!Events.empty())
		{
			const auto& E = Events[0]; EXPECT(E.Cause == DestructionEventCause::Contact && E.Chunks[0].ChunkIndex == 1);
			EXPECT_NEAR((E.Position-Vector3(1,0.5f,0)).Length(),0,0); EXPECT_NEAR((E.Impulse-Vector3(0,2,0)).Length(),0,0); EXPECT(!E.ImpulseEstimated);
		}
		Point.Impulse = Vector3(0,0.5f,0); Set.OnContact(Info); EXPECT(Set.TakeEvents().empty() && Set.GetEventStats().Filtered == 1);
		Point.Impulse = Vector3(0,2,0); Point.Position.x = std::numeric_limits<float>::quiet_NaN(); Set.OnContact(Info);
		EXPECT(Set.TakeEvents().empty() && Set.GetEventStats().Invalid == 1);
		Point.Position = Vector3(1,0.5f,0); Point.Impulse.SetZero(); C->GetRigidBody()->SetLinearVelocity(Vector3(0,-2,0));
		Set.OnContact(Info); Events = Set.TakeEvents(); EXPECT(Events.size() == 1 && Events[0].ImpulseEstimated);
		S.MinimumContactImpulse = 0; Set.ConfigureEvents(S); C->GetRigidBody()->SetLinearVelocity(Vector3::Zero());
		Set.OnContact(Info); Events = Set.TakeEvents(); EXPECT(Events.size() == 1 && Events[0].Impulse == Vector3::Zero());
	}
	void AddFloor(PhysicsWorld& World)
	{
		RigidBodyParam P; P.rigidType = RigidType::Static;
		World.CreateRigidBody(GeometryFactory::CreateOBB(Vector3(0,-1.5f,0),Vector3(8,0.5f,8)),P);
	}
	void RealContactsAndNoninterference()
	{
		DestructionSet Source; Author(Source); Source.GetDestructionConstants().PartitionAlgorithm = DestructionPartitionAlgorithm::None;
		Source.GetDestructionConstants().DamagePropagationStrength = 0; const auto Asset = DestructionAsset::Capture(Source);
		DestructionSet* Active = nullptr; bool CallbackRan = false, PollRejected = false, ConfigureRejected = false;
		PhysicsWorldParam P; P.gravityAcc = Vector3(0,-9.8f,0); P.narrowphase = NarrowPhaseSolver::PCM;
		PhysicsWorld Reference(P); P.onContact = [&](const PhysicsContactInfo& Info)
		{
			DefaultOnContact(Info); CallbackRan = true;
			try { Active->TakeEvents(); } catch (const std::logic_error&) { PollRejected = true; }
			try { Active->ConfigureEvents(Active->GetEventSettings()); } catch (const std::logic_error&) { ConfigureRejected = true; }
		};
		PhysicsWorld Observed(P); DestructionSet A(Reference), B(Observed); Active = &B;
		AddFloor(Reference); AddFloor(Observed); A.InitializeAsset(Asset); B.InitializeAsset(Asset); Enable(B);
		bool SawContact = false;
		for (int Frame = 0; Frame < 70; ++Frame)
		{
			Reference.Simulate(1.0f/60); Observed.Simulate(1.0f/60);
			for (const auto& E : B.TakeEvents())
			{
				if (E.Kind == DestructionEventKind::Contact) { SawContact = true; EXPECT(E.WorldTick == Observed.GetTick() && E.DeltaTime > 0); }
			}
			for (int I = 0; I < 3; ++I)
			{
				Transform PA, PB; EXPECT(A.FindClusterContaining(I)->GetWorldTransform(I,PA)); EXPECT(B.FindClusterContaining(I)->GetWorldTransform(I,PB));
				EXPECT_NEAR((PA.pos-PB.pos).Length(),0,1e-6);
			}
		}
		EXPECT(CallbackRan && PollRejected && ConfigureRejected && SawContact); EXPECT(A.TakeEvents().empty());
		EXPECT(B.GetEventStats().Dropped == 0 && B.GetEventStats().Invalid == 0);
	}
}
void TestDestructionEventStream()
{
	auto Run = [](const char* Name, void (*Test)())
	{
		printf("Destruction events: %s\n",Name);
		try { Test(); } catch (const std::exception& E) { printf("Destruction event test exception: %s\n",E.what()); EXPECT(false); }
	};
	Run("IdentityAndKinematics",IdentityAndKinematics); Run("CapacityAndFiltering",CapacityAndFiltering);
	Run("DirectedSupportBond",DirectedSupportBond);
	Run("ReleaseAndExternalGraphEdits",ReleaseAndExternalGraphEdits); Run("StressAndPartitionCauses",StressAndPartitionCauses);
	Run("HierarchyNotifications",HierarchyNotifications); Run("ContactPayloadAndFilters",ContactPayloadAndFilters);
	Run("RealContactsAndNoninterference",RealContactsAndNoninterference);
}
}

void TestDestructionEventStream() { Test_DestructionEvents_Cases::TestDestructionEventStream(); }

namespace Test_DestructionCache_Cases
{
using namespace Riemann;
namespace
{
	template<class F> void Reject(F Function)
	{ bool Caught = false; try { Function(); } catch (const std::exception&) { Caught = true; } EXPECT(Caught); }
	DestructionAsset Asset()
	{
		DestructionSet S; S.GetDestructionConstants().MinClusterVolume = 0;
		for (int I = 0; I < 3; ++I)
		{
			const Vector3 P((float)I,2,0); S.AddChunk(Box3(P-Vector3(0.5f),P+Vector3(0.5f)),(float)(I+1));
			if (I) EXPECT(S.AddBond(I-1,I,DestructionBondType::Connect,2,10+I));
		}
		for (size_t I = 0; I < S.GetGraph().Size(); ++I) for (auto& B : S.GetGraph()[I].Bonds)
			B.frame.Origin = (S.GetChunks()[B.v0].Centroid+S.GetChunks()[B.v1].Centroid)*0.5f;
		S.GetGraph().RefreshBondCache(); S.RebuildClustersFromGraph(); return DestructionAsset::Capture(S);
	}
	void Enable(DestructionSet& S, size_t Capacity = 2048)
	{ DestructionEventSettings E; E.Enabled = true; E.MaximumQueuedEvents = Capacity; S.ConfigureEvents(E); }
	Transform Placement()
	{
		Quaternion Q; Q.FromRotationAxis(Vector3(0.3f,0.8f,-0.4f).Unit(),1.07f); return Transform(Vector3(4,-2,3),Q);
	}
	void SamplingAndRelocation()
	{
		const auto A = Asset(); PhysicsWorld W{PhysicsWorldParam()}; DestructionSet S(W); const auto P = Placement(); S.InitializeAsset(A,P); Enable(S);
		DestructionCacheRecorder Recorder(S); auto* Cluster = S.FindClusterContaining(0); auto* Body = Cluster->GetRigidBody();
		Quaternion Delta; Delta.FromRotationAxis(Vector3(1,-1,2).Unit(),1.3f); Body->Q = Delta; Body->X += Vector3(2,3,-1);
		Body->V = Vector3(1,2,3); Body->W = Vector3(0.2f,-0.3f,0.5f); Recorder.Record(1,S);
		const auto Cache = Recorder.Finish(); const auto Copy = Cache;
		EXPECT(Cache.GetStats().FrameCount == 2 && Cache.GetStats().TopologyCount == 1 && Cache.GetStats().EventCount == 0);
		EXPECT(Cache.GetFrame(0).Topology == Cache.GetFrame(1).Topology); EXPECT(&Cache.GetFrame(0) == &Copy.GetFrame(0));
		EXPECT(Cache.GetSourceInstanceId() == S.GetInstanceId() && Cache.GetSourceGeneration() == S.GetGeneration());
		const auto End = Cache.Sample(1,P), Middle = Cache.Sample(0.5), Start = Cache.Sample(0);
		EXPECT(End.LowerFrame == 1 && End.Interpolation == 0 && Middle.LowerFrame == 0 && Middle.UpperFrame == 1);
		EXPECT_NEAR(Middle.Interpolation,0.5f,0);
		for (int I = 0; I < 3; ++I)
		{
			Transform Actual; EXPECT(Cluster->GetWorldTransform(I,Actual));
			EXPECT_NEAR((End.Chunks[I].Pose.pos-Actual.pos).Length(),0,2e-5);
			const Vector3 Velocity = Body->V+Body->W.Cross(Actual.pos-Body->X);
			EXPECT_NEAR((End.Chunks[I].LinearVelocity-Velocity).Length(),0,2e-5);
			EXPECT_NEAR((Middle.Chunks[I].Pose.pos-Middle.Chunks[0].Pose.pos).Length(),(Start.Chunks[I].Pose.pos-Start.Chunks[0].Pose.pos).Length(),2e-5);
		}
		Quaternion Q; Q.FromRotationAxis(Vector3(-1,0.4f,0.7f).Unit(),-0.9f); const Transform Destination(Vector3(-3,6,2),Q);
		DestructionSet Relocated; Relocated.InitializeAsset(A,Destination); const auto Sample = Cache.Sample(1,Destination);
		for (int I = 0; I < 3; ++I)
		{
			Transform Original; Cluster->GetWorldTransform(I,Original); const auto& Hull = *S.GetChunks()[I].CollisionConvex; const auto& Target = *Relocated.GetChunks()[I].CollisionConvex;
			for (size_t J = 0; J < Hull.NumVertices; ++J)
			{
				const Vector3 Expected = Destination.LocalToWorld(P.WorldToLocal(Original.LocalToWorld(Hull.Vertices[J])));
				EXPECT_NEAR((Sample.Chunks[I].Pose.LocalToWorld(Target.Vertices[J])-Expected).Length(),0,3e-5);
			}
		}
		EXPECT(Cache.Sample(-2).Time == 0 && Cache.Sample(2).Time == 1);
		S.Clear(); W.Reset(); EXPECT(Cache.Sample(1).Chunks.size() == 3 && Cache.GetAsset().GetStats().Fingerprint == A.GetStats().Fingerprint);
		Reject([&] { Recorder.Record(2,S); }); Reject([&] { Recorder.Finish(); }); Reject([&] { Cache.GetFrame(2); });
		Reject([&] { Cache.Sample(std::numeric_limits<double>::quiet_NaN()); });
		Transform Bad; Bad.quat.x = 2; Reject([&] { Cache.Sample(0,Bad); });
	}
	void FractureRemovalAndEventIntervals()
	{
		PhysicsWorld W{PhysicsWorldParam()}; DestructionSet S(W); S.InitializeAsset(Asset(),Placement()); Enable(S); DestructionCacheRecorder Recorder(S);
		S.DoFractures({{11}}); auto Events = S.TakeEvents(); EXPECT(Events.size() == 2);
		S.FindClusterContaining(0)->GetRigidBody()->X += Vector3(2,0,0); S.FindClusterContaining(1)->GetRigidBody()->X -= Vector3(1,0,0);
		Recorder.Record(1,S,Events); S.AddDestructionInfo(DestructionEventType::Destruct,{0}); S.ProcessDestructionInfo();
		auto Removed = S.TakeEvents(); EXPECT(!Removed.empty()); Recorder.Record(2,S,Removed); const auto Cache = Recorder.Finish();
		const auto Before = Cache.Sample(0.999), Broken = Cache.Sample(1), BeforeRemoval = Cache.Sample(1.999), AfterRemoval = Cache.Sample(2);
		EXPECT(Before.Topology->Clusters.size() == 1 && Before.Topology->Graph.HasBond(0,1));
		EXPECT(Broken.Topology->Clusters.size() == 2 && !Broken.Topology->Graph.HasBond(0,1));
		EXPECT_NEAR((Before.Chunks[2].Pose.pos-Before.Chunks[0].Pose.pos).Length(),2,2e-5);
		EXPECT(!BeforeRemoval.Chunks[0].IsFree && AfterRemoval.Chunks[0].IsFree && AfterRemoval.Chunks[0].IsSmashed);
		EXPECT_NEAR((BeforeRemoval.Chunks[0].Pose.pos-Broken.Chunks[0].Pose.pos).Length(),0,1e-6);
		EXPECT(!AfterRemoval.Chunks[0].HasRigidBody && AfterRemoval.Chunks[0].ClusterId == 0);
		EXPECT(Cache.GetStats().TopologyCount == 3 && Cache.GetStats().EventCount == Events.size()+Removed.size());
		EXPECT(Cache.GetEvents(0,0.999).empty()); EXPECT(Cache.GetEvents(0.999,1).size() == Events.size());
		EXPECT(Cache.GetEvents(1,2).size() == Removed.size()); EXPECT(Cache.GetEvents(1,1).empty());
		const auto WorldEvents = Cache.GetEvents(0,1,S.GetAssetPlacement()); EXPECT(WorldEvents.size() == Events.size());
		for (size_t I = 0; I < Events.size(); ++I)
		{
			EXPECT(WorldEvents[I].Time == 1 && WorldEvents[I].Event.Sequence == Events[I].Sequence && WorldEvents[I].Event.InstanceId == S.GetInstanceId());
			EXPECT_NEAR((WorldEvents[I].Event.Position-Events[I].Position).Length(),0,2e-5);
			for (size_t J = 0; J < Events[I].Chunks.size(); ++J) EXPECT_NEAR((WorldEvents[I].Event.Chunks[J].Pose.pos-Events[I].Chunks[J].Pose.pos).Length(),0,2e-5);
		}
		Reject([&] { Cache.GetEvents(2,1); });
	}
	void MissingDuplicateAndReconfiguredEvents()
	{
		PhysicsWorld W{PhysicsWorldParam()}; DestructionSet S(W); S.InitializeAsset(Asset()); Enable(S); DestructionCacheRecorder R(S);
		S.DoFractures({{11}}); Reject([&] { R.Record(1,S); }); auto Events = S.TakeEvents();
		Reject([&] { R.Record(1,S); }); auto Bad = Events; Bad[1] = Bad[0]; Reject([&] { R.Record(1,S,Bad); });
		Bad = Events; ++Bad[0].InstanceId; Reject([&] { R.Record(1,S,Bad); });
		Bad = Events; Bad[0].Position.x = std::numeric_limits<float>::quiet_NaN(); Reject([&] { R.Record(1,S,Bad); });
		EXPECT(R.GetStats().FrameCount == 1); R.Record(1,S,Events); EXPECT(R.GetStats().FrameCount == 2);
		Reject([&] { R.Record(2,S,Events); }); const auto Revision = S.GetEventRevision(); S.ConfigureEvents(S.GetEventSettings());
		EXPECT(S.GetEventRevision() > Revision); Reject([&] { R.Record(2,S); }); EXPECT(R.GetStats().FrameCount == 2);
		DestructionSet Overflow(W); Overflow.InitializeAsset(Asset()); Enable(Overflow,1); DestructionCacheRecorder Lost(Overflow);
		Overflow.DoFractures({{11}}); auto Batch = Overflow.TakeEvents(); EXPECT(Overflow.GetEventStats().Dropped > 0);
		Reject([&] { Lost.Record(1,Overflow,Batch); }); EXPECT(Lost.GetStats().FrameCount == 1);
	}
	void LimitsAndAuthoringValidation()
	{
		PhysicsWorld W{PhysicsWorldParam()}; DestructionSet S(W); const auto A = Asset(); S.InitializeAsset(A);
		Reject([&] { DestructionCacheRecorder MissingStream(S); }); DestructionCacheSettings C; C.RecordEvents = false;
		S.GetDestructionConstants().ContactOffset = -1; EXPECT(!S.GetDestructionConstants().IsValid());
		Reject([&] { DestructionCacheRecorder BadPolicy(S,C); }); S.GetDestructionConstants().ContactOffset = 0;
		DestructionCacheRecorder Measure(S,C); const auto InitialBytes = Measure.GetStats().PayloadBytes;
		C.MaximumPayloadBytes = InitialBytes; DestructionCacheRecorder Budget(S,C); Reject([&] { Budget.Record(1,S); });
		EXPECT(Budget.GetStats().PayloadBytes == InitialBytes && Budget.GetStats().FrameCount == 1);
		C.MaximumPayloadBytes = 1; Reject([&] { DestructionCacheRecorder TooSmall(S,C); });
		C = DestructionCacheSettings(); C.RecordEvents = false; C.MaximumFrames = 2; DestructionCacheRecorder Frames(S,C);
		Frames.Record(1,S); Reject([&] { Frames.Record(2,S); }); EXPECT(Frames.GetStats().FrameCount == 2);
		Reject([&] { Measure.Record(0,S); }); Reject([&] { Measure.Record(-1,S); });
		auto& Chunk = S.GetChunks()[0]; const Vector3 V = Chunk.CollisionConvex->Vertices[0]; Chunk.CollisionConvex->Vertices[0].x += 0.1f;
		Reject([&] { Measure.Record(1,S); }); Chunk.CollisionConvex->Vertices[0] = V;
		const float D = Chunk.CollisionConvex->Faces[0].plane.D; Chunk.CollisionConvex->Faces[0].plane.D = std::numeric_limits<float>::quiet_NaN();
		Reject([&] { Measure.Record(1,S); }); Chunk.CollisionConvex->Faces[0].plane.D = D;
		Chunk.Mass *= 2; Reject([&] { Measure.Record(1,S); }); Chunk.Mass /= 2;
		auto* Bond = S.GetGraph().FindBond(0,1); const auto Origin = Bond->frame.Origin; Bond->frame.Origin.x += 0.1f;
		Reject([&] { Measure.Record(1,S); }); Bond->frame.Origin = Origin;
		const auto Constants = S.GetDestructionConstants(); S.GetDestructionConstants().ContactOffset += 0.1f;
		Reject([&] { Measure.Record(1,S); }); S.GetDestructionConstants() = Constants;
		auto* Body = S.FindClusterContaining(0)->GetRigidBody(); const auto Q = Body->Q; Body->Q = Quaternion(0,0,0,0);
		Reject([&] { Measure.Record(1,S); }); Body->Q = Q;
		Measure.Record(1,S); EXPECT(Measure.GetStats().FrameCount == 2);
		DestructionSet Other(W); Other.InitializeAsset(A); Reject([&] { Measure.Record(2,Other); });
		W.Reset(); Reject([&] { Measure.Record(2,S); }); const auto Cache = Measure.Finish(); EXPECT(Cache.GetStats().FrameCount == 2);
		DestructionSet Detached; Detached.InitializeAsset(A); Reject([&] { DestructionCacheRecorder NoWorld(Detached,C); });
		DestructionCache Empty; Reject([&] { Empty.Sample(0); }); Reject([&] { Empty.GetStats(); });
	}
	void WorldBoundaryAndRetiredSource()
	{
		DestructionSet* Active = nullptr; DestructionCacheRecorder* Recording = nullptr; bool Rejected = false;
		PhysicsWorldParam P; P.gravityAcc = Vector3(0,-9.8f,0); P.onContact = [&](const PhysicsContactInfo& Info)
		{
			DefaultOnContact(Info); try { Recording->Record(1,*Active); } catch (const std::logic_error&) { Rejected = true; }
		};
		PhysicsWorld W(P); RigidBodyParam Floor; Floor.rigidType = RigidType::Static;
		W.CreateRigidBody(GeometryFactory::CreateOBB(Vector3(0,-0.5f,0),Vector3(8,0.5f,8)),Floor);
		DestructionSet S(W); S.InitializeAsset(Asset()); Active = &S; DestructionCacheSettings C; C.RecordEvents = false;
		DestructionCacheRecorder R(S,C); Recording = &R;
		for (int I = 0; I < 60; ++I) { W.Simulate(1.0f/60); R.Record((I+1)/60.0,S); }
		EXPECT(Rejected); const auto Cache = R.Finish(); S.Clear(); EXPECT(Cache.GetStats().FrameCount == 61 && Cache.Sample(1).Chunks.size() == 3);
	}
}
void TestDestructionCache()
{
	auto Run = [](const char* Name, void (*Test)())
	{
		printf("Destruction cache: %s\n",Name);
		try { Test(); } catch (const std::exception& E) { printf("Destruction cache exception: %s\n",E.what()); EXPECT(false); }
	};
	Run("SamplingAndRelocation",SamplingAndRelocation); Run("FractureRemovalAndEventIntervals",FractureRemovalAndEventIntervals);
	Run("MissingDuplicateAndReconfiguredEvents",MissingDuplicateAndReconfiguredEvents); Run("LimitsAndAuthoringValidation",LimitsAndAuthoringValidation);
	Run("WorldBoundaryAndRetiredSource",WorldBoundaryAndRetiredSource);
}
}

void TestDestructionCache() { Test_DestructionCache_Cases::TestDestructionCache(); }

namespace Test_DestructionPlayback_Cases
{
using namespace Riemann;
namespace
{
	template<class F> void Reject(F Function)
	{ bool Caught = false; try { Function(); } catch (const std::exception&) { Caught = true; } EXPECT(Caught); }
	Transform Placement()
	{
		Quaternion Q; Q.FromRotationAxis(Vector3(1,2,-1).Unit(),0.8f); return Transform(Vector3(6,3,-4),Q);
	}
	DestructionCache Clip()
	{
		DestructionSet Author; Author.GetDestructionConstants().MinClusterVolume = 0;
		for (int I = 0; I < 3; ++I)
		{
			const Vector3 P((float)I,2,0); Author.AddChunk(Box3(P-Vector3(0.5f),P+Vector3(0.5f)),(float)(I+1));
			if (I) EXPECT(Author.AddBond(I-1,I,DestructionBondType::Connect,2,10+I));
		}
		Author.RebuildClustersFromGraph(); const auto Asset = DestructionAsset::Capture(Author);
		PhysicsWorld W{PhysicsWorldParam()}; DestructionSet S(W); S.InitializeAsset(Asset,Placement());
		DestructionEventSettings E; E.Enabled = true; S.ConfigureEvents(E); DestructionCacheRecorder R(S);
		auto* Body = S.FindClusterContaining(0)->GetRigidBody(); Body->X += Vector3(1,2,-1);
		Body->Q.FromRotationAxis(Vector3(1,-1,2).Unit(),0.7f); Body->V = Vector3(1,0,0); Body->W = Vector3(0,0.2f,0);
		R.Record(1,S); S.DoFractures({{11}}); R.Record(2,S,S.TakeEvents());
		S.AddDestructionInfo(DestructionEventType::Destruct,{0}); S.ProcessDestructionInfo(); R.Record(3,S,S.TakeEvents());
		return R.Finish();
	}
	void RegisteredKinematics()
	{
		for (auto Solver : {ResolutionPhaseSolver::SequentialImpulse,ResolutionPhaseSolver::LCPGlobal,ResolutionPhaseSolver::PBD,ResolutionPhaseSolver::XPBD})
		{
			PhysicsWorldParam P; P.resolutionSolver = Solver; PhysicsWorld W(P);
			RigidBodyParam K; K.rigidType = RigidType::Kinematic; K.linearVelocity = Vector3(1,0,0);
			auto* A = W.CreateRigidBody(GeometryFactory::CreateOBB(Vector3::Zero(),Vector3(0.5f)),K);
			RigidBodyParam D; D.rigidType = RigidType::Dynamic; D.linearVelocity = Vector3(-1,0,0);
			auto* B = W.CreateRigidBody(GeometryFactory::CreateOBB(Vector3(0.9f,0,0),Vector3(0.5f)),D);
			EXPECT(A->CastKinematic() && W.GetKinematicBodyCount() == 1 && A->InvMass == 0 && A->CM == Vector3::Zero());
			W.Simulate(1.0f/60); EXPECT_NEAR(A->X.Length(),0,1e-6); EXPECT_NEAR(A->V.x,1,1e-6); EXPECT(B->V.x > -0.5f);
			std::vector<Geometry*> Shapes; W.GetAllGeometries(&Shapes); EXPECT(Shapes.size() == 2);
			EXPECT(W.RemoveRigidBody(A)); EXPECT(!W.RemoveRigidBody(A)); A->ReleaseGeometries(); delete A;
			EXPECT(W.GetKinematicBodyCount() == 0); W.Reset();
		}
		PhysicsWorld W{PhysicsWorldParam()}; RigidBodyParam Bad; Bad.rigidType = (RigidType)255;
		Reject([&] { W.CreateRigidBody(Bad,Transform::Identity()); }); Reject([&] { W.CreateRigidBody(nullptr,Bad); });
	}
	void SamplesQueriesAndTopology()
	{
		const auto Cache = Clip(); PhysicsWorld W{PhysicsWorldParam()}; const auto P = Placement(); DestructionPlayback Player(W,Cache,P);
		EXPECT(Player.IsAttached() && !Player.IsPlaying() && Player.GetBodyCount() == 1 && W.GetKinematicBodyCount() == 1);
		Geometry* Initial = Player.GetChunkGeometry(0); const auto Revision = Player.GetRevision();
		for (double Time : {0.5,1.0,1.99,2.0,2.5,3.0,0.0})
		{
			Player.Seek(Time); const auto Sample = Cache.Sample(Time,P);
			EXPECT(Player.GetBodyCount() == Sample.Topology->Clusters.size());
			for (const auto& C : Sample.Chunks)
			{
				auto* Shape = Player.GetChunkGeometry(C.ChunkIndex);
				EXPECT((Shape != nullptr) == C.HasRigidBody);
				if (!Shape) continue;
				auto* Body = Shape->GetParent<RigidBody>(); const auto Pose = Body->GetGeometryTransform(Shape);
				EXPECT(Body->InvMass == 0 && Body->CM == Vector3::Zero() && Body->V == Vector3::Zero());
				EXPECT_NEAR((Pose.pos-C.Pose.pos).Length(),0,3e-5);
				const auto* Hull = Shape->GetShapeObj<ConvexMesh>(); const auto Bounds = Body->GetGeometryBounds(Shape);
				for (size_t I = 0; I < Hull->NumVertices; ++I)
					EXPECT_NEAR((Pose.LocalToWorld(Hull->Vertices[I])-C.Pose.LocalToWorld(Hull->Vertices[I])).Length(),0,3e-5);
				EXPECT((Bounds.GetCenter()-C.Pose.pos).Length() < 1e-4f);
			}
			if (Time < 2 && Player.GetRevision() == Revision) EXPECT(Player.GetChunkGeometry(0) == Initial);
			if (Time < 3)
			{
				RayCastOption O; O.MaxDist = 20; RayCastResult R;
				const Vector3 Center = Sample.Chunks[0].Pose.pos;
				EXPECT(W.GetGeometryQuery()->RayCastQuery(Center+Vector3(0,0,10),Vector3(0,0,-1),O,&R));
			}
		}
		EXPECT(Player.GetRevision() > Revision); EXPECT(Player.GetChunkGeometry(-1) == nullptr && Player.GetChunkGeometry(99) == nullptr);
		Player.Detach(); EXPECT(!Player.IsAttached() && W.GetKinematicBodyCount() == 0); Player.Detach();
	}
	void TimeEventsAndEpochs()
	{
		const auto Cache = Clip(); PhysicsWorldParam P; P.temporalSubsteps.Enabled = true; P.temporalSubsteps.Substeps = 4;
		PhysicsWorld W(P); DestructionPlayback Player(W,Cache); const auto Epoch = Player.GetEpoch();
		W.Simulate(0.5f); EXPECT(Player.GetTime() == 0); Player.SetPlaying(true); W.Simulate(0.5f);
		EXPECT_NEAR(Player.GetTime(),0.5,1e-8); EXPECT(Player.TakeEvents().empty());
		const auto Sample = Cache.Sample(0.5); EXPECT_NEAR((Player.GetBody(0)->V-Sample.Clusters[0].LinearVelocity).Length(),0,1e-6);
		Player.SetPlaying(false); W.Simulate(1); EXPECT(Player.GetTime() == 0.5 && Player.GetBody(0)->V == Vector3::Zero());
		Player.SetPlaying(true); W.Simulate(1.5f); const auto Events = Player.TakeEvents(); const auto Source = Cache.GetEvents(0,2);
		EXPECT(Events.size() == Source.size() && !Events.empty());
		for (size_t I = 0; I < Events.size(); ++I)
		{
			EXPECT(Events[I].PlaybackId == Player.GetInstanceId() && Events[I].Epoch == Epoch);
			EXPECT(Events[I].Source.Event.InstanceId == Cache.GetSourceInstanceId());
			EXPECT(Events[I].Source.Event.Sequence == Source[I].Event.Sequence && Events[I].Source.Time == Source[I].Time);
		}
		W.Simulate(0.5f); EXPECT(Player.TakeEvents().empty()); Player.Seek(0); EXPECT(Player.GetEpoch() == Epoch+1 && !Player.IsPlaying());
		Player.SetPlaying(true); W.Simulate(2); EXPECT(Player.TakeEvents().size() == Events.size());
		W.Simulate(2); EXPECT(Player.GetTime() == 3 && !Player.IsPlaying() && Player.GetBody(0)->V == Vector3::Zero());
		EXPECT(!Player.TakeEvents().empty()); EXPECT(Player.TakeEvents().empty());
		DestructionPlaybackSettings S; S.Rate = 2; S.MaximumQueuedEvents = 1; DestructionPlayback Fast(W,Cache,Transform::Identity(),S);
		Fast.SetPlaying(true); W.Simulate(1); EXPECT(Fast.GetTime() == 2 && Fast.TakeEvents().size() == 1);
		EXPECT(Fast.GetDroppedEventCount() == Source.size()-1);
	}
	void DistantConvexRays()
	{
		PhysicsWorld W{PhysicsWorldParam()}; DestructionPlayback Player(W,Clip());
		const auto* Shape = Player.GetChunkGeometry(0); const auto* Hull = Shape->GetShapeObj<ConvexMesh>();
		const Vector3 Center = Shape->GetShapeBounds().GetCenter(); float Near = -1, Scaled = -1, Inside = -1;
		EXPECT(Hull->IntersectRay(Center+Vector3(0,0,10),Vector3(0,0,-1),&Near)); EXPECT(Near > 1);
		EXPECT(Hull->IntersectRay(Center+Vector3(0,0,10),Vector3(0,0,-0.5f),&Scaled)); EXPECT_NEAR(Scaled,Near*2,1e-5);
		EXPECT(Hull->IntersectRay(Center,Vector3::UnitX(),&Inside)); EXPECT(Inside == 0);
		EXPECT(!Hull->IntersectRay(Center,Vector3::Zero(),&Inside));
		EXPECT(!Hull->IntersectRay(Shape->GetShapeBounds().Max+Vector3(1),Vector3::UnitY(),&Inside));
		RayCastOption O; O.MaxDist = 1; RayCastResult R; const auto WorldCenter = Player.GetSample().Chunks[0].Pose.pos;
		EXPECT(!W.GetGeometryQuery()->RayCastQuery(WorldCenter+Vector3(0,0,10),Vector3(0,0,-1),O,&R));
	}
	void CoupledWorldAndCallbackBoundary()
	{
		for (bool Coupled : {false,true})
		{
			DestructionPlayback* Active = nullptr; PhysicsWorld* World = nullptr; int Rejected = 0;
			PhysicsWorldParam P; P.resolutionSolver = ResolutionPhaseSolver::XPBD;
			P.temporalSubsteps.Enabled = true; P.temporalSubsteps.Substeps = 4;
			P.onContact = [&](const PhysicsContactInfo& Info)
			{
				DefaultOnContact(Info);
				try { Active->Seek(0); } catch (const std::logic_error&) { ++Rejected; }
				try { Active->SetPlaying(false); } catch (const std::logic_error&) { ++Rejected; }
				try { Active->TakeEvents(); } catch (const std::logic_error&) { ++Rejected; }
				try { Active->Detach(); } catch (const std::logic_error&) { ++Rejected; }
				try { World->RemoveRigidBody(Active->GetBody(0)); } catch (const std::logic_error&) { ++Rejected; }
			};
			PhysicsWorld W(P); World = &W; DestructionPlayback Player(W,Clip()); Active = &Player;
			TetrahedralAssetInput Input; Input.Positions = {Vector3(50,0,0),Vector3(51,0,0),Vector3(50,1,0),Vector3(50,0,1)};
			Input.Elements.push_back({{0,1,2,3},0}); DeformableBodySettings Settings;
			Settings.Collision.RigidCollision = true; Settings.Gravity = Vector3::Zero(); Settings.GroundEnabled = false;
			if (Coupled) W.CreateDeformableBody(TetrahedralAsset::Prepare(Input),Settings);
			RigidBodyParam Floor; W.CreateRigidBody(GeometryFactory::CreateOBB(Vector3(0,-0.5f,0),Vector3(6,0.5f,6)),Floor);
			RigidBodyParam Dynamic; Dynamic.rigidType = RigidType::Dynamic;
			W.CreateRigidBody(GeometryFactory::CreateSphere(Vector3(0,0.4f,0),0.5f),Dynamic);
			Player.SetPlaying(true); W.Simulate(2);
			EXPECT(Rejected >= 5 && Player.IsAttached()); EXPECT(Player.GetTime() == 2 && Player.GetBodyCount() == 2);
			EXPECT(!Player.TakeEvents().empty()); W.Reset(); EXPECT(!Player.IsAttached());
		}
	}

	void LiveCCDAndPlaybackPolicy()
	{
		for (bool Continue : {false,true})
		{
			PhysicsWorldParam P; P.ccdContinuation.Enabled = Continue; PhysicsWorld W(P); DestructionPlayback Player(W,Clip());
			RigidBodyParam D; D.rigidType = RigidType::Dynamic; D.motionType = MorionType::Continuous; D.linearVelocity = Vector3(60,0,0);
			auto* Bullet = W.CreateRigidBody(GeometryFactory::CreateSphere(Vector3(-3,2,0),0.25f),D);
			W.Simulate(0.1f); EXPECT(Bullet->X.x < -0.6f); EXPECT(W.GetLastCCDHitCount() > 0); EXPECT(Player.GetTime() == 0);
		}
		PhysicsWorld W{PhysicsWorldParam()}; DestructionPlaybackSettings S; S.QueryEnabled = false; S.CollisionEnabled = false;
		DestructionPlayback Hidden(W,Clip(),Transform::Identity(),S); RayCastOption O; RayCastResult R;
		EXPECT(!W.GetGeometryQuery()->RayCastQuery(Vector3(0,2,10),Vector3(0,0,-1),O,&R));
		Hidden.SetPlaying(true); W.Simulate(2); EXPECT(Hidden.GetTime() == 2 && Hidden.GetBodyCount() == 2);
		EXPECT(!W.GetGeometryQuery()->RayCastQuery(Hidden.GetSample().Chunks[0].Pose.pos+Vector3(0,0,10),Vector3(0,0,-1),O,&R));
	}
	void WorldLifetimeAndEdits()
	{
		const auto Cache = Clip(); auto W = std::make_unique<PhysicsWorld>(PhysicsWorldParam()); auto Player = std::make_unique<DestructionPlayback>(*W,Cache);
		auto* Body = Player->GetBody(0); EXPECT(W->RemoveRigidBody(Body)); Body->ReleaseGeometries(); delete Body;
		EXPECT(!Player->IsPlaying() && Player->GetChunkGeometry(0) == nullptr && Player->GetBody(0) == nullptr);
		Player->Seek(1); EXPECT(Player->GetChunkGeometry(0) != nullptr && W->GetKinematicBodyCount() == 1);
		W->Reset(); EXPECT(!Player->IsAttached() && Player->GetBodyCount() == 0 && Player->GetChunkGeometry(0) == nullptr);
		Reject([&] { Player->Seek(0); }); Reject([&] { Player->SetPlaying(true); });
		Player.reset(new DestructionPlayback(*W,Cache)); W.reset(); EXPECT(!Player->IsAttached()); Player.reset();
		PhysicsWorld World{PhysicsWorldParam()}; DestructionPlayback Live(World,Cache); const auto Time = Live.GetTime();
		Reject([&] { Live.Seek(std::numeric_limits<double>::quiet_NaN()); }); EXPECT(Live.GetTime() == Time);
		DestructionPlaybackSettings Bad; Bad.Rate = 0; Reject([&] { DestructionPlayback Invalid(World,Cache,Transform::Identity(),Bad); });
		DestructionCache Empty; Reject([&] { DestructionPlayback Invalid(World,Empty); });
		EXPECT(World.GetKinematicBodyCount() == 1);
	}
}
void TestDestructionPlayback()
{
	auto Run = [](const char* Name, void (*Test)())
	{
		printf("Destruction playback: %s\n",Name);
		try { Test(); } catch (const std::exception& E) { printf("Destruction playback exception: %s\n",E.what()); EXPECT(false); }
	};
	Run("RegisteredKinematics",RegisteredKinematics); Run("SamplesQueriesAndTopology",SamplesQueriesAndTopology);
	Run("TimeEventsAndEpochs",TimeEventsAndEpochs); Run("WorldLifetimeAndEdits",WorldLifetimeAndEdits);
	Run("DistantConvexRays",DistantConvexRays); Run("CoupledWorldAndCallbackBoundary",CoupledWorldAndCallbackBoundary);
	Run("LiveCCDAndPlaybackPolicy",LiveCCDAndPlaybackPolicy);
}
}

void TestDestructionPlayback() { Test_DestructionPlayback_Cases::TestDestructionPlayback(); }
