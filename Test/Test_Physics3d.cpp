#include <chrono>

#include "Test.h"
#include "../Src/Maths/Maths.h"
#include "../Src/RigidBodyDynamics/RigidBody.h"
#include "../Src/Collision/AABBTree.h"
#include "../Src/Collision/GeometryQuery.h"
#include "../Src/Tools/PhysxBinaryParser.h"

uint64_t GetGuid(Geometry* g)
{
	return g->GetParent<RigidBody>()->GetGuid();
}

void TestPhysxBin()
{
	printf("Running TestPhysxBin Japan\n");
	std::vector<RigidBody*> collection;
	std::vector<Geometry*> geometries;
	bool load_succ = LoadPhysxBinary("data/Japan.xml.bin", nullptr, &geometries);
	EXPECT(load_succ);
	if (load_succ)
	{
		GeometryQuery query;
		query.BuildStaticGeometry(geometries, 1);
		RayCastOption Option;
		RayCastResult Result;

		query.RayCastTest(Vector3(-521.23f, 55.87f, 399.15f), Vector3(0, -1, 0).Unit(), Option, &Result);
		EXPECT(FuzzyEqual(Result.hitPoint.y, 55.53f, 0.1f));
	}

	printf("Running TestPhysxBin fighting_new\n");

	collection.clear();
	geometries.clear();
	load_succ = LoadPhysxBinary("data/fighting_new.xml.bin", &collection, &geometries);
	EXPECT(load_succ);

	if (load_succ)
	{
		GeometryQuery query;
		query.BuildStaticGeometry(geometries, 1);

		TreeStatistics stat;
		query.GetStaticTree()->Statistic(stat);

		RayCastOption Option;
		RayCastResult Result;

		IntersectOption OOption;
		IntersectResult OResult;

		bool ret = query.RayCastTest(Vector3(-569, 50, 427), Vector3(1, -1, 1).Unit(), Option, &Result);
		EXPECT(ret);
		EXPECT(GetGuid(Result.hitGeom) == 2759574283328);

		Vector3 Pos = Result.hitPoint;

		ret = query.RayCastTest(Vector3(-569, -50, 427), Vector3(1, 0, 1).Unit(), Option, &Result);

		ret = query.IntersectTest_Box(Vector3(Pos.x, Pos.y + 15.0f, Pos.z), Vector3::One(), OOption, &OResult);
		EXPECT(!ret);

		OOption.maxOverlaps = 1;
		ret = query.IntersectTest_Box(Vector3(Pos.x, Pos.y, Pos.z), 1.0f * Vector3::One(), OOption, &OResult);
		EXPECT(ret);
		EXPECT(GetGuid(OResult.overlapGeoms[0]) == 2759574283328);

		ret = query.RayCastTest(Vector3(Pos.x + 0.01f, Pos.y - 10.0f, Pos.z + 0.01f), -Vector3::UnitY(), Option, &Result);
		// EXPECT(!ret);		// TODO filter the world box

		ret = query.RayCastTest(Vector3(Pos.x + 0.01f, Pos.y, Pos.z + 0.01f), -Vector3::UnitY(), Option, &Result);
		EXPECT(ret);
		EXPECT(FloatDiff(Result.hitPoint.y, Pos.y) < 0.2f);
		EXPECT(GetGuid(Result.hitGeom) == 2759584952560);

		ret = query.RayCastTest(Vector3(-2222, 0, -773), -Vector3::UnitY(), Option, &Result);
		EXPECT(ret);
		EXPECT(GetGuid(Result.hitGeom) == 2309460023584);

		ret = query.RayCastTest(Vector3(-569, 0, 427), -Vector3::UnitY(), Option, &Result);
		EXPECT(ret);
		EXPECT(GetGuid(Result.hitGeom) == 2309201640896);
	}

	return;
}

void TestRaycastBenchmark()
{
	printf("Running TestRaycastBenchmark\n");
	std::vector<RigidBody*> collection;
	std::vector<Geometry*> geometries;
	bool load_succ = LoadPhysxBinary("data/fighting_new.xml.bin", &collection, &geometries);
	EXPECT(load_succ);
	
	if (load_succ)
	{
		GeometryQuery query;
		query.BuildStaticGeometry(geometries, 5);

		TreeStatistics stat;
		query.GetStaticTree()->Statistic(stat);

		RayCastOption Option;
		RayCastResult Result;

		bool ret;
		int rays = 10000;
		auto t1 = std::chrono::steady_clock::now();
		for (int i = 0; i < rays; ++i)
		{
			ret = query.RayCastTest(Vector3(-569, 0, 427), Vector3(1, -1, 1).Unit(), Option, &Result);
			EXPECT(ret);
			EXPECT(GetGuid(Result.hitGeom) == 2309201640896);

			if (i == 0)
			{
			//    Option.Cache.prevhitGeom = Result.hitGeom;
			//    Option.Cache.prevStack.Restore(Result.stack);
			}
		}
		auto t2 = std::chrono::steady_clock::now();
		auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
		printf("cost Raycast = %lld, rays = %d\n", diff, 10000);
	}

	return;
}



void TestPhysics3d()
{
	TestPhysxBin();
	TestRaycastBenchmark();
}

