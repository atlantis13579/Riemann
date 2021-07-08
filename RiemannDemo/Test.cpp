
#include <assert.h>
#include <vector>

#include "../Src/LinearSystem/JacobiIteration_CPU.h"
#include "../Src/LinearSystem/GaussSeidelIteration_CPU.h"
#include "../Src/LinearSystem/LUFactorization.h"
#include "../Src/Maths/BoundingBox3d.h"
#include "../Src/Maths/Maths.h"
#include "../Src/Maths/Transform.h"
#include "../Src/CollisionPrimitive/AxisAlignedBox.h"
#include "../Src/CollisionPrimitive/Plane.h"
#include "../Src/CollisionPrimitive/Sphere.h"
#include "../Src/CollisionPrimitive/Ray.h"
#include "../Src/CollisionPrimitive/Triangle.h"
#include "../Src/CollisionPrimitive/Cylinder.h"
#include "../Src/CollisionPrimitive/Capsule.h"
#include "../Src/Collision/AABBTree.h"
#include "../Src/Collision/GeometryQuery.h"
#include "../Src/Collision/SAP.h"
#include "../Src/Collision/SAP_Incremental.h"

void TestAABBTree()
{
    std::vector<BoundingBox3d> boxes;
    boxes.emplace_back(Vector3d(0, 0, 0), Vector3d(1, 1, 1));
    boxes.emplace_back(Vector3d(0, 0, 0), Vector3d(1, 1, 3));
    boxes.emplace_back(Vector3d(1, 1, 1), Vector3d(1, 1, 2));

    AABBTree tree;

    AABBTreeBuildData param(&boxes[0], (int)boxes.size(), 1);

    tree.Release();
    tree.StaticBuild(param);

    int p = tree.Traverse(Vector3d(-1.0f, -1.0f, -1.0f));
    assert(p == -1);
    p = tree.Traverse(Vector3d(0.5f, 0.5f, 0.5f));
    assert(p >= 0);
    p = tree.Traverse(Vector3d(0.5f, 0.5f, 2.5f));
    assert(p == 1);

    float t;
    Ray ray(Vector3d(0.5f, 0.5f, 100.0f), Vector3d(0.0f, 0.0f, -1.0f));
    int hit = tree.RayCastBoundingBox(ray, &t);
    assert(hit >= 0);
    Vector3d hit_pos = ray.PointAt(t);
    
    ray.Origin = Vector3d(0.5f, 0.5f, 0.5f);
    hit = tree.RayCastBoundingBox(ray, &t);
    assert(hit >= 0);

    hit_pos = ray.PointAt(t);

    for (int i = 0; i < 10000; ++i)
    {
        Vector3d point1 = Vector3d::Random() * 100.0f;
        Vector3d point2 = Vector3d::Random() * 100.0f;
        boxes.emplace_back(point1, point1 + point2);
    }

    AABBTreeBuildData param2(&boxes[0], (int)boxes.size(), 1);

    tree.Release();
    tree.StaticBuild(param2);

    for (int i = 0; i < 10000; ++i)
    {
        Vector3d point = Vector3d::Random() * 100.0f;
        p = tree.Traverse(point);

        ray.Origin = point;
        hit = tree.RayCastBoundingBox(ray, &t);
        if (hit >= 0)
        {
            BoundingBox3d bb = boxes[hit];
            // assert(ray.IntersectAABB(bb.Min, bb.Max, &t));      // TODO
        }
    }

    return;
}

void TestGeometryQuery()
{
    GeometryQuery scene;
    
    std::vector<GeometryObject*> objs;
    objs.emplace_back(GeometryObjectFactory::CreatePlane(Vector3d::Zero(), Vector3d::UnitZ(), 0.0f));
    objs.emplace_back(GeometryObjectFactory::CreatePlane(Vector3d::Zero(), Vector3d::UnitZ(), -10.0f));
    objs.emplace_back(GeometryObjectFactory::CreateAABB(Vector3d::Zero(), Vector3d(-1, -1, -1), Vector3d(1, 1, 1)));
    scene.BuildStaticGeometry(objs, 1);

    RayCastResult result;
    scene.RayCast(Vector3d(0.0f, 0.0f, 5.0f), Vector3d(0.0f, 0.0f, -1.0f), &result);
    assert(result.hit);
    assert(fabsf(result.t - 4.0f) < 0.001f);

    scene.RayCast(Vector3d(0.0f, 0.0f, -5.0f), Vector3d(0.0f, 0.0f, -1.0f), &result);
    assert(!result.hit);

    scene.RayCast(Vector3d(0.0f, 0.0f, 15.0f), Vector3d(0.0f, 0.0f, -1.0f), &result);
    assert(result.hit);
    assert(fabsf(result.t - 5.0f) < 0.001f);

    for (auto obj : objs)
    {
        delete obj;
    }

    return;
}

void TestSAP()
{
    std::set<sap_key> overlaps;
    std::vector<BoundingBox3d> boxes;
    boxes.emplace_back(Vector3d(0, 0, 0), Vector3d(2, 2, 2));
    boxes.emplace_back(Vector3d(1, 1, 1), Vector3d(3, 3, 3));
    boxes.emplace_back(Vector3d(0, 0, 0), Vector3d(15, 15, 15));
    sap_direct(boxes, &overlaps);
    assert(overlaps.size() == 3);

    boxes[2] = BoundingBox3d(Vector3d(10, 10, 10), Vector3d(15, 15, 15));
    sap_direct(boxes, &overlaps);
    assert(overlaps.size() == 1);

    for (int i = 0; i < 100; ++i)
    {
        Vector3d point1 = Vector3d::Random() * 100.0f;
        Vector3d point2 = Vector3d::Random() * 100.0f;
        boxes.emplace_back(point1, point1 + point2);
    }
    sap_direct(boxes, &overlaps);

    for (size_t i = 0; i < boxes.size(); ++i)
    for (size_t j = 0; j < boxes.size(); ++j)
    {
        if (i == j) continue;
        sap_key key = sap_pack_key((int)i, (int)j);
        if (boxes[i].Intersect(boxes[j]))
        {
            assert(overlaps.count(key) == 1);
        }
        else
        {
            assert(overlaps.count(key) == 0);
        }
    }

    return;
}

void TestSAPInc()
{
    std::vector<GeometryObject*> objs;
    objs.emplace_back(GeometryObjectFactory::CreateAABB(Vector3d::Zero(), Vector3d(0, 0, 0), Vector3d(1, 1, 1)));
    objs.emplace_back(GeometryObjectFactory::CreateAABB(Vector3d::Zero(), Vector3d(2, 2, 2), Vector3d(3, 3, 3)));
    objs.emplace_back(GeometryObjectFactory::CreateAABB(Vector3d::Zero(), Vector3d(10, 10, 10), Vector3d(20, 20, 20)));

    std::vector<std::vector<sweep_point>> axis;
    axis.resize(3);
    sap_incremental_init(objs, 0, axis[0]);
    sap_incremental_init(objs, 1, axis[1]);
    sap_incremental_init(objs, 2, axis[2]);

    std::set<sap_key> overlaps;
    sap_incremental(axis, objs, &overlaps);
    assert(overlaps.size() == 0);

    objs[2]->SetPositionOffset(Vector3d(-10, -10, -10));
    sap_incremental(axis, objs, &overlaps);
    assert(overlaps.size() == 2);

    sap_incremental(axis, objs, &overlaps);
    assert(overlaps.size() == 2);

    objs[2]->SetPositionOffset(Vector3d(10, 10, 10));
    sap_incremental(axis, objs, &overlaps);
    assert(overlaps.size() == 0);

    for (int i = 0; i < 100; ++i)
    {
        Vector3d point1 = Vector3d::Random() * 100.0f;
        Vector3d point2 = point1 + Vector3d::Random() * 100.0f;
        objs.emplace_back(GeometryObjectFactory::CreateAABB(Vector3d::Zero(), point1, point2));
    }
    sap_incremental(axis, objs, &overlaps);
    sap_incremental(axis, objs, &overlaps);
    sap_incremental(axis, objs, &overlaps);
    sap_incremental(axis, objs, &overlaps);
    sap_incremental(axis, objs, &overlaps);
    sap_incremental(axis, objs, &overlaps);

    return;
}

void TestMainEntry()
{
    TestAABBTree();
    TestGeometryQuery();
    TestSAP();
    TestSAPInc();
    return;
}
