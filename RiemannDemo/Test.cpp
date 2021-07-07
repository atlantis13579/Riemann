
#include <assert.h>
#include <vector>

#include "../Src/LinearSystem/JacobiIteration_CPU.h"
#include "../Src/LinearSystem/GaussSeidelIteration_CPU.h"
#include "../Src/LinearSystem/LUFactorization.h"
#include "../Src/Maths/BoundingBox3d.h"
#include "../Src/Maths/Maths.h"
#include "../Src/Geometry/AxisAlignedBox.h"
#include "../Src/Geometry/Plane.h"
#include "../Src/Geometry/Sphere.h"
#include "../Src/Geometry/Ray.h"
#include "../Src/Geometry/Triangle.h"
#include "../Src/Geometry/Cylinder.h"
#include "../Src/Geometry/Capsule.h"
#include "../Src/Collision/AABBTree.h"
#include "../Src/Collision/GeometryQuery.h"

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
        Vector3d point1 = Vector3d(RandomFloat01(), RandomFloat01(), RandomFloat01()) * 100.0f;
        Vector3d point2 = Vector3d(RandomFloat01(), RandomFloat01(), RandomFloat01()) * 100.0f;
        boxes.emplace_back(point1, point1 + point2);
    }

    AABBTreeBuildData param2(&boxes[0], (int)boxes.size(), 1);

    tree.Release();
    tree.StaticBuild(param2);

    for (int i = 0; i < 10000; ++i)
    {
        Vector3d point = Vector3d(RandomFloat01(), RandomFloat01(), RandomFloat01()) * 100.0f;
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
    
    std::vector<GeometryObject> objs;
    objs.emplace_back(GeometryShapeType::PLANE, new Plane(Vector3d::UnitZ(), 0.0f));
    objs.emplace_back(GeometryShapeType::PLANE, new Plane(Vector3d::UnitZ(), -10.0f));
    objs.emplace_back(GeometryShapeType::AABB, new AxisAlignedBox3(Vector3d(-1, -1, -1), Vector3d(1, 1, 1)));
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

    return;
}

void TestMainEntry()
{
    TestAABBTree();
    TestGeometryQuery();
    return;
}
