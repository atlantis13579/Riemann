
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
#include "../Src/Geometry/Triangle.h"
#include "../Src/Geometry/Cylinder.h"
#include "../Src/Geometry/Capsule.h"
#include "../Src/Collision/AABBTree.h"

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
    p = tree.Traverse(Vector3d(0.5f, 0.5f, 0.5f));
    p = tree.Traverse(Vector3d(0.5f, 0.5f, 2.5f));

    boxes.clear();
    for (int i = 0; i < 10000; ++i)
    {
        boxes.emplace_back(
                Vector3d(RandomFloat01(), RandomFloat01(), RandomFloat01()) * 100.0f,
                Vector3d(RandomFloat01(), RandomFloat01(), RandomFloat01()) * 100.0f);
    }

    AABBTreeBuildData param2(&boxes[0], (int)boxes.size(), 1);

    tree.Release();
    tree.StaticBuild(param2);

    for (int i = 0; i < 10000; ++i)
    {
        Vector3d point = Vector3d(RandomFloat01(), RandomFloat01(), RandomFloat01()) * 100.0f;
        p = tree.Traverse(point);
    }

    return;
}


void TestMainEntry()
{
    TestAABBTree();

    return;
}
