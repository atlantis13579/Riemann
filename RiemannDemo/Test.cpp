
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
    boxes.emplace_back(Vector3d(0, 0, 0), Vector3d(1, 1, 2));
    boxes.emplace_back(Vector3d(1, 1, 1), Vector3d(1, 1, 2));

    AABBTree tree;

    {
        AABBTreeBuildParams param(&boxes[0], (int)boxes.size(), 1);

        tree.Release();
        tree.StaticBuild(param);
    }

    {
        boxes.clear();
        for (int i = 0; i < 10000; ++i)
        {
            boxes.emplace_back(
                Vector3d(RandomFloat01(), RandomFloat01(), RandomFloat01()) * 100.0f,
                Vector3d(RandomFloat01(), RandomFloat01(), RandomFloat01()) * 100.0f);
        }

        AABBTreeBuildParams param(&boxes[0], (int)boxes.size(), 1);

        tree.Release();
        tree.StaticBuild(param);
    }

    return;
}


void TestMainEntry()
{
    TestAABBTree();

    return;
}
