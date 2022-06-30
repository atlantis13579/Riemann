#include "PythonModule.h"

#include <assert.h>
#include "../RigidBodyDynamics/RigidBodySimulation.h"
#include "../Collision/GeometryQuery.h"

#ifdef BUILD_PYTHON_MODULE
extern "C"
{

void* LoadPhysxScene(const char *filepath)
{
    RigidBodySimulationParam param;
    RigidBodySimulation *world = new RigidBodySimulation(param);
    assert(world);
    if (!world->LoadPhysxScene(filepath))
    {
        delete world;
        return nullptr;
    }
    return world;
}

float RayCast(void *p, float x0, float y0, float z0, float dx, float dy, float dz)
{
    if (p == nullptr)
    {
        return -1.0f;
    }
    
    RigidBodySimulation* world = (RigidBodySimulation*)p;
    assert(world);
    
    RayCastOption Option;
    RayCastResult Result;
    GeometryQuery* query = world->GetGeometryQuery();
    assert(query);
    
    bool success = query->RayCast(Vector3d(x0, y0, z0), Vector3d(dx, dy, dz).Unit(), Option, &Result);
    if (success)
    {
        return (Result.hitPoint - Vector3d(x0, y0, z0)).Length();
    }
    return -1.0f;
}

float RayCast2(void *p, float x0, float y0, float z0, float x1, float y1, float z1)
{
    Vector3d Dir = Vector3d(x1 - x0, y1 - y0, z1 - z0).Unit();
    return RayCast(p, x0, y0, z0, Dir.x, Dir.y, Dir.z);
}

}
#endif // BUILD_PYTHON_MODULE