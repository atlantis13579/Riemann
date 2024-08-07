
#include "PythonModule.h"

#ifdef BUILD_PYTHON_MODULE

#include <assert.h>
#include "../../Core/JobSystem.h"
#include "../../RigidBodyDynamics/RigidBodySimulation.h"
#include "../../Collision/GeometryQuery.h"
#include "../ImageSpace/ImageProcessing.h"
#include "libPng.h"



extern "C"
{
void* LoadScene(const char *filepath)
{
    Riemann::RigidBodySimulationParam param;
    Riemann::RigidBodySimulation *world = new Riemann::RigidBodySimulation(param);
    assert(world);
    if (!world->LoadScene(filepath, true))
    {
        delete world;
        return nullptr;
    }
    return world;
}

void  DeleteScene(void* p)
{
	if (p)
	{
        Riemann::RigidBodySimulation* world = (Riemann::RigidBodySimulation*)p;
        delete world;
	}
}

float RayCast(void *p, float x0, float y0, float z0, float dx, float dy, float dz)
{
    if (p == nullptr)
    {
        return -1.0f;
    }
    
    Riemann::RigidBodySimulation* world = (Riemann::RigidBodySimulation*)p;
    assert(world);
    
    Riemann::RayCastOption Option;
    Riemann::RayCastResult Result;
    Riemann::GeometryQuery* query = world->GetGeometryQuery();
    assert(query);
    
    bool success = query->RayCastQuery(Vector3(x0, y0, z0), Vector3(dx, dy, dz).Unit(), Option, &Result);
    if (success)
    {
        return (Result.hitPoint - Vector3(x0, y0, z0)).Length();
    }
    return -1.0f;
}

float RayCast2(void *p, float x0, float y0, float z0, float x1, float y1, float z1)
{
    Vector3 Dir = Vector3(x1 - x0, y1 - y0, z1 - z0).Unit();
    return RayCast(p, x0, y0, z0, Dir.x, Dir.y, Dir.z);
}

Riemann::JobSystem* jobSys = nullptr;

void RenderDepthImage(void* world_ptr, void* ptr, int width, int height, float fov, float nearz, float farz,
                      float x0, float y0, float z0, float dx, float dy, float dz, float ux, float uy, float uz, bool debug_draw)
{
    if (ptr == nullptr || world_ptr == nullptr)
    {
        return;
    }

	if (width <= 2 || height <= 2)
	{
		return;
	}

    if (jobSys == nullptr)
    {
        jobSys = new Riemann::JobSystem();
        jobSys->CreateWorkers(-1);
    }

    bool downscale = true;
    if ((width & 1) || (height & 1))
    {
        downscale = false;
    }

    int w2 = downscale ? width / 2 : width;
    int h2 = downscale ? height / 2 : height;
    std::vector<float>  buffer;
    if (downscale)
    {
        buffer.resize(w2 * h2);
    }
    float* fp = downscale ? &buffer[0] : (float*)ptr;

    Riemann::RigidBodySimulation* world = (Riemann::RigidBodySimulation*)world_ptr;
	assert(world);

    Riemann::GeometryQuery* query = world->GetGeometryQuery();
	assert(query);

    Vector3 cameraOrigin(x0, y0, z0);
    Vector3 cameraDirection = Vector3(dx, dy, dz).Unit();
    Vector3 cameraUp = Vector3(ux, uy, uz).Unit();
    float AspectRatio = 1.0f * w2 / h2;

    nearz = std::max(0.1f, nearz);
	float CX = 2.0f * nearz * tanf(0.5f * fov);
	float CY = CX / AspectRatio;

    Vector3 cameraX = cameraUp.Cross(cameraDirection) * CX;
    Vector3 cameraY = cameraX.Cross(cameraDirection).Unit() * CY;

    Riemann::JobGraph graph;

	for (int y = 0; y < h2; ++y)
    {
        graph.AddJob(std::to_string(y).c_str(), [=] {
            Riemann::RayCastOption Option;
            Option.MaxDist = farz;
            for (int x = 0; x < w2; ++x)
            {
                Vector3 rayDirection = cameraDirection * nearz + ((y - h2 * 0.5f) / h2) * cameraY + ((x - w2 * 0.5f) / w2) * cameraX;
                Riemann::RayCastResult Result;
                bool success = query->RayCastQuery(cameraOrigin, rayDirection.Unit(), Option, &Result);
                if (success)
                {
                    float depth = Result.hitTimeMin;
                    fp[y * w2 + w2 - 1 - x] = std::min(depth, Option.MaxDist);
                }
                else
                {
                    fp[y * w2 + w2 - 1 - x] = Option.MaxDist;
                }
                Option.Cache.prevhitGeom = Result.hitGeom;
            }});
    }

    jobSys->ExecuteGraph(graph);

    if (downscale)
    {
        fp = (float*)ptr;
        ImageUpscale2X<float>(&buffer[0], w2, h2, ScaleMethod::LAPLACIAN, fp);
    }

    if (debug_draw)
    {
        LibPNG::WritePNG("../TestData/depth.png", fp, width, height);
    }
}

}
#endif // BUILD_PYTHON_MODULE
