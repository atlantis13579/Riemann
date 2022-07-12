
#include "PythonModule.h"

#ifdef BUILD_PYTHON_MODULE

#include <assert.h>
#include "../RigidBodyDynamics/RigidBodySimulation.h"
#include "../Collision/GeometryQuery.h"
#include "../ImageSpace/ImageProcessing.h"
#include "../Tools/libPng.h"

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

void  DeletePhysxScene(void* p)
{
	if (p)
	{
        RigidBodySimulation* world = (RigidBodySimulation*)p;
        delete world;
	}
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

void RenderDepthImage(void* world_ptr, void* ptr, int width, int height, float fov, float nearz, float farz,
                      float x0, float y0, float z0, float dx, float dy, float dz, float ux, float uy, float uz, bool debug_draw)
{
    if (ptr == nullptr || world_ptr == nullptr)
    {
        return;
    }

	if (width <= 1 || height <= 1)
	{
		return;
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

	RigidBodySimulation* world = (RigidBodySimulation*)world_ptr;
	assert(world);

	RayCastOption Option;
    Option.MaxDist = farz;

	GeometryQuery* query = world->GetGeometryQuery();
	assert(query);

    Vector3d cameraOrigin(x0, y0, z0);
    Vector3d cameraDirection = Vector3d(dx, dy, dz).Unit();
    Vector3d cameraUp = Vector3d(ux, uy, uz).Unit();
    float AspectRatio = 1.0f * w2 / h2;

    nearz = std::max(0.1f, nearz);
	float CX = 2.0f * nearz * tanf(0.5f * fov);
	float CY = CX / AspectRatio;

    Vector3d cameraX = cameraUp.Cross(cameraDirection) * CX;
    Vector3d cameraY = cameraX.Cross(cameraDirection).Unit() * CY;

    #pragma omp parallel for schedule(dynamic, 1)
	for (int y = 0; y < h2; ++y)
    {
        for (int x = 0; x < w2; ++x)
        {
            Vector3d rayDirection = cameraDirection * nearz + ((y - h2 * 0.5f) / h2) * cameraY + ((x - w2 * 0.5f) / w2) * cameraX;
            
            RayCastResult Result;
            bool success = query->RayCast(cameraOrigin, rayDirection.Unit(), Option, &Result);
			if (success)
			{
				float depth = Result.hitTimeMin;
                fp[y * w2 + w2 - 1 - x] = std::min(depth, Option.MaxDist);
			}
            else
            {
                fp[y * w2 + w2 - 1 - x] = Option.MaxDist;
            }
        }
    }

    if (downscale)
    {
        fp = (float*)ptr;
        ImageUpscale2X<float>(&buffer[0], w2, h2, ScaleMethod::LAPLACIAN, fp);
    }

    if (debug_draw)
    {
        LibPNG::WritePNG("data/depth.png", fp, width, height);
    }
}

}
#endif // BUILD_PYTHON_MODULE
