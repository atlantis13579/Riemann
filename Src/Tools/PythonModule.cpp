#include "PythonModule.h"

#include <omp.h> 
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

inline int  toInt(float x)
{
    return int(x * 255.0f + 0.5f);
}

static void WritePPM(const char* filename, float* p, int w, int h)
{
	FILE* f = fopen(filename, "w");
	if (f)
    {
        float minz = FLT_MAX;
		float maxz = -FLT_MAX;
		for (int i = 0; i < w * h; ++i)
		{
            if (p[i] <= 0.0f) continue;
			if (p[i] > maxz)
				maxz = p[i];
            if (p[i] < minz)
                minz = p[i];
		}
		fprintf(f, "P3\n%d %d\n%d\n", w, h, 255);
        for (int i = 0; i < w * h; ++i)
        {
            if (p[i] <= 0.0f)
            {
                fprintf(f, "255 0 0 ");
                continue;
            }
            float x = (p[i] - minz) / (maxz - minz);
            fprintf(f, "%d %d %d ", toInt(x), toInt(x), toInt(x));
        }
        fclose(f);
	}
}

void RenderDepthImage(void* p, void* dataptr, int width, int height, float fov, float nearz, float farz,
                      float x0, float y0, float z0, float dx, float dy, float dz, float ux, float uy, float uz)
{
    if (dataptr == nullptr || p == nullptr)
    {
        return;
    }

	if (width <= 1 || height <= 1)
	{
		return;
	}

	RigidBodySimulation* world = (RigidBodySimulation*)p;
	assert(world);

	RayCastOption Option;
    Option.MaxDist = 50.0f;

	GeometryQuery* query = world->GetGeometryQuery();
	assert(query);

    float* fp = (float*)dataptr;

    Vector3d cameraOrigin(x0, y0, z0);
    Vector3d cameraDirection = Vector3d(dx, dy, dz).Unit();
    Vector3d cameraUp = Vector3d(ux, uy, uz).Unit();
    float AspectRatio = 1.0f * width / height;
    float NearZ = 0.1f;

	float CX = 2.0f * NearZ * tanf(0.5f * fov);
	float CY = CX * AspectRatio;

    Vector3d cameraX = cameraUp.Cross(cameraDirection) * CX;
    Vector3d cameraY = cameraX.Cross(cameraDirection).Unit() * CY;

    float dt = cameraX.Dot(cameraY);

    #pragma omp parallel for schedule(dynamic, 1)
	for (int y = 0; y < height; ++y)
    {
        for (int x = 0; x < width; ++x)
        {
            Vector3d rayDirection = cameraDirection * NearZ + ((y - height * 0.5f) / height) * cameraY + ((x - width * 0.5f) / width) * cameraX;
            
            RayCastResult Result;
            bool success = query->RayCast(cameraOrigin, rayDirection.Unit(), Option, &Result);
			if (success)
			{
				float depth = Result.hitTimeMin;
                fp[y * width + x] = std::min(depth, Option.MaxDist);
			}
            else
            {
                fp[y * width + x] = 0.0f;
            }
        }
    }

    WritePPM("data/depth.ppm", fp, width, height);
}

}
#endif // BUILD_PYTHON_MODULE