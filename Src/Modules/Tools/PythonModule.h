#pragma once

#define BUILD_PYTHON_MODULE

#ifdef BUILD_PYTHON_MODULE
extern "C"
{
void* LoadPhysxScene(const char *filepath);
void  DeletePhysxScene(void* p);

float RayCast(void *p, float x0, float y0, float z0, float dx, float dy, float dz);
float RayCast2(void *p, float x0, float y0, float z0, float x1, float y1, float z1);
void RenderDepthImage(void* p, void* dataptr, int width, int height, float fov, float nearz, float farz,
					  float x0, float y0, float z0, float dx, float dy, float dz, float ux, float uy, float uz, bool debug_draw = false);
}
#endif // BUILD_PYTHON_MODULE
