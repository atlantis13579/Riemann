#pragma once

#include <vector>
#include "../../Maths/Vector3.h"

namespace Riemann
{
class RigidBody;
class Geometry;

bool	LoadPhysxBinary(const char* Filename, std::vector<RigidBody*> *bodies, std::vector<Geometry*>* geoms);
void*	LoadPhysxBinaryMmap(const char* Filename, std::vector<RigidBody*>* bodies, std::vector<Geometry*>* geoms, size_t& mem_size);
bool	LoadPhysxBinaryTriangles(const char* Filename, std::vector<Vector3>& vertices, std::vector<int>& indices);
void	ReleaseSharedMem(void* addr, size_t size);

}
