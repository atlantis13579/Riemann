#pragma once

#include <vector>

class RigidBody;
class Geometry;

bool	LoadPhysxBinary(const char* Filename, std::vector<RigidBody*> *bodies, std::vector<Geometry*>* geoms);
void*	LoadPhysxBinaryMmap(const char* Filename, std::vector<RigidBody*>* bodies, std::vector<Geometry*>* geoms, size_t& mem_size);
void	ReleaseSharedMem(void* addr, size_t size);