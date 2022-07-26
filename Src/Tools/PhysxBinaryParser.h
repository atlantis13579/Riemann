#pragma once

#include <vector>

class RigidBody;

bool	LoadPhysxBinary(const char* Filename, std::vector<RigidBody*> *bodies);
void*	LoadPhysxBinaryMmap(const char* Filename, std::vector<RigidBody*>* bodies, size_t& mem_size);
void	ReleaseSharedMem(void* addr, size_t size);