#pragma once

#include <vector>
class Geometry;

bool	LoadPhysxBinary(const char* Filename, std::vector<Geometry*> *GeometryList);
void*	LoadPhysxBinaryMmap(const char* Filename, std::vector<Geometry*>* GeometryList, size_t& mem_size);
void	ReleaseSharedMem(void* addr, size_t size);