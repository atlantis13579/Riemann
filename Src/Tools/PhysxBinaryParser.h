#pragma once

#include <vector>
class Geometry;

bool	LoadPhysxBinary(const char* Filename, std::vector<Geometry*> *GeometryList);
void*	LoadPhysxBinaryMmap(const char* Filename, std::vector<Geometry*>* GeometryList);