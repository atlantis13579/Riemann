#pragma once

#include <vector>
class Geometry;

bool LoadPhysxBinary(const char* Filename, std::vector<Geometry*> *GeometryList);
