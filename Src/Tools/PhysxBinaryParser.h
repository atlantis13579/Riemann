#pragma once

#include <vector>

class Geometry;

class PhysxBinaryParser
{
public:
	static bool ParseCollectionFromBinary(const char* Filename, std::vector<Geometry*> *GeometryList);
};
