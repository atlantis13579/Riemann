#pragma once

#include <vector>

class Geometry;

class MotionIntegration
{
public:
	static void Integrate(std::vector<Geometry*> Entities, float dt);
};