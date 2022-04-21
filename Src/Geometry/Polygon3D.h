#pragma once

#include "../Maths/Vector3d.h"

float TriangleArea3D(const Vector3d& a, const Vector3d& b, const Vector3d& c);

float PolygonArea3D(const Vector3d* polygon, int nvert);