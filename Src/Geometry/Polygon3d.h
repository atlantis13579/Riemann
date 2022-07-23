#pragma once

#include "../Maths/Vector3.h"

float PolygonArea3D(const Vector3d* polygon, int nvert);

bool ClipPolygon3D(const Vector3d* poly1, int n1, const Vector3d* poly2, int n2, const Vector3d& dir,
					float inSpeculativeContactDistanceSq, Vector3d* outContactPoints1, int& c1, Vector3d* outContactPoints2, int& c2);