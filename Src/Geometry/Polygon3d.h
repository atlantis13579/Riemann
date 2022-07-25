#pragma once

#include "../Maths/Vector3.h"

float PolygonArea3D(const Vector3* polygon, int nvert);

void ClipPolygonByPlane3D(const Vector3* polygon, int n, const Vector3& Origin, const Vector3& Normal, Vector3* clipped, int* nc);

void ClipPolygonByProjectSegment3D(const Vector3* polygon, int n, const Vector3& P0, const Vector3& P1,
									const Vector3& Normal, Vector3* clipped, int* nc);

void ClipPolygonByProjectPolygon3D(const Vector3* polygon, int n1, const Vector3* projPoly, int n2,
									const Vector3& projNormal, Vector3* clipped, int* nc);

void ClipPolygonByAABB3D(const Vector3* polygon, int n1, const Vector3& Min, const Vector3& Max, Vector3* clipped, int* nc);

bool ClipPolygonAgainPolygon3D(const Vector3* poly1, int n1, const Vector3* poly2, int n2, const Vector3& dir,
								float maxsqrDistFactor, Vector3* clipped1, int* c1, Vector3* clipped2, int* c2);
