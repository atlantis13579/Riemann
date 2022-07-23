#pragma once

#include "../Maths/Vector3.h"

float PolygonArea3D(const Vector3d* polygon, int nvert);

void ClipPolygonByPlane3D(const Vector3d* polygon, int n, const Vector3d& Origin, const Vector3d& Normal, Vector3d* clipped, int* nc);

void ClipPolygonByProjectSegment3D(const Vector3d* polygon, int n, const Vector3d& P0, const Vector3d& P1,
									const Vector3d& Normal, Vector3d* clipped, int* nc);

void ClipPolygonByProjectPolygon3D(const Vector3d* polygon, int n1, const Vector3d* projPoly, int n2,
									const Vector3d& projNormal, Vector3d* clipped, int* nc);

void ClipPolygonByAABB3D(const Vector3d* polygon, int n1, const Vector3d& Min, const Vector3d& Max, Vector3d* clipped, int* nc);

bool ClipPolygonAgainPolygon3D(const Vector3d* poly1, int n1, const Vector3d* poly2, int n2, const Vector3d& dir,
								float maxDistSqr, Vector3d* clipped1, int* c1, Vector3d* clipped2, int* c2);