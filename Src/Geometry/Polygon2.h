#pragma once

#include "../Maths/Vector2.h"

namespace Geometry
{
	float TriangleArea2D(const Vector2& a, const Vector2& b, const Vector2& c);
	float TriangleArea2D_Signed(const Vector2& a, const Vector2& b, const Vector2& c);

	Vector2 ClosestPointInPolygon2D(const Vector2& pt, const Vector2* polygon, int nvert, bool* inside);

	bool PointInPolygon2D(const Vector2& pt, const Vector2* polygon, int nvert);

	float PolygonArea2D(const Vector2* polygon, int nvert);

	int PointFarthestFromEdge(const Vector2& a, const Vector2& b, Vector2* polygon, int nvert);

	float MinAreaRect(const Vector2* pt, int numPts, Vector2& c, Vector2 u[2]);
}