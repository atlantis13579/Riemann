#include <float.h>
#include <vector>

#include "Polygon2d.h"

static float SqrtDistancePointToSegment2D(const Vector2d& point, const Vector2d& P0, const Vector2d& P1, float* dt)
{
	float pqx = P1.x - P0.x;
	float pqy = P1.y - P0.y;
	float dx = point.x - P0.x;
	float dy = point.y - P0.y;
	float d = pqy * pqy + pqx * pqx;
	float t = pqy * dy + pqx * dx;
	if (d > 0) t /= d;
	if (t < 0) t = 0;
	else if (t > 1) t = 1;
	dy = P0.y + t * pqy - point.y;
	dx = P0.x + t * pqx - point.x;
	if (dt) *dt = t;
	return dy * dy + dx * dx;
}

static bool SqrDistancePointToPolygon2D(const Vector2d& point, const Vector2d* polygon, int nvert, float* ed, float* et)
{
	int i, j;
	bool c = false;
	for (i = 0, j = nvert - 1; i < nvert; j = i++) {
		const Vector2d& vi = polygon[i];
		const Vector2d& vj = polygon[j];
		if (((vi.x > point.x) != (vj.x > point.x)) &&
			(point.y < (vj.y - vi.y) * (point.x - vi.x) / (vj.x - vi.x) + vi.y))
			c = !c;
		ed[j] = SqrtDistancePointToSegment2D(point, vj, vi, &et[j]);
	}
	return c;
}

Vector2d ClosestPointInPolygon2D(const Vector2d& pt, const Vector2d* polygon, int nvert, bool* inside)
{
	std::vector<float>  edged;
	std::vector<float>  edget;
	edged.resize(nvert);
	edget.resize(nvert);
	*inside = SqrDistancePointToPolygon2D(pt, polygon, nvert, &edged[0], &edget[0]);
	if (*inside) {
		// Point is inside the polygon, return the point.
		return pt;
	}

	// Point is outside the polygon, clamp to nearest edge.
	float dmin = FLT_MAX;
	int imin = -1;
	for (int i = 0; i < nvert; ++i) {
		if (edged[i] < dmin) {
			dmin = edged[i];
			imin = i;
		}
	}

	const Vector2d& va = polygon[imin];
	const Vector2d& vb = polygon[(imin + 1) % nvert];
	return Vector2d::Lerp(va, vb, edget[imin]);
}


bool PointInPolygon2D(const Vector2d& pt, const Vector2d* polygon, int nvert)
{
	bool c = false;
	for (int i = 0, j = nvert - 1; i < nvert; j = i++) {
		const Vector2d* pi = polygon + i;
		const Vector2d* pj = polygon + j;
		if (((pi->y > pt.y) != (pj->y > pt.y)) &&
			(pt.x < (pj->x - pi->x) * (pt.y - pi->y) /
				(pj->y - pi->y) + pi->x))
			c = !c;
	}
	return c;
}

float PolygonArea2D(const Vector2d* polygon, int nvert)
{
	if (nvert < 3)
		return 0.0f;

	float area = 0.0f;
	for (int i = 0; i < nvert; ++i)
	{
		int k = (i + 1) % nvert;
		area += (polygon[i].x * polygon[k].y - polygon[i].y * polygon[k].x);
	}
	return 0.5f * fabsf(area);
}

