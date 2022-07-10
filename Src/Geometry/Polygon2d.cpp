#include <float.h>
#include <vector>

#include "Polygon2d.h"

static float DistancePtSegSqr(const Vector2d& pt, const Vector2d& p, const Vector2d& q, float* dt)
{
	float pqx = q.x - p.x;
	float pqy = q.y - p.y;
	float dx = pt.x - p.x;
	float dy = pt.y - p.y;
	float d = pqy * pqy + pqx * pqx;
	float t = pqy * dy + pqx * dx;
	if (d > 0) t /= d;
	if (t < 0) t = 0;
	else if (t > 1) t = 1;
	dy = p.y + t * pqy - pt.y;
	dx = p.x + t * pqx - pt.x;
	if (dt) *dt = t;
	return dy * dy + dx * dx;
}

static bool SqrDistanceToEdges(const Vector2d& pt, const Vector2d* polygon, int nvert, float* ed, float* et)
{
	int i, j;
	bool c = false;
	for (i = 0, j = nvert - 1; i < nvert; j = i++) {
		const Vector2d& vi = polygon[i];
		const Vector2d& vj = polygon[j];
		if (((vi.x > pt.x) != (vj.x > pt.x)) &&
			(pt.y < (vj.y - vi.y) * (pt.x - vi.x) / (vj.x - vi.x) + vi.y))
			c = !c;
		ed[j] = DistancePtSegSqr(pt, vj, vi, &et[j]);
	}
	return c;
}

Vector2d ClosestPointInPolygon2D(const Vector2d& pt, const Vector2d* polygon, int nvert, bool* inside)
{
	std::vector<float>  edged;
	std::vector<float>  edget;
	edged.resize(nvert);
	edget.resize(nvert);
	*inside = SqrDistanceToEdges(pt, polygon, nvert, &edged[0], &edget[0]);
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

