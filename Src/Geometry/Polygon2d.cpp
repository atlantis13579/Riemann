#include <float.h>
#include <vector>

#include "Polygon2d.h"

float Signed2DTriArea(const Vector2& a, const Vector2& b, const Vector2& c)
{
	return (a.x - c.x) * (b.y - c.y) - (a.y - c.y) * (b.x - c.x);
}

int SegmentIntersectSegment2D(const Vector2& a, const Vector2& b, const Vector2& c, const Vector2& d, float *t, Vector2 *p)
{
	float a1 = Signed2DTriArea(a, b, d);
	float a2 = Signed2DTriArea(a, b, c);

	if (a1 * a2 < 0.0f)
	{
		float a3 = Signed2DTriArea(c, d, a);
		float a4 = a3 + a2 - a1;
		if (a3 * a4 < 0.0f)
		{
			*t = a3 / (a3 - a4);
			*p = a + (*t) * (b - a);
			return true;
		}
	}

	return false;
}

static float SqrtDistancePointToSegment2D(const Vector2& point, const Vector2& P0, const Vector2& P1, float* dt)
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

static bool SqrDistancePointToPolygon2D(const Vector2& point, const Vector2* polygon, int nvert, float* ed, float* et)
{
	int i, j;
	bool c = false;
	for (i = 0, j = nvert - 1; i < nvert; j = i++) {
		const Vector2& vi = polygon[i];
		const Vector2& vj = polygon[j];
		if (((vi.x > point.x) != (vj.x > point.x)) &&
			(point.y < (vj.y - vi.y) * (point.x - vi.x) / (vj.x - vi.x) + vi.y))
			c = !c;
		ed[j] = SqrtDistancePointToSegment2D(point, vj, vi, &et[j]);
	}
	return c;
}

Vector2 ClosestPointInPolygon2D(const Vector2& pt, const Vector2* polygon, int nvert, bool* inside)
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

	const Vector2& va = polygon[imin];
	const Vector2& vb = polygon[(imin + 1) % nvert];
	return Vector2::Lerp(va, vb, edget[imin]);
}


bool PointInPolygon2D(const Vector2& pt, const Vector2* polygon, int nvert)
{
	bool c = false;
	for (int i = 0, j = nvert - 1; i < nvert; j = i++) {
		const Vector2* pi = polygon + i;
		const Vector2* pj = polygon + j;
		if (((pi->y > pt.y) != (pj->y > pt.y)) &&
			(pt.x < (pj->x - pi->x) * (pt.y - pi->y) /
				(pj->y - pi->y) + pi->x))
			c = !c;
	}
	return c;
}

float PolygonArea2D(const Vector2* polygon, int nvert)
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

int PointFarthestFromEdge(const Vector2& a, const Vector2& b, Vector2 *polygon, int nvert)
{
	Vector2 e = b - a;
	Vector2 eperp = Vector2(-e.y, e.x);
	int bestIndex = -1;
	float maxVal = -FLT_MAX, rightMostVal = -FLT_MAX;
	for (int i = 1; i < nvert; i++)
	{
		float d = DotProduct(polygon[i] - a, eperp);
		float r = DotProduct(polygon[i] - a, e);
		if (d > maxVal || (d == maxVal && r > rightMostVal)) {
			bestIndex = i;
			maxVal = d;
			rightMostVal = r;
		}
	}
	return bestIndex;
}

float MinAreaRect(const Vector2 *points, int n, Vector2 &center, Vector2 axis[2])
{
	float minArea = FLT_MAX;
	for (int i = 0, j = n - 1; i < n; j = i, i++)
	{
		Vector2 e0 = (points[i] - points[j]).Unit();
		Vector2 e1 = Vector2(-e0.y, e0.x); // = Perp2D(e0)
		float min0 = 0.0f, min1 = 0.0f, max0 = 0.0f, max1 = 0.0f;
		for (int k = 0; k < n; k++)
		{
			Vector2 d = points[k] - points[j];
			float dot = d.Dot(e0);
			min0 = std::min(min0, dot);
			max0 = std::max(max0, dot);
			dot = d.Dot(e1);
			min1 = std::min(min1, dot);
			max1 = std::max(max1, dot);
		}
		const float area = (max0 - min0) * (max1 - min1);
		if (area < minArea)
		{
			minArea = area;
			center = points[j] + 0.5f * ((min0 + max0) * e0 + (min1 + max1) * e1);
			axis[0] = e0;
			axis[1] = e1;
		}
	}
	return minArea;
}
