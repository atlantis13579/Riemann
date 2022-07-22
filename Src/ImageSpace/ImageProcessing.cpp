
#include "ImageProcessing.h"

#include <vector>
#include <queue>
#include <map>

#include "../Maths/Vector2.h"

const int kMinPolygonPoints = 5;

int triangle_area(const Vector2i &p1, const Vector2i &p2, const Vector2i &p3)
{
	return abs((p2 - p1).Cross(p3 - p1));
}

void PolygonSimplification_VisvalingamWhyatt(std::vector<Vector2i> &polygon, int treshold)
{
	while (polygon.size() > kMinPolygonPoints)
	{
		int i_min = -1;
		int area_min = 0x7FFFFFFF;
		int num = (int)polygon.size();
		for (int i0 = 0; i0 < num; ++i0) {
			int i1 = (i0 + num - 1) % num;
			int i2 = (i0 + num + 1) % num;
			int area = triangle_area(polygon[i0], polygon[i1], polygon[i2]);
			if (area < area_min) {
				i_min = i0;
				area_min = area;
			}
		}

		if (i_min == -1)
			break;
		if (area_min > treshold)
			break;

		std::vector<Vector2i>::iterator it = polygon.begin() + i_min;
		polygon.erase(it);
	}
}

static int distanceToSegment(const Vector2i &p, const Vector2i &start, const Vector2i &end) {
	if (end.x == start.x) {
		return abs(p.x - end.x);
	}
	float m1 = (end.y - start.y) * 1.0f / (end.x - start.x);
	float c1 = start.y - m1 * start.x;
	float interPointX = 0.0f;
	float interPointY = 0.0f;
	if (fabs(m1) <  0.001f) {
		interPointX = (float)p.x;
		interPointY = (float)c1;
	}
	else {
		float m2 = -1.0f / m1;
		float c2 = p.y - m2 * p.x;
		interPointX = (c1 - c2) / (m2 - m1);
		interPointY = m2 * interPointX + c2;
	}
	return (int)sqrtf((p.x - interPointX)*(p.x - interPointX) + (p.y - interPointY)* (p.y - interPointY));
}

static void _douglas_peucker(const std::vector<Vector2i> &points, int istart, int iend, int treshold, std::vector<Vector2i> *filtered) {
	if (iend - istart < 2) {
		return;
	}

	int		dmax = 0;
	int		index = 0;
	for (int i = istart + 1; i < iend; ++i) {
		int d = distanceToSegment(points[i], points[istart], points[iend]);
		if (d > dmax) {
			index = i;
			dmax = d;
		}
	}

	if (dmax > treshold) {
		_douglas_peucker(points, istart, index, treshold, filtered);
		filtered->push_back(points[index]);
		_douglas_peucker(points, index, iend, treshold, filtered);
	}
}

void SegmentsSimplification_DouglasPeucker(std::vector<Vector2i> &polygon, int treshold, std::vector<Vector2i>* filtered) {
	if (polygon.size() <= 2) {
		return;
	}
	filtered->clear();
	filtered->push_back(polygon.front());
	_douglas_peucker(polygon, 0, (int)polygon.size() - 1, treshold, filtered);
	filtered->push_back(polygon.back());
}

static float dot_product(const Vector2i&a, const Vector2i&b, const Vector2i&c)
{
	Vector2d ab = Vector2d((float)(a.x - b.x), (float)(a.y - b.y));
	Vector2d cb = Vector2d((float)(c.x - b.x), (float)(c.y - b.y));
	ab.Normalize();
	cb.Normalize();
	return ab.Dot(cb);
}

void ConcaveHullSimplification(std::vector<Vector2i> &polygon, int treshold, std::vector<Vector2i>* new_polygon)
{
	int max_i = -1;
	float max_dot = -1.0f;
	int n = (int)polygon.size();
	for (int i = 0; i < n; ++i)
	{
		float dot = dot_product(polygon[(i + n - 1) % n], polygon[i], polygon[(i + 1) % n]);
		
		if (dot > max_dot) {
			max_dot = dot;
			max_i = i;
		}
	}

	float inflection_treshold = cosf(120 * 3.14159265f / 180);

	if (max_i < 0)
	{
		return;
	}

	std::vector<Vector2i> temp, simply;

	int curr = max_i;
	int next = (curr + 1) % n;

	temp.push_back(polygon[curr]);
	while (1)
	{
		bool need_break = max_i == next;

		float dot = dot_product(polygon[curr], polygon[next], polygon[(next + 1) % n]);
		temp.push_back(polygon[next]);
		if (dot > inflection_treshold || need_break) {
			simply.clear();
			SegmentsSimplification_DouglasPeucker(temp, treshold, &simply);
			for (size_t j = 0; j < simply.size() - 1; ++j) {
				new_polygon->push_back(simply[j]);
			}
			temp.clear();

			curr = next;
			temp.push_back(polygon[curr]);
		}
		next = (next + 1) % n;

		if (need_break)
			break;
	}
	// assert(new_polygon->size() <= polygon.size());
}
