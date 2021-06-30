#pragma once

#include <vector>
#include <queue>

#include "../Maths/Vector2i.h"

// X 0 X
// 3 X 1 
// X 2 X
int neighbour4x_safe(int idx, int nx, int ny, int dir) {
	int y = idx / nx;
	int x = idx - y * nx;
	switch (dir) {
	case 0:
		return y < ny - 1 ? idx + nx : -1;
	case 1:
		return x < nx - 1 ? idx + 1 : -1;
	case 2:
		return y > 0 ? idx - nx : -1;
	case 3:
		return x > 0 ? idx - 1 : -1;
	}
	return -1;
}

// 7 0 1
// 6 X 2 
// 5 4 3
int neighbour8x_safe(int idx, int nx, int ny, int dir) {
	int y = idx / nx;
	int x = idx - y * nx;
	switch (dir) {
	case 0:
		return y < ny - 1 ? idx + nx : -1;
	case 1:
		return x < nx - 1 && x < nx - 1 ? idx + nx + 1 : -1;
	case 2:
		return x < nx - 1 ? idx + 1 : -1;
	case 3:
		return x < nx - 1 && y > 0 ? idx - nx + 1 : -1;
	case 4:
		return y > 0 ? idx - nx : -1;
	case 5:
		return y > 0 && x > 0 ? idx - nx - 1 : -1;
	case 6:
		return x > 0 ? idx - 1 : -1;
	case 7:
		return x > 0 && y < ny - 1 ? idx + nx - 1 : -1;
	}
	return -1;
}


template<typename T>
void copy_object(T* src, T* dst, int nx, int ny, int seed)
{
	std::queue<int> qu;

	qu.push(seed);
	dst[seed] = src[seed];
	src[seed] = 0;

	while (!qu.empty()) {
		int curr = qu.front();
		qu.pop();
		for (int d = 0; d < 4; ++d) {
			int nei = neighbour4x_safe(curr, nx, ny, d);
			if (nei < 0)
				continue;
			if (src[nei] == 0) {
				continue;
			}
			qu.push(nei);
			dst[nei] = src[nei];
			src[nei] = 0;
		}
	}
}


template<typename T>
void FillObject4x(T* cell, int nx, int ny, int seed, T value)
{
	std::queue<int> qu;

	qu.push(seed);
	cell[seed] = value;

	while (!qu.empty()) {
		int curr = qu.front();
		qu.pop();
		for (int d = 0; d < 4; ++d) {
			int nei = neighbour4x_safe(curr, nx, ny, d);
			if (nei < 0)
				continue;
			if (cell[nei] == 0) {
				continue;
			}
			qu.push(nei);
			cell[nei] = value;
		}
	}
}


template<typename T>
void EdgeDetection(T* cell, int nx, int ny, int seed, std::vector<Vector2i>* polygon)
{
	polygon->push_back(Vector2i(seed / nx, seed - (seed / nx) * nx));

	int curr = seed, dir = 1;
	while (1)
	{
		for (int i = 1; i >= -2; --i)
		{
			int d = (dir + i + 4) % 4;
			int n = neighbour4x_safe(curr, nx, ny, d);
			if (cell[n] == 0)
				continue;
			curr = n;
			polygon->push_back(Vector2i(curr / nx, curr - (curr / nx) * nx));
			dir = d;
			cell[curr] = 1;
			break;
		}
		if (curr == seed)
			return;
	}
}


template<typename T>
void ExtractObjects(T* cell, int nx, int ny, std::vector<std::vector<Vector2i>>* polygons, const int MinPolygonPoints)
{
	polygons->clear();

	for (int i = 1; i < ny - 1; ++i)
	for (int j = 1; j < nx - 1; ++j)
	{
		const int k = i * nx + j;
		if (cell[k] == 0)
			continue;
		std::vector<Vector2i>	polygon;
		EdgeDetection(cell, nx, ny, k, &polygon);
		if (polygon.size() > MinPolygonPoints) {
			polygons->push_back(polygon);
		}
		FillObject4x<unsigned char>(cell, nx, ny, k, 0);
	}
}

void PolygonSimplification_VisvalingamWhyatt(std::vector<Vector2i>& polygon, int treshold);

void SegmentsSimplification_DouglasPeucker(std::vector<Vector2i>& segments, int treshold, std::vector<Vector2i>* filtered);

void ConcaveHullSimplification(std::vector<Vector2i>& polygon, int treshold, std::vector<Vector2i>* new_polygon);