#pragma once

#include <vector>

template<typename T>
bool RasterizeLineBresenham(T* cell, int nx, int x1, int y1, int x2, int y2, T value) {
	int F, x, y;
	if (x1 > x2) {
		std::swap(x1, x2);
		std::swap(y1, y2);
	}

	if (x1 == x2) {
		if (y1 > y2)
			std::swap(y1, y2);

		x = x1, y = y1;
		while (y <= y2) {
			cell[y * nx + x] = value;
			++y;
		}
		return true;
	}
	else if (y1 == y2) {
		x = x1, y = y1;
		while (x <= x2) {
			cell[y * nx + x] = value;
			++x;
		}
		return true;
	}

	int dy = y2 - y1;
	int dx = x2 - x1;
	int dy2 = (dy << 1);
	int dx2 = (dx << 1);
	int dy2_minus_dx2 = dy2 - dx2;
	int dy2_plus_dx2 = dy2 + dx2;

	if (dy >= 0) {
		if (dy <= dx) {
			F = dy2 - dx;
			x = x1, y = y1;
			while (x <= x2) {
				cell[y * nx + x] = value;
				if (F <= 0) {
					F += dy2;
				}
				else {
					++y;
					F += dy2_minus_dx2;
				}
				++x;
			}
		}
		else {
			F = dx2 - dy;
			x = x1, y = y1;
			while (y <= y2) {
				cell[y * nx + x] = value;
				if (F <= 0) {
					F += dx2;
				}
				else {
					++x;
					F -= dy2_minus_dx2;
				}
				++y;
			}
		}
	}
	else {
		if (dx >= -dy) {
			F = -dy2 - dx;
			x = x1, y = y1;
			while (x <= x2) {
				cell[y * nx + x] = value;
				if (F <= 0) {
					F -= dy2;
				}
				else {
					--y;
					F -= dy2_plus_dx2;
				}
				++x;
			}
		}
		else {
			F = dx2 + dy;
			x = x1, y = y1;
			while (y >= y2) {
				cell[y * nx + x] = value;
				if (F <= 0) {
					F += dx2;
				}
				else {
					++x;
					F += dy2_plus_dx2;
				}
				--y;
			}
		}
	}
	return true;
}

template<typename T>
void RasterizePolygons(T* cell, int nx, int ny, const std::vector<std::vector<Vector2i>>& polygons, T Value)
{
	for (size_t i = 0; i < polygons.size(); ++i)
	for (size_t j = 0; j < polygons[i].size(); ++j)
	{
		int k = (j + 1) % polygons[i].size();
		int x1 = polygons[i][j].x;
		int y1 = polygons[i][j].y;
		int x2 = polygons[i][k].x;
		int y2 = polygons[i][k].y;
		RasterizeLineBresenham(cell, nx, x1, y1, x2, y2, Value);
	}
}
