#pragma once

#include "../Maths/Vector2d.h"

Vector2d ClosestPointInPolygon2D(const Vector2d& pt, const Vector2d* polygon, int nvert, bool* inside);

bool PointInPolygon2D(const Vector2d& pt, const Vector2d* polygon, int nvert);

float PolygonArea2D(const Vector2d* polygon, int nvert);