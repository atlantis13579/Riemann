#pragma once

#include "../Maths/Vector2.h"

Vector2 ClosestPointInPolygon2D(const Vector2& pt, const Vector2* polygon, int nvert, bool* inside);

bool PointInPolygon2D(const Vector2& pt, const Vector2* polygon, int nvert);

float PolygonArea2D(const Vector2* polygon, int nvert);