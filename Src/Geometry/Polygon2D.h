#pragma once

#include <vector>
#include "../Maths/Vector2d.h"

Vector2d ClosestPointInPolygon(const Vector2d& pt, const Vector2d* polygon, int nvert, bool* inside);

bool IsPointInPolygon(const Vector2d& pt, const Vector2d* polygon, int nvert);