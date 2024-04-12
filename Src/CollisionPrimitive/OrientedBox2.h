#pragma once

#include "../Maths/Vector2.h"

class OrientedBox2
{
public:
	Vector2	Center;
	Vector2	HalfExtent;
	float	Rot;

	OrientedBox2(const Vector2& _c, const Vector2& _h, float _rot)
	{
		Center = _c;
		HalfExtent = _h;
		Rot = _rot;
	}

public:
	Vector2 Support(const Vector2& Dir) const
	{
		Vector2 DirLocal = Dir.Rotate(-Rot);
		Vector2 SupportLocal = Vector2(
			DirLocal.x > 0 ? Center.x + HalfExtent.x : Center.x - HalfExtent.x,
			DirLocal.y > 0 ? Center.y + HalfExtent.y : Center.y - HalfExtent.y
		);
		return SupportLocal.Rotate(Rot);
	}
};