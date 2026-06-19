#pragma once

#include "../Maths/Vector2.h"

#define MAX_CONVEX_VERTICES		(8)

class Convex2
{
public:
	Vector2	Vertices[MAX_CONVEX_VERTICES];
	int		NumVertices;
	float	Rot;

	Convex2(const Vector2* _pv, const int _n)
	{
		memcpy(Vertices, _pv, sizeof(Vertices[0]) * _n);
		NumVertices = _n;
		Rot = 0.0f;
	}

	Vector2 Support(const Vector2& Dir) const
	{
		float max_proj = -FLT_MAX;
		int   max_i = 0;
		for (int i = 0; i < NumVertices; ++i)
		{
			const Vector2& pt = Vertices[i];
			float proj = (Dir.Dot(pt)) * (pt.Length());
			if (proj >= max_proj)
			{
				max_proj = proj;
				max_i = i;
			}
		}
		return Vertices[max_i].Rotate(Rot);
	}
};