#pragma once

#include <float.h>
#include <string.h>
#include "Shape2d.h"

#define MAX_CONVEX_VERTICES		(8)

class Convex : public Shape2d
{
public:
	Point2	Vertices[MAX_CONVEX_VERTICES];
	int		NumVertices;
	float	Rot;

	Convex(const Point2* _pv, int _n)
	{
		memcpy(Vertices, _pv, sizeof(Vertices[0]) * _n);
		NumVertices = _n;
		Rot = 0.0f;
	}

public:
	virtual Point2 Support(const Point2& Dir) const override final
	{
		Point2 DirLocal = Dir.Rotate(-Rot);
		float max_proj = -FLT_MAX;
		int   max_i = -1;
		for (int i = 0; i < NumVertices; ++i)
		{
			const Point2& pt = Vertices[i];
			float proj = (Dir.Dot(pt)) * (pt.Length());
			if (proj > max_proj)
			{
				max_proj = proj;
				max_i = i;
			}
		}
		return Vertices[max_i].Rotate(Rot);
	}
};