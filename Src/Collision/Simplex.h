#pragma once

#include <assert.h>
#include "../Maths/Vector3d.h"
#include "MinkowskiSum.h"

#define SIMPLEX2_EPS (0.0f)
#define SIMPLEX3_EPS (0.0f)
#define SIMPLEX4_EPS (0.0f)

class Simplex
{
public:
	struct Vertex
	{
		Vector3d	d, p;			// dir and position
	};
	Vertex		v[4];
	float		w[4];				// barycentric coordinate
	int			dimension;

public:
	Simplex()
	{
		dimension = 0;
	}

	const Vector3d& LastPoint() const
	{
		return v[dimension - 1].p;
	}

	void AddPoint(const Vector3d& dir, float ww, MinkowskiSum* Shape)
	{
		w[dimension] = ww;
		v[dimension].d = dir.Unit();
		if (Shape)
		{
			v[dimension].p = Shape->Support(v[dimension].d);
		}
		dimension++;
	}

	void RemovePoint()
	{
		dimension--;
		assert(dimension >= 1);
	}

	Simplex& operator =(const Simplex& rhs)
	{
		memcpy(this, &rhs, sizeof(Simplex));
		return *this;
	}

	void UpdatePointSet(int mask)
	{
		int new_dimension = 0;
		for (int i = 0; i < dimension; ++i)
		{
			if (mask & (1 << i))
			{
				v[new_dimension] = v[i];
				new_dimension++;
			}
		}
		dimension = new_dimension;
		return;
	}

	bool ProjectOrigin(Vector3d& pos, int& mask)
	{
		float sqdist = -1.0f;
		switch (dimension)
		{
		case 2:
			return ProjectOriginSegment(v[0].p, v[1].p, pos, mask);
		case 3:
			return ProjectOriginTriangle(v[0].p, v[1].p, v[2].p, pos, mask);
		case 4:
			return ProjectOriginTetrahedral(v[0].p, v[1].p, v[2].p, v[3].p, pos, mask);
		default:
			assert(false);
			break;
		}
		return false;
	}

	bool EncloseOrigin()
	{
		switch (dimension)
		{
		case 1:
		{
			for (int i = 0; i < 3; ++i)
			{
				Vector3d axis = Vector3d(0, 0, 0);
				axis[i] = 1;
				AddPoint(axis, 0, nullptr);
				if (EncloseOrigin())
					return true;
				RemovePoint();
				AddPoint(-axis, 0, nullptr);
				if (EncloseOrigin())
					return true;
				RemovePoint();
			}
			break;
		}
		case 2:
		{
			const Vector3d d = v[1].p - v[0].p;
			for (int i = 0; i < 3; ++i)
			{
				Vector3d axis = Vector3d(0, 0, 0);
				axis[i] = 1;
				Vector3d p = CrossProduct(d, axis);
				if (p.SquareLength() > 0)
				{
					AddPoint(p, 0, nullptr);
					if (EncloseOrigin())
						return true;
					RemovePoint();
					AddPoint(-p, 0, nullptr);
					if (EncloseOrigin())
						return true;
					RemovePoint();
				}
			}
			break;
		}
		case 3:
		{
			Vector3d n = CrossProduct(v[1].p - v[0].p, v[2].p - v[0].p);
			if (n.SquareLength() > 0)
			{
				AddPoint(n, 0, nullptr);
				if (EncloseOrigin())
					return true;
				RemovePoint();
				AddPoint(-n, 0, nullptr);
				if (EncloseOrigin())
					return true;
				RemovePoint();
			}
			break;
		}
		case 4:
		{
			const float det = Determinant(v[0].p - v[3].p, v[1].p - v[3].p, v[2].p - v[3].p);
			if (fabsf(det) > 0)
				return true;
			break;
		}
		}
		return false;
	}


private:
	static bool ProjectOriginSegment(const Vector3d& a, const Vector3d& b, Vector3d& pos, int& mask)
	{
		Vector3d d = b - a;
		float l = d.SquareLength();
		if (l > SIMPLEX2_EPS)
		{
			float t = -DotProduct(a, d) / l;
			if (t >= 1.0f)
			{
				pos = b;
				mask = 0b0010;
			}
			else if (t <= 0.0f)
			{
				pos = a;
				mask = 0b0001;
			}
			else
			{
				pos = a + d * t;
				mask = 0b0011;
			}
			return true;
		}
		return false;
	}

	static bool ProjectOriginTriangle(const Vector3d& a, const Vector3d& b, const Vector3d& c, Vector3d& pos, int& mask)
	{
		Vector3d	v[] = { a, b, c };
		Vector3d	dl[] = { a - b, b - c, c - a };
		Vector3d	n = CrossProduct(dl[0], dl[1]);

		const float l = n.SquareLength();
		if (l > SIMPLEX3_EPS)
		{
			float mindist = FLT_MAX;
			Vector3d subpos;
			int submask = 0;
			for (int i = 0; i < 3; ++i)
			{
				Vector3d ns = CrossProduct(dl[i], n);
				float dp = DotProduct(v[i], ns);
				if (dp > 0)
				{
					int j = (i + 1) % 3;
					float dist = ProjectOriginSegment(v[i], v[j], subpos, submask);
					if (dist < mindist)
					{
						mindist = dist;
						mask = ((submask & 1) ? 1 << i : 0) + ((submask & 2) ? 1 << j : 0);
						pos = subpos;
					}
				}
			}
			if (mindist == FLT_MAX)
			{
				const float d = DotProduct(a, n);
				const float s = sqrtf(l);
				Vector3d p = n * (d / l);
				mindist = p.SquareLength();
				pos = sqrtf(mindist) * n;
				mask = 0b0111;
			}
			return true;
		}
		return false;
	}

	static bool ProjectOriginTetrahedral(const Vector3d& a, const Vector3d& b, const Vector3d& c, const Vector3d& d, Vector3d& pos, int& mask)
	{
		const Vector3d* vt[] = { &a, &b, &c, &d };
		Vector3d dl[] = { a - d, b - d, c - d };
		float vl = Determinant(dl[0], dl[1], dl[2]);
		bool ng = (vl * DotProduct(a, CrossProduct(b - c, a - b))) <= 0;
		if (ng && (fabsf(vl) > SIMPLEX4_EPS))
		{
			float mindist = FLT_MAX;
			Vector3d subpos;
			int submask = 0;
			for (int i = 0; i < 3; ++i)
			{
				int j = (i + 1) % 3;
				float s = vl * DotProduct(d, CrossProduct(dl[i], dl[j]));
				if (s > 0)
				{
					float dist = ProjectOriginTriangle(*vt[i], *vt[j], d, subpos, submask);
					if (dist < mindist)
					{
						mindist = dist;
						pos = subpos;
						mask = (submask & 1 ? 1 << i : 0) + (submask & 2 ? 1 << j : 0) + (submask & 4 ? 8 : 0);
					}
				}
			}
			if (mindist == FLT_MAX)
			{
				mindist = 0;
				pos = Vector3d::Zero();
				mask = 0b1111;
			}
			return true;
		}
		return false;
	}
};
