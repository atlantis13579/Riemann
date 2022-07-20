#pragma once

#include <assert.h>
#include <float.h>
#include "../Maths/Vector3d.h"
#include "MinkowskiSum.h"

#define SIMPLEX2_EPS (1e-9f)
#define SIMPLEX3_EPS (1e-9f)
#define SIMPLEX4_EPS (1e-9f)

class Simplex
{
public:
	struct Vertex
	{
		Vector3d	dir, pos;		// dir and position
	};
	Vertex		v[4];
	float		w[4];				// barycentric coordinate
	int			dimension;
	MinkowskiSum* m_shape;

public:
	Simplex()
	{
		Init(nullptr);
	}

	void Init(MinkowskiSum* _shape)
	{
		m_shape = _shape;
		dimension = 0;
	}

	const Vector3d& LastPoint() const
	{
		return v[dimension - 1].pos;
	}

	void AddPoint(const Vector3d& dir)
	{
		v[dimension].dir = dir.Unit();
		v[dimension].pos = m_shape->Support(v[dimension].dir);
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
		switch (dimension)
		{
		case 2:
			return ProjectOriginSegment(v[0].pos, v[1].pos, pos, mask);
		case 3:
			return ProjectOriginTriangle(v[0].pos, v[1].pos, v[2].pos, pos, mask);
		case 4:
			return ProjectOriginTetrahedral(v[0].pos, v[1].pos, v[2].pos, v[3].pos, pos, mask);
		default:
			assert(false);
			break;
		}
		return false;
	}

	Vector3d Support(const Vector3d& Dir) const
	{
		return m_shape->Support(Dir);
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
				AddPoint(axis);
				if (EncloseOrigin())
					return true;
				RemovePoint();
				AddPoint(-axis);
				if (EncloseOrigin())
					return true;
				RemovePoint();
			}
			break;
		}
		case 2:
		{
			const Vector3d d = v[1].pos - v[0].pos;
			for (int i = 0; i < 3; ++i)
			{
				Vector3d axis = Vector3d(0, 0, 0);
				axis[i] = 1;
				Vector3d p = CrossProduct(d, axis);
				if (p.SquareLength() > 0)
				{
					AddPoint(p);
					if (EncloseOrigin())
						return true;
					RemovePoint();
					AddPoint(-p);
					if (EncloseOrigin())
						return true;
					RemovePoint();
				}
			}
			break;
		}
		case 3:
		{
			Vector3d n = CrossProduct(v[1].pos - v[0].pos, v[2].pos - v[0].pos);
			if (n.SquareLength() > 0)
			{
				AddPoint(n);
				if (EncloseOrigin())
					return true;
				RemovePoint();
				AddPoint(-n);
				if (EncloseOrigin())
					return true;
				RemovePoint();
			}
			break;
		}
		case 4:
		{
			const float det = Determinant(v[0].pos - v[3].pos, v[1].pos - v[3].pos, v[2].pos - v[3].pos);
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
		if (l <= SIMPLEX2_EPS)
		{
			return false;
		}
	
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

	static bool ProjectOriginTriangle(const Vector3d& a, const Vector3d& b, const Vector3d& c, Vector3d& pos, int& mask)
	{
		Vector3d	v[] = { a, b, c };
		Vector3d	dl[] = { a - b, b - c, c - a };
		Vector3d	n = CrossProduct(dl[0], dl[1]);

		const float l = n.SquareLength();
		if (l <= SIMPLEX3_EPS)
		{
			return false;
		}

		Vector3d subpos;
		int submask = 0;
		float min_sqr_dist = FLT_MAX;
		for (int i = 0; i < 3; ++i)
		{
			Vector3d ns = CrossProduct(dl[i], n);
			float dp = DotProduct(v[i], ns);
			if (dp > 0)
			{
				int j = (i + 1) % 3;
				if (ProjectOriginSegment(v[i], v[j], subpos, submask) && subpos.SquareLength() < min_sqr_dist)
				{
					min_sqr_dist = subpos.SquareLength();
					pos = subpos;
					mask = ((submask & 1) ? 1 << i : 0) + ((submask & 2) ? 1 << j : 0);
				}
			}
		}
		if (min_sqr_dist == FLT_MAX)
		{
			float signedLen = DotProduct(a, n) / l;
			pos = n * signedLen;
			mask = 0b0111;
		}
		return true;
	}

	static bool ProjectOriginTetrahedral(const Vector3d& a, const Vector3d& b, const Vector3d& c, const Vector3d& d, Vector3d& pos, int& mask)
	{
		const Vector3d* vt[] = { &a, &b, &c, &d };
		Vector3d dl[] = { a - d, b - d, c - d };
		Vector3d n = CrossProduct(b - c, a - b);
		float vl = Determinant(dl[0], dl[1], dl[2]);
		float dp = DotProduct(a, n);
		if (vl * dp > 0 || fabsf(vl) <= SIMPLEX4_EPS)
		{
			return false;
		}

		Vector3d subpos;
		int submask = 0;
		float min_sqr_dist = FLT_MAX;
		for (int i = 0; i < 3; ++i)
		{
			int j = (i + 1) % 3;
			float s = vl * DotProduct(d, CrossProduct(dl[i], dl[j]));
			if (s > 0)
			{
				if (ProjectOriginTriangle(*vt[i], *vt[j], d, subpos, submask) && subpos.SquareLength() < min_sqr_dist)
				{
					min_sqr_dist = subpos.SquareLength();
					pos = subpos;
					mask = ((submask & 1) ? 1 << i : 0) + ((submask & 2) ? 1 << j : 0) + ((submask & 4) ? 8 : 0);
				}
			}
		}
		if (min_sqr_dist == FLT_MAX)
		{
			pos = Vector3d::Zero();
			mask = 0b1111;
		}
		return true;
	}
};
