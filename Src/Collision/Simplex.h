#pragma once

#include <assert.h>
#include <float.h>
#include "../Maths/Vector3.h"
#include "MinkowskiSum.h"

#define SIMPLEX1_EPS (1e-9f)
#define SIMPLEX2_EPS (1e-9f)
#define SIMPLEX3_EPS (1e-9f)
#define SIMPLEX4_EPS (1e-9f)

class Simplex
{
public:
	struct Vertex
	{
		Vector3	dir, pos;		// dir and position
	};
	Vertex			v[4];
	float			w[4];			// barycentric coordinate
	int				dimension;
	MinkowskiSum*	m_shape;

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

	Simplex& operator=(const Simplex& rhs)
	{
		memcpy((void*)this, &rhs, sizeof(Simplex));
		return *this;
	}

	const Vector3& LastPoint() const
	{
		return v[dimension - 1].pos;
	}

	void AddPoint(const Vector3& dir)
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

	bool ProjectOrigin(Vector3& proj, int& mask)
	{
		switch (dimension)
		{
		case 4:
			return ProjectOriginToTetrahedral(v[0].pos, v[1].pos, v[2].pos, v[3].pos, proj, mask);
		case 3:
			return ProjectOriginToTriangle(v[0].pos, v[1].pos, v[2].pos, proj, mask);
		case 2:
			return ProjectOriginToSegment(v[0].pos, v[1].pos, proj, mask);
		case 1:
			return ProjectOriginToPoint(v[0].pos, proj, mask);
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
				Vector3 axis = Vector3(0, 0, 0);
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
			const Vector3 d = v[1].pos - v[0].pos;
			for (int i = 0; i < 3; ++i)
			{
				Vector3 axis = Vector3(0, 0, 0);
				axis[i] = 1;
				Vector3 p = CrossProduct(d, axis);
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
			Vector3 n = CrossProduct(v[1].pos - v[0].pos, v[2].pos - v[0].pos);
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
			const float det = ScalerTripleProduct(v[0].pos - v[3].pos, v[1].pos - v[3].pos, v[2].pos - v[3].pos);
			if (fabsf(det) > 0)
				return true;
			break;
		}
		}
		return false;
	}

private:
	static bool ProjectOriginToPoint(const Vector3& a, Vector3& proj, int& mask)
	{
		float l = a.SquareLength();
		if (l <= SIMPLEX1_EPS)
		{
			return false;
		}

		proj = a;
		mask = 0b0001;
		return true;
	}

	static bool ProjectOriginToSegment(const Vector3& a, const Vector3& b, Vector3& proj, int& mask)
	{
		Vector3 d = b - a;
		float l = d.SquareLength();
		if (l <= SIMPLEX2_EPS)
		{
			return false;
		}
	
		float t = -DotProduct(a, d) / l;
		if (t >= 1.0f)
		{
			proj = b;
			mask = 0b0010;
		}
		else if (t <= 0.0f)
		{
			proj = a;
			mask = 0b0001;
		}
		else
		{
			proj = a + d * t;
			mask = 0b0011;
		}
		return true;
	}

	static bool ProjectOriginToTriangle(const Vector3& a, const Vector3& b, const Vector3& c, Vector3& proj, int& mask)
	{
		Vector3	v[] = { a, b, c };
		Vector3	dl[] = { a - b, b - c, c - a };
		Vector3	n = CrossProduct(dl[0], dl[1]);

		const float l = n.SquareLength();
		if (l <= SIMPLEX3_EPS)
		{
			return false;
		}

		Vector3 subpos;
		int submask = 0;
		float min_sqr_dist = FLT_MAX;
		for (int i = 0; i < 3; ++i)
		{
			Vector3 ns = CrossProduct(dl[i], n);
			float dp = DotProduct(v[i], ns);
			if (dp > 0)
			{
				int j = (i + 1) % 3;
				if (ProjectOriginToSegment(v[i], v[j], subpos, submask) && subpos.SquareLength() < min_sqr_dist)
				{
					min_sqr_dist = subpos.SquareLength();
					proj = subpos;
					mask = ((submask & 1) ? 1 << i : 0) + ((submask & 2) ? 1 << j : 0);
				}
			}
		}
		if (min_sqr_dist == FLT_MAX)
		{
			float signedProjLen = DotProduct(a, n) / l;
			proj = n * signedProjLen;
			mask = 0b0111;
		}
		return true;
	}

	static bool ProjectOriginToTetrahedral(const Vector3& a, const Vector3& b, const Vector3& c, const Vector3& d, Vector3& proj, int& mask)
	{
		const Vector3* vt[] = { &a, &b, &c, &d };
		Vector3 dl[] = { a - d, b - d, c - d };
		Vector3 n = CrossProduct(b - c, a - b);
		float vl = ScalerTripleProduct(dl[0], dl[1], dl[2]);
		float dp = DotProduct(a, n);
		if (vl * dp > 0 || fabsf(vl) <= SIMPLEX4_EPS)
		{
			return false;
		}

		Vector3 subpos;
		int submask = 0;
		float min_sqr_dist = FLT_MAX;
		for (int i = 0; i < 3; ++i)
		{
			int j = (i + 1) % 3;
			float s = vl * DotProduct(d, CrossProduct(dl[i], dl[j]));
			if (s > 0)
			{
				if (ProjectOriginToTriangle(*vt[i], *vt[j], d, subpos, submask) && subpos.SquareLength() < min_sqr_dist)
				{
					min_sqr_dist = subpos.SquareLength();
					proj = subpos;
					mask = ((submask & 1) ? 1 << i : 0) + ((submask & 2) ? 1 << j : 0) + ((submask & 4) ? 8 : 0);
				}
			}
		}
		if (min_sqr_dist == FLT_MAX)
		{
			proj = Vector3::Zero();
			mask = 0b1111;
		}
		return true;
	}
};
