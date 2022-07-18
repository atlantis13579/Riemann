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
	float		w[4];				// arycentric coordinate
	int			dimension;

public:
	Simplex()
	{
		dimension = 0;
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

	Vector3d UpdatePointSet(float* weights, int mask)
	{
		Vector3d newDir = Vector3d::Zero();
		int new_dimension = 0;
		for (int i = 0; i < dimension; ++i)
		{
			if (mask & (1 << i))
			{
				v[new_dimension] = v[i];
				w[new_dimension++] = weights[i];
				newDir = newDir + v[i].p * weights[i];
			}
		}
		dimension = new_dimension;
		return newDir;
	}

	float ProjectOrigin(float* coords, int& mask)
	{
		float sqdist = -1.0f;
		switch (dimension)
		{
		case 2:
			sqdist = ProjectOriginSegment(v[0].p, v[1].p, coords, mask);
			break;
		case 3:
			sqdist = ProjectOriginTriangle(v[0].p, v[1].p, v[2].p, coords, mask);
			break;
		case 4:
			sqdist = ProjectOriginTetrahedral(v[0].p, v[1].p, v[2].p, v[3].p, coords, mask);
			break;
		default:
			assert(false);
			break;
		}
		return sqdist;
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
		}
		break;
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
		}
		break;
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
		}
		break;
		case 4:
		{
			const float det = Determinant(v[0].p - v[3].p, v[1].p - v[3].p, v[2].p - v[3].p);
			if (fabsf(det) > 0)
				return true;
		}
		break;
		}
		return false;
	}


private:
	static float ProjectOriginSegment(const Vector3d& a, const Vector3d& b, float* coords, int& mask)
	{
		Vector3d d = b - a;
		float l = d.SquareLength();
		if (l > SIMPLEX2_EPS)
		{
			float t = -DotProduct(a, d) / l;
			if (t >= 1.0f)
			{
				coords[0] = 0.0f;
				coords[1] = 1.0f;
				mask = 2;
				return b.SquareLength();
			}
			else if (t <= 0.0f)
			{
				coords[0] = 1.0f;
				coords[1] = 0.0f;
				mask = 1;
				return a.SquareLength();
			}
			else
			{
				coords[1] = t;
				coords[0] = 1.0f - t;
				mask = 3;
				return (a + d * t).SquareLength();
			}
		}
		return -1;
	}

	static float ProjectOriginTriangle(const Vector3d& a, const Vector3d& b, const Vector3d& c, float* coords, int& mask)
	{
		const int	imd3[] = { 1, 2, 0 };
		Vector3d	v[] = { a, b, c };
		Vector3d	dl[] = { a - b, b - c, c - a };
		Vector3d	n = CrossProduct(dl[0], dl[1]);

		const float l = n.SquareLength();
		if (l > SIMPLEX3_EPS)
		{
			float mindist = -1;
			float subw[2] = { 0.f, 0.f };
			int subm(0);
			for (int i = 0; i < 3; ++i)
			{
				Vector3d ns = CrossProduct(dl[i], n);
				float dp = DotProduct(v[i], ns);
				if (dp > 0)
				{
					int j = imd3[i];
					float subd = ProjectOriginSegment(v[i], v[j], subw, subm);
					if ((mindist < 0) || (subd < mindist))
					{
						mindist = subd;
						mask = ((subm & 1) ? 1 << i : 0) +
							((subm & 2) ? 1 << j : 0);
						coords[i] = subw[0];
						coords[j] = subw[1];
						coords[imd3[j]] = 0;
					}
				}
			}
			if (mindist < 0)
			{
				const float d = DotProduct(a, n);
				const float s = sqrtf(l);
				Vector3d p = n * (d / l);
				mindist = p.SquareLength();
				mask = 7;
				coords[0] = CrossProduct(dl[1], b - p).Length() / s;
				coords[1] = CrossProduct(dl[2], c - p).Length() / s;
				coords[2] = 1 - (coords[0] + coords[1]);
			}
			return mindist;
		}
		return -1;
	}

	static float ProjectOriginTetrahedral(const Vector3d& a, const Vector3d& b, const Vector3d& c, const Vector3d& d, float* weights, int& mask)
	{
		const int imd3[] = { 1, 2, 0 };
		const Vector3d* vt[] = { &a, &b, &c, &d };
		Vector3d dl[] = { a - d, b - d, c - d };
		float vl = Determinant(dl[0], dl[1], dl[2]);
		bool ng = (vl * DotProduct(a, CrossProduct(b - c, a - b))) <= 0;
		if (ng && (fabsf(vl) > SIMPLEX4_EPS))
		{
			float mindist = -1;
			float subw[3] = { 0.f, 0.f, 0.f };
			int subm(0);
			for (int i = 0; i < 3; ++i)
			{
				int j = imd3[i];
				float s = vl * DotProduct(d, CrossProduct(dl[i], dl[j]));
				if (s > 0)
				{
					float subd = ProjectOriginTriangle(*vt[i], *vt[j], d, subw, subm);
					if ((mindist < 0) || (subd < mindist))
					{
						mindist = subd;
						mask = (subm & 1 ? 1 << i : 0) +
							(subm & 2 ? 1 << j : 0) +
							(subm & 4 ? 8 : 0);
						weights[i] = subw[0];
						weights[j] = subw[1];
						weights[imd3[j]] = 0;
						weights[3] = subw[2];
					}
				}
			}
			if (mindist < 0)
			{
				mindist = 0;
				mask = 15;
				weights[0] = Determinant(c, b, d) / vl;
				weights[1] = Determinant(a, c, d) / vl;
				weights[2] = Determinant(b, a, d) / vl;
				weights[3] = 1 - (weights[0] + weights[1] + weights[2]);
			}
			return mindist;
		}
		return -1;
	}
};
