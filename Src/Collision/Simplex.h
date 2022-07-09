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
		Vector3d d, p;			// dir and position
	};
	Vertex		v[4];
	float		w[4];
	int			dimension;

public:
	Simplex()
	{
		dimension = 0;
	}

	void AddVertex(const Vector3d& vv, float ww, MinkowskiSum* Shape)
	{
		w[dimension] = ww;
		v[dimension].d = vv.Unit();
		if (Shape)
		{
			v[dimension].p = Shape->Support(v[dimension].d);
		}
		dimension++;
	}

	void RemoveVertex()
	{
		dimension--;
		assert(dimension >= 1);
	}

	Simplex& operator =(const Simplex& rhs)
	{
		memcpy(this, &rhs, sizeof(Simplex));
		return *this;
	}

	float ProjectOrigin(float* weights, int& mask)
	{
		float sqdist = -1.0f;
		switch (dimension)
		{
		case 2:
			sqdist = ProjectOrigin2(v[0].p, v[1].p, weights, mask);
			break;
		case 3:
			sqdist = ProjectOrigin3(v[0].p, v[1].p, v[2].p, weights, mask);
			break;
		case 4:
			sqdist = ProjectOrigin4(v[0].p, v[1].p, v[2].p, v[3].p, weights, mask);
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
				AddVertex(axis, 0, nullptr);
				if (EncloseOrigin())
					return true;
				RemoveVertex();
				AddVertex(-axis, 0, nullptr);
				if (EncloseOrigin())
					return true;
				RemoveVertex();
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
					AddVertex(p, 0, nullptr);
					if (EncloseOrigin())
						return true;
					RemoveVertex();
					AddVertex(-p, 0, nullptr);
					if (EncloseOrigin())
						return true;
					RemoveVertex();
				}
			}
		}
		break;
		case 3:
		{
			Vector3d n = CrossProduct(v[1].p - v[0].p, v[2].p - v[0].p);
			if (n.SquareLength() > 0)
			{
				AddVertex(n, 0, nullptr);
				if (EncloseOrigin())
					return true;
				RemoveVertex();
				AddVertex(-n, 0, nullptr);
				if (EncloseOrigin())
					return true;
				RemoveVertex();
			}
		}
		break;
		case 4:
		{
			if (fabsf(Determinant(v[0].p - v[3].p, v[1].p - v[3].p,	v[2].p - v[3].p)) > 0)
				return true;
		}
		break;
		}
		return false;
	}


private:
	static float ProjectOrigin2(Vector3d& a, Vector3d& b, float* w, int& m)
	{
		Vector3d d = b - a;
		float l = d.SquareLength();
		if (l > SIMPLEX2_EPS)
		{
			float t = l > 0 ? -DotProduct(a, d) / l : 0;
			if (t >= 1)
			{
				w[0] = 0;
				w[1] = 1;
				m = 2;
				return b.SquareLength();
			}
			else if (t <= 0)
			{
				w[0] = 1;
				w[1] = 0;
				m = 1;
				return a.SquareLength();
			}
			else
			{
				w[0] = 1 - (w[1] = t);
				m = 3;
				return (a + d * t).SquareLength();
			}
		}
		return -1;
	}

	static float ProjectOrigin3(Vector3d& a, Vector3d& b, Vector3d& c, float* w, int& m)
	{
		const int	imd3[] = { 1, 2, 0 };
		Vector3d*	vt[] = { &a, &b, &c };
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
				if (DotProduct(*vt[i], CrossProduct(dl[i], n)) > 0)
				{
					int j = imd3[i];
					float subd = ProjectOrigin2(*vt[i], *vt[j], subw, subm);
					if ((mindist < 0) || (subd < mindist))
					{
						mindist = subd;
						m = ((subm & 1) ? 1 << i : 0) +
							((subm & 2) ? 1 << j : 0);
						w[i] = subw[0];
						w[j] = subw[1];
						w[imd3[j]] = 0;
					}
				}
			}
			if (mindist < 0)
			{
				const float d = DotProduct(a, n);
				const float s = sqrtf(l);
				Vector3d p = n * (d / l);
				mindist = p.SquareLength();
				m = 7;
				w[0] = CrossProduct(dl[1], b - p).Length() / s;
				w[1] = CrossProduct(dl[2], c - p).Length() / s;
				w[2] = 1 - (w[0] + w[1]);
			}
			return mindist;
		}
		return -1;
	}

	static float ProjectOrigin4(Vector3d& a, Vector3d& b, Vector3d& c, Vector3d& d, float* w, int& m)
	{
		const int imd3[] = { 1, 2, 0 };
		Vector3d* vt[] = { &a, &b, &c, &d };
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
					float subd = ProjectOrigin3(*vt[i], *vt[j], d, subw, subm);
					if ((mindist < 0) || (subd < mindist))
					{
						mindist = subd;
						m = (subm & 1 ? 1 << i : 0) +
							(subm & 2 ? 1 << j : 0) +
							(subm & 4 ? 8 : 0);
						w[i] = subw[0];
						w[j] = subw[1];
						w[imd3[j]] = 0;
						w[3] = subw[2];
					}
				}
			}
			if (mindist < 0)
			{
				mindist = 0;
				m = 15;
				w[0] = Determinant(c, b, d) / vl;
				w[1] = Determinant(a, c, d) / vl;
				w[2] = Determinant(b, a, d) / vl;
				w[3] = 1 - (w[0] + w[1] + w[2]);
			}
			return mindist;
		}
		return -1;
	}
};
