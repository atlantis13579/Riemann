#pragma once

#include "Minkowski.h"
#include "../Maths/Vector3d.h"

// Gilbert–Johnson–Keerthi algorithm
// http://allenchou.net/2013/12/game-physics-collision-detection-gjk/

enum class  GJK_status
{
	Valid,
	Inside,
	Failed
};

#define GJK_MAX_ITERATIONS 128
#define GJK_ACCURACY ((float)0.0001)
#define GJK_MIN_DISTANCE ((float)0.0001)
#define GJK_DUPLICATED_EPS ((float)0.0001)
#define GJK_SIMPLEX2_EPS ((float)0.0)
#define GJK_SIMPLEX3_EPS ((float)0.0)
#define GJK_SIMPLEX4_EPS ((float)0.0)

class GJK
{
public:
	struct Vertex
	{
		Vector3d d, w;
	};
	struct Simplex
	{
		Vertex*		c[4];
		float		p[4];
		int			dimension;
	};

	MinkowskiSum*		m_Shape;
	Simplex*		m_Simplex;
	Simplex			m_Simplices[2];
	Vertex			m_store[4];
	Vertex*			m_free[4];
	int				m_nfree;
	int				m_current;

	GJK()
	{
		memset(this, 0, sizeof(GJK));
	}

	GJK_status Evaluate(MinkowskiSum* shape, const Vector3d& Guess)
	{
		m_Shape = shape;

		m_free[0] = &m_store[0];
		m_free[1] = &m_store[1];
		m_free[2] = &m_store[2];
		m_free[3] = &m_store[3];
		m_nfree = 4;
		m_current = 0;

		m_Simplices[0].dimension = 0;
		Vector3d Ray = Guess;
		float sqrl = Ray.SquareLength();
		AddVertice(m_Simplices[0], sqrl > 0 ? Ray * -1 : Vector3d(1, 0, 0));
		m_Simplices[0].p[0] = 1;
		Ray = m_Simplices[0].c[0]->w;
		float sqdist = sqrl;

		GJK_status status = GJK_status::Valid;
		int clastw = 0;
		Vector3d lastw[4] = { Ray, Ray, Ray, Ray };

		float Alpha = 0.0f;
		int iter = 0;
		do
		{
			const int next = 1 - m_current;
			Simplex& cs = m_Simplices[m_current];
			Simplex& ns = m_Simplices[next];

			float rl = Ray.Length();
			if (rl < GJK_MIN_DISTANCE)
			{
				status = GJK_status::Inside;
				break;
			}

			AddVertice(cs, Ray * -1);
			const Vector3d& w = cs.c[cs.dimension - 1]->w;
			bool found = false;
			for (int i = 0; i < 4; ++i)
			{
				Vector3d diff;
				diff = w - lastw[i];
				if (diff.SquareLength() < GJK_DUPLICATED_EPS)
				{
					found = true;
					break;
				}
			}
			if (found)
			{
				RemoVevertice(m_Simplices[m_current]);
				break;
			}
			else
			{
				lastw[clastw = (clastw + 1) & 3] = w;
			}

			float Omega = DotProduct(Ray, w) / rl;
			Alpha = Omega > Alpha ? Omega : Alpha;
			if (((rl - Alpha) - (GJK_ACCURACY * rl)) <= 0)
			{
				RemoVevertice(m_Simplices[m_current]);
				break;
			}

			float weights[4];
			int mask = 0;
			switch (cs.dimension)
			{
			case 2:
				sqdist = ProjectOriginDim2(cs.c[0]->w, cs.c[1]->w, weights, mask);
				break;
			case 3:
				sqdist = ProjectOriginDim3(cs.c[0]->w, cs.c[1]->w, cs.c[2]->w, weights, mask);
				break;
			case 4:
				sqdist = ProjectOriginDim4(cs.c[0]->w, cs.c[1]->w, cs.c[2]->w, cs.c[3]->w, weights, mask);
				break;
			}
			if (sqdist >= 0)
			{
				ns.dimension = 0;
				Ray = Vector3d(0, 0, 0);
				m_current = next;
				for (unsigned int i = 0, ni = cs.dimension; i < ni; ++i)
				{
					if (mask & (1 << i))
					{
						ns.c[ns.dimension] = cs.c[i];
						ns.p[ns.dimension++] = weights[i];
						Ray = Ray + cs.c[i]->w * weights[i];
					}
					else
					{
						m_free[m_nfree++] = cs.c[i];
					}
				}
				if (mask == 15) status = GJK_status::Inside;
			}
			else
			{
				RemoVevertice(m_Simplices[m_current]);
				break;
			}
			status = ((++iter) < GJK_MAX_ITERATIONS) ? status : GJK_status::Failed;
		} while (status == GJK_status::Valid);

		m_Simplex = &m_Simplices[m_current];

		return status;
	}

	bool EncloseOrigin()
	{
		switch (m_Simplex->dimension)
		{
		case 1:
		{
			for (unsigned int i = 0; i < 3; ++i)
			{
				Vector3d axis = Vector3d(0, 0, 0);
				axis[i] = 1;
				AddVertice(*m_Simplex, axis);
				if (EncloseOrigin())
					return true;
				RemoVevertice(*m_Simplex);
				AddVertice(*m_Simplex, axis * -1);
				if (EncloseOrigin())
					return true;
				RemoVevertice(*m_Simplex);
			}
		}
		break;
		case 2:
		{
			const Vector3d d = m_Simplex->c[1]->w - m_Simplex->c[0]->w;
			for (unsigned int i = 0; i < 3; ++i)
			{
				Vector3d axis = Vector3d(0, 0, 0);
				axis[i] = 1;
				Vector3d p = CrossProduct(d, axis);
				if (p.SquareLength() > 0)
				{
					AddVertice(*m_Simplex, p);
					if (EncloseOrigin())
						return true;
					RemoVevertice(*m_Simplex);
					AddVertice(*m_Simplex, p * -1);
					if (EncloseOrigin())
						return true;
					RemoVevertice(*m_Simplex);
				}
			}
		}
		break;
		case 3:
		{
			Vector3d n = CrossProduct(m_Simplex->c[1]->w - m_Simplex->c[0]->w,
				m_Simplex->c[2]->w - m_Simplex->c[0]->w);
			if (n.SquareLength() > 0)
			{
				AddVertice(*m_Simplex, n);
				if (EncloseOrigin())
					return true;
				RemoVevertice(*m_Simplex);
				AddVertice(*m_Simplex, n * -1);
				if (EncloseOrigin())
					return true;
				RemoVevertice(*m_Simplex);
			}
		}
		break;
		case 4:
		{
			if (fabsf(Determinant(	m_Simplex->c[0]->w - m_Simplex->c[3]->w,
									m_Simplex->c[1]->w - m_Simplex->c[3]->w,
									m_Simplex->c[2]->w - m_Simplex->c[3]->w)) > 0)
				return true;
		}
		break;
		}
		return false;
	}

	void GetSupport(Vector3d& d, Vertex& sv)
	{
		sv.d = d * (1 / d.Length());
		sv.w = m_Shape->Support(sv.d);
	}

	void AddVertice(Simplex& simplex, Vector3d v)
	{
		simplex.p[simplex.dimension] = 0;
		simplex.c[simplex.dimension] = m_free[--m_nfree];
		GetSupport(v, *simplex.c[simplex.dimension++]);
	}

	void RemoVevertice(Simplex& simplex)
	{
		m_free[m_nfree++] = simplex.c[--simplex.dimension];
	}

	static float Determinant(const Vector3d& a, const Vector3d& b, const Vector3d& c)
	{
		return (a.y * b.z * c.x + a.z * b.x * c.y -
			a.x * b.z * c.y - a.y * b.x * c.z +
			a.x * b.y * c.z - a.z * b.y * c.x);
	}

	static float ProjectOriginDim2(Vector3d& a, Vector3d& b, float* w, int& m)
	{
		Vector3d d = b - a;
		float l = d.SquareLength();
		if (l > GJK_SIMPLEX2_EPS)
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

	static float ProjectOriginDim3(Vector3d& a, Vector3d& b, Vector3d& c, float* w, int& m)
	{
		const int	imd3[] = { 1, 2, 0 };
		Vector3d* vt[] = { &a, &b, &c };
		Vector3d	dl[] = { a - b, b - c, c - a };
		Vector3d	n = CrossProduct(dl[0], dl[1]);

		const float l = n.SquareLength();
		if (l > GJK_SIMPLEX3_EPS)
		{
			float mindist = -1;
			float subw[2] = { 0.f, 0.f };
			int subm(0);
			for (int i = 0; i < 3; ++i)
			{
				if (DotProduct(*vt[i], CrossProduct(dl[i], n)) > 0)
				{
					int j = imd3[i];
					float subd = ProjectOriginDim2(*vt[i], *vt[j], subw, subm);
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

	static float ProjectOriginDim4(Vector3d& a, Vector3d& b, Vector3d& c, Vector3d& d, float* w, int& m)
	{
		const int imd3[] = { 1, 2, 0 };
		Vector3d* vt[] = { &a, &b, &c, &d };
		Vector3d dl[] = { a - d, b - d, c - d };
		float vl = Determinant(dl[0], dl[1], dl[2]);
		bool ng = (vl * DotProduct(a, CrossProduct(b - c, a - b))) <= 0;
		if (ng && (fabsf(vl) > GJK_SIMPLEX4_EPS))
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
					float subd = ProjectOriginDim3(*vt[i], *vt[j], d, subw, subm);
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

