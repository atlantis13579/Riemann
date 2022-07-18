#pragma once

#include "Simplex.h"

// Expanding Polytope Algorithm
// http://allenchou.net/2013/12/game-physics-contact-generation-epa/

#define EPA_ACCURACY ((float)0.0001)
#define EPA_PLANE_EPS ((float)0.00001)
#define EPA_INSIDE_EPS ((float)0.01)
#define EPA_MAX_VERTICES 128
#define EPA_MAX_ITERATIONS 255
#define EPA_FALLBACK (10 * EPA_ACCURACY)
#define EPA_MAX_FACES (EPA_MAX_VERTICES * 2)

enum class EPA_status
{
	Valid,
	Touching,
	Degenerated,
	NonConvex,
	InvalidHull,
	OutOfFaces,
	OutOfVertices,
	AccuraryReached,
	FallBack,
	Failed
};

class EPA
{
public:
	struct Face
	{
		Vector3d n;
		float d;
		Simplex::Vertex* c[3];
		Face* f[3];
		Face* l[2];
		int e[3];
		int pass;
	};
	class List
	{
	public:
		Face* root;
		int count;
		List() : root(0), count(0) {}

		inline void Append(Face* face)
		{
			face->l[0] = 0;
			face->l[1] = root;
			if (root) root->l[0] = face;
			root = face;
			++count;
		}

		inline void Remove(Face* face)
		{
			if (face->l[1]) face->l[1]->l[0] = face->l[0];
			if (face->l[0]) face->l[0]->l[1] = face->l[1];
			if (face == root) root = face->l[1];
			--count;
		}

		Face* FindBest()
		{
			Face* minf = root;
			float mind = minf->d * minf->d;
			for (Face* f = minf->l[1]; f; f = f->l[1])
			{
				const float sqd = f->d * f->d;
				if (sqd < mind)
				{
					minf = f;
					mind = sqd;
				}
			}
			return minf;
		}
	};
	struct sHorizon
	{
		Face* cf;
		Face* ff;
		int nf;
		sHorizon() : cf(0), ff(0), nf(0) {}
	};

	EPA_status status;
	Simplex simplex;
	Vector3d normal;
	float penetration_depth;
	Simplex::Vertex m_sv_store[EPA_MAX_VERTICES];
	Face m_fc_store[EPA_MAX_FACES];
	int m_nextsv;
	List m_hull;
	List m_stock;

	EPA()
	{
		status = EPA_status::Failed;
		normal = Vector3d(0, 0, 0);
		penetration_depth = 0;
		m_nextsv = 0;
		for (int i = 0; i < EPA_MAX_FACES; ++i)
		{
			m_stock.Append(&m_fc_store[EPA_MAX_FACES - i - 1]);
		}
	}

	EPA_status Solve(Simplex& _simplex, MinkowskiSum *Shape, Vector3d InitGuess)
	{
		memcpy(&simplex, &_simplex, sizeof(Simplex));

		if ((simplex.dimension > 1) && simplex.EncloseOrigin())
		{
			while (m_hull.root)
			{
				Face* f = m_hull.root;
				m_hull.Remove(f);
				m_stock.Append(f);
			}
			status = EPA_status::Valid;
			m_nextsv = 0;

			if (Determinant(simplex.v[0].p - simplex.v[3].p,
							simplex.v[1].p - simplex.v[3].p,
							simplex.v[2].p - simplex.v[3].p) < 0)
			{
				std::swap(simplex.v[0], simplex.v[1]);
				std::swap(simplex.w[0], simplex.w[1]);
			}

			Face* tetra[] = { NewFace(&simplex.v[0], &simplex.v[1], &simplex.v[2], true),
							  NewFace(&simplex.v[1], &simplex.v[0], &simplex.v[3], true),
							  NewFace(&simplex.v[2], &simplex.v[1], &simplex.v[3], true),
							  NewFace(&simplex.v[0], &simplex.v[2], &simplex.v[3], true) };
			if (m_hull.count == 4)
			{
				Face* best = m_hull.FindBest();
				Face outer = *best;
				int pass = 0;
				int iterations = 0;
				Bind(tetra[0], 0, tetra[1], 0);
				Bind(tetra[0], 1, tetra[2], 0);
				Bind(tetra[0], 2, tetra[3], 0);
				Bind(tetra[1], 1, tetra[3], 2);
				Bind(tetra[1], 2, tetra[2], 1);
				Bind(tetra[2], 2, tetra[3], 1);
				status = EPA_status::Valid;
				for (; iterations < EPA_MAX_ITERATIONS; ++iterations)
				{
					if (m_nextsv < EPA_MAX_VERTICES)
					{
						sHorizon horizon;
						Simplex::Vertex* w = &m_sv_store[m_nextsv++];
						bool valid = true;
						best->pass = (int)(++pass);

						w->d = best->n.Unit();
						w->p = Shape->Support(w->d);

						const float wdist = DotProduct(best->n, w->p) - best->d;
						if (wdist > EPA_ACCURACY)
						{
							for (int j = 0; (j < 3) && valid; ++j)
							{
								valid &= Expand(pass, w, best->f[j], best->e[j], horizon);
							}
							if (valid && (horizon.nf >= 3))
							{
								Bind(horizon.cf, 1, horizon.ff, 2);
								m_hull.Remove(best);
								m_stock.Append(best);
								best = m_hull.FindBest();
								outer = *best;
							}
							else
							{
								status = EPA_status::InvalidHull;
								break;
							}
						}
						else
						{
							status = EPA_status::AccuraryReached;
							break;
						}
					}
					else
					{
						status = EPA_status::OutOfVertices;
						break;
					}
				}
				const Vector3d projection = outer.n * outer.d;
				normal = outer.n;
				penetration_depth = outer.d;
				simplex.dimension = 3;
				simplex.v[0] = *outer.c[0];
				simplex.v[1] = *outer.c[1];
				simplex.v[2] = *outer.c[2];
				simplex.w[0] = CrossProduct(outer.c[1]->p - projection, outer.c[2]->p - projection).Length();
				simplex.w[1] = CrossProduct(outer.c[2]->p - projection, outer.c[0]->p - projection).Length();
				simplex.w[2] = CrossProduct(outer.c[0]->p - projection, outer.c[1]->p - projection).Length();
				const float sum = simplex.w[0] + simplex.w[1] + simplex.w[2];
				simplex.w[0] /= sum;
				simplex.w[1] /= sum;
				simplex.w[2] /= sum;
				return (status);
			}
		}

		status = EPA_status::FallBack;
		normal = -InitGuess;
		const float nl = normal.Length();
		if (nl > 0)
			normal = normal * (1 / nl);
		else
			normal = Vector3d(1, 0, 0);
		penetration_depth = 0;
		simplex.dimension = 1;
		simplex.v[0] = simplex.v[0];
		simplex.w[0] = 1;
		return (status);
	}

	static inline void Bind(Face* fa, int ea, Face* fb, int eb)
	{
		fa->e[ea] = (int)eb;
		fa->f[ea] = fb;
		fb->e[eb] = (int)ea;
		fb->f[eb] = fa;
	}

	bool GetEdgeDist(Face* face, Simplex::Vertex* a, Simplex::Vertex* b, float& dist)
	{
		Vector3d ba = b->p - a->p;
		Vector3d n_ab = CrossProduct(ba, face->n);   // Outward facing edge normal direction, on triangle plane
		float a_dot_nab = DotProduct(a->p, n_ab);  // Only care about the sign to determine inside/outside, so not normalization required

		if (a_dot_nab < 0)
		{
			// Outside of edge a->b

			const float ba_l2 = ba.SquareLength();
			const float a_dot_ba = DotProduct(a->p, ba);
			const float b_dot_ba = DotProduct(b->p, ba);

			if (a_dot_ba > 0)
			{
				// Pick distance Simplex::Vertex a
				dist = a->p.Length();
			}
			else if (b_dot_ba < 0)
			{
				// Pick distance Simplex::Vertex b
				dist = b->p.Length();
			}
			else
			{
				// Pick distance to edge a->b
				const float a_dot_b = DotProduct(a->p, b->p);

				float t = (a->p.SquareLength() * b->p.SquareLength() - a_dot_b * a_dot_b) / ba_l2;
				float bigger = t >= 0 ? t : 0;
				dist = sqrtf(bigger);
			}

			return true;
		}

		return false;
	}

	Face* NewFace(Simplex::Vertex* a, Simplex::Vertex* b, Simplex::Vertex* c, bool forced)
	{
		if (m_stock.root)
		{
			Face* face = m_stock.root;
			m_stock.Remove(face);
			m_hull.Append(face);
			face->pass = 0;
			face->c[0] = a;
			face->c[1] = b;
			face->c[2] = c;
			face->n = CrossProduct(b->p - a->p, c->p - a->p);
			const float l = face->n.Length();
			const bool v = l > EPA_ACCURACY;

			if (v)
			{
				if (!(GetEdgeDist(face, a, b, face->d) ||
					GetEdgeDist(face, b, c, face->d) ||
					GetEdgeDist(face, c, a, face->d)))
				{
					// Origin projects to the interior of the triangle
					// Use distance to triangle plane
					face->d = DotProduct(a->p, face->n) / l;
				}

				face->n = face->n * (1 / l);
				if (forced || (face->d >= -EPA_PLANE_EPS))
				{
					return face;
				}
				else
					status = EPA_status::NonConvex;
			}
			else
				status = EPA_status::Degenerated;

			m_hull.Remove(face);
			m_stock.Append(face);
			return 0;
		}
		status = m_stock.root ? EPA_status::OutOfVertices : EPA_status::OutOfFaces;
		return 0;
	}

	bool Expand(int pass, Simplex::Vertex* w, Face* f, int e, sHorizon& horizon)
	{
		const int i1m3[] = { 1, 2, 0 };
		const int i2m3[] = { 2, 0, 1 };
		if (f->pass != pass)
		{
			int e1 = i1m3[e];
			if ((DotProduct(f->n, w->p) - f->d) < -EPA_PLANE_EPS)
			{
				Face* nf = NewFace(f->c[e1], f->c[e], w, false);
				if (nf)
				{
					Bind(nf, 0, f, e);
					if (horizon.cf)
						Bind(horizon.cf, 1, nf, 2);
					else
						horizon.ff = nf;
					horizon.cf = nf;
					++horizon.nf;
					return true;
				}
			}
			else
			{
				int e2 = i2m3[e];
				f->pass = (int)pass;
				if (Expand(pass, w, f->f[e1], f->e[e1], horizon) && Expand(pass, w, f->f[e2], f->e[e2], horizon))
				{
					m_hull.Remove(f);
					m_stock.Append(f);
					return (true);
				}
			}
		}
		return false;
	}
};
