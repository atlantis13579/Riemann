
#include "EPAPenetration.h"

// Expanding Polytope Algorithm
// http://allenchou.net/2013/12/game-physics-contact-generation-epa/

#define EPA_ACCURACY (0.0001f)
#define EPA_PLANE_EPS (0.00001f)
#define EPA_INSIDE_EPS (0.01f)
#define EPA_MAX_VERTICES (128)
#define EPA_MAX_ITERATIONS (255)
#define EPA_FALLBACK (10 * EPA_ACCURACY)
#define EPA_MAX_FACES (EPA_MAX_VERTICES * 2)

struct Face
{
	Vector3d n;
	float d;
	Simplex::Vertex* v[3];
	Face* f[3];
	Face* l[2];
	unsigned char e[3];
	unsigned char pass;

	static inline void Bind(Face* fa, int ea, Face* fb, int eb)
	{
		fa->e[ea] = (int)eb;
		fa->f[ea] = fb;
		fb->e[eb] = (int)ea;
		fb->f[eb] = fa;
	}
};

class HullBuilder
{
public:
	class List
	{
	public:
		Face* root;
		int count;
		List() : root(nullptr), count(0) {}

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
		sHorizon() : cf(nullptr), ff(nullptr), nf(0) {}
	};

	HullBuilder()
	{
		m_nextsv = 0;
		for (int i = 0; i < EPA_MAX_FACES; ++i)
		{
			m_stock.Append(&m_fc_store[EPA_MAX_FACES - i - 1]);
		}
	}

	Face* NewFace(Simplex::Vertex* a, Simplex::Vertex* b, Simplex::Vertex* c, EPA_result& result, bool forced)
	{
		if (m_stock.root)
		{
			Face* face = m_stock.root;
			m_stock.Remove(face);
			m_hull.Append(face);
			face->pass = 0;
			face->v[0] = a;
			face->v[1] = b;
			face->v[2] = c;
			face->n = CrossProduct(b->pos - a->pos, c->pos - a->pos);

			const float l = face->n.Length();
			if (l > EPA_ACCURACY)
			{
				if (!(GetEdgeDist(face, a, b, face->d) ||
					GetEdgeDist(face, b, c, face->d) ||
					GetEdgeDist(face, c, a, face->d)))
				{
					// Origin projects to the interior of the triangle
					// Use distance to triangle plane
					face->d = DotProduct(a->pos, face->n) / l;
				}

				face->n = face->n * (1 / l);
				if (forced || (face->d >= -EPA_PLANE_EPS))
				{
					return face;
				}
				else
				{
					result = EPA_result::NonConvex;
				}
			}
			else
			{
				result = EPA_result::Degenerated;
			}

			m_hull.Remove(face);
			m_stock.Append(face);
		}
		result = m_stock.root ? EPA_result::OutOfVertices : EPA_result::OutOfFaces;
		return nullptr;
	}

	bool Expand(unsigned char pass, Simplex::Vertex* w, Face* f, int e, EPA_result& result, sHorizon& horizon)
	{
		if (f->pass != pass)
		{
			int e1 = (e + 1) % 3;
			if ((DotProduct(f->n, w->pos) - f->d) < -EPA_PLANE_EPS)
			{
				Face* nf = NewFace(f->v[e1], f->v[e], w, result, false);
				if (nf)
				{
					Face::Bind(nf, 0, f, e);
					if (horizon.cf)
						Face::Bind(horizon.cf, 1, nf, 2);
					else
						horizon.ff = nf;
					horizon.cf = nf;
					++horizon.nf;
					return true;
				}
			}
			else
			{
				int e2 = (e + 2) % 3;
				f->pass = pass;
				if (Expand(pass, w, f->f[e1], f->e[e1], result, horizon) && Expand(pass, w, f->f[e2], f->e[e2], result, horizon))
				{
					m_hull.Remove(f);
					m_stock.Append(f);
					return (true);
				}
			}
		}
		return false;
	}

	Face* FindBest()
	{
		return m_hull.FindBest();
	}

	int GetCount() const
	{
		return m_hull.count;
	}

	int GetVertexCount() const
	{
		return m_nextsv;
	}

	Simplex::Vertex* AppendVertex()
	{
		return &m_sv_store[m_nextsv++];
	}

	void Free(Face* f)
	{
		m_hull.Remove(f);
		m_stock.Append(f);
	}

private:
	static bool GetEdgeDist(Face* face, Simplex::Vertex* a, Simplex::Vertex* b, float& dist)
	{
		Vector3d ba = b->pos - a->pos;
		Vector3d n_ab = CrossProduct(ba, face->n);   // Outward facing edge normal direction, on triangle plane
		float a_dot_nab = DotProduct(a->pos, n_ab);  // Only care about the sign to determine inside/outside, so not normalization required

		if (a_dot_nab >= 0)
		{
			return false;
		}

		// Outside of edge a->b

		const float ba_l2 = ba.SquareLength();
		const float a_dot_ba = DotProduct(a->pos, ba);
		const float b_dot_ba = DotProduct(b->pos, ba);

		if (a_dot_ba > 0)
		{
			// Pick distance Simplex::Vertex a
			dist = a->pos.Length();
		}
		else if (b_dot_ba < 0)
		{
			// Pick distance Simplex::Vertex b
			dist = b->pos.Length();
		}
		else
		{
			// Pick distance to edge a->b
			const float a_dot_b = DotProduct(a->pos, b->pos);
			float t = (a->pos.SquareLength() * b->pos.SquareLength() - a_dot_b * a_dot_b) / ba_l2;
			float bigger = t >= 0 ? t : 0;
			dist = sqrtf(bigger);
		}

		return true;
	}

private:
	Simplex::Vertex m_sv_store[EPA_MAX_VERTICES];
	Face m_fc_store[EPA_MAX_FACES];
	int m_nextsv;
	List m_hull;
	List m_stock;
};

EPA_result EPAPenetration::Solve(Simplex& simplex, const Vector3d& InitGuess)
{
	if (simplex.dimension > 1 && simplex.EncloseOrigin())
	{
		result = EPA_result::Valid;

		if (Determinant(simplex.v[0].pos - simplex.v[3].pos,
						simplex.v[1].pos - simplex.v[3].pos,
						simplex.v[2].pos - simplex.v[3].pos) < 0)
		{
			std::swap(simplex.v[0], simplex.v[1]);
		}

		HullBuilder hull;
		Face* tetra[] = { hull.NewFace(&simplex.v[0], &simplex.v[1], &simplex.v[2], result, true),
						  hull.NewFace(&simplex.v[1], &simplex.v[0], &simplex.v[3], result, true),
						  hull.NewFace(&simplex.v[2], &simplex.v[1], &simplex.v[3], result, true),
						  hull.NewFace(&simplex.v[0], &simplex.v[2], &simplex.v[3], result, true) };

		if (hull.GetCount() == 4)
		{
			Face* best = hull.FindBest();
			Face outer = *best;
			int pass = 0;
			Face::Bind(tetra[0], 0, tetra[1], 0);
			Face::Bind(tetra[0], 1, tetra[2], 0);
			Face::Bind(tetra[0], 2, tetra[3], 0);
			Face::Bind(tetra[1], 1, tetra[3], 2);
			Face::Bind(tetra[1], 2, tetra[2], 1);
			Face::Bind(tetra[2], 2, tetra[3], 1);

			result = EPA_result::Valid;

			int iterations = 0;
			while (iterations++ < EPA_MAX_ITERATIONS)
			{
				if (hull.GetVertexCount() >= EPA_MAX_VERTICES)
				{
					result = EPA_result::OutOfVertices;
					break;
				}

				HullBuilder::sHorizon horizon;
				Simplex::Vertex* v = hull.AppendVertex();
				bool valid = true;
				best->pass = (unsigned char)(++pass);

				v->dir = best->n.Unit();
				v->pos = simplex.Support(v->dir);

				const float wdist = DotProduct(best->n, v->pos) - best->d;
				if (wdist <= EPA_ACCURACY)
				{
					result = EPA_result::AccuraryReached;
					break;
				}

				for (int j = 0; j < 3 && valid; ++j)
				{
					valid &= hull.Expand(pass, v, best->f[j], best->e[j], result, horizon);
				}

				if (valid && (horizon.nf >= 3))
				{
					Face::Bind(horizon.cf, 1, horizon.ff, 2);
					hull.Free(best);
					best = hull.FindBest();
					outer = *best;
				}
				else
				{
					result = EPA_result::InvalidHull;
					break;
				}
			}

			const Vector3d projection = outer.n * outer.d;
			penetration_normal = outer.n;
			penetration_depth = outer.d;
			simplex.dimension = 3;
			Simplex::Vertex vv[3] = { *outer.v[0], *outer.v[1], *outer.v[2] };
			simplex.v[0] = vv[0];
			simplex.v[1] = vv[1];
			simplex.v[2] = vv[2];
			simplex.w[0] = CrossProduct(vv[1].pos - projection, vv[2].pos - projection).Length();
			simplex.w[1] = CrossProduct(vv[2].pos - projection, vv[0].pos - projection).Length();
			simplex.w[2] = CrossProduct(vv[0].pos - projection, vv[1].pos - projection).Length();
			const float sum = simplex.w[0] + simplex.w[1] + simplex.w[2];
			simplex.w[0] /= sum;
			simplex.w[1] /= sum;
			simplex.w[2] /= sum;
			return result;
		}
	}

	result = EPA_result::FallBack;
	penetration_normal = -InitGuess;
	const float nl = penetration_normal.Length();
	if (nl > 0)
		penetration_normal = penetration_normal * (1 / nl);
	else
		penetration_normal = Vector3d(1, 0, 0);
	penetration_depth = 0;
	simplex.dimension = 1;
	simplex.v[0] = simplex.v[0];
	simplex.w[0] = 1;
	return result;
}
