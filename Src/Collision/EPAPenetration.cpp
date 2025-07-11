
#include "EPAPenetration.h"
#include "../Core/StaticList.h"
#include "../Core/StaticPool.h"

namespace Riemann
{
	// Expanding Polytope Algorithm
	// http://allenchou.net/2013/12/game-physics-contact-generation-epa/

	#define EPA_ACCURACY (0.0001f)
	#define EPA_PLANE_EPS (0.00001f)
	#define EPA_INSIDE_EPS (0.01f)
	#define EPA_MAX_VERTICES (128)
	#define EPA_MAX_FACES (EPA_MAX_VERTICES * 2)

	struct EpaFace
	{
		Vector3 normal;
		float dist;
		Simplex::Vertex* v[3];
		EpaFace* adjacent[3];
		EpaFace* next, * prev;
		uint8_t edge[3];
		uint8_t pass;

		static void Bind(EpaFace* fa, uint8_t ea, EpaFace* fb, uint8_t eb)
		{
			fa->edge[ea] = eb;
			fa->adjacent[ea] = fb;
			fb->edge[eb] = ea;
			fb->adjacent[eb] = fa;
		}
	};

	class PolytopeBuilder
	{
	public:
		struct Horizon
		{
			EpaFace* cf;
			EpaFace* ff;
			int nf;
			Horizon() : cf(nullptr), ff(nullptr), nf(0) {}
		};

		PolytopeBuilder()
		{
		}

		EpaFace* NewFace(Simplex::Vertex* a, Simplex::Vertex* b, Simplex::Vertex* c, EPA_status& result, bool forced)
		{
			if (m_face_pool.empty())
			{
				result = EPA_status::OutOfFaces;
				return nullptr;
			}

			EpaFace* face = m_face_pool.pop();
			m_poly.append(face);
			face->pass = 0;
			face->v[0] = a;
			face->v[1] = b;
			face->v[2] = c;
			face->normal = CrossProduct(b->pos - a->pos, c->pos - a->pos);

			const float l = face->normal.Length();
			if (l > EPA_ACCURACY)
			{
				if (!(GetEdgeDist(face, a, b, face->dist) ||
					GetEdgeDist(face, b, c, face->dist) ||
					GetEdgeDist(face, c, a, face->dist)))
				{
					// Origin projects to the interior of the triangle
					// Use distance to triangle plane
					face->dist = DotProduct(a->pos, face->normal) / l;
				}

				face->normal = face->normal * (1.0f / l);
				if (forced || face->dist >= -EPA_PLANE_EPS)
				{
					return face;
				}
				else
				{
					result = EPA_status::NonConvex;
				}
			}
			else
			{
				result = EPA_status::Degenerated;
			}

			m_poly.remove(face);
			m_face_pool.append(face);
			return nullptr;
		}

		bool Expand(uint8_t pass, Simplex::Vertex* w, EpaFace* f, uint8_t e, EPA_status& result, Horizon& horizon)
		{
			if (f->pass == pass)
			{
				return false;
			}

			int e1 = (e + 1) % 3;
			if ((DotProduct(f->normal, w->pos) - f->dist) < -EPA_PLANE_EPS)
			{
				EpaFace* nf = NewFace(f->v[e1], f->v[e], w, result, false);
				if (nf)
				{
					EpaFace::Bind(nf, 0, f, e);
					if (horizon.cf)
						EpaFace::Bind(horizon.cf, 1, nf, 2);
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
				if (Expand(pass, w, f->adjacent[e1], f->edge[e1], result, horizon) && Expand(pass, w, f->adjacent[e2], f->edge[e2], result, horizon))
				{
					m_poly.remove(f);
					m_face_pool.append(f);
					return (true);
				}
			}
			return false;
		}

		EpaFace* FindClosestToOrigin()
		{
			EpaFace* closest = m_poly.back();
			float min_sqrdist = closest->dist * closest->dist;
			for (EpaFace* f = closest->next; f; f = f->next)
			{
				const float sqrdist = f->dist * f->dist;
				if (sqrdist < min_sqrdist)
				{
					closest = f;
					min_sqrdist = sqrdist;
				}
			}
			return closest;
		}

		int NumFaces() const
		{
			return m_poly.size();
		}

		Simplex::Vertex* AddVertex()
		{
			return m_vertex_pool.get();
		}

		void Remove(EpaFace* f)
		{
			m_poly.remove(f);
			m_face_pool.append(f);
		}

	private:
		static bool GetEdgeDist(EpaFace* face, Simplex::Vertex* a, Simplex::Vertex* b, float& dist)
		{
			Vector3 ba = b->pos - a->pos;
			Vector3 n_ab = CrossProduct(ba, face->normal);   // Outward facing edge normal direction, on triangle plane
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
		List<EpaFace>									m_poly;
		StaticList<EpaFace, EPA_MAX_FACES>				m_face_pool;
		StaticPool<Simplex::Vertex, EPA_MAX_VERTICES>	m_vertex_pool;
	};

	EPA_status EPAPenetration::Solve(const Simplex& _src)
	{
		result = _src;

		if (result.dimension > 1 && result.EncloseOrigin())
		{
			status = EPA_status::Valid;

			if (ScalarTripleProduct(result.v[0].pos - result.v[3].pos,
				result.v[1].pos - result.v[3].pos,
				result.v[2].pos - result.v[3].pos) < 0)
			{
				std::swap(result.v[0], result.v[1]);
			}

			PolytopeBuilder polytope;
			EpaFace* tetra[] = { polytope.NewFace(&result.v[0], &result.v[1], &result.v[2], status, true),
								polytope.NewFace(&result.v[1], &result.v[0], &result.v[3], status, true),
								polytope.NewFace(&result.v[2], &result.v[1], &result.v[3], status, true),
								polytope.NewFace(&result.v[0], &result.v[2], &result.v[3], status, true) };

			if (polytope.NumFaces() == 4)
			{
				EpaFace* best = polytope.FindClosestToOrigin();
				EpaFace candidate = *best;
				uint8_t pass = 0;
				EpaFace::Bind(tetra[0], 0, tetra[1], 0);
				EpaFace::Bind(tetra[0], 1, tetra[2], 0);
				EpaFace::Bind(tetra[0], 2, tetra[3], 0);
				EpaFace::Bind(tetra[1], 1, tetra[3], 2);
				EpaFace::Bind(tetra[1], 2, tetra[2], 1);
				EpaFace::Bind(tetra[2], 2, tetra[3], 1);

				status = EPA_status::Valid;

				int iter = 0;
				int kMaxIterations = 255;
				while (iter++ < kMaxIterations)
				{
					Simplex::Vertex* v = polytope.AddVertex();
					if (v == nullptr)
					{
						status = EPA_status::OutOfVertices;
						break;
					}

					best->pass = ++pass;

					v->dir = best->normal.Unit();
					v->pos = result.m_shape->Support(v->dir);

					const float new_dist = DotProduct(best->normal, v->pos) - best->dist;
					if (new_dist <= EPA_ACCURACY)
					{
						status = EPA_status::AccuraryReached;
						break;
					}

					PolytopeBuilder::Horizon horizon;
					bool valid = true;
					for (int j = 0; j < 3 && valid; ++j)
					{
						valid &= polytope.Expand(pass, v, best->adjacent[j], best->edge[j], status, horizon);
					}

					if (valid && horizon.nf >= 3)
					{
						EpaFace::Bind(horizon.cf, 1, horizon.ff, 2);
						polytope.Remove(best);
						best = polytope.FindClosestToOrigin();
						candidate = *best;
					}
					else
					{
						status = EPA_status::InvalidHull;
						break;
					}
				}

				const Vector3 projection = candidate.normal * candidate.dist;
				penetration_normal = candidate.normal;
				penetration_depth = candidate.dist;
				result.dimension = 3;
				Simplex::Vertex v[3] = { *candidate.v[0], *candidate.v[1], *candidate.v[2] };
				result.v[0] = v[0];
				result.v[1] = v[1];
				result.v[2] = v[2];
				result.w[0] = CrossProduct(v[1].pos - projection, v[2].pos - projection).Length();
				result.w[1] = CrossProduct(v[2].pos - projection, v[0].pos - projection).Length();
				result.w[2] = CrossProduct(v[0].pos - projection, v[1].pos - projection).Length();
				const float sum = result.w[0] + result.w[1] + result.w[2];
				result.w[0] /= sum;
				result.w[1] /= sum;
				result.w[2] /= sum;
				return status;
			}
		}

		status = EPA_status::FallBack;
		penetration_normal = result.m_shape->Center();
		const float nl = penetration_normal.Length();
		if (nl > 0)
			penetration_normal = penetration_normal * (1 / nl);
		else
			penetration_normal = Vector3(1, 0, 0);
		penetration_depth = 0;
		result.dimension = 1;
		result.w[0] = 1;
		return status;
	}
}