
#include <assert.h>
#include <cmath>
#include "Delaunay.h"

namespace Geometry
{
	inline int fast_mod(const int i, const int c)
	{
		return i >= c ? i % c : i;
	}

	// Kahan and Babuska summation, Neumaier variant; accumulates less FP error
	inline float sum(const std::vector<float>& x)
	{
		float sum = x[0];
		float err = 0.0;

		for (int i = 1; i < x.size(); i++)
		{
			const float k = x[i];
			const float m = sum + k;
			err += fabsf(sum) >= fabsf(k) ? sum - m + k : k - m + sum;
			sum = m;
		}
		return sum + err;
	}

	inline bool orient(
		const float px,
		const float py,
		const float qx,
		const float qy,
		const float rx,
		const float ry)
	{
		return (qy - py) * (rx - qx) - (qx - px) * (ry - qy) < 0.0;
	}

	inline float circumradius(
		const float ax,
		const float ay,
		const float bx,
		const float by,
		const float cx,
		const float cy)
	{
		const float dx = bx - ax;
		const float dy = by - ay;
		const float ex = cx - ax;
		const float ey = cy - ay;

		const float bl = dx * dx + dy * dy;
		const float cl = ex * ex + ey * ey;
		const float d = dx * ey - dy * ex;

		const float x = (ey * bl - dy * cl) * 0.5f / d;
		const float y = (dx * cl - ex * bl) * 0.5f / d;

		if ((bl > 0.0 || bl < 0.0) && (cl > 0.0 || cl < 0.0) && (d > 0.0 || d < 0.0))
		{
			return x * x + y * y;
		}
		else
		{
			return std::numeric_limits<float>::max();
		}
	}

	inline Vector2 circumcenter(
		const float ax,
		const float ay,
		const float bx,
		const float by,
		const float cx,
		const float cy)
	{
		const float dx = bx - ax;
		const float dy = by - ay;
		const float ex = cx - ax;
		const float ey = cy - ay;

		const float bl = dx * dx + dy * dy;
		const float cl = ex * ex + ey * ey;
		const float d = dx * ey - dy * ex;

		const float x = ax + (ey * bl - dy * cl) * 0.5f / d;
		const float y = ay + (dx * cl - ex * bl) * 0.5f / d;

		return Vector2(x, y);
	}

	struct compare
	{
		compare(const std::vector<Vector2>& points, const Vector2& c) : m_points(points), m_center(c)
		{
		}

		const std::vector<Vector2>& m_points;
		const Vector2& m_center;

		bool operator()(int i, int j)
		{
			const float d1 = (m_points[i] - m_center).SquareLength();
			const float d2 = (m_points[j] - m_center).SquareLength();
			const float d = d1 - d2;
			const float dx = m_points[i].x - m_points[j].x;
			const float dy = m_points[i].y - m_points[j].y;

			if (d > 0.0 || d < 0.0)
			{
				return d < 0;
			}
			else if (dx > 0.0 || dx < 0.0)
			{
				return dx < 0;
			}
			else {
				return dy < 0;
			}
		}
	};

	inline bool in_circle(
		const float ax,
		const float ay,
		const float bx,
		const float by,
		const float cx,
		const float cy,
		const float px,
		const float py)
	{
		const float dx = ax - px;
		const float dy = ay - py;
		const float ex = bx - px;
		const float ey = by - py;
		const float fx = cx - px;
		const float fy = cy - py;

		const float ap = dx * dx + dy * dy;
		const float bp = ex * ex + ey * ey;
		const float cp = fx * fx + fy * fy;

		return (dx * (ey * cp - bp * fy) -
			dy * (ex * cp - bp * fx) +
			ap * (ex * fy - ey * fx)) < 0.0;
	}

	inline bool check_pts_equal(float x1, float y1, float x2, float y2)
	{
		constexpr float EPSILON = std::numeric_limits<float>::epsilon();
		return fabsf(x1 - x2) <= EPSILON && fabsf(y1 - y2) <= EPSILON;
	}

	// monotonically increases with real angle, but doesn't need expensive trigonometry
	inline float pseudo_angle(const float dx, const float dy)
	{
		const float p = dx / (std::abs(dx) + std::abs(dy));
		return (dy > 0.0 ? 3.0f - p : 1.0f + p) / 4.0f; // [0..1)
	}

	struct DelaunatorPoint {
		int i;
		float x;
		float y;
		int t;
		int prev;
		int next;
		bool removed;
	};

	class Delaunator {

	public:
		std::vector<Vector2> coords;
		std::vector<int> triangles;
		std::vector<int> halfedges;
		std::vector<int> hull_prev;
		std::vector<int> hull_next;
		std::vector<int> hull_tri;
		int hull_start;

		explicit Delaunator(const std::vector<Vector2>& _coords) : coords(_coords)
		{
			int n = (int)coords.size();
			if (n < 3)
			{
				return;
			}

			float max_x = std::numeric_limits<float>::min();
			float max_y = std::numeric_limits<float>::min();
			float min_x = std::numeric_limits<float>::max();
			float min_y = std::numeric_limits<float>::max();
			std::vector<int> ids;
			ids.reserve(n);

			for (int i = 0; i < n; i++)
			{
				const float x = coords[i].x;
				const float y = coords[i].y;

				if (x < min_x) min_x = x;
				if (y < min_y) min_y = y;
				if (x > max_x) max_x = x;
				if (y > max_y) max_y = y;

				ids.push_back(i);
			}

			Vector2 c((min_x + max_x) / 2, (min_y + max_y) / 2);
			float min_dist = std::numeric_limits<float>::max();

			int i0 = -1;
			int i1 = -1;
			int i2 = -1;

			for (int i = 0; i < n; i++)
			{
				const float d = (c - coords[i]).SquareLength();
				if (d < min_dist)
				{
					i0 = i;
					min_dist = d;
				}
			}

			const float i0x = coords[i0].x;
			const float i0y = coords[i0].y;

			min_dist = std::numeric_limits<float>::max();

			for (int i = 0; i < n; i++)
			{
				if (i == i0) continue;
				const float d = (coords[i0] - coords[i]).SquareLength();
				if (d < min_dist && d > 0.0)
				{
					i1 = i;
					min_dist = d;
				}
			}

			float i1x = coords[i1].x;
			float i1y = coords[i1].y;

			float min_radius = std::numeric_limits<float>::max();

			for (int i = 0; i < n; i++)
			{
				if (i == i0 || i == i1) continue;

				const float r = circumradius(i0x, i0y, i1x, i1y, coords[i].x, coords[i].y);
				if (r < min_radius)
				{
					i2 = i;
					min_radius = r;
				}
			}

			if (!(min_radius < std::numeric_limits<float>::max()))
			{
				assert(false);
			}

			float i2x = coords[i2].x;
			float i2y = coords[i2].y;

			if (orient(i0x, i0y, i1x, i1y, i2x, i2y))
			{
				std::swap(i1, i2);
				std::swap(i1x, i2x);
				std::swap(i1y, i2y);
			}

			Vector2 center = circumcenter(i0x, i0y, i1x, i1y, i2x, i2y);

			std::sort(ids.begin(), ids.end(), compare{ coords, center });

			m_hash_size = static_cast<int>(std::llround(std::ceil(std::sqrt(n))));
			m_hash.resize(m_hash_size);
			std::fill(m_hash.begin(), m_hash.end(), -1);

			hull_prev.resize(n);
			hull_next.resize(n);
			hull_tri.resize(n);

			hull_start = i0;

			int hull_size = 3;

			hull_next[i0] = hull_prev[i2] = i1;
			hull_next[i1] = hull_prev[i0] = i2;
			hull_next[i2] = hull_prev[i1] = i0;

			hull_tri[i0] = 0;
			hull_tri[i1] = 1;
			hull_tri[i2] = 2;

			m_hash[hash_key(i0x, i0y)] = i0;
			m_hash[hash_key(i1x, i1y)] = i1;
			m_hash[hash_key(i2x, i2y)] = i2;

			int max_triangles = n < 3 ? 1 : 2 * n - 5;
			triangles.reserve(max_triangles * 3);
			halfedges.reserve(max_triangles * 3);
			add_triangle(i0, i1, i2, -1, -1, -1);
			float xp = std::numeric_limits<float>::quiet_NaN();
			float yp = std::numeric_limits<float>::quiet_NaN();
			for (int k = 0; k < n; k++)
			{
				const int i = ids[k];
				const float x = coords[i].x;
				const float y = coords[i].y;

				if (k > 0 && check_pts_equal(x, y, xp, yp))
					continue;
				xp = x;
				yp = y;

				if (check_pts_equal(x, y, i0x, i0y) ||
					check_pts_equal(x, y, i1x, i1y) ||
					check_pts_equal(x, y, i2x, i2y))
					continue;

				int start = 0;
				int key = hash_key(x, y);
				for (int j = 0; j < m_hash_size; j++)
				{
					start = m_hash[fast_mod(key + j, m_hash_size)];
					if (start != -1 && start != hull_next[start])
						break;
				}

				start = hull_prev[start];
				int e = start;
				int q;

				while (q = hull_next[e], !orient(x, y, coords[e].x, coords[e].y, coords[q].x, coords[q].y))
				{
					e = q;
					if (e == start)
					{
						e = -1;
						break;
					}
				}

				if (e == -1)
					continue;

				int t = add_triangle(
					e,
					i,
					hull_next[e],
					-1,
					-1,
					hull_tri[e]);

				hull_tri[i] = legalize(t + 2);
				hull_tri[e] = t;
				hull_size++;


				int next = hull_next[e];
				while (
					q = hull_next[next],
					orient(x, y, coords[next].x, coords[next].y, coords[q].x, coords[q].y))
				{
					t = add_triangle(next, i, q, hull_tri[i], -1, hull_tri[next]);
					hull_tri[i] = legalize(t + 2);
					hull_next[next] = next; // mark as removed
					hull_size--;
					next = q;
				}

				if (e == start)
				{
					while (
						q = hull_prev[e],
						orient(x, y, coords[q].x, coords[q].y, coords[e].x, coords[e].y))
					{
						t = add_triangle(q, i, e, -1, hull_tri[e], hull_tri[q]);
						legalize(t + 2);
						hull_tri[q] = t;
						hull_next[e] = e; // mark as removed
						hull_size--;
						e = q;
					}
				}

				// update the hull indices
				hull_prev[i] = e;
				hull_start = e;
				hull_prev[next] = i;
				hull_next[e] = i;
				hull_next[i] = next;

				m_hash[hash_key(x, y)] = i;
				m_hash[hash_key(coords[e].x, coords[e].y)] = e;
			}
		}

		float get_hull_area()
		{
			std::vector<float> hull_area;
			int e = hull_start;
			do
			{
				hull_area.push_back((coords[e].x - coords[hull_prev[e]].x) * (coords[e].y + coords[hull_prev[e]].y));
				e = hull_next[e];
			} while (e != hull_start);
			return sum(hull_area);
		}

	private:
		std::vector<int> m_hash;
		Vector2 m_center;
		int m_hash_size;
		std::vector<int> m_edge_stack;

		int legalize(int a)
		{
			int i = 0;
			int ar = 0;
			m_edge_stack.clear();

			// recursion eliminated with a fixed-size stack
			while (true)
			{
				const int b = halfedges[a];

				/* if the pair of triangles doesn't satisfy the Delaunay condition
				* (p1 is inside the circumcircle of [p0, pl, pr]), flip them,
				* then do the same check/flip recursively for the new pair of triangles
				*
				*           pl                    pl
				*          /||\                  /  \
				*       al/ || \bl            al/    \a
				*        /  ||  \              /      \
				*       /  a||b  \    flip    /___ar___\
				*     p0\   ||   /p1   =>   p0\---bl---/p1
				*        \  ||  /              \      /
				*       ar\ || /br             b\    /br
				*          \||/                  \  /
				*           pr                    pr
				*/
				const int a0 = 3 * (a / 3);
				ar = a0 + (a + 2) % 3;

				if (b == -1)
				{
					if (i > 0)
					{
						i--;
						a = m_edge_stack[i];
						continue;
					}
					else
					{
						//i = -1;
						break;
					}
				}

				const int b0 = 3 * (b / 3);
				const int al = a0 + (a + 1) % 3;
				const int bl = b0 + (b + 2) % 3;

				const int p0 = triangles[ar];
				const int pr = triangles[a];
				const int pl = triangles[al];
				const int p1 = triangles[bl];

				const bool illegal = in_circle(
					coords[p0].x,
					coords[p0].y,
					coords[pr].x,
					coords[pr].y,
					coords[pl].x,
					coords[pl].y,
					coords[p1].x,
					coords[p1].y);

				if (illegal)
				{
					triangles[a] = p1;
					triangles[b] = p0;

					auto hbl = halfedges[bl];

					// edge swapped on the other side of the hull (rare); fix the halfedge reference
					if (hbl == -1)
					{
						int e = hull_start;
						do
						{
							if (hull_tri[e] == bl)
							{
								hull_tri[e] = a;
								break;
							}
							e = hull_next[e];
						} while (e != hull_start);
					}
					link(a, hbl);
					link(b, halfedges[ar]);
					link(ar, bl);
					int br = b0 + (b + 1) % 3;

					if (i < m_edge_stack.size())
					{
						m_edge_stack[i] = br;
					}
					else
					{
						m_edge_stack.push_back(br);
					}
					i++;

				}
				else
				{
					if (i > 0)
					{
						i--;
						a = m_edge_stack[i];
						continue;
					}
					else
					{
						break;
					}
				}
			}
			return ar;
		}

		int hash_key(const float x, const float y) const
		{
			const float dx = x - m_center.x;
			const float dy = y - m_center.y;
			return fast_mod(
				static_cast<int>(std::llround(std::floor(pseudo_angle(dx, dy) * static_cast<float>(m_hash_size)))),
				m_hash_size);
		}

		int add_triangle(int i0, int i1, int i2, int a, int b, int c)
		{
			int t = (int)triangles.size();
			triangles.push_back(i0);
			triangles.push_back(i1);
			triangles.push_back(i2);
			link(t, a);
			link(t + 1, b);
			link(t + 2, c);
			return t;
		}

		void link(const int a, const int b)
		{
			int s = (int)halfedges.size();
			if (a == s)
			{
				halfedges.push_back(b);
			}
			else if (a < s)
			{
				halfedges[a] = b;
			}
			else
			{
				assert(false);
			}
			if (b != -1)
			{
				int s2 = (int)halfedges.size();
				if (b == s2)
				{
					halfedges.push_back(a);
				}
				else if (b < s2)
				{
					halfedges[b] = a;
				}
				else
				{
					assert(false);
				}
			}
		}

	};

	bool Delaunay::Triangulate(const std::vector<Vector2>& points)
	{
		Delaunator d(points);

		Points = points;
		for (size_t i = 0; i < d.triangles.size(); i += 3)
		{
			Triangles.emplace_back(d.triangles[i], d.triangles[i + 1], d.triangles[i + 2]);
		}
		return Triangles.size() > 0;
	}

	int Delaunay::GetNumTriangles() const
	{
		return (int)Triangles.size();
	}

	void Delaunay::GetTriangle(int index, Vector2& A, Vector2& B, Vector2& C)
	{
		if (index < (int)Triangles.size())
		{
			A = Points[Triangles[index].x];
			B = Points[Triangles[index].y];
			C = Points[Triangles[index].z];
		}
	}

}