
#include <assert.h>
#include <cmath>
#include <deque>
#include <stack>
#include <string>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include "../Maths/Maths.h"
#include "../Maths/Box2.h"
#include "../Maths/Index2.h"
#include "Delaunay.h"

namespace Riemann
{
	static inline int fast_mod(const int i, const int c)
	{
		return i>= c ? i % c : i;
	}

	// Kahan and Babuska summation, Neumaier variant; accumulates less FP error
	static inline float sum(const std::vector<float>& x)
	{
		float sum = x[0];
		float err = 0.0;

		for (size_t i = 1; i < x.size(); ++i)
		{
			const float k = x[i];
			const float m = sum + k;
			err += fabsf(sum)>= fabsf(k) ? sum - m + k : k - m + sum;
			sum = m;
		}
		return sum + err;
	}

	static inline bool orient(
		const float px,
		const float py,
		const float qx,
		const float qy,
		const float rx,
		const float ry)
	{
		return (qy - py) * (rx - qx) - (qx - px) * (ry - qy) < 0.0;
	}

	static inline float circumradius(
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

		if ((bl> 0.0 || bl < 0.0) && (cl> 0.0 || cl < 0.0) && (d> 0.0 || d < 0.0))
		{
			return x * x + y * y;
		}
		else
		{
			return std::numeric_limits<float>::max();
		}
	}

	static inline Vector2 circumcenter(
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

	static inline bool in_circle(
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

	static inline bool check_pts_equal(float x1, float y1, float x2, float y2)
	{
		constexpr float EPSILON = std::numeric_limits<float>::epsilon();
		return fabsf(x1 - x2) <= EPSILON && fabsf(y1 - y2) <= EPSILON;
	}

	// monotonically increases with real angle, but doesn't need expensive trigonometry
	static inline float pseudo_angle(const float dx, const float dy)
	{
		const float p = dx / (std::abs(dx) + std::abs(dy));
		return (dy> 0.0 ? 3.0f - p : 1.0f + p) / 4.0f; // [0..1)
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

			if (d> 0.0 || d < 0.0)
			{
				return d < 0;
			}
			else if (dx> 0.0 || dx < 0.0)
			{
				return dx < 0;
			}
			else {
				return dy < 0;
			}
		}
	};

	struct DelaunatorPoint {
		int i;
		float x;
		float y;
		int t;
		int prev;
		int next;
		bool removed;
	};

	class Delaunator
	{
	public:
		std::vector<Vector2> coords;
		std::vector<int> triangles;
		std::vector<int> halfedges;
		std::vector<int> hull_prev;
		std::vector<int> hull_next;
		std::vector<int> hull_tri;
		int hull_start {false};

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
				if (x> max_x) max_x = x;
				if (y> max_y) max_y = y;

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
				if (d < min_dist && d > 0.0f)
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

				if (k> 0 && check_pts_equal(x, y, xp, yp))
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

                while (static_cast<void>(q = hull_next[e]), !orient(x, y, coords[e].x, coords[e].y, coords[q].x, coords[q].y))
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
                       static_cast<void>(q = hull_next[next]),
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
                           static_cast<void>(q = hull_prev[e]),
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
			}
			while (e != hull_start);
			return sum(hull_area);
		}

	private:
		std::vector<int> m_hash;
		Vector2 m_center;
		int m_hash_size {0};
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
					if (i> 0)
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
					if (i> 0)
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
		return Triangles.size()> 0;
	}

	int Delaunay::GetNumTriangles() const
	{
		return (int)Triangles.size();
	}

	void Delaunay::GetTriangle(int index, Vector2& A, Vector2& B, Vector2& C)
	{
		if (index < (int)Triangles.size())
		{
			A = Points[Triangles[index].v1];
			B = Points[Triangles[index].v2];
			C = Points[Triangles[index].v3];
		}
	}

	enum class NodeSplitDirection
	{
		X,
		Y,
	};

	template <int NumVerticesInLeaf, int InitialStackDepth, int StackDepthIncrement>
	class KDTree
	{
	public:
		struct Node
		{
			int children[2];
			std::vector<int> data;

			Node()
			{
				setChildren(0, 0);
				data.reserve(NumVerticesInLeaf);
			}

			void setChildren(int c1, int c2)
			{
				children[0] = c1;
				children[1] = c2;
			}

			bool isLeaf() const
			{
				return children[0] == children[1];
			}
		};

		KDTree()
			: m_rootDir(NodeSplitDirection::X)
			, m_min(-std::numeric_limits<float>::max(),	-std::numeric_limits<float>::max())
			, m_max(std::numeric_limits<float>::max(), std::numeric_limits<float>::max())
			, m_size(0)
			, m_isRootBoxInitialized(false)
			, m_tasksStack(InitialStackDepth, NearestTask())
		{
			m_root = addNewNode();
		}

		KDTree(const Vector2& min, const Vector2& max)
			: m_rootDir(NodeSplitDirection::X)
			, m_min(min)
			, m_max(max)
			, m_size(0)
			, m_isRootBoxInitialized(true)
			, m_tasksStack(InitialStackDepth, NearestTask())
		{
			m_root = addNewNode();
		}

		int size() const
		{
			return m_size;
		}

		void insert(int idx, const std::vector<Vector2>& points)
		{
			++m_size;
			const Vector2& pos = points[idx];
			while (!isInsideBox(pos, m_min, m_max))
			{
				extendTree(pos);
			}
			int node = m_root;
			Vector2 min = m_min;
			Vector2 max = m_max;
			NodeSplitDirection dir = m_rootDir;

			NodeSplitDirection newDir(NodeSplitDirection::X);
			float mid = 0.0f;
			Vector2 newMin, newMax;
			while (true)
			{
				if (m_nodes[node].isLeaf())
				{
					std::vector<int>& pd = m_nodes[node].data;
					if (pd.size() < NumVerticesInLeaf)
					{
						pd.push_back(idx);
						return;
					}

					if (!m_isRootBoxInitialized)
					{
						initializeRootBox(points);
						min = m_min;
						max = m_max;
					}

					calcSplitInfo(min, max, dir, mid, newDir, newMin, newMax);
					const int c1 = addNewNode(), c2 = addNewNode();
					Node& n = m_nodes[node];
					n.setChildren(c1, c2);
					std::vector<int>& c1data = m_nodes[c1].data;
					std::vector<int>& c2data = m_nodes[c2].data;

					for (auto it = n.data.begin(); it != n.data.end(); ++it)
					{
						if (whichChild(points[*it], mid, dir) == 0)
						{
							c1data.push_back(*it);
						}
						else
						{
							c2data.push_back(*it);
						}
					}
					n.data = std::vector<int>();
				}
				else
				{
					calcSplitInfo(min, max, dir, mid, newDir, newMin, newMax);
				}

				const std::size_t iChild = whichChild(points[idx], mid, dir);
				if (iChild == 0)
					max = newMax;
				else
					min = newMin;
				node = m_nodes[node].children[iChild];
				dir = newDir;
			}
		}

		std::pair<Vector2, int> nearest(const Vector2& point, const std::vector<Vector2>& points) const
		{
			std::pair<Vector2, int> out;
			int iTask = -1;
			float minDistSq = std::numeric_limits<float>::max();
			m_tasksStack[++iTask] =	NearestTask(m_root, m_min, m_max, m_rootDir, minDistSq);
			while (iTask != -1)
			{
				const NearestTask t = m_tasksStack[iTask--];
				if (t.distSq> minDistSq)
					continue;
				const Node& n = m_nodes[t.node];
				if (n.isLeaf())
				{
					for (std::vector<int>::const_iterator it = n.data.begin(); it != n.data.end(); ++it)
					{
						const Vector2& p = points[*it];
						const float distSq = (point - p).SquareLength();
						if (distSq < minDistSq)
						{
							minDistSq = distSq;
							out.first = p;
							out.second = *it;
						}
					}
				}
				else
				{
					float mid = 0.0f;
					NodeSplitDirection newDir;
					Vector2 newMin, newMax;
					calcSplitInfo(t.min, t.max, t.dir, mid, newDir, newMin, newMax);

					const float distToMid = t.dir == NodeSplitDirection::X
						? (point.x - mid)
						: (point.y - mid);
					const float toMidSq = distToMid * distToMid;

					const std::size_t iChild = whichChild(point, mid, t.dir);
					if (iTask + 2>= static_cast<int>(m_tasksStack.size()))
					{
						m_tasksStack.resize(
							m_tasksStack.size() + StackDepthIncrement);
					}
					if (iChild == 0)
					{
						m_tasksStack[++iTask] = NearestTask(
							n.children[1], newMin, t.max, newDir, toMidSq);
						m_tasksStack[++iTask] = NearestTask(
							n.children[0], t.min, newMax, newDir, toMidSq);
					}
					else
					{
						m_tasksStack[++iTask] = NearestTask(
							n.children[0], t.min, newMax, newDir, toMidSq);
						m_tasksStack[++iTask] = NearestTask(
							n.children[1], newMin, t.max, newDir, toMidSq);
					}
				}
			}
			return out;
		}

	private:
		int addNewNode()
		{
			const int newNodeIndex = static_cast<int>(m_nodes.size());
			m_nodes.push_back(Node());
			return newNodeIndex;
		}

		std::size_t whichChild(const Vector2& point, const float& split, const NodeSplitDirection dir) const
		{
			return static_cast<size_t>(
				dir == NodeSplitDirection::X ? point.x> split : point.y> split);
		}

		static void calcSplitInfo(
			const Vector2& min,
			const Vector2& max,
			const NodeSplitDirection dir,
			float& midOut,
			NodeSplitDirection& newDirOut,
			Vector2& newMinOut,
			Vector2& newMaxOut)
		{
			newMaxOut = max;
			newMinOut = min;
			switch (dir)
			{
			case NodeSplitDirection::X:
				midOut = (min.x + max.x) / 2.0f;
				newDirOut = NodeSplitDirection::Y;
				newMinOut.x = midOut;
				newMaxOut.x = midOut;
				return;
			case NodeSplitDirection::Y:
				midOut = (min.y + max.y) / 2.0f;
				newDirOut = NodeSplitDirection::X;
				newMinOut.y = midOut;
				newMaxOut.y = midOut;
				return;
			}
		}

		static bool isInsideBox(const Vector2& p, const Vector2& min, const Vector2& max)
		{
			return p.x>= min.x && p.x <= max.x && p.y>= min.y && p.y <= max.y;
		}

		void extendTree(const Vector2& point)
		{
			const int newRoot = addNewNode();
			const int newLeaf = addNewNode();
			switch (m_rootDir)
			{
			case NodeSplitDirection::X:
				m_rootDir = NodeSplitDirection::Y;
				point.y < m_min.y ? m_nodes[newRoot].setChildren(newLeaf, m_root)
					: m_nodes[newRoot].setChildren(m_root, newLeaf);
				if (point.y < m_min.y)
					m_min.y -= m_max.y - m_min.y;
				else if (point.y> m_max.y)
					m_max.y += m_max.y - m_min.y;
				break;
			case NodeSplitDirection::Y:
				m_rootDir = NodeSplitDirection::X;
				point.x < m_min.x ? m_nodes[newRoot].setChildren(newLeaf, m_root)
					: m_nodes[newRoot].setChildren(m_root, newLeaf);
				if (point.x < m_min.x)
					m_min.x -= m_max.x - m_min.x;
				else if (point.x> m_max.x)
					m_max.x += m_max.x - m_min.x;
				break;
			}
			m_root = newRoot;
		}

		void initializeRootBox(const std::vector<Vector2>& points)
		{
			const std::vector<int>& data = m_nodes[m_root].data;
			m_min = points[data.front()];
			m_max = m_min;
			for (int i : data)
			{
				const Vector2& p = points[i];
				m_min = Vector2(std::min(m_min.x, p.x), std::min(m_min.y, p.y));
				m_max = Vector2(std::max(m_max.x, p.x), std::max(m_max.y, p.y));
			}

			const float padding = 1.0f;
			if (m_min.x == m_max.x)
			{
				m_min.x -= padding;
				m_max.x += padding;
			}
			if (m_min.y == m_max.y)
			{
				m_min.y -= padding;
				m_max.y += padding;
			}
			m_isRootBoxInitialized = true;
		}

	private:
		int m_root;
		std::vector<Node> m_nodes;
		NodeSplitDirection m_rootDir;
		Vector2 m_min;
		Vector2 m_max;
		int m_size;

		bool m_isRootBoxInitialized;

		// used for nearest query
		struct NearestTask
		{
			int node;
			Vector2 min, max;
			NodeSplitDirection dir;
			float distSq;
			NearestTask()
			{}
			NearestTask(
				const int node_,
				const Vector2& min_,
				const Vector2& max_,
				const NodeSplitDirection dir_,
				const float distSq_)
				: node(node_)
				, min(min_)
				, max(max_)
				, dir(dir_)
				, distSq(distSq_)
			{}
		};
		mutable std::vector<NearestTask> m_tasksStack;
	};

	template <size_t NumVerticesInLeaf = 32, size_t InitialStackDepth = 32, size_t StackDepthIncrement = 32>
	class LocatorKDTree
	{
	public:
		void initialize(const std::vector<Vector2>& points)
		{
			Vector2 min = points.front();
			Vector2 max = min;
			for (const Vector2& pt : points)
			{
				min = Vector2(std::min(min.x, pt.x), std::min(min.y, pt.y));
				max = Vector2(std::max(max.x, pt.x), std::max(max.y, pt.y));
			}
			m_kdTree = KDTree_t(min, max);
			for (size_t i = 0; i < points.size(); ++i)
			{
				m_kdTree.insert((int)i, points);
			}
		}

		void addPoint(int i, const std::vector<Vector2>& points)
		{
			m_kdTree.insert(i, points);
		}

		int nearPoint(const Vector2& pos, const std::vector<Vector2>& points) const
		{
			return m_kdTree.nearest(pos, points).second;
		}

		int size() const
		{
			return m_kdTree.size();
		}

		bool empty() const
		{
			return  m_kdTree.size() == 0;
		}

	private:
		typedef KDTree<NumVerticesInLeaf, InitialStackDepth, StackDepthIncrement> KDTree_t;
		KDTree_t m_kdTree;
	};

	enum class VertexInsertionOrder
	{
		Auto,
		AsProvided,
	};

	enum class SuperGeometryType
	{
		SuperTriangle,
		Custom,
	};

	enum class IntersectingConstraintEdges
	{
		NotAllowed,
		TryResolve,
		DontCheck,
	};

	enum class PtLineLocation
    {
		Left,
		Right,
		OnLine,
    };

	enum class PtTriLocation
	{
		Inside,
		Outside,
		OnEdge1,
		OnEdge2,
		OnEdge3,
		OnVertex,
	};

	typedef unsigned short LayerDepth;

	struct Triangle
	{
        Triangle(const DelaunayTriangle& _v, const DelaunayTriangle& _n)
        {
            vertices = _v;
            neighbors = _n;
        }

        DelaunayTriangle vertices;
		DelaunayTriangle neighbors;

		std::pair<int, int> next(const int i) const
		{
			assert(vertices[0] == i || vertices[1] == i || vertices[2] == i);
			if (vertices[0] == i)
			{
				return std::make_pair(neighbors[0], vertices[1]);
			}
			if (vertices[1] == i)
			{
				return std::make_pair(neighbors[1], vertices[2]);
			}
			return std::make_pair(neighbors[2], vertices[0]);
		}

		std::pair<int, int> prev(const int i) const
		{
			assert(vertices[0] == i || vertices[1] == i || vertices[2] == i);
			if (vertices[0] == i)
				return std::make_pair(neighbors[2], vertices[2]);
			if (vertices[1] == i)
				return std::make_pair(neighbors[0], vertices[0]);
			return std::make_pair(neighbors[1], vertices[1]);
		}

		bool containsVertex(const int idx) const
		{
			for (int i = 0; i < 3; ++i)
			{
				if (vertices[i] == idx)
					return true;
			}

			return false;
		}
	};

	template <class Compare, class ForwardIterator>
	static int sort3(ForwardIterator x, ForwardIterator y, ForwardIterator z, Compare c)
	{
		int r = 0;
		if (!c(*y, *x)) // if x <= y
		{
			if (!c(*z, *y))     // if y <= z
				return r;      // x <= y && y <= z
							   // x <= y && y > z
			std::swap(*y, *z); // x <= z && y < z
			r = 1;
			if (c(*y, *x)) // if x > y
			{
				std::swap(*x, *y); // x < y && y <= z
				r = 2;
			}
			return r; // x <= y && y < z
		}
		if (c(*z, *y)) // x > y, if y > z
		{
			std::swap(*x, *z); // x < y && y < z
			r = 1;
			return r;
		}
		std::swap(*x, *y); // x > y && y <= z
		r = 1;             // x < y && x <= z
		if (c(*z, *y))      // if y > z
		{
			std::swap(*y, *z); // x <= y && y < z
			r = 2;
		}
		return r;
	} // x <= y && y <= z

	template <class Compare, class BirdirectionalIterator>
	static void selection_sort(
		BirdirectionalIterator first,
		BirdirectionalIterator last,
		Compare comp)
	{
		BirdirectionalIterator lm1 = last;
		for (--lm1; first != lm1; ++first)
		{
			BirdirectionalIterator i = std::min_element(first, last, comp);
			if (i != first)
				std::swap(*first, *i);
		}
	}

	template <class Compare, class RandomAccessIterator>
	static void _nth_element(
		RandomAccessIterator first,
		RandomAccessIterator nth,
		RandomAccessIterator last,
		Compare comp)
	{
		typedef typename std::iterator_traits<RandomAccessIterator>::difference_type
			difference_type;
		const difference_type limit = 7;
		while (true)
		{
		restart:
			if (nth == last)
				return;
			difference_type len = last - first;
			switch (len)
			{
			case 0:
			case 1:
				return;
			case 2:
				if (comp(*--last, *first))
					std::swap(*first, *last);
				return;
			case 3: {
				RandomAccessIterator m = first;
				sort3<Compare>(first, ++m, --last, comp);
				return;
			}
			}
			if (len <= limit)
			{
				selection_sort<Compare>(first, last, comp);
				return;
			}
			RandomAccessIterator m = first + len / 2;
			RandomAccessIterator lm1 = last;
			int n_swaps = sort3<Compare>(first, m, --lm1, comp);
			RandomAccessIterator i = first;
			RandomAccessIterator j = lm1;
			if (!comp(*i, *m))
			{
				while (true)
				{
					if (i == --j)
					{
						++i;
						j = last;
						if (!comp(*first, *--j))
						{
							while (true)
							{
								if (i == j)
									return;
								if (comp(*first, *i))
								{
									std::swap(*i, *j);
									++n_swaps;
									++i;
									break;
								}
								++i;
							}
						}
						if (i == j)
							return;
						while (true)
						{
							while (!comp(*first, *i))
								++i;
							while (comp(*first, *--j))
								;
							if (i >= j)
								break;
							std::swap(*i, *j);
							++n_swaps;
							++i;
						}
						if (nth < i)
							return;
						first = i;
						goto restart;
					}
					if (comp(*j, *m))
					{
						std::swap(*i, *j);
						++n_swaps;
						break;
					}
				}
			}
			++i;
			if (i < j)
			{
				while (true)
				{
					while (comp(*i, *m))
						++i;
					while (!comp(*--j, *m))
						;
					if (i >= j)
						break;
					std::swap(*i, *j);
					++n_swaps;
					if (m == i)
						m = j;
					++i;
				}
			}
			if (i != m && comp(*m, *i))
			{
				std::swap(*i, *m);
				++n_swaps;
			}
			if (nth == i)
				return;
			if (n_swaps == 0)
			{
				if (nth < i)
				{
					j = m = first;
					while (++j != i)
					{
						if (comp(*j, *m))
							goto not_sorted;
						m = j;
					}
					return;
				}
				else
				{
					j = m = i;
					while (++j != last)
					{
						if (comp(*j, *m))
							goto not_sorted;
						m = j;
					}
					return;
				}
			}
		not_sorted:
			if (nth < i)
			{
				last = i;
			}
			else
			{
				first = ++i;
			}
		}
	}

	template <class _RandomAccessIterator, class _Compare>
	static inline void portable_nth_element(
		_RandomAccessIterator first,
		_RandomAccessIterator nth,
		_RandomAccessIterator last,
		_Compare comp)
	{
		_nth_element<typename std::add_lvalue_reference<_Compare>::type>(
			first, nth, last, comp);
	}

	static inline float log2_bc(float x)
    {
        return std::log2f(x);
    }

    static float lerp(const float& a, const float& b, const float t)
    {
        return (float(1) - t) * a + t * b;
    }

    static float orient2d(float ax, float ay, float bx, float by, float cx, float cy)
    {
        const float acx = ax - cx;
        const float bcx = bx - cx;
        const float acy = ay - cy;
        const float bcy = by - cy;
        const float detleft = acx * bcy;
        const float detright = acy * bcx;
        float det = detleft - detright;
        return det;
    }

    static float orient2d(const Vector2& p, const Vector2& v1, const Vector2& v2)
    {
        return orient2d(v1.x, v1.y, v2.x, v2.y, p.x, p.y);
    }

	static PtLineLocation classifyOrientation(const float orientation, const float orientationTolerance = 1e-6f)
    {
        if (orientation < -orientationTolerance)
            return PtLineLocation::Right;
        if (orientation > orientationTolerance)
            return PtLineLocation::Left;
        return PtLineLocation::OnLine;
    }

	static PtLineLocation locatePointLine(const Vector2& p, const Vector2& v1, const Vector2& v2, const float orientationTolerance = 1e-6f)
    {
        return classifyOrientation(orient2d(p, v1, v2), orientationTolerance);
    }

	static PtTriLocation locatePointTriangle(
		const Vector2& p,
		const Vector2& v1,
		const Vector2& v2,
		const Vector2& v3)
	{
		PtTriLocation result = PtTriLocation::Inside;
		PtLineLocation edgeCheck = locatePointLine(p, v1, v2);
		if (edgeCheck == PtLineLocation::Right)
			return PtTriLocation::Outside;
		if (edgeCheck == PtLineLocation::OnLine)
			result = PtTriLocation::OnEdge1;
		edgeCheck = locatePointLine(p, v2, v3);
		if (edgeCheck == PtLineLocation::Right)
			return PtTriLocation::Outside;
		if (edgeCheck == PtLineLocation::OnLine)
		{
			result = (result == PtTriLocation::Inside) ? PtTriLocation::OnEdge2
				: PtTriLocation::OnVertex;
		}
		edgeCheck = locatePointLine(p, v3, v1);
		if (edgeCheck == PtLineLocation::Right)
			return PtTriLocation::Outside;
		if (edgeCheck == PtLineLocation::OnLine)
		{
			result = (result == PtTriLocation::Inside) ? PtTriLocation::OnEdge3
				: PtTriLocation::OnVertex;
		}
		return result;
	}

	struct EdgeHash
	{
		size_t operator()(const DelaunayEdge& e) const
		{
			return hashEdge(e);
		}

	private:
		static void hashCombine(size_t& seed, const int& key)
		{
			typedef std::hash<int> Hasher;
			seed ^= Hasher()(key) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
		}
		static size_t hashEdge(const DelaunayEdge& vv)
		{
			size_t seed1(0);
			Maths::Hash::HashCombine(seed1, vv.v1, vv.v2);
			size_t seed2(0);
			Maths::Hash::HashCombine(seed2, vv.v2, vv.v1);
			return std::min(seed1, seed2);
		}
	};

	static inline int vertexInd(const DelaunayTriangle& vv, const int iV)
	{
		assert(vv[0] == iV || vv[1] == iV || vv[2] == iV);
		if (vv[0] == iV)
			return 0;
		if (vv[1] == iV)
			return 1;
		return 2;
	}

	static inline int ccw(int i)
    {
        return (i + 1) % 3;
    }

	static inline int cw(int i)
    {
        return (i + 2) % 3;
    }

	static inline bool isOnEdge(const PtTriLocation location)
	{
		return location == PtTriLocation::OnEdge1 ||
			location == PtTriLocation::OnEdge2 ||
			location == PtTriLocation::OnEdge3;
	}

	static inline bool isInCircumcircle(
		const Vector2& p,
		const Vector2& a,
		const Vector2& b,
		const Vector2& c)
	{
		const float dx = a.x - p.x;
		const float dy = a.y - p.y;
		const float ex = b.x - p.x;
		const float ey = b.y - p.y;
		const float fx = c.x - p.x;
		const float fy = c.y - p.y;

		const float ap = dx * dx + dy * dy;
		const float bp = ex * ex + ey * ey;
		const float cp = fx * fx + fy * fy;

		return (dx * (ey * cp - bp * fy) -
			dy * (ex * cp - bp * fx) +
			ap * (ex * fy - ey * fx)) > 0.0;
	}

	static inline int edgeNeighborInd(const DelaunayTriangle& vv, const int iVedge1, const int iVedge2)
	{
		assert(vv[0] == iVedge1 || vv[1] == iVedge1 || vv[2] == iVedge1);
		assert(vv[0] == iVedge2 || vv[1] == iVedge2 || vv[2] == iVedge2);
		assert(
			(vv[0] != iVedge1 && vv[0] != iVedge2) ||
			(vv[1] != iVedge1 && vv[1] != iVedge2) ||
			(vv[2] != iVedge1 && vv[2] != iVedge2));
		/*
		 *      vv[2]
		 *       /\
		 *  n[2]/  \n[1]
		 *     /____\
		 * vv[0] n[0] vv[1]
		 */
		if (vv[0] == iVedge1)
		{
			if (vv[1] == iVedge2)
				return 0;
			return 2;
		}
		if (vv[0] == iVedge2)
		{
			if (vv[1] == iVedge1)
				return 0;
			return 2;
		}
		return 1;
	}

	static inline int edgeNeighbor(const PtTriLocation location)
	{
		assert(isOnEdge(location));
		return (int)location - (int)PtTriLocation::OnEdge1;
	}

	static inline int edgeNeighbor(const Triangle& tri, int iVedge1, int iVedge2)
	{
		return tri.neighbors[edgeNeighborInd(tri.vertices, iVedge1, iVedge2)];
	}

	static inline int opposedVertexInd(const DelaunayTriangle& nn, const int iTopo)
	{
		assert(nn[0] == iTopo || nn[1] == iTopo || nn[2] == iTopo);
		if (nn[0] == iTopo)
			return 2;
		if (nn[1] == iTopo)
			return 0;
		return 1;
	}

	static inline int opposedTriangleInd(const DelaunayTriangle& vv, const int iVert)
	{
		assert(vv[0] == iVert || vv[1] == iVert || vv[2] == iVert);
		if (vv[0] == iVert)
			return 1;
		if (vv[1] == iVert)
			return 2;
		return 0;
	}

	static inline int opposedTriangle(const Triangle& tri, const int iVert)
	{
		return tri.neighbors[opposedTriangleInd(tri.vertices, iVert)];
	}

	static inline int opposedVertex(const Triangle& tri, const int iTopo)
	{
		return tri.vertices[opposedVertexInd(tri.neighbors, iTopo)];
	}

	static inline int opoNbr(const int intex)
    {
        if (intex == 0)
            return 1;
        else if (intex == 1)
            return 2;
        else if (intex == 2)
            return 0;
        assert(false);
        return 0;
    }

    namespace defaults
    {
        const int nTargetVerts = 0;
        const SuperGeometryType superGeomType = SuperGeometryType::SuperTriangle;
        const VertexInsertionOrder vertexInsertionOrder = VertexInsertionOrder::Auto;
        const IntersectingConstraintEdges intersectingEdgesStrategy = IntersectingConstraintEdges::NotAllowed;
        const float minDistToConstraintEdge = 0;
    }

	static Vector2 intersectionPosition(
		const Vector2& a,
		const Vector2& b,
		const Vector2& c,
		const Vector2& d)
	{
		// note: for better accuracy we interpolate x and y separately
		// on a segment with the shortest x/y-projection correspondingly
		const float a_cd = orient2d(c.x, c.y, d.x, d.y, a.x, a.y);
		const float b_cd = orient2d(c.x, c.y, d.x, d.y, b.x, b.y);
		const float t_ab = a_cd / (a_cd - b_cd);

		const float c_ab = orient2d(a.x, a.y, b.x, b.y, c.x, c.y);
		const float d_ab = orient2d(a.x, a.y, b.x, b.y, d.x, d.y);
		const float t_cd = c_ab / (c_ab - d_ab);

		return Vector2(
			std::fabs(a.x - b.x) < std::fabs(c.x - d.x) ? lerp(a.x, b.x, t_ab)
			: lerp(c.x, d.x, t_cd),
			std::fabs(a.y - b.y) < std::fabs(c.y - d.y) ? lerp(a.y, b.y, t_ab)
			: lerp(c.y, d.y, t_cd));
	}

	// add element to 'to' if not already in 'to'
	template <typename Allocator1>
	static void insert_unique(std::vector<Allocator1>& to, const Allocator1& elem)
	{
		if (std::find(to.begin(), to.end(), elem) == to.end())
		{
			to.push_back(elem);
		}
	}

	// add elements of 'from' that are not present in 'to' to 'to'
	template <typename Allocator1, typename Allocator2>
	static void insert_unique(
		std::vector<Allocator1>& to,
		const std::vector<Allocator2>& from)
	{
		typedef typename std::vector<Allocator2>::const_iterator Cit;
		to.reserve(to.size() + from.size());
		for (Cit cit = from.begin(); cit != from.end(); ++cit)
		{
			insert_unique(to, *cit);
		}
	}

	template<typename T>
    class FixedCapacityQueue
    {
    public:
        FixedCapacityQueue(const std::size_t capacity)
            : m_vec(capacity)
            , m_front(m_vec.begin())
            , m_back(m_vec.begin())
            , m_size(0)
        {}
        bool empty() const
        {
            return m_size == 0;
        }
        const T& front() const
        {
            return *m_front;
        }
        void pop()
        {
            assert(m_size > 0);
            ++m_front;
            if (m_front == m_vec.end())
                m_front = m_vec.begin();
            --m_size;
        }
        void push(const T& t)
        {
            assert(m_size < m_vec.size());
            *m_back = t;
            ++m_back;
            if (m_back == m_vec.end())
                m_back = m_vec.begin();
            ++m_size;
        }

    private:
        std::vector<T> m_vec;
        typename std::vector<T>::iterator m_front;
        typename std::vector<T>::iterator m_back;
        std::size_t m_size;
    };

    class less_than_x
    {
        const std::vector<Vector2>& m_vertices;

    public:
        less_than_x(const std::vector<Vector2>& vertices)
            : m_vertices(vertices)
        {}
        bool operator()(const int a, const int b) const
        {
            return m_vertices[a].x < m_vertices[b].x;
        }
    };

    class less_than_y
    {
        const std::vector<Vector2>& m_vertices;

    public:
        less_than_y(const std::vector<Vector2>& vertices)
            : m_vertices(vertices)
        {}
        bool operator()(const int a, const int b) const
        {
            return m_vertices[a].y < m_vertices[b].y;
        }
    };

	class ConstrainedTriangulation
	{
	public:
		std::vector<Vector2> vertices;
		std::vector<Triangle> triangles;
		std::unordered_set<DelaunayEdge, EdgeHash> fixedEdges;
		std::unordered_map<DelaunayEdge, LayerDepth, EdgeHash> overlapCount;
		std::unordered_map<DelaunayEdge, std::vector<DelaunayEdge>, EdgeHash> pieceToOriginals;

		ConstrainedTriangulation()
			: m_nTargetVerts(defaults::nTargetVerts)
			, m_superGeomType(defaults::superGeomType)
			, m_vertexInsertionOrder(defaults::vertexInsertionOrder)
			, m_intersectingEdgesStrategy(defaults::intersectingEdgesStrategy)
			, m_minDistToConstraintEdge(defaults::minDistToConstraintEdge)
		{}

		explicit ConstrainedTriangulation(VertexInsertionOrder vertexInsertionOrder)
			: m_nTargetVerts(defaults::nTargetVerts)
			, m_superGeomType(defaults::superGeomType)
			, m_vertexInsertionOrder(vertexInsertionOrder)
			, m_intersectingEdgesStrategy(defaults::intersectingEdgesStrategy)
			, m_minDistToConstraintEdge(defaults::minDistToConstraintEdge)
		{}

		ConstrainedTriangulation(
			VertexInsertionOrder vertexInsertionOrder,
			IntersectingConstraintEdges intersectingEdgesStrategy,
			float minDistToConstraintEdge)
			: m_nTargetVerts(defaults::nTargetVerts)
			, m_superGeomType(defaults::superGeomType)
			, m_vertexInsertionOrder(vertexInsertionOrder)
			, m_intersectingEdgesStrategy(intersectingEdgesStrategy)
			, m_minDistToConstraintEdge(minDistToConstraintEdge)
		{}

		ConstrainedTriangulation(
			VertexInsertionOrder vertexInsertionOrder,
			const LocatorKDTree<>& nearPtLocator,
			IntersectingConstraintEdges intersectingEdgesStrategy,
			float minDistToConstraintEdge)
			: m_nearPtLocator(nearPtLocator)
			, m_nTargetVerts(defaults::nTargetVerts)
			, m_superGeomType(defaults::superGeomType)
			, m_vertexInsertionOrder(vertexInsertionOrder)
			, m_intersectingEdgesStrategy(intersectingEdgesStrategy)
			, m_minDistToConstraintEdge(minDistToConstraintEdge)
		{}

		void insertVertices(const std::vector<Vector2>& newVertices)
		{
			if (isFinalized())
			{
				assert(false);
				return;
			}

			const bool isFirstTime = vertices.empty();
			Box2 box;
			box.Empty();
			if (isFirstTime) // called first time
			{
				box = Box2(newVertices.data(), newVertices.size());
				addSuperTriangle(box);
			}
			tryInitNearestPointLocator();

			const int nExistingVerts = (int)vertices.size();
			const int nVerts = (int)(nExistingVerts + newVertices.size());
			// optimization, try to pre-allocate tris
			triangles.reserve(triangles.size() + 2 * nVerts);
			vertices.reserve(nVerts);
			m_vertTris.reserve(nVerts);
			for (const Vector2& v : newVertices)
				addNewVertex(v, -1);

			switch (m_vertexInsertionOrder)
			{
			case VertexInsertionOrder::AsProvided:
				insertVertices_AsProvided(nExistingVerts);
				break;
			case VertexInsertionOrder::Auto:
				isFirstTime ? insertVertices_KDTreeBFS(nExistingVerts, box.Min, box.Max)
					: insertVertices_Randomized(nExistingVerts);
				break;
			}
		}

		void insertEdges(const std::vector<DelaunayEdge>& edges)
		{
			if (isFinalized())
			{
				assert(false);
				return;
			}

			std::vector<TriangulatePseudoPolygonTask> tppIterations;
			std::vector<DelaunayEdge> remaining;
			for (const DelaunayEdge& e : edges)
			{
				// +3 to account for super-triangle vertices
				const DelaunayEdge edge(e.v1 + m_nTargetVerts, e.v2 + m_nTargetVerts);
				insertEdge(edge, edge, remaining, tppIterations);
			}
			eraseDummies();
		}

		void conformToEdges(const std::vector<DelaunayEdge>& edges)
		{
			if (isFinalized())
			{
				assert(false);
				return;
			}

			tryInitNearestPointLocator();
			// state shared between different runs for performance gains
			std::vector<ConformToEdgeTask> remaining;
			for (const DelaunayEdge& edge : edges)
			{
				// +3 to account for super-triangle vertices
				const DelaunayEdge e(edge.v1 + m_nTargetVerts, edge.v2 + m_nTargetVerts);
				conformToEdge(e, std::vector<DelaunayEdge>{(1, e)}, 0, remaining);
			}
			eraseDummies();
		}

		std::vector<std::vector<int>> calculateTrianglesByVertex(
			const std::vector<Triangle>& triangles,
			const size_t verticesSize)
		{
			std::vector<std::vector<int>> vertTris(verticesSize);
			for (size_t iT = 0; iT < triangles.size(); ++iT)
			{
				const DelaunayTriangle& vv = triangles[iT].vertices;
				for (int j = 0; j < 3; ++j)
				{
					int vj = vv[j];
					vertTris[vj].push_back((int)iT);
				}
			}
			return vertTris;
		}

		inline bool verifyTopology()
		{
			// Check if vertices' adjacent triangles contain vertex
			const std::vector<std::vector<int>> vertTris = calculateTrianglesByVertex(
				triangles, vertices.size());
			for (int iV = 0; iV < (int)vertices.size(); ++iV)
			{
				const std::vector<int>& vTris = vertTris[iV];
				for (int it : vTris)
				{
					const DelaunayTriangle& vv = triangles[it].vertices;
					if (Vector3i_find(vv, iV) < 0)
						return false;
				}
			}
			// Check if triangle neighbor links are fine
			for (int iT = 0; iT < (int)triangles.size(); ++iT)
			{
				const Triangle& t = triangles[iT];
				for (int k = 0; k < 3; ++k)
				{
					int it = t.neighbors[k];
					if (it == -1)
						continue;
					const DelaunayTriangle& nn = triangles[it].neighbors;
					if (Vector3i_find(nn, iT) < 0)
						return false;
				}
			}
			// Check if triangle's vertices have triangle as adjacent
			for (int iT = 0; iT < (int)triangles.size(); ++iT)
			{
				const Triangle& t = triangles[iT];
				for (int k = 0; k < 3; ++k)
				{
					int it = t.vertices[k];
					const std::vector<int>& tt = vertTris[it];
					if (std::find(tt.begin(), tt.end(), iT) == tt.end())
						return false;
				}
			}
			return true;
		}

		static bool touchesSuperTriangle(const Triangle& t)
		{
			return t.vertices[0] < 3 || t.vertices[1] < 3 || t.vertices[2] < 3;
		}

		void eraseSuperTriangle()
		{
			if (m_superGeomType != SuperGeometryType::SuperTriangle)
				return;
			// find triangles adjacent to super-triangle's vertices
			std::unordered_set<int> toErase;
			for (int iT(0); iT < int(triangles.size()); ++iT)
			{
				if (touchesSuperTriangle(triangles[iT]))
					toErase.insert(iT);
			}
			finalizeTriangulation(toErase);
		}

		void eraseOuterTriangles()
		{
			// make dummy triangles adjacent to super-triangle's vertices
			assert(m_vertTris[0] != -1);
			const std::stack<int> seed(std::deque<int>(1, m_vertTris[0]));
			const std::unordered_set<int> toErase = growToBoundary(seed);
			finalizeTriangulation(toErase);
		}

		void eraseOuterTrianglesAndHoles()
		{
			const std::vector<LayerDepth> triDepths = calculateTriangleDepths();
			std::unordered_set<int> toErase;
			toErase.reserve(triangles.size());
			for (std::size_t iT = 0; iT != triangles.size(); ++iT)
			{
				if (triDepths[iT] % 2 == 0)
					toErase.insert(static_cast<int>(iT));
			}
			finalizeTriangulation(toErase);
		}

		void initializedWithCustomSuperGeometry()
		{
			m_nearPtLocator.initialize(vertices);
			m_nTargetVerts = static_cast<int>(vertices.size());
			m_superGeomType = SuperGeometryType::Custom;
		}

		bool isFinalized() const
		{
			return m_vertTris.empty() && !vertices.empty();
		}

		std::vector<LayerDepth> calculateTriangleDepths() const
		{
			std::vector<LayerDepth> triDepths(
				triangles.size(), std::numeric_limits<LayerDepth>::max());
			std::stack<int> seeds(std::deque<int>(1, m_vertTris[0]));
			LayerDepth layerDepth = 0;
			LayerDepth deepestSeedDepth = 0;

			std::unordered_map<LayerDepth, std::unordered_set<int>> seedsByDepth;
			do
			{
				const std::unordered_map<int, LayerDepth>& newSeeds =
					peelLayer(seeds, layerDepth, triDepths);

				seedsByDepth.erase(layerDepth);
				typedef std::unordered_map<int, LayerDepth>::const_iterator Iter;
				for (Iter it = newSeeds.begin(); it != newSeeds.end(); ++it)
				{
					deepestSeedDepth = std::max(deepestSeedDepth, it->second);
					seedsByDepth[it->second].insert(it->first);
				}
				const std::unordered_set<int>& nextLayerSeeds = seedsByDepth[layerDepth + 1];
				seeds = std::stack<int>(
					std::deque<int>(nextLayerSeeds.begin(), nextLayerSeeds.end()));
				++layerDepth;
			} while (!seeds.empty() || deepestSeedDepth > layerDepth);

			return triDepths;
		}

        /* Flip edge between float and Topo:
         *
         *                v4         | - old edge
         *               /|\         ~ - new edge
         *              / | \
         *          n3 /  float' \ n4
         *            /   |   \
         *           /    |    \
         *     float -> v1~~~~~~~~~v3 <- Topo
         *           \    |    /
         *            \   |   /
         *          n1 \Topo'/ n2
         *              \ | /
         *               \|/
         *                v2
         */
		void flipEdge(int iT, int iTopo)
        {
            Triangle& t = triangles[iT];
            Triangle& tOpo = triangles[iTopo];
            const DelaunayTriangle& triNs = t.neighbors;
            const DelaunayTriangle& triOpoNs = tOpo.neighbors;
            const DelaunayTriangle& triVs = t.vertices;
            const DelaunayTriangle& triOpoVs = tOpo.vertices;
            // find vertices and neighbors
            int i = opposedVertexInd(t.neighbors, iTopo);
            const int v1 = triVs[i];
            const int v2 = triVs[ccw(i)];
            const int n1 = triNs[i];
            const int n3 = triNs[cw(i)];
            i = opposedVertexInd(tOpo.neighbors, iT);
            const int v3 = triOpoVs[i];
            const int v4 = triOpoVs[ccw(i)];
            const int n4 = triOpoNs[i];
            const int n2 = triOpoNs[cw(i)];
            // change vertices and neighbors
            t = Triangle(DelaunayTriangle(v4, v1, v3), DelaunayTriangle(n3, iTopo, n4));
            tOpo = Triangle(DelaunayTriangle(v2, v3, v1), DelaunayTriangle(n2, iT, n1));
            // adjust neighboring triangles and vertices
            changeNeighbor(n1, iT, iTopo);
            changeNeighbor(n4, iTopo, iT);
            // only adjust adjacent triangles if triangulation is not finalized:
            // can happen when called from outside on an already finalized
            // triangulation
            if (!isFinalized())
            {
                setAdjacentTriangle(v4, iT);
                setAdjacentTriangle(v2, iTopo);
            }
        }

        /* Flip edge between float and Topo:
         *
         *                v4         | - old edge
         *               /|\         ~ - new edge
         *              / | \
         *          n3 /  float' \ n4
         *            /   |   \
         *           /    |    \
         *     float -> v1 ~~~~~~~~ v3 <- Topo
         *           \    |    /
         *            \   |   /
         *          n1 \Topo'/ n2
         *              \ | /
         *               \|/
         *                v2
         */
		void flipEdge(
			int iT,
			int iTopo,
			int v1,
			int v2,
			int v3,
			int v4,
			int n1,
			int n2,
			int n3,
			int n4)
        {
            // change vertices and neighbors
            triangles[iT] = Triangle(DelaunayTriangle(v4, v1, v3), DelaunayTriangle(n3, iTopo, n4));
            triangles[iTopo] = Triangle(DelaunayTriangle(v2, v3, v1), DelaunayTriangle(n2, iT, n1));
            // adjust neighboring triangles and vertices
            changeNeighbor(n1, iT, iTopo);
            changeNeighbor(n4, iTopo, iT);
            // only adjust adjacent triangles if triangulation is not finalized:
            // can happen when called from outside on an already finalized
            // triangulation
            if (!isFinalized())
            {
                setAdjacentTriangle(v4, iT);
                setAdjacentTriangle(v2, iTopo);
            }
        }

		void removeTriangles(const std::unordered_set<int>& removedTriangles)
		{
			if (removedTriangles.empty())
				return;
			// remove triangles and calculate triangle index mapping
			std::unordered_map<int, int> intMap;
			for (int iT = 0, iTnew = 0; iT < (int)triangles.size(); ++iT)
			{
				if (removedTriangles.count(iT))
					continue;
				intMap[iT] = iTnew;
				triangles[iTnew] = triangles[iT];
				iTnew++;
			}
			triangles.erase(triangles.end() - removedTriangles.size(), triangles.end());
			// adjust triangles' neighbors
			for (int iT = 0; iT < (int)triangles.size(); ++iT)
			{
				Triangle& t = triangles[iT];
				// update neighbors to account for removed triangles
				DelaunayTriangle& nn = t.neighbors;
				for (int i = 0; i < 3; ++i)
				{
					int &n = nn[i];
					if (removedTriangles.count(n))
					{
						n = -1;
					}
					else if (n != -1)
					{
						n = intMap[n];
					}
				}
			}
		}

		std::vector<int>& VertTrisInternal() { return m_vertTris; }
		const std::vector<int>& VertTrisInternal() const { return m_vertTris; }

	private:

		void addSuperTriangle(const Box2& box)
		{
			m_nTargetVerts = 3;
			m_superGeomType = SuperGeometryType::SuperTriangle;

			const Vector2 center((box.Min.x + box.Max.x) * 0.5f, (box.Min.y + box.Max.y) * 0.5f );
			const float w = box.Max.x - box.Min.x;
			const float h = box.Max.y - box.Min.y;
			float r = std::sqrt(w * w + h * h) * 0.5f; // incircle radius
			r = r > 0 ? r * 1.1f : 1e-6f;

			// Note: for very large floating point numbers rounding can lead to wrong
			// super-triangle coordinates. This is a very rare corner-case so the
			// handling is very primitive.
			{ // note: '<=' means '==' but avoids the warning
				while (center.y <= center.y - r)
					r *= 2.0f;
			}

			const float R = 2.0f * r;                        // excircle radius
			const float shiftX = R * sqrtf(3.0f) * 0.5f; // R * cos(30 deg)
			const Vector2 posV1( center.x - shiftX, center.y - r );
			const Vector2 posV2( center.x + shiftX, center.y - r );
			const Vector2 posV3( center.x, center.y + R );
			addNewVertex(posV1, 0);
			addNewVertex(posV2, 0);
			addNewVertex(posV3, 0);
			const Triangle superTri(DelaunayTriangle(0, 1, 2), DelaunayTriangle(-1, -1, -1));
			addTriangle(superTri);
			if (m_vertexInsertionOrder != VertexInsertionOrder::Auto)
			{
				m_nearPtLocator.initialize(vertices);
			}
		}

		void addNewVertex(const Vector2& pos, const int iT)
		{
			vertices.push_back(pos);
			m_vertTris.push_back(iT);
		}

		void insertVertex(int iVert)
		{
			const Vector2& v = vertices[iVert];
			const int walkStart = m_nearPtLocator.nearPoint(v, vertices);
			insertVertex(iVert, walkStart);
			tryAddVertexToLocator(iVert);
		}

		void insertVertex(int iVert, int walkStart)
		{
			const Index2 trisAt = walkingSearchTrianglesAt(iVert, walkStart);
			std::stack<int> triStack =
				trisAt[1] == -1
				? insertVertexInsideTriangle(iVert, trisAt[0])
				: insertVertexOnEdge(iVert, trisAt[0], trisAt[1]);
			ensureDelaunayByEdgeFlips(iVert, triStack);
		}

		void ensureDelaunayByEdgeFlips(const int iV1, std::stack<int>& triStack)
		{
			int iTopo, n1, n2, n3, n4;
			int iV2, iV3, iV4;
			while (!triStack.empty())
			{
				const int iT = triStack.top();
				triStack.pop();

				edgeFlipInfo(iT, iV1, iTopo, iV2, iV3, iV4, n1, n2, n3, n4);
				if (iTopo != -1 && isFlipNeeded(iV1, iV2, iV3, iV4))
				{
					flipEdge(iT, iTopo, iV1, iV2, iV3, iV4, n1, n2, n3, n4);
					triStack.push(iT);
					triStack.push(iTopo);
				}
			}
		}

		std::vector<DelaunayEdge> insertVertex_FlipFixedEdges(const int iV1)
		{
			std::vector<DelaunayEdge> flippedFixedEdges;

			const Vector2& v1 = vertices[iV1];
			const int startVertex = m_nearPtLocator.nearPoint(v1, vertices);
			Index2 trisAt = walkingSearchTrianglesAt(iV1, startVertex);
			std::stack<int> triStack =
				trisAt[1] == -1 ? insertVertexInsideTriangle(iV1, trisAt[0])
				: insertVertexOnEdge(iV1, trisAt[0], trisAt[1]);

			int iTopo, n1, n2, n3, n4;
			int iV2, iV3, iV4;
			while (!triStack.empty())
			{
				const int iT = triStack.top();
				triStack.pop();

				edgeFlipInfo(iT, iV1, iTopo, iV2, iV3, iV4, n1, n2, n3, n4);
				if (iTopo != -1 && isFlipNeeded(iV1, iV2, iV3, iV4))
				{
					// if flipped edge is fixed, remember it
					const DelaunayEdge flippedEdge(iV2, iV4);
					if (!fixedEdges.empty() &&
						fixedEdges.find(flippedEdge) != fixedEdges.end())
					{
						flippedFixedEdges.push_back(flippedEdge);
					}

					flipEdge(iT, iTopo, iV1, iV2, iV3, iV4, n1, n2, n3, n4);
					triStack.push(iT);
					triStack.push(iTopo);
				}
			}

			tryAddVertexToLocator(iV1);
			return flippedFixedEdges;
		}

		typedef std::tuple<int, int, int, int, int> TriangulatePseudoPolygonTask;

		void insertEdge(DelaunayEdge edge, const DelaunayEdge originalEdge, std::vector<DelaunayEdge>& remaining, std::vector<TriangulatePseudoPolygonTask>& tppIterations)
		{
			// use iteration over recursion to avoid stack overflows
			remaining.clear();
			remaining.push_back(edge);
			while (!remaining.empty())
			{
				edge = remaining.back();
				remaining.pop_back();
				insertEdgeIteration(edge, originalEdge, remaining, tppIterations);
			}
		}

		void insertEdgeIteration(const DelaunayEdge edge, const DelaunayEdge originalEdge, std::vector<DelaunayEdge>& remaining, std::vector<TriangulatePseudoPolygonTask>& tppIterations)
		{
			const int iA = edge.v1;
			int iB = edge.v2;
			if (iA == iB) // edge connects a vertex to itself
				return;

			if (hasEdge(iA, iB))
			{
				fixEdge(edge, originalEdge);
				return;
			}

			const Vector2& a = vertices[iA];
			const Vector2& b = vertices[iB];
			const float distanceTolerance =
				m_minDistToConstraintEdge == float(0)
				? float(0)
				: m_minDistToConstraintEdge * (a - b).Length();

			DelaunayTriangle x = intersectedTriangle(iA, a, b, distanceTolerance);

			int iT = x.v1;
			int iVL = x.v2;
			int iVR = x.v3;

			// if one of the triangle vertices is on the edge, move edge start
			if (iT == -1)
			{
				const DelaunayEdge edgePart(iA, iVL);
				fixEdge(edgePart, originalEdge);
				remaining.push_back(DelaunayEdge(iVL, iB));
				return;
			}
			Triangle t = triangles[iT];
			std::vector<int> intersected(1, iT);
			std::vector<int> polyL, polyR;
			polyL.reserve(2);
			polyL.push_back(iA);
			polyL.push_back(iVL);
			polyR.reserve(2);
			polyR.push_back(iA);
			polyR.push_back(iVR);
			std::unordered_map<DelaunayEdge, int, EdgeHash> outerTris;
			outerTris[DelaunayEdge(iA, iVL)] = edgeNeighbor(t, iA, iVL);
			outerTris[DelaunayEdge(iA, iVR)] = edgeNeighbor(t, iA, iVR);
			int iV = iA;

			while (!t.containsVertex(iB))
			{
				const int iTopo = opposedTriangle(t, iV);
				const Triangle& tOpo = triangles[iTopo];
				const int iVopo = opposedVertex(tOpo, iT);

				switch (m_intersectingEdgesStrategy)
				{
				case IntersectingConstraintEdges::NotAllowed:
					if (fixedEdges.count(DelaunayEdge(iVL, iVR)))
					{
						// make sure to report original input edges in the exception
						DelaunayEdge e1 = originalEdge;
						DelaunayEdge e2 = DelaunayEdge(iVL, iVR);
						e2 = pieceToOriginals.count(e2)
							? pieceToOriginals.at(e2).front()
							: e2;
						// don't count super-triangle vertices
						e1 = DelaunayEdge(e1.v1 - m_nTargetVerts, e1.v2 - m_nTargetVerts);
						e2 = DelaunayEdge(e2.v1 - m_nTargetVerts, e2.v2 - m_nTargetVerts);
						assert(false);
					}
					break;
				case IntersectingConstraintEdges::TryResolve: {
					if (!fixedEdges.count(DelaunayEdge(iVL, iVR)))
						break;
					// split edge at the intersection of two constraint edges
					const Vector2 newV = intersectionPosition(
						vertices[iA], vertices[iB], vertices[iVL], vertices[iVR]);
					const int iNewVert =
						splitFixedEdgeAt(DelaunayEdge(iVL, iVR), newV, iT, iTopo);
					// TODO: is it's possible to re-use pseudo-polygons
					//  for inserting [iA, iNewVert] edge half?
					remaining.push_back(DelaunayEdge(iA, iNewVert));
					remaining.push_back(DelaunayEdge(iNewVert, iB));
					return;
				}
				case IntersectingConstraintEdges::DontCheck:
					assert(!fixedEdges.count(DelaunayEdge(iVL, iVR)));
					break;
				}

				const PtLineLocation loc =
					locatePointLine(vertices[iVopo], a, b, distanceTolerance);
				if (loc == PtLineLocation::Left)
				{
					const DelaunayEdge e(polyL.back(), iVopo);
					const int outer = edgeNeighbor(tOpo, e.v1, e.v2);
					if (!outerTris.insert(std::make_pair(e, outer)).second)
						outerTris.at(e) = -1; // hanging edge detected
					polyL.push_back(iVopo);
					iV = iVL;
					iVL = iVopo;
				}
				else if (loc == PtLineLocation::Right)
				{
					const DelaunayEdge e(polyR.back(), iVopo);
					const int outer = edgeNeighbor(tOpo, e.v1, e.v2);
					if (!outerTris.insert(std::make_pair(e, outer)).second)
						outerTris.at(e) = -1; // hanging edge detected
					polyR.push_back(iVopo);
					iV = iVR;
					iVR = iVopo;
				}
				else // encountered point on the edge
					iB = iVopo;

				intersected.push_back(iTopo);
				iT = iTopo;
				t = triangles[iT];
			}
			outerTris[DelaunayEdge(polyL.back(), iB)] = edgeNeighbor(t, polyL.back(), iB);
			outerTris[DelaunayEdge(polyR.back(), iB)] = edgeNeighbor(t, polyR.back(), iB);
			polyL.push_back(iB);
			polyR.push_back(iB);

			assert(!intersected.empty());
			// make sure start/end vertices have a valid adjacent triangle
			// that is not intersected by an edge
			if (m_vertTris[iA] == intersected.front())
				pivotVertexTriangleCW(iA);
			if (m_vertTris[iB] == intersected.back())
				pivotVertexTriangleCW(iB);
			// Remove intersected triangles
			typedef std::vector<int>::const_iterator intCit;
			for (intCit it = intersected.begin(); it != intersected.end(); ++it)
				makeDummy(*it);
			{ // Triangulate pseudo-polygons on both sides
				std::reverse(polyR.begin(), polyR.end());
				const int iTL = addTriangle();
				const int iTR = addTriangle();
				triangulatePseudoPolygon(polyL, outerTris, iTL, iTR, tppIterations);
				triangulatePseudoPolygon(polyR, outerTris, iTR, iTL, tppIterations);
			}

			if (iB != edge.v2) // encountered point on the edge
			{
				// fix edge part
				const DelaunayEdge edgePart(iA, iB);
				fixEdge(edgePart, originalEdge);
				remaining.push_back(DelaunayEdge(iB, edge.v2));
				return;
			}
			else
			{
				fixEdge(edge, originalEdge);
			}
		}

		typedef std::tuple<DelaunayEdge, std::vector<DelaunayEdge>, LayerDepth> ConformToEdgeTask;

		void conformToEdge(DelaunayEdge edge, std::vector<DelaunayEdge> originals, LayerDepth overlaps, std::vector<ConformToEdgeTask>& remaining)
		{
			// use iteration over recursion to avoid stack overflows
			remaining.clear();
			remaining.emplace_back(edge, originals, overlaps);
			while (!remaining.empty())
			{
				tie(edge, originals, overlaps) = remaining.back();
				remaining.pop_back();
				conformToEdgeIteration(edge, originals, overlaps, remaining);
			}
		}

		static int Vector3i_find(const DelaunayTriangle& nn, int idx)
		{
			for (int i = 0; i < 3; ++i)
			{
				if (nn[i] == idx)
				{
					return i;
				}
			}
			return -1;
		}

		void conformToEdgeIteration(DelaunayEdge edge, const std::vector<DelaunayEdge>& originals, LayerDepth overlaps, std::vector<ConformToEdgeTask>& remaining)
		{
			const int iA = edge.v1;
			int iB = edge.v2;
			if (iA == iB) // edge connects a vertex to itself
				return;

			if (hasEdge(iA, iB))
			{
				fixEdge(edge);
				if (overlaps > 0)
					overlapCount[edge] = overlaps;
				// avoid marking edge as a part of itself
				if (!originals.empty() && edge != originals.front())
				{
					insert_unique(pieceToOriginals[edge], originals);
				}
				return;
			}

			const Vector2& a = vertices[iA];
			const Vector2& b = vertices[iB];
			const float distanceTolerance =
				m_minDistToConstraintEdge == float(0)
				? float(0)
				: m_minDistToConstraintEdge * (a - b).Length();

			DelaunayTriangle xx = intersectedTriangle(iA, a, b, distanceTolerance);
			int iT = xx.v1;
			int iVleft = xx.v2;
			int iVright = xx.v3;

			// if one of the triangle vertices is on the edge, move edge start
			if (iT == -1)
			{
				const DelaunayEdge edgePart(iA, iVleft);
				fixEdge(edgePart);
				if (overlaps > 0)
					overlapCount[edgePart] = overlaps;
				insert_unique(pieceToOriginals[edgePart], originals);
				remaining.emplace_back(DelaunayEdge(iVleft, iB), originals, overlaps);
				return;
			}

			int iV = iA;
			Triangle t = triangles[iT];
			while (Vector3i_find(t.vertices, iB) >= 0)
			{
				const int iTopo = opposedTriangle(t, iV);
				const Triangle& tOpo = triangles[iTopo];
				const int iVopo = opposedVertex(tOpo, iT);
				const Vector2 vOpo = vertices[iVopo];

				switch (m_intersectingEdgesStrategy)
				{
				case IntersectingConstraintEdges::NotAllowed:
					if (fixedEdges.count(DelaunayEdge(iVleft, iVright)))
					{
						// make sure to report original input edges in the exception
						DelaunayEdge e1 = pieceToOriginals.count(edge)
							? pieceToOriginals.at(edge).front()
							: edge;
						DelaunayEdge e2(iVleft, iVright);
						e2 = pieceToOriginals.count(e2)
							? pieceToOriginals.at(e2).front()
							: e2;
						// don't count super-triangle vertices
						e1 = DelaunayEdge(e1.v1 - m_nTargetVerts, e1.v2 - m_nTargetVerts);
						e2 = DelaunayEdge(e2.v1 - m_nTargetVerts, e2.v2 - m_nTargetVerts);

						assert(false);
					}
					break;
				case IntersectingConstraintEdges::TryResolve: {
					if (!fixedEdges.count(DelaunayEdge(iVleft, iVright)))
						break;
					// split edge at the intersection of two constraint edges
					const Vector2 newV = intersectionPosition(
						vertices[iA],
						vertices[iB],
						vertices[iVleft],
						vertices[iVright]);
					const int iNewVert =
						splitFixedEdgeAt(DelaunayEdge(iVleft, iVright), newV, iT, iTopo);
					remaining.emplace_back(DelaunayEdge(iNewVert, iB), originals, overlaps);
					remaining.emplace_back(DelaunayEdge(iA, iNewVert), originals, overlaps);
					return;
				}
				case IntersectingConstraintEdges::DontCheck:
					assert(!fixedEdges.count(DelaunayEdge(iVleft, iVright)));
					break;
				}

				iT = iTopo;
				t = triangles[iT];

				const PtLineLocation loc =
					locatePointLine(vOpo, a, b, distanceTolerance);
				if (loc == PtLineLocation::Left)
				{
					iV = iVleft;
					iVleft = iVopo;
				}
				else if (loc == PtLineLocation::Right)
				{
					iV = iVright;
					iVright = iVopo;
				}
				else // encountered point on the edge
					iB = iVopo;
			}

			// encountered one or more points on the edge: add remaining edge part
			if (iB != edge.v2)
			{
				remaining.emplace_back(DelaunayEdge(iB, edge.v2), originals, overlaps);
			}

			// add mid-point to triangulation
			const int iMid = static_cast<int>(vertices.size());
			const Vector2& start = vertices[iA];
			const Vector2& end = vertices[iB];
			addNewVertex(
				Vector2((start.x + end.x) * 0.5f, (start.y + end.y) * 0.5f),
				-1);
			const std::vector<DelaunayEdge> flippedFixedEdges =
				insertVertex_FlipFixedEdges(iMid);

			remaining.emplace_back(DelaunayEdge(iMid, iB), originals, overlaps);
			remaining.emplace_back(DelaunayEdge(iA, iMid), originals, overlaps);

			// re-introduce fixed edges that were flipped
			// and make sure overlap count is preserved
			for (std::vector<DelaunayEdge>::const_iterator it = flippedFixedEdges.begin();
				it != flippedFixedEdges.end();
				++it)
			{
				const DelaunayEdge& flippedFixedEdge = *it;
				fixedEdges.erase(flippedFixedEdge);

				LayerDepth prevOverlaps = 0;
				const auto overlapsIt = overlapCount.find(flippedFixedEdge);
				if (overlapsIt != overlapCount.end())
				{
					prevOverlaps = overlapsIt->second;
					overlapCount.erase(overlapsIt);
				}
				// override overlapping boundaries count when re-inserting an edge
				std::vector<DelaunayEdge> prevOriginals(1, flippedFixedEdge);
				const auto originalsIt = pieceToOriginals.find(flippedFixedEdge);
				if (originalsIt != pieceToOriginals.end())
				{
					prevOriginals = originalsIt->second;
				}
				remaining.emplace_back(flippedFixedEdge, prevOriginals, prevOverlaps);
			}
		}

		DelaunayTriangle intersectedTriangle(const int iA, const Vector2& a, const Vector2& b, const float orientationTolerance = 1e-6f) const
		{
			const int startTri = m_vertTris[iA];
			int iT = startTri;
			do
			{
				const Triangle t = triangles[iT];
				const int i = vertexInd(t.vertices, iA);
				const int iP2 = t.vertices[ccw(i)];
				const float orientP2 = orient2d(vertices[iP2], a, b);
				const PtLineLocation locP2 = classifyOrientation(orientP2);
				if (locP2 == PtLineLocation::Right)
				{
					const int iP1 = t.vertices[cw(i)];
					const float orientP1 = orient2d(vertices[iP1], a, b);
					const PtLineLocation locP1 = classifyOrientation(orientP1);
					if (locP1 == PtLineLocation::OnLine)
					{
						return DelaunayTriangle(-1, iP1, iP1);
					}
					if (locP1 == PtLineLocation::Left)
					{
						if (orientationTolerance)
						{
							float closestOrient;
							int iClosestP;
							if (std::abs(orientP1) <= std::abs(orientP2))
							{
								closestOrient = orientP1;
								iClosestP = iP1;
							}
							else
							{
								closestOrient = orientP2;
								iClosestP = iP2;
							}
							if (classifyOrientation(
								closestOrient, orientationTolerance) ==
								PtLineLocation::OnLine)
							{
								return DelaunayTriangle(-1, iClosestP, iClosestP);
							}
						}
						return DelaunayTriangle(iT, iP1, iP2);
					}
				}
				iT = t.next(iA).first;
			} while (iT != startTri);

			assert(false);
			return DelaunayTriangle(-1, -1, -1);
		}

        /* Insert point into triangle: split into 3 triangles:
         *  - create 2 new triangles
         *  - re-use old triangle for the 3rd
         *                      v3
         *                    / | \
         *                   /  |  \ <-- original triangle (t)
         *                  /   |   \
         *              n3 /    |    \ n2
         *                /newT2|newT1\
         *               /      v      \
         *              /    __/ \__    \
         *             /  __/       \__  \
         *            / _/      t'     \_ \
         *          v1 ___________________ v2
         *                     n1
         */
		std::stack<int> insertVertexInsideTriangle(int v, int iT)
        {
            const int iNewT1 = addTriangle();
            const int iNewT2 = addTriangle();

            Triangle& t = triangles[iT];
            const DelaunayTriangle vv = t.vertices;
            const DelaunayTriangle nn = t.neighbors;
            const int v1 = vv[0], v2 = vv[1], v3 = vv[2];
            const int n1 = nn[0], n2 = nn[1], n3 = nn[2];
            // make two new triangles and convert current triangle to 3rd new
            // triangle
            triangles[iNewT1] = Triangle(DelaunayTriangle(v2, v3, v), DelaunayTriangle(n2, iNewT2, iT));
            triangles[iNewT2] = Triangle(DelaunayTriangle(v3, v1, v), DelaunayTriangle(n3, iT, iNewT1));
            t = Triangle(DelaunayTriangle(v1, v2, v), DelaunayTriangle(n1, iNewT1, iNewT2));
            // adjust adjacent triangles
            setAdjacentTriangle(v, iT);
            setAdjacentTriangle(v3, iNewT1);
            // change triangle neighbor's neighbors to new triangles
            changeNeighbor(n2, iT, iNewT1);
            changeNeighbor(n3, iT, iNewT2);
            // return newly added triangles
            std::stack<int> newTriangles;
            newTriangles.push(iT);
            newTriangles.push(iNewT1);
            newTriangles.push(iNewT2);
            return newTriangles;
        }

        /* Inserting a point on the edge between two triangles
         *    T1 (top)        v1
         *                   /|\
         *              n1 /  |  \ n4
         *               /    |    \
         *             /  T1' | Tnew1\
         *           v2-------v-------v4
         *             \  T2' | Tnew2/
         *               \    |    /
         *              n2 \  |  / n3
         *                   \|/
         *   T2 (bottom)      v3
         */
		std::stack<int> insertVertexOnEdge(int v, int iT1, int iT2)
        {
            const int iTnew1 = addTriangle();
            const int iTnew2 = addTriangle();

            Triangle& t1 = triangles[iT1];
            Triangle& t2 = triangles[iT2];
            int i = opposedVertexInd(t1.neighbors, iT2);
            const int v1 = t1.vertices[i];
            const int v2 = t1.vertices[ccw(i)];
            const int n1 = t1.neighbors[i];
            const int n4 = t1.neighbors[cw(i)];
            i = opposedVertexInd(t2.neighbors, iT1);
            const int v3 = t2.vertices[i];
            const int v4 = t2.vertices[ccw(i)];
            const int n3 = t2.neighbors[i];
            const int n2 = t2.neighbors[cw(i)];
            // add new triangles and change existing ones
            t1 = Triangle(DelaunayTriangle(v, v1, v2), DelaunayTriangle(iTnew1, n1, iT2));
            t2 = Triangle(DelaunayTriangle(v, v2, v3), DelaunayTriangle(iT1, n2, iTnew2));
            triangles[iTnew1] = Triangle(DelaunayTriangle(v, v4, v1), DelaunayTriangle(iTnew2, n4, iT1));
            triangles[iTnew2] = Triangle(DelaunayTriangle(v, v3, v4), DelaunayTriangle(iT2, n3, iTnew1));
            // adjust adjacent triangles
            setAdjacentTriangle(v, iT1);
            setAdjacentTriangle(v4, iTnew1);
            // adjust neighboring triangles and vertices
            changeNeighbor(n4, iT1, iTnew1);
            changeNeighbor(n3, iT2, iTnew2);
            // return newly added triangles
            std::stack<int> newTriangles;
            newTriangles.push(iT1);
            newTriangles.push(iTnew2);
            newTriangles.push(iT2);
            newTriangles.push(iTnew1);
            return newTriangles;
        }

		Index2 trianglesAt(const Vector2& pos) const
		{
			Index2 out(-1, -1);
			for (int i = 0; i < int(triangles.size()); ++i)
			{
				const Triangle& t = triangles[i];
				const Vector2& v1 = vertices[t.vertices[0]];
				const Vector2& v2 = vertices[t.vertices[1]];
				const Vector2& v3 = vertices[t.vertices[2]];
				const PtTriLocation loc = locatePointTriangle(pos, v1, v2, v3);
				if (loc == PtTriLocation::Outside)
					continue;
				out[0] = i;
				if (isOnEdge(loc))
					out[1] = t.neighbors[edgeNeighbor(loc)];
				return out;
			}
			assert(false);
		}

		Index2 walkingSearchTrianglesAt(const int iV, const int startVertex) const
		{
			const Vector2 v = vertices[iV];
			Index2 out(-1, -1);
			const int iT = walkTriangles(startVertex, v);
			// Finished walk, locate point in current triangle
			const Triangle& t = triangles[iT];
			const Vector2& v1 = vertices[t.vertices[0]];
			const Vector2& v2 = vertices[t.vertices[1]];
			const Vector2& v3 = vertices[t.vertices[2]];
			const PtTriLocation loc = locatePointTriangle(v, v1, v2, v3);

			if (loc == PtTriLocation::Outside)
				assert(false);
			if (loc == PtTriLocation::OnVertex)
			{
				const int iDupe = v1 == v ? t.vertices[0]
					: v2 == v ? t.vertices[1]
					: t.vertices[2];
                (void)iDupe;
				assert(false);
			}

			out[0] = iT;
			if (isOnEdge(loc))
				out[1] = t.neighbors[edgeNeighbor(loc)];
			return out;
		}

		int walkTriangles(const int startVertex, const Vector2& pos) const
		{
			// begin walk in search of triangle at pos
			int currTri = m_vertTris[startVertex];
			bool found = false;
			Maths::SplitMix64RandGen prng;
			while (!found)
			{
				const Triangle& t = triangles[currTri];
				found = true;
				// stochastic offset to randomize which edge we check first
				const int offset = prng() % 3;
				for (int i_ = 0; i_ < 3; ++i_)
				{
					const int i((i_ + offset) % 3);
					const Vector2& vStart = vertices[t.vertices[i]];
					const Vector2& vEnd = vertices[t.vertices[ccw(i)]];
					const PtLineLocation edgeCheck =
						locatePointLine(pos, vStart, vEnd);
					const int iN = t.neighbors[i];
					if (edgeCheck == PtLineLocation::Right && iN != -1)
					{
						found = false;
						currTri = iN;
						break;
					}
				}
			}
			return currTri;
		}

		/*
		 *                       v4         original edge: (v1, v3)
		 *                      /|\   flip-candidate edge: (v,  v2)
		 *                    /  |  \
		 *              n3  /    |    \  n4
		 *                /      |      \
		 * new vertex--> v1    float | Topo  v3
		 *                \      |      /
		 *              n1  \    |    /  n2
		 *                    \  |  /
		 *                      \|/
		 *                       v2
		 */
		void edgeFlipInfo(
			int iT,
			int iV1,
			int& iTopo,
			int& iV2,
			int& iV3,
			int& iV4,
			int& n1,
			int& n2,
			int& n3,
			int& n4)
		{
			/*     v[2]
				   / \
			  n[2]/   \n[1]
				 /_____\
			v[0]  n[0]  v[1]  */
			const Triangle& t = triangles[iT];
			if (t.vertices[0] == iV1)
			{
				iV2 = t.vertices[1];
				iV4 = t.vertices[2];
				n1 = t.neighbors[0];
				n3 = t.neighbors[2];
				iTopo = t.neighbors[1];
			}
			else if (t.vertices[1] == iV1)
			{
				iV2 = t.vertices[2];
				iV4 = t.vertices[0];
				n1 = t.neighbors[1];
				n3 = t.neighbors[0];
				iTopo = t.neighbors[2];
			}
			else
			{
				iV2 = t.vertices[0];
				iV4 = t.vertices[1];
				n1 = t.neighbors[2];
				n3 = t.neighbors[1];
				iTopo = t.neighbors[0];
			}
			if (iTopo == -1)
				return;
			const Triangle& tOpo = triangles[iTopo];
			if (tOpo.neighbors[0] == iT)
			{
				iV3 = tOpo.vertices[2];
				n2 = tOpo.neighbors[1];
				n4 = tOpo.neighbors[2];
			}
			else if (tOpo.neighbors[1] == iT)
			{
				iV3 = tOpo.vertices[0];
				n2 = tOpo.neighbors[2];
				n4 = tOpo.neighbors[0];
			}
			else
			{
				iV3 = tOpo.vertices[1];
				n2 = tOpo.neighbors[0];
				n4 = tOpo.neighbors[1];
			}
		}

        /*!
         * Handles super-triangle vertices.
         * Super-tri points are not infinitely far and influence the input points
         * Three cases are possible:
         *  1.  If one of the opposed vertices is super-tri: no flip needed
         *  2.  One of the shared vertices is super-tri:
         *      check if on point is same side of line formed by non-super-tri
         * vertices as the non-super-tri shared vertex
         *  3.  None of the vertices are super-tri: normal circumcircle test
         */
         /*
          *                       v4         original edge: (v2, v4)
          *                      /|\   flip-candidate edge: (v1, v3)
          *                    /  |  \
          *                  /    |    \
          *                /      |      \
          * new vertex--> v1      |       v3
          *                \      |      /
          *                  \    |    /
          *                    \  |  /
          *                      \|/
          *                       v2
          */
		bool isFlipNeeded(const int iV1, const int iV2, const int iV3, const int iV4) const
		{
			if (fixedEdges.count(DelaunayEdge(iV2, iV4)))
				return false; // flip not needed if the original edge is fixed
			const Vector2& v1 = vertices[iV1];
			const Vector2& v2 = vertices[iV2];
			const Vector2& v3 = vertices[iV3];
			const Vector2& v4 = vertices[iV4];
			if (m_superGeomType == SuperGeometryType::SuperTriangle)
			{
				// If flip-candidate edge touches super-triangle in-circumference
				// test has to be replaced with orient2d test against the line
				// formed by two non-artificial vertices (that don't belong to
				// super-triangle)
				if (iV1 < 3) // flip-candidate edge touches super-triangle
				{
					// does original edge also touch super-triangle?
					if (iV2 < 3)
						return locatePointLine(v2, v3, v4) ==
						locatePointLine(v1, v3, v4);
					if (iV4 < 3)
						return locatePointLine(v4, v2, v3) ==
						locatePointLine(v1, v2, v3);
					return false; // original edge does not touch super-triangle
				}
				if (iV3 < 3) // flip-candidate edge touches super-triangle
				{
					// does original edge also touch super-triangle?
					if (iV2 < 3)
					{
						return locatePointLine(v2, v1, v4) ==
							locatePointLine(v3, v1, v4);
					}
					if (iV4 < 3)
					{
						return locatePointLine(v4, v2, v1) ==
							locatePointLine(v3, v2, v1);
					}
					return false; // original edge does not touch super-triangle
				}
				// flip-candidate edge does not touch super-triangle
				if (iV2 < 3)
					return locatePointLine(v2, v3, v4) == locatePointLine(v1, v3, v4);
				if (iV4 < 3)
					return locatePointLine(v4, v2, v3) == locatePointLine(v1, v2, v3);
			}
			return isInCircumcircle(v1, v2, v3, v4);
		}

		void changeNeighbor(int iT, int oldNeighbor, int newNeighbor)
		{
			if (iT == -1)
				return;
			DelaunayTriangle& nn = triangles[iT].neighbors;
			assert(
				nn[0] == oldNeighbor || nn[1] == oldNeighbor || nn[2] == oldNeighbor);
			if (nn[0] == oldNeighbor)
				nn[0] = newNeighbor;
			else if (nn[1] == oldNeighbor)
				nn[1] = newNeighbor;
			else
				nn[2] = newNeighbor;
		}

		void changeNeighbor(
			int iT,
			int iVedge1,
			int iVedge2,
			int newNeighbor)
		{
			assert(iT != -1);
			Triangle& t = triangles[iT];
			t.neighbors[edgeNeighborInd(t.vertices, iVedge1, iVedge2)] = newNeighbor;
		}

		void triangulatePseudoPolygon(const std::vector<int>& poly, std::unordered_map<DelaunayEdge, int, EdgeHash>& outerTris, int iT, int iN, std::vector<TriangulatePseudoPolygonTask>& iterations)
		{
			assert(poly.size() > 2);
			// note: uses iteration instead of recursion to avoid stack overflows
			iterations.clear();
			iterations.emplace_back(
				0,
				static_cast<int>(poly.size() - 1),
				iT,
				iN,
				0);
			while (!iterations.empty())
			{
				triangulatePseudoPolygonIteration(poly, outerTris, iterations);
			}
		}

		void triangulatePseudoPolygonIteration(const std::vector<int>& poly, std::unordered_map<DelaunayEdge, int, EdgeHash>& outerTris, std::vector<TriangulatePseudoPolygonTask>& iterations)
		{
			int iA, iB;
			int iT, iParent;
			int iInParent;
			assert(!iterations.empty());
			std::tie(iA, iB, iT, iParent, iInParent) = iterations.back();
			iterations.pop_back();
			assert(iB - iA > 1 && iT != -1 && iParent != -1);
			Triangle& t = triangles[iT];
			// find Delaunay point
			const int iC = findDelaunayPoint(poly, iA, iB);

			const int a = poly[iA];
			const int b = poly[iB];
			const int c = poly[iC];

			// split pseudo-polygon in two parts and triangulate them
			// note: second part needs to be pushed on stack first to be processed first

			// second part: points after the Delaunay point
			if (iB - iC > 1)
			{
				const int iNext = addTriangle();
				iterations.push_back(std::make_tuple(iC, iB, iNext, iT, int(1)));
			}
			else // pseudo-poly is reduced to a single outer edge
			{
				const DelaunayEdge outerEdge(b, c);
				const int outerTri = outerTris.at(outerEdge);
				if (outerTri != -1)
				{
					assert(outerTri != iT);
					t.neighbors[1] = outerTri;
					changeNeighbor(outerTri, c, b, iT);
				}
				else
					outerTris.at(outerEdge) = iT;
			}
			// first part: points before the Delaunay point
			if (iC - iA > 1)
			{ // add next triangle and add another iteration
				const int iNext = addTriangle();
				iterations.push_back(std::make_tuple(iA, iC, iNext, iT, int(2)));
			}
			else
			{ // pseudo-poly is reduced to a single outer edge
				const DelaunayEdge outerEdge(c, a);
				const int outerTri = outerTris.at(outerEdge);
				if (outerTri != -1)
				{
					assert(outerTri != iT);
					t.neighbors[2] = outerTri;
					changeNeighbor(outerTri, c, a, iT);
				}
				else
					outerTris.at(outerEdge) = iT;
			}
			// Finalize triangle
			// note: only when triangle is finalized to we add it as a neighbor to
			// parent to maintain triangulation topology consistency
			triangles[iParent].neighbors[iInParent] = iT;
			t.neighbors[0] = iParent;
			t.vertices = DelaunayTriangle(a, b, c);
			setAdjacentTriangle(c, iT);
		}

		int findDelaunayPoint(const std::vector<int>& poly, const int iA, const int iB) const
		{
			assert(iB - iA > 1);
			const Vector2& a = vertices[poly[iA]];
			const Vector2& b = vertices[poly[iB]];
			int out = iA + 1;
			const Vector2* c = &vertices[poly[out]]; // caching for better performance
			for (int i = iA + 1; i < iB; ++i)
			{
				const Vector2& v = vertices[poly[i]];
				if (isInCircumcircle(v, a, b, *c))
				{
					out = i;
					c = &v;
				}
			}
			assert(out > iA && out < iB); // point is between ends
			return out;
		}

		int addTriangle(const Triangle& t)
		{
			if (m_dummyTris.empty())
			{
				triangles.push_back(t);
				return int(triangles.size() - 1);
			}
			const int nxtDummy = m_dummyTris.back();
			m_dummyTris.pop_back();
			triangles[nxtDummy] = t;
			return nxtDummy;
		}

		int addTriangle()
		{
			if (m_dummyTris.empty())
			{
				const Triangle dummy(DelaunayTriangle(-1, -1, -1), DelaunayTriangle(-1, -1, -1));
				triangles.push_back(dummy);
				return int(triangles.size() - 1);
			}
			const int nxtDummy = m_dummyTris.back();
			m_dummyTris.pop_back();
			return nxtDummy;
		}

		static DelaunayEdge RemapNoSuperTriangle(const DelaunayEdge& e)
		{
			return DelaunayEdge(e.v1 - 3, e.v2 - 3);
		}

		void finalizeTriangulation(const std::unordered_set<int>& removedTriangles)
		{
			eraseDummies();
			m_vertTris = std::vector<int>();
			// remove super-triangle
			if (m_superGeomType == SuperGeometryType::SuperTriangle)
			{
				vertices.erase(vertices.begin(), vertices.begin() + 3);

				// fixed edges
				{
					std::unordered_set<DelaunayEdge, EdgeHash> updatedFixedEdges;
					for (auto e = fixedEdges.begin(); e != fixedEdges.end(); ++e)
					{
						updatedFixedEdges.insert(RemapNoSuperTriangle(*e));
					}
					fixedEdges = updatedFixedEdges;
				}

				// overlap count
				{
					std::unordered_map<DelaunayEdge, LayerDepth, EdgeHash> updatedOverlapCount;
					for (auto it = overlapCount.begin(); it != overlapCount.end(); ++it)
					{
						updatedOverlapCount.insert(std::make_pair(
							RemapNoSuperTriangle(it->first), it->second));
					}
					overlapCount = updatedOverlapCount;
				}
				{ // split edges mapping
					std::unordered_map<DelaunayEdge, std::vector<DelaunayEdge>, EdgeHash> updatedPieceToOriginals;
					for (auto it = pieceToOriginals.begin(); it != pieceToOriginals.end();
						++it)
					{
						std::vector<DelaunayEdge> ee = it->second;
						for (std::vector<DelaunayEdge>::iterator eeIt = ee.begin(); eeIt != ee.end();
							++eeIt)
						{
							*eeIt = RemapNoSuperTriangle(*eeIt);
						}
						updatedPieceToOriginals.insert(
							std::make_pair(RemapNoSuperTriangle(it->first), ee));
					}
					pieceToOriginals = updatedPieceToOriginals;
				}
			}
			// remove other triangles
			removeTriangles(removedTriangles);
			// adjust triangle vertices: account for removed super-triangle
			if (m_superGeomType == SuperGeometryType::SuperTriangle)
			{
				for (std::vector<Triangle>::iterator t = triangles.begin(); t != triangles.end(); ++t)
				{
					DelaunayTriangle& vv = t->vertices;
					vv.v1 -= 3;
					vv.v2 -= 3;
					vv.v3 -= 3;
				}
			}
		}

		std::unordered_set<int> growToBoundary(std::stack<int> seeds) const
		{
			std::unordered_set<int> traversed;
			while (!seeds.empty())
			{
				const int iT = seeds.top();
				seeds.pop();
				traversed.insert(iT);
				const Triangle& t = triangles[iT];
				for (int i = 0; i < 3; ++i)
				{
					const DelaunayEdge opEdge(t.vertices[ccw(i)], t.vertices[cw(i)]);
					if (fixedEdges.count(opEdge))
						continue;
					const int iN = t.neighbors[opoNbr(i)];
					if (iN != -1 && traversed.count(iN) == 0)
						seeds.push(iN);
				}
			}
			return traversed;
		}

		void fixEdge(const DelaunayEdge& edge)
		{
			if (!fixedEdges.insert(edge).second)
			{
				++overlapCount[edge]; // if edge is already fixed increment the counter
			}
		}

		void fixEdge(const DelaunayEdge& edge, const DelaunayEdge& originalEdge)
		{
			fixEdge(edge);
			if (edge != originalEdge)
				insert_unique(pieceToOriginals[edge], originalEdge);
		}

		void splitFixedEdge(const DelaunayEdge& edge, const int iSplitVert)
		{
			// split constraint (fixed) edge that already exists in triangulation
			const DelaunayEdge half1(edge.v1, iSplitVert);
			const DelaunayEdge half2(iSplitVert, edge.v2);
			// remove the edge that and add its halves
			fixedEdges.erase(edge);
			fixEdge(half1);
			fixEdge(half2);
			// maintain overlaps
			const auto overlapIt = overlapCount.find(edge);
			if (overlapIt != overlapCount.end())
			{
				overlapCount[half1] += overlapIt->second;
				overlapCount[half2] += overlapIt->second;
				overlapCount.erase(overlapIt);
			}
			// maintain piece-to-original mapping
			std::vector<DelaunayEdge> newOriginals(1, edge);
			const auto originalsIt = pieceToOriginals.find(edge);
			if (originalsIt != pieceToOriginals.end())
			{ // edge being split was split before: pass-through originals
				newOriginals = originalsIt->second;
				pieceToOriginals.erase(originalsIt);
			}
			insert_unique(pieceToOriginals[half1], newOriginals);
			insert_unique(pieceToOriginals[half2], newOriginals);
		}

		int addSplitEdgeVertex(const Vector2& splitVert, const int iT, const int iTopo)
		{
			// add a new point on the edge that splits an edge in two
			const int iSplitVert = static_cast<int>(vertices.size());
			addNewVertex(splitVert, -1);
			std::stack<int> triStack = insertVertexOnEdge(iSplitVert, iT, iTopo);
			tryAddVertexToLocator(iSplitVert);
			ensureDelaunayByEdgeFlips(iSplitVert, triStack);
			return iSplitVert;
		}

		int splitFixedEdgeAt(const DelaunayEdge& edge, const Vector2& splitVert, const int iT, const int iTopo)
		{
			const int iSplitVert = addSplitEdgeVertex(splitVert, iT, iTopo);
			splitFixedEdge(edge, iSplitVert);
			return iSplitVert;
		}

		void makeDummy(const int iT)
		{
			m_dummyTris.push_back(iT);
		}

		void eraseDummies()
		{
			if (m_dummyTris.empty())
				return;
			const std::unordered_set<int> dummySet(m_dummyTris.begin(), m_dummyTris.end());
			std::unordered_map<int, int> intMap;
			intMap[-1] = -1;
			for (int iT = 0, iTnew = 0; iT < int(triangles.size()); ++iT)
			{
				if (dummySet.count(iT))
					continue;
				intMap[iT] = iTnew;
				triangles[iTnew] = triangles[iT];
				iTnew++;
			}
			triangles.erase(triangles.end() - dummySet.size(), triangles.end());

			// remap adjacent triangle indices for vertices
			for (std::vector<int>::iterator iT = m_vertTris.begin(); iT != m_vertTris.end();
				++iT)
			{
				*iT = intMap[*iT];
			}
			// remap neighbor indices for triangles
			for (std::vector<Triangle>::iterator t = triangles.begin(); t != triangles.end(); ++t)
			{
				DelaunayTriangle& nn = t->neighbors;
				for (int i = 0; i < 3; ++i)
				{
					int &iN = nn[i];
					iN = intMap[iN];
				}
			}
			// clear dummy triangles
			m_dummyTris = std::vector<int>();
		}

		std::unordered_map<int, LayerDepth> peelLayer(std::stack<int> seeds, const LayerDepth layerDepth, std::vector<LayerDepth>& triDepths) const
		{
			std::unordered_map<int, LayerDepth> behindBoundary;
			while (!seeds.empty())
			{
				const int iT = seeds.top();
				seeds.pop();
				triDepths[iT] = std::min(triDepths[iT], layerDepth);
				behindBoundary.erase(iT);
				const Triangle& t = triangles[iT];
				for (int i(0); i < int(3); ++i)
				{
					const DelaunayEdge opEdge(t.vertices[ccw(i)], t.vertices[cw(i)]);
					const int iN = t.neighbors[opoNbr(i)];
					if (iN == -1 || triDepths[iN] <= layerDepth)
						continue;
					if (fixedEdges.count(opEdge))
					{
						const auto cit = overlapCount.find(opEdge);
						const LayerDepth triDepth = cit == overlapCount.end()
							? layerDepth + 1
							: layerDepth + cit->second + 1;
						behindBoundary[iN] = triDepth;
						continue;
					}
					seeds.push(iN);
				}
			}
			return behindBoundary;
		}

		void insertVertices_AsProvided(int superGeomVertCount)
		{
			for (int iV = superGeomVertCount; iV < vertices.size(); ++iV)
			{
				insertVertex(iV);
			}
		}

		void insertVertices_Randomized(int superGeomVertCount)
		{
			std::size_t vertexCount = vertices.size() - superGeomVertCount;
			std::vector<int> ii(vertexCount);
			iota(ii.begin(), ii.end(), superGeomVertCount);
			Maths::RandomShuffe(ii.data(), (int)ii.size());
			for (std::vector<int>::iterator it = ii.begin(); it != ii.end(); ++it)
			{
				insertVertex(*it);
			}
		}

		static inline std::size_t maxQueueLengthBFSKDTree(const std::size_t vertexCount)
		{
			const int filledLayerPow2 =
				static_cast<int>(std::floor(log2_bc(vertexCount * 1.0f)) - 1);
			const std::size_t nodesInFilledTree =
				static_cast<std::size_t>(std::pow(2., filledLayerPow2 + 1) - 1);
			const std::size_t nodesInLastFilledLayer =
				static_cast<std::size_t>(std::pow(2., filledLayerPow2));
			const std::size_t nodesInLastLayer = vertexCount - nodesInFilledTree;
			return nodesInLastLayer >= nodesInLastFilledLayer
				? nodesInLastFilledLayer + nodesInLastLayer -
				nodesInLastFilledLayer
				: nodesInLastFilledLayer;
		}

		template <class ForwardIt, class T>
		static void iota(ForwardIt first, ForwardIt last, T value)
		{
			while (first != last)
			{
				*first++ = value;
				++value;
			}
		}

		void insertVertices_KDTreeBFS(int superGeomVertCount, Vector2 boxMin, Vector2 boxMax)
		{
			// calculate original indices
			const int vertexCount = static_cast<int>(vertices.size()) - superGeomVertCount;
			if (vertexCount <= 0)
				return;
			std::vector<int> ii(vertexCount);
			iota(ii.begin(), ii.end(), superGeomVertCount);

			typedef std::vector<int>::iterator It;
			FixedCapacityQueue<std::tuple<It, It, Vector2, Vector2, int>> queue(
				maxQueueLengthBFSKDTree(vertexCount));
			queue.push(std::make_tuple(ii.begin(), ii.end(), boxMin, boxMax, 0));

            std::vector<int>::iterator first, last;
			Vector2 newBoxMin, newBoxMax;
			int parent, mid;

			const less_than_x cmpX(vertices);
			const less_than_y cmpY(vertices);

			while (!queue.empty())
			{
				std::tie(first, last, boxMin, boxMax, parent) = queue.front();
				queue.pop();
				assert(first != last);

				const std::ptrdiff_t len = std::distance(first, last);
				if (len == 1)
				{
					insertVertex(*first, parent);
					continue;
				}
				const std::vector<int>::iterator midIt = first + len / 2;
				if (boxMax.x - boxMin.x >= boxMax.y - boxMin.y)
				{
					portable_nth_element(first, midIt, last, cmpX);
					mid = *midIt;
					const float split = vertices[mid].x;
					newBoxMin.x = split;
					newBoxMin.y = boxMin.y;
					newBoxMax.x = split;
					newBoxMax.y = boxMax.y;
				}
				else
				{
					portable_nth_element(first, midIt, last, cmpY);
					mid = *midIt;
					const float split = vertices[mid].y;
					newBoxMin.x = boxMin.x;
					newBoxMin.y = split;
					newBoxMax.x = boxMax.x;
					newBoxMax.y = split;
				}
				insertVertex(mid, parent);
				if (first != midIt)
				{
					queue.push(std::make_tuple(first, midIt, boxMin, newBoxMax, mid));
				}
				if (midIt + 1 != last)
				{
					queue.push(std::make_tuple(midIt + 1, last, newBoxMin, boxMax, mid));
				}
			}
		}

		std::pair<int, int> edgeTriangles(const int a, const int b) const
		{
			const int triStart = m_vertTris[a];
			assert(triStart != -1);
			int iT = triStart, iTNext = triStart;
			int iV = -1;
			do
			{
				const Triangle& t = triangles[iT];
				std::tie(iTNext, iV) = t.next(a);
				assert(iTNext != -1);
				if (iV == b)
				{
					return std::make_pair(iT, iTNext);
				}
				iT = iTNext;
			} while (iT != triStart);
			return std::make_pair(-1, -1);
		}

		bool hasEdge(const int a, const int b) const
		{
			return edgeTriangles(a, b).first != -1;
		}

		void setAdjacentTriangle(const int v, const int t)
		{
			assert(t != -1);
			m_vertTris[v] = t;
			assert(
				triangles[t].vertices[0] == v || triangles[t].vertices[1] == v ||
				triangles[t].vertices[2] == v);
		}

		void pivotVertexTriangleCW(const int v)
		{
			assert(m_vertTris[v] != -1);
			m_vertTris[v] = triangles[m_vertTris[v]].next(v).first;
			assert(m_vertTris[v] != -1);
			assert(
				triangles[m_vertTris[v]].vertices[0] == v ||
				triangles[m_vertTris[v]].vertices[1] == v ||
				triangles[m_vertTris[v]].vertices[2] == v);
		}

		void tryAddVertexToLocator(const int v)
		{
			if (!m_nearPtLocator.empty()) // only if locator is initialized already
				m_nearPtLocator.addPoint(v, vertices);
		}

		void tryInitNearestPointLocator()
		{
			if (!vertices.empty() && m_nearPtLocator.empty())
			{
				m_nearPtLocator.initialize(vertices);
			}
		}

		std::vector<int> m_dummyTris;
		LocatorKDTree<> m_nearPtLocator;
		int m_nTargetVerts;
		SuperGeometryType m_superGeomType;
		VertexInsertionOrder m_vertexInsertionOrder;
		IntersectingConstraintEdges m_intersectingEdgesStrategy;
		float m_minDistToConstraintEdge;
		std::vector<int> m_vertTris;
	};

	bool ConstrainedDelaunay::Triangulate(const std::vector<Vector2>& points, std::vector<DelaunayEdge>& edges)
	{
		ConstrainedTriangulation d;
		
		d.insertVertices(points);
		if (edges.size() > 0)
		{
			d.insertEdges(edges);
			d.eraseOuterTrianglesAndHoles();
		}
		else
		{
			d.eraseSuperTriangle();
		}

		bool succ = d.verifyTopology();
        (void)succ;
        
		Points = d.vertices;
		Triangles.clear();
		for (size_t i = 0; i < d.triangles.size(); ++i)
		{
			DelaunayTriangle v = d.triangles[i].vertices;
			Triangles.emplace_back(v.v1, v.v2, v.v3);
		}
		return Triangles.size() > 0;
	}

	int ConstrainedDelaunay::GetNumTriangles() const
	{
		return (int)Triangles.size();
	}

	void ConstrainedDelaunay::GetTriangle(int index, Vector2& A, Vector2& B, Vector2& C)
	{
		if (index < (int)Triangles.size())
		{
			A = Points[Triangles[index].v1];
			B = Points[Triangles[index].v2];
			C = Points[Triangles[index].v3];
		}
	}

}
