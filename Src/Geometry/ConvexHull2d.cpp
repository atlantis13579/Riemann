
#include "ConvexHull2d.h"

#include "../Maths/Maths.h"

namespace Riemann
{
	void __convex_points_sort(int* p, int low, int high, int ori, const std::vector<Vector2>& points)
	{
		const float kEpilson = 0.00001f;

		int i, j, pivot;
		if (low < high)
		{
			int pi = RandomInt(low, high);
			pivot = p[pi];
			p[pi] = p[low];
			p[low] = pivot;

			i = low;
			j = high;

			while (i < j)
			{
				while (i < j && (points[pivot].y < points[p[j]].y - kEpilson || (fabsf(points[pivot].y - points[p[j]].y) <= kEpilson && points[pivot].x <= points[p[j]].x)))
					--j;
				if (i < j)
					p[i++] = p[j];

				while (i < j && (points[pivot].y > points[p[i]].y + kEpilson || (fabsf(points[pivot].y - points[p[i]].y) <= kEpilson && points[pivot].x >= points[p[i]].x)))
					++i;
				if (i < j)
					p[j--] = p[i];
			}

			p[i] = pivot;

			__convex_points_sort(p, low, i - 1, ori, points);
			__convex_points_sort(p, i + 1, high, ori, points);
		}
	}

	void ConvexHull_GrahamScan(const std::vector<Vector2>& points, std::vector<Vector2>* hull)
	{
		hull->clear();
		int nPts = (int)points.size();

		std::vector<int> order;
		order.resize(nPts);

		int ii, jj;
		for (ii = 0; ii < nPts; ++ii)
			order[ii] = ii;

		__convex_points_sort(&order[0], 0, nPts - 1, ii, points);

		hull->push_back(points[order[0]]);
		hull->push_back(points[order[1]]);
		for (jj = 1, ii = 2; ii < nPts; ++ii)
		{
			Vector2 i1 = jj == 0 ? hull->back() : hull->at(jj - 1);
			Vector2 i2 = hull->at(jj);
			int i3 = order[ii];
			float ddd = (i1 - points[i3]).Cross(i2 - points[i3]);
			while (jj >= 1 && (i1 - points[i3]).Cross(i2 - points[i3]) <= 0)
			{
				--jj;
				i1 = jj == 0 ? hull->back() : hull->at(jj - 1);
				i2 = hull->at(jj);
				ddd = (i1 - points[i3]).Cross(i2 - points[i3]);
			}
			// assert(jj + 1 < nPts);
			++jj;
			if (jj < (int)hull->size())
				(*hull)[jj] = points[i3];
			else
				hull->push_back(points[i3]);
		}

		// assert(jj < nPts && (hull->at(jj) - points[order[nPts - 1]]).SquareLength() < 0.001f );

		int top = jj + 1;
		for (ii = nPts - 2; ii >= 0; --ii)
		{
			Vector2 i1 = jj == 0 ? hull->back() : hull->at(jj - 1);
			Vector2 i2 = hull->at(jj);
			int i3 = order[ii];
			while (jj >= top && (i1 - points[i3]).Cross(i2 - points[i3]) <= 0)
			{
				--jj;
				i1 = hull->at(jj - 1);
				i2 = hull->at(jj);
			}
			if (jj + 1 == nPts)
				return;
			++jj;
			if (jj < (int)hull->size())
				(*hull)[jj] = points[i3];
			else
				hull->push_back(points[i3]);
		}

		hull->resize(jj + 1);
	}


	static bool IsFarAway(const Vector2& v0, const Vector2& v1, const Vector2& v2)
	{
		return (v0 - v1).SquareLength() < (v0 - v2).SquareLength();
	}

	void ConvexHull_GiftWrapping(const std::vector<Vector2>& points, std::vector<Vector2>* hull)
	{
		const float kEpilson = 0.00001f;

		int nPts = (int)points.size();
		int istart = 0;
		for (int j = 1; j < nPts; ++j)		// O(N)
		{
			if (points[j].y < points[istart].y - kEpilson || (fabsf(points[j].y - points[istart].y) <= kEpilson && points[j].x < points[istart].x))
				istart = j;
		}

		hull->push_back(points[istart]);

		int i2 = istart;
		for (int j = 1; j < nPts; ++j)		// O(M)
		{
			int i1 = i2;
			i2 = i1;

			for (int i = 0; i < nPts; ++i)	// O(N)
			{
				float cross = (points[i2] - points[i1]).Cross(points[i] - points[i1]);
				if (cross < -kEpilson || (fabsf(cross) <= kEpilson && IsFarAway(points[i1], points[i2], points[i])))
				{
					i2 = i;
				}
			}

			if (i2 == istart)
				break;
			hull->push_back(points[i2]);
		}
	}

	#define	n(_s)	(((_s)+1)%nPts)

	void RotatingCaliper(const std::vector<Vector2>& convex, int* i1, int* i2)
	{
		if (convex.size() < 1)
		{
			return;
		}

		float max_dist = 0.0f;
		int nPts = (int)convex.size();
		for (int i = 0, j = 1; i < nPts - 1; ++i)
		{
			while ((convex[n(i)] - convex[i]).Cross(convex[j] - convex[i]) <= (convex[n(i)] - convex[i]).Cross(convex[n(j)] - convex[i]))
				j = n(j);

			float dist = (convex[i] - convex[j]).SquareLength();
			if (dist > max_dist)
			{
				max_dist = dist;
				*i1 = i;
				*i2 = j;
			}
		}

		////////// Verify ///////////////
		// max_dist = (v[*i1] - v[*i2]).SquareLength();
		// for (int i = 0; i < nPts; ++i)
		// for (int j = 0; j < nPts; ++j)
		// {
		// 	float dist = (v[i] - v[j]).SquareLength();
		//	assert(dist <= max_dist);
		// }
		//////////////////////////////
	}

	#undef n
}