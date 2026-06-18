#pragma once

#include <vector>
#include "../Maths/Vector2.h"

namespace Riemann
{
	struct DelaunayEdge
	{
	public:
		DelaunayEdge(int _v1, int _v2)
		{
			if (_v1 < _v2)
			{
				v1 = _v1;
				v2 = _v2;
			}
			else
			{
				v1 = _v2;
				v2 = _v1;
			}
		}

		bool	operator==(const DelaunayEdge& rhs) const
		{
			return v1 == rhs.v1 && v2 == rhs.v2;
		}

		bool	operator!=(const DelaunayEdge& rhs) const
		{
			return v1 != rhs.v1 || v2 != rhs.v2;
		}

		int v1;
		int v2;
	};

	struct DelaunayTriangle
	{
	public:
		DelaunayTriangle() {}
		DelaunayTriangle(int _v1, int _v2, int _v3)
		{
			v1 = _v1;
			v2 = _v2;
			v3 = _v3;
		}

		inline int	operator[] (int i) const
		{
			const int* p = (const int*)this;
			return p[i];
		}

		inline int& operator[] (int i)
		{
			int* p = (int*)this;
			return p[i];
		}

		int v1;
		int v2;
		int v3;
	};


	class Delaunay
	{
	public:
		bool Triangulate(const std::vector<Vector2>& points);

		int GetNumTriangles() const;
		void GetTriangle(int index, Vector2& A, Vector2& B, Vector2& C);

		std::vector<Vector2>			Points;
		std::vector<DelaunayTriangle>	Triangles;
	};

	class ConstrainedDelaunay
	{
	public:
		bool Triangulate(const std::vector<Vector2>& points, std::vector<DelaunayEdge>& edges);

		int GetNumTriangles() const;
		void GetTriangle(int index, Vector2& A, Vector2& B, Vector2& C);

		std::vector<Vector2>			Points;
		std::vector<DelaunayTriangle>	Triangles;
	};
}