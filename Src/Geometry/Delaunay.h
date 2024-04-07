#pragma once

#include <vector>
#include "../Maths/Vector2.h"
#include "../Maths/Vector3.h"
#include "../Maths/Box2.h"

namespace Geometry
{
	class Delaunay
	{
	public:
		bool Triangulate(const std::vector<Vector2>& points);

		int GetNumTriangles() const;
		void GetTriangle(int index, Vector2& A, Vector2& B, Vector2& C);

		std::vector<Vector2>	Points;
		std::vector<Vector3i>	Triangles;
	};
}