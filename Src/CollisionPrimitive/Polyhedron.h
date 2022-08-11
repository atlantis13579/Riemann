#pragma once

#include <float.h>
#include <vector>
#include "../Maths/Box3d.h"
#include "../Maths/Vector3.h"

// Polyhedron is a compact version of ConvexMesh
// For a regular Polyhedron,
// the number of verties each face N = 2E / F

template<int V, int E, int F>		// V - E + F == 2
class Polyhedron
{
	static_assert(V - E + F == 2, "Euler number is not 2");
	static_assert(2 * E % F == 0, "Not a regular Polyhedron");
public:
	Vector3	v[V];
	uint8_t	Indices[2*E+F];
	
	constexpr int	GetNumVertices() const
	{
		return V;
	}

	constexpr int	GetNumEdges() const
	{
		return E;
	}

	constexpr int	GetNumFaces() const
	{
		return F;
	}
	
	constexpr int	GetPolygonSize() const
	{
		return (2 * E) / F;
	}
	
	Vector3	operator[](int i)
	{
		return v[i];
	}

	const Vector3& operator[](int i) const
	{
		return v[i];
	}
	
	Vector3* GetData()
	{
		return v;
	}

	const Vector3* GetData() const
	{
		return v;
	}
	
	void		BuildRegularPolyhedron(float Radius)
	{
		
	}
	
	Vector3		ComputeCenterOfMass() const
	{
		Vector3 cm;
		for (int i = 0; i < V; ++i)
		{
			cm += v[i];
		}
		return cm / V;
	}
	
	Box3d		ComputeBoundingVolume() const
	{
		Box3d BV;
		BV.SetEmpty();
		for (uint16_t i = 0; i < V; ++i)
		{
			BV.Encapsulate(v[i]);
		}
		return BV;
	}
	
	float 		GetVolume() const;
	
	Vector3		GetSupport(const Vector3& Direction) const
	{
		float max_dot = -FLT_MAX;
		Vector3 max_v = Vector3::Zero();

		for (int i = 0; i < V; ++i)
		{
			const float dot = v[i].Dot(Direction);
			if (dot > max_dot)
			{
				max_dot = dot;
				max_v = v[i];
			}
		}
		return max_v;
	}
	
	void		GetMesh(std::vector<Vector3>& Vertices, std::vector<uint16_t>& Indices, std::vector<Vector3>& Normals);
	void		GetWireframe(std::vector<Vector3>& Vertices, std::vector<uint16_t>& Indices);
};

using Tetrahedron = Polyhedron<4, 6, 4>;
using Hexahedron = Polyhedron<8, 12, 6>;
using Octahedron = Polyhedron<6, 12, 8>;
using Dodecahedron = Polyhedron<20, 30, 12>;
using Icosahedron = Polyhedron<12, 30, 20>;