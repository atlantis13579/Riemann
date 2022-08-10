#pragma once

#include <float.h>
#include <vector>
#include "../Maths/Vector3.h"
#include "ShapeType.h"

class Tetrahedron
{
public:
	Vector3 A, B, C, D;

public:
	Tetrahedron()
	{
	}

	Tetrahedron(const Vector3& InA, const Vector3& InB, const Vector3& InC, const Vector3& InD)
	{
		Init(InA, InB, InC, InD);
	}

	static constexpr ShapeType3d	StaticType()
	{
		return ShapeType3d::TETRAHEDRAL;
	}

	void			Init(const Vector3& InA, const Vector3& InB, const Vector3& InC, const Vector3& InD)
	{
		A = InA;
		B = InB;
		C = InC;
		D = InD;
	}

	Vector3 operator[](int i)
	{
		return (&A)[i];
	}

	const Vector3& operator[](int i) const
	{
		return (&A)[i];
	}
	
	Vector3* GetData()
	{
		return &A;
	}

	const Vector3* GetData() const
	{
		return &A;
	}

	float GetVolume() const
	{
		return Determinant(A - D, B - D, C - D);
	}

	Vector3		GetSupport(const Vector3& Direction) const
	{
		float max_dot = -FLT_MAX;
		Vector3 max_v = Vector3::Zero();
		const Vector3* p = GetData();

		for (uint16_t i = 0; i < 3; ++i)
		{
			const float dot = p[i].Dot(Direction);
			if (dot > max_dot)
			{
				max_dot = dot;
				max_v = p[i];
			}
		}
		return max_v;
	}

	Vector3		ComputeCenterOfMass() const
	{
		return (A + B + C + D) / 4;
	}

	void		GetMesh(std::vector<Vector3>& Vertices, std::vector<uint16_t>& Indices, std::vector<Vector3>& Normals)
	{
	}

	void		GetWireframe(std::vector<Vector3>& Vertices, std::vector<uint16_t>& Indices)
	{
		Vertices = std::vector<Vector3>({ A , B, C, D });
		Indices = std::vector<uint16_t>({ 0,1, 1,2, 2,0, 0,3, 1,3, 2,3 });
	}
};
