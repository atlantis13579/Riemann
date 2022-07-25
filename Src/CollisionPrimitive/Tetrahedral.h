#pragma once

#include "../Maths/Vector3.h"
#include "ShapeType.h"

class Tetrahedral
{
public:
	Vector3 A, B, C, D;

public:
	Tetrahedral()
	{
	}

	Tetrahedral(const Vector3& InA, const Vector3& InB, const Vector3& InC, const Vector3& InD)
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
};
