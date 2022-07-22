#pragma once

#include "../Maths/Vector3.h"
#include "ShapeType.h"

class Tetrahedral
{
public:
	Vector3d A, B, C, D;

public:
	Tetrahedral()
	{
	}

	Tetrahedral(const Vector3d& InA, const Vector3d& InB, const Vector3d& InC, const Vector3d& InD)
	{
		Init(InA, InB, InC, InD);
	}

	static constexpr ShapeType3d	StaticType()
	{
		return ShapeType3d::TETRAHEDRAL;
	}

	void			Init(const Vector3d& InA, const Vector3d& InB, const Vector3d& InC, const Vector3d& InD)
	{
		A = InA;
		B = InB;
		C = InC;
		D = InD;
	}

	Vector3d operator[](int i)
	{
		return (&A)[i];
	}

	const Vector3d& operator[](int i) const
	{
		return (&A)[i];
	}
};
