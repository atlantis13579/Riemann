#pragma once

#include "../Maths/Tensor.h"
#include "../Maths/Vector3d.h"

class VectorField
{
public:
	VectorField(int nX, int nY)
	{

	}

private:
	Tensor<Vector3d, 2>	m_Fields;
};