#pragma once

#include "../Maths/Tensor.h"
#include "../Maths/Box3d.h"
#include "../Maths/Vector3d.h"

template<typename TensorType>
class DenseTensorField3d
{
public:
	DenseTensorField3d(int nX, int nY, int nZ) : m_Fields({nX, nY, nZ})
	{
		m_IsConstField = false;
		m_ConstTensor = TensorType::Zero();

		m_Size = TVector3<int>(nX, nY, nZ);
	}

	DenseTensorField3d(const TensorType& vec)
	{
		m_IsConstField = true;
		m_ConstTensor = vec;
	}

	~DenseTensorField3d()
	{

	}

public:
	TensorType GetTensorByPosition(const Vector3d& pos) const
	{
		if (m_IsConstField)
		{
			return m_ConstTensor;
		}

		const int nx = (int)((pos.x - m_BV.Min.x) * m_InvCellSize.x);
		const int ny = (int)((pos.y - m_BV.Min.y) * m_InvCellSize.y);
		const int nz = (int)((pos.z - m_BV.Min.z) * m_InvCellSize.z);
		if (nx < 0 || nx >= m_Size.x || ny < 0 || ny >= m_Size.y || nz < 0 || nz >= m_Size.z)
		{
			return m_ConstTensor;
		}

		return m_Fields(nx, ny, nz);
	}

private:
	bool					m_IsConstField;
	TensorType				m_ConstTensor;
	Box3d					m_BV;
	TVector3<int>			m_Size;
	Vector3d				m_InvCellSize;
	Tensor<TensorType, 3>	m_Fields;
};