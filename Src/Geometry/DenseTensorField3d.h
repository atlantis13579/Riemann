#pragma once

#include "../Maths/Tensor.h"
#include "../Maths/Box3d.h"
#include "../Maths/Vector3d.h"
#include "../Maths/Maths.h"

enum InterpMethod
{
	NO_INTERP = 0,
	BILINEAR
};

template<typename TensorType>
class DenseTensorField3d
{
public:
	DenseTensorField3d(const Box3d & BV, int nX, int nY, int nZ) : m_Fields({nX, nY, nZ})
	{
		m_IsConstField = false;
		m_ConstTensor = TensorType::Zero();

		m_BV = BV;
		m_Size = TVector3<int>(nX, nY, nZ);
		m_CellSize = (BV.Max - BV.Min) / m_Size;
		m_InvCellSize = 1.0f / m_CellSize;
		m_InterpMethod = InterpMethod::BILINEAR;
	}

	DenseTensorField3d(const TensorType& v)
	{
		m_IsConstField = true;
		m_ConstTensor = v;
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

		const float fx = (pos.x - m_BV.Min.x) * m_InvCellSize.x - 0.5f;
		const float fy = (pos.y - m_BV.Min.y) * m_InvCellSize.y - 0.5f;
		const float fz = (pos.z - m_BV.Min.z) * m_InvCellSize.z - 0.5f;
		const int nx = (int)(fx);
		const int ny = (int)(fy);
		const int nz = (int)(fz);
		if (nx < 0 || nx >= m_Size.x || ny < 0 || ny >= m_Size.y || nz < 0 || nz >= m_Size.z)
		{
			return m_ConstTensor;
		}

		if (m_InterpMethod == InterpMethod::BILINEAR)
		{
			if (nx == m_Size.x - 1 || nx == m_Size.x - 1 )
			{
				return m_Fields(nx, ny, nz);
			}
			const float dx = fx - nx;
			const float dy = fy - ny;
			const float dz = fz - nz;
			TensorType y11 = Interp(m_Fields(nx, ny, nz), m_Fields(nx + 1, ny, nz), dx);
			TensorType y12 = Interp(m_Fields(nx, ny, nz + 1), m_Fields(nx + 1, ny, nz + 1), dx);
			TensorType y21 = Interp(m_Fields(nx, ny + 1, nz), m_Fields(nx + 1, ny + 1, nz), dx);
			TensorType y22 = Interp(m_Fields(nx, ny + 1, nz + 1), m_Fields(nx + 1, ny + 1, nz + 1), dx);
			TensorType z1 = Interp(y11, y21, dy);
			TensorType z2 = Interp(y12, y22, dy);
			return Interp(z1, z2, dz);
		}

		return m_Fields(nx, ny, nz);
	}

private:
	bool					m_IsConstField;
	TensorType				m_ConstTensor;
	Box3d					m_BV;
	TVector3<int>			m_Size;
	Vector3d				m_CellSize, m_InvCellSize;
	Tensor<TensorType, 3>	m_Fields;
	InterpMethod			m_InterpMethod;
};