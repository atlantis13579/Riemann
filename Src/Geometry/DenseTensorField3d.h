#pragma once

#include "../Maths/Tensor.h"
#include "../Maths/Box3.h"
#include "../Maths/Vector2.h"
#include "../Maths/Vector3.h"
#include "../Maths/Maths.h"

namespace Geometry
{
	enum InterpMethod
	{
		NO_INTERP = 0,
		BILINEAR
	};

	template<typename ScalerType>
	class DenseTensorField2d
	{
	public:
		DenseTensorField2d(const Vector2& Bmin, const Vector2& Bmax, int nX, int nY) : m_Fields({ nX, nY })
		{
			m_ConstTensor = ScalerType::Zero();

			m_Bmin = Bmin;
			m_Bmax = Bmax;
			m_Size = Vector2i(nX, nY);
			m_CellSize = (m_Bmax - m_Bmin);
			m_CellSize.x /= m_Size.x;
			m_CellSize.y /= m_Size.y;
			m_InvCellSize = Vector2(1.0f / m_CellSize.x, 1.0f / m_CellSize.y);
			m_InterpMethod = InterpMethod::BILINEAR;
		}

		DenseTensorField2d() : m_Fields({ 1, 1 })
		{
			m_Bmin = Vector2::Zero();
			m_Bmax = Vector2::One();
			m_Size = Vector2i(1, 1);
			m_CellSize = m_InvCellSize = Vector2::One();
		}

		~DenseTensorField2d()
		{
		}

	public:
		ScalerType GetTensorByPosition(const Vector3& pos) const
		{
			const float fx = (pos.x - m_Bmin.x) * m_InvCellSize.x - 0.5f;
			const float fy = (pos.y - m_Bmin.y) * m_InvCellSize.y - 0.5f;
			const int nx = (int)(fx);
			const int ny = (int)(fy);
			if (nx < 0 || nx >= m_Size.x || ny < 0 || ny >= m_Size.y)
			{
				return m_ConstTensor;
			}

			if (m_InterpMethod == InterpMethod::BILINEAR)
			{
				if (nx == m_Size.x - 1 || ny == m_Size.y - 1)
				{
					return m_Fields(nx, ny);
				}
				const float dx = fx - nx;
				const float dy = fy - ny;
				ScalerType y1 = LinearInterp(m_Fields(nx, ny), m_Fields(nx + 1, ny), dx);
				ScalerType y2 = LinearInterp(m_Fields(nx, ny + 1), m_Fields(nx + 1, ny + 1), dx);
				return LinearInterp(y1, y2, dy);
			}

			return m_Fields(nx, ny);
		}

	private:
		ScalerType				m_ConstTensor;
		Vector2					m_Bmin, m_Bmax;
		Vector2i				m_Size;
		Vector2					m_CellSize, m_InvCellSize;
		Maths::Tensor<ScalerType, 2>	m_Fields;
		InterpMethod			m_InterpMethod;
	};


	template<typename ScalerType>
	class DenseTensorField3d
	{
	public:
		DenseTensorField3d(const Box3& BV, int nX, int nY, int nZ) : m_Fields({ nX, nY, nZ })
		{
			m_ConstTensor = ScalerType::Zero();

			m_BV = BV;
			m_Size = Vector3i(nX, nY, nZ);
			m_CellSize = (BV.Max - BV.Min);
			m_CellSize.x /= m_Size.x;
			m_CellSize.y /= m_Size.y;
			m_CellSize.z /= m_Size.z;
			m_InvCellSize = Vector3(1.0f / m_CellSize.x, 1.0f / m_CellSize.y, 1.0f / m_CellSize.z);
			m_InterpMethod = InterpMethod::BILINEAR;
		}

		DenseTensorField3d() : m_Fields({ 1, 1, 1 })
		{
			m_BV = Box3::Unit();
			m_Size = Vector3i(1, 1, 1);
			m_CellSize = m_InvCellSize = Vector3::One();
		}

		~DenseTensorField3d()
		{
		}

	public:
		ScalerType GetTensorByPosition(const Vector3& pos) const
		{
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
				if (nx == m_Size.x - 1 || ny == m_Size.y - 1 || nz == m_Size.z - 1)
				{
					return m_Fields(nx, ny, nz);
				}
				const float dx = fx - nx;
				const float dy = fy - ny;
				const float dz = fz - nz;
				ScalerType y11 = LinearInterp(m_Fields(nx, ny, nz), m_Fields(nx + 1, ny, nz), dx);
				ScalerType y12 = LinearInterp(m_Fields(nx, ny, nz + 1), m_Fields(nx + 1, ny, nz + 1), dx);
				ScalerType y21 = LinearInterp(m_Fields(nx, ny + 1, nz), m_Fields(nx + 1, ny + 1, nz), dx);
				ScalerType y22 = LinearInterp(m_Fields(nx, ny + 1, nz + 1), m_Fields(nx + 1, ny + 1, nz + 1), dx);
				ScalerType z1 = LinearInterp(y11, y21, dy);
				ScalerType z2 = LinearInterp(y12, y22, dy);
				return LinearInterp(z1, z2, dz);
			}

			return m_Fields(nx, ny, nz);
		}

	private:
		ScalerType				m_ConstTensor;
		Box3					m_BV;
		Vector3i				m_Size;
		Vector3					m_CellSize, m_InvCellSize;
		Maths::Tensor<ScalerType, 3>	m_Fields;
		InterpMethod			m_InterpMethod;
	};
}
