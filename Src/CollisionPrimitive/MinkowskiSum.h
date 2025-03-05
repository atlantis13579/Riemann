#pragma once

#include "../Maths/Vector3.h"

namespace Riemann
{
	class MinkowskiSum
	{
	public:
		virtual Vector3 Center() const = 0;
		virtual Vector3 Support(const Vector3& Dir) const = 0;
	};

	template <class Shape1, class Shape2>
	class MinkowskiSumTwoShape : public MinkowskiSum
	{
	public:
		MinkowskiSumTwoShape(const Shape1* _p1, const Shape2* _p2)
		{
			m_shape1 = _p1;
			m_shape2 = _p2;
		}

		virtual Vector3 Center() const override final
		{
			return m_shape1->GetCenter() - m_shape2->GetCenter();
		}

		virtual Vector3 Support(const Vector3& Direction) const override final
		{
			Vector3 support1 = m_shape1->GetSupport(Direction);
			Vector3 support2 = m_shape2->GetSupport(-Direction);
			Vector3 diff = support1 - support2;
			return diff;
		}

	private:
		const Shape1* m_shape1;
		const Shape2* m_shape2;
	};
}