#pragma once

#include "../Maths/Vector3.h"

namespace Riemann
{
	class GjkShape
	{
	public:
		virtual Vector3 Center() const = 0;
		virtual Vector3 Support(const Vector3& Dir) const = 0;
	};

	template <class Shape1>
	class ConvexShape : public GjkShape
	{
	public:
		ConvexShape(const Shape1* _p1)
		{
			m_shape1 = _p1;
		}

		virtual Vector3 Center() const override final
		{
			return m_shape1->GetCenter();
		}

		virtual Vector3 Support(const Vector3& Direction) const override final
		{
			return m_shape1->GetSupport(Direction);
		}

	private:
		const Shape1* m_shape1;
	};

	template <class Shape1, class Shape2>
	class MinkowskiSum : public GjkShape
	{
	public:
		MinkowskiSum(const Shape1* _p1, const Shape2* _p2)
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
			const Vector3 support1 = m_shape1->GetSupport(Direction);
			const Vector3 support2 = m_shape2->GetSupport(-Direction);
			const Vector3 diff = support1 - support2;
			return diff;
		}

	private:
		const Shape1* m_shape1;
		const Shape2* m_shape2;
	};
}