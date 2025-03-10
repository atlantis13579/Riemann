#pragma once

#include "../Maths/Vector3.h"
#include "../Maths/Transform.h"

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

	template <class Shape1>
	class TransformedConvexShape : public GjkShape
	{
	public:
		TransformedConvexShape(const Shape1* _p1, const Transform& transform)
		{
			m_shape1 = _p1;
			m_transform = transform;
		}

		virtual Vector3 Center() const override final
		{
			const Vector3 LocalCenter = m_shape1->GetCenter();
			const Vector3 WorldCenter = m_transform.LocalToWorld(LocalCenter);
			return WorldCenter;
		}

		virtual Vector3 Support(const Vector3& Direction) const override final
		{
			const Vector3 DirLocal = m_transform.WorldToLocalDirection(Direction);
			const Vector3 SupportLocal = m_shape1->GetSupport(DirLocal);
			const Vector3 SupportWorld = m_transform.LocalToWorld(SupportLocal);
			return SupportWorld;
		}

	private:
		const Shape1*	m_shape1;
		Transform		m_transform;
	};

	template <class Shape1, class Shape2>
	class TransformedMinkowskiSum : public GjkShape
	{
	public:
		TransformedMinkowskiSum(const Shape1* p1, const Transform& transform1, const Shape2* p2, const Transform& transform2)
		{
			m_shape1 = p1;
			m_shape2 = p2;
			m_transform1 = transform1;
			m_transform2 = transform2;
		}

		inline Vector3 Support1(const Vector3& Direction) const
		{
			const Vector3 DirLocal = m_transform1.WorldToLocalDirection(Direction);
			const Vector3 SupportLocal = m_shape1->GetSupport(DirLocal);
			const Vector3 SupportWorld = m_transform1.LocalToWorld(SupportLocal);
			return SupportWorld;
		}

		inline Vector3 Support2(const Vector3& Direction) const
		{
			const Vector3 DirLocal = m_transform2.WorldToLocalDirection(Direction);
			const Vector3 SupportLocal = m_shape2->GetSupport(DirLocal);
			const Vector3 SupportWorld = m_transform2.LocalToWorld(SupportLocal);
			return SupportWorld;
		}

		inline Vector3 Center1() const
		{
			const Vector3 LocalCenter = m_shape1->GetCenter();
			const Vector3 WorldCenter = m_transform1.LocalToWorld(LocalCenter);
			return WorldCenter;
		}

		inline Vector3 Center2() const
		{
			const Vector3 LocalCenter = m_shape2->GetCenter();
			const Vector3 WorldCenter = m_transform2.LocalToWorld(LocalCenter);
			return WorldCenter;
		}

		virtual Vector3 Center() const override final
		{
			const Vector3 position1 = Center1();
			const Vector3 position2 = Center2();
			return position1 - position2;
		}

		virtual Vector3 Support(const Vector3& Direction) const override final
		{
			const Vector3 support1 = Support1(Direction);
			const Vector3 support2 = Support2(-Direction);
			return support1 - support2;
		}

	private:
		const Shape1*	m_shape1;
		const Shape2*	m_shape2;
		Transform		m_transform1;
		Transform		m_transform2;
	};
}