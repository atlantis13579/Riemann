#pragma once

#include "GeometryObject.h"
#include "../CollisionPrimitive/MinkowskiSum.h"

namespace Riemann
{
	class TransformedGeometry : public GjkShape
	{
	public:
		explicit TransformedGeometry(const Geometry* _g1) : Geom1(_g1)
		{
		}

		const Geometry* Geom1;

		virtual Vector3 Center() const override
		{
			return Geom1->GetCenter_WorldSpace();
		}

		virtual Vector3 Support(const Vector3& Direction) const override
		{
			return Geom1->GetSupport_WorldSpace(Direction);
		}

		Vector3 GetCenter() const
		{
			return Geom1->GetCenter_WorldSpace();
		}

		Vector3 GetSupport(const Vector3& Direction) const
		{
			return Geom1->GetSupport_WorldSpace(Direction);
		}
	};

	class GeometryDifference : public GjkShape
	{
	public:
		GeometryDifference(const Geometry* _g1, const Geometry* _g2) : Geom1(_g1), Geom2(_g2)
		{
		}

		const Geometry* Geom1;
		const Geometry* Geom2;

		inline Vector3 Support1(const Vector3& Direction) const
		{
			return Geom1->GetSupport_WorldSpace(Direction);
		}

		inline Vector3 Support2(const Vector3& Direction) const
		{
			return Geom2->GetSupport_WorldSpace(Direction);
		}

		virtual Vector3 Center() const override
		{
			const Vector3 position1 = Geom1->GetCenter_WorldSpace();
			const Vector3 position2 = Geom2->GetCenter_WorldSpace();
			return position1 - position2;
		}

		virtual Vector3 Support(const Vector3& Direction) const override
		{
			const Vector3 support1 = Support1(Direction);
			const Vector3 support2 = Support2(-Direction);
			Vector3 diff = support1 - support2;
			return diff;
		}
	};
}