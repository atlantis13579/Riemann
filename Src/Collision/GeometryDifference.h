#pragma once

#include "GeometryObject.h"
#include "MinkowskiSum.h"

class GeometryDifference : public MinkowskiSum
{
public:
	GeometryDifference() {}
	GeometryDifference(const Geometry* _g1, const Geometry* _g2) : Geom1(_g1), Geom2(_g2)
	{
	}

	const Geometry* Geom1;
	const Geometry* Geom2;

	inline Vector3 Support1(const Vector3& Dir) const
	{
		return Geom1->GetSupport_WorldSpace(Dir);
	}

	inline Vector3 Support2(const Vector3& Dir) const
	{
		return Geom2->GetSupport_WorldSpace(Dir);
	}
	
	virtual Vector3 Center() const override
	{
		Vector3 position1 = Geom1->GetBoundingVolume_WorldSpace().GetCenter();
		Vector3 position2 = Geom2->GetBoundingVolume_WorldSpace().GetCenter();
		return position1 - position2;
	}

	virtual Vector3 Support(const Vector3& Dir) const override
	{
		Vector3 support1 = Support1(Dir);
		Vector3 support2 = Support2(-Dir);
		Vector3 diff = support1 - support2;
		return diff;
	}
};
