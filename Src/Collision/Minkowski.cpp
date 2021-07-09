
#include "Minkowski.h"

#include "GeometryObject.h"

Vector3d Minkowski::Support1(const Vector3d& Dir)
{
	return Geom1->GetSupportWorld(Dir);
}

Vector3d Minkowski::Support2(const Vector3d& Dir)
{
	return Geom2->GetSupportWorld(Dir);
}

Vector3d Minkowski::Support(const Vector3d& Dir)
{
	Vector3d support1 = Support1(Dir);
	Vector3d support2 = Support2(Dir);
	return support1 - support2;
}
