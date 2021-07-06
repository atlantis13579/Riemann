
#include "GeometryObject.h"

#include "../Geometry/AxisAlignedBox.h"
#include "../Geometry/Plane.h"
#include "../Geometry/Sphere.h"
#include "../Geometry/Triangle.h"
#include "../Geometry/Cylinder.h"
#include "../Geometry/Capsule.h"
#include "../Collision/AABBTree.h"

// static
GeometryObject* GeometryObject::CreateObject(GeometryShapeType Type, void* Object)
{
	GeometryObject* Obj = new GeometryObject(Type, Object);
	return Obj;
}

GetBoundingBoxFunc	GeometryObject::getboundingboxTable[GeometryShapeType::COUNT] = { 0 };
RayCastFunc			GeometryObject::rasycastTable[GeometryShapeType::COUNT] = { 0 };

#define	IMPL_GEOMETRY_OBJ(_type, _name)										\
	class Init_Geom_##_name													\
	{																		\
		public : Init_Geom_##_name()										\
		{																	\
			GeometryObject::getboundingboxTable[_type] =					\
				GeometryObject::GetBoundingBox_##_name;						\
			GeometryObject::rasycastTable[_type] =							\
				GeometryObject::RayCast_##_name;							\
		}																	\
	};																		\
	static Init_Geom_##_name s_junk_##_name;								\
	BoundingBox3d GeometryObject::GetBoundingBox_##_name(void *Obj)			\
	{																		\
		_name* p = reinterpret_cast<_name*>(Obj);							\
		return p->GetBoundingBox();											\
	}																		\
	bool GeometryObject::RayCast_##_name(void *Obj, const Vector3d& Origin, const Vector3d& Dir, float* t)			\
	{																		\
		_name* p = reinterpret_cast<_name*>(Obj);							\
		return p->IntersectRay(Origin, Dir, t);								\
	}

IMPL_GEOMETRY_OBJ(GeometryShapeType::AABB, AxisAlignedBox3);
IMPL_GEOMETRY_OBJ(GeometryShapeType::PLANE, Plane);
IMPL_GEOMETRY_OBJ(GeometryShapeType::SPHERE, Sphere);