
#include "GeometryObject.h"

#include "../Geometry/AxisAlignedBox.h"
#include "../Geometry/Plane.h"
#include "../Geometry/Sphere.h"
#include "../Geometry/Triangle.h"
#include "../Geometry/Cylinder.h"
#include "../Geometry/Capsule.h"
#include "../Collision/AABBTree.h"

// static
GeometryObject* GeometryObject::CreateObject(const GeometryShapeType Type, void* ShapeObj, void* Entity)
{
	GeometryObject* Obj = new GeometryObject(Type, ShapeObj, Entity);
	return Obj;
}

BoundingBox3d GeometryObject::GetBoundingBox(const GeometryObject* Geom)
{
	GetAABBFunc func = GeometryObject::getaabbTable[Geom->Shape.Type];
#ifdef DEBUG
	assert(func);
#endif
	return func(Geom->Shape.Object);
}

bool GeometryObject::RayCast(const GeometryObject* Geom, const Vector3d& Origin, const Vector3d& Dir, float* t)
{
	RayCastFunc func = GeometryObject::raycastTable[Geom->Shape.Type];
#ifdef DEBUG
	assert(func);
#endif
	return func(Geom->Shape.Object, Origin, Dir, t);
}

Vector3d GeometryObject::GetSupport(const GeometryObject* Geom, const Vector3d& Dir)
{
	SupportFunc func = GeometryObject::supportTable[Geom->Shape.Type];
#ifdef DEBUG
	assert(func);
#endif
	return func(Geom->Shape.Object, Dir);
}

Vector3d GeometryObject::GetSupport(const GeometryObject* Geom1, const GeometryObject* Geom2, const Vector3d& Dir)
{
	SupportFunc func1 = GeometryObject::supportTable[Geom1->Shape.Type];
	SupportFunc func2 = GeometryObject::supportTable[Geom2->Shape.Type];
#ifdef DEBUG
	assert(func1 && func2);
#endif
	Vector3d p1 = func1(Geom1->Shape.Object, Dir);
	Vector3d p2 = func2(Geom2->Shape.Object, -Dir);
	return p1 - p2;
}

Matrix3d GeometryObject::GetInertia(const GeometryObject* Geom, float Mass)
{
	InertiaFunc func = GeometryObject::inertiaTable[Geom->Shape.Type];
#ifdef DEBUG
	assert(func);
#endif
	return func(Geom->Shape.Object, Mass);
}

GetAABBFunc			GeometryObject::getaabbTable[GeometryShapeType::COUNT] = { 0 };
RayCastFunc			GeometryObject::raycastTable[GeometryShapeType::COUNT] = { 0 };
SupportFunc			GeometryObject::supportTable[GeometryShapeType::COUNT] = { 0 };
InertiaFunc			GeometryObject::inertiaTable[GeometryShapeType::COUNT] = { 0 };

#define	IMPL_GEOMETRY_OBJ(_type, _name)											\
	class Geom_Register_##_name													\
	{																			\
		public : Geom_Register_##_name()										\
		{																		\
			GeometryObject::getaabbTable[_type] =								\
				GeometryObject::GetBoundingBox_##_name;							\
			GeometryObject::raycastTable[_type] =								\
				GeometryObject::RayCast_##_name;								\
			GeometryObject::supportTable[_type] =								\
				GeometryObject::GetSupport_##_name;								\
			GeometryObject::inertiaTable[_type] =								\
				GeometryObject::GetInertia_##_name;								\
		}																		\
	};																			\
	static Geom_Register_##_name s_register_##_name;							\
	BoundingBox3d GeometryObject::GetBoundingBox_##_name(void *Obj)				\
	{																			\
		_name* p = reinterpret_cast<_name*>(Obj);								\
		return p->GetBoundingBox();												\
	}																			\
	bool GeometryObject::RayCast_##_name(void *Obj, const Vector3d& Origin, const Vector3d& Dir, float* t)			\
	{																			\
		_name* p = reinterpret_cast<_name*>(Obj);								\
		return p->IntersectRay(Origin, Dir, t);									\
	}																			\
	Vector3d GeometryObject::GetSupport_##_name(void *Obj, const Vector3d& Dir)	\
	{																			\
		_name* p = reinterpret_cast<_name*>(Obj);								\
		return p->GetSupport(Dir);												\
	}																			\
	Matrix3d GeometryObject::GetInertia_##_name(void *Obj, float Mass)			\
	{																			\
		_name* p = reinterpret_cast<_name*>(Obj);								\
		return p->GetInertiaTensor(Mass);										\
	}																			\

IMPL_GEOMETRY_OBJ(GeometryShapeType::AABB, AxisAlignedBox3);
IMPL_GEOMETRY_OBJ(GeometryShapeType::PLANE, Plane);
// IMPL_GEOMETRY_OBJ(GeometryShapeType::SPHERE, Sphere);