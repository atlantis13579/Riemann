
#include "GeometryObject.h"

#include "../CollisionPrimitive/AxisAlignedBox.h"
#include "../CollisionPrimitive/Plane.h"
#include "../CollisionPrimitive/Sphere.h"
#include "../CollisionPrimitive/Triangle.h"
#include "../CollisionPrimitive/Cylinder.h"
#include "../CollisionPrimitive/Capsule.h"

GeometryObject::GeometryObject(const Vector3d& Position, GeometryShapeType _Type, void* _ShapeObj, void* _Entity /*= nullptr*/)
{
	m_Shape.Type = _Type;
	m_Shape.Object = _ShapeObj;
	m_Entity = _Entity;
	m_BoxWorld = GetBoundingBoxLocal();

	SetPosition(Position);
}

GeometryObject::~GeometryObject()
{
	DestoryFunc func = GeometryObject::destoryTable[m_Shape.Type];
#ifdef DEBUG
	assert(func);
#endif
	func(m_Shape.Object);
}

BoundingBox3d		GeometryObject::GetBoundingBoxLocal() const
{
	GetAABBFunc func = GeometryObject::getaabbTable[m_Shape.Type];
#ifdef DEBUG
	assert(func);
#endif
	return func(m_Shape.Object);
}

const	BoundingBox3d& GeometryObject::GetBoundingBoxWorld() const
{
	return m_BoxWorld;
}

void	GeometryObject::SetPosition(const Vector3d& Position)
{
	m_Transform.SetTranslation(Position);
	m_BoxWorld = GetBoundingBoxLocal() + Position;
}

void	GeometryObject::SetRotation(const Quaternion& Rotation)
{
	m_Transform.SetRotation(Rotation);
}

bool	GeometryObject::RayCast(const Vector3d& Origin, const Vector3d& Dir, float* t)
{
	RayCastFunc func = GeometryObject::raycastTable[m_Shape.Type];
#ifdef DEBUG
	assert(func);
#endif
	return func(m_Shape.Object, Origin, Dir, t);
}

Vector3d	GeometryObject::GetSupport(const Vector3d& Dir)
{
	SupportFunc func = GeometryObject::supportTable[m_Shape.Type];
#ifdef DEBUG
	assert(func);
#endif
	return func(m_Shape.Object, Dir);
}

Vector3d	GeometryObject::GetSupport(const GeometryObject* Geom1, const GeometryObject* Geom2, const Vector3d& Dir)
{
	SupportFunc func1 = GeometryObject::supportTable[Geom1->m_Shape.Type];
	SupportFunc func2 = GeometryObject::supportTable[Geom2->m_Shape.Type];
#ifdef DEBUG
	assert(func1 && func2);
#endif
	Vector3d p1 = func1(Geom1->m_Shape.Object, Dir);
	Vector3d p2 = func2(Geom2->m_Shape.Object, -Dir);
	return p1 - p2;
}

Matrix3d	GeometryObject::GetInertia(float Mass)
{
	InertiaFunc func = GeometryObject::inertiaTable[m_Shape.Type];
#ifdef DEBUG
	assert(func);
#endif
	return func(m_Shape.Object, Mass);
}

GetAABBFunc			GeometryObject::getaabbTable[GeometryShapeType::COUNT] = { 0 };
RayCastFunc			GeometryObject::raycastTable[GeometryShapeType::COUNT] = { 0 };
SupportFunc			GeometryObject::supportTable[GeometryShapeType::COUNT] = { 0 };
InertiaFunc			GeometryObject::inertiaTable[GeometryShapeType::COUNT] = { 0 };
DestoryFunc			GeometryObject::destoryTable[GeometryShapeType::COUNT] = { 0 };

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
			GeometryObject::destoryTable[_type] =								\
				GeometryObject::Destory_##_name;								\
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
	void GeometryObject::Destory_##_name(void *Obj)								\
	{																			\
		_name* p = reinterpret_cast<_name*>(Obj);								\
		delete p;																\
	}																			\

IMPL_GEOMETRY_OBJ(GeometryShapeType::AABB, AxisAlignedBox3);
IMPL_GEOMETRY_OBJ(GeometryShapeType::PLANE, Plane);
// IMPL_GEOMETRY_OBJ(GeometryShapeType::SPHERE, Sphere);

GeometryObject* GeometryObjectFactory::CreateAABB(const Vector3d& Position, const Vector3d& Bmin, const Vector3d& Bmax)
{
	AxisAlignedBox3* shape = new AxisAlignedBox3(Bmin, Bmax);
	return new GeometryObject(Position, GeometryShapeType::AABB, shape, nullptr);
}

GeometryObject* GeometryObjectFactory::CreatePlane(const Vector3d& Position, const Vector3d& Normal, float D)
{
	Plane* shape = new Plane(Normal, D);
	return new GeometryObject(Position, GeometryShapeType::PLANE, shape, nullptr);
}

