
#include "GeometryObject.h"

#include "../CollisionPrimitive/OrientedBox3d.h"
#include "../CollisionPrimitive/Plane3d.h"
#include "../CollisionPrimitive/Sphere3d.h"
#include "../CollisionPrimitive/Triangle3d.h"
#include "../CollisionPrimitive/Cylinder3d.h"
#include "../CollisionPrimitive/Capsule3d.h"

Geometry::Geometry(const Vector3d& Position, GeometryShapeType _Type, void* _ShapeObj, void* _Entity /*= nullptr*/)
{
	m_Shape.Type = _Type;
	m_Shape.Object = _ShapeObj;
	m_Entity = _Entity;
	m_BoxWorld = GetBoundingBoxLocal();

	SetPosition(Position);
}

Geometry::~Geometry()
{
	DestoryFunc func = Geometry::destoryTable[m_Shape.Type];
#ifdef DEBUG
	assert(func);
#endif
	func(m_Shape.Object);
}

Box3d		Geometry::GetBoundingBoxLocal() const
{
	GetAABBFunc func = Geometry::getaabbTable[m_Shape.Type];
#ifdef DEBUG
	assert(func);
#endif
	return func(m_Shape.Object);
}

const Box3d& Geometry::GetBoundingBoxWorld() const
{
	return m_BoxWorld;
}

Vector3d			Geometry::GetPositionWorld() const
{
	return m_Transform.GetTranslation();
}

void				Geometry::SetPositionOffset(const Vector3d& Offset)
{
	Vector3d World = m_Transform.GetTranslation() + Offset;
	SetPosition(World);
}

void				Geometry::SetPosition(const Vector3d& Position)
{
	m_Transform.SetTranslation(Position);
	m_BoxWorld = GetBoundingBoxLocal().Transform(m_Transform.GetWorldMatrix());
	return;
}

Matrix3d			Geometry::GetRotationMatrix() const
{
	return m_Transform.GetRotationMatrix();
}

Quaternion			Geometry::GetRotation() const
{
	return m_Transform.GetRotation();
}

void				Geometry::SetRotation(const Quaternion& Rotation)
{
	m_Transform.SetRotation(Rotation);
	m_BoxWorld = GetBoundingBoxLocal().Transform(m_Transform.GetWorldMatrix());
}

const Matrix4d&		Geometry::GetWorldMatrix()
{
	return m_Transform.GetWorldMatrix();
}

const Matrix4d&		Geometry::GetInverseWorldMatrix()
{
	return m_Transform.GetInverseWorldMatrix();
}

void*				Geometry::GetEntity()
{
	return m_Entity;
}

void				Geometry::SetEntity(void* Entity)
{
	m_Entity = Entity;
}

Transform*			Geometry::GetTransform()
{
	return &m_Transform;
}

bool				Geometry::RayCast(const Vector3d& Origin, const Vector3d& Dir, float* t)
{
	RayCastFunc func = Geometry::raycastTable[m_Shape.Type];
#ifdef DEBUG
	assert(func);
#endif
	return func(m_Shape.Object, Origin, Dir, t);
}

Vector3d			Geometry::GetSupportWorld(const Vector3d& Dir)
{
	Vector3d DirLocal = m_Transform.WorldToLocal(Dir);
	Vector3d SupportLocal = GetSupportLocal(DirLocal);
	Vector3d SupportWorld = m_Transform.LocalToWorld(SupportLocal);
	return SupportWorld;
}

Vector3d			Geometry::GetSupportLocal(const Vector3d& Dir) const
{
	SupportFunc func = Geometry::supportTable[m_Shape.Type];
#ifdef DEBUG
	assert(func);
#endif
	return func(m_Shape.Object, Dir);
}

Matrix3d			Geometry::GetInertia(float Mass) const
{
	InertiaFunc func = Geometry::inertiaTable[m_Shape.Type];
#ifdef DEBUG
	assert(func);
#endif
	return func(m_Shape.Object, Mass);
}

Matrix3d			Geometry::GetInverseInertia(float Mass) const
{
	return GetInertia(Mass).Inverse();
}

GetAABBFunc			Geometry::getaabbTable[GeometryShapeType::COUNT] = { 0 };
RayCastFunc			Geometry::raycastTable[GeometryShapeType::COUNT] = { 0 };
SupportFunc			Geometry::supportTable[GeometryShapeType::COUNT] = { 0 };
InertiaFunc			Geometry::inertiaTable[GeometryShapeType::COUNT] = { 0 };
DestoryFunc			Geometry::destoryTable[GeometryShapeType::COUNT] = { 0 };

#define	IMPL_GEOMETRY_OBJ(_type, _name)											\
	class Geom_Register_##_name													\
	{																			\
		public : Geom_Register_##_name()										\
		{																		\
			Geometry::getaabbTable[_type] =										\
				Geometry::GetBoundingBox_##_name;								\
			Geometry::raycastTable[_type] =										\
				Geometry::RayCast_##_name;										\
			Geometry::supportTable[_type] =										\
				Geometry::GetSupport_##_name;									\
			Geometry::inertiaTable[_type] =										\
				Geometry::GetInertia_##_name;									\
			Geometry::destoryTable[_type] =										\
				Geometry::Destory_##_name;										\
		}																		\
	};																			\
	static Geom_Register_##_name s_register_##_name;							\
	Box3d Geometry::GetBoundingBox_##_name(void *Obj)							\
	{																			\
		_name* p = reinterpret_cast<_name*>(Obj);								\
		return p->GetBoundingBox();												\
	}																			\
	bool Geometry::RayCast_##_name(void *Obj, const Vector3d& Origin, const Vector3d& Dir, float* t)			\
	{																			\
		_name* p = reinterpret_cast<_name*>(Obj);								\
		return p->IntersectRay(Origin, Dir, t);									\
	}																			\
	Vector3d Geometry::GetSupport_##_name(void *Obj, const Vector3d& Dir)		\
	{																			\
		_name* p = reinterpret_cast<_name*>(Obj);								\
		return p->GetSupport(Dir);												\
	}																			\
	Matrix3d Geometry::GetInertia_##_name(void *Obj, float Mass)				\
	{																			\
		_name* p = reinterpret_cast<_name*>(Obj);								\
		return p->GetInertiaTensor(Mass);										\
	}																			\
	void Geometry::Destory_##_name(void *Obj)									\
	{																			\
		_name* p = reinterpret_cast<_name*>(Obj);								\
		delete p;																\
	}																			\

IMPL_GEOMETRY_OBJ(GeometryShapeType::OBB, OrientedBox3d);
IMPL_GEOMETRY_OBJ(GeometryShapeType::PLANE, Plane3d);
IMPL_GEOMETRY_OBJ(GeometryShapeType::SPHERE, Sphere3d);
IMPL_GEOMETRY_OBJ(GeometryShapeType::CAPSULE, Capsule3d);
IMPL_GEOMETRY_OBJ(GeometryShapeType::TRIANGLE, Triangle3d);

Geometry* GeometryFactory::CreateOBB(const Vector3d& Position, const Vector3d& Bmin, const Vector3d& Bmax)
{
	OrientedBox3d* shape = new OrientedBox3d(Bmin, Bmax);
	return new Geometry(Position, GeometryShapeType::OBB, shape, nullptr);
}

Geometry* GeometryFactory::CreatePlane(const Vector3d& Position, const Vector3d& Normal, float D)
{
	Plane3d* shape = new Plane3d(Normal, D);
	return new Geometry(Position, GeometryShapeType::PLANE, shape, nullptr);
}

Geometry* GeometryFactory::CreateSphere(const Vector3d& Position, const Vector3d& Center, float Radius)
{
	Sphere3d* shape = new Sphere3d(Center, Radius);
	return new Geometry(Position, GeometryShapeType::SPHERE, shape, nullptr);
}

Geometry* GeometryFactory::CreateCapsule(const Vector3d& Position, const Vector3d& X1, const Vector3d& X2, float Radius)
{
	Capsule3d* shape = new Capsule3d(X1, X2, Radius);
	return new Geometry(Position, GeometryShapeType::CAPSULE, shape, nullptr);
}

Geometry* GeometryFactory::CreateTriangle(const Vector3d& Position, const Vector3d& A, const Vector3d& B, const Vector3d& C)
{
	Triangle3d* shape = new Triangle3d(A, B, C);
	return new Geometry(Position, GeometryShapeType::TRIANGLE, shape, nullptr);
}