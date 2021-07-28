
#include "GeometryObject.h"

#include "../CollisionPrimitive/OrientedBox3d.h"
#include "../CollisionPrimitive/Plane3d.h"
#include "../CollisionPrimitive/Sphere3d.h"
#include "../CollisionPrimitive/Triangle3d.h"
#include "../CollisionPrimitive/Cylinder3d.h"
#include "../CollisionPrimitive/Capsule3d.h"
#include "TriangleMesh.h"

Geometry::Geometry(const Vector3d& Position, GeometryShapeType _Type, void* _ShapeObj, void* _Entity /*= nullptr*/)
{
	m_Shape.Type = _Type;
	m_Shape.Object = _ShapeObj;
	m_Entity = _Entity;
	m_BoxWorld = GetBoundingVolumeLocalSpace();

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

Box3d		Geometry::GetBoundingVolumeLocalSpace() const
{
	GetAABBFunc func = Geometry::getaabbTable[m_Shape.Type];
#ifdef DEBUG
	assert(func);
#endif
	return func(m_Shape.Object);
}

const Box3d& Geometry::GetBoundingVolumeWorldSpace() const
{
	return m_BoxWorld;
}

Vector3d			Geometry::GetPosition() const
{
	return m_Transform.GetTranslation();
}

void				Geometry::SetPosition(const Vector3d& Position)
{
	m_Transform.SetTranslation(Position);
	m_BoxWorld = GetBoundingVolumeLocalSpace().Transform(m_Transform.GetWorldMatrix());
	return;
}

Matrix3d			Geometry::GetRotationMatrix() const
{
	return m_Transform.GetRotationMatrix();
}

Quaternion			Geometry::GetRotationQuat() const
{
	return m_Transform.GetRotation();
}

void				Geometry::SetRotationQuat(const Quaternion& Rotation)
{
	m_Transform.SetRotation(Rotation);
	m_BoxWorld = GetBoundingVolumeLocalSpace().Transform(m_Transform.GetWorldMatrix());
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

GeometryShapeType	Geometry::GetShapeType()
{
	return m_Shape.Type;
}

void*				Geometry::GetShapeGeometry()
{
	return m_Shape.Object;
}

bool				Geometry::RayCast(const Vector3d& Origin, const Vector3d& Dir, float* t)
{
	RayCastFunc func = Geometry::raycastTable[m_Shape.Type];
#ifdef DEBUG
	assert(func);
#endif
	return func(m_Shape.Object, Origin, Dir, t);
}

bool				Geometry::Overlap(const Geometry* Geom)
{
	GeometryShapeType Type1 = m_Shape.Type;
	GeometryShapeType Type2 = Geom->m_Shape.Type;
	OverlapFunc func = Geometry::overlapTable[Type1][Type1];
#ifdef DEBUG
	assert(func);
#endif
	return func(m_Shape.Object, Geom->m_Shape.Object);
}

bool				Geometry::Sweep(const Geometry* Geom, const Vector3d& Dir, float* t)
{
	GeometryShapeType Type1 = m_Shape.Type;
	GeometryShapeType Type2 = Geom->m_Shape.Type;
	SweepFunc func = Geometry::sweepTable[Type1][Type1];
#ifdef DEBUG
	assert(func);
#endif
	return func(m_Shape.Object, Geom->m_Shape.Object, Dir, t);
}

Vector3d			Geometry::GetSupportWorldSpace(const Vector3d& Dir)
{
	Vector3d DirLocal = m_Transform.WorldToLocal(Dir);
	Vector3d SupportLocal = GetSupportLocalSpace(DirLocal);
	Vector3d SupportWorld = m_Transform.LocalToWorld(SupportLocal);
	return SupportWorld;
}

Vector3d			Geometry::GetSupportLocalSpace(const Vector3d& Dir) const
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
SweepFunc			Geometry::sweepTable[GeometryShapeType::COUNT][GeometryShapeType::COUNT] = { 0 };
OverlapFunc			Geometry::overlapTable[GeometryShapeType::COUNT][GeometryShapeType::COUNT] = {0};

#define	REG_GEOMETRY_OBJ(_type, _name)									\
	Geometry::getaabbTable[_type] =	Geometry::GetBoundingVolume<_name>;	\
	Geometry::raycastTable[_type] =	Geometry::RayCast<_name>;			\
	Geometry::supportTable[_type] =	Geometry::GetSupport<_name>;		\
	Geometry::inertiaTable[_type] =	Geometry::GetInertia<_name>;		\
	Geometry::destoryTable[_type] =	Geometry::Destory<_name>;			\

class Geometry_Registration
{
public:
	Geometry_Registration()
	{
		REG_GEOMETRY_OBJ(GeometryShapeType::OBB, OrientedBox3d)
		REG_GEOMETRY_OBJ(GeometryShapeType::PLANE, Plane3d)
		REG_GEOMETRY_OBJ(GeometryShapeType::SPHERE, Sphere3d)
		REG_GEOMETRY_OBJ(GeometryShapeType::CAPSULE, Capsule3d)
		REG_GEOMETRY_OBJ(GeometryShapeType::TRIANGLE, Triangle3d)
		REG_GEOMETRY_OBJ(GeometryShapeType::TRIANGLE_MESH, TriangleMesh)
	}
};
Geometry_Registration s_geom_registration;

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

Geometry* GeometryFactory::CreateTriangleMesh(const Vector3d& Position)
{
	TriangleMesh* shape = new TriangleMesh;
	return new Geometry(Position, GeometryShapeType::TRIANGLE_MESH, shape, nullptr);
}