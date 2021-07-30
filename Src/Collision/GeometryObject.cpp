
#include "GeometryObject.h"

#include "../CollisionPrimitive/AxisAlignedBox3d.h"
#include "../CollisionPrimitive/Plane3d.h"
#include "../CollisionPrimitive/Sphere3d.h"
#include "../CollisionPrimitive/Triangle3d.h"
#include "../CollisionPrimitive/Cylinder3d.h"
#include "../CollisionPrimitive/Capsule3d.h"
#include "../CollisionPrimitive/ConvexMesh.h"
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

bool				Geometry::RayCast(const Vector3d& Origin, const Vector3d& Dir, float* t)
{
	RayCastFunc func = Geometry::raycastTable[m_Shape.Type];
#ifdef DEBUG
	assert(func);
#endif
	return func(m_Shape.Object, m_Transform.WorldToLocal(Origin), m_Transform.RotateWorldToLocal(Dir), t);
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
		REG_GEOMETRY_OBJ(GeometryShapeType::OBB, AxisAlignedBox3d)
		REG_GEOMETRY_OBJ(GeometryShapeType::PLANE, Plane3d)
		REG_GEOMETRY_OBJ(GeometryShapeType::SPHERE, Sphere3d)
		REG_GEOMETRY_OBJ(GeometryShapeType::CAPSULE, Capsule3d)
		REG_GEOMETRY_OBJ(GeometryShapeType::TRIANGLE, Triangle3d)
		REG_GEOMETRY_OBJ(GeometryShapeType::TRIANGLE_MESH, TriangleMesh)
		REG_GEOMETRY_OBJ(GeometryShapeType::CONVEX_MESH, ConvexMesh)
	}
};
Geometry_Registration s_geom_registration;

Geometry* GeometryFactory::CreateOBB(const Vector3d& Center, const Vector3d& Extent, const Quaternion& Rot)
{
	AxisAlignedBox3d* shape = new AxisAlignedBox3d(-Extent, Extent);
	Geometry *p = new Geometry(Center, GeometryShapeType::OBB, shape, nullptr);
	p->SetRotationQuat(Rot);
	return p;
}

Geometry* GeometryFactory::CreatePlane(const Vector3d& Center, const Vector3d& Normal)
{
	Plane3d* shape = new Plane3d(Normal, 0.0f);
	return new Geometry(Center, GeometryShapeType::PLANE, shape, nullptr);
}

Geometry* GeometryFactory::CreateSphere(const Vector3d& Center, float Radius)
{
	Sphere3d* shape = new Sphere3d(Vector3d::Zero(), Radius);
	return new Geometry(Center, GeometryShapeType::SPHERE, shape, nullptr);
}

Geometry* GeometryFactory::CreateCapsule(const Vector3d& X1, const Vector3d& X2, float Radius)
{
	Vector3d Center = (X1 + X2) * 0.5f;
	Capsule3d* shape = new Capsule3d(X1 - Center, X2 - Center, Radius);
	return new Geometry(Center, GeometryShapeType::CAPSULE, shape, nullptr);
}

Geometry* GeometryFactory::CreateTriangle(const Vector3d& A, const Vector3d& B, const Vector3d& C)
{
	Vector3d Center = (A + B + C) / 3.0f;
	Triangle3d* shape = new Triangle3d(A - Center, B - Center, C - Center);
	return new Geometry(Center, GeometryShapeType::TRIANGLE, shape, nullptr);
}

Geometry* GeometryFactory::CreateTriangleMesh()
{
	TriangleMesh* shape = new TriangleMesh;
	return new Geometry(Vector3d::Zero(), GeometryShapeType::TRIANGLE_MESH, shape, nullptr);
}

Geometry* GeometryFactory::CreateConvexMesh()
{
	ConvexMesh* shape = new ConvexMesh;
	return new Geometry(Vector3d::Zero(), GeometryShapeType::CONVEX_MESH, shape, nullptr);
}