
#include "GeometryObject.h"

#include "../CollisionPrimitive/AxisAlignedBox3d.h"
#include "../CollisionPrimitive/Plane3d.h"
#include "../CollisionPrimitive/Sphere3d.h"
#include "../CollisionPrimitive/Triangle3d.h"
#include "../CollisionPrimitive/HeightField3d.h"
#include "../CollisionPrimitive/Cylinder3d.h"
#include "../CollisionPrimitive/Capsule3d.h"
#include "../CollisionPrimitive/ConvexMesh.h"
#include "TriangleMesh.h"

template<class GEOM_TYPE>
class TGeometry : public Geometry, public GEOM_TYPE
{
public:
	TGeometry(const Vector3d& Translation, const Quaternion& Rotation)
	{
		m_Type = GEOM_TYPE::StaticType();
		SetPosition(Translation);
		SetRotationQuat(Rotation);
	}
	virtual ~TGeometry() {}

	virtual const void*		GetGeometryObj() const override final
	{
		return static_cast<const GEOM_TYPE*>(this);
	}

	virtual void*			GetGeometryObj() override final
	{
		return static_cast<GEOM_TYPE*>(this);
	}

	virtual Matrix3d		GetInertiaLocalSpace(float Mass) const override final
	{
		return GEOM_TYPE::GetInertiaTensor(Mass);
	}

	virtual Vector3d		GetSupportLocalSpace(const Vector3d& Dir) const override final
	{
		return GEOM_TYPE::GetSupport(Dir);
	}

	virtual Box3d			GetBoundingVolumeLocalSpace() const override final
	{
		return GEOM_TYPE::GetBoundingVolume();
	}
};

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

void* Geometry::GetEntity()
{
	return m_Entity;
}

void				Geometry::SetEntity(void* Entity)
{
	m_Entity = Entity;
}

bool				Geometry::RayCast(const Vector3d& Origin, const Vector3d& Dir, float* t)
{
	RayCastFunc func = Geometry::raycastTable[m_Type];
#ifdef DEBUG
	assert(func);
#endif
	void* Obj = GetGeometryObj();
	const Vector3d Origin_Local = m_Transform.WorldToLocal(Origin);
	const Vector3d Dir_Local = m_Transform.RotateWorldToLocal(Dir);
	return func(Obj, Origin_Local, Dir_Local, t);
}

bool				Geometry::Overlap(const Geometry* Geom) const
{
	GeometryType Type1 = GetGeometryType();
	GeometryType Type2 = Geom->GetGeometryType();
	OverlapFunc func = Geometry::overlapTable[Type1][Type1];
#ifdef DEBUG
	assert(func);
#endif
	return func(GetGeometryObj(), Geom->GetGeometryObj());
}

bool				Geometry::Sweep(const Geometry* Geom, const Vector3d& Dir, float* t) const
{
	GeometryType Type1 = GetGeometryType();
	GeometryType Type2 = Geom->GetGeometryType();
	SweepFunc func = Geometry::sweepTable[Type1][Type1];
#ifdef DEBUG
	assert(func);
#endif
	return func(GetGeometryObj(), Geom->GetGeometryObj(), Dir, t);
}

Vector3d			Geometry::GetSupportWorldSpace(const Vector3d& Dir)
{
	Vector3d DirLocal = m_Transform.WorldToLocal(Dir);
	Vector3d SupportLocal = GetSupportLocalSpace(DirLocal);
	Vector3d SupportWorld = m_Transform.LocalToWorld(SupportLocal);
	return SupportWorld;
}

Matrix3d			Geometry::GetInverseInertia(float Mass) const
{
	return GetInertiaLocalSpace(Mass).Inverse();
}

template <class T>
static bool			_RayCast(void* Obj, const Vector3d& Origin, const Vector3d& Dir, float* t)
{
	T* p = reinterpret_cast<T*>(Obj);
	return p->IntersectRay(Origin, Dir, t);
}

RayCastFunc			Geometry::raycastTable[GeometryType::GEOMETRY_COUNT] = { 0 };
SweepFunc			Geometry::sweepTable[GeometryType::GEOMETRY_COUNT][GeometryType::GEOMETRY_COUNT] = { 0 };
OverlapFunc			Geometry::overlapTable[GeometryType::GEOMETRY_COUNT][GeometryType::GEOMETRY_COUNT] = {0};

#define	REG_GEOMETRY_OBJ(_type, _name)									\
	Geometry::raycastTable[_type] =	_RayCast<_name>;

class Geometry_Registration
{
public:
	Geometry_Registration()
	{
		REG_GEOMETRY_OBJ(GeometryType::BOX, AxisAlignedBox3d)
		REG_GEOMETRY_OBJ(GeometryType::PLANE, Plane3d)
		REG_GEOMETRY_OBJ(GeometryType::SPHERE, Sphere3d)
		REG_GEOMETRY_OBJ(GeometryType::CAPSULE, Capsule3d)
		REG_GEOMETRY_OBJ(GeometryType::HEIGHTFIELD, HeightField3d)
		REG_GEOMETRY_OBJ(GeometryType::CONVEX_MESH, ConvexMesh)
		REG_GEOMETRY_OBJ(GeometryType::TRIANGLE, Triangle3d)
		REG_GEOMETRY_OBJ(GeometryType::TRIANGLE_MESH, TriangleMesh)
	}
};
Geometry_Registration s_geom_registration;

Geometry* GeometryFactory::CreateOBB(const Vector3d& Center, const Vector3d& Extent, const Quaternion& Rot)
{
	TGeometry<AxisAlignedBox3d>* p = new TGeometry<AxisAlignedBox3d>(Center, Rot);
	p->Min = -Extent;
	p->Max = Extent;
	return (Geometry*)p;
}

Geometry* GeometryFactory::CreatePlane(const Vector3d& Center, const Vector3d& Normal)
{
	TGeometry<Plane3d>* p = new TGeometry<Plane3d>(Center, Quaternion::One());
	p->Normal = Normal;
	p->D = 0.0f;
	return (Geometry*)p;
}

Geometry* GeometryFactory::CreateSphere(const Vector3d& Center, float Radius)
{
	TGeometry<Sphere3d>* p = new TGeometry<Sphere3d>(Center, Quaternion::One());
	p->Radius = Radius;
	return (Geometry*)p;
}

Geometry* GeometryFactory::CreateCapsule(const Vector3d& X1, const Vector3d& X2, float Radius)
{
	Vector3d Center = (X1 + X2) * 0.5f;
	TGeometry<Capsule3d>* p = new TGeometry<Capsule3d>(Center, Quaternion::One());
	p->Init(X1 - Center, X2 - Center, Radius);
	return (Geometry*)p;
}

Geometry* GeometryFactory::CreateHeightField(const Box3d &Bv, int nRows, int nCols)
{
	TGeometry<HeightField3d>* p = new TGeometry<HeightField3d>(Vector3d::Zero(), Quaternion::One());
	p->Init(Bv, nRows, nCols);
	return (Geometry*)p;
}

Geometry* GeometryFactory::CreateConvexMesh()
{
	TGeometry<ConvexMesh>* p = new TGeometry<ConvexMesh>(Vector3d::Zero(), Quaternion::One());
	return (Geometry*)p;
}

Geometry* GeometryFactory::CreateTriangle(const Vector3d& A, const Vector3d& B, const Vector3d& C)
{
	Vector3d Center = (A + B + C) / 3.0f;
	TGeometry<Triangle3d>* p = new TGeometry<Triangle3d>(Center, Quaternion::One());
	p->Init(A - Center, B - Center, C - Center);
	return (Geometry*)p;
}

Geometry* GeometryFactory::CreateTriangleMesh()
{
	TGeometry<TriangleMesh>* p = new TGeometry<TriangleMesh>(Vector3d::Zero(), Quaternion::One());
	return (Geometry*)p;
}
