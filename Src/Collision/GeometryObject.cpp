
#include "GeometryObject.h"

#include "../CollisionPrimitive/AxisAlignedBox3d.h"
#include "../CollisionPrimitive/Plane3d.h"
#include "../CollisionPrimitive/Sphere3d.h"
#include "../CollisionPrimitive/Triangle3d.h"
#include "../CollisionPrimitive/HeightField3d.h"
#include "../CollisionPrimitive/Cylinder3d.h"
#include "../CollisionPrimitive/Capsule3d.h"
#include "../CollisionPrimitive/ConvexMesh.h"
#include "../CollisionPrimitive/TriangleMesh.h"

template<class GEOM_TYPE>
class TGeometry : public Geometry, public GEOM_TYPE
{
public:
	TGeometry()
	{
		m_Type = GEOM_TYPE::StaticType();
	}
	virtual ~TGeometry() {}

	virtual Matrix3d		GetInertia_LocalSpace(float Mass) const override final
	{
		return GEOM_TYPE::GetInertiaTensor(Mass);
	}

	virtual Vector3d		GetSupport_LocalSpace(const Vector3d& Dir) const override final
	{
		return GEOM_TYPE::GetSupport(Dir);
	}

	virtual Box3d			GetBoundingVolume_LocalSpace() const override final
	{
		return GEOM_TYPE::GetBoundingVolume();
	}
};

const Box3d&		Geometry::GetBoundingVolume_WorldSpace() const
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
	UpdateBoundingVolume();
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
	UpdateBoundingVolume();
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
	void* Obj = GetShapeObjPtr();
	const Vector3d Origin_Local = m_Transform.WorldToLocal(Origin);
	const Vector3d Dir_Local = m_Transform.RotateWorldToLocal(Dir);
	return func(Obj, Origin_Local, Dir_Local, t);
}

bool				Geometry::Overlap(const Geometry* Geom) const
{
	ShapeType Type1 = GetShapeType();
	ShapeType Type2 = Geom->GetShapeType();
	OverlapFunc func = Geometry::overlapTable[Type1][Type1];
#ifdef DEBUG
	assert(func);
#endif
	return func(GetShapeObjPtr(), Geom->GetShapeObjPtr());
}

bool				Geometry::Sweep(const Geometry* Geom, const Vector3d& Dir, float* t) const
{
	ShapeType Type1 = GetShapeType();
	ShapeType Type2 = Geom->GetShapeType();
	SweepFunc func = Geometry::sweepTable[Type1][Type1];
#ifdef DEBUG
	assert(func);
#endif
	return func(GetShapeObjPtr(), Geom->GetShapeObjPtr(), Dir, t);
}

void Geometry::UpdateBoundingVolume()
{
	m_BoxWorld = GetBoundingVolume_LocalSpace().Transform(m_Transform.GetWorldMatrix());
}

Vector3d			Geometry::GetSupport_WorldSpace(const Vector3d& Dir)
{
	Vector3d DirLocal = m_Transform.WorldToLocal(Dir);
	Vector3d SupportLocal = GetSupport_LocalSpace(DirLocal);
	Vector3d SupportWorld = m_Transform.LocalToWorld(SupportLocal);
	return SupportWorld;
}

Matrix3d			Geometry::GetInverseInertia_WorldSpace(float Mass) const
{
	return GetInertia_LocalSpace(Mass).Inverse();
}

template <class T>
static bool			RayCastT(void* Obj, const Vector3d& Origin, const Vector3d& Dir, float* t)
{
	T* p = reinterpret_cast<T*>(Obj);
	return p->IntersectRay(Origin, Dir, t);
}

RayCastFunc			Geometry::raycastTable[ShapeType::GEOMETRY_COUNT] = { 0 };
SweepFunc			Geometry::sweepTable[ShapeType::GEOMETRY_COUNT][ShapeType::GEOMETRY_COUNT] = { 0 };
OverlapFunc			Geometry::overlapTable[ShapeType::GEOMETRY_COUNT][ShapeType::GEOMETRY_COUNT] = {0};

#define	REG_GEOMETRY_OBJ(_type, _name)									\
	Geometry::raycastTable[_type] =	RayCastT<_name>;

class GeometryRegistration
{
public:
	GeometryRegistration()
	{
		REG_GEOMETRY_OBJ(ShapeType::BOX, AxisAlignedBox3d)
		REG_GEOMETRY_OBJ(ShapeType::PLANE, Plane3d)
		REG_GEOMETRY_OBJ(ShapeType::SPHERE, Sphere3d)
		REG_GEOMETRY_OBJ(ShapeType::CAPSULE, Capsule3d)
		REG_GEOMETRY_OBJ(ShapeType::HEIGHTFIELD, HeightField3d)
		REG_GEOMETRY_OBJ(ShapeType::CONVEX_MESH, ConvexMesh)
		REG_GEOMETRY_OBJ(ShapeType::TRIANGLE, Triangle3d)
		REG_GEOMETRY_OBJ(ShapeType::TRIANGLE_MESH, TriangleMesh)
	}
};
GeometryRegistration s_geom_registration;

void GeometryFactory::DeleteGeometry(Geometry* Geom)
{
	delete Geom;
}

Geometry* GeometryFactory::CreateOBB(const Vector3d& Center, const Vector3d& Extent, const Quaternion& Rot)
{
	TGeometry<AxisAlignedBox3d>* p = new TGeometry<AxisAlignedBox3d>();
	p->Min = -Extent;
	p->Max = Extent;
	p->SetPosition(Center);
	p->SetRotationQuat(Rot);
	return (Geometry*)p;
}

Geometry* GeometryFactory::CreatePlane(const Vector3d& Center, const Vector3d& Normal)
{
	TGeometry<Plane3d>* p = new TGeometry<Plane3d>();
	p->Normal = Normal;
	p->D = 0.0f;
	p->SetPosition(Center);
	p->SetRotationQuat(Quaternion::One());
	return (Geometry*)p;
}

Geometry* GeometryFactory::CreateSphere(const Vector3d& Center, float Radius)
{
	TGeometry<Sphere3d>* p = new TGeometry<Sphere3d>();
	p->Radius = Radius;
	p->SetPosition(Center);
	p->SetRotationQuat(Quaternion::One());
	return (Geometry*)p;
}

Geometry* GeometryFactory::CreateCapsule(const Vector3d& X1, const Vector3d& X2, float Radius)
{
	Vector3d Center = (X1 + X2) * 0.5f;
	TGeometry<Capsule3d>* p = new TGeometry<Capsule3d>();
	p->Init(X1 - Center, X2 - Center, Radius);
	p->SetPosition(Center);
	p->SetRotationQuat(Quaternion::One());
	return (Geometry*)p;
}

Geometry* GeometryFactory::CreateHeightField(const Box3d &Bv, int nRows, int nCols)
{
	TGeometry<HeightField3d>* p = new TGeometry<HeightField3d>();
	p->Init(Bv, nRows, nCols);
	p->SetPosition(Vector3d::Zero());
	p->SetRotationQuat(Quaternion::One());
	return (Geometry*)p;
}

Geometry* GeometryFactory::CreateConvexMesh()
{
	TGeometry<ConvexMesh>* p = new TGeometry<ConvexMesh>();
	p->SetPosition(Vector3d::Zero());
	p->SetRotationQuat(Quaternion::One());
	return (Geometry*)p;
}

Geometry* GeometryFactory::CreateTriangle(const Vector3d& A, const Vector3d& B, const Vector3d& C)
{
	Vector3d Center = (A + B + C) / 3.0f;
	TGeometry<Triangle3d>* p = new TGeometry<Triangle3d>();
	p->Init(A - Center, B - Center, C - Center);
	p->SetPosition(Center);
	p->SetRotationQuat(Quaternion::One());
	return (Geometry*)p;
}

Geometry* GeometryFactory::CreateTriangleMesh()
{
	TGeometry<TriangleMesh>* p = new TGeometry<TriangleMesh>();
	p->SetPosition(Vector3d::Zero());
	p->SetRotationQuat(Quaternion::One());
	return (Geometry*)p;
}
