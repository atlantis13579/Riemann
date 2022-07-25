
#include "GeometryObject.h"
#include "GeometryQuery.h"
#include "GeometryIntersection.h"

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
		GeometryFactory::ObjectCount[(int)m_Type]++;
	}
	virtual ~TGeometry()
	{
		GeometryFactory::ObjectCount[(int)m_Type]--;
	}

	virtual Matrix3		GetInertia_LocalSpace(float Mass) const override final
	{
		return GEOM_TYPE::GetInertiaTensor(Mass);
	}

	virtual Vector3		GetSupport_LocalSpace(const Vector3& Dir) const override final
	{
		return GEOM_TYPE::GetSupport(Dir);
	}

	virtual void			GetSupportFace_LocalSpace(const Vector3& Dir, SupportFace& Face) const override final
	{
		Face.SetSize(GEOM_TYPE::GetSupportFace(Dir, Face.GetData()));
	}

	virtual Box3d			GetBoundingVolume_LocalSpace() const override final
	{
		return GEOM_TYPE::GetBoundingVolume();
	}

	virtual bool			RayCast(const Vector3& Origin, const Vector3& Dir, const RayCastOption* Option, RayCastResult* Result) const override final
	{
		Result->AddTestCount(1);

		const Vector3 Origin_Local = m_Transform.WorldToLocalEx(Origin);
		const Vector3 Dir_Local = m_Transform.WorldToLocalDirection(Dir);
		const GEOM_TYPE* p = (const GEOM_TYPE*)this;
		float t;
		if (p->IntersectRay(Origin_Local, Dir_Local, &t) && t < Option->MaxDist)
		{
			Result->hitTime = t;
			return true;
		}
		return false;
	}
};

template<>
bool				TGeometry<TriangleMesh>::RayCast(const Vector3& Origin, const Vector3& Dir, const RayCastOption* Option, RayCastResult* Result) const
{
	TriMeshHitOption HitOption;
	HitOption.hitNearest = Option->Type == RayCastOption::RAYCAST_NEAREST;
	HitOption.maxDist = Option->MaxDist;

	TriMeshHitResult HitResult = { 0 };
	const Vector3 Origin_Local = m_Transform.WorldToLocalEx(Origin);
	const Vector3 Dir_Local = m_Transform.WorldToLocalDirection(Dir);
	const TriangleMesh* p = (const TriangleMesh*)this;
	bool Ret = p->IntersectRay(Origin_Local, Dir_Local, HitOption, &HitResult);
	Result->hitTime = HitResult.hitTime;
	Result->AddTestCount(HitResult.hitTestCount);
	return Ret;
}

template<>
bool				TGeometry<HeightField3d>::RayCast(const Vector3& Origin, const Vector3& Dir, const RayCastOption* Option, RayCastResult* Result) const
{
	HeightFieldHitOption HitOption;
	HitOption.maxDist = Option->MaxDist;

	HeightFieldHitResult HitResult = { 0 };
	const Vector3 Origin_Local = m_Transform.WorldToLocalEx(Origin);
	const Vector3 Dir_Local = m_Transform.WorldToLocalDirection(Dir);
	const HeightField3d* p = (const HeightField3d*)this;
	bool Ret = p->IntersectRay(Origin_Local, Dir_Local, HitOption, &HitResult);
	Result->hitTime = HitResult.hitTime;
	Result->AddTestCount(HitResult.hitTestCount);
	return Ret;
}

const Box3d&		Geometry::GetBoundingVolume_WorldSpace() const
{
	return m_BoxWorld;
}

Vector3			Geometry::GetPosition() const
{
	return m_Transform.GetTranslation();
}

void				Geometry::SetPosition(const Vector3& Position)
{
	m_Transform.SetTranslation(Position);
	return;
}

Matrix3			Geometry::GetRotationMatrix() const
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
}

const Matrix4&		Geometry::GetWorldMatrix()
{
	return m_Transform.GetWorldMatrix();
}

const Matrix4&		Geometry::GetInverseWorldMatrix()
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

bool				Geometry::Overlap(const Geometry* Geom) const
{
	OverlapFunc func = GeometryIntersection::GetOverlapFunc(m_Type, Geom->GetShapeType());
	assert(func);
	Transform trans;
	trans.LoadLocal1ToLocal2(m_Transform, *Geom->GetTransform());
	return func(GetShapeObjPtr(), Geom->GetShapeObjPtr(), trans);
}

bool				Geometry::Sweep(const Geometry* Geom, const Vector3& Dir, float* t) const
{
	SweepFunc func = GeometryIntersection::GetSweepFunc(m_Type, Geom->GetShapeType());
	assert(func);
	return func(GetShapeObjPtr(), Geom->GetShapeObjPtr(), Dir, t);
}

void 				Geometry::UpdateBoundingVolume()
{
	m_BoxWorld = GetBoundingVolume_LocalSpace().Transform(m_Transform.GetWorldMatrix());
}

Vector3			Geometry::GetSupport_WorldSpace(const Vector3& Dir) const
{
	Vector3 DirLocal = m_Transform.WorldToLocalDirection(Dir);
	Vector3 SupportLocal = GetSupport_LocalSpace(DirLocal);
	Vector3 SupportWorld = m_Transform.LocalToWorldEx(SupportLocal);
	return SupportWorld;
}

void Geometry::GetSupportFace_WorldSpace(const Vector3& Dir, SupportFace& Face) const
{
	Vector3 DirLocal = m_Transform.WorldToLocalDirection(Dir);
	GetSupportFace_LocalSpace(DirLocal, Face);
	for (int i = 0; i < Face.GetSize(); ++i)
	{
		Face[i] = m_Transform.LocalToWorldEx(Face[i]);
	}
}

Matrix3			Geometry::GetInverseInertia_LocalSpace(float InvMass) const
{
	if (InvMass == 0.0f)
	{
		return Matrix3::Zero();
	}
	return GetInertia_LocalSpace(1.0f / InvMass).Inverse();
}

void 				GeometryFactory::DeleteGeometry(Geometry* Geom)
{
	delete Geom;
}

int GeometryFactory::ObjectCount[(int)ShapeType3d::TYPE_COUNT] = { 0 };

Geometry* GeometryFactory::CreateOBB(const Vector3& Center, const Vector3& HalfExtent, const Quaternion& Rot)
{
	TGeometry<AxisAlignedBox3d>* p = new TGeometry<AxisAlignedBox3d>();
	p->Min = -HalfExtent;
	p->Max = HalfExtent;
	p->SetPosition(Center);
	p->SetRotationQuat(Rot);
	p->UpdateBoundingVolume();
	return (Geometry*)p;
}

Geometry* GeometryFactory::CreatePlane(const Vector3& Center, const Vector3& Normal, float HalfThickness)
{
	TGeometry<Plane3d>* p = new TGeometry<Plane3d>();
	p->Normal = Vector3::UnitY();
	p->D = 0.0f;
	p->HalfThickness = HalfThickness;
	p->SetPosition(Center);
	Quaternion quat;
	quat.FromTwoAxis(p->Normal, Normal);
	p->SetRotationQuat(quat);
	p->UpdateBoundingVolume();
	return (Geometry*)p;
}

Geometry* GeometryFactory::CreateSphere(const Vector3& Center, float Radius)
{
	TGeometry<Sphere3d>* p = new TGeometry<Sphere3d>();
	p->Center = Vector3::Zero();
	p->Radius = Radius;
	p->SetPosition(Center);
	p->SetRotationQuat(Quaternion::One());
	p->UpdateBoundingVolume();
	return (Geometry*)p;
}

Geometry* GeometryFactory::CreateCapsule(const Vector3& X1, const Vector3& X2, float Radius)
{
	Quaternion quat = Quaternion::One();
	if ((X2 - X1).SquareLength() < 1e-9)
	{
		quat.FromTwoAxis(Vector3::UnitY(), X2 - X1);
	}
	Vector3 Center = (X1 + X2) * 0.5f;
	TGeometry<Capsule3d>* p = new TGeometry<Capsule3d>();
	p->Init(X1 - Center, X2 - Center, Radius);
	p->SetPosition(Center);
	p->SetRotationQuat(quat);
	p->UpdateBoundingVolume();
	return (Geometry*)p;
}

Geometry* GeometryFactory::CreateHeightField(const Box3d &Bv, int nRows, int nCols)
{
	TGeometry<HeightField3d>* p = new TGeometry<HeightField3d>();
	p->Init(Bv, nRows, nCols);
	p->SetPosition(Vector3::Zero());
	p->SetRotationQuat(Quaternion::One());
	p->UpdateBoundingVolume();
	return (Geometry*)p;
}

Geometry* GeometryFactory::CreateConvexMesh()
{
	TGeometry<ConvexMesh>* p = new TGeometry<ConvexMesh>();
	p->SetPosition(Vector3::Zero());
	p->SetRotationQuat(Quaternion::One());
	p->UpdateBoundingVolume();
	return (Geometry*)p;
}

Geometry* GeometryFactory::CreateTriangle(const Vector3& A, const Vector3& B, const Vector3& C)
{
	Vector3 Center = (A + B + C) / 3.0f;
	TGeometry<Triangle3d>* p = new TGeometry<Triangle3d>();
	p->Init(A - Center, B - Center, C - Center);
	p->SetPosition(Center);
	p->SetRotationQuat(Quaternion::One());
	p->UpdateBoundingVolume();
	return (Geometry*)p;
}

Geometry* GeometryFactory::CreateTriangleMesh()
{
	TGeometry<TriangleMesh>* p = new TGeometry<TriangleMesh>();
	p->SetPosition(Vector3::Zero());
	p->SetRotationQuat(Quaternion::One());
	p->UpdateBoundingVolume();
	return (Geometry*)p;
}
