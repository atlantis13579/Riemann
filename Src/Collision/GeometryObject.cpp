
#include "GeometryObject.h"
#include "GeometryQuery.h"
#include "GeometryIntersection.h"

#include "../CollisionPrimitive/AxisAlignedBox3.h"
#include "../CollisionPrimitive/Plane3.h"
#include "../CollisionPrimitive/Sphere3.h"
#include "../CollisionPrimitive/Ray3.h"
#include "../CollisionPrimitive/Triangle3.h"
#include "../CollisionPrimitive/HeightField3.h"
#include "../CollisionPrimitive/Cylinder3.h"
#include "../CollisionPrimitive/Capsule3.h"
#include "../CollisionPrimitive/ConvexMesh.h"
#include "../CollisionPrimitive/TriangleMesh.h"

namespace Riemann
{
	int GeometryFactory::ObjectCount[(int)PrimitiveType::TYPE_COUNT] = { 0 };

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

		virtual void			UpdateVolumeProperties() override final
		{
			GEOM_TYPE::CalculateVolumeProperties(&m_VolumeProperties, m_Density);
		}

		virtual Vector3			CalculateSupport_LocalSpace(const Vector3& Direction) const override final
		{
			return GEOM_TYPE::GetSupport(Direction);
		}

		virtual Vector3			GetCenter_LocalSpace() const override final
		{
			return GEOM_TYPE::GetCenter();
		}

		virtual void			CalculateSupportFace_LocalSpace(const Vector3& Direction, SupportFace& Face) const override final
		{
			Face.resize(GEOM_TYPE::GetSupportFace(Direction, Face.data()));
		}

		virtual bool			RayCast(const Vector3& Origin, const Vector3& Direction, const RayCastOption* Option, RayCastResult* Result) const override final
		{
			if (Option->Filter && !Option->Filter->IsCollidable(Option->FilterData, this->GetFilterData()))
			{
				return false;
			}
			Result->AddTestCount(1);

			const Vector3 Origin_Local = m_WorldTransform.WorldToLocal(Origin);
			const Vector3 Dir_Local = m_WorldTransform.WorldToLocalDirection(Direction);
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
	bool				TGeometry<TriangleMesh>::RayCast(const Vector3& Origin, const Vector3& Direction, const RayCastOption* Option, RayCastResult* Result) const
	{
		if (Option->Filter && !Option->Filter->IsCollidable(Option->FilterData, this->GetFilterData()))
		{
			return false;
		}

		TriMeshHitOption HitOption;
		HitOption.hitBothSides = Option->HitBothSides;
		HitOption.hitNearest = Option->Type == RayCastOption::RAYCAST_NEAREST;
		HitOption.maxDist = Option->MaxDist;

		TriMeshHitResult HitResult = { 0 };
		const Vector3 Origin_Local = m_WorldTransform.WorldToLocal(Origin);
		const Vector3 Dir_Local = m_WorldTransform.WorldToLocalDirection(Direction);
		const TriangleMesh* p = (const TriangleMesh*)this;
		bool Ret = p->IntersectRay(Origin_Local, Dir_Local, HitOption, &HitResult);
		Result->hitTime = HitResult.hitTime;
		Result->AddTestCount(HitResult.hitTestCount);
		return Ret;
	}

	template<>
	bool				TGeometry<HeightField3>::RayCast(const Vector3& Origin, const Vector3& Direction, const RayCastOption* Option, RayCastResult* Result) const
	{
		if (Option->Filter && !Option->Filter->IsCollidable(Option->FilterData, this->GetFilterData()))
		{
			return false;
		}

		HeightFieldHitOption HitOption;
		HitOption.hitBothSides = Option->HitBothSides;
		HitOption.maxDist = Option->MaxDist;

		HeightFieldHitResult HitResult = { 0 };
		const Vector3 Origin_Local = m_WorldTransform.WorldToLocal(Origin);
		const Vector3 Dir_Local = m_WorldTransform.WorldToLocalDirection(Direction);
		const HeightField3* p = (const HeightField3*)this;
		bool Ret = p->IntersectRay(Origin_Local, Dir_Local, HitOption, &HitResult);
		Result->hitTime = HitResult.hitTime;
		Result->AddTestCount(HitResult.hitTestCount);
		return Ret;
	}

	Geometry::Geometry()
	{
		m_Density = 1.0f;
		m_Parent = nullptr;
		m_NodeId = -1;
	}

	Geometry::~Geometry()
	{

	}

	// static
	Transform		Geometry::CalculateCenterOfMassPoseMultibody(const std::vector<Geometry*>& geoms)
	{
		Transform p;
		p.pos = Vector3::Zero();
		float vol = 0.0f;

		for (size_t i = 0; i < geoms.size(); ++i)
		{
			Geometry* g = geoms[i];
			p.pos += g->GetWorldPosition() * g->GetMassParameters()->Mass;
			vol += g->GetMassParameters()->Volume;
		}
		if (vol > 1e-6f)
		{
			p.pos = p.pos / vol;
		}

		p.quat = (geoms.size() == 1) ? geoms[1]->GetWorldRotation() : Quaternion::One();

		return p;
	}

	bool		Geometry::Intersect(const Geometry* Geom) const
	{
		IntersectFunc func = GeometryIntersection::GetIntersectFunc(m_Type, Geom->GetShapeType());
		if (func)
		{
			return func(GetShapeObjPtr(), Geom->GetShapeObjPtr(), &m_WorldTransform, Geom->GetWorldTransform());
		}

		func = GeometryIntersection::GetIntersectFunc(Geom->GetShapeType(), m_Type);
		if (func)
		{
			return func(Geom->GetShapeObjPtr(), GetShapeObjPtr(), Geom->GetWorldTransform(), &m_WorldTransform);
		}

		assert(false);
		return false;
	}

	bool		Geometry::Penetration(const Geometry* Geom, Vector3* Normal, float* Depth) const
	{
		PenetrationFunc func = GeometryIntersection::GetPenetrationFunc(m_Type, Geom->GetShapeType());
		if (func)
		{
			return func(GetShapeObjPtr(), Geom->GetShapeObjPtr(), &m_WorldTransform, Geom->GetWorldTransform(), Normal, Depth);
		}

		func = GeometryIntersection::GetPenetrationFunc(Geom->GetShapeType(), m_Type);
		if (func)
		{
			bool succ = func(Geom->GetShapeObjPtr(), GetShapeObjPtr(), Geom->GetWorldTransform(), &m_WorldTransform, Normal, Depth);
			if (succ)
			{
				*Normal = -*Normal;
				return true;
			}
			return false;
		}

		assert(false);
		return false;
	}

	bool		Geometry::SweepTestFast(const Vector3& Direction, const Vector3& Bmin, const Vector3& Bmax, float* t) const
	{
		Box3 box = GetBoundingVolume_WorldSpace();
		Vector3 size = box.GetSize();
		Vector3 BminExtend = Bmin - size;
		Vector3 BmaxExtend = Bmax + size;
        return Ray3::RayIntersectAABB(box.GetCenter(), Direction, BminExtend, BmaxExtend, t);
	}

	bool		Geometry::Sweep(const Vector3& Direction, const Geometry* Geom, Vector3* position, Vector3* normal, float* t) const
	{
		SweepFunc func = GeometryIntersection::GetSweepFunc(m_Type, Geom->GetShapeType());
		assert(func);
		return func(GetShapeObjPtr(), Geom->GetShapeObjPtr(), &m_WorldTransform, Geom->GetWorldTransform(), Direction, position, normal, t);
	}

	void 		Geometry::UpdateBoundingVolume()
	{
		m_BoxWorld = Box3::Transform(m_VolumeProperties.BoundingVolume, m_WorldTransform.pos, m_WorldTransform.quat);
	}

	Vector3		Geometry::GetCenter_WorldSpace() const
	{
		Vector3 CenterLocal = GetCenter_LocalSpace();
		Vector3 CenterWorld = m_WorldTransform.LocalToWorld(CenterLocal);
		return CenterLocal;
	}

	Vector3		Geometry::GetSupport_WorldSpace(const Vector3& Direction) const
	{
		Vector3 DirLocal = m_WorldTransform.WorldToLocalDirection(Direction);
		Vector3 SupportLocal = CalculateSupport_LocalSpace(DirLocal);
		Vector3 SupportWorld = m_WorldTransform.LocalToWorld(SupportLocal);
		return SupportWorld;
	}

	void		Geometry::GetSupportFace_WorldSpace(const Vector3& Direction, SupportFace& Face) const
	{
		Vector3 DirLocal = m_WorldTransform.WorldToLocalDirection(Direction);
		CalculateSupportFace_LocalSpace(DirLocal, Face);
		for (int i = 0; i < Face.size(); ++i)
		{
			Face[i] = m_WorldTransform.LocalToWorld(Face[i]);
		}
	}

	Matrix3		Geometry::GetInertiaTensor_LocalSpace() const
	{
		return m_VolumeProperties.InertiaMat;
	}

	void 		GeometryFactory::DeleteGeometry(Geometry* Geom)
	{
		delete Geom;
	}

	Geometry* GeometryFactory::CreateOBB_placement(void* pBuf, const Vector3& Center, const Vector3& HalfExtent, const Quaternion& Rot)
	{
		TGeometry<AxisAlignedBox3>* p = pBuf ? new (pBuf) TGeometry<AxisAlignedBox3>() : new TGeometry<AxisAlignedBox3>();
		p->Min = -HalfExtent;
		p->Max = HalfExtent;
		p->UpdateVolumeProperties();
		p->SetWorldTransform(Center, Rot);

		return (Geometry*)p;
	}

	Geometry* GeometryFactory::CreateSphere_placement(void* pBuf, const Vector3& Center, float Radius)
	{
		TGeometry<Sphere3>* p = pBuf ? new (pBuf)TGeometry<Sphere3>() : new TGeometry<Sphere3>();
		p->Center = Vector3::Zero();
		p->Radius = Radius;
		p->UpdateVolumeProperties();
		p->SetWorldTransform(Center, Quaternion::One());
		return (Geometry*)p;
	}

	Geometry* GeometryFactory::CreateCapsule_placement(void* pBuf, const Vector3& x0, const Vector3& x1, float Radius)
	{
		Quaternion quat = Quaternion::One();
		if (!(x1 - x0).IsZero())
		{
			quat.FromTwoAxis(Vector3::UnitY(), x1 - x0);
		}
		Vector3 Center = (x0 + x1) * 0.5f;
		const float HalfHeight = (x1 - x0).Length();
		TGeometry<Capsule3>* p = pBuf ? new (pBuf)TGeometry<Capsule3>() : new TGeometry<Capsule3>();
		p->Init(Vector3(0.0f, -HalfHeight, 0.0f), Vector3(0.0f, HalfHeight, 0.0f), Radius);
		p->UpdateVolumeProperties();
		p->SetWorldTransform(Center, quat);
		return (Geometry*)p;
	}

	Geometry* GeometryFactory::CreateOBB(const Vector3& Center, const Vector3& HalfExtent, const Quaternion& Rot)
	{
		TGeometry<AxisAlignedBox3>* p = new TGeometry<AxisAlignedBox3>();
		return CreateOBB_placement(p, Center, HalfExtent, Rot);
	}

	Geometry* GeometryFactory::CreatePlane(const Vector3& Center, const Vector3& Normal)
	{
		TGeometry<Plane3>* p = new TGeometry<Plane3>();
		p->Normal = Vector3::UnitY();
		p->D = 0.0f;
		Quaternion quat;
		quat.FromTwoAxis(Vector3::UnitY(), Normal);
		p->UpdateVolumeProperties();
		p->SetWorldTransform(Center, quat);
		return (Geometry*)p;
	}

	Geometry* GeometryFactory::CreateSphere(const Vector3& Center, float Radius)
	{
		TGeometry<Sphere3>* p = new TGeometry<Sphere3>();
		return CreateSphere_placement(p, Center, Radius);
	}

	Geometry* GeometryFactory::CreateCylinder(const Vector3& x0, const Vector3& x1, float Radius)
	{
		Quaternion quat = Quaternion::One();
		if ((x1 - x0).SquareLength() > 1e-6)
		{
			quat.FromTwoAxis(Vector3::UnitY(), x1 - x0);
		}
		Vector3 Center = (x0 + x1) * 0.5f;
		const float HalfHeight = (x1 - x0).Length();
		TGeometry<Cylinder3>* p = new TGeometry<Cylinder3>();
		p->Init(Vector3(0.0f, -HalfHeight, 0.0f), Vector3(0.0f, HalfHeight, 0.0f), Radius);
		p->UpdateVolumeProperties();
		p->SetWorldTransform(Center, quat);
		return (Geometry*)p;
	}

	Geometry* GeometryFactory::CreateCapsule(const Vector3& x0, const Vector3& x1, float Radius)
	{
		TGeometry<Capsule3>* p = new TGeometry<Capsule3>();
		return CreateCapsule_placement(p, x0, x1, Radius);
	}

	Geometry* GeometryFactory::CreateConvexMesh()
	{
		TGeometry<ConvexMesh>* p = new TGeometry<ConvexMesh>();
		p->UpdateVolumeProperties();
		p->SetWorldTransform(Vector3::Zero(), Quaternion::One());
		return (Geometry*)p;
	}

	Geometry* GeometryFactory::CreateHeightField(const Box3& Bv, int nRows, int nCols)
	{
		TGeometry<HeightField3>* p = new TGeometry<HeightField3>();
		p->Init(Bv, nRows, nCols);
		p->UpdateVolumeProperties();
		p->SetWorldTransform(Vector3::Zero(), Quaternion::One());
		return (Geometry*)p;
	}

	Geometry* GeometryFactory::CreateTriangleMesh()
	{
		TGeometry<TriangleMesh>* p = new TGeometry<TriangleMesh>();
		p->UpdateVolumeProperties();
		p->SetWorldTransform(Vector3::Zero(), Quaternion::One());
		return (Geometry*)p;
	}
}
