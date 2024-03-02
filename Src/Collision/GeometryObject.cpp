
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

namespace Riemann
{
	int GeometryFactory::ObjectCount[(int)ShapeType3d::TYPE_COUNT] = { 0 };

	template<class GEOM_TYPE>
	class TGeometry : public GeometryBase, public GEOM_TYPE
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

		virtual void			CalculateSupportFace_LocalSpace(const Vector3& Direction, SupportFace& Face) const override final
		{
			Face.SetSize(GEOM_TYPE::GetSupportFace(Direction, Face.GetData()));
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
		TriMeshHitOption HitOption;
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
	bool				TGeometry<HeightField3d>::RayCast(const Vector3& Origin, const Vector3& Direction, const RayCastOption* Option, RayCastResult* Result) const
	{
		HeightFieldHitOption HitOption;
		HitOption.maxDist = Option->MaxDist;

		HeightFieldHitResult HitResult = { 0 };
		const Vector3 Origin_Local = m_WorldTransform.WorldToLocal(Origin);
		const Vector3 Dir_Local = m_WorldTransform.WorldToLocalDirection(Direction);
		const HeightField3d* p = (const HeightField3d*)this;
		bool Ret = p->IntersectRay(Origin_Local, Dir_Local, HitOption, &HitResult);
		Result->hitTime = HitResult.hitTime;
		Result->AddTestCount(HitResult.hitTestCount);
		return Ret;
	}

	GeometryBase::GeometryBase()
	{
		m_Density = 1.0f;
		m_Parent = nullptr;
		m_NodeId = -1;
	}

	GeometryBase::~GeometryBase()
	{

	}

	// static
	Transform		GeometryBase::CalculateCenterOfMassPoseMultibody(const std::vector<GeometryBase*>& geoms)
	{
		Transform p;
		p.pos = Vector3::Zero();
		float vol = 0.0f;

		for (size_t i = 0; i < geoms.size(); ++i)
		{
			GeometryBase* g = geoms[i];
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

	bool		GeometryBase::Intersect(const GeometryBase* Geom) const
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

	bool		GeometryBase::Penetration(const GeometryBase* Geom, Vector3* Normal, float* Depth) const
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

	bool		GeometryBase::SweepAABB(const Vector3& Direction, const Vector3& Bmin, const Vector3& Bmax, Vector3* normal, float* t) const
	{
		char stack[MAX_GEOMETRY_STACK_SIZE];
		GeometryBase* aabb = GeometryFactory::CreateOBB_placement(stack, (Bmin + Bmax) * 0.5f, (Bmax - Bmin) * 0.5f);
		return Sweep(Direction, aabb, normal, t);
	}

	bool		GeometryBase::Sweep(const Vector3& Direction, const GeometryBase* Geom, Vector3* normal, float* t) const
	{
		SweepFunc func = GeometryIntersection::GetSweepFunc(m_Type, Geom->GetShapeType());
		assert(func);
		return func(GetShapeObjPtr(), Geom->GetShapeObjPtr(), &m_WorldTransform, Geom->GetWorldTransform(), Direction, normal, t);
	}

	void 		GeometryBase::UpdateBoundingVolume()
	{
		m_BoxWorld = m_VolumeProperties.BoundingVolume.Transform(m_WorldTransform.transform.pos, m_WorldTransform.transform.quat);
	}

	Vector3		GeometryBase::GetSupport_WorldSpace(const Vector3& Direction) const
	{
		Vector3 DirLocal = m_WorldTransform.WorldToLocalDirection(Direction);
		Vector3 SupportLocal = CalculateSupport_LocalSpace(DirLocal);
		Vector3 SupportWorld = m_WorldTransform.LocalToWorld(SupportLocal);
		return SupportWorld;
	}

	void		GeometryBase::GetSupportFace_WorldSpace(const Vector3& Direction, SupportFace& Face) const
	{
		Vector3 DirLocal = m_WorldTransform.WorldToLocalDirection(Direction);
		CalculateSupportFace_LocalSpace(DirLocal, Face);
		for (int i = 0; i < Face.GetSize(); ++i)
		{
			Face[i] = m_WorldTransform.LocalToWorld(Face[i]);
		}
	}

	Matrix3		GeometryBase::GetInertiaTensor_LocalSpace() const
	{
		return m_VolumeProperties.InertiaMat;
	}

	void 		GeometryFactory::DeleteGeometry(GeometryBase* Geom)
	{
		delete Geom;
	}

	GeometryBase* GeometryFactory::CreateOBB_placement(void* pBuf, const Vector3& Center, const Vector3& HalfExtent, const Quaternion& Rot)
	{
		TGeometry<AxisAlignedBox3d>* p = pBuf ? new (pBuf) TGeometry<AxisAlignedBox3d>() : new TGeometry<AxisAlignedBox3d>();
		p->Min = -HalfExtent;
		p->Max = HalfExtent;
		p->UpdateVolumeProperties();
		p->SetWorldTransform(Center, Rot);

		return (GeometryBase*)p;
	}

	GeometryBase* GeometryFactory::CreateSphere_placement(void* pBuf, const Vector3& Center, float Radius)
	{
		TGeometry<Sphere3d>* p = pBuf ? new (pBuf)TGeometry<Sphere3d>() : new TGeometry<Sphere3d>();
		p->Center = Vector3::Zero();
		p->Radius = Radius;
		p->UpdateVolumeProperties();
		p->SetWorldTransform(Center, Quaternion::One());
		return (GeometryBase*)p;
	}

	GeometryBase* GeometryFactory::CreateCapsule_placement(void* pBuf, const Vector3& X0, const Vector3& X1, float Radius)
	{
		Quaternion quat = Quaternion::One();
		if ((X1 - X0).SquareLength() > 1e-6)
		{
			quat.FromTwoAxis(Vector3::UnitY(), X1 - X0);
		}
		Vector3 Center = (X0 + X1) * 0.5f;
		TGeometry<Capsule3d>* p = pBuf ? new (pBuf)TGeometry<Capsule3d>() : new TGeometry<Capsule3d>();
		p->Init(quat.Conjugate() * (X0 - Center), quat.Conjugate() * (X1 - Center), Radius);
		p->UpdateVolumeProperties();
		p->SetWorldTransform(Center, quat);
		return (GeometryBase*)p;
	}

	GeometryBase* GeometryFactory::CreateOBB(const Vector3& Center, const Vector3& HalfExtent, const Quaternion& Rot)
	{
		TGeometry<AxisAlignedBox3d>* p = new TGeometry<AxisAlignedBox3d>();
		return CreateOBB_placement(p, Center, HalfExtent, Rot);
	}

	GeometryBase* GeometryFactory::CreatePlane(const Vector3& Center, const Vector3& Normal)
	{
		TGeometry<Plane3d>* p = new TGeometry<Plane3d>();
		p->Normal = Vector3::UnitY();
		p->D = 0.0f;
		Quaternion quat;
		quat.FromTwoAxis(Vector3::UnitY(), Normal);
		p->UpdateVolumeProperties();
		p->SetWorldTransform(Center, quat);
		return (GeometryBase*)p;
	}

	GeometryBase* GeometryFactory::CreateSphere(const Vector3& Center, float Radius)
	{
		TGeometry<Sphere3d>* p = new TGeometry<Sphere3d>();
		return CreateSphere_placement(p, Center, Radius);
	}

	GeometryBase* GeometryFactory::CreateCylinder(const Vector3& X0, const Vector3& X1, float Radius)
	{
		Quaternion quat = Quaternion::One();
		if ((X1 - X0).SquareLength() > 1e-6)
		{
			quat.FromTwoAxis(Vector3::UnitY(), X1 - X0);
		}
		Vector3 Center = (X0 + X1) * 0.5f;
		TGeometry<Cylinder3d>* p = new TGeometry<Cylinder3d>();
		p->Init(quat.Conjugate() * (X0 - Center), quat.Conjugate() * (X1 - Center), Radius);
		p->UpdateVolumeProperties();
		p->SetWorldTransform(Center, quat);
		return (GeometryBase*)p;
	}

	GeometryBase* GeometryFactory::CreateCapsule(const Vector3& X0, const Vector3& X1, float Radius)
	{
		TGeometry<Capsule3d>* p = new TGeometry<Capsule3d>();
		return CreateCapsule_placement(p, X0, X1, Radius);
	}

	GeometryBase* GeometryFactory::CreateConvexMesh()
	{
		TGeometry<ConvexMesh>* p = new TGeometry<ConvexMesh>();
		p->UpdateVolumeProperties();
		p->SetWorldTransform(Vector3::Zero(), Quaternion::One());
		return (GeometryBase*)p;
	}

	GeometryBase* GeometryFactory::CreateHeightField(const Box3d& Bv, int nRows, int nCols)
	{
		TGeometry<HeightField3d>* p = new TGeometry<HeightField3d>();
		p->Init(Bv, nRows, nCols);
		p->UpdateVolumeProperties();
		p->SetWorldTransform(Vector3::Zero(), Quaternion::One());
		return (GeometryBase*)p;
	}

	GeometryBase* GeometryFactory::CreateTriangleMesh()
	{
		TGeometry<TriangleMesh>* p = new TGeometry<TriangleMesh>();
		p->UpdateVolumeProperties();
		p->SetWorldTransform(Vector3::Zero(), Quaternion::One());
		return (GeometryBase*)p;
	}
}