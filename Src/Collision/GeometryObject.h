#pragma once

#include <vector>
#include "../Core/StaticArray.h"
#include "../Maths/Transform.h"
#include "../Maths/Box3.h"
#include "../CollisionPrimitive/PrimitiveType.h"

namespace Riemann
{
	class GeometryFactory;
	class GeometryRegistration;
	struct RayCastOption;
	struct RayCastResult;
	struct IntersectOption;
	struct IntersectResult;
	struct SweepOption;
	struct SweepResult;

	struct CollisionData
	{
		CollisionData()
		{
			v0 = 0;
			v1 = 0;
			v2 = 0;
			v3 = 0;
		}
		unsigned int v0;
		unsigned int v1;
		unsigned int v2;
		unsigned int v3;
	};

	typedef	StaticArray<Vector3, MAX_FACE_POINTS> SupportFace;

	class Geometry
	{
		friend class GeometryFactory;
		friend class GeometryRegistration;
	public:
		Geometry();
		virtual ~Geometry();

		inline const Vector3& GetWorldPosition() const
		{
			return m_WorldTransform.pos;
		}

		inline const Quaternion& GetWorldRotation() const
		{
			return m_WorldTransform.quat;
		}

		inline const Transform* GetWorldTransform() const
		{
			return &m_WorldTransform;
		}

		inline Transform* GetWorldTransform()
		{
			return &m_WorldTransform;
		}

		inline void				SetWorldPosition(const Vector3& Position)
		{
			m_WorldTransform.pos = Position;
			UpdateBoundingVolume();
		}

		inline void				SetWorldRotation(const Quaternion& Rotation)
		{
			m_WorldTransform.quat = Rotation;
			UpdateBoundingVolume();
		}

		inline void				SetWorldTransform(const Vector3& Position, const Quaternion& Rotation)
		{
			m_WorldTransform.pos = Position;
			m_WorldTransform.quat = Rotation;
			UpdateBoundingVolume();
		}

		inline void				SetLocalPosition(const Vector3& Position)
		{
			m_LocalTransform.pos = Position;
		}

		inline void				SetLocalRotation(const Quaternion& Rotation)
		{
			m_LocalTransform.quat = Rotation;
		}

		inline void				SetLocalTransform(const Vector3& Position, const Quaternion& Rotation)
		{
			m_LocalTransform.pos = Position;
			m_LocalTransform.quat = Rotation;
		}

		inline const Transform* GetLocalTransform() const
		{
			return &m_LocalTransform;
		}

		template<class T>
		inline T* GetParent()
		{
			return static_cast<T*>(m_Parent);
		}

		inline void				SetParent(void* parent)
		{
			m_Parent = parent;
		}

		inline MassParameters* GetMassParameters()
		{
			return &m_VolumeProperties;
		}

		inline const Box3& GetBoundingVolume_WorldSpace() const
		{
			return m_BoxWorld;
		}

		inline const Box3& GetBoundingVolume_LocalSpace() const
		{
			return m_VolumeProperties.BoundingVolume;
		}

		inline void				SetBoundingVolume_LocalSpace(const Box3& box)
		{
			m_VolumeProperties.BoundingVolume = box;
		}

		inline PrimitiveType		GetShapeType() const
		{
			return m_Type;
		}

		inline const CollisionData& GetFilterData() const
		{
			return m_FilterData;
		}

		template<class GEOM_TYPE>
		inline GEOM_TYPE* GetShapeObj()
		{
			if (m_Type == GEOM_TYPE::StaticType())
			{
				return static_cast<GEOM_TYPE*>(GetShapeObjPtr());
			}
			return nullptr;
		}

		template<class GEOM_TYPE>
		inline const GEOM_TYPE* GetShapeObj() const
		{
			if (m_Type == GEOM_TYPE::StaticType())
			{
				return static_cast<const GEOM_TYPE*>(GetShapeObjPtr());
			}
			return nullptr;
		}

		inline void				SetNodeId(int NodeId) { m_NodeId = NodeId; }
		inline int				GetNodeId() const { return m_NodeId; }

		static Transform		CalculateCenterOfMassPoseMultibody(const std::vector<Geometry*>& geoms);

		virtual bool			RayCast(const Vector3& Origin, const Vector3& Dir, const RayCastOption* Option, RayCastResult* Result) const = 0;
		bool					Intersect(const Geometry* Geom) const;
		bool					Penetration(const Geometry* Geom, Vector3* Normal, float* Depth) const;
		bool					SweepTestFast(const Vector3& Direction, const Vector3& Bmin, const Vector3& Bmax, float* t) const;
		bool					Sweep(const Vector3& Direction, const Geometry* Geom, Vector3* position, Vector3* normal, float* t) const;

		virtual void			UpdateVolumeProperties() = 0;
		void					UpdateBoundingVolume();

		Vector3					GetCenter_WorldSpace() const;
		Vector3					GetSupport_WorldSpace(const Vector3& Direction) const;
		void					GetSupportFace_WorldSpace(const Vector3& Direction, SupportFace& Face) const;
		Matrix3					GetInertiaTensor_LocalSpace() const;

	private:
		virtual Vector3			CalculateSupport_LocalSpace(const Vector3& Direction) const = 0;
		virtual void			CalculateSupportFace_LocalSpace(const Vector3& Direction, SupportFace& Face) const = 0;
		virtual Vector3			GetCenter_LocalSpace() const = 0;

		const void* GetShapeObjPtr() const
		{
			const unsigned char* p = reinterpret_cast<const unsigned char*>(this);
			return static_cast<const void*>(p + sizeof(Geometry));
		}

		void* GetShapeObjPtr()
		{
			unsigned char* p = reinterpret_cast<unsigned char*>(this);
			return static_cast<void*>(p + sizeof(Geometry));
		}

	protected:
		PrimitiveType		m_Type;
		Box3				m_BoxWorld;
		Transform	m_WorldTransform;
		Transform			m_LocalTransform;
		CollisionData		m_FilterData;
		MassParameters		m_VolumeProperties;
		float				m_Density;
		void*				m_Parent;
		int					m_NodeId;		// nodeId from DynamicAABB Tree
	};

	class GeometryFactory
	{
	public:
		static void		 DeleteGeometry(Geometry* Geom);

		static Geometry* CreateOBB(const Vector3& Center, const Vector3& HalfExtent, const Quaternion& Rot = Quaternion::One());
		static Geometry* CreatePlane(const Vector3& Center, const Vector3& Normal);
		static Geometry* CreateSphere(const Vector3& Center, float Radius);
		static Geometry* CreateCylinder(const Vector3& x0, const Vector3& x1, float Radius);
		static Geometry* CreateCapsule(const Vector3& x0, const Vector3& x1, float Radius);
		static Geometry* CreateHeightField(const Box3& Bv, int nRows, int nCols);
		static Geometry* CreateConvexMesh();
		static Geometry* CreateTriangleMesh();

		static Geometry* CreateOBB_placement(void* pBuf, const Vector3& Center, const Vector3& HalfExtent, const Quaternion& Rot = Quaternion::One());
		static Geometry* CreateSphere_placement(void* pBuf, const Vector3& Center, float Radius);
		static Geometry* CreateCapsule_placement(void* pBuf, const Vector3& x0, const Vector3& x1, float Radius);

		static int		ObjectCount[(int)PrimitiveType::TYPE_COUNT];
	};
}
