#pragma once

#include <stdint.h>
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

	enum GeometryFlagBits : uint32_t
	{
		GeometryFlag_Query = 1u << 0,
		GeometryFlag_Simulation = 1u << 1,
	};

	class Geometry
	{
		friend class GeometryFactory;
		friend class GeometryRegistration;
	public:
		Geometry();
		virtual ~Geometry();

		inline const Vector3& GetPosition() const
		{
			return m_Transform.pos;
		}

		inline const Quaternion& GetRotation() const
		{
			return m_Transform.quat;
		}

		inline const Transform* GetTransform() const
		{
			return &m_Transform;
		}

		inline Transform* GetTransform()
		{
			return &m_Transform;
		}

		inline void				SetPosition(const Vector3& Position)
		{
			m_Transform.pos = Position;
			UpdateBounds();
		}

		inline void				SetRotation(const Quaternion& Rotation)
		{
			m_Transform.quat = Rotation;
			UpdateBounds();
		}

		inline void				SetTransform(const Vector3& Position, const Quaternion& Rotation)
		{
			m_Transform.pos = Position;
			m_Transform.quat = Rotation;
			UpdateBounds();
		}

		template<class T>
		inline T* GetParent()
		{
			return static_cast<T*>(m_Parent);
		}

		template<class T>
		inline const T* GetParent() const
		{
			return static_cast<const T*>(m_Parent);
		}

		inline void				SetParent(void* parent)
		{
			m_Parent = parent;
		}

		inline MassParameters* GetMassParameters()
		{
			return &m_VolumeProperties;
		}

		inline const Box3& GetBounds() const
		{
			return m_Bounds;
		}

		inline const Box3& GetShapeBounds() const
		{
			return m_VolumeProperties.BoundingVolume;
		}

		inline void				SetShapeBounds(const Box3& box)
		{
			m_VolumeProperties.BoundingVolume = box;
			UpdateBounds();
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

		inline uint32_t			GetFlags() const { return m_Flags; }
		inline void				SetFlags(uint32_t Flags) { m_Flags = Flags; }
		inline bool				HasFlag(uint32_t Flag) const { return (m_Flags & Flag) != 0; }
		inline void				SetFlag(uint32_t Flag, bool Enabled)
		{
			if (Enabled)
			{
				m_Flags |= Flag;
			}
			else
			{
				m_Flags &= ~Flag;
			}
		}
		inline bool				IsQueryEnabled() const { return HasFlag(GeometryFlag_Query); }
		inline void				SetQueryEnabled(bool Enabled) { SetFlag(GeometryFlag_Query, Enabled); }
		inline bool				IsSimulationEnabled() const { return HasFlag(GeometryFlag_Simulation); }
		inline void				SetSimulationEnabled(bool Enabled) { SetFlag(GeometryFlag_Simulation, Enabled); }

		static Transform		CalculateCenterOfMassPoseMultibody(const std::vector<Geometry*>& geoms);

		virtual bool			RayCast(const Transform& ShapeTransform, const Vector3& Origin, const Vector3& Dir, const RayCastOption* Option, RayCastResult* Result) const = 0;
		virtual bool			RayCast(const Vector3& Origin, const Vector3& Dir, const RayCastOption* Option, RayCastResult* Result) const = 0;
		bool					Intersect(const Transform& ShapeTransform, const Geometry* Geom, const Transform& GeomTransform) const;
		bool					Intersect(const Geometry* Geom) const;
		bool					Penetration(const Geometry* Geom, Vector3* Normal, float* Depth) const;
		bool					SweepTestFast(const Transform& ShapeTransform, const Vector3& Direction, const Vector3& Bmin, const Vector3& Bmax, float* t) const;
		bool					SweepTestFast(const Vector3& Direction, const Vector3& Bmin, const Vector3& Bmax, float* t) const;
		bool					Sweep(const Transform& ShapeTransform, const Vector3& Direction, const Geometry* Geom, const Transform& GeomTransform, Vector3* position, Vector3* normal, float* t) const;
		bool					Sweep(const Vector3& Direction, const Geometry* Geom, Vector3* position, Vector3* normal, float* t) const;

		virtual void			UpdateVolumeProperties() = 0;
		void					UpdateBounds();

		Vector3					GetCenter() const;
		Vector3					GetSupport(const Vector3& Direction) const;
		void					GetSupportFace(const Vector3& Direction, SupportFace& Face) const;
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
		Box3				m_Bounds;
		Transform			m_Transform;
		CollisionData		m_FilterData;
		MassParameters		m_VolumeProperties;
		float				m_Density;
		void*				m_Parent;
		uint32_t			m_Flags;
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
