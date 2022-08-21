#pragma once

#include "../Core/StaticArray.h"
#include "../Maths/Vector3.h"
#include "../Maths/Quaternion.h"
#include "../Maths/Box3d.h"
#include "../CollisionPrimitive/ShapeType.h"

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
	}
	unsigned int v0;
};

typedef	StaticArray<Vector3, MAX_FACE_POINTS> SupportFace;

struct GeometryTransform
{
	Vector3		Translation;
	Quaternion	Rotation;

	inline Vector3	LocalToWorld(const Vector3& Point) const
	{
		return Rotation * Point + Translation;
	}

	inline Vector3	LocalToWorldDirection(const Vector3& Direction) const
	{
		return Rotation * Direction;
	}

	inline Vector3	WorldToLocal(const Vector3& Point) const
	{
		return Rotation.Conjugate() * (Point - Translation);
	}

	inline Vector3	WorldToLocalDirection(const Vector3& Direction) const
	{
		return Rotation.Conjugate() * Direction;
	}
};

// Transformation between object 1's local space and object 2's local space
struct Geometry2Transform
{
	Vector3		Translation1;
	Vector3		Translation2;
	Quaternion	Rotation1;
	Quaternion	Rotation2;
	
	Geometry2Transform(const GeometryTransform* t1, const GeometryTransform* t2)
	{
		Rotation1 = t1->Rotation;
		Rotation2 = t2->Rotation;
		Translation1 = t1->Translation;
		Translation2 = t2->Translation;
	}

	Vector3		Local1ToLocal2(const Vector3& Point) const
	{
		return Rotation2.Conjugate() * (Rotation1 * Point + Translation1 - Translation2);
	}

	Vector3		Local1ToLocal2Direction(const Vector3& Direction) const
	{
		Quaternion quat = Rotation1 * Rotation2.Conjugate();
		return quat * Direction;
	}

	Matrix3		Local1ToLocal2RotationMatrix() const
	{
		Quaternion quat = Rotation1 * Rotation2.Conjugate();
		return quat.ToRotationMatrix3();
	}

	Vector3		Local2ToLocal1(const Vector3& Point) const
	{
		return Rotation1.Conjugate() * (Rotation2 * Point + Translation2 - Translation1);
	}

	Vector3		Local2ToLocal1Direction(const Vector3& Direction) const
	{
		Quaternion quat = Rotation2 * Rotation1.Conjugate();
		return quat * Direction;
	}

	Matrix3		Local2ToLocal1RotationMatrix() const
	{
		Quaternion quat = Rotation2 * Rotation1.Conjugate();
		return quat.ToRotationMatrix3();
	}

	Vector3		Local1ToWorld(const Vector3& Point) const
	{
		return Rotation1 * Point + Translation1;
	}

	Vector3		Local1ToWorldDirection(const Vector3& Direction) const
	{
		return Rotation1 * Direction;
	}

	Matrix3		Local1ToWorldRotationMatrix() const
	{
		return Rotation1.ToRotationMatrix3();
	}

	Vector3		Local2ToWorld(const Vector3& Point) const
	{
		return Rotation2 * Point + Translation2;
	}

	Vector3		Local2ToWorldDirection(const Vector3& Direction) const
	{
		return Rotation2 * Direction;
	}

	Matrix3		Local2ToWorldRotationMatrix() const
	{
		return Rotation2.ToRotationMatrix3();
	}
};

class Geometry
{
	friend class GeometryFactory;
	friend class GeometryRegistration;
public:
	Geometry();
	virtual ~Geometry() {}

	inline const Vector3&	GetCenterOfMass() const
	{
		return m_CenterOfMassTransform.Translation;
	}

	inline void				SetCenterOfMass(const Vector3& Position)
	{
		m_CenterOfMassTransform.Translation = Position;
	}

	inline const Quaternion& GetRotation() const
	{
		return m_CenterOfMassTransform.Rotation;
	}

	inline void				SetRotation(const Quaternion& Rotation)
	{
		m_CenterOfMassTransform.Rotation = Rotation;
	}

	template<class T>
	inline T*				GetParent()
	{
		return static_cast<T*>(m_Parent);
	}

	inline void				SetParent(void* parent)
	{
		m_Parent = parent;
	}

	inline Geometry*		GetNext()
	{
		return m_Next;
	}
	
	inline const Geometry*		GetNext() const
	{
		return m_Next;
	}

	inline void				LinkNext(Geometry* next)
	{
		m_Next = next;
	}

	inline const GeometryTransform* GetCenterOfMassTransform() const
	{
		return &m_CenterOfMassTransform;
	}

	inline GeometryTransform* GetCenterOfMassTransform()
	{
		return &m_CenterOfMassTransform;
	}

	inline ShapeType3d		GetShapeType() const
	{
		return m_Type;
	}
	
	inline const CollisionData&	GetFilterData() const
	{
		return m_FilterData;
	}

	template<class GEOM_TYPE>
	inline GEOM_TYPE*			GetShapeObj()
	{
		if (m_Type == GEOM_TYPE::StaticType())
		{
			return static_cast<GEOM_TYPE*>(GetShapeObjPtr());
		}
		return nullptr;
	}

	template<class GEOM_TYPE>
	inline const GEOM_TYPE*		GetShapeObj() const
	{
		if (m_Type == GEOM_TYPE::StaticType())
		{
			return static_cast<const GEOM_TYPE*>(GetShapeObjPtr());
		}
		return nullptr;
	}
	
	static Vector3			GetCenterOfMassMultibody(const Geometry* Geom);
	static Quaternion		GetRotationMultibody(const Geometry* Geom);
	static Matrix3			GetInverseInertiaMultibody(const Geometry* Geom, float InvMass);

	virtual bool			RayCast(const Vector3& Origin, const Vector3 &Dir, const RayCastOption* Option, RayCastResult *Result) const = 0;
	bool					Intersect(const Geometry* Geom) const;
	bool					Penetration(const Geometry* Geom, Vector3 *Normal, float* Depth) const;
	bool					SweepAABB(const Vector3& Direction, const Vector3 &Bmin, const Vector3& Bmax, Vector3 *Normal, float* t) const;
	bool					Sweep(const Vector3& Direction, const Geometry* Geom, Vector3 *normal, float* t) const;
	
	void					UpdateBoundingVolume();

	inline const Box3d&		GetBoundingVolume_WorldSpace() const
	{
		return m_BoxWorld;
	}

	Vector3					GetSupport_WorldSpace(const Vector3& Direction) const;
	void					GetSupportFace_WorldSpace(const Vector3& Direction, SupportFace& Face) const;
	Matrix3					GetInverseInertia_LocalSpace(float InvMass) const;

private:
	virtual Matrix3			GetInertia_LocalSpace(float InvMass) const = 0;
	virtual Vector3			GetSupport_LocalSpace(const Vector3& Direction) const = 0;
	virtual void			GetSupportFace_LocalSpace(const Vector3& Direction, SupportFace& Face) const = 0;
	virtual Box3d			GetBoundingVolume_LocalSpace() const = 0;
	virtual float			GetVolume() const = 0;

	const void*				GetShapeObjPtr() const
	{
		const unsigned char* p = reinterpret_cast<const unsigned char*>(this);
		return static_cast<const void*>(p + sizeof(Geometry));
	}

	void*					GetShapeObjPtr()
	{
		unsigned char* p = reinterpret_cast<unsigned char*>(this);
		return static_cast<void*>(p + sizeof(Geometry));
	}

protected:
	ShapeType3d				m_Type;
	Box3d					m_BoxWorld;
	GeometryTransform		m_CenterOfMassTransform;
	CollisionData			m_FilterData;
	void*					m_Parent;
	Geometry*				m_Next;
};

class GeometryIterator
{
public:
	virtual ~GeometryIterator() {}
	virtual Geometry* GetNext() = 0;
	virtual int GetSize() const = 0;
};

class GeometryFactory
{
public:
	static void		 DeleteGeometry(Geometry *Geom);

	static Geometry* CreateOBB(const Vector3& Center, const Vector3 & HalfExtent, const Quaternion& Rot = Quaternion::One());
	static Geometry* CreatePlane(const Vector3& Center, const Vector3& Normal);
	static Geometry* CreateSphere(const Vector3& Center, float Radius);
	static Geometry* CreateCylinder(const Vector3& X0, const Vector3& X1, float Radius);
	static Geometry* CreateCapsule(const Vector3& X0, const Vector3& X1, float Radius);
	static Geometry* CreateHeightField(const Box3d& Bv, int nRows, int nCols);
	static Geometry* CreateConvexMesh();
	static Geometry* CreateTriangle(const Vector3& A, const Vector3& B, const Vector3& C);
	static Geometry* CreateTriangleMesh();

	static Geometry* CreateOBB_placement(void* pBuf, const Vector3& Center, const Vector3& HalfExtent, const Quaternion& Rot = Quaternion::One());
	static Geometry* CreateSphere_placement(void* pBuf, const Vector3& Center, float Radius);
	static Geometry* CreateCapsule_placement(void* pBuf, const Vector3& X0, const Vector3& X1, float Radius);

	static int		ObjectCount[(int)ShapeType3d::TYPE_COUNT];
};
