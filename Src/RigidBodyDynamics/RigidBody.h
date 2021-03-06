
#pragma once

#include <stdint.h>
#include <vector>
#include "../Maths/Vector3.h"
#include "../Maths/Matrix3.h"
#include "../Maths/Quaternion.h"

class Geometry;
class RigidBodyStatic;
class RigidBodyDynamic;
struct PhysicsMaterial;

enum class RigidType : uint8_t
{
	Static,
	Dynamic,
	Kinematic,
};

enum class MorionType : uint8_t
{
	Discrete,
	Continuous,
};

struct RigidBodyParam
{
	RigidBodyParam()
	{
		memset(this, 0, sizeof(RigidBodyParam));
		pos = Vector3::Zero();
		quat = Quaternion::One();
		invMass = 1.0f;
		linearDamping = 0.999f;
		angularDamping = 0.999f;
		contactReportThreshold = 0.1f;
		maxContactImpulse = 10.0f;
		sleepThreshold = 0.01f;
		freezeThreshold = 0.01f;
		disableGravity = false;
		rigidType = RigidType::Static;
		motionType = MorionType::Discrete;
	}

	Vector3		pos;
	Quaternion	quat;
	float		invMass;
	Matrix3		inertia;
	Vector3		linearVelocity;
	Vector3		angularVelocity;
	float		linearDamping;
	float		angularDamping;
	float		contactReportThreshold;
	float		maxContactImpulse;
	float		sleepThreshold;
	float		freezeThreshold;
	bool		disableGravity;
	RigidType	rigidType;
	MorionType	motionType;
};

class RigidBody
{
public:
	Geometry*	mGeometry;
	RigidType	mRigidType;

	uint64_t	mGuid;

	// State variable
	Vector3		X;				// Position
	Quaternion	Q;				// Rotation Quaternion
	Vector3		P;				// Linear Momentum
	Vector3		L;				// Angular Momentum
	
	// Constant quantities
	float		InvMass;		// Inverse of Total Mass
	Matrix3		InvInertia;		// Inverse of Inertia Tensor
	
	int			mNodeId;		// nodeId from DynamicAABB Tree
	
private:
	PhysicsMaterial* mMaterial;	// Not hold the memory

public:
	RigidBody();
	virtual ~RigidBody();

	void				AddGeometry(Geometry* Geom);
	void				GetGeometries(std::vector<Geometry*>* Geometries);

	uint64_t			GetGuid() const
	{
		return mGuid;
	}

	void				SetGuid(uint64_t guid)
	{
		mGuid = guid;
	}

	Vector3 			GetLinearVelocity() const;
	const Vector3&		GetLinearMomentum() const;
	Vector3				GetAngularVelocity() const;
	const Vector3&		GetAngularMomentum() const;
	const Matrix3&		GetInverseInertia() const;
	Matrix3				GetInverseInertia_WorldSpace() const;
	const float&		GetInverseMass() const;

	float				GetKinematicsEnergy() const;
	float				GetLinearKinematicsEnergy() const;
	float				GetAngularKinematicsEnergy() const;

	RigidBodyStatic*	CastStatic();
	RigidBodyDynamic*	CastDynamic();
	
	void				SetLinearVelocity(const Vector3 &v);
	void				SetAngularVelocity(const Vector3 &w);

	void				AddLinearVelocity(const Vector3& dv);
	void				AddLinearMomentum(const Vector3& dp);
	void				AddAngularVelocity(const Vector3& dw);

	void				SetDefaultPhysicsMaterial(int idx);

	float				GetRestitution() const;
	float				GetFrictionDynamic() const;
	float				GetFrictionStatic() const;

	static RigidBody*	CreateRigidBody(const RigidBodyParam& param, Geometry* geom);
};

class RigidBodyStatic : public RigidBody
{
public:
	virtual ~RigidBodyStatic() {}

	static RigidBodyStatic* CreateRigidBody(const RigidBodyParam& param, Geometry* geom);

private:
	RigidBodyStatic(const RigidBodyParam& param, Geometry* geom);
};

class RigidBodyDynamic : public RigidBody
{
public:
	virtual ~RigidBodyDynamic() {}

	Vector3		ExtForce;
	Vector3		ExtTorque;

	MorionType	mMotionType;
	float		LinearDamping;
	float		AngularDamping;
	float		MaxContactImpulse;
	float		SleepThreshold;
	float		FreezeThreshold;
	bool		DisableGravity;
	bool		Sleeping;

	void		ApplyForce(const Vector3& Force);
	void		ApplyTorgue(const Vector3& Torque);
	void		ApplyTorgue(const Vector3& RelativePosToCenterOfMass, const Vector3& Force);
	
	bool		AutoSleep();
	void		Sleep();
	void		Wakeup();
	
	static RigidBodyDynamic* CreateRigidBody(const RigidBodyParam& param, Geometry* geom);

private:
	RigidBodyDynamic(const RigidBodyParam& param, Geometry* geom);
};

class RigidBodyKinematics : public RigidBody
{
public:
	RigidBodyKinematics();
	virtual	~RigidBodyKinematics() {}
	void	SetPosition(const Vector3& pos);
	void	SetRotation(const Quaternion& quat);
	void	SetTransform(const Vector3& pos, const Quaternion& quat);
};

void	GetAllGeometries(std::vector<RigidBody*> bodies, std::vector<Geometry*>* geometries);
