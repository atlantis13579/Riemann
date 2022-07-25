
#pragma once

#include <vector>
#include "../Maths/Vector3.h"
#include "../Maths/Matrix3.h"
#include "../Maths/Quaternion.h"

class Geometry;
class RigidBodyDynamic;
struct PhysicsMaterial;

enum class RigidType : uint8_t
{
	Static,
	Dynamic,
	Kinematic,
};

struct RigidBodyParam
{
	RigidBodyParam()
	{
		memset(this, 0, sizeof(RigidBodyParam));
		InvMass = 1.0f;
		LinearDamping = 0.999f;
		AngularDamping = 0.999f;
		ContactReportThreshold = 0.1f;
		MaxContactImpulse = 10.0f;
		SleepThreshold = 0.1f;
		FreezeThreshold = 0.1f;
		DisableGravity = false;
		Static = true;
	}

	float		InvMass;
	Matrix3	Inertia;
	Vector3	LinearVelocity;
	Vector3	AngularVelocity;
	float		LinearDamping;
	float		AngularDamping;
	float		ContactReportThreshold;
	float		MaxContactImpulse;
	float		SleepThreshold;
	float		FreezeThreshold;
	bool		DisableGravity;
	bool		Static;
};

class RigidBody
{
public:
	Geometry*	Shape;
	bool		Static;

	// State variable
	Vector3	X;				// Position
	Quaternion	Q;				// Rotation Quaternion
	Vector3	P;				// Linear Momentum
	Vector3	L;				// Angular Momentum
	
	// Constant quantities
	float		InvMass;		// Inverse of Total Mass
	Matrix3	InvInertia;		// Inverse of Inertia Tensor

private:
	PhysicsMaterial* Material;	// Not hold the memory

public:
	RigidBody();

	Vector3 		GetLinearVelocity() const;
	const Vector3& GetLinearMomentum() const;
	Vector3		GetAngularVelocity() const;
	const Vector3&	GetAngularMomentum() const;
	const Matrix3&	GetInverseInertia() const;
	Matrix3		GetInverseInertia_WorldSpace() const;
	const float&	GetInverseMass() const;

	float			GetKinematicsEnergy() const;
	float			GetLinearKinematicsEnergy() const;
	float			GetAngularKinematicsEnergy() const;

	RigidBodyDynamic* CastDynamic();
	
	void			SetLinearVelocity(const Vector3 &v);
	void			SetAngularVelocity(const Vector3 &w);

	void			AddLinearVelocity(const Vector3& dv);
	void			AddLinearMomentum(const Vector3& dp);
	void			AddAngularVelocity(const Vector3& dw);

	void			SetDefaultPhysicsMaterial(int idx);

	float			GetRestitution() const;
	float			GetFrictionDynamic() const;
	float			GetFrictionStatic() const;
};


class RigidBodyStatic : public RigidBody
{
public:
	void SetTransform(const Vector3& pos, const Quaternion& quat);
	void SetPosition(const Vector3& pos);
	void SetRotation(const Quaternion& quat);
	void AppendShapes(std::vector<Geometry*>* Shapes);

	static RigidBodyStatic* CreateRigidBody(Geometry* Shape);
};

class RigidBodyDynamic : public RigidBody
{
public:
	Vector3	ExtForce;
	Vector3	ExtTorque;

	float		LinearDamping;
	float		AngularDamping;
	float		MaxContactImpulse;
	float		SleepThreshold;
	float		FreezeThreshold;
	bool		DisableGravity;
	bool		Sleep;

	void		ApplyForce(const Vector3& Force);
	void		ApplyTorgue(const Vector3& Torque);
	void		ApplyTorgue(const Vector3& RelativePosToCenterOfMass, const Vector3& Force);
	void		AppendShapes(std::vector<Geometry*> *Shape);

	static RigidBodyDynamic* CreateRigidBody(Geometry* Shape, const RigidBodyParam &param);
};
