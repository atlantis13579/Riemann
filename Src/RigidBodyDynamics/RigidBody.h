
#pragma once

#include <vector>
#include "../Maths/Vector3d.h"
#include "../Maths/Matrix3d.h"
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
	Matrix3d	Inertia;
	Vector3d	LinearVelocity;
	Vector3d	AngularVelocity;
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
	Vector3d	X;				// Position
	Quaternion	Q;				// Rotation Quaternion
	Vector3d	P;				// Linear Momentum
	Vector3d	L;				// Angular Momentum
	
	// Constant quantities
	float		InvMass;		// Inverse of Total Mass
	Matrix3d	InvInertia;		// Inverse of Inertia Tensor

private:
	PhysicsMaterial* Material;	// Not hold the memory

public:
	RigidBody();

	Vector3d 		GetLinearVelocity() const;
	Vector3d		GetAngularVelocity() const;
	const Matrix3d&	GetInverseInertia() const;
	Matrix3d		GetInverseInertia_WorldSpace() const;
	const float&	GetInverseMass() const;

	RigidBodyDynamic* CastDynamic();
	
	void			SetLinearVelocity(const Vector3d &v);
	void			SetAngularVelocity(const Vector3d &v);
	void			SetLinearVelocityThreadSafe(const Vector3d& v);
	void			SetAngularVelocityThreadSafe(const Vector3d& v);

	float			GetContactBeta() const;
	float			GetRestitution() const;
	float			GetFrictionDynamic() const;
	float			GetFrictionStatic() const;
};


class RigidBodyStatic : public RigidBody
{
public:
	void SetTransform(const Vector3d& pos, const Quaternion& quat);
	void SetPosition(const Vector3d& pos);
	void SetRotation(const Quaternion& quat);
	void AppendShapes(std::vector<Geometry*>* Shapes);

	static RigidBodyStatic* CreateRigidBody(Geometry* Shape);
};

class RigidBodyDynamic : public RigidBody
{
public:
	Vector3d	ExtForce;
	Vector3d	ExtTorque;

	float		LinearDamping;
	float		AngularDamping;
	float		MaxContactImpulse;
	float		SleepThreshold;
	float		FreezeThreshold;
	bool		DisableGravity;
	bool		Sleep;

	void		ApplyForce(const Vector3d& Force);
	void		ApplyTorgue(const Vector3d& Torque);
	void		ApplyTorgue(const Vector3d& RelativePosToCenterOfMass, const Vector3d& Force);
	void		AppendShapes(std::vector<Geometry*> *Shape);

	static RigidBodyDynamic* CreateRigidBody(Geometry* Shape, const RigidBodyParam &param);
};
