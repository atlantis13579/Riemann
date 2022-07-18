
#pragma once

#include <vector>
#include "../Maths/Vector3d.h"
#include "../Maths/Matrix3d.h"
#include "../Maths/Quaternion.h"

class Geometry;

struct RigidBodyParam
{
	RigidBodyParam()
	{
		memset(this, 0, sizeof(RigidBodyParam));
		InvMass = 1.0f;
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
	
public:
	Vector3d 		GetLinearVelocity() const;
	Vector3d		GetAngularVelocity() const;
	const Matrix3d&	GetInverseInertia() const;
	Matrix3d		GetInverseInertia_WorldSpace() const;
	const float&	GetInverseMass() const;
	
	void			SetLinearVelocity(const Vector3d &v);
	void			SetAngularVelocity(const Vector3d &v);
	
	float			GetContactBeta() const { return 0.4f; }	// TODO
	float			GetRestitution() const { return 0.0f; }	// TODO
	float			GetFriction() const { return 1.0f; }	// TODO
};


class RigidBodyStatic : public RigidBody
{
public:
	static RigidBodyStatic* CreateRigidBody(Geometry* Shape, const RigidBodyParam& param);

	void SetTransform(const Vector3d& pos, const Quaternion& quat);
	void SetPosition(const Vector3d& pos);
	void SetRotation(const Quaternion& quat);
	void AppendShapes(std::vector<Geometry*>* Shapes);
};

class RigidBodyDynamic : public RigidBody
{
public:
	// ----
	Vector3d	ExtForce;
	Vector3d	ExtTorque;

	// ----
	float		LinearDamping;
	float		AngularDamping;
	float		MaxContactImpulse;
	float		SleepThreshold;
	float		FreezeThreshold;
	bool		DisableGravity;
	bool		Sleep;
	bool		Static;

	void		ApplyForce(const Vector3d& _Force);
	void		ApplyTorgue(const Vector3d& _Torque);

	void		AppendShapes(std::vector<Geometry*> *Shape);

	static RigidBodyDynamic* CreateRigidBody(Geometry* Shape, const RigidBodyParam &param);
};
