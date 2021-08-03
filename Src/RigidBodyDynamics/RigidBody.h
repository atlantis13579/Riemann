
#pragma once

#include <vector>
#include "../Maths/Vector3d.h"
#include "../Maths/Matrix3d.h"
#include "../Maths/Quaternion.h"
#include "PhysicsEntity.h"

class Geometry;

struct RigidBodyParam
{
	RigidBodyParam()
	{
		memset(this, 0, sizeof(RigidBodyParam));
		Mass = 1.0f;
		Static = true;

	}

	float		Mass;
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

class RigidBody : public PhysicsEntity
{
public:
	// Constant quantities
	float		Mass;			// Total Mass
	Matrix3d	InvInertia;		// Inverse of Inertia Tensor 
	
	// State variable
	Vector3d	X;				// Position
	Quaternion	Q;				// Rotation Quaternion
	Vector3d	P;				// Linear Momentum
	Vector3d	L;				// Angular Momentum

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

	static RigidBody* CreateRigidBody(Geometry* Shape, const RigidBodyParam &param);

	Geometry*	Shape;
};
