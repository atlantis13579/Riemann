
#pragma once

#include "../Maths/Vector3d.h"
#include "../Maths/Matrix3d.h"
#include "../Maths/Quaternion.h"
#include "PhysicsEntity.h"

class RigidBody : public PhysicsEntity
{
	// Constant quantities
	float		mass;			// Total Mass
	Matrix3d	invInertia;		// Inverse of Inertia Tensor 
	
	// State variable
	Vector3d	X;				// Position
	Quaternion	Q;				// Rotation Quaternion
	Vector3d	P;				// Linear Momentum
	Vector3d	L;				// Angular Momentum

	Vector3d	Force;
	Vector3d	Torque;
};
