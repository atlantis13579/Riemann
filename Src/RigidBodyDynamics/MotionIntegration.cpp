
#include <assert.h>
#include "MotionIntegration.h"
#include "RigidBody.h"
#include "../Collision/GeometryObject.h"
#include "../Solver/NumericalODESolver.h"

// Physically Based Modeling by David Baraff
// https://www.cs.cmu.edu/~baraff/sigcourse/
// https://fgiesen.wordpress.com/2012/08/24/quaternion-differentiation/
// https://www.ashwinnarayan.com/post/how-to-integrate-quaternions/
// ----------------

static void Integrate_ExplicitEuler(std::vector<RigidBodyDynamic*> Bodies, float dt)
{
	for (size_t i = 0; i < Bodies.size(); ++i)
	{
		RigidBodyDynamic* Body = Bodies[i];
		assert(Body);
		if (Body->Sleep)
		{
			continue;
		}
		
		Body->X = Body->X + (Body->P * Body->InvMass) * dt;						// X' = v = P / m
		Matrix3 R = Body->Q.ToRotationMatrix();
		Matrix3 invInertiaWorld = R * Body->InvInertia * R.Transpose();
		Vector3 AngularVelocity = invInertiaWorld * Body->L;
		Quaternion dQ = 0.5f * Quaternion(0.0f, AngularVelocity) * Body->Q;		// Q' = 0.5 * AngularVelocity * Q
		Body->Q = (Body->Q + dQ * dt).Unit();
		
		Body->P = (Body->P + Body->ExtForce * dt) * Body->LinearDamping;		// P' = Force
		Body->L = (Body->L + Body->ExtTorque * dt) * Body->AngularDamping;		// L' = Torque
		
		Body->mGeometry->SetPosition(Body->X);
		Body->mGeometry->SetRotationQuat(Body->Q);
		Body->mGeometry->UpdateBoundingVolume();
		Body->ExtForce.SetZero();
		Body->ExtTorque.SetZero();
	}
}

static void Integrate_MidpointEuler(std::vector<RigidBodyDynamic*> Bodies, float dt)
{
}

static void Integrate_SymplecticEuler(std::vector<RigidBodyDynamic*> Bodies, float dt)
{
	for (size_t i = 0; i < Bodies.size(); ++i)
	{
		RigidBodyDynamic* Body = Bodies[i];
		assert(Body);
		if (Body->Sleep)
		{
			continue;
		}
		
		Body->P = (Body->P + Body->ExtForce * dt) * Body->LinearDamping;		// P' = Force
		Body->L = (Body->L + Body->ExtTorque * dt) * Body->AngularDamping;		// L' = Torque
		
		Body->X = Body->X + (Body->P * Body->InvMass) * dt;						// X' = v = P / m
		Matrix3 R = Body->Q.ToRotationMatrix();
		Matrix3 invInertiaWorld = R * Body->InvInertia * R.Transpose();
		Vector3 AngularVelocity = invInertiaWorld * Body->L;
		Quaternion dQ = 0.5f * Quaternion(0.0f, AngularVelocity) * Body->Q;		// Q' = 0.5 * AngularVelocity * Q
		Body->Q = (Body->Q + dQ * dt).Unit();
		
		Body->mGeometry->SetPosition(Body->X);
		Body->mGeometry->SetRotationQuat(Body->Q);
		Body->mGeometry->UpdateBoundingVolume();
		Body->ExtForce.SetZero();
		Body->ExtTorque.SetZero();
	}
}

static void Integrate_ImplicitEuler(std::vector<RigidBodyDynamic*> Bodies, float dt)
{
}

// static
void MotionIntegration::Integrate(std::vector<RigidBodyDynamic*> Bodies, float dt, IntegrateMethod method)
{
	switch (method)
	{
	case IntegrateMethod::ExplicitEuler:
		Integrate_ExplicitEuler(Bodies, dt);
		break;
	case IntegrateMethod::MidpointEuler:
		Integrate_MidpointEuler(Bodies, dt);
		break;
	case IntegrateMethod::SymplecticEuler:
		Integrate_SymplecticEuler(Bodies, dt);
		break;
	case IntegrateMethod::ImplicitEuler:
		Integrate_ImplicitEuler(Bodies, dt);
		break;
	default:
		assert(false);
		break;
	}
}
