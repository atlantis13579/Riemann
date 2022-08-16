
#include <assert.h>
#include "MotionIntegration.h"
#include "RigidBody.h"
#include "RigidBodySimulation.h"
#include "../Collision/GeometryObject.h"
#include "../Solver/NumericalODESolver.h"

// Physically Based Modeling by David Baraff
// https://www.cs.cmu.edu/~baraff/sigcourse/
// https://fgiesen.wordpress.com/2012/08/24/quaternion-differentiation/
// https://www.ashwinnarayan.com/post/how-to-integrate-quaternions/
// ----------------

static int Integrate_ExplicitEuler(std::vector<RigidBodyDynamic*> Bodies, float dt)
{
	int count = 0;
	for (size_t i = 0; i < Bodies.size(); ++i)
	{
		RigidBodyDynamic* Body = Bodies[i];
		assert(Body);
		if (Body->Sleeping)
		{
			continue;
		}

		Body->X = Body->X + (Body->V) * dt;									// X' = v
		Matrix3 R = Body->Q.ToRotationMatrix3();
		Matrix3 invInertiaWorld = R * Body->InvInertia * R.Transpose();
		Quaternion dQ = 0.5f * Quaternion(0.0f, Body->W) * Body->Q;			// Q' = 0.5 * W * Q
		Body->Q = (Body->Q + dQ * dt).Unit();
		
		float LinearDamping = 1.0f -(1.0f - Body->LinearDamping) * dt;
		float AngularDamping = 1.0f -(1.0f - Body->AngularDamping) * dt;
		Body->V = (Body->V + Body->InvMass * Body->ExtForce * dt) * LinearDamping;		// V' = Force / mass
		Body->W = (Body->W + invInertiaWorld * Body->ExtTorque * dt) * AngularDamping;	// W' = Torque / invInertia
		
		Body->mGeometry->SetCenterOfMass(Body->X);
		Body->mGeometry->SetRotation(Body->Q);
		Body->mGeometry->UpdateBoundingVolume();
		count++;
	}
	return count;
}

static int Integrate_SymplecticEuler(std::vector<RigidBodyDynamic*> Bodies, float dt)
{
	int count = 0;
	for (size_t i = 0; i < Bodies.size(); ++i)
	{
		RigidBodyDynamic* Body = Bodies[i];
		assert(Body);
		if (Body->Sleeping)
		{
			continue;
		}

		Matrix3 R = Body->Q.ToRotationMatrix3();
		Matrix3 invInertiaWorld = R * Body->InvInertia * R.Transpose();
		float LinearDamping = 1.0f - (1.0f - Body->LinearDamping) * dt;
		float AngularDamping = 1.0f - (1.0f - Body->AngularDamping) * dt;
		Body->V = (Body->V + Body->InvMass * Body->ExtForce * dt) * LinearDamping;		// V' = Force / mass
		Body->W = (Body->W + invInertiaWorld * Body->ExtTorque * dt) * AngularDamping;	// W' = Torque / invInertia
		
		Body->X = Body->X + (Body->V) * dt;										// X' = v
		Quaternion dQ = 0.5f * Quaternion(0.0f, Body->W) * Body->Q;				// Q' = 0.5 * W * Q
		Body->Q = (Body->Q + dQ * dt).Unit();
		
		Body->mGeometry->SetCenterOfMass(Body->X);
		Body->mGeometry->SetRotation(Body->Q);
		Body->mGeometry->UpdateBoundingVolume();
		count++;
	}
	return count;
}

// static
void MotionIntegration::Integrate(std::vector<RigidBodyDynamic*> Bodies, float dt, uint8_t method)
{
	IntegrateMethod m = (IntegrateMethod)method;
	switch (m)
	{
	case IntegrateMethod::ExplicitEuler:
		Integrate_ExplicitEuler(Bodies, dt);
		break;
	case IntegrateMethod::SymplecticEuler:
		Integrate_SymplecticEuler(Bodies, dt);
		break;
	default:
		assert(false);
		break;
	}
}
