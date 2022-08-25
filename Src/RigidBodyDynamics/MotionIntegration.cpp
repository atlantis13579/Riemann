
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

static void UpdateBody(RigidBodyDynamic* Body)
{
	Geometry* g = Body->mGeometry;
	while (g)
	{
		g->SetCenterOfMass(Body->X);
		g->SetRotation(Body->Q);
		g->UpdateBoundingVolume();
		g = g->GetNext();
	}
}

static int Integrate_ExplicitEuler(const std::vector<RigidBodyDynamic*>& Bodies, float dt)
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
		Quaternion dQ = 0.5f * Quaternion(0.0f, Body->W) * Body->Q;			// Q' = 0.5 * W * Q
		Body->Q = (Body->Q + dQ * dt).Unit();
		
		Matrix3 R = Body->Q.ToRotationMatrix3();
		Matrix3 invInertiaWorld = R * Body->InvInertia * R.Transpose();
		float LinearDamping = 1.0f -(1.0f - Body->LinearDamping) * dt;
		float AngularDamping = 1.0f -(1.0f - Body->AngularDamping) * dt;
		Body->V = (Body->V + Body->InvMass * Body->ExtForce * dt) * LinearDamping;		// V' = Force / mass
		Body->W = (Body->W + invInertiaWorld * Body->ExtTorque * dt) * AngularDamping;	// W' = Torque / invInertia
		
		UpdateBody(Body);
		count++;
	}
	return count;
}

static int Integrate_SymplecticEuler(const std::vector<RigidBodyDynamic*>& Bodies, float dt)
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
		
		UpdateBody(Body);
		count++;
	}
	return count;
}

// static
void MotionIntegration::Integrate(std::vector<RigidBodyDynamic*> Bodies, float dt, uint8_t flag)
{
	IntegrateMethod method = (IntegrateMethod)(flag & 0x01);
	switch (method)
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
