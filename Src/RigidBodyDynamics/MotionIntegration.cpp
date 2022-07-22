
#include <assert.h>
#include "MotionIntegration.h"
#include "RigidBody.h"
#include "../Collision/GeometryObject.h"
#include "../Solver/NumericalODESolver.h"

static void Integrate_ExplicitEuler(std::vector<RigidBodyDynamic*> Entities, float dt)
{
	for (size_t i = 0; i < Entities.size(); ++i)
	{
		RigidBodyDynamic* Rigid = Entities[i];
		if (Rigid == nullptr || Rigid->Static || Rigid->Sleep)
		{
			continue;
		}
		
		// Physically Based Modeling by David Baraff
		// https://www.cs.cmu.edu/~baraff/sigcourse/
		// https://fgiesen.wordpress.com/2012/08/24/quaternion-differentiation/
		// https://www.ashwinnarayan.com/post/how-to-integrate-quaternions/
		// ----------------
		Rigid->P = (Rigid->P + Rigid->ExtForce * dt) * Rigid->LinearDamping;		// P' = Force
		Rigid->X = Rigid->X + (Rigid->P * Rigid->InvMass) * dt;						// X' = v = P / m
		Rigid->L = (Rigid->L + Rigid->ExtTorque * dt) * Rigid->AngularDamping;		// L' = Torque
		Matrix3d R = Rigid->Q.ToRotationMatrix();
		Matrix3d invInertiaWorld = R * Rigid->InvInertia * R.Transpose();
		Vector3d AngularVelocity = invInertiaWorld * Rigid->L;
		Quaternion dQ = 0.5f * Quaternion(0.0f, AngularVelocity) * Rigid->Q;		// Q' = 0.5 * AngularVelocity * Q 
		Rigid->Q = (Rigid->Q + dQ * dt).Unit();
		
		Rigid->Shape->SetPosition(Rigid->X);
		Rigid->Shape->SetRotationQuat(Rigid->Q);
		Rigid->Shape->UpdateBoundingVolume();
		Rigid->ExtForce.SetZero();
		Rigid->ExtTorque.SetZero();
	}
}

static void Integrate_MidpointEuler(std::vector<RigidBodyDynamic*> Entities, float dt)
{
}

static void Integrate_SymplecticEuler(std::vector<RigidBodyDynamic*> Entities, float dt)
{
}

static void Integrate_ImplicitEuler(std::vector<RigidBodyDynamic*> Entities, float dt)
{
}

// static
void MotionIntegration::Integrate(std::vector<RigidBodyDynamic*> Entities, float dt, IntegrateMethod method)
{
	switch (method)
	{
	case IntegrateMethod::ExplicitEuler:
		Integrate_ExplicitEuler(Entities, dt);
		break;
	case IntegrateMethod::MidpointEuler:
		Integrate_MidpointEuler(Entities, dt);
		break;
	case IntegrateMethod::SymplecticEuler:
		Integrate_SymplecticEuler(Entities, dt);
		break;
	case IntegrateMethod::ImplicitEuler:
		Integrate_ImplicitEuler(Entities, dt);
		break;
	default:
		assert(false);
		break;
	}
}
