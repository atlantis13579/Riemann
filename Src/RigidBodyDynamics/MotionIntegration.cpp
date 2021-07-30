
#include "MotionIntegration.h"
#include "RigidBody.h"
#include "../Solver/NumericalODESolver.h"

// static
void MotionIntegration::Integrate(std::vector<Geometry*> Entities, float dt)
{
	for (size_t i = 0; i < Entities.size(); ++i)
	{
		Geometry* Geom = Entities[i];
		RigidBody* Rigid = (RigidBody*)Geom->GetEntity();

		if (Rigid == nullptr || Rigid->Static || Rigid->Sleep)
		{
			continue;
		}

		Rigid->P += Rigid->ExtForce * dt;				// P' = Force
		Rigid->X += (Rigid->P / Rigid->Mass) * dt;		// X' = v = P / m
		Geom->SetPosition(Rigid->X);

		// Physically Based Modeling by David Baraff 
		// https://www.cs.cmu.edu/~baraff/sigcourse/
		// https://fgiesen.wordpress.com/2012/08/24/quaternion-differentiation/
		// https://www.ashwinnarayan.com/post/how-to-integrate-quaternions/
		// ----------------

		Rigid->L += Rigid->ExtTorque * dt;					// L' = Torque
		Matrix3d R = Rigid->Q.ToRotationMatrix();
		Matrix3d invInertiaWorld = R * Rigid->InvInertia * R.Transpose();
		Vector3d AngularVelocity = invInertiaWorld * Rigid->L;
		Quaternion dQ = 0.5f * Quaternion(0.0f, AngularVelocity) * Rigid->Q;		// Q' = 0.5 * AngularVelocity * Q 
		Rigid->Q += dQ * dt;
		Geom->SetRotationQuat(Rigid->Q);

		Rigid->ExtForce.SetEmpty();
		Rigid->ExtTorque.SetEmpty();
	}
}
