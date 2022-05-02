
#include "MotionIntegration.h"
#include "RigidBody.h"
#include "../Solver/NumericalODESolver.h"

Vector3d		ProjectiveVelocityBlending(const Vector3d& P0, const Vector3d& P1, const Vector3d& V0, const Vector3d& V1, const Vector3d& A1, float t, float blend)
{
	Vector3d V = V0 + (V1 - V0) * blend;
	Vector3d _P0 = P0 + V0 * t + 0.5f * A1 * t * t;
	Vector3d _P1 = P1 + V * t + 0.5f * A1 * t * t;
	Vector3d Pos = _P0 + (_P1 - P0) * blend;
	return Pos;
}

// static
void MotionIntegration::Integrate(std::vector<RigidBodyDynamic*> Entities, float dt)
{
	for (size_t i = 0; i < Entities.size(); ++i)
	{
		RigidBodyDynamic* Rigid = (RigidBodyDynamic*)Entities[i];

		if (Rigid == nullptr || Rigid->Static || Rigid->Sleep)
		{
			continue;
		}

		Rigid->P += Rigid->ExtForce * dt;				// P' = Force
		Rigid->X += (Rigid->P / Rigid->Mass) * dt;		// X' = v = P / m
		Rigid->Shape->SetPosition(Rigid->X);

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
		Rigid->Shape->SetRotationQuat(Rigid->Q);

		Rigid->ExtForce.SetEmpty();
		Rigid->ExtTorque.SetEmpty();
	}
}
