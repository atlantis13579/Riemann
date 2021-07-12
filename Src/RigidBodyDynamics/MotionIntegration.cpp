
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

		Vector3d Force = Rigid->Force + Vector3d(0, -0.0098f, 0);
		Rigid->Velocity = Force / Rigid->mass;
		Rigid->P += Rigid->Velocity * dt;
		Geom->SetPosition(Rigid->P);

		// Physically Based Modeling by David Baraff 
		// https://www.cs.cmu.edu/~baraff/sigcourse/
		// https://fgiesen.wordpress.com/2012/08/24/quaternion-differentiation/
		// https://www.ashwinnarayan.com/post/how-to-integrate-quaternions/
		// ----------------

		Quaternion quat = Geom->GetRotation();

		Matrix3d mat = quat.ToRotationMatrix();
		Matrix3d invInertia = mat * Rigid->invInertia * mat.Transpose();
		Vector3d omega = invInertia * Rigid->L;
		Quaternion delta(1.0f, omega.x * dt * 0.5f, omega.y * dt * 0.5f, omega.z * dt * 0.5f);
		Quaternion target = delta * quat;
		Geom->SetRotation(target);
	}
}
