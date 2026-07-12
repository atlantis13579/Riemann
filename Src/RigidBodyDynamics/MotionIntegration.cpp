
#include <assert.h>
#include "MotionIntegration.h"
#include "RigidBody.h"
#include "PhysicsWorld.h"
#include "../Collision/GeometryObject.h"
#include "../Solver/NumericalODESolver.h"

// Physically Based Modeling by David Baraff
// https://www.cs.cmu.edu/~baraff/sigcourse/
// https://fgiesen.wordpress.com/2012/08/24/quaternion-differentiation/
// https://www.ashwinnarayan.com/post/how-to-integrate-quaternions/
// ----------------

namespace Riemann
{
	static Quaternion IntegrateRotation(const Quaternion& Rotation, const Vector3& AngularVelocity, float dt, bool Exact)
	{
		const float angularSpeed = AngularVelocity.Length();
		if (angularSpeed <= 1.0e-8f || dt <= 0.0f)
		{
			return Rotation;
		}
		if (!Exact)
		{
			const Quaternion derivative = 0.5f * Quaternion(0.0f, AngularVelocity) * Rotation;
			return (Rotation + derivative * dt).Unit();
		}
		Quaternion deltaRotation;
		deltaRotation.FromRotationAxis(AngularVelocity / angularSpeed, angularSpeed * dt);
		return (deltaRotation * Rotation).Unit();
	}

	static int IntegrateBodyVelocities(const std::vector<RigidBodyDynamic*>& Bodies, float Dt)
	{
		int count = 0;
		for (size_t i = 0; i < Bodies.size(); ++i)
		{
			RigidBodyDynamic* Body = Bodies[i];
			assert(Body);
			if (!Body->Sleeping)
			{
				Matrix3 R = Body->Q.ToRotationMatrix3();
				Matrix3 invInertiaWorld = R * Body->InvInertia * R.Transpose();
				float LinearDamping = 1.0f - (1.0f - Body->LinearDamping) * Dt;
				float AngularDamping = 1.0f - (1.0f - Body->AngularDamping) * Dt;
				Body->V = (Body->V + Body->InvMass * Body->ExtForce * Dt) * LinearDamping;
				Body->W = (Body->W + invInertiaWorld * Body->ExtTorque * Dt) * AngularDamping;
				count++;
			}

			// Consume only the forces gathered before the velocity phase. Forces applied
			// by callbacks later in the step remain queued for the next frame.
			Body->ExtForce.SetZero();
			Body->ExtTorque.SetZero();
		}
		return count;
	}

	static int IntegrateBodyPoses(const std::vector<RigidBodyDynamic*>& Bodies, float Dt)
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

			const Vector3 motionVelocity = Body->HadCCDContact ? Body->CCDPreSolveVelocity : Body->V;
			const Vector3 motionAngularVelocity = Body->HadCCDContact ? Body->CCDPreSolveAngularVelocity : Body->W;
			Body->X = Body->X + motionVelocity * (Dt * Body->CCDMotionFraction);
			Body->Q = IntegrateRotation(Body->Q, motionAngularVelocity, Dt * Body->CCDMotionFraction,
				Body->mMotionType == MorionType::Continuous);

			Body->UpdateGeometries();
			count++;
		}
		return count;
	}

	// static
	void MotionIntegration::Integrate(std::vector<RigidBodyDynamic*> Bodies, float Dt, uint8_t Method)
	{
		IntegrateVelocities(Bodies, Dt);
		IntegratePoses(Bodies, Dt, Method);
	}

	// static
	void MotionIntegration::IntegrateVelocities(const std::vector<RigidBodyDynamic*>& Bodies, float Dt)
	{
		IntegrateBodyVelocities(Bodies, Dt);
	}

	// static
	void MotionIntegration::IntegratePoses(const std::vector<RigidBodyDynamic*>& Bodies, float Dt, uint8_t Method)
	{
		const IntegrateMethod IntegrationMethod = (IntegrateMethod)(Method & 0x01);
		switch (IntegrationMethod)
		{
		case IntegrateMethod::ExplicitEuler:
		case IntegrateMethod::SymplecticEuler:
			// Forces are integrated before constraint solving for both modes. The pose
			// phase must consume the resulting constraint-corrected velocity so CCD and
			// discrete contacts describe the same motion.
			IntegrateBodyPoses(Bodies, Dt);
			break;
		default:
			assert(false);
			break;
		}
	}
}
