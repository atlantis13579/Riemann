
#include "RigidBody.h"

void		RigidBody::ApplyForce(const Vector3d& _Force)
{
	this->ExtForce += _Force;
}

void		RigidBody::ApplyTorgue(const Vector3d& _Torque)
{
	this->ExtTorque += _Torque;
}

RigidBody*	RigidBody::CreateRigidBody(Geometry* Geom, const RigidBodyParam& param)
{
	RigidBody* Rigid = new RigidBody;
	Rigid->Mass = param.Mass;
	Rigid->InvInertia = Geom->GetInverseInertia(Rigid->Mass);
	Rigid->X = Geom->GetPosition();
	Rigid->Q = Geom->GetRotationQuat();
	Rigid->P = param.LinearVelocity * Rigid->Mass;
	Rigid->L = Rigid->InvInertia.Inverse() * param.AngularVelocity;
	Rigid->ExtForce = Vector3d::Zero();
	Rigid->ExtTorque = Vector3d::Zero();
	Rigid->LinearDamping = param.LinearDamping;
	Rigid->AngularDamping = param.AngularDamping;
	Rigid->MaxContactImpulse = param.MaxContactImpulse;
	Rigid->SleepThreshold = param.SleepThreshold;
	Rigid->FreezeThreshold = param.FreezeThreshold;
	Rigid->DisableGravity = param.DisableGravity;
	Rigid->Static = param.Static;
	Rigid->Sleep = Rigid->Static;
	return Rigid;
}
