
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
	Rigid->mass = param.mass;
	Rigid->invInertia = Geom->GetInverseInertia(Rigid->mass);
	Rigid->X = Geom->GetPosition();
	Rigid->Q = Geom->GetRotationQuat();
	Rigid->P = Vector3d::Zero();
	Rigid->L = Vector3d::Zero();
	Rigid->ExtForce = Vector3d::Zero();
	Rigid->ExtTorque = Vector3d::Zero();
	Rigid->Static = param.Static;
	Rigid->Sleep = Rigid->Static;
	return Rigid;
}
