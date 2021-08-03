
#include "RigidBody.h"

void		RigidBody::ApplyForce(const Vector3d& _Force)
{
	this->ExtForce += _Force;
}

void		RigidBody::ApplyTorgue(const Vector3d& _Torque)
{
	this->ExtTorque += _Torque;
}

void RigidBody::AppendShapes(std::vector<Geometry*> *Shapes)
{
	Shapes->push_back(this->Shape);
}

RigidBody*	RigidBody::CreateRigidBody(Geometry* Shape, const RigidBodyParam& param)
{
	RigidBody* Rigid = new RigidBody;
	Rigid->Mass = param.Mass;
	Rigid->InvInertia = Shape->GetInverseInertia_WorldSpace(Rigid->Mass);
	Rigid->X = Shape->GetPosition();
	Rigid->Q = Shape->GetRotationQuat();
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
	Rigid->Shape = Shape;
	return Rigid;
}
