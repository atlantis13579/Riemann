
#include "RigidBody.h"

void				RigidBodyDynamic::ApplyForce(const Vector3d& _Force)
{
	this->ExtForce += _Force;
}

void				RigidBodyDynamic::ApplyTorgue(const Vector3d& _Torque)
{
	this->ExtTorque += _Torque;
}

void				RigidBodyDynamic::AppendShapes(std::vector<Geometry*> *Shapes)
{
	Shapes->push_back(this->Shape);
}

RigidBodyDynamic*	RigidBodyDynamic::CreateRigidBody(Geometry* Shape, const RigidBodyParam& param)
{
	RigidBodyDynamic* Rigid = new RigidBodyDynamic;
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

RigidBodyStatic* RigidBodyStatic::CreateRigidBody(Geometry* Shape, const RigidBodyParam& param)
{
	RigidBodyStatic* Rigid = new RigidBodyStatic;
	Rigid->X = Shape->GetPosition();
	Rigid->Q = Shape->GetRotationQuat();
	Rigid->Shape = Shape;
	return Rigid;
}

void RigidBodyStatic::SetTransform(const Vector3d& pos, const Quaternion& quat)
{
	X = pos;
	Q = quat;
	Shape->SetPosition(pos);
	Shape->SetRotationQuat(quat);
}

void RigidBodyStatic::SetPosition(const Vector3d& pos)
{
	X = pos;
	Shape->SetPosition(pos);
}

void RigidBodyStatic::SetRotation(const Quaternion& quat)
{
	Q = quat;
	Shape->SetRotationQuat(quat);
}
