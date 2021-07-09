
#include "RigidBody.h"

RigidBody* RigidBody::CreateRigidBody(Geometry* Geom, const RigidBodyParam& param)
{
	RigidBody* Rigid = new RigidBody;
	Rigid->mass = param.mass;
	Rigid->invInertia = Geom->GetInverseInertia(Rigid->mass);
	Rigid->P = Geom->GetPositionWorld();
	Rigid->Q = Geom->GetRotation();
	Rigid->P = Vector3d::Zero();
	Rigid->L = Vector3d::Zero();
	Rigid->Torque = Vector3d::Zero();
	Rigid->Force = Vector3d::Zero();
	Rigid->Static = param.Static;
	Rigid->Sleep = Rigid->Static;

	// TODO, Temp
	Rigid->L = Vector3d(0.10f, 0.01f, 0.0f);
	Rigid->Force = Vector3d(0.0f, -0.0098f, 0.0f);
	return Rigid;
}
