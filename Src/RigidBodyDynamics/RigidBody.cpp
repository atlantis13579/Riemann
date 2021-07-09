
#include "RigidBody.h"

RigidBody* RigidBody::CreateRigidBody(Geometry* Geom, const RigidBodyParam& param)
{
	// TODO
	RigidBody* Rigid = new RigidBody;
	Rigid->P = Geom->GetPositionWorld();
	Rigid->L = Vector3d(0.10f, 0.01f, 0.0f);
	Rigid->mass = param.mass;
	Rigid->Force = Vector3d(0.0f, -0.0098f, 0.0f);
	Rigid->invInertia = Geom->GetInverseInertia(Rigid->mass);
	Rigid->Static = param.Static;
	return Rigid;
}
