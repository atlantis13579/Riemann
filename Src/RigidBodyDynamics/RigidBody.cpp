
#include "RigidBody.h"

RigidBody* RigidBody::CreateRigidBody(Geometry* Geom, const RigidBodyParam& param)
{
	RigidBody* Rigid = new RigidBody;
	Rigid->mass = param.mass;
	Rigid->invInertia = Geom->GetInverseInertia(Rigid->mass);
	Rigid->X = Geom->GetPosition();
	Rigid->Q = Geom->GetRotationQuat();
	Rigid->P = Vector3d::Zero();
	Rigid->L = Vector3d::Zero();
	Rigid->Force = Vector3d::Zero();
	Rigid->Torque = Vector3d::Zero();
	Rigid->Static = param.Static;
	Rigid->Sleep = Rigid->Static;

	// TODO, Temp
	Rigid->Force = Vector3d(0, -9.8f, 0);
	Rigid->Torque = Rigid->Force.Cross(Vector3d::UnitZ()) * Geom->GetBoundingVolumeLocalSpace().GetSizeZ() * 10000.0f;
	return Rigid;
}
