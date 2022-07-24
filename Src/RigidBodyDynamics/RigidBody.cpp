
#include "RigidBody.h"
#include "PhysicsMaterial.h"
#include "../Collision/GeometryObject.h"

static const float kMinimumInvMass = 1e-6f;

RigidBody::RigidBody()
{
	Shape = nullptr;
	Material = nullptr;
}

Vector3d 			RigidBody::GetLinearVelocity() const
{
	return P * InvMass;
}

const Vector3d&		RigidBody::GetLinearMomentum() const
{
	return P;
}

Vector3d			RigidBody::GetAngularVelocity() const
{
	return InvInertia * L;
}

const Vector3d&		RigidBody::GetAngularMomentum() const
{
	return L;
}

const Matrix3d&	 	RigidBody::GetInverseInertia() const
{
	return InvInertia;
}

Matrix3d			RigidBody::GetInverseInertia_WorldSpace() const
{
	Matrix3d R = Q.ToRotationMatrix();
	Matrix3d invInertiaWorld = R * InvInertia * R.Transpose();
	return invInertiaWorld;
}

const float&		RigidBody::GetInverseMass() const
{
	return InvMass;
}

float RigidBody::GetKinematicsEnergy() const
{
	float l = GetLinearKinematicsEnergy();
	float a = GetAngularKinematicsEnergy();
	return l + a;
}

float RigidBody::GetLinearKinematicsEnergy() const
{
	return 0.5f * P.SquareLength() * InvMass;
}

float RigidBody::GetAngularKinematicsEnergy() const
{
	return 0.5f * L.Dot(InvInertia * L);
}

RigidBodyDynamic* RigidBody::CastDynamic()
{
	return Static ? nullptr : (RigidBodyDynamic*)this;
}

void				RigidBody::SetLinearVelocity(const Vector3d &v)
{
	P = InvMass > kMinimumInvMass ? v / InvMass : Vector3d::Zero();
}

void				RigidBody::SetAngularVelocity(const Vector3d &w)
{
	L = InvInertia.Invertible() ? InvInertia.Inverse() * w : Vector3d::Zero();
}

void				RigidBody::AddLinearVelocity(const Vector3d& dv)
{
	P = InvMass > kMinimumInvMass ? P + dv / InvMass : Vector3d::Zero();
}

void				RigidBody::AddLinearMomentum(const Vector3d& dp)
{
	P = InvMass > kMinimumInvMass ? P + dp : Vector3d::Zero();
}

void				RigidBody::AddAngularVelocity(const Vector3d& dw)
{
	L = InvInertia.Invertible() ? (L + InvInertia.Inverse() * dw) : Vector3d::Zero();
}

void				RigidBody::SetDefaultPhysicsMaterial(int idx)
{
	Material = (PhysicsMaterial*)&PhysicsMaterial::defaultMeterialTable[idx];
}

float				RigidBody::GetRestitution() const
{
	return Material ? Material->Restitution : PhysicsMaterial::DefaultRestitution();
}

float				RigidBody::GetFrictionDynamic() const
{
	return Material ? Material->FrictionDynamic : PhysicsMaterial::DefaultFrictionDynamic();
}

float				RigidBody::GetFrictionStatic() const
{
	return Material ? Material->FrictionStatic : PhysicsMaterial::DefaultFrictionStatic();
}

void				RigidBodyDynamic::ApplyForce(const Vector3d& Force)
{
	this->ExtForce += Force;
}

void				RigidBodyDynamic::ApplyTorgue(const Vector3d& Torque)
{
	this->ExtTorque += Torque;
}

void RigidBodyDynamic::ApplyTorgue(const Vector3d& RelativePosToCenterOfMass, const Vector3d& Force)
{
	this->ExtTorque += RelativePosToCenterOfMass.Cross(Force);
}

void				RigidBodyDynamic::AppendShapes(std::vector<Geometry*> *Shapes)
{
	Shapes->push_back(this->Shape);
}

RigidBodyDynamic*	RigidBodyDynamic::CreateRigidBody(Geometry* Shape, const RigidBodyParam& param)
{
	RigidBodyDynamic* Rigid = new RigidBodyDynamic;
	Rigid->InvMass = param.InvMass < kMinimumInvMass ? 0.0f : param.InvMass;
	Rigid->InvInertia = Shape->GetInverseInertia_LocalSpace(Rigid->InvMass);
	Rigid->X = Shape->GetPosition();
	Rigid->Q = Shape->GetRotationQuat();
	Rigid->P = Rigid->InvMass > 0.0f ? param.LinearVelocity / Rigid->InvMass : Vector3d::Zero();
	Rigid->L = Rigid->InvInertia.Invertible() ? Rigid->InvInertia.Inverse() * param.AngularVelocity : Vector3d::Zero();
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

RigidBodyStatic* RigidBodyStatic::CreateRigidBody(Geometry* Shape)
{
	RigidBodyStatic* Rigid = new RigidBodyStatic;
	Rigid->InvMass = 0.0f;
	Rigid->InvInertia = Matrix3d::Zero();
	Rigid->X = Shape->GetPosition();
	Rigid->Q = Shape->GetRotationQuat();
	Rigid->P = Vector3d::Zero();
	Rigid->L = Vector3d::Zero();
	Rigid->Shape = Shape;
	Rigid->Static = true;
	return Rigid;
}

void RigidBodyStatic::SetTransform(const Vector3d& pos, const Quaternion& quat)
{
	X = pos;
	Q = quat;
	Shape->SetPosition(pos);
	Shape->SetRotationQuat(quat);
	Shape->UpdateBoundingVolume();
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
	Shape->UpdateBoundingVolume();
}

void RigidBodyStatic::AppendShapes(std::vector<Geometry*>* Shapes)
{
	Shapes->push_back(this->Shape);
}
