
#include <assert.h>
#include "RigidBody.h"
#include "PhysicsMaterial.h"
#include "../Collision/GeometryObject.h"

static const float kMinimumInvMass = 1e-6f;

RigidBody::RigidBody()
{
	mGeometry = nullptr;
	mMaterial = nullptr;
	mNodeId = -1;
}

RigidBody::~RigidBody()
{
	mGeometry = nullptr;
}

void RigidBody::AddGeometry(Geometry* Geom)
{
	assert(Geom->GetNext() == nullptr);
	if (mGeometry)
	{
		assert(mGeometry->GetNext() == nullptr);
		mGeometry->SetNext(Geom);
		Geom->SetNext(nullptr);
	}
	else
	{
		mGeometry = Geom;
	}
	Geom->SetParent(this);
}

void RigidBody::GetGeometries(std::vector<Geometry*>* Geometries)
{
	for (Geometry* g = mGeometry; g; g = g->GetNext())
	{
		Geometries->push_back(g);
	}
}

const Vector3& 	RigidBody::GetLinearVelocity() const
{
	return V;
}

Vector3			RigidBody::GetLinearMomentum() const
{
	return InvMass > kMinimumInvMass ? Vector3::Zero() : V / InvMass;
}

const Vector3&	RigidBody::GetAngularVelocity() const
{
	return W;
}

Vector3			RigidBody::GetAngularMomentum() const
{
	return InvInertia.Invertible() ? InvInertia.Inverse() * W : Vector3::Zero();;
}

const Matrix3&	 	RigidBody::GetInverseInertia() const
{
	return InvInertia;
}

Matrix3		RigidBody::GetInverseInertia_WorldSpace() const
{
	Matrix3 R = Q.ToRotationMatrix3();
	Matrix3 invInertiaWorld = R * InvInertia * R.Transpose();
	return invInertiaWorld;
}

const float&	RigidBody::GetInverseMass() const
{
	return InvMass;
}

float		RigidBody::GetKinematicsEnergy() const
{
	float l = GetLinearKinematicsEnergy();
	float a = GetAngularKinematicsEnergy();
	return l + a;
}

float		RigidBody::GetLinearKinematicsEnergy() const
{
	return InvMass > kMinimumInvMass ? 0.5f * V.SquareLength() / InvMass : 0.0f;
}

float		RigidBody::GetAngularKinematicsEnergy() const
{
	return InvInertia.Invertible() ? 0.5f * W.Dot(InvInertia.Inverse() * W) : 0.0f;
}

RigidBodyStatic* RigidBody::CastStatic()
{
	return mRigidType == RigidType::Static ? (RigidBodyStatic*)this : nullptr;
}

RigidBodyDynamic* RigidBody::CastDynamic()
{
	return mRigidType == RigidType::Dynamic ? (RigidBodyDynamic*)this : nullptr;
}

void		RigidBody::SetLinearVelocity(const Vector3 &v)
{
	V = v;
}

void		RigidBody::SetAngularVelocity(const Vector3 &w)
{
	W = w;
}

void		RigidBody::SetLinearMomentum(const Vector3& p)
{
	V = InvMass * p;
}

void		RigidBody::SetAngularMomentum(const Vector3& l)
{
	W = InvInertia * l;
}

void		RigidBody::AddLinearVelocity(const Vector3& dv)
{
	V += dv;
}

void		RigidBody::AddLinearMomentum(const Vector3& dp)
{
	V += InvMass * dp;
}

void		RigidBody::AddAngularVelocity(const Vector3& dw)
{
	W += dw;
}

void RigidBody::AddAngularMomentum(const Vector3& dl)
{
	W += InvInertia * dl;
}

void		RigidBody::SetDefaultPhysicsMaterial(int idx)
{
	mMaterial = (PhysicsMaterial*)&PhysicsMaterial::defaultMeterialTable[idx];
}

float		RigidBody::GetRestitution() const
{
	return mMaterial ? mMaterial->Restitution : PhysicsMaterial::DefaultRestitution();
}

float		RigidBody::GetFrictionDynamic() const
{
	return mMaterial ? mMaterial->FrictionDynamic : PhysicsMaterial::DefaultFrictionDynamic();
}

float		RigidBody::GetFrictionStatic() const
{
	return mMaterial ? mMaterial->FrictionStatic : PhysicsMaterial::DefaultFrictionStatic();
}

// static
RigidBody* RigidBody::CreateRigidBody(const RigidBodyParam& param, Geometry* geom)
{
	if (param.rigidType == RigidType::Static)
	{
		RigidBodyStatic* Rigid = RigidBodyStatic::CreateRigidBody(param, geom);
		return Rigid;
	}
	else if (param.rigidType == RigidType::Dynamic)
	{
		RigidBodyDynamic* Rigid = RigidBodyDynamic::CreateRigidBody(param, geom);
		return Rigid;
	}
	return nullptr;
}

RigidBodyStatic* RigidBodyStatic::CreateRigidBody(const RigidBodyParam& param, Geometry* geom)
{
	return new RigidBodyStatic(param, geom);
}

RigidBodyStatic::RigidBodyStatic(const RigidBodyParam& param, Geometry* geom)
{
	this->mRigidType = RigidType::Static;
	this->InvMass = 0.0f;
	this->InvInertia = Matrix3::Zero();
	this->X = geom ? geom->GetCenterOfMass() : param.pos;
	this->Q = geom ? geom->GetRotation() : param.quat;
	this->V = Vector3::Zero();
	this->W = Vector3::Zero();
	this->mGeometry = geom;
	if (geom) this->AddGeometry(geom);
}

RigidBodyDynamic::RigidBodyDynamic(const RigidBodyParam& param, Geometry* geom)
{
	this->mRigidType = RigidType::Dynamic;
	this->mMotionType = param.motionType;
	this->mGeometry = geom;
	this->InvMass = param.invMass < kMinimumInvMass ? 0.0f : param.invMass;
	this->InvInertia = geom->GetInverseInertia_LocalSpace(this->InvMass);
	this->X = geom ? geom->GetCenterOfMass() : param.pos;
	this->Q = geom ? geom->GetRotation() : param.quat;
	this->V = param.linearVelocity;
	this->W = param.angularVelocity;
	this->ExtForce = Vector3::Zero();
	this->ExtTorque = Vector3::Zero();
	this->LinearDamping = param.linearDamping;
	this->AngularDamping = param.angularDamping;
	this->MaxContactImpulse = param.maxContactImpulse;
	this->SleepThreshold = param.sleepThreshold;
	this->FreezeThreshold = param.freezeThreshold;
	this->DisableGravity = param.disableGravity;
	this->Sleeping = false;
	if (geom) this->AddGeometry(geom);
}

void		RigidBodyDynamic::ApplyForce(const Vector3& Force)
{
	this->ExtForce += Force;
}

void		RigidBodyDynamic::ApplyTorgue(const Vector3& Torque)
{
	this->ExtTorque += Torque;
}

void 		RigidBodyDynamic::ApplyTorgue(const Vector3& RelativePosToCenterOfMass, const Vector3& Force)
{
	this->ExtTorque += RelativePosToCenterOfMass.Cross(Force);
}


void RigidBodyDynamic::ApplyLinearAcceleration(const Vector3& LinAcc)
{
	if (InvMass > kMinimumInvMass)
		ApplyForce(LinAcc / InvMass);
}

void RigidBodyDynamic::ApplyAngularAcceleration(const Vector3& AngAcc)
{
	if (InvInertia.Invertible())
		ApplyTorgue(InvInertia.Inverse() * AngAcc);
}

bool		RigidBodyDynamic::AutoSleep()
{
	const float energy = GetKinematicsEnergy();
	if (energy < SleepThreshold)
	{
		Sleep();
	}
	else
	{
		Wakeup();
	}
	return Sleeping;
}

void		RigidBodyDynamic::Sleep()
{
	if (!Sleeping)
	{
		Sleeping = true;
		SetLinearMomentum(Vector3::Zero());
		SetAngularMomentum(Vector3::Zero());
	}
}

void		RigidBodyDynamic::Wakeup()
{
	Sleeping = false;
}

RigidBodyDynamic* RigidBodyDynamic::CreateRigidBody(const RigidBodyParam& param, Geometry* geom)
{
	return new RigidBodyDynamic(param, geom);
}

RigidBodyKinematics::RigidBodyKinematics()
{
	mRigidType = RigidType::Kinematic;
	InvMass = 0.0f;
	InvInertia = Matrix3::Zero();
	X = Vector3::Zero();
	Q = Quaternion::One();
	V = Vector3::Zero();
	W = Vector3::Zero();
}

void RigidBodyKinematics::SetPosition(const Vector3& pos)
{
	X = pos;
	for (Geometry *g = mGeometry; g ; g = g->GetNext())
	{
		g->SetCenterOfMass(pos);
		g->UpdateBoundingVolume();
	}
}

void RigidBodyKinematics::SetRotation(const Quaternion& quat)
{
	Q = quat;
	for (Geometry* g = mGeometry; g; g = g->GetNext())
	{
		g->SetRotation(quat);
		g->UpdateBoundingVolume();
	}
}

void		RigidBodyKinematics::SetTransform(const Vector3& pos, const Quaternion& quat)
{
	X = pos;
	Q = quat;
	for (Geometry* g = mGeometry; g; g = g->GetNext())
	{
		g->SetCenterOfMass(pos);
		g->SetRotation(quat);
		g->UpdateBoundingVolume();
	}
}

void		GetAllGeometries(std::vector<RigidBody*> bodies, std::vector<Geometry*> *geometries)
{
	for (size_t i = 0; i < bodies.size(); ++i)
	{
		bodies[i]->GetGeometries(geometries);
	}
}
