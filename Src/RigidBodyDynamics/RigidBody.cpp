
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

Vector3 	RigidBody::GetLinearVelocity() const
{
	return P * InvMass;
}

const Vector3&	RigidBody::GetLinearMomentum() const
{
	return P;
}

Vector3		RigidBody::GetAngularVelocity() const
{
	return InvInertia * L;
}

const Vector3&		RigidBody::GetAngularMomentum() const
{
	return L;
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
	return 0.5f * P.SquareLength() * InvMass;
}

float		RigidBody::GetAngularKinematicsEnergy() const
{
	return 0.5f * L.Dot(InvInertia * L);
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
	P = InvMass > kMinimumInvMass ? v / InvMass : Vector3::Zero();
}

void		RigidBody::SetAngularVelocity(const Vector3 &w)
{
	L = InvInertia.Invertible() ? InvInertia.Inverse() * w : Vector3::Zero();
}

void		RigidBody::AddLinearVelocity(const Vector3& dv)
{
	P = InvMass > kMinimumInvMass ? P + dv / InvMass : Vector3::Zero();
}

void		RigidBody::AddLinearMomentum(const Vector3& dp)
{
	P = InvMass > kMinimumInvMass ? P + dp : Vector3::Zero();
}

void		RigidBody::AddAngularVelocity(const Vector3& dw)
{
	L = InvInertia.Invertible() ? (L + InvInertia.Inverse() * dw) : Vector3::Zero();
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
	this->P = Vector3::Zero();
	this->L = Vector3::Zero();
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
	this->P = this->InvMass > 0.0f ? param.linearVelocity / this->InvMass : Vector3::Zero();
	this->L = this->InvInertia.Invertible() ? this->InvInertia.Inverse() * param.angularVelocity : Vector3::Zero();
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


bool		RigidBodyDynamic::AutoSleep()
{
	float k = GetKinematicsEnergy();
	if (k < SleepThreshold)
	{
		Sleep();
		return true;
	}
	return Sleeping;
}

void		RigidBodyDynamic::Sleep()
{
	Sleeping = true;
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
	P = Vector3::Zero();
	L = Vector3::Zero();
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
