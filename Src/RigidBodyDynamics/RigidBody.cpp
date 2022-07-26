
#include <assert.h>
#include "RigidBody.h"
#include "PhysicsMaterial.h"
#include "../Collision/GeometryObject.h"

static const float kMinimumInvMass = 1e-6f;

RigidBody::RigidBody()
{
	mGeometry = nullptr;
	mMaterial = nullptr;
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
}

void RigidBody::GetGeometries(std::vector<Geometry*>* Geometries)
{
	for (Geometry* g = mGeometry; g; g = g->GetNext())
	{
		Geometries->push_back(g);
	}
}

Vector3 			RigidBody::GetLinearVelocity() const
{
	return P * InvMass;
}

const Vector3&		RigidBody::GetLinearMomentum() const
{
	return P;
}

Vector3			RigidBody::GetAngularVelocity() const
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

Matrix3			RigidBody::GetInverseInertia_WorldSpace() const
{
	Matrix3 R = Q.ToRotationMatrix();
	Matrix3 invInertiaWorld = R * InvInertia * R.Transpose();
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

RigidBodyStatic* RigidBody::CastStatic()
{
	return mRigidType == RigidType::Static ? (RigidBodyStatic*)this : nullptr;
}

RigidBodyDynamic* RigidBody::CastDynamic()
{
	return mRigidType == RigidType::Dynamic ? (RigidBodyDynamic*)this : nullptr;
}

void				RigidBody::SetLinearVelocity(const Vector3 &v)
{
	P = InvMass > kMinimumInvMass ? v / InvMass : Vector3::Zero();
}

void				RigidBody::SetAngularVelocity(const Vector3 &w)
{
	L = InvInertia.Invertible() ? InvInertia.Inverse() * w : Vector3::Zero();
}

void				RigidBody::AddLinearVelocity(const Vector3& dv)
{
	P = InvMass > kMinimumInvMass ? P + dv / InvMass : Vector3::Zero();
}

void				RigidBody::AddLinearMomentum(const Vector3& dp)
{
	P = InvMass > kMinimumInvMass ? P + dp : Vector3::Zero();
}

void				RigidBody::AddAngularVelocity(const Vector3& dw)
{
	L = InvInertia.Invertible() ? (L + InvInertia.Inverse() * dw) : Vector3::Zero();
}

void				RigidBody::SetDefaultPhysicsMaterial(int idx)
{
	mMaterial = (PhysicsMaterial*)&PhysicsMaterial::defaultMeterialTable[idx];
}

float				RigidBody::GetRestitution() const
{
	return mMaterial ? mMaterial->Restitution : PhysicsMaterial::DefaultRestitution();
}

float				RigidBody::GetFrictionDynamic() const
{
	return mMaterial ? mMaterial->FrictionDynamic : PhysicsMaterial::DefaultFrictionDynamic();
}

float				RigidBody::GetFrictionStatic() const
{
	return mMaterial ? mMaterial->FrictionStatic : PhysicsMaterial::DefaultFrictionStatic();
}

void				RigidBodyDynamic::ApplyForce(const Vector3& Force)
{
	this->ExtForce += Force;
}

void				RigidBodyDynamic::ApplyTorgue(const Vector3& Torque)
{
	this->ExtTorque += Torque;
}

void RigidBodyDynamic::ApplyTorgue(const Vector3& RelativePosToCenterOfMass, const Vector3& Force)
{
	this->ExtTorque += RelativePosToCenterOfMass.Cross(Force);
}

void				RigidBodyDynamic::AppendShapes(std::vector<Geometry*> *Shapes)
{
	Shapes->push_back(this->mGeometry);
}

void RigidBodyStatic::SetTransform(const Vector3& pos, const Quaternion& quat)
{
	X = pos;
	Q = quat;
	mGeometry->SetPosition(pos);
	mGeometry->SetRotationQuat(quat);
	mGeometry->UpdateBoundingVolume();
}

void RigidBodyStatic::SetPosition(const Vector3& pos)
{
	X = pos;
	mGeometry->SetPosition(pos);
}

void RigidBodyStatic::SetRotation(const Quaternion& quat)
{
	Q = quat;
	mGeometry->SetRotationQuat(quat);
	mGeometry->UpdateBoundingVolume();
}

void RigidBodyStatic::AppendShapes(std::vector<Geometry*>* Shapes)
{
	Shapes->push_back(this->mGeometry);
}

RigidBodyDynamic* RigidBodyDynamic::CreateRigidBody(const RigidBodyParam& param, Geometry* geom)
{
	RigidBodyDynamic* body = new RigidBodyDynamic;
	body->InvMass = param.invMass < kMinimumInvMass ? 0.0f : param.invMass;
	body->InvInertia = geom->GetInverseInertia_LocalSpace(body->InvMass);
	body->X = geom ? geom->GetPosition() : param.pos;
	body->Q = geom ? geom->GetRotationQuat() : param.quat;
	body->P = body->InvMass > 0.0f ? param.linearVelocity / body->InvMass : Vector3::Zero();
	body->L = body->InvInertia.Invertible() ? body->InvInertia.Inverse() * param.angularVelocity : Vector3::Zero();
	body->ExtForce = Vector3::Zero();
	body->ExtTorque = Vector3::Zero();
	body->LinearDamping = param.linearDamping;
	body->AngularDamping = param.angularDamping;
	body->MaxContactImpulse = param.maxContactImpulse;
	body->SleepThreshold = param.sleepThreshold;
	body->FreezeThreshold = param.freezeThreshold;
	body->DisableGravity = param.disableGravity;
	body->mRigidType = RigidType::Dynamic;
	body->Sleep = false;
	body->mGeometry = geom;
	body->mMotionType = param.motionType;
	if (geom) geom->SetParent(body);
	return body;
}

RigidBodyStatic* RigidBodyStatic::CreateRigidBody(const RigidBodyParam& param, Geometry* geom)
{
	RigidBodyStatic* body = new RigidBodyStatic;
	body->InvMass = 0.0f;
	body->InvInertia = Matrix3::Zero();
	body->X = geom ? geom->GetPosition() : param.pos;
	body->Q = geom ? geom->GetRotationQuat() : param.quat;
	body->P = Vector3::Zero();
	body->L = Vector3::Zero();
	body->mGeometry = geom;
	body->mRigidType = RigidType::Dynamic;
	body->mMotionType = param.motionType;
	if (geom) geom->SetParent(body);
	return body;
}

// static
RigidBody* RigidBody::CreateRigidBody(const RigidBodyParam& param, Geometry* geom)
{
	if (param.rigidType == RigidType::Static)
	{
		RigidBodyStatic* Rigid = RigidBodyStatic::CreateRigidBody(param, geom);
		return Rigid;
	}

	RigidBodyDynamic* Rigid = RigidBodyDynamic::CreateRigidBody(param, geom);
	return Rigid;
}

void GetAllGeometries(std::vector<RigidBody*> bodies, std::vector<Geometry*> *geometries)
{
	for (size_t i = 0; i < bodies.size(); ++i)
	{
		bodies[i]->GetGeometries(geometries);
	}
}
