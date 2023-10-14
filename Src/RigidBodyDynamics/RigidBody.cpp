
#include <assert.h>
#include "RigidBody.h"
#include "PhysicsMaterial.h"
#include "../Collision/GeometryObject.h"

RigidBody::RigidBody()
{
	mGeometry = nullptr;
	mMaterialId = 0;
	mNodeId = -1;
}

RigidBody::~RigidBody()
{
	mGeometry = nullptr;
}

void RigidBody::AddGeometry(Geometry* Geom)
{
	if (mGeometry)
	{
		Geometry* g = mGeometry;
		while (g->GetNext())
		{
			g = g->GetNext();
		}
		assert(g->GetNext() == nullptr);
		g->LinkNext(Geom);
	}
	else
	{
		mGeometry = Geom;
	}

	Geometry* g = Geom;
	while (g)
	{
		g->SetParent(this);
		g = g->GetNext();
	}
}

void RigidBody::GetGeometries(std::vector<Geometry*>* Geometries)
{
	for (Geometry* g = mGeometry; g; g = g->GetNext())
	{
		Geometries->push_back(g);
	}
}

int RigidBody::GetNumGeometries() const
{
	int count = 0;
	Geometry* g = mGeometry;
	while (g)
	{
		g = g->GetNext();
		count++;
	}
	return count;
}

void RigidBody::ReleaseGeometries()
{
	Geometry* g = mGeometry;
	while (g)
	{
		Geometry* next = g->GetNext();
		delete g;
		g = next;
	}
}

Matrix3		RigidBody::GetInverseInertia_WorldSpace() const
{
	Matrix3 R = Q.ToRotationMatrix3();
	Matrix3 invInertiaWorld = R * InvInertia * R.Transpose();
	return invInertiaWorld;
}

float		RigidBody::GetKinematicsEnergy() const
{
	float l = GetLinearKinematicsEnergy();
	float a = GetAngularKinematicsEnergy();
	return l + a;
}

float RigidBody::GetLinearKinematicsEnergy() const
{
	return InvMass > kMinimumInvMass ? 0.5f * V.SquareLength() / InvMass : 0.0f;
}

float RigidBody::GetAngularKinematicsEnergy() const
{
	return InvInertia.Invertible() ? 0.5f * W.Dot(InvInertia.Inverse() * W) : 0.0f;
}

void		RigidBody::SetDefaultPhysicsMaterial(uint16_t idx)
{
	mMaterialId = idx;
}

float		RigidBody::GetRestitution() const
{
	return PhysicsMaterial::getMaterial(mMaterialId)->Restitution;
}

float		RigidBody::GetFrictionDynamic() const
{
	return PhysicsMaterial::getMaterial(mMaterialId)->FrictionDynamic;
}

float		RigidBody::GetFrictionStatic() const
{
	return PhysicsMaterial::getMaterial(mMaterialId)->FrictionStatic;
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
	this->X = geom ? geom->GetCenterOfMass() : param.init_pose.pos;
	this->Q = geom ? geom->GetRotation() : param.init_pose.quat;
	this->V = Vector3::Zero();
	this->W = Vector3::Zero();
	if (geom) this->AddGeometry(geom);
}

RigidBodyDynamic::RigidBodyDynamic(const RigidBodyParam& param, Geometry* geom)
{
	this->mRigidType = RigidType::Dynamic;
	this->mMotionType = param.motionType;
	this->InvMass = param.invMass < kMinimumInvMass ? 0.0f : param.invMass;
	this->InvInertia = Geometry::GetInverseInertiaMultibody(geom, this->InvMass);
	this->X = geom ? Geometry::GetCenterOfMassMultibody(geom) : param.init_pose.pos;
	this->Q = geom ? Geometry::GetRotationMultibody(geom) : param.init_pose.quat;
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
	this->Freezing = false;
	this->SolverV = Vector3(INFINITY, INFINITY, INFINITY);
	this->SolverW = Vector3(INFINITY, INFINITY, INFINITY);
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

void RigidBodyDynamic::Freeze()
{
	Freezing = true;
	Sleeping = true;
}

void RigidBodyDynamic::Defreeze()
{
	Freezing = false;
}

void		RigidBodyDynamic::Wakeup()
{
	if (!Freezing)
	{
		Sleeping = false;
	}
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
