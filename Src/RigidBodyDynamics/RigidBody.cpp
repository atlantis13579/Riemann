
#include <assert.h>
#include "RigidBody.h"
#include "PhysicsMaterial.h"
#include "../Collision/GeometryObject.h"
#include "../CollisionPrimitive/MassParameters.h"

namespace Riemann
{
	RigidBody::RigidBody()
	{
		mMaterialId = 0;
	}

	RigidBody::~RigidBody()
	{
	}

	void RigidBody::AddGeometry(GeometryBase* Geom)
	{
		if (Geom)
		{
			mGeometries.push_back(Geom);
			Geom->SetParent(this);

			UpdateMassParameters();
		}
	}

	void RigidBody::GetGeometries(std::vector<GeometryBase*>* Geometries)
	{
		for (GeometryBase* g : mGeometries)
		{
			Geometries->push_back(g);
		}
	}

	size_t RigidBody::GetNumGeometries() const
	{
		return mGeometries.size();
	}

	void RigidBody::ReleaseGeometries()
	{
		for (GeometryBase* g : mGeometries)
		{
			delete g;
		}
		mGeometries.clear();
	}

	Matrix3		RigidBody::GetInverseInertia_WorldSpace() const
	{
		Matrix3 R = Q.ToRotationMatrix3();
		Matrix3 invInertiaWorld = R * InvInertia * R.Transpose();
		return invInertiaWorld;
	}

	void RigidBody::UpdateGeometries()
	{
		for (GeometryBase* g : mGeometries)
		{
			const Transform* local_trans = g->GetLocalTransform();
			Vector3 world_pos = X + Q * (local_trans->pos - CM);
			Quaternion world_quat = Q * local_trans->quat;
			g->SetWorldTransform(world_pos, world_quat);
		}
	}

	void RigidBody::UpdateMassParameters()
	{
		if (mRigidType == RigidType::Static)
		{
			return;
		}

		std::vector<const Transform*> vPose;
		std::vector<const MassParameters*> vProperties;
		MassParameters P;

		for (GeometryBase* g : mGeometries)
		{
			vPose.emplace_back(g->GetLocalTransform());
			vProperties.push_back(g->GetMassParameters());
		}

		ComputeCompositeMassParameters(vPose, vProperties, P);

		CM = P.CenterOfMass;
		InvMass = 1.0f / P.Mass;
		InvInertia = P.InertiaMat.Inverse();
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
	RigidBody* RigidBody::CreateRigidBody(const RigidBodyParam& param, const Transform& init_pose)
	{
		if (param.rigidType == RigidType::Static)
		{
			RigidBodyStatic* Rigid = RigidBodyStatic::CreateRigidBody(param, init_pose);
			return Rigid;
		}
		else if (param.rigidType == RigidType::Dynamic)
		{
			RigidBodyDynamic* Rigid = RigidBodyDynamic::CreateRigidBody(param, init_pose);
			return Rigid;
		}
		return nullptr;
	}

	RigidBodyStatic* RigidBodyStatic::CreateRigidBody(const RigidBodyParam& param, const Transform& init_pose)
	{
		return new RigidBodyStatic(param, init_pose);
	}

	RigidBodyStatic::RigidBodyStatic(const RigidBodyParam& param, const Transform& init_pose)
	{
		this->mRigidType = RigidType::Static;
		this->InvMass = 0.0f;
		this->InvInertia = Matrix3::Zero();
		this->X = init_pose.pos;
		this->Q = init_pose.quat;
		this->V = Vector3::Zero();
		this->W = Vector3::Zero();
	}

	RigidBodyDynamic::RigidBodyDynamic(const RigidBodyParam& param, const Transform& init_pose)
	{
		this->mRigidType = RigidType::Dynamic;
		this->mMotionType = param.motionType;
		this->InvMass = 1.0f;
		this->InvInertia = Matrix3(1.0f, 1.0f, 1.0f);
		this->X = init_pose.pos;
		this->Q = init_pose.quat;
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

	RigidBodyDynamic* RigidBodyDynamic::CreateRigidBody(const RigidBodyParam& param, const Transform& init_pose)
	{
		return new RigidBodyDynamic(param, init_pose);
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
		for (GeometryBase* g : mGeometries)
		{
			g->SetWorldPosition(pos);
		}
	}

	void RigidBodyKinematics::SetRotation(const Quaternion& quat)
	{
		Q = quat;
		for (GeometryBase* g : mGeometries)
		{
			g->SetWorldRotation(quat);
			g->UpdateBoundingVolume();
		}
	}

	void		RigidBodyKinematics::SetTransform(const Vector3& pos, const Quaternion& quat)
	{
		X = pos;
		Q = quat;
		for (GeometryBase* g : mGeometries)
		{
			g->SetWorldTransform(pos, quat);
		}
	}

	void		GetAllGeometries(std::vector<RigidBody*> bodies, std::vector<GeometryBase*>* geometries)
	{
		for (size_t i = 0; i < bodies.size(); ++i)
		{
			bodies[i]->GetGeometries(geometries);
		}
	}
}