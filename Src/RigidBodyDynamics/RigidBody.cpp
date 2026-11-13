
#include <assert.h>
#include <algorithm>
#include <stdexcept>
#include "RigidBody.h"
#include "Ragdoll.h"
#include "PhysicsMaterial.h"
#include "MassParameters.h"
#include "../Collision/GeometryObject.h"

namespace Riemann
{
	RigidBody::RigidBody()
	{
		mMaterialId = 0;
		mRestitutionOverride = -1.0f;
		Parent = nullptr;
		CollisionGroup = nullptr;
		EnableCollisionWithinGroup = true;
		CM = Vector3::Zero();
	}

	RigidBody::~RigidBody()
	{
		// Collision ignores are reciprocal, so every remaining pointer is still alive.
		for (RigidBody* other : mIgnoredCollisionBodies)
		{
			other->mIgnoredCollisionBodies.erase(this);
		}
		mIgnoredCollisionBodies.clear();
	}

	void RigidBody::IgnoreCollisionWith(RigidBody* Other, bool Ignore)
	{
		if (Other == nullptr || Other == this)
		{
			return;
		}

		const bool Changed = IsCollisionIgnoredWith(Other) != Ignore;
		if (Ignore)
		{
			mIgnoredCollisionBodies.insert(Other);
			Other->mIgnoredCollisionBodies.insert(this);
		}
		else
		{
			mIgnoredCollisionBodies.erase(Other);
			Other->mIgnoredCollisionBodies.erase(this);
		}
		if (Changed) for (RigidBody* Body : { this, Other })
		{
			auto* Dynamic = Body->CastDynamic();
			if (Dynamic && Dynamic->IslandSleepManaged) Dynamic->Wakeup();
		}
	}

	bool RigidBody::IsCollisionIgnoredWith(const RigidBody* Other) const
	{
		return Other != nullptr && mIgnoredCollisionBodies.find(const_cast<RigidBody*>(Other)) != mIgnoredCollisionBodies.end();
	}

	bool RigidBody::CanCollideWith(const RigidBody* Other) const
	{
		if (Other == nullptr || Other == this || IsCollisionIgnoredWith(Other))
		{
			return false;
		}
		if (CollisionGroup != nullptr && CollisionGroup == Other->CollisionGroup
			&& (!EnableCollisionWithinGroup || !Other->EnableCollisionWithinGroup))
		{
			return false;
		}
		return true;
	}

	void RigidBody::AddGeometry(Geometry* Geom)
	{
		if (Geom)
		{
			mGeometries.push_back(Geom);
			mGeometryHandles.push_back(GeometryHandle());
			Geom->SetParent(this);

			UpdateMassParameters();
		}
	}

	void RigidBody::GetGeometries(std::vector<Geometry*>* Geometries)
	{
		for (Geometry* g : mGeometries)
		{
			Geometries->push_back(g);
		}
	}

	size_t RigidBody::GetNumGeometries() const
	{
		return mGeometries.size();
	}

	GeometryHandle RigidBody::GetGeometryHandle(size_t Index) const
	{
		return Index < mGeometryHandles.size() ? mGeometryHandles[Index] : GeometryHandle();
	}

	void RigidBody::SetGeometryHandle(size_t Index, GeometryHandle Handle)
	{
		if (Index >= mGeometryHandles.size())
		{
			return;
		}
		mGeometryHandles[Index] = Handle;
	}

	GeometryHandle RigidBody::FindGeometryHandle(const Geometry* Geom) const
	{
		const size_t index = FindGeometryIndex(Geom);
		return index < mGeometryHandles.size() ? mGeometryHandles[index] : GeometryHandle();
	}

	size_t RigidBody::FindGeometryIndex(const Geometry* Geom) const
	{
		for (size_t i = 0; i < mGeometries.size(); ++i)
		{
			if (mGeometries[i] == Geom)
			{
				return i;
			}
		}
		return mGeometries.size();
	}

	void RigidBody::ReleaseGeometries()
	{
		for (Geometry* g : mGeometries)
		{
			delete g;
		}
		mGeometries.clear();
		mGeometryHandles.clear();
	}

	Matrix3		RigidBody::GetInverseInertia_WorldSpace() const
	{
		if (mRigidType == RigidType::Dynamic && static_cast<const RigidBodyDynamic*>(this)->Freezing) return Matrix3::Zero();
		Matrix3 R = Q.ToRotationMatrix3();
		Matrix3 invInertiaWorld = R * InvInertia * R.Transpose();
		return invInertiaWorld;
	}

	Vector3		RigidBody::BodyLocalToWorld(const Vector3& Position) const
	{
		return X + Q * (Position - CM);
	}

	float RigidBody::GetInverseMass() const
	{
		return mRigidType == RigidType::Dynamic && static_cast<const RigidBodyDynamic*>(this)->Freezing ? 0 : InvMass;
	}

	Vector3		RigidBody::BodyLocalToWorldDirection(const Vector3& Direction) const
	{
		return Q * Direction;
	}

	Vector3		RigidBody::WorldToBodyLocal(const Vector3& Position) const
	{
		return Q.Conjugate() * (Position - X) + CM;
	}

	Vector3		RigidBody::WorldToBodyLocalDirection(const Vector3& Direction) const
	{
		return Q.Conjugate() * Direction;
	}

	Transform	RigidBody::GetGeometryTransform(const Geometry* Geom) const
	{
		if (Geom == nullptr)
		{
			return Transform::Identity();
		}

		const Transform* localTransform = Geom->GetTransform();
		return Transform(BodyLocalToWorld(localTransform->pos), Q * localTransform->quat);
	}

	Box3		RigidBody::GetGeometryBounds(const Geometry* Geom) const
	{
		if (Geom == nullptr)
		{
			return Box3::Empty();
		}

		const Transform transform = GetGeometryTransform(Geom);
		return Box3::Transform(Geom->GetShapeBounds(), transform.pos, transform.quat);
	}

	void RigidBody::UpdateGeometries()
	{
	}

	void RigidBody::UpdateMassParameters()
	{
		if (mRigidType != RigidType::Dynamic)
		{
			return;
		}

		std::vector<const Transform*> vPose;
		std::vector<const MassParameters*> vProperties;
		MassParameters P;

		for (Geometry* g : mGeometries)
		{
			vPose.emplace_back(g->GetTransform());
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
		mRestitutionOverride = -1.0f;
		mFrictionDynamicOverride = mFrictionStaticOverride = -1.0f;
	}

	void		RigidBody::SetRestitution(float restitution)
	{
		mRestitutionOverride = std::max(0.0f, std::min(restitution, 1.0f));
	}

	float		RigidBody::GetRestitution() const
	{
		return mRestitutionOverride >= 0.0f
			? mRestitutionOverride
			: PhysicsMaterial::getMaterial(mMaterialId)->Restitution;
	}

	float		RigidBody::GetFrictionDynamic() const
	{
		return mFrictionDynamicOverride >= 0.0f ? mFrictionDynamicOverride : PhysicsMaterial::getMaterial(mMaterialId)->FrictionDynamic;
	}

	float		RigidBody::GetFrictionStatic() const
	{
		return mFrictionStaticOverride >= 0.0f ? mFrictionStaticOverride : PhysicsMaterial::getMaterial(mMaterialId)->FrictionStatic;
	}

	void RigidBody::SetFriction(float Dynamic, float Static)
	{
		if (!std::isfinite(Dynamic) || !std::isfinite(Static) || Dynamic < 0.0f || Static < Dynamic)
			throw std::invalid_argument("Friction requires finite coefficients with static >= dynamic >= 0");
		mFrictionDynamicOverride = Dynamic;
		mFrictionStaticOverride = Static;
		if (CastDynamic()) CastDynamic()->Wakeup();
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
		else if (param.rigidType == RigidType::Kinematic)
		{
			auto* Rigid = new RigidBodyKinematics;
			Rigid->SetTransform(init_pose.pos, init_pose.quat);
			Rigid->V = param.linearVelocity;
			Rigid->W = param.angularVelocity;
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
		this->CCDMotionFraction = 1.0f;
		this->HadCCDContact = false;
		this->CCDPreSolveVelocity = Vector3::Zero();
		this->CCDPreSolveAngularVelocity = Vector3::Zero();
	}

	void		RigidBodyDynamic::ApplyForce(const Vector3& Force)
	{
		if (Force != Vector3::Zero())
		{
			Wakeup();
		}
		this->ExtForce += Force;
	}

	void		RigidBodyDynamic::ApplyTorgue(const Vector3& Torque)
	{
		if (Torque != Vector3::Zero())
		{
			Wakeup();
		}
		this->ExtTorque += Torque;
	}

	void 		RigidBodyDynamic::ApplyTorgue(const Vector3& RelativePosToCenterOfMass, const Vector3& Force)
	{
		const Vector3 Torque = RelativePosToCenterOfMass.Cross(Force);
		if (Torque != Vector3::Zero())
		{
			Wakeup();
		}
		this->ExtTorque += Torque;
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
		if (ReducedRagdoll) { ReducedRagdoll->Sleep(); return; }
		if (!Sleeping)
		{
			Sleeping = true;
			SetLinearMomentum(Vector3::Zero());
			SetAngularMomentum(Vector3::Zero());
		}
	}

	void RigidBodyDynamic::Freeze()
	{
		if (ReducedRagdoll) { ReducedRagdoll->SetFrozen(true); return; }
		Freezing = true;
		Sleeping = true;
	}

	void RigidBodyDynamic::Defreeze()
	{
		if (ReducedRagdoll) { ReducedRagdoll->SetFrozen(false); return; }
		Freezing = false;
	}

	void		RigidBodyDynamic::Wakeup()
	{
		if (ReducedRagdoll) { ReducedRagdoll->Wakeup(); return; }
		++m_WakeRevision;
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
	}

	void RigidBodyKinematics::SetRotation(const Quaternion& quat)
	{
		Q = quat;
	}

	void		RigidBodyKinematics::SetTransform(const Vector3& pos, const Quaternion& quat)
	{
		X = pos;
		Q = quat;
	}

	void		GetAllGeometries(std::vector<RigidBody*> bodies, std::vector<Geometry*>* geometries)
	{
		for (size_t i = 0; i < bodies.size(); ++i)
		{
			bodies[i]->GetGeometries(geometries);
		}
	}
}
