#include "Ragdoll.h"

#include <algorithm>
#include <unordered_map>
#include <cmath>
#include <stdexcept>
#include "Jacobian.h"

#include "PhysicsWorld.h"
#include "RagdollDefinition.h"
#include "Constraints.h"
#include "RigidBody.h"
#include "../Collision/GeometryObject.h"

namespace Riemann
{
	RagdollSettings::RagdollSettings()
		: Scale(1.0f)
		, LinearDamping(0.995f)
		, AngularDamping(0.995f)
		, LinearVelocity(Vector3::Zero())
		, AngularVelocity(Vector3::Zero())
		, JointVelocity(0.0f)
		, EnableCCD(false)
		, DisableSelfCollision(false)
		, DefinitionFile("Contents/Config/default_ragdoll.json")
	{
	}

	RagdollSettings RagdollSettings::Reduced()
	{
		RagdollSettings Settings;
		Settings.Algorithm = RagdollAlgorithm::ReducedCoordinates;
		return Settings;
	}

	Ragdoll::Ragdoll(PhysicsWorld* world, const RagdollSettings& settings, const Transform& rootTransform)
		: m_World(world)
		, m_Settings(settings)
		, m_RootTransform(rootTransform)
		, m_Valid(false)
	{
		if (IsReduced() && m_Settings.EnableCCD)
		{
			m_Error = "Reduced-coordinate ragdolls require discrete motion";
			return;
		}
		m_Settings.Scale = std::max(0.05f, m_Settings.Scale);
		Build();
	}

	RigidBodyDynamic* Ragdoll::GetBone(RagdollBone bone) const
	{
		if (IsReduced() && !m_OwnsBodies) return nullptr;
		const size_t index = (size_t)bone;
		return index < m_Bodies.size() ? m_Bodies[index] : nullptr;
	}

	void Ragdoll::Destroy()
	{
		if (m_World == nullptr)
		{
			return;
		}
		if (IsReduced())
		{
			if (m_World->RemoveRagdoll(this)) return;
			DetachReducedBodies();
			if (!m_OwnsBodies) return;
		}
		auto Constraints = std::move(m_Constraints);
		for (PositionConstraint* constraint : Constraints)
		{
			m_World->RemoveConstraint(constraint);
		}
		m_Constraints.clear();
		auto Bodies = std::move(m_Bodies);
		for (RigidBodyDynamic* body : Bodies)
		{
			if (m_World->RemoveRigidBody(body))
			{
				body->ReleaseGeometries();
				delete body;
			}
		}
		m_Bodies.clear();
		m_Valid = false;
	}
	void Ragdoll::OnBodyRemoved(RigidBody* Body)
	{
		if (IsReduced()) return;
		for (auto*& Bone : m_Bodies) if (Bone == Body)
		{
			Bone = nullptr; m_Valid = false; m_Error = "A ragdoll body was removed";
		}
	}
	void Ragdoll::OnConstraintRemoved(PositionConstraint* Constraint)
	{
		const auto Found = std::find(m_Constraints.begin(), m_Constraints.end(), Constraint);
		if (Found == m_Constraints.end()) return;
		m_Constraints.erase(Found); m_Valid = false; m_Error = "A ragdoll constraint was removed";
	}

	Vector3 Ragdoll::ToWorld(const Vector3& localPosition) const
	{
		return m_RootTransform.LocalToWorld(localPosition * m_Settings.Scale);
	}

	Vector3 Ragdoll::ToWorldDirection(const Vector3& localDirection) const
	{
		return m_RootTransform.LocalToWorldDirection(localDirection).SafeUnit();
	}

	RigidBodyDynamic* Ragdoll::CreateBoxBone(const Vector3& center, const Vector3& halfExtent, const Quaternion& rotation)
	{
		Geometry* geometry = GeometryFactory::CreateOBB(ToWorld(center), halfExtent * m_Settings.Scale, m_RootTransform.quat * rotation);
		RigidBodyParam bodyParam;
		bodyParam.rigidType = RigidType::Dynamic;
		bodyParam.linearDamping = m_Settings.LinearDamping;
		bodyParam.angularDamping = m_Settings.AngularDamping;
		bodyParam.sleepThreshold = 0.0f;
		bodyParam.motionType = m_Settings.EnableCCD ? MorionType::Continuous : MorionType::Discrete;
		RigidBodyDynamic* body = m_World->CreateRigidBody(geometry, bodyParam)->CastDynamic();
		body->CollisionGroup = this;
		body->EnableCollisionWithinGroup = !m_Settings.DisableSelfCollision;
		m_Bodies.push_back(body);
		return body;
	}

	RigidBodyDynamic* Ragdoll::CreateCapsuleBone(const Vector3& start, const Vector3& end, float radius)
	{
		Geometry* geometry = GeometryFactory::CreateCapsule(ToWorld(start), ToWorld(end), radius * m_Settings.Scale);
		RigidBodyParam bodyParam;
		bodyParam.rigidType = RigidType::Dynamic;
		bodyParam.linearDamping = m_Settings.LinearDamping;
		bodyParam.angularDamping = m_Settings.AngularDamping;
		bodyParam.sleepThreshold = 0.0f;
		bodyParam.motionType = m_Settings.EnableCCD ? MorionType::Continuous : MorionType::Discrete;
		RigidBodyDynamic* body = m_World->CreateRigidBody(geometry, bodyParam)->CastDynamic();
		body->CollisionGroup = this;
		body->EnableCollisionWithinGroup = !m_Settings.DisableSelfCollision;
		m_Bodies.push_back(body);
		return body;
	}

	RigidBodyDynamic* Ragdoll::CreateSphereBone(const Vector3& center, float radius)
	{
		Geometry* geometry = GeometryFactory::CreateSphere(ToWorld(center), radius * m_Settings.Scale);
		RigidBodyParam bodyParam;
		bodyParam.rigidType = RigidType::Dynamic;
		bodyParam.linearDamping = m_Settings.LinearDamping;
		bodyParam.angularDamping = m_Settings.AngularDamping;
		bodyParam.sleepThreshold = 0.0f;
		bodyParam.motionType = m_Settings.EnableCCD ? MorionType::Continuous : MorionType::Discrete;
		RigidBodyDynamic* body = m_World->CreateRigidBody(geometry, bodyParam)->CastDynamic();
		body->CollisionGroup = this;
		body->EnableCollisionWithinGroup = !m_Settings.DisableSelfCollision;
		m_Bodies.push_back(body);
		return body;
	}

	void Ragdoll::Build()
	{
		if (m_World == nullptr)
		{
			return;
		}

		RagdollDefinition definition;
		std::vector<std::string> candidates;
		candidates.push_back(m_Settings.DefinitionFile);
		candidates.push_back("../" + m_Settings.DefinitionFile);
		candidates.push_back("../../" + m_Settings.DefinitionFile);
		candidates.push_back("../../../" + m_Settings.DefinitionFile);
		bool loaded = false;
		for (const std::string& candidate : candidates)
		{
			if (LoadRagdollDefinition(candidate.c_str(), &definition, &m_Error))
			{
				loaded = true;
				break;
			}
		}
		if (!loaded) return;

		const char* expectedBoneNames[] = { "pelvis", "torso", "head",
			"left_upper_arm", "left_lower_arm", "right_upper_arm", "right_lower_arm",
			"left_upper_leg", "left_lower_leg", "right_upper_leg", "right_lower_leg" };
		if (definition.Bones.size() != (size_t)RagdollBone::Count)
		{
			m_Error = "ragdoll definition must contain every bone";
			return;
		}

		std::unordered_map<std::string, RigidBodyDynamic*> bodiesByName;
		for (size_t index = 0; index < definition.Bones.size(); ++index)
		{
			const RagdollBoneDefinition& bone = definition.Bones[index];
			if (bone.Name != expectedBoneNames[index])
			{
				m_Error = "ragdoll bones are missing or out of order";
				return;
			}
			RigidBodyDynamic* body = nullptr;
			if (bone.Shape == RagdollBoneShape::Box)
				body = CreateBoxBone(bone.Center, bone.HalfExtent, bone.Rotation);
			else if (bone.Shape == RagdollBoneShape::Capsule)
				body = CreateCapsuleBone(bone.Start, bone.End, bone.Radius);
			else if (bone.Shape == RagdollBoneShape::Sphere)
				body = CreateSphereBone(bone.Center, bone.Radius);
			if (body == nullptr)
			{
				m_Error = "failed to create ragdoll bone " + bone.Name;
				return;
			}
			bodiesByName[bone.Name] = body;
		}
		if (IsReduced())
		{
			try { BuildReduced(definition); }
			catch (const std::exception& Error) { m_Error = Error.what(); }
			return;
		}

		for (const RagdollJointDefinition& joint : definition.Joints)
		{
			const auto bodyA = bodiesByName.find(joint.BodyA);
			const auto bodyB = bodiesByName.find(joint.BodyB);
			if (bodyA == bodiesByName.end() || bodyB == bodiesByName.end())
			{
				m_Error = "ragdoll joint references an unknown bone";
				return;
			}
			PositionConstraint* constraint = nullptr;
			if (joint.Type == RagdollJointType::Cone)
			{
				constraint = m_World->CreateConeConstraint(bodyA->second, bodyB->second,
					ToWorld(joint.Anchor), ToWorldDirection(joint.AxisA),
					ToWorldDirection(joint.AxisB), joint.ConeLimit);
			}
			else
			{
				constraint = m_World->CreateHingeConstraint(bodyA->second, bodyB->second,
					ToWorld(joint.Anchor), ToWorldDirection(joint.AxisA),
					joint.MinAngle, joint.MaxAngle);
			}
			if (constraint == nullptr)
			{
				m_Error = "failed to create ragdoll joint";
				return;
			}
			m_Constraints.push_back(constraint);
		}

		if (!m_Settings.DisableSelfCollision)
		{
			// Jointed neighbors overlap at their anchors and should not collide. Other
			// bone pairs retain self collision, matching common ragdoll group filters.
			for (PositionConstraint* constraint : m_Constraints)
			{
				constraint->GetBodyA()->IgnoreCollisionWith(constraint->GetBodyB());
			}
		}

		const Vector3 rootPosition = ToWorld(Vector3::Zero());
		for (RigidBodyDynamic* body : m_Bodies)
		{
			body->V = m_Settings.LinearVelocity + m_Settings.AngularVelocity.Cross(body->X - rootPosition);
			body->W = m_Settings.AngularVelocity;
		}
		for (size_t index = 0; index < definition.Bones.size(); ++index)
		{
			const Vector3 axis = definition.Bones[index].JointVelocityAxis;
			if (!axis.IsZero()) m_Bodies[index]->W += ToWorldDirection(axis) * m_Settings.JointVelocity;
		}

		m_Valid = m_Bodies.size() == (size_t)RagdollBone::Count
			&& m_Constraints.size() == definition.Joints.size();
		if (m_Valid) m_Error.clear();
	}

	void Ragdoll::BuildReduced(const RagdollDefinition& Definition)
	{
		const size_t Count = m_Bodies.size();
		std::unordered_map<std::string, size_t> Indices;
		for (size_t I = 0; I < Count; ++I) Indices.emplace(Definition.Bones[I].Name, I);
		std::vector<const RagdollJointDefinition*> Incoming(Count, nullptr);
		std::vector<Transform> Frames(Count);
		Frames[0] = Transform(m_Bodies[0]->X, m_Bodies[0]->Q);
		for (const RagdollJointDefinition& Joint : Definition.Joints)
		{
			const auto Parent = Indices.find(Joint.BodyA), Child = Indices.find(Joint.BodyB);
			if (Parent == Indices.end() || Child == Indices.end() || Child->second == 0
				|| Parent->second >= Child->second || Incoming[Child->second])
				throw std::invalid_argument("Ragdoll joints must form a parent-first tree");
			Incoming[Child->second] = &Joint;
			Frames[Child->second] = Transform(ToWorld(Joint.Anchor), m_Bodies[Child->second]->Q);
		}
		if (std::any_of(Incoming.begin() + 1, Incoming.end(), [](const RagdollJointDefinition* Joint) { return Joint == nullptr; }))
			throw std::invalid_argument("Every reduced ragdoll bone requires an incoming joint");

		m_Settings.Links.resize(Count);
		m_Settings.SelfCollision = !m_Settings.DisableSelfCollision;
		for (size_t I = 0; I < Count; ++I)
		{
			auto& Link = m_Settings.Links[I];
			const auto* Body = m_Bodies[I];
			if (Body->InvMass <= 0 || !Body->InvInertia.Invertible())
				throw std::invalid_argument("Reduced ragdoll bones need positive mass and inertia");
			Link.Mass = 1 / Body->InvMass;
			Link.Inertia = Body->InvInertia.Inverse();
			Link.CenterOfMass = Frames[I].WorldToLocal(Body->X);
			if (I == 0)
			{
				Link.Joint = RagdollReducedJointType::Floating;
				Link.ParentToJoint = Frames[I];
				continue;
			}
			const auto& Joint = *Incoming[I];
			Link.Parent = (int)Indices.at(Joint.BodyA);
			Link.Joint = Joint.Type == RagdollJointType::Hinge
				? RagdollReducedJointType::Revolute : RagdollReducedJointType::Spherical;
			Link.ParentToJoint = Frames[(size_t)Link.Parent].TransformInv(Frames[I]);
			Link.Axis = Frames[I].WorldToLocalDirection(ToWorldDirection(Joint.AxisA));
			if (Joint.Type == RagdollJointType::Cone)
			{
				Link.ConeAxisParent = Frames[(size_t)Link.Parent].WorldToLocalDirection(ToWorldDirection(Joint.AxisA));
				Link.ConeAxisChild = Frames[I].WorldToLocalDirection(ToWorldDirection(Joint.AxisB));
				Link.ConeLimit = Joint.ConeLimit;
			}
		}
		m_Dynamics.reset(new FeatherstoneSolver(m_Settings.Links, m_Settings.Numerics));
		AttachReducedBodies();
		for (size_t I = 1; I < Count; ++I)
		{
			const auto& Joint = *Incoming[I];
			if (Joint.Type != RagdollJointType::Hinge) continue;
			auto& Control = m_Controls[(size_t)m_Dynamics->GetLinkDofOffset(I)];
			Control.LimitEnabled = true;
			Control.Lower = Joint.MinAngle;
			Control.Upper = Joint.MaxAngle;
		}
		if (m_Settings.SelfCollision)
			for (size_t I = 1; I < Count; ++I)
				m_Bodies[(size_t)m_Settings.Links[I].Parent]->IgnoreCollisionWith(m_Bodies[I]);

		auto Velocity = m_Dynamics->GetVelocity();
		const Vector3 Angular = Frames[0].WorldToLocalDirection(m_Settings.AngularVelocity);
		const Vector3 Linear = Frames[0].WorldToLocalDirection(m_Settings.LinearVelocity);
		Velocity[0] = Angular.x; Velocity[1] = Angular.y; Velocity[2] = Angular.z;
		Velocity[3] = Linear.x; Velocity[4] = Linear.y; Velocity[5] = Linear.z;
		for (size_t I = 1; I < Count; ++I)
		{
			if (m_Settings.Links[I].Joint != RagdollReducedJointType::Revolute
				|| Definition.Bones[I].JointVelocityAxis.IsZero()) continue;
			const Vector3 Axis = Frames[I].WorldToLocalDirection(ToWorldDirection(Definition.Bones[I].JointVelocityAxis));
			Velocity[(size_t)m_Dynamics->GetLinkDofOffset(I)] = Axis.Dot(m_Settings.Links[I].Axis) * m_Settings.JointVelocity;
		}
		m_Dynamics->SetVelocity(Velocity);
		Synchronize(false);
		m_Valid = true;
		m_Error.clear();
	}
}

namespace Riemann
{
	/* Link bodies are collision/query proxies for one reduced-coordinate state.
	   Their poses and velocities are outputs, not independently integrated states.
	   ABA consumes physical forces at joint origins; collision impulses use the
	   same factorized inverse mass operator. The COM shift below keeps each shape's
	   offset from its COM unchanged when the model supplies a new joint frame.

	   Scalar drives use implicit spring/damper impulses, capped once per substep.
	   Limits constrain predicted coordinates through unilateral velocity rows.
	   Spherical/floating velocity targets use child-frame motion coordinates;
	   scalar position limits/targets are restricted to hinges and sliders. A ball
	   joint needs an orientation error, not three unrelated scalar angle errors.
	   Damping belongs to the reduced-coordinate model; legacy body damping is not applied
	   independently because that could create velocities outside the joint space. */
	Ragdoll::Ragdoll(PhysicsWorld& World, const RagdollSettings& Settings,
		const std::vector<RigidBodyDynamic*>& Bodies)
		: m_World(&World), m_Settings(Settings), m_RootTransform(Transform::Identity()), m_Bodies(Bodies), m_Valid(true), m_OwnsBodies(false), m_Dynamics(new FeatherstoneSolver(Settings.Links, Settings.Numerics))
	{
		if (Settings.Algorithm != RagdollAlgorithm::ReducedCoordinates || Bodies.size() != Settings.Links.size() || Settings.ControlIterations <= 0
			|| !std::isfinite(Settings.MinimumResponse) || Settings.MinimumResponse <= 0)
			throw std::invalid_argument("Invalid reduced ragdoll runtime settings");
		AttachReducedBodies();
	}
	void Ragdoll::AttachReducedBodies()
	{
		const auto& Bodies = m_Bodies;
		const auto& Settings = m_Settings;
		m_Saved.reserve(Bodies.size());
		m_Controls.resize(m_Dynamics->GetDofCount()); m_Forces.resize(m_Controls.size()); m_Rows.resize(m_Controls.size());
		m_ConeRows.resize(Bodies.size()*2);
		m_CoordinateLinks.resize(m_Controls.size());
		for (size_t I = 0; I < Bodies.size(); ++I)
		{
			auto* Body = Bodies[I];
			for (int Dof = 0; Dof < m_Dynamics->GetLinkDofCount(I); ++Dof)
				m_CoordinateLinks[m_Dynamics->GetLinkDofOffset(I) + Dof] = I;
			if (!Body || Body->ReducedRagdoll || Body->Parent || Body->Freezing
				|| std::find(Bodies.begin(), Bodies.begin() + I, Body) != Bodies.begin() + I)
				throw std::invalid_argument("Ragdoll links must be distinct, unfrozen, unbound dynamic bodies");
			m_Saved.push_back({ Body->CM, Body->InvMass, Body->InvInertia, Body->CollisionGroup, Body->EnableCollisionWithinGroup });
		}
		for (size_t I = 0; I < Bodies.size(); ++I)
		{
			auto* Body = Bodies[I]; const auto& Link = m_Dynamics->GetLinkSettings(I);
			for (auto* Shape : Body->Geometries()) Shape->GetTransform()->pos += Link.CenterOfMass - Body->CM;
			Body->CM = Link.CenterOfMass; Body->InvMass = 1 / Link.Mass; Body->InvInertia = Link.Inertia.Inverse();
			Body->ReducedRagdoll = this; Body->RagdollLink = I;
			Body->CollisionGroup = this; Body->EnableCollisionWithinGroup = Settings.SelfCollision;
		}
		Synchronize(true); Wakeup();
	}
	const FeatherstoneSolver& Ragdoll::GetDynamics() const
	{
		if (!m_Dynamics) throw std::logic_error("Joint-constraint ragdolls have no reduced-coordinate dynamics");
		return *m_Dynamics;
	}
	Ragdoll::~Ragdoll()
	{
		DetachReducedBodies();
	}
	void Ragdoll::DetachReducedBodies()
	{
		if (!IsReduced() || m_Saved.size() != m_Bodies.size()) return;
		for (size_t I = 0; I < m_Bodies.size(); ++I)
		{
			auto* Body = m_Bodies[I]; const auto& Saved = m_Saved[I];
			for (auto* Shape : Body->Geometries()) Shape->GetTransform()->pos += Saved.Center - Body->CM;
			Body->CM = Saved.Center; Body->InvMass = Saved.InvMass; Body->InvInertia = Saved.InvInertia;
			Body->CollisionGroup = Saved.Group; Body->EnableCollisionWithinGroup = Saved.WithinGroup;
			Body->ReducedRagdoll = nullptr; Body->RagdollLink = 0; Body->UpdateGeometries();
		}
		m_Saved.clear();
	}
	void Ragdoll::Synchronize(bool Poses)
	{
		for (size_t I = 0; I < m_Bodies.size(); ++I)
		{
			auto* Body = m_Bodies[I]; const auto Frame = m_Dynamics->GetLinkTransform(I);
			if (Poses) { Body->X = m_Dynamics->GetCenterOfMass(I); Body->Q = Frame.quat; Body->UpdateGeometries(); }
			const auto Motion = m_Dynamics->GetLinkVelocity(I);
			Body->W = Motion.GetAngular(); Body->V = Motion.GetLinear() + Body->W.Cross(Body->X - Frame.pos);
			Body->SolverV = Body->V; Body->SolverW = Body->W;
		}
	}
	void Ragdoll::SetConfiguration(const std::vector<RagdollConfiguration>& Value)
	{
		m_Dynamics->SetConfiguration(Value); Synchronize(true); Wakeup();
		for (auto* Body : m_Bodies) m_World->UpdateSceneQuery(Body);
	}
	void Ragdoll::SetVelocity(const std::vector<double>& Value)
	{
		m_Dynamics->SetVelocity(Value); Synchronize(false); Wakeup();
	}
	void Ragdoll::SetJointForces(const std::vector<double>& Value)
	{
		if (Value.size() != m_Forces.size() || std::any_of(Value.begin(), Value.end(), [](double X) { return !std::isfinite(X); }))
			throw std::invalid_argument("Invalid generalized ragdoll force");
		m_Forces = Value; Wakeup();
	}
	void Ragdoll::SetControl(size_t Dof, const RagdollControl& Value)
	{
		if (Dof >= m_Controls.size() || !Value.Drive.IsValid() || !std::isfinite(Value.Lower)
			|| !std::isfinite(Value.Upper) || (Value.LimitEnabled && Value.Lower > Value.Upper))
			throw std::invalid_argument("Invalid reduced ragdoll control");
		if (m_Dynamics->GetLinkDofCount(m_CoordinateLinks[Dof]) != 1 && (Value.LimitEnabled || Value.Drive.Settings.PositionEnabled))
			throw std::invalid_argument("Position targets and limits require a scalar reduced ragdoll joint");
		m_Controls[Dof] = Value; Wakeup();
	}
	RagdollMechanism* Ragdoll::CreateMechanism(const RagdollMechanismSettings& Settings)
	{
		std::unique_ptr<RagdollMechanism> Row(new RagdollMechanism(*this, Settings));
		auto* Result = Row.get(); m_Mechanisms.push_back(std::move(Row)); Wakeup(); return Result;
	}
	bool Ragdoll::RemoveMechanism(RagdollMechanism* Mechanism)
	{
		const auto Found = std::find_if(m_Mechanisms.begin(), m_Mechanisms.end(), [&](const std::unique_ptr<RagdollMechanism>& Row) { return Row.get() == Mechanism; });
		if (Found == m_Mechanisms.end()) return false;
		m_Mechanisms.erase(Found); Wakeup(); return true;
	}
	bool Ragdoll::HasActiveDrive() const
	{
		return std::any_of(m_Controls.begin(), m_Controls.end(), [](const RagdollControl& C) { return C.Drive.Settings.IsActive(); });
	}
	bool Ragdoll::IsSleeping() const { return m_Bodies.front()->Sleeping; }
	bool Ragdoll::IsFrozen() const { return m_Bodies.front()->Freezing; }
	void Ragdoll::Wakeup()
	{
		for (auto* Body : m_Bodies) { ++Body->m_WakeRevision; if (!Body->Freezing) Body->Sleeping = false; }
	}
	void Ragdoll::Sleep()
	{
		m_Dynamics->SetVelocity(std::vector<double>(m_Controls.size()));
		for (auto* Body : m_Bodies) Body->Sleeping = true;
		Synchronize(false);
	}
	void Ragdoll::SetFrozen(bool Frozen)
	{
		for (auto* Body : m_Bodies) Body->Freezing = Frozen;
		if (Frozen) Sleep(); else Wakeup();
	}
	std::vector<SpatialVector> Ragdoll::ComputeResponse(const std::vector<SpatialVector>& Wrenches)
	{
		if (IsFrozen()) return std::vector<SpatialVector>(m_Bodies.size());
		return m_Dynamics->ComputeLinkVelocities(m_Dynamics->ComputeImpulseResponse(Wrenches));
	}
	void Ragdoll::ApplyImpulse(size_t Link, const Vector3& Point, const Vector3& Impulse)
	{
		const auto Origin = m_Dynamics->GetLinkTransform(Link).pos;
		std::vector<SpatialVector> Wrenches(m_Bodies.size()); Wrenches.at(Link) = SpatialVector((Point - Origin).Cross(Impulse), Impulse);
		const auto Response = m_Dynamics->ComputeImpulseResponse(Wrenches);
		if (IsFrozen()) return;
		auto Velocity = m_Dynamics->GetVelocity();
		for (size_t I = 0; I < Velocity.size(); ++I) Velocity[I] += Response[I];
		SetVelocity(Velocity);
	}
	void Ragdoll::ImportVelocities(const RagdollVelocityContext& Context)
	{
		std::vector<double> Velocity(m_Controls.size());
		for (size_t I = 0; I < m_Bodies.size(); ++I)
		{
			const auto& Link = m_Dynamics->GetLinkSettings(I); const auto Frame = m_Dynamics->GetLinkTransform(I);
			const auto Child = Context.Read(m_Bodies[I]);
			Vector3 W = Child.w, V = Child.v - W.Cross(m_Bodies[I]->X - Frame.pos);
			if (Link.Parent >= 0)
			{
				auto* Parent = m_Bodies[Link.Parent]; const auto Motion = Context.Read(Parent);
				V -= Motion.v + Motion.w.Cross(Frame.pos - Parent->X); W -= Motion.w;
			}
			W = Frame.quat.Conjugate() * W; V = Frame.quat.Conjugate() * V;
			const int Offset = m_Dynamics->GetLinkDofOffset(I), Count = m_Dynamics->GetLinkDofCount(I);
			if (Link.Joint == RagdollReducedJointType::Revolute) Velocity[Offset] = W.Dot(Link.Axis);
			else if (Link.Joint == RagdollReducedJointType::Prismatic) Velocity[Offset] = V.Dot(Link.Axis);
			else if (Count > 1) for (int D = 0; D < Count; ++D) Velocity[Offset + D] = D < 3 ? W[D] : V[D - 3];
		}
		m_Dynamics->SetVelocity(Velocity); Synchronize(false);
	}
	double Ragdoll::Coordinate(size_t Dof) const
	{
		const size_t Link = m_CoordinateLinks[Dof];
		return m_Dynamics->GetLinkDofCount(Link) == 1 ? m_Dynamics->GetConfiguration()[Link].Position : 0;
	}
	void Ragdoll::BeginStep(float Dt, bool Drives)
	{
		m_Drives = Drives;
		if ((Drives && HasActiveDrive()) || std::any_of(m_Forces.begin(), m_Forces.end(), [](double X) { return X != 0; })) Wakeup();
		std::vector<SpatialVector> Wrenches(m_Bodies.size());
		for (size_t I = 0; I < m_Bodies.size(); ++I)
		{
			auto* Body = m_Bodies[I]; const Vector3 Offset = Body->X - m_Dynamics->GetLinkTransform(I).pos;
			Wrenches[I] = SpatialVector(Body->ExtTorque + Offset.Cross(Body->ExtForce), Body->ExtForce);
			Body->ExtForce.SetZero(); Body->ExtTorque.SetZero();
		}
		for (auto& Row : m_Rows) { Row.Response.clear(); Row.DriveImpulse = Row.LowerImpulse = Row.UpperImpulse = 0; }
		for (auto& Row : m_ConeRows) { Row.Gradient.clear(); Row.Response.clear(); Row.Impulse = 0; }
		for (auto& Row : m_Mechanisms) Row->BeginStep(Dt);
		if (IsSleeping() || IsFrozen()) return;
		m_Dynamics->IntegrateVelocity(m_Dynamics->ComputeAcceleration(m_Forces, Vector3::Zero(), Wrenches), Dt);
		Synchronize(false);
		UpdateControlResponses();
		RagdollVelocityContext Context;
		for (int Iteration = 0; Iteration < m_Settings.ControlIterations; ++Iteration) SolveControls(Context, Dt);
	}
	void Ragdoll::UpdateControlResponses()
	{
		for (size_t I = 0; I < m_Controls.size(); ++I)
			if (m_Controls[I].LimitEnabled || (m_Drives && m_Controls[I].Drive.Settings.IsActive()))
			{
				std::vector<double> Unit(m_Controls.size()); Unit[I] = 1;
				m_Rows[I].Response = m_Dynamics->ComputeImpulseResponse({}, Unit);
			}
		for (size_t I = 1; I < m_Bodies.size(); ++I)
		{
			const auto& Link = m_Dynamics->GetLinkSettings(I);
			if (Link.ConeLimit >= 3.14159265358979323846) continue;
			const Vector3 A = (m_Dynamics->GetLinkTransform((size_t)Link.Parent).quat * Link.ConeAxisParent).SafeUnit();
			const Vector3 B = (m_Dynamics->GetLinkTransform(I).quat * Link.ConeAxisChild).SafeUnit();
			auto Prepare = [&](ConeRow& Row, const Vector3& Axis)
			{
				std::vector<SpatialVector> Wrenches(m_Bodies.size());
				Wrenches[(size_t)Link.Parent] = SpatialVector(-Axis, Vector3::Zero());
				Wrenches[I] = SpatialVector(Axis, Vector3::Zero());
				Row.Gradient = m_Dynamics->ComputeGeneralizedForce(Wrenches);
				Row.Response = m_Dynamics->ComputeImpulseResponse({}, Row.Gradient);
				Row.Weight = 0; for (size_t D = 0; D < Row.Gradient.size(); ++D) Row.Weight += Row.Gradient[D] * Row.Response[D];
			};
			if (Link.ConeLimit == 0)
			{
				Vector3 P, Q; A.GetPerpVectors(P, Q);
				const Vector3 Error = A.Cross(B);
				for (int Side = 0; Side < 2; ++Side)
				{
					auto& Row = m_ConeRows[2*I+(size_t)Side]; const Vector3 Axis = Side == 0 ? P : Q;
					Row.Bilateral = true; Row.Angle = Error.Dot(Axis); Prepare(Row, Axis);
				}
				continue;
			}
			auto& Row = m_ConeRows[2*I]; Row.Bilateral = false;
			Row.Angle = std::acos(std::max(-1.0, std::min(1.0, (double)A.Dot(B))));
			Vector3 Axis = A.Cross(B);
			if (Axis.SquareLength() < 1e-12f)
			{
				if (Row.Angle < Link.ConeLimit) { Row.Gradient.clear(); Row.Response.clear(); continue; }
				Vector3 Other; A.GetPerpVectors(Axis, Other);
			}
			Axis.Normalize();
			Prepare(Row, Axis);
		}
		for (auto& Row : m_Mechanisms) Row->Prepare();
	}
	void Ragdoll::SolveControls(RagdollVelocityContext& Context, float Dt)
	{
		if (IsSleeping() || IsFrozen() || Dt <= 0) return;
		if (std::all_of(m_Rows.begin(), m_Rows.end(), [](const ControlRow& Row) { return Row.Response.empty(); })
			&& std::all_of(m_ConeRows.begin(), m_ConeRows.end(), [](const ConeRow& Row) { return Row.Response.empty(); })
			&& std::all_of(m_Mechanisms.begin(), m_Mechanisms.end(), [](const std::unique_ptr<RagdollMechanism>& Row) { return Row->m_Response.empty(); })) return;
		ImportVelocities(Context); auto Velocity = m_Dynamics->GetVelocity();
		for (size_t I = 0; I < m_Controls.size(); ++I)
		{
			auto& Row = m_Rows[I]; if (Row.Response.empty()) continue;
			const double Weight = Row.Response[I]; if (Weight <= m_Settings.MinimumResponse) continue;
			const auto& Control = m_Controls[I]; const auto& Drive = Control.Drive.Settings;
			auto Apply = [&](double Delta) { for (size_t J = 0; J < Velocity.size(); ++J) Velocity[J] += Row.Response[J] * Delta; };
			const double Position = Coordinate(I);
			if (m_Drives && Drive.IsActive())
			{
				const double Scale = Drive.Acceleration ? 1 / Weight : 1;
				const double K = Drive.PositionEnabled ? Drive.Stiffness * Scale : 0, D = Drive.Damping * Scale;
				const double TargetV = Drive.VelocityEnabled ? Control.Drive.TargetVelocity : 0;
				const double Load = K * (Control.Drive.TargetPosition - Position) + D * TargetV;
				const double Feedback = D + Dt * K;
				const double Delta = (Dt * (Load - Feedback * Velocity[I]) - Row.DriveImpulse) / (1 + Dt * Feedback * Weight);
				const double Cap = double(Drive.MaxForce) * Dt, Previous = Row.DriveImpulse;
				Row.DriveImpulse = std::max(-Cap, std::min(Cap, Previous + Delta)); Apply(Row.DriveImpulse - Previous);
			}
			if (Control.LimitEnabled)
			{
				const double PreviousLower = Row.LowerImpulse;
				Row.LowerImpulse = std::max(0.0, PreviousLower + ((Control.Lower - Position) / Dt - Velocity[I]) / Weight);
				Apply(Row.LowerImpulse - PreviousLower);
				const double PreviousUpper = Row.UpperImpulse;
				Row.UpperImpulse = std::max(0.0, PreviousUpper + (Velocity[I] - (Control.Upper - Position) / Dt) / Weight);
				Apply(PreviousUpper - Row.UpperImpulse);
			}
		}
		for (size_t I = 1; I < m_Bodies.size(); ++I) for (int Side = 0; Side < 2; ++Side)
		{
			auto& Row = m_ConeRows[2*I+(size_t)Side]; if (Row.Response.empty() || Row.Weight <= m_Settings.MinimumResponse) continue;
			const double Limit = m_Dynamics->GetLinkSettings(I).ConeLimit;
			double Rate = 0; for (size_t D = 0; D < Velocity.size(); ++D) Rate += Row.Gradient[D] * Velocity[D];
			const double Previous = Row.Impulse;
			const double Delta = ((Row.Bilateral ? -Row.Angle : Limit - Row.Angle) / Dt - Rate) / Row.Weight;
			Row.Impulse = Row.Bilateral ? Previous + Delta : std::min(0.0, Previous + Delta);
			for (size_t D = 0; D < Velocity.size(); ++D) Velocity[D] += Row.Response[D] * (Row.Impulse - Previous);
		}
		for (auto& Row : m_Mechanisms) Row->Solve(Velocity);
		m_Dynamics->SetVelocity(Velocity); Synchronize(false); Context.WriteFromBodies(this);
	}
	void Ragdoll::IntegratePoses(float Dt)
	{
		if (IsSleeping() || IsFrozen()) return;
		m_Dynamics->IntegrateConfiguration(Dt); Synchronize(true);
		// The optional post-projection contact pass runs at these new poses.
		// Refresh its response columns while retaining this substep's force caps.
		UpdateControlResponses();
	}
}

namespace Riemann
{
	/* A mechanism adds a scalar relation to a reduced ragdoll's joint space. It
	   closes a kinematic loop without changing the tree used by Featherstone ABA:
	     C(q) = coordinate(q) - target, J = dC/dq, w = J H^-1 J^T.
	   A row impulse changes every joint by H^-1 J^T lambda. Sum coefficients or
	   attachment wrenches BEFORE computing this response; independent link masses
	   miss the coupling within the tree. Responses are cached at each configuration
	   and iterated with joint drives, limits, contact normals and friction.

	   Joint couplings include mimic (qFollower-ratio*qReference=offset), gears
	   (qA+ratio*qB=phase), rack/pinion (qSlider-radius*qPinion=offset), and fixed
	   tendons (sum coefficient*q=rest). Angles are unwrapped radians. Gear signs,
	   radii and joint axes are authoring choices; no tooth collision is generated.
	   Scalar terms require hinges or sliders. Separate branches of one fixed-base
	   tree can represent multiple shafts; coupling separate ragdolls is not
	   supported by this owner-local API.

	   A spatial tendon follows an ordered polyline. Link points use joint-frame
	   coordinates; Link=-1 denotes a world anchor. Each segment contributes its
	   length and equal/opposite unit forces at its ends. Their moments about joint
	   origins map through the wrench transpose to the length gradient in O(links +
	   attachments). A guide point receives both neighboring contributions. Repeated
	   points below MinimumSegmentLength have zero derivative at the cusp. This is
	   an ideal cable over authored guides, not automatic pulley wrapping or sliding
	   surface contact. Multiple rows can share joints or guides to form a network.

	   Hard rows use bounded velocity bias. TensionOnly enforces C<=0 and lambda<=0;
	   a slack cable allows extension up to its remaining length within the substep.
	   Compliant rows use backward Euler for F=-k*C-d*Cdot. The accumulated impulse
	   term in the residual prevents solver iterations from applying extra force.
	   A compliant cable is inactive while its predicted free length remains slack.
	   MaxForce caps the TOTAL impulse per substep, including repeated contact passes.
	   Positive reported force increases coordinate; cable tension is its negative.
	   Settings changes wake the whole owner; passive rows do not defeat sleeping.
	   Reference: Featherstone (2008), constraint forces in joint space. */
	namespace
	{
		bool Finite(const Vector3& V) { return std::isfinite(V.x) && std::isfinite(V.y) && std::isfinite(V.z); }
		double Dot(const std::vector<double>& A, const std::vector<double>& B)
		{
			double Result = 0; for (size_t I = 0; I < A.size(); ++I) Result += A[I] * B[I]; return Result;
		}
		double Clamp(double Value, double Lower, double Upper) { return std::max(Lower, std::min(Upper, Value)); }
	}
	RagdollMechanismSettings RagdollMechanismSettings::Mimic(size_t Reference, size_t Follower, double Ratio, double Offset)
	{
		RagdollMechanismSettings S; S.Joints = { { Reference, -Ratio }, { Follower, 1 } }; S.Target = Offset; return S;
	}
	RagdollMechanismSettings RagdollMechanismSettings::Gear(size_t A, size_t B, double Ratio, double Phase)
	{
		RagdollMechanismSettings S; S.Joints = { { A, 1 }, { B, Ratio } }; S.Target = Phase; return S;
	}
	RagdollMechanismSettings RagdollMechanismSettings::Rack(size_t Pinion, size_t Slider, double Radius, double Offset)
	{
		return Mimic(Pinion, Slider, Radius, Offset);
	}
	RagdollMechanism::RagdollMechanism(Ragdoll& Owner, const RagdollMechanismSettings& Settings) : m_Owner(Owner)
	{
		Validate(Settings); m_Settings = Settings;
	}
	void RagdollMechanism::Validate(const RagdollMechanismSettings& S) const
	{
		if (!std::isfinite(S.Target) || !std::isfinite(S.Stiffness) || S.Stiffness < 0
			|| !std::isfinite(S.Damping) || S.Damping < 0 || !std::isfinite(S.MaxForce) || S.MaxForce < 0
			|| !std::isfinite(S.ErrorReduction) || S.ErrorReduction < 0 || S.ErrorReduction > 1
			|| !std::isfinite(S.MaxCorrectionSpeed) || S.MaxCorrectionSpeed <= 0
			|| !std::isfinite(S.MinimumSegmentLength) || S.MinimumSegmentLength <= 0)
			throw std::invalid_argument("Invalid reduced ragdoll mechanism response settings");
		const auto& Model = m_Owner.GetDynamics();
		if (S.Type == RagdollMechanismType::JointCoupling)
		{
			if (S.Joints.empty() || !S.Path.empty()) throw std::invalid_argument("Joint coupling requires scalar joint terms only");
			std::vector<double> Coefficients(Model.GetDofCount());
			for (const auto& Term : S.Joints)
			{
				if (Term.Link >= Model.GetLinkCount() || Model.GetLinkDofCount(Term.Link) != 1 || !std::isfinite(Term.Coefficient))
					throw std::invalid_argument("Mechanism terms require finite coefficients and scalar reduced ragdoll joints");
				Coefficients[Model.GetLinkDofOffset(Term.Link)] += Term.Coefficient;
			}
			if (std::any_of(Coefficients.begin(), Coefficients.end(), [](double C) { return !std::isfinite(C); })
				|| std::all_of(Coefficients.begin(), Coefficients.end(), [](double C) { return C == 0; }))
				throw std::invalid_argument("Mechanism joint coefficients cancel or overflow");
		}
		else if (S.Type == RagdollMechanismType::SpatialTendon)
		{
			if (!S.Joints.empty() || S.Path.size() < 2 || S.Target < 0 || Model.GetDofCount() == 0)
				throw std::invalid_argument("Spatial tendon requires a path, movable tree and nonnegative rest length");
			bool HasLink = false;
			for (const auto& Point : S.Path)
			{
				if (Point.Link < -1 || (Point.Link >= 0 && (size_t)Point.Link >= Model.GetLinkCount()) || !Finite(Point.Point))
					throw std::invalid_argument("Invalid reduced ragdoll tendon attachment");
				HasLink |= Point.Link >= 0;
			}
			if (!HasLink) throw std::invalid_argument("Spatial tendon requires a reduced ragdoll attachment");
		}
		else throw std::invalid_argument("Invalid reduced ragdoll mechanism type");
	}
	void RagdollMechanism::SetSettings(const RagdollMechanismSettings& Settings)
	{
		Validate(Settings); m_Settings = Settings; m_Gradient.clear(); m_Response.clear(); m_Impulse = 0; m_Owner.Wakeup();
	}
	double RagdollMechanism::Evaluate(std::vector<double>* Gradient) const
	{
		const auto& Model = m_Owner.GetDynamics(); double Coordinate = 0;
		if (Gradient) Gradient->assign(Model.GetDofCount(), 0);
		if (m_Settings.Type == RagdollMechanismType::JointCoupling)
		{
			for (const auto& Term : m_Settings.Joints)
			{
				Coordinate += Term.Coefficient * Model.GetConfiguration()[Term.Link].Position;
				if (Gradient) (*Gradient)[Model.GetLinkDofOffset(Term.Link)] += Term.Coefficient;
			}
		}
		else
		{
			std::vector<Vector3> Points; Points.reserve(m_Settings.Path.size());
			for (const auto& P : m_Settings.Path)
				Points.push_back(P.Link < 0 ? P.Point : Model.GetLinkTransform(P.Link).LocalToWorld(P.Point));
			std::vector<SpatialVector> Wrenches(Gradient ? Model.GetLinkCount() : 0);
			for (size_t I = 1; I < Points.size(); ++I)
			{
				const Vector3 Delta = Points[I] - Points[I - 1]; const double Length = Delta.Length(); Coordinate += Length;
				if (!Gradient || Length <= m_Settings.MinimumSegmentLength) continue;
				const Vector3 Direction = Delta / (float)Length;
				for (size_t End : { I - 1, I })
				{
					const auto& P = m_Settings.Path[End]; if (P.Link < 0) continue;
					const auto Frame = Model.GetLinkTransform(P.Link); const Vector3 Force = End == I ? Direction : -Direction;
					Wrenches[P.Link] += SpatialVector((Frame.quat * P.Point).Cross(Force), Force);
				}
			}
			if (Gradient) *Gradient = Model.ComputeGeneralizedForce(Wrenches);
		}
		if (!std::isfinite(Coordinate)) throw std::runtime_error("Nonfinite reduced ragdoll mechanism coordinate");
		return Coordinate;
	}
	double RagdollMechanism::GetCoordinate() const { return Evaluate(nullptr); }
	std::vector<double> RagdollMechanism::GetGradient() const { std::vector<double> Gradient; Evaluate(&Gradient); return Gradient; }
	double RagdollMechanism::GetRate() const { return Dot(GetGradient(), m_Owner.GetDynamics().GetVelocity()); }
	void RagdollMechanism::BeginStep(double Dt)
	{
		m_Dt = Dt; m_Impulse = m_Weight = 0; m_Gradient.clear(); m_Response.clear();
	}
	void RagdollMechanism::Prepare()
	{
		if (!m_Settings.Enabled || m_Settings.MaxForce == 0) return;
		m_Coordinate = Evaluate(&m_Gradient);
		m_Response = m_Owner.m_Dynamics->ComputeImpulseResponse({}, m_Gradient);
		m_Weight = Dot(m_Gradient, m_Response);
		if (!std::isfinite(m_Weight)) throw std::runtime_error("Ragdoll mechanism response overflow");
	}
	void RagdollMechanism::Solve(std::vector<double>& Velocity)
	{
		if (m_Response.empty() || m_Dt <= 0 || m_Weight <= m_Owner.GetSettings().MinimumResponse) return;
		const double Error = m_Coordinate - m_Settings.Target, Rate = Dot(m_Gradient, Velocity);
		double Delta = 0;
		if (m_Settings.Compliant)
		{
			const double Feedback = m_Settings.Damping + m_Dt * m_Settings.Stiffness;
			const double FreeRate = Rate - m_Weight * m_Impulse;
			if (m_Settings.TensionOnly && Error + m_Dt * FreeRate <= 0) Delta = -m_Impulse;
			else Delta = (m_Dt * (-m_Settings.Stiffness * Error - Feedback * Rate) - m_Impulse)
				/ (1 + m_Dt * Feedback * m_Weight);
		}
		else
		{
			const double Bias = m_Settings.TensionOnly && Error < 0 ? -Error / m_Dt
				: Clamp(-m_Settings.ErrorReduction * Error / m_Dt, -m_Settings.MaxCorrectionSpeed, m_Settings.MaxCorrectionSpeed);
			Delta = (Bias - Rate) / m_Weight;
		}
		if (!std::isfinite(Delta)) throw std::runtime_error("Ragdoll mechanism impulse overflow");
		const double Maximum = std::numeric_limits<double>::max();
		const double Cap = m_Settings.MaxForce > Maximum / m_Dt ? Maximum : m_Settings.MaxForce * m_Dt, Previous = m_Impulse;
		m_Impulse = Clamp(Previous + Delta, -Cap, m_Settings.TensionOnly ? 0 : Cap);
		for (size_t I = 0; I < Velocity.size(); ++I) Velocity[I] += m_Response[I] * (m_Impulse - Previous);
	}
}

namespace Riemann
{
	/* A contact row's response spans all links of either endpoint's reduced ragdoll.
	   When both endpoints belong to one tree, combine their wrenches BEFORE the
	   ABA solve: summing independent endpoint inverse masses loses cross terms.
	   Cached velocity changes target the solver's working buffer. Configuration is
	   fixed during a velocity sweep, so these columns remain valid across iterations.
	   Commit reconstructs generalized velocities from relative parent/child motion;
	   no independent link integration or post-hoc joint projection is performed. */
	void RagdollImpulseResponse::Apply(float Impulse) const
	{
		for (const auto& C : Changes) { C.Velocity->v += C.Linear * Impulse; C.Velocity->w += C.Angular * Impulse; }
	}
	void RagdollVelocityContext::Begin(const std::vector<RigidBody*>& Bodies, GeneralizedVelocity* Velocities)
	{
		if (!Bodies.empty() && !Velocities) throw std::invalid_argument("Ragdoll context requires a velocity buffer");
		m_Velocities.clear(); m_Ragdolls.clear();
		for (size_t I = 0; I < Bodies.size(); ++I)
		{
			m_Velocities[Bodies[I]] = Velocities + I;
			if (Bodies[I]->ReducedRagdoll && std::find(m_Ragdolls.begin(), m_Ragdolls.end(), Bodies[I]->ReducedRagdoll) == m_Ragdolls.end())
				m_Ragdolls.push_back(Bodies[I]->ReducedRagdoll);
		}
		for (auto* Owner : m_Ragdolls) for (auto* Link : Owner->GetBodies())
			if (!m_Velocities.count(Link)) throw std::invalid_argument("Ragdoll context must contain every link");
	}
	GeneralizedVelocity RagdollVelocityContext::Read(RigidBody* Body) const
	{
		if (!Body) return { Vector3::Zero(), Vector3::Zero() };
		const auto Found = m_Velocities.find(Body);
		return Found == m_Velocities.end() ? GeneralizedVelocity{ Body->V, Body->W } : *Found->second;
	}
	double RagdollVelocityContext::Project(const RagdollImpulseResponse& Response, RigidBody* A,
		const Vector3& LA, const Vector3& AA, RigidBody* B, const Vector3& LB, const Vector3& AB) const
	{
		double Result = 0;
		RigidBody* Bodies[] = { A, B }; const Vector3 Linear[] = { LA, LB }, Angular[] = { AA, AB };
		for (int E = 0; E < 2; ++E)
		{
			const auto Slot = m_Velocities.find(Bodies[E]); if (Slot == m_Velocities.end()) continue;
			for (const auto& Change : Response.Changes) if (Change.Velocity == Slot->second)
				Result += Linear[E].Dot(Change.Linear) + Angular[E].Dot(Change.Angular);
		}
		return Result;
	}
	double RagdollVelocityContext::Response(RigidBody* A, const Vector3& LA, const Vector3& AA,
		RigidBody* B, const Vector3& LB, const Vector3& AB, RagdollImpulseResponse& Out) const
	{
		Out.Changes.clear(); double Weight = 0;
		RigidBody* Bodies[] = { A, B }; const Vector3 Linear[] = { LA, LB }, Angular[] = { AA, AB };
		for (int Endpoint = 0; Endpoint < 2; ++Endpoint)
		{
			auto* Body = Bodies[Endpoint]; auto* Dynamic = Body ? Body->CastDynamic() : nullptr;
			if (!Dynamic || Dynamic->Freezing) continue;
			if (!Body->ReducedRagdoll)
			{
				const Vector3 DV = Linear[Endpoint] * Body->InvMass, DW = Body->GetInverseInertia_WorldSpace() * Angular[Endpoint];
				const auto Found = m_Velocities.find(Body);
				if (Found != m_Velocities.end()) Out.Changes.push_back({ Found->second, DV, DW });
				Weight += Linear[Endpoint].Dot(DV) + Angular[Endpoint].Dot(DW); continue;
			}
			auto* Owner = Body->ReducedRagdoll;
			if (Endpoint == 1 && A && A->ReducedRagdoll == Owner) continue;
			std::vector<SpatialVector> Wrenches(Owner->GetBodies().size());
			for (int E = 0; E < 2; ++E) if (Bodies[E] && Bodies[E]->ReducedRagdoll == Owner)
			{
				const size_t Link = Bodies[E]->RagdollLink;
				const Vector3 Origin = Owner->GetDynamics().GetLinkTransform(Link).pos;
				Wrenches[Link] += SpatialVector(Angular[E] + (Bodies[E]->X - Origin).Cross(Linear[E]), Linear[E]);
			}
			const auto Changes = Owner->ComputeResponse(Wrenches);
			for (size_t I = 0; I < Changes.size(); ++I)
			{
				auto* LinkBody = Owner->GetBodies()[I]; const Vector3 Origin = Owner->GetDynamics().GetLinkTransform(I).pos;
				const Vector3 DW = Changes[I].GetAngular(), DV = Changes[I].GetLinear() + DW.Cross(LinkBody->X - Origin);
				const auto Found = m_Velocities.find(LinkBody);
				if (Found != m_Velocities.end()) Out.Changes.push_back({ Found->second, DV, DW });
				for (int E = 0; E < 2; ++E) if (Bodies[E] == LinkBody) Weight += Linear[E].Dot(DV) + Angular[E].Dot(DW);
			}
		}
		return Weight;
	}
	void RagdollVelocityContext::WriteFromBodies(Ragdoll* Owner)
	{
		for (auto* Body : Owner->GetBodies()) { const auto It = m_Velocities.find(Body); if (It != m_Velocities.end()) *It->second = { Body->V, Body->W }; }
	}
	void RagdollVelocityContext::SolveControls(float Dt, const std::vector<RigidBody*>& IslandBodies)
	{
		for (auto* Owner : m_Ragdolls)
			if (std::any_of(IslandBodies.begin(), IslandBodies.end(), [&](RigidBody* B) { return B->ReducedRagdoll == Owner; })) Owner->SolveControls(*this, Dt);
	}
	void RagdollVelocityContext::Commit()
	{
		for (auto* Owner : m_Ragdolls) Owner->ImportVelocities(*this);
	}
}
