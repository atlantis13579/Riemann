#pragma once

#include <string>
#include <vector>
#include <memory>
#include <limits>
#include <unordered_map>
#include "FeatherstoneSolver.h"
#include "Constraints.h"

#include "../Maths/Transform.h"

namespace Riemann
{
	class Geometry;
	class PhysicsWorld;
	class PositionConstraint;
	class RigidBodyDynamic;
	class RigidBody;

	enum class RagdollBone
	{
		Pelvis,
		Torso,
		Head,
		LeftUpperArm,
		LeftLowerArm,
		RightUpperArm,
		RightLowerArm,
		LeftUpperLeg,
		LeftLowerLeg,
		RightUpperLeg,
		RightLowerLeg,
		Count,
	};

	enum class RagdollAlgorithm { JointConstraints, ReducedCoordinates };

	struct RagdollSettings
	{
		RagdollSettings();

		float Scale;
		float LinearDamping;
		float AngularDamping;
		Vector3 LinearVelocity;
		Vector3 AngularVelocity;
		float JointVelocity;
		bool EnableCCD;
		bool DisableSelfCollision;
		std::string DefinitionFile;
		RagdollAlgorithm Algorithm = RagdollAlgorithm::JointConstraints;
		std::vector<RagdollLinkSettings> Links;
		FeatherstoneSettings Numerics;
		bool SelfCollision = false;
		int ControlIterations = 16;
		double MinimumResponse = 1e-12;
		static RagdollSettings Reduced();
	};

	struct RagdollControl
	{
		ScalarJointDrive Drive;
		bool LimitEnabled = false;
		double Lower = 0, Upper = 0;
	};

	class Ragdoll;
	struct RagdollJointTerm
	{
		size_t Link = 0;
		double Coefficient = 1;
	};
	struct RagdollTendonPoint
	{
		int Link = -1;
		Vector3 Point = Vector3::Zero();
	};
	enum class RagdollMechanismType { JointCoupling, SpatialTendon };
	struct RagdollMechanismSettings
	{
		RagdollMechanismType Type = RagdollMechanismType::JointCoupling;
		std::vector<RagdollJointTerm> Joints;
		std::vector<RagdollTendonPoint> Path;
		double Target = 0;
		bool Enabled = true;
		bool TensionOnly = false;
		bool Compliant = false;
		double Stiffness = 1000, Damping = 20;
		double MaxForce = std::numeric_limits<double>::max();
		double ErrorReduction = 0.2, MaxCorrectionSpeed = 10;
		double MinimumSegmentLength = 1e-6;
		static RagdollMechanismSettings Mimic(size_t Reference, size_t Follower, double Ratio, double Offset = 0);
		static RagdollMechanismSettings Gear(size_t A, size_t B, double Ratio, double Phase = 0);
		static RagdollMechanismSettings Rack(size_t Pinion, size_t Slider, double Radius, double Offset = 0);
	};
	class RagdollMechanism
	{
	public:
		RagdollMechanism(const RagdollMechanism&) = delete;
		RagdollMechanism& operator=(const RagdollMechanism&) = delete;
		const RagdollMechanismSettings& GetSettings() const { return m_Settings; }
		void SetSettings(const RagdollMechanismSettings& Settings);
		double GetCoordinate() const;
		double GetRate() const;
		std::vector<double> GetGradient() const;
		double GetImpulse() const { return m_Impulse; }
		double GetForce() const { return m_Dt > 0 ? m_Impulse / m_Dt : 0; }
	private:
		friend class Ragdoll;
		RagdollMechanism(Ragdoll& Owner, const RagdollMechanismSettings& Settings);
		void Validate(const RagdollMechanismSettings& Settings) const;
		double Evaluate(std::vector<double>* Gradient) const;
		void BeginStep(double Dt);
		void Prepare();
		void Solve(std::vector<double>& Velocity);
		Ragdoll& m_Owner;
		RagdollMechanismSettings m_Settings;
		std::vector<double> m_Gradient, m_Response;
		double m_Coordinate = 0, m_Weight = 0, m_Impulse = 0, m_Dt = 0;
	};
	struct GeneralizedVelocity;
	struct RagdollImpulseResponse
	{
		struct Change { GeneralizedVelocity* Velocity; Vector3 Linear, Angular; };
		std::vector<Change> Changes;
		void Apply(float Impulse) const;
	};
	class RagdollVelocityContext
	{
	public:
		void Begin(const std::vector<RigidBody*>& Bodies, GeneralizedVelocity* Velocities);
		double Response(RigidBody* A, const Vector3& LinearA, const Vector3& AngularA,
			RigidBody* B, const Vector3& LinearB, const Vector3& AngularB, RagdollImpulseResponse& Out) const;
		GeneralizedVelocity Read(RigidBody* Body) const;
		double Project(const RagdollImpulseResponse& Response, RigidBody* A, const Vector3& LinearA, const Vector3& AngularA,
			RigidBody* B, const Vector3& LinearB, const Vector3& AngularB) const;
		void WriteFromBodies(Ragdoll* Owner);
		void SolveControls(float Dt, const std::vector<RigidBody*>& IslandBodies);
		void Commit();
		bool HasReducedRagdolls() const { return !m_Ragdolls.empty(); }
	private:
		std::unordered_map<RigidBody*, GeneralizedVelocity*> m_Velocities;
		std::vector<Ragdoll*> m_Ragdolls;
	};

	class Ragdoll
	{
	public:
		Ragdoll(PhysicsWorld* world, const RagdollSettings& settings, const Transform& rootTransform);
		~Ragdoll();

		bool IsValid() const { return m_Valid; }
		RagdollAlgorithm GetAlgorithm() const { return m_Settings.Algorithm; }
		bool IsReduced() const { return GetAlgorithm() == RagdollAlgorithm::ReducedCoordinates; }
		const std::string& GetError() const { return m_Error; }
		void Destroy();
		RigidBodyDynamic* GetBone(RagdollBone bone) const;
		const std::vector<RigidBodyDynamic*>& GetBodies() const { return m_Bodies; }
		const std::vector<PositionConstraint*>& GetConstraints() const { return m_Constraints; }
		size_t GetBodyCount() const { return m_Bodies.size(); }
		size_t GetConstraintCount() const { return m_Constraints.size(); }

		const FeatherstoneSolver& GetDynamics() const;
		const RagdollSettings& GetSettings() const { return m_Settings; }
		const RagdollControl& GetControl(size_t Dof) const { return m_Controls.at(Dof); }
		void SetControl(size_t Dof, const RagdollControl& Control);
		void SetConfiguration(const std::vector<RagdollConfiguration>& Configuration);
		void SetVelocity(const std::vector<double>& Velocity);
		void SetJointForces(const std::vector<double>& Force);
		RagdollMechanism* CreateMechanism(const RagdollMechanismSettings& Settings);
		bool RemoveMechanism(RagdollMechanism* Mechanism);
		size_t GetMechanismCount() const { return m_Mechanisms.size(); }
		RagdollMechanism* GetMechanism(size_t Index) const { return m_Mechanisms.at(Index).get(); }
		void ApplyImpulse(size_t Link, const Vector3& WorldPoint, const Vector3& Impulse);
		void Wakeup();
		void Sleep();
		void SetFrozen(bool Frozen);
		bool IsSleeping() const;
		bool IsFrozen() const;
		bool HasActiveDrive() const;
		std::vector<SpatialVector> ComputeResponse(const std::vector<SpatialVector>& Wrenches);
		void ImportVelocities(const RagdollVelocityContext& Context);
		void SolveControls(RagdollVelocityContext& Context, float Dt);


	private:
		friend class PhysicsWorld;
		friend class RagdollMechanism;
		Ragdoll(PhysicsWorld& World, const RagdollSettings& Settings, const std::vector<RigidBodyDynamic*>& Bodies);
		void OnBodyRemoved(RigidBody* Body);
		void OnConstraintRemoved(PositionConstraint* Constraint);
		RigidBodyDynamic* CreateBoxBone(const Vector3& center, const Vector3& halfExtent, const Quaternion& rotation);
		RigidBodyDynamic* CreateCapsuleBone(const Vector3& start, const Vector3& end, float radius);
		RigidBodyDynamic* CreateSphereBone(const Vector3& center, float radius);
		Vector3 ToWorld(const Vector3& localPosition) const;
		Vector3 ToWorldDirection(const Vector3& localDirection) const;
		void Build();
		void BuildReduced(const struct RagdollDefinition& Definition);
		void AttachReducedBodies();
		void DetachReducedBodies();

	private:
		PhysicsWorld* m_World;
		RagdollSettings m_Settings;
		Transform m_RootTransform;
		std::vector<RigidBodyDynamic*> m_Bodies;
		std::vector<PositionConstraint*> m_Constraints;
		std::string m_Error;
		bool m_Valid;
		bool m_OwnsBodies = true;
		void BeginStep(float Dt, bool Drives);
		void IntegratePoses(float Dt);
		void Synchronize(bool Poses);
		void UpdateControlResponses();
		double Coordinate(size_t Dof) const;
		std::unique_ptr<FeatherstoneSolver> m_Dynamics;
		struct SavedBody { Vector3 Center; float InvMass; Matrix3 InvInertia; void* Group; bool WithinGroup; };
		std::vector<SavedBody> m_Saved;
		std::vector<RagdollControl> m_Controls;
		std::vector<size_t> m_CoordinateLinks;
		std::vector<double> m_Forces;
		std::vector<std::unique_ptr<RagdollMechanism>> m_Mechanisms;
		struct ControlRow { std::vector<double> Response; double DriveImpulse = 0, LowerImpulse = 0, UpperImpulse = 0; };
		std::vector<ControlRow> m_Rows;
		struct ConeRow { std::vector<double> Gradient, Response; double Angle = 0, Weight = 0, Impulse = 0; bool Bilateral = false; };
		std::vector<ConeRow> m_ConeRows;
		bool m_Drives = true;
	};
}
