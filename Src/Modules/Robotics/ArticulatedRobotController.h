#pragma once

#include "RobotController.h"
#include "../../RigidBodyDynamics/FeatherstoneSolver.h"

namespace Riemann
{
	enum class RobotDynamicsBackend { Featherstone, SparseJointSpaceDynamics };

	struct RobotFootContact
	{
		size_t Link;
		Vector3 Center;
		float Radius;
	};
	struct ArticulatedRobotSettings
	{
		RobotDynamicsBackend Backend = RobotDynamicsBackend::SparseJointSpaceDynamics;
		std::vector<RagdollLinkSettings> Links;
		std::vector<RagdollConfiguration> Pose;
		std::vector<RobotFootContact> Feet;
		Vector3 Gravity = Vector3(0, -9.81f, 0);
		double TimeStep = 0.002;
		double Friction = 1;
		// Writes generalized actuator forces for one physics tick.
		std::function<void(const FeatherstoneSolver&, const Vector3&, size_t, std::vector<double>&)> Actuate;
	};

	class ArticulatedRobotController final : public RobotController
	{
	public:
		~ArticulatedRobotController() override;
		// Attach a convex shape expressed in the joint frame for static scenery contacts.
		void AddLinkCollider(size_t Link, const Transform& Local, std::unique_ptr<Geometry> Shape);
		Vector3 GetPosition() const override { return m_Dynamics->GetLinkTransform(0).pos; }
		void SetMoveVelocity(const Vector3& WorldVelocity) override;
		bool IsGrounded() const override { return m_Supported; }
		const FeatherstoneSolver& GetDynamics() const { return *m_Dynamics; }
		RobotDynamicsBackend GetBackend() const { return m_Settings.Backend; }
		double GetTime() const { return m_Tick * m_Settings.TimeStep; }
	private:
		friend class PhysicsWorld;
		ArticulatedRobotController(PhysicsWorld& World, const ArticulatedRobotSettings& Settings);
		void Simulate(float Dt) override;
		void Step();
		struct Collider
		{
			size_t Link;
			Transform Local;
			std::unique_ptr<Geometry> Shape;
		};
		struct ContactPoint
		{
			size_t Link;
			Vector3 Local, Normal;
			double Gap;
			bool Foot;
		};
		void GatherBodyContacts(std::vector<ContactPoint>& Points);
		std::vector<Collider> m_Colliders;
		std::unique_ptr<NarrowPhase> m_Contacts;
		PhysicsWorld& m_World;
		ArticulatedRobotSettings m_Settings;
		std::unique_ptr<FeatherstoneSolver> m_Dynamics;
		Vector3 m_Command = Vector3::Zero();
		size_t m_Tick = 0;
		double m_Remainder = 0;
		bool m_Supported = false;
	};
}
