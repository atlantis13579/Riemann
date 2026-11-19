#pragma once

#include <cstdint>
#include <vector>

#include "../../Deformation/DeformableBody.h"
#include "../../Collision/GeometryAggregate.h"
#include "../../Maths/Transform.h"

namespace Riemann
{
	class RigidBody;
	class PhysicsWorld;
	class SliderConstraint;

	struct VehicleDeformationSettings
	{
		VehicleDeformationSettings();

		Vector3 HalfExtent;
		Vector3i Cells;
		float CabinHeight;
		float ShellMass;
		// Length of each longitudinal crush zone. Zero uses a uniform cage.
		float CrushZoneLength = 0.0f;
		DeformableBodySettings Material;
		float ImpactRadius;
		float ImpulseScale;
		float MinImpactImpulse;
		// Limit the sum of scaled impulse magnitudes consumed in one simulation step.
		float MaxDeformationImpulse;
	};

	// The rigid body supplies collision response; this local cage supplies the visual shell.
	// The borrowed body must outlive this object, or be detached before destruction.
	class VehicleDeformation : public GeometryAggregate
	{
	public:
		explicit VehicleDeformation(const VehicleDeformationSettings& settings = VehicleDeformationSettings());
		~VehicleDeformation() override;
		VehicleDeformation(const VehicleDeformation&) = delete;
		VehicleDeformation& operator=(const VehicleDeformation&) = delete;

		// cageToBody uses the body's local origin, including its center-of-mass offset.
		bool Attach(RigidBody* body, const Transform& cageToBody = Transform::Identity());
		// The chassis collider must leave the front crush stroke clear. The world
		// and chassis must outlive the attachment.
		bool CreateFrontRails(PhysicsWorld& world, float yieldForce);
		const RigidBody* GetFrontBumper(int side) const;
		void Detach();
		// Supply the physical impulse applied to this body, in world coordinates.
		bool QueueImpact(const Vector3& worldPoint, const Vector3& worldImpulse);
		void OnContact(const PhysicsContactInfo& contactInfo) override;
		// Call once after the world's rigid-body step; the cage is not registered with World.
		void Simulate(float dt);
		// Restore shape and material state, retaining the attachment and chassis pose.
		void Reset();

		RigidBody* GetBody() const { return m_Body; }
		const DeformableBody& GetCage() const { return m_Cage; }
		const VehicleDeformationSettings& GetSettings() const { return m_Settings; }
		Geometry* GetRenderGeometry() const { return m_Cage.GetRenderGeometry(); }
		Transform GetWorldTransform() const;
		uint64_t GetRevision() const { return m_Revision; }
		size_t GetPendingImpactCount() const { return m_Impacts.size(); }
		float GetMaxDisplacement() const;
		float GetMaxPlasticStrain() const;

	private:
		struct Impact
		{
			Vector3 Point;
			Vector3 Impulse;
		};
		void BuildCage();
		void UpdateFrontRails();
		void SyncRenderGeometry();

		VehicleDeformationSettings m_Settings;
		DeformableBody m_Cage;
		RigidBody* m_Body = nullptr;
		struct CrushRail
		{
			RigidBody* Bumper = nullptr;
			SliderConstraint* Slider = nullptr;
			Vector3 LocalCenter = Vector3::Zero();
		};
		PhysicsWorld* m_RailWorld = nullptr;
		CrushRail m_Rails[2];
		std::vector<int> m_BumperNodes;
		Transform m_CageToBody;
		Transform m_LastWorldTransform;
		std::vector<Impact> m_Impacts;
		std::vector<Vector3> m_LastPositions;
		uint64_t m_Revision = 0;
	};
}
