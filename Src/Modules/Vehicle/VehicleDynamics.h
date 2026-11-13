#pragma once

#include <vector>
#include "../../Collision/GeometryHandle.h"
#include "../../Collision/GeometryObject.h"

namespace Riemann
{
	class PhysicsWorld;
	class RigidBody;
	class RigidBodyDynamic;

	enum class VehicleDifferential { Open, LimitedSlip, Locked };

	struct VehicleWheelSettings
	{
		Vector3 Attachment = Vector3::Zero();
		Vector3 SuspensionDirection = -Vector3::UnitY();
		float Radius = 0.35f;
		float Inertia = 1.2f;
		float Travel = 0.4f;
		float RestLength = 0.3f;
		float SpringStiffness = 30000.0f;
		float Damping = 4500.0f;
		float MaximumSuspensionForce = 60000.0f;
		float LongitudinalStiffness = 10.0f;
		float LateralStiffness = 8.0f;
		float Friction = 1.0f;
		float BrakeTorque = 1500.0f;
		float HandbrakeTorque = 0.0f;
		float AngularDamping = 0.1f;
		float SteeringFactor = 0.0f;
		float DriveWeight = 1.0f;
		int Axle = 0;
		bool IsValid() const;
	};

	struct VehicleTorquePoint { float Speed, Torque; };
	struct VehicleDynamicsSettings
	{
		std::vector<VehicleWheelSettings> Wheels;
		Vector3 Forward = Vector3::UnitZ();
		Vector3 Up = Vector3::UnitY();
		float MaximumSteerAngle = 0.55f;
		float Wheelbase = 2.8f;
		bool AckermannSteering = true;
		float EngineInertia = 0.3f;
		float PeakEngineTorque = 400.0f;
		float IdleEngineSpeed = 80.0f;
		float MaximumEngineSpeed = 700.0f;
		float EngineDamping = 0.15f;
		float IdleGovernor = 5.0f;
		std::vector<VehicleTorquePoint> TorqueCurve = { { 0, 0.65f }, { 0.5f, 1 }, { 1, 0.75f } };
		std::vector<float> ForwardGears = { 3.2f, 2.1f, 1.4f, 1.0f };
		float ReverseRatio = -3.2f;
		float FinalDrive = 3.6f;
		float ClutchStrength = 20.0f;
		float MaximumClutchTorque = 600.0f;
		float GearChangeTime = 0.2f;
		VehicleDifferential Differential = VehicleDifferential::Open;
		float DifferentialStrength = 20.0f;
		float MaximumDifferentialTorque = 1500.0f;
		float MinimumSlipSpeed = 0.5f;
		float MinimumGroundCosine = 0.2f;
		float AxisTolerance = 1.0e-5f;
		float SleepWheelSpeed = 0.1f;
		int SolverIterations = 12;
		CollisionData FilterData;
		bool IsValid() const;
	};

	struct VehicleControls
	{
		float Throttle = 0, Steering = 0, Brake = 0, Handbrake = 0, Clutch = 1;
		int Gear = 1;
	};

	struct VehicleWheelState
	{
		bool Grounded = false;
		GeometryHandle Ground;
		Vector3 ContactPoint = Vector3::Zero(), ContactNormal = Vector3::Zero();
		float SuspensionLength = 0, SuspensionForce = 0;
		float SteeringAngle = 0, AngularSpeed = 0, Rotation = 0;
		float LongitudinalSlip = 0, LateralSlip = 0;
		float LongitudinalForce = 0, LateralForce = 0, DriveTorque = 0, BrakeTorque = 0;
	};

	class VehicleDynamics
	{
	public:
		~VehicleDynamics() = default;
		VehicleDynamics(const VehicleDynamics&) = delete;
		VehicleDynamics& operator=(const VehicleDynamics&) = delete;
		RigidBodyDynamic* GetChassis() const { return m_Chassis; }
		const VehicleDynamicsSettings& GetSettings() const { return m_Settings; }
		const VehicleControls& GetControls() const { return m_Controls; }
		const std::vector<VehicleWheelState>& GetWheels() const { return m_Wheels; }
		Transform GetWheelTransform(size_t Index) const;
		float GetEngineSpeed() const { return m_EngineSpeed; }
		int GetGear() const { return m_Gear; }
		float GetClutchTorque() const { return m_ClutchTorque; }
		bool IsEnabled() const { return m_Enabled; }
		void SetEnabled(bool Enabled);
		void SetControls(const VehicleControls& Controls);
		void SetDifferential(VehicleDifferential Mode);
		void Reset();
		bool RequiresAwake() const;

	private:
		friend class PhysicsWorld;
		VehicleDynamics(PhysicsWorld& World, RigidBodyDynamic* Chassis, const VehicleDynamicsSettings& Settings);
		void Prepare(float Dt);
		void Simulate(float Dt);
		void UpdateDrivetrain(float Dt);
		void SolveDrivetrain(float Dt);
		RigidBody* GroundBody(size_t Index) const;
		Vector3 GroundVelocity(size_t Index) const;
		void ApplyContactImpulse(size_t Index, const Vector3& Impulse);
		float InverseContactMass(size_t Index, const Vector3& Direction) const;
		float GearRatio() const;
		float TorqueMultiplier() const;

		struct WheelRuntime
		{
			Transform GroundTransform;
			Vector3 PlatformVelocity = Vector3::Zero();
			Vector3 Forward = Vector3::UnitZ(), Side = Vector3::UnitX();
			float NormalImpulse = 0, ForwardImpulse = 0, SideImpulse = 0, BrakeImpulse = 0;
			float Compression = 0, DriveWeight = 0;
		};
		PhysicsWorld& m_World;
		RigidBodyDynamic* m_Chassis;
		VehicleDynamicsSettings m_Settings;
		VehicleControls m_Controls;
		std::vector<VehicleWheelState> m_Wheels;
		std::vector<WheelRuntime> m_Runtime;
		struct DifferentialRow { size_t A, B; float Impulse = 0; };
		std::vector<DifferentialRow> m_DifferentialRows;
		float m_ClutchImpulse = 0;
		float m_EngineSpeed = 0, m_ClutchTorque = 0, m_ShiftRemaining = 0;
		int m_Gear = 1;
		bool m_Enabled = true;
	};
}
