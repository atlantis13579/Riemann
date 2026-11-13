#include "VehicleDynamics.h"
#include "../../RigidBodyDynamics/PhysicsWorld.h"
#include "../../RigidBodyDynamics/Ragdoll.h"
#include "../../Collision/GeometryQuery.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace Riemann
{
	/*
	Query-wheel vehicle dynamics
	----------------------------
	The chassis remains an ordinary world rigid body. Wheels are suspension and
	rotor degrees of freedom, not additional rigid bodies. Sphere sweeps provide
	a finite wheel footprint; width and unsprung-body/gyroscopic dynamics are not
	modelled by this approximation. VehicleDeformation remains a separate visual
	attachment, so changing tire or drivetrain models does not change crash assets.

	Prepare runs after kinematics and before gravity integration. Simulate runs
	after external-force integration and before the selected rigid-body solver.
	Consequently every force below is integrated exactly once as a physical impulse;
	it does not pass through the legacy queued ExtForce conversion. A dynamic road
	receives the opposite contact impulse at the same point.

	For compression x and normal relative speed v, backward Euler gives a spring
	impulse J = h[k*x - (c+h*k)*v] / [1+h*(c+h*k)*K], where K is inverse effective
	mass at the contact. Accumulated projected impulses let all wheels converge
	together instead of making the result depend strongly on wheel iteration order.
	Tire stiffness maps longitudinal slip speed and lateral slip to impulses.
	Wheel inertia participates in the longitudinal effective mass; a shared friction
	disk limits the combined tire impulse to mu*Jnormal. Brakes are accumulated
	angular constraints, so they can hold a wheel that tire friction tries to spin.

	The engine and driven wheels exchange an implicit clutch impulse through the
	gear ratio. The coupling denominator includes both engine and wheel inertias;
	equal and opposite work is preserved by the gear transform, while clutch slip
	dissipates energy. Open differentials distribute torque by normalized weights.
	Limited-slip/locked variants additionally exchange angular impulses within each
	axle. Locked rows have no torque cap; limited-slip rows use a viscous coupling
	and a total per-step torque limit. A gear change opens the clutch for a
	configurable interval. A zero-weight trailer has no connection to the engine.

	Settings use metres, kilograms, seconds, radians, newtons and newton-metres.
	Attachment is the body-local wheel centre at maximum suspension compression.
	RestLength is its unloaded extension along SuspensionDirection. On sloped roads
	we approximate the spring in the contact normal direction using projected
	compression. Tire stiffness is dimensionless force/load per unit slip; clutch
	and differential strength are angular damping coefficients. GetWheelTransform
	maps a wheel with local X axle, Y up and Z forward into the chassis frame.

	Reference: Gillespie, Fundamentals of Vehicle Dynamics (1992). The spring and slip laws here are deliberately
	readable models, with numerical policies and material parameters in settings.
	*/
	namespace
	{
		constexpr float Pi = 3.14159265358979323846f;
		bool Finite(const Vector3& V) { return std::isfinite(V.x) && std::isfinite(V.y) && std::isfinite(V.z); }
		bool Positive(float V) { return std::isfinite(V) && V > 0; }
		bool Nonnegative(float V) { return std::isfinite(V) && V >= 0; }
		float Clamp(float V, float Low, float High) { return std::max(Low, std::min(High, V)); }
		float Sign(float V) { return V < 0 ? -1.0f : 1.0f; }
		Vector3 PointVelocity(const RigidBody* Body, const Vector3& Point)
		{
			if (!Body || (Body->mRigidType == RigidType::Dynamic && static_cast<const RigidBodyDynamic*>(Body)->Freezing)) return Vector3::Zero();
			return Body->V + Body->W.Cross(Point - Body->X);
		}
		float InverseMass(const RigidBody* Body, const Vector3& Point, const Vector3& Direction)
		{
			if (!Body || Body->mRigidType != RigidType::Dynamic) return 0;
			if (static_cast<const RigidBodyDynamic*>(Body)->Freezing) return 0;
			if (Body->ReducedRagdoll)
			{
				auto* Owner = Body->ReducedRagdoll; const size_t Link = Body->RagdollLink;
				const Vector3 Offset = Point - Owner->GetDynamics().GetLinkTransform(Link).pos;
				std::vector<SpatialVector> Wrenches(Owner->GetBodies().size());
				Wrenches[Link] = SpatialVector(Offset.Cross(Direction), Direction);
				const auto Response = Owner->ComputeResponse(Wrenches)[Link];
				return Direction.Dot(Response.GetLinear() + Response.GetAngular().Cross(Offset));
			}
			const Vector3 Angular = (Point - Body->X).Cross(Direction);
			return Body->InvMass + Angular.Dot(Body->GetInverseInertia_WorldSpace() * Angular);
		}
		void Impulse(RigidBody* Body, const Vector3& Point, const Vector3& Value)
		{
			if (!Body || Body->mRigidType != RigidType::Dynamic) return;
			if (static_cast<const RigidBodyDynamic*>(Body)->Freezing) return;
			if (Body->ReducedRagdoll) { Body->ReducedRagdoll->ApplyImpulse(Body->RagdollLink, Point, Value); return; }
			Body->V += Value * Body->InvMass;
			Body->W += Body->GetInverseInertia_WorldSpace() * (Point - Body->X).Cross(Value);
		}
	}

	bool VehicleWheelSettings::IsValid() const
	{
		return Finite(Attachment) && Finite(SuspensionDirection) && Positive(SuspensionDirection.SquareLength())
			&& Positive(Radius) && Positive(Inertia) && Positive(Travel) && Nonnegative(RestLength) && RestLength <= Travel
			&& Positive(SpringStiffness) && Nonnegative(Damping) && Positive(MaximumSuspensionForce)
			&& Positive(LongitudinalStiffness) && Positive(LateralStiffness) && Nonnegative(Friction)
			&& Nonnegative(BrakeTorque) && Nonnegative(HandbrakeTorque) && Nonnegative(AngularDamping)
			&& std::isfinite(SteeringFactor) && std::abs(SteeringFactor) <= 1 && Nonnegative(DriveWeight) && Axle >= 0;
	}
	bool VehicleDynamicsSettings::IsValid() const
	{
		if (Wheels.empty() || ForwardGears.empty() || TorqueCurve.size() < 2
			|| !Finite(Forward) || !Finite(Up) || !Positive(Forward.SquareLength()) || !Positive(Up.SquareLength())
			|| !Positive(AxisTolerance) || AxisTolerance >= 1 || std::abs(Forward.Unit().Dot(Up.Unit())) > AxisTolerance
			|| !Nonnegative(MaximumSteerAngle) || MaximumSteerAngle >= Pi / 2 || !Positive(Wheelbase)
			|| !Positive(EngineInertia) || !Nonnegative(PeakEngineTorque) || !Nonnegative(IdleEngineSpeed)
			|| !Positive(MaximumEngineSpeed) || MaximumEngineSpeed <= IdleEngineSpeed || !Nonnegative(EngineDamping) || !Nonnegative(IdleGovernor)
			|| !std::isfinite(ReverseRatio) || ReverseRatio >= 0 || !Positive(FinalDrive)
			|| !Nonnegative(ClutchStrength) || !Nonnegative(MaximumClutchTorque) || !Nonnegative(GearChangeTime)
			|| !Nonnegative(DifferentialStrength) || !Nonnegative(MaximumDifferentialTorque)
			|| !Positive(MinimumSlipSpeed) || !Positive(MinimumGroundCosine) || MinimumGroundCosine > 1
			|| !Nonnegative(SleepWheelSpeed) || SolverIterations <= 0
			|| (Differential != VehicleDifferential::Open && Differential != VehicleDifferential::LimitedSlip && Differential != VehicleDifferential::Locked)) return false;
		float Weight = 0;
		for (const auto& W : Wheels) { if (!W.IsValid()) return false; Weight += W.DriveWeight; }
		if (!std::isfinite(Weight)) return false;
		for (float Gear : ForwardGears) if (!Positive(Gear) || !Positive(Gear * FinalDrive)) return false;
		if (!Positive(-ReverseRatio * FinalDrive)) return false;
		if (TorqueCurve.front().Speed != 0 || TorqueCurve.back().Speed != 1) return false;
		float Previous = -1;
		for (const auto& Point : TorqueCurve)
		{
			if (!Nonnegative(Point.Speed) || Point.Speed <= Previous || Point.Speed > 1 || !Nonnegative(Point.Torque)) return false;
			Previous = Point.Speed;
		}
		return true;
	}

	VehicleDynamics::VehicleDynamics(PhysicsWorld& World, RigidBodyDynamic* Chassis, const VehicleDynamicsSettings& Settings)
		: m_World(World), m_Chassis(Chassis), m_Settings(Settings)
	{
		if (!Chassis || !Positive(Chassis->InvMass) || !Settings.IsValid()) throw std::invalid_argument("Invalid vehicle chassis or settings");
		m_Settings.Forward.Normalize(); m_Settings.Up.Normalize();
		float TotalWeight = 0;
		for (auto& W : m_Settings.Wheels) { W.SuspensionDirection.Normalize(); TotalWeight += W.DriveWeight; }
		m_Runtime.resize(Settings.Wheels.size()); m_Wheels.resize(Settings.Wheels.size());
		for (size_t I = 0; I < m_Runtime.size(); ++I) m_Runtime[I].DriveWeight = TotalWeight > 0 ? Settings.Wheels[I].DriveWeight / TotalWeight : 0;
		for (size_t A = 0; A < m_Wheels.size(); ++A) for (size_t B = A + 1; B < m_Wheels.size(); ++B)
			if (Settings.Wheels[A].Axle == Settings.Wheels[B].Axle && m_Runtime[A].DriveWeight > 0 && m_Runtime[B].DriveWeight > 0)
				m_DifferentialRows.push_back({ A, B });
		Reset();
	}
	void VehicleDynamics::Reset()
	{
		m_EngineSpeed = m_Settings.IdleEngineSpeed; m_ClutchTorque = 0; m_ShiftRemaining = 0; m_Gear = m_Controls.Gear;
		m_ClutchImpulse = 0;
		for (auto& Row : m_DifferentialRows) Row.Impulse = 0;
		for (size_t I = 0; I < m_Wheels.size(); ++I)
		{
			m_Wheels[I] = VehicleWheelState(); m_Wheels[I].SuspensionLength = m_Settings.Wheels[I].Travel;
			const float Weight = m_Runtime[I].DriveWeight; m_Runtime[I] = WheelRuntime(); m_Runtime[I].DriveWeight = Weight;
		}
		m_Chassis->Wakeup();
	}
	void VehicleDynamics::SetEnabled(bool Enabled)
	{
		if (m_Enabled == Enabled) return;
		m_Enabled = Enabled; Reset();
	}
	void VehicleDynamics::SetControls(const VehicleControls& C)
	{
		auto Unit = [](float V) { return Nonnegative(V) && V <= 1; };
		if (!Unit(C.Throttle) || !Unit(C.Brake) || !Unit(C.Handbrake) || !Unit(C.Clutch)
			|| !std::isfinite(C.Steering) || std::abs(C.Steering) > 1 || C.Gear < -1 || C.Gear > (int)m_Settings.ForwardGears.size())
			throw std::invalid_argument("Invalid vehicle controls");
		if (C.Throttle != m_Controls.Throttle || C.Steering != m_Controls.Steering || C.Brake != m_Controls.Brake
			|| C.Handbrake != m_Controls.Handbrake || C.Clutch != m_Controls.Clutch || C.Gear != m_Controls.Gear) m_Chassis->Wakeup();
		if (C.Gear != m_Controls.Gear) m_ShiftRemaining = m_Settings.GearChangeTime;
		m_Controls = C;
	}
	bool VehicleDynamics::RequiresAwake() const
	{
		if (!m_Enabled) return false;
		if (m_Controls.Throttle > 0 || (m_Controls.Gear != 0 && m_Controls.Clutch > 0 && m_Settings.IdleGovernor > 0)) return true;
		for (const auto& W : m_Wheels) if (std::abs(W.AngularSpeed) > m_Settings.SleepWheelSpeed) return true;
		return false;
	}
	void VehicleDynamics::SetDifferential(VehicleDifferential Mode)
	{
		if (Mode != VehicleDifferential::Open && Mode != VehicleDifferential::LimitedSlip && Mode != VehicleDifferential::Locked)
			throw std::invalid_argument("Invalid vehicle differential");
		if (m_Settings.Differential == Mode) return;
		m_Settings.Differential = Mode; m_Chassis->Wakeup();
	}
	float VehicleDynamics::GearRatio() const
	{
		if (m_Gear == 0) return 0;
		return (m_Gear < 0 ? m_Settings.ReverseRatio : m_Settings.ForwardGears[(size_t)m_Gear - 1]) * m_Settings.FinalDrive;
	}
	float VehicleDynamics::TorqueMultiplier() const
	{
		const float Speed = Clamp(m_EngineSpeed / m_Settings.MaximumEngineSpeed, 0, 1);
		for (size_t I = 1; I < m_Settings.TorqueCurve.size(); ++I)
		{
			const auto A = m_Settings.TorqueCurve[I - 1], B = m_Settings.TorqueCurve[I];
			if (Speed <= B.Speed) return A.Torque + (B.Torque - A.Torque) * (Speed - A.Speed) / (B.Speed - A.Speed);
		}
		return m_Settings.TorqueCurve.back().Torque;
	}
	RigidBody* VehicleDynamics::GroundBody(size_t Index) const
	{
		const auto* Record = m_World.GetGeometryWorldRecord(m_Wheels[Index].Ground);
		return Record ? Record->State.Body : nullptr;
	}
	Vector3 VehicleDynamics::GroundVelocity(size_t Index) const
	{
		auto* Body = GroundBody(Index);
		return Body && Body->mRigidType == RigidType::Dynamic ? PointVelocity(Body, m_Wheels[Index].ContactPoint) : m_Runtime[Index].PlatformVelocity;
	}
	float VehicleDynamics::InverseContactMass(size_t Index, const Vector3& Direction) const
	{
		const auto& W = m_Wheels[Index];
		return InverseMass(m_Chassis, W.ContactPoint, Direction) + InverseMass(GroundBody(Index), W.ContactPoint, Direction);
	}
	void VehicleDynamics::ApplyContactImpulse(size_t Index, const Vector3& Value)
	{
		const auto& W = m_Wheels[Index];
		Impulse(m_Chassis, W.ContactPoint, Value); Impulse(GroundBody(Index), W.ContactPoint, -Value);
	}

	void VehicleDynamics::Prepare(float Dt)
	{
		if (!m_Enabled) return;
		bool Wake = RequiresAwake();
		const Vector3 Up = m_Chassis->BodyLocalToWorldDirection(m_Settings.Up);
		const Vector3 Right = m_Settings.Up.Cross(m_Settings.Forward);
		for (size_t I = 0; I < m_Wheels.size(); ++I)
		{
			auto& W = m_Wheels[I]; auto& R = m_Runtime[I]; const auto& S = m_Settings.Wheels[I];
			const auto OldGround = W.Ground;
			const Transform OldTransform = R.GroundTransform;
			W.Grounded = false; W.Ground = GeometryHandle(); W.SuspensionLength = S.Travel;
			W.SuspensionForce = W.LongitudinalForce = W.LateralForce = W.DriveTorque = W.BrakeTorque = 0;
			R.NormalImpulse = R.ForwardImpulse = R.SideImpulse = R.BrakeImpulse = 0;
			W.SteeringAngle = m_Controls.Steering * m_Settings.MaximumSteerAngle * S.SteeringFactor;
			if (m_Settings.AckermannSteering && std::abs(W.SteeringAngle) > m_Settings.AxisTolerance)
			{
				const float Radius = m_Settings.Wheelbase / std::tan(W.SteeringAngle);
				const float WheelRadius = Radius - S.Attachment.Dot(Right);
				W.SteeringAngle = std::atan(m_Settings.Wheelbase / (Sign(Radius) * std::max(m_Settings.AxisTolerance, std::abs(WheelRadius))));
			}
			const Vector3 Start = m_Chassis->BodyLocalToWorld(S.Attachment);
			const Vector3 Down = m_Chassis->BodyLocalToWorldDirection(S.SuspensionDirection);
			SweepOption O; O.MaxDist = S.Travel; O.FilterData = m_Settings.FilterData; O.SourceBody = m_Chassis;
			O.PreFilter = [](const Geometry* G) { return G->IsSimulationEnabled() && !G->IsTriggerEnabled(); };
			O.PostFilter = [&](const Geometry*, const Vector3& P, const Vector3& N, float T)
			{ return Finite(P) && Finite(N) && Nonnegative(T) && N.Dot(Up) >= m_Settings.MinimumGroundCosine && N.Dot(-Down) >= m_Settings.MinimumGroundCosine; };
			SweepResult Hit;
			if (m_World.GetGeometryQuery()->SphereCastQuery(Start, S.Radius, Down, O, &Hit))
			{
				const auto* Proxy = m_World.GetGeometryQuery()->GetProxy(Hit.hitGeom);
				const auto* Record = Proxy ? m_World.GetGeometryWorldRecord(Proxy->Handle) : nullptr;
				if (Record)
				{
					W.Grounded = true; W.Ground = Proxy->Handle; W.ContactPoint = Hit.hitPosition; W.ContactNormal = Hit.hitNormal.Unit();
					W.SuspensionLength = Clamp(Hit.hitTimeMin, 0, S.Travel);
					R.GroundTransform = Record->State.ShapeToWorld;
					R.PlatformVelocity = PointVelocity(Record->State.Body, W.ContactPoint);
					if (OldGround == W.Ground && OldTransform != R.GroundTransform)
						R.PlatformVelocity = (W.ContactPoint - OldTransform.LocalToWorld(R.GroundTransform.WorldToLocal(W.ContactPoint))) / Dt;
					Wake |= OldGround != W.Ground || (OldGround == W.Ground && OldTransform != R.GroundTransform);
					if (m_World.m_IslandSleeping.GetSettings().Enabled) m_World.m_IslandSleeping.AddSupport(m_Chassis, Record->State.Body);
					if (auto* Ground = Record->State.Body ? Record->State.Body->CastDynamic() : nullptr)
					{
						if (!Ground->Freezing)
						{
							Wake |= Ground->Sleeping != m_Chassis->Sleeping;
							if (!m_Chassis->Sleeping && Ground->Sleeping) Ground->Wakeup();
						}
					}
				}
			}
			Wake |= OldGround.IsValid() && !W.Grounded;
			R.Compression = W.Grounded ? (S.RestLength - W.SuspensionLength) * W.ContactNormal.Dot(-Down) : 0;
			Quaternion Steering; Steering.FromRotationAxis(m_Settings.Up, W.SteeringAngle);
			Vector3 Forward = m_Chassis->BodyLocalToWorldDirection(Steering * m_Settings.Forward);
			if (W.Grounded) Forward -= W.ContactNormal * Forward.Dot(W.ContactNormal);
			R.Forward = Forward.SafeUnit(); R.Side = (W.Grounded ? W.ContactNormal : Up).Cross(R.Forward).SafeUnit();
		}
		if (Wake)
		{
			m_Chassis->Wakeup();
			if (m_World.m_IslandSleeping.GetSettings().Enabled) m_World.m_IslandSleeping.Activate(m_Chassis);
		}
	}

	void VehicleDynamics::UpdateDrivetrain(float Dt)
	{
		m_Gear = m_Controls.Gear;
		m_ShiftRemaining = std::max(0.0f, m_ShiftRemaining - Dt);
		const float ThrottleTorque = m_EngineSpeed < m_Settings.MaximumEngineSpeed ? m_Controls.Throttle * m_Settings.PeakEngineTorque * TorqueMultiplier() : 0;
		const float IdleTorque = Clamp((m_Settings.IdleEngineSpeed - m_EngineSpeed) * m_Settings.IdleGovernor, 0, m_Settings.PeakEngineTorque);
		m_EngineSpeed = std::max(0.0f, (m_EngineSpeed + Dt * (ThrottleTorque + IdleTorque) / m_Settings.EngineInertia)
			/ (1 + Dt * m_Settings.EngineDamping / m_Settings.EngineInertia));
		m_ClutchImpulse = 0; m_ClutchTorque = 0;
		for (auto& Row : m_DifferentialRows) Row.Impulse = 0;
		for (size_t I = 0; I < m_Wheels.size(); ++I)
			m_Wheels[I].AngularSpeed /= 1 + m_Settings.Wheels[I].AngularDamping * Dt / m_Settings.Wheels[I].Inertia;
	}
	void VehicleDynamics::SolveDrivetrain(float Dt)
	{
		float ShaftSpeed = 0, DrivenWeight = 0, InverseInertia = 1 / m_Settings.EngineInertia;
		const float Ratio = GearRatio();
		for (size_t I = 0; I < m_Wheels.size(); ++I)
		{
			const auto& W = m_Wheels[I]; const auto& S = m_Settings.Wheels[I]; const float Weight = m_Runtime[I].DriveWeight;
			ShaftSpeed += Weight * W.AngularSpeed;
			DrivenWeight += Weight;
			InverseInertia += Ratio * Ratio * Weight * Weight / S.Inertia;
		}
		const float Coupling = m_ShiftRemaining > 0 || Ratio == 0 || DrivenWeight == 0 ? 0 : m_Controls.Clutch * m_Settings.ClutchStrength * Dt;
		const float Limit = m_Settings.MaximumClutchTorque * m_Controls.Clutch * Dt;
		const float Accumulated = Clamp(m_ClutchImpulse + (Coupling * (m_EngineSpeed - Ratio * ShaftSpeed) - m_ClutchImpulse)
			/ (1 + Coupling * InverseInertia), -Limit, Limit);
		const float ClutchImpulse = Accumulated - m_ClutchImpulse;
		m_ClutchImpulse = Accumulated; m_ClutchTorque = Accumulated / Dt;
		m_EngineSpeed -= ClutchImpulse / m_Settings.EngineInertia;
		for (size_t I = 0; I < m_Wheels.size(); ++I)
		{
			const float TorqueImpulse = Ratio * m_Runtime[I].DriveWeight * ClutchImpulse;
			m_Wheels[I].AngularSpeed += TorqueImpulse / m_Settings.Wheels[I].Inertia;
			m_Wheels[I].DriveTorque += TorqueImpulse / Dt;
		}
		if (m_Settings.Differential == VehicleDifferential::Open) return;
		for (auto& Row : m_DifferentialRows)
		{
			const size_t A = Row.A, B = Row.B;
			const auto& WA = m_Settings.Wheels[A]; const auto& WB = m_Settings.Wheels[B];
			const float K = 1 / WA.Inertia + 1 / WB.Inertia;
			const float Relative = m_Wheels[B].AngularSpeed - m_Wheels[A].AngularSpeed;
			float Accumulated = Row.Impulse + Relative / K;
			if (m_Settings.Differential == VehicleDifferential::LimitedSlip)
			{
				const float Coupling = m_Settings.DifferentialStrength * Dt;
				Accumulated = Row.Impulse + (Coupling * Relative - Row.Impulse) / (1 + Coupling * K);
				const float Maximum = m_Settings.MaximumDifferentialTorque * Dt; Accumulated = Clamp(Accumulated, -Maximum, Maximum);
			}
			const float J = Accumulated - Row.Impulse; Row.Impulse = Accumulated;
			m_Wheels[A].AngularSpeed += J / WA.Inertia; m_Wheels[B].AngularSpeed -= J / WB.Inertia;
			m_Wheels[A].DriveTorque += J / Dt; m_Wheels[B].DriveTorque -= J / Dt;
		}
	}

	void VehicleDynamics::Simulate(float Dt)
	{
		if (!m_Enabled) return;
		UpdateDrivetrain(Dt);
		if (m_Chassis->Sleeping) return;
		for (int Iteration = 0; Iteration < m_Settings.SolverIterations; ++Iteration)
		{
			for (size_t I = 0; I < m_Wheels.size(); ++I)
			{
				auto& W = m_Wheels[I]; auto& R = m_Runtime[I]; const auto& S = m_Settings.Wheels[I];
				const float BrakeLimit = (m_Controls.Brake * S.BrakeTorque + m_Controls.Handbrake * S.HandbrakeTorque) * Dt;
				const float Brake = Clamp(R.BrakeImpulse - W.AngularSpeed * S.Inertia, -BrakeLimit, BrakeLimit);
				W.AngularSpeed += (Brake - R.BrakeImpulse) / S.Inertia; R.BrakeImpulse = Brake;
				if (!W.Grounded) continue;
				const float SpringRate = S.Damping + Dt * S.SpringStiffness;
				const float Softness = 1 / (Dt * SpringRate);
				const float Bias = S.SpringStiffness * R.Compression / SpringRate;
				Vector3 Relative = PointVelocity(m_Chassis, W.ContactPoint) - GroundVelocity(I);
				const float NormalDelta = (Bias - Relative.Dot(W.ContactNormal) - Softness * R.NormalImpulse)
					/ (InverseContactMass(I, W.ContactNormal) + Softness);
				const float Normal = Clamp(R.NormalImpulse + NormalDelta, 0, S.MaximumSuspensionForce * Dt);
				ApplyContactImpulse(I, W.ContactNormal * (Normal - R.NormalImpulse)); R.NormalImpulse = Normal;
				Relative = PointVelocity(m_Chassis, W.ContactPoint) - GroundVelocity(I);
				const float LongSpeed = Relative.Dot(R.Forward), SideSpeed = Relative.Dot(R.Side);
				const float Speed = std::max(m_Settings.MinimumSlipSpeed, std::max(std::abs(LongSpeed), std::abs(W.AngularSpeed * S.Radius)));
				const float Load = Normal / Dt;
				const float CX = S.LongitudinalStiffness * Load / Speed, CY = S.LateralStiffness * Load / Speed;
				float Forward = 0, Side = 0;
				if (CX > 0 && CY > 0)
				{
					const float KX = InverseContactMass(I, R.Forward) + S.Radius * S.Radius / S.Inertia;
					const float KY = InverseContactMass(I, R.Side);
					Forward = R.ForwardImpulse + (W.AngularSpeed * S.Radius - LongSpeed - R.ForwardImpulse / (Dt * CX)) / (KX + 1 / (Dt * CX));
					Side = R.SideImpulse + (-SideSpeed - R.SideImpulse / (Dt * CY)) / (KY + 1 / (Dt * CY));
				}
				auto* Ground = GroundBody(I);
				const float Friction = S.Friction * (Ground ? Ground->GetFrictionDynamic() : 1.0f);
				const float Magnitude = std::sqrt(Forward * Forward + Side * Side);
				const float Limit = Friction * Normal;
				if (Magnitude > Limit && Magnitude > 0) { Forward *= Limit / Magnitude; Side *= Limit / Magnitude; }
				const float Delta = Forward - R.ForwardImpulse;
				ApplyContactImpulse(I, R.Forward * Delta + R.Side * (Side - R.SideImpulse));
				W.AngularSpeed -= S.Radius * Delta / S.Inertia;
				R.ForwardImpulse = Forward; R.SideImpulse = Side;
			}
			// Tire reactions must participate in the same clutch/differential solve.
			// Accumulated limits apply once per substep, independent of iteration count.
			SolveDrivetrain(Dt);
		}
		for (size_t I = 0; I < m_Wheels.size(); ++I)
		{
			auto& W = m_Wheels[I]; const auto& R = m_Runtime[I]; const auto& S = m_Settings.Wheels[I];
			W.Rotation = std::fmod(W.Rotation + W.AngularSpeed * Dt, 2 * Pi);
			W.SuspensionForce = R.NormalImpulse / Dt; W.LongitudinalForce = R.ForwardImpulse / Dt;
			W.LateralForce = R.SideImpulse / Dt; W.BrakeTorque = R.BrakeImpulse / Dt;
			const Vector3 Relative = W.Grounded ? PointVelocity(m_Chassis, W.ContactPoint) - GroundVelocity(I) : Vector3::Zero();
			const float Speed = std::max(m_Settings.MinimumSlipSpeed, std::max(std::abs(Relative.Dot(R.Forward)), std::abs(W.AngularSpeed * S.Radius)));
			W.LongitudinalSlip = (W.AngularSpeed * S.Radius - Relative.Dot(R.Forward)) / Speed;
			W.LateralSlip = std::atan2(Relative.Dot(R.Side), std::max(m_Settings.MinimumSlipSpeed, std::abs(Relative.Dot(R.Forward))));
		}
	}
	Transform VehicleDynamics::GetWheelTransform(size_t Index) const
	{
		const auto& S = m_Settings.Wheels.at(Index); const auto& W = m_Wheels.at(Index);
		Quaternion Steer, Roll, Frame;
		Steer.FromRotationAxis(m_Settings.Up, W.SteeringAngle);
		Roll.FromRotationAxis(m_Settings.Up.Cross(m_Settings.Forward), W.Rotation);
		const Vector3 Right = m_Settings.Up.Cross(m_Settings.Forward);
		Frame.FromRotationMatrix3(Matrix3(Right, m_Settings.Up, m_Settings.Forward, false));
		return Transform(m_Chassis->BodyLocalToWorld(S.Attachment + S.SuspensionDirection * W.SuspensionLength), m_Chassis->Q * Steer * Roll * Frame);
	}
}
