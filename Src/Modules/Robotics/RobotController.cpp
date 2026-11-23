#include "RobotController.h"
#include "../../RigidBodyDynamics/PhysicsWorld.h"
#include "../../RigidBodyDynamics/NarrowPhase.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace Riemann
{
	/* Query-based character locomotion
	   -------------------------------
	   The world owns a query-only shape; character motion is prescribed rather
	   than integrated by the rigid-body solver. Gravity/jump and desired velocity
	   are conveniences over Move(displacement, dt), which is also usable directly
	   with AutomaticUpdate disabled. This does not apply impulses to rigid bodies.

	   Movement first recovers penetrations using the same narrow phase as rigid
	   bodies, then sweeps a skin-inflated shape. SweepTolerance leaves an additional
	   numerical gap outside the primitive sweep tolerance. Each hit adds a
	   half-space to the admissible motion cone. Up, lateral and down phases allow
	   stairs without converting arbitrary upward input into extra step height.
	   An unwalkable upward-facing surface becomes a wall for lateral locomotion;
	   downward motion keeps its real normal so gravity can slide down the slope.

	   A step is a transaction: up, forward, then down onto a walkable surface.
	   All legs must succeed before the elevated pose is committed. Ground snap
	   is disabled while ascending. Platforms carry the character's local center
	   through their rigid transform, including rotation, and that displacement
	   is swept against the environment. Generation-checked shape handles avoid
	   retaining a pointer to a removed platform. The character stays upright.

	   Sweeps and overlap recovery handle walls, steps and floor support. Numerical policies
	   are exposed as settings; iteration limits report incomplete recovery/motion
	   rather than teleporting through an unresolved obstruction.
	*/
	namespace
	{
		bool Finite(const Vector3& V) { return std::isfinite(V.x) && std::isfinite(V.y) && std::isfinite(V.z); }
		bool Positive(float V) { return std::isfinite(V) && V > 0.0f; }
		bool Nonnegative(float V) { return std::isfinite(V) && V >= 0.0f; }
	}

	bool RobotControllerSettings::IsValid() const
	{
		return (Shape == RobotShape::Capsule || Shape == RobotShape::Box)
			&& Positive(Radius) && Nonnegative(HalfHeight) && Finite(HalfExtent)
			&& HalfExtent.x > 0 && HalfExtent.y > 0 && HalfExtent.z > 0
			&& Finite(Up) && Positive(Up.SquareLength()) && Positive(SkinWidth)
			&& Nonnegative(StepHeight) && Nonnegative(GroundSnapDistance) && Nonnegative(Gravity)
			&& Positive(SlopeLimitCosine) && SlopeLimitCosine <= 1
			&& Positive(MinimumMoveDistance) && Positive(RecoveryTolerance) && Positive(SweepTolerance) && SweepTolerance < SkinWidth
			&& Positive(MaximumRecoveryDistance) && Positive(NormalTolerance) && NormalTolerance < 1
			&& SlideIterations > 0 && RecoveryIterations > 0;
	}

	SweepRobotController::SweepRobotController(PhysicsWorld& World, const RobotControllerSettings& Settings, const Vector3& Position)
		: m_World(World), m_Settings(Settings), m_NarrowPhase(NarrowPhase::Create_GJKEPA())
	{
		if (!Settings.IsValid() || !Finite(Position))
			throw std::invalid_argument("Invalid character controller settings or position");
		m_Settings.Up.Normalize();
		Quaternion Rotation; Rotation.FromTwoAxis(Vector3::UnitY(), m_Settings.Up);
		m_Geometry.reset(Settings.Shape == RobotShape::Capsule
			? GeometryFactory::CreateCapsule(Position - m_Settings.Up * Settings.HalfHeight, Position + m_Settings.Up * Settings.HalfHeight, Settings.Radius)
			: GeometryFactory::CreateOBB(Position, Settings.HalfExtent, Rotation));
		m_SweepGeometry.reset(Settings.Shape == RobotShape::Capsule
			? GeometryFactory::CreateCapsule(Position - m_Settings.Up * Settings.HalfHeight, Position + m_Settings.Up * Settings.HalfHeight, Settings.Radius + Settings.SkinWidth)
			: GeometryFactory::CreateOBB(Position, Settings.HalfExtent + Vector3(Settings.SkinWidth), Rotation));
		m_Geometry->SetSimulationEnabled(false);
		m_Geometry->SetFilterData(Settings.FilterData);
		ContactGenerationSettings Contacts; Contacts.UseShapeOffsets = false;
		m_NarrowPhase->SetContactGeneration(Contacts, 0.0f);
	}

	SweepRobotController::~SweepRobotController() = default;
	Vector3 SweepRobotController::GetPosition() const { return m_Geometry->GetPosition(); }

	void SweepRobotController::SetPositionInternal(const Vector3& Position)
	{
		m_Geometry->SetTransform(Position, m_Geometry->GetRotation());
		m_SweepGeometry->SetTransform(Position, m_Geometry->GetRotation());
	}

	void SweepRobotController::SetPosition(const Vector3& Position)
	{
		if (!Finite(Position))
			throw std::invalid_argument("Invalid character position");
		SetPositionInternal(Position); m_Ground = RobotGround(); m_StepSupport = GeometryHandle(); m_VerticalVelocity = 0;
		m_LastMove = RobotMoveResult();
		m_World.UpdateSceneQuery(m_World.UpdateGeometryWorldState(m_Geometry.get(), m_World.m_Clock.tick));
	}

	void SweepRobotController::SetMoveVelocity(const Vector3& Velocity)
	{
		if (!Finite(Velocity))
			throw std::invalid_argument("Invalid character velocity");
		m_MoveVelocity = Velocity - m_Settings.Up * Velocity.Dot(m_Settings.Up);
	}

	void SweepRobotController::SetFilterData(const CollisionData& Data)
	{
		m_Settings.FilterData = Data; m_Geometry->SetFilterData(Data); m_Ground = RobotGround(); m_StepSupport = GeometryHandle();
	}

	bool SweepRobotController::Jump(float Speed)
	{
		if (!Positive(Speed) || !IsGrounded())
			return false;
		m_VerticalVelocity = Speed; m_Ground = RobotGround(); m_StepSupport = GeometryHandle(); return true;
	}

	bool SweepRobotController::Blocks(const Geometry* Shape) const
	{
		const GeometryWorldRecord* Ignored = m_World.GetGeometryWorldRecord(m_IgnoredPlatform);
		return Shape && Shape != m_Geometry.get() && Shape->IsQueryEnabled() && !Shape->IsTriggerEnabled()
			&& (!Ignored || Ignored->State.Geom != Shape);
	}

	bool SweepRobotController::Cast(const Vector3& Displacement, SweepResult& Hit) const
	{
		const float Length = Displacement.Length();
		if (Length < m_Settings.MinimumMoveDistance)
			return false;
		const Vector3 Direction = Displacement / Length;
		SweepOption Option; Option.MaxDist = Length; Option.FilterData = m_Settings.FilterData;
		Option.PreFilter = [this](const Geometry* Shape) { return Blocks(Shape); };
		Option.PostFilter = [&](const Geometry*, const Vector3& Position, const Vector3& Normal, float Distance)
		{
			return Finite(Position) && Finite(Normal) && std::isfinite(Distance) && Distance >= 0
				&& Normal.Dot(Direction) < -m_Settings.NormalTolerance;
		};
		return m_World.GetGeometryQuery()->SweepGeometry(m_SweepGeometry.get(), Direction, Option, &Hit);
	}

	float SweepRobotController::TravelDistance(const Vector3& Direction, const SweepResult& Hit) const
	{
		const float Closing = std::max(m_Settings.NormalTolerance, -Direction.Dot(Hit.hitNormal.SafeUnit()));
		return std::max(0.0f, Hit.hitTimeMin - m_Settings.SweepTolerance / Closing);
	}

	void SweepRobotController::RecoverOverlaps()
	{
		for (int Iteration = 0; Iteration <= m_Settings.RecoveryIterations; ++Iteration)
		{
			IntersectOption Option; Option.maxOverlaps = UINT_MAX; Option.BoundsOnly = true;
			Option.FilterData = m_Settings.FilterData;
			Option.PreFilter = [this](const Geometry* Shape) { return Blocks(Shape); };
			IntersectResult Candidates; const Box3 Bounds = m_SweepGeometry->GetBounds();
			m_World.GetGeometryQuery()->IntersectQueryBox(Bounds.GetCenter(), Bounds.GetExtent(), Option, &Candidates);
			// Infinite half-spaces can contain the entire query bounds. Include them
			// even when their finite broad-phase proxy only covers the boundary.
			for (const GeometryWorldState* State : m_World.m_GeometryWorldStates)
				if (State && State->Geom && State->Geom->GetShapeType() == PrimitiveType::PLANE && Blocks(State->Geom)
					&& CollisionFilter::Allows(m_World.GetGeometryQuery()->GetDefaultCollisionFilter(), m_Settings.FilterData, State->Geom->GetFilterData())
					&& std::find(Candidates.overlapGeoms.begin(), Candidates.overlapGeoms.end(), State->Geom) == Candidates.overlapGeoms.end())
					Candidates.overlapGeoms.push_back(State->Geom);
			GeometryWorldState Character; Character.Geom = m_SweepGeometry.get();
			Character.ShapeToWorld = *m_Geometry->GetTransform(); Character.WorldBounds = Bounds;
			std::vector<GeometryWorldState> States(1, Character);
			std::vector<OverlapPair> Pairs;
			for (Geometry* Shape : Candidates.overlapGeoms)
			{
				const GeometryQueryProxy* Proxy = m_World.GetGeometryQuery()->GetProxy(Shape);
				const GeometryWorldRecord* Record = Proxy ? m_World.GetGeometryWorldRecord(Proxy->Handle) : nullptr;
				if (!Record)
					continue;
				Pairs.emplace_back(0, (int)States.size()); States.push_back(Record->State);
			}
			std::vector<const GeometryWorldState*> Pointers;
			for (const GeometryWorldState& State : States) Pointers.push_back(&State);
			std::vector<ContactManifold*> Contacts;
			m_NarrowPhase->CollisionDetection(GeometryWorldStateSpan(Pointers.data(), Pointers.size()), Pairs, &Contacts);
			Vector3 Correction = Vector3::Zero(); float Deepest = m_Settings.RecoveryTolerance;
			for (const ContactManifold* Patch : Contacts) for (int I = 0; I < Patch->NumContactPointCount; ++I)
			{
				const Contact& Point = Patch->ContactPoints[I];
				if (Point.PenetrationDepth > Deepest && Finite(Point.Normal))
				{
					Deepest = Point.PenetrationDepth;
					Correction = -Point.Normal.SafeUnit() * (Deepest + m_Settings.SweepTolerance);
				}
			}
			if (Correction.SquareLength() == 0)
				return;
			const float Budget = m_Settings.MaximumRecoveryDistance - m_LastMove.RecoveryDistance;
			if (Iteration == m_Settings.RecoveryIterations || Budget <= m_Settings.RecoveryTolerance)
			{
				m_LastMove.RecoveryIncomplete = true;
				return;
			}
			const float Length = Correction.Length();
			if (Length > Budget)
				Correction *= Budget / Length;
			SetPositionInternal(GetPosition() + Correction);
			m_LastMove.RecoveryDistance += Correction.Length();
		}
	}

	bool SweepRobotController::StepSurface(const SweepResult& Hit, Vector3& Normal, Vector3& Point) const
	{
		Normal = Hit.hitNormal.SafeUnit();
		Point = m_SweepGeometry->GetSupport(-Normal) - m_Settings.Up * Hit.hitTimeMin;
		if (Normal.Dot(m_Settings.Up) >= m_Settings.SlopeLimitCosine)
			return true;
		if (m_Settings.Shape != RobotShape::Capsule || Normal.Dot(m_Settings.Up) <= m_Settings.NormalTolerance)
			return false;
		const GeometryQueryProxy* Proxy = m_World.GetGeometryQuery()->GetProxy(Hit.hitGeom);
		if (!Proxy || !Proxy->ShapeToWorld)
			return false;
		// A capsule touching a stair edge has a diagonal separating normal even
		// when the supporting face is horizontal. Probe just inside that edge;
		// this validates the face without extending the requested horizontal move.
		const Vector3 Into = (-Normal + m_Settings.Up * Normal.Dot(m_Settings.Up)).SafeUnit();
		const float ProbeRadius = m_Settings.SweepTolerance * 2;
		const float ProbeHeight = m_Settings.SkinWidth * 2;
		const Vector3 Origin = Point + Into * m_Settings.SkinWidth + m_Settings.Up * ProbeHeight;
		std::unique_ptr<Geometry> Probe(GeometryFactory::CreateSphere(Origin, ProbeRadius));
		Vector3 ProbePoint = Vector3::Zero(), ProbeNormal = Vector3::Zero(); float Distance = 0;
		if (!Probe->Sweep(*Probe->GetTransform(), -m_Settings.Up, Hit.hitGeom, *Proxy->ShapeToWorld, &ProbePoint, &ProbeNormal, &Distance)
			|| !std::isfinite(Distance) || !Finite(ProbeNormal) || Distance < 0 || Distance > ProbeHeight * 2)
			return false;
		Normal = ProbeNormal.SafeUnit();
		return Normal.Dot(m_Settings.Up) >= m_Settings.SlopeLimitCosine;
	}

	bool SweepRobotController::TryStep(const Vector3& Displacement)
	{
		if (m_Settings.StepHeight <= 0 || Displacement.Length() < m_Settings.MinimumMoveDistance)
			return false;
		const Vector3 Start = GetPosition(), Up = m_Settings.Up;
		SweepResult Hit;
		if (Cast(Up * m_Settings.StepHeight, Hit))
			return false;
		SetPositionInternal(Start + Up * m_Settings.StepHeight);
		if (Cast(Displacement, Hit))
		{
			SetPositionInternal(Start);
			return false;
		}
		SetPositionInternal(GetPosition() + Displacement);
		const float DownDistance = m_Settings.StepHeight + m_Settings.GroundSnapDistance + m_Settings.SkinWidth;
		Vector3 SurfaceNormal, SurfacePoint;
		if (!Cast(-Up * DownDistance, Hit) || !StepSurface(Hit, SurfaceNormal, SurfacePoint))
		{
			SetPositionInternal(Start);
			return false;
		}
		const float FootHeight = (m_Geometry->GetSupport(-Up) - Up * m_Settings.StepHeight).Dot(Up);
		if (SurfacePoint.Dot(Up) - FootHeight > m_Settings.StepHeight + m_Settings.SkinWidth)
		{
			SetPositionInternal(Start);
			return false;
		}
		const Vector3 Landing = GetPosition() - Up * TravelDistance(-Up, Hit);
		if ((Landing - Start).Dot(Up) > m_Settings.StepHeight + m_Settings.RecoveryTolerance)
		{
			SetPositionInternal(Start);
			return false;
		}
		const GeometryQueryProxy* Proxy = m_World.GetGeometryQuery()->GetProxy(Hit.hitGeom);
		m_StepSupport = Proxy ? Proxy->Handle : GeometryHandle();
		SetPositionInternal(Landing); m_LastMove.Stepped = true; m_LastMove.HitBelow = true;
		return true;
	}

	void SweepRobotController::Slide(const Vector3& Displacement, bool Horizontal, bool AllowStep)
	{
		Vector3 Remaining = Displacement;
		std::vector<Vector3> Planes;
		for (int Iteration = 0; Iteration < m_Settings.SlideIterations; ++Iteration)
		{
			const float Length = Remaining.Length();
			if (Length < m_Settings.MinimumMoveDistance)
				return;
			const Vector3 Direction = Remaining / Length;
			SweepResult Hit;
			if (!Cast(Remaining, Hit))
			{
				SetPositionInternal(GetPosition() + Remaining);
				return;
			}
			const float Travel = std::min(Length, TravelDistance(Direction, Hit));
			SetPositionInternal(GetPosition() + Direction * Travel);
			Remaining -= Direction * Travel;
			Vector3 Normal = Hit.hitNormal.SafeUnit(); const float UpDot = Normal.Dot(m_Settings.Up);
			if (UpDot >= m_Settings.SlopeLimitCosine)
				m_LastMove.HitBelow = true;
			else if (UpDot <= -m_Settings.SlopeLimitCosine)
				m_LastMove.HitAbove = true;
			else
				m_LastMove.HitSide = true;
			if (Horizontal && AllowStep && UpDot < m_Settings.SlopeLimitCosine && TryStep(Remaining))
				return;
			if (Horizontal && UpDot > m_Settings.NormalTolerance && UpDot < m_Settings.SlopeLimitCosine)
				Normal = (Normal - m_Settings.Up * UpDot).SafeUnit();
			Planes.push_back(Normal);
			const Vector3 Before = Remaining;
			// Repeated projections also constrain corners with more than two planes.
			for (size_t Pass = 0; Pass < Planes.size(); ++Pass)
				for (const Vector3& Plane : Planes) Remaining -= Plane * std::min(0.0f, Remaining.Dot(Plane));
			if ((Before - Remaining).Length() < m_Settings.MinimumMoveDistance && Travel < m_Settings.MinimumMoveDistance)
				return;
		}
		m_LastMove.IterationLimitReached = Remaining.Length() >= m_Settings.MinimumMoveDistance;
	}

	void SweepRobotController::FindGround(float ProbeDistance, bool Snap)
	{
		m_Ground = RobotGround(); SweepResult Hit;
		if (!Cast(-m_Settings.Up * (ProbeDistance + m_Settings.SkinWidth / m_Settings.SlopeLimitCosine), Hit))
			return;
		const GeometryQueryProxy* Proxy = m_World.GetGeometryQuery()->GetProxy(Hit.hitGeom);
		m_Ground.Handle = Proxy ? Proxy->Handle : GeometryHandle();
		m_Ground.Normal = Hit.hitNormal.SafeUnit(); m_Ground.Position = Hit.hitPosition;
		m_Ground.Distance = Hit.hitTimeMin;
		bool Walkable = m_Ground.Normal.Dot(m_Settings.Up) >= m_Settings.SlopeLimitCosine;
		Vector3 SurfaceNormal, SurfacePoint;
		if (!Walkable && m_Ground.Handle == m_StepSupport && StepSurface(Hit, SurfaceNormal, SurfacePoint))
			Walkable = true;
		const float Travel = TravelDistance(-m_Settings.Up, Hit);
		const float Clearance = m_Settings.SweepTolerance / std::max(m_Settings.NormalTolerance, m_Ground.Normal.Dot(m_Settings.Up));
		if (Walkable && (Snap || Hit.hitTimeMin < Clearance))
			SetPositionInternal(GetPosition() - m_Settings.Up * (Hit.hitTimeMin - Clearance));
		m_Ground.Walkable = Walkable && (Snap || Travel <= m_Settings.RecoveryTolerance);
		if (m_Ground.Walkable)
			m_LastMove.HitBelow = true;
	}

	void SweepRobotController::FollowPlatform()
	{
		if (!m_Settings.FollowPlatforms || !m_Ground.Walkable)
			return;
		const GeometryWorldRecord* Record = m_World.GetGeometryWorldRecord(m_Ground.Handle);
		if (!Record || !Blocks(Record->State.Geom)
			|| !CollisionFilter::Allows(m_World.GetGeometryQuery()->GetDefaultCollisionFilter(), m_Settings.FilterData, Record->State.Geom->GetFilterData()))
		{
			m_Ground = RobotGround();
			return;
		}
		const Transform Current = Record->State.ShapeToWorld;
		const Vector3 Delta = Current.LocalToWorld(m_PlatformLocalPosition) - m_PlatformTransform.LocalToWorld(m_PlatformLocalPosition);
		const Vector3 Start = GetPosition();
		m_IgnoredPlatform = m_Ground.Handle;
		Slide(Delta, false, false);
		m_IgnoredPlatform = GeometryHandle();
		m_LastMove.PlatformDisplacement = GetPosition() - Start;
	}

	void SweepRobotController::RememberPlatform()
	{
		const GeometryWorldRecord* Record = m_Ground.Walkable ? m_World.GetGeometryWorldRecord(m_Ground.Handle) : nullptr;
		if (!Record)
			return;
		m_PlatformTransform = Record->State.ShapeToWorld;
		m_PlatformLocalPosition = m_PlatformTransform.WorldToLocal(GetPosition());
	}

	RobotMoveResult SweepRobotController::Move(const Vector3& Displacement, float Dt)
	{
		if (!Finite(Displacement) || !Nonnegative(Displacement.SquareLength()) || !Positive(Dt))
			throw std::invalid_argument("Invalid character displacement or time step");
		return MoveInternal(Displacement, Dt, false);
	}

	// Sweep the rising part even if this step ends below its starting height.
	// Return the descending displacement so lateral motion can precede landing.
	float SweepRobotController::AdvanceVertical(float Dt)
	{
		const float Gravity = m_Settings.Gravity;
		float Remaining = Dt;
		if (m_VerticalVelocity > 0)
		{
			const float Speed = m_VerticalVelocity;
			float Time = Gravity > 0 ? std::min(Dt, Speed / Gravity) : Dt;
			const float Rise = Time * (Speed - 0.5f * Gravity * Time);
			const Vector3 Start = GetPosition();
			Slide(m_Settings.Up * Rise, false, false);
			if (m_LastMove.HitAbove)
			{
				const float Travel = std::max(0.0f, (GetPosition() - Start).Dot(m_Settings.Up));
				const float ImpactSpeed = std::sqrt(std::max(0.0f, Speed * Speed - 2 * Gravity * Travel));
				Time = std::min(Time, 2 * Travel / (Speed + ImpactSpeed));
				m_VerticalVelocity = 0;
			}
			else
				m_VerticalVelocity = std::max(0.0f, Speed - Gravity * Time);
			Remaining -= Time;
		}
		const float Fall = Remaining * (m_VerticalVelocity - 0.5f * Gravity * Remaining);
		m_VerticalVelocity -= Gravity * Remaining;
		return std::min(0.0f, Fall);
	}

	RobotMoveResult SweepRobotController::MoveInternal(const Vector3& Displacement, float Dt, bool IntegrateGravity)
	{
		m_World.SynchronizeSceneQueries();
		const Vector3 Start = GetPosition(); m_LastMove = RobotMoveResult();
		const bool InFlight = IntegrateGravity && !IsGrounded();
		FollowPlatform(); RecoverOverlaps();
		if (!m_LastMove.RecoveryIncomplete)
		{
			const float Vertical = IntegrateGravity ? 0.0f : Displacement.Dot(m_Settings.Up);
			if (!InFlight)
				FindGround(m_Settings.RecoveryTolerance, false);
			const bool WasGrounded = IsGrounded();
			float Fall = std::min(0.0f, Vertical);
			if (IntegrateGravity)
				Fall = AdvanceVertical(Dt);
			else if (Vertical > 0)
				Slide(m_Settings.Up * Vertical, false, false);
			Slide(Displacement - m_Settings.Up * Vertical, true, WasGrounded && Vertical <= 0);
			if (Fall < 0)
				Slide(m_Settings.Up * Fall, false, false);
			if (InFlight && m_VerticalVelocity > 0)
				m_Ground = RobotGround();
			else
				FindGround(WasGrounded && Vertical <= 0 ? m_Settings.GroundSnapDistance : m_Settings.RecoveryTolerance,
					Vertical <= 0 && !InFlight);
			if (IntegrateGravity && IsGrounded())
				m_VerticalVelocity = 0;
			RememberPlatform();
		}
		else
			m_Ground = RobotGround();
		m_LastMove.Displacement = GetPosition() - Start;
		m_World.UpdateSceneQuery(m_World.UpdateGeometryWorldState(m_Geometry.get(), m_World.m_Clock.tick));
		return m_LastMove;
	}

	void SweepRobotController::Simulate(float Dt)
	{
		if (!m_Settings.AutomaticUpdate)
			return;
		MoveInternal(m_MoveVelocity * Dt, Dt, true);
	}
}
