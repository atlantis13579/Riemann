#include "ArticulatedRobotController.h"
#include "SparseJointSpaceDynamics.h"
#include "../../RigidBodyDynamics/PhysicsWorld.h"
#include "../../RigidBodyDynamics/NarrowPhase.h"
#include <algorithm>
#include <cmath>
#include <numeric>
#include <stdexcept>

namespace Riemann
{
	namespace
	{
		struct ContactAxis
		{
			std::vector<double> Jacobian, Response;
			double InverseMass, Impulse = 0;

			void Solve(std::vector<double>& Velocity, double Target, double Limit, bool Normal)
			{
				const double Speed = std::inner_product(Jacobian.begin(), Jacobian.end(), Velocity.begin(), 0.0);
				const double Next = std::max(Normal ? 0.0 : -Limit, std::min(Limit, Impulse + (Target - Speed) / InverseMass));
				for (size_t J = 0; J < Velocity.size(); ++J) Velocity[J] += Response[J] * (Next - Impulse);
				Impulse = Next;
			}
		};
		struct RobotContactConstraint
		{
			ContactAxis Axes[3];
			double Target;
			bool Foot;
		};
	}

	ArticulatedRobotController::ArticulatedRobotController(PhysicsWorld& World, const ArticulatedRobotSettings& Settings)
		: m_World(World), m_Settings(Settings)
	{
		if (!std::isfinite(Settings.TimeStep) || Settings.TimeStep <= 0 || !std::isfinite(Settings.Friction)
			|| Settings.Friction < 0 || !std::isfinite(Settings.Gravity.SquareLength()) || Settings.Gravity.SquareLength() == 0
			|| Settings.Links.empty() || Settings.Links.front().Joint != RagdollReducedJointType::Floating)
			throw std::invalid_argument("Invalid floating robot settings");
		for (const RobotFootContact& Foot : Settings.Feet)
			if (Foot.Link >= Settings.Links.size() || !std::isfinite(Foot.Radius) || Foot.Radius <= 0 || !std::isfinite(Foot.Center.SquareLength()))
				throw std::invalid_argument("Invalid robot foot");
		switch (Settings.Backend)
		{
		case RobotDynamicsBackend::Featherstone:
			m_Dynamics = std::make_unique<FeatherstoneSolver>(Settings.Links);
			break;
		case RobotDynamicsBackend::SparseJointSpaceDynamics:
			m_Dynamics = std::make_unique<SparseJointSpaceDynamics>(Settings.Links);
			break;
		default:
			throw std::invalid_argument("Unknown articulated robot backend");
		}
		m_Dynamics->SetConfiguration(Settings.Pose);
	}

	ArticulatedRobotController::~ArticulatedRobotController() = default;

	void ArticulatedRobotController::AddLinkCollider(size_t Link, const Transform& Local, std::unique_ptr<Geometry> Shape)
	{
		if (Link >= m_Dynamics->GetLinkCount() || !Shape || !std::isfinite(Local.pos.SquareLength()) ||
			!std::isfinite(Local.quat.SquareLength()) || Local.quat.SquareLength() < 1e-12f)
			throw std::invalid_argument("Invalid robot link collider");
		if (!m_Contacts)
		{
			m_Contacts.reset(NarrowPhase::Create_GJKEPA());
			ContactGenerationSettings Settings; Settings.UseShapeOffsets = false;
			m_Contacts->SetContactGeneration(Settings, (float)m_Settings.TimeStep);
			m_Contacts->SetTriangleContactsEnabled(true);
		}
		m_Colliders.push_back({Link, Transform(Local.pos, Local.quat.Unit()), std::move(Shape)});
	}

	void ArticulatedRobotController::GatherBodyContacts(std::vector<ContactPoint>& Points)
	{
		GeometryQuery* Query = m_World.GetGeometryQuery();
		IntersectOption Option; Option.BoundsOnly = true; Option.maxOverlaps = UINT_MAX;
		Option.PreFilter = [](const Geometry* G)
		{
			const RigidBody* Body = G->GetParent<RigidBody>();
			return G->IsSimulationEnabled() && !G->IsTriggerEnabled() && (!Body || Body->mRigidType == RigidType::Static);
		};
		for (Collider& C : m_Colliders)
		{
			const Transform Link = m_Dynamics->GetLinkTransform(C.Link), World = Link * C.Local;
			C.Shape->SetTransform(World.pos, World.quat);
			const Box3 Bounds = C.Shape->GetBounds();
			IntersectResult Hits;
			if (!Query->IntersectQueryBox(Bounds.GetCenter(), Bounds.GetExtent(), Option, &Hits))
				continue;
			GeometryWorldState Body;
			Body.Geom = C.Shape.get(); Body.ShapeToWorld = World; Body.WorldBounds = Bounds;
			std::vector<GeometryWorldState> States{Body};
			std::vector<OverlapPair> Pairs;
			for (Geometry* Shape : Hits.overlapGeoms)
			{
				const GeometryQueryProxy* Proxy = Query->GetProxy(Shape);
				if (!Proxy || !Proxy->ShapeToWorld || !Proxy->WorldBounds || !Bounds.Intersect(*Proxy->WorldBounds))
					continue;
				GeometryWorldState Other;
				Other.Geom = Shape; Other.Body = Shape->GetParent<RigidBody>();
				Other.ShapeToWorld = *Proxy->ShapeToWorld; Other.WorldBounds = *Proxy->WorldBounds;
				Pairs.emplace_back(0, (int)States.size());
				States.push_back(Other);
			}
			if (Pairs.empty())
				continue;
			std::vector<const GeometryWorldState*> Ptrs;
			for (const GeometryWorldState& State : States) Ptrs.push_back(&State);
			std::vector<ContactManifold*> Manifolds;
			m_Contacts->CollisionDetection(GeometryWorldStateSpan(Ptrs.data(), Ptrs.size()), Pairs, &Manifolds);
			for (const ContactManifold* M : Manifolds) for (int I = 0; I < M->NumContactPointCount; ++I)
			{
				const Contact& P = M->ContactPoints[I];
				if (P.PenetrationDepth > 0 && P.Normal.SquareLength() > 0.5f)
					Points.push_back({C.Link, Link.WorldToLocal(P.PositionWorldA), -P.Normal.Unit(), -P.PenetrationDepth, false});
			}
		}
	}

	void ArticulatedRobotController::SetMoveVelocity(const Vector3& WorldVelocity)
	{
		if (!std::isfinite(WorldVelocity.SquareLength()))
			throw std::invalid_argument("Invalid robot velocity command");
		m_Command = WorldVelocity;
	}

	void ArticulatedRobotController::Simulate(float Dt)
	{
		m_Remainder += Dt;
		while (m_Remainder + 1.0e-12 >= m_Settings.TimeStep)
		{
			Step();
			++m_Tick;
			m_Remainder -= m_Settings.TimeStep;
		}
	}

	void ArticulatedRobotController::Step()
	{
		const double Dt = m_Settings.TimeStep;
		std::vector<double> Torque(m_Dynamics->GetDofCount(), 0);
		if (m_Settings.Actuate)
			m_Settings.Actuate(*m_Dynamics, m_Command, m_Tick, Torque);
		const std::vector<double> Acceleration = m_Dynamics->ComputeAcceleration(Torque, m_Settings.Gravity);
		std::vector<double> Velocity = m_Dynamics->GetVelocity();
		for (size_t J = 0; J < Velocity.size(); ++J) Velocity[J] += Dt * Acceleration[J];

		const Vector3 Up = -m_Settings.Gravity.Unit();
		std::vector<ContactPoint> Points;
		for (const RobotFootContact& Foot : m_Settings.Feet)
		{
			const Transform Link = m_Dynamics->GetLinkTransform(Foot.Link);
			const Vector3 Center = Link * Foot.Center;
			constexpr float ProbeHeight = 0.06f, Margin = 0.002f;
			SweepOption Option;
			Option.MaxDist = ProbeHeight + Margin;
			Option.PreFilter = [](const Geometry* G) { return G->IsSimulationEnabled() && !G->IsTriggerEnabled(); };
			Option.PostFilter = [&](const Geometry*, const Vector3&, const Vector3& N, float) { return N.Dot(Up) > 0.1f; };
			SweepResult Hit;
			if (!m_World.GetGeometryQuery()->SphereCastQuery(Center + Up * ProbeHeight, Foot.Radius, -Up, Option, &Hit))
				continue;
			const Vector3 Normal = Hit.hitNormal.Unit();
			const float Gap = (Hit.hitTimeMin - ProbeHeight) * Normal.Dot(Up);
			const Vector3 Local = Foot.Center - Link.quat.Conjugate() * (Normal * Foot.Radius);
			Points.push_back({Foot.Link, Local, Normal, Gap, true});
		}
		GatherBodyContacts(Points);
		std::vector<RobotContactConstraint> Contacts;
		for (const ContactPoint& P : Points)
		{
			const RagdollMatrix Jp = m_Dynamics->ComputeLinkJacobian(P.Link, P.Local);
			const Vector3 Normal = P.Normal;
			const Vector3 Side = Normal.Cross(std::abs(Normal.x) < 0.8f ? Vector3::UnitX() : Vector3::UnitZ()).Unit();
			const Vector3 Directions[] = {Normal, Side, Normal.Cross(Side)};
			RobotContactConstraint Contact;
			Contact.Foot = P.Foot;
			Contact.Target = -P.Gap * (P.Gap >= 0 ? 1.0 : 0.15) / Dt;
			for (int A = 0; A < 3; ++A)
			{
				ContactAxis& Axis = Contact.Axes[A];
				Axis.Jacobian.resize(Velocity.size());
				for (size_t J = 0; J < Velocity.size(); ++J)
					Axis.Jacobian[J] = Jp[3][J] * Directions[A].x + Jp[4][J] * Directions[A].y + Jp[5][J] * Directions[A].z;
				Axis.Response = m_Dynamics->ComputeImpulseResponse({}, Axis.Jacobian);
				Axis.InverseMass = std::inner_product(Axis.Jacobian.begin(), Axis.Jacobian.end(), Axis.Response.begin(), 0.0);
			}
			Contacts.push_back(std::move(Contact));
		}
		// Project unilateral normal and tangential contact impulses onto the
		// friction pyramid. Every impulse changes the velocity of the whole tree.
		for (int Pass = 0; Pass < 12; ++Pass)
			for (RobotContactConstraint& C : Contacts)
			{
				C.Axes[0].Solve(Velocity, C.Target, std::numeric_limits<double>::max(), true);
				const double Limit = m_Settings.Friction * C.Axes[0].Impulse;
				C.Axes[1].Solve(Velocity, 0, Limit, false);
				C.Axes[2].Solve(Velocity, 0, Limit, false);
			}
		m_Supported = std::any_of(Contacts.begin(), Contacts.end(), [](const RobotContactConstraint& C) { return C.Foot && C.Axes[0].Impulse > 0; });
		m_Dynamics->SetVelocity(Velocity);
		m_Dynamics->IntegrateConfiguration(Dt);
	}
}
