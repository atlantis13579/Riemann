#include "Island.h"
#include "CollidingContact.h"
#include "Constraints.h"
#include "Jacobian.h"
#include "RigidBody.h"
#include "../Collision/GeometryObject.h"
#include "../Core/Graph.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <unordered_set>

namespace Riemann
{
	/* Contacts and joints are edges of the same dynamic-body graph. Union by rank
	   with path compression builds connected components in near-linear time. A
	   compound contributes one node, even when several of its shapes have contacts.
	   Static, kinematic and frozen bodies are read-only boundary conditions: sharing
	   a floor or a driven platform does not couple otherwise independent islands.
	   Sleeping dynamic nodes remain present so wake propagation can use this graph.

	   Output order follows body/edge insertion, never unordered-map iteration. The
	   graph borrows pointers for one solve; it owns neither contacts nor joints.
	   Indexed edges let position solvers use their own persistent contact storage.

	   Velocity coupling is block Gauss-Seidel: a joint block consumes the latest
	   contact velocity and the next contact sweep consumes its result. Contact rows
	   and accumulated impulses survive every sweep; warm starting happens once.
	   Synchronization touches only joint endpoints. This allows the existing joint
	   block equations and contact solver backends to retain their numerical methods.
	*/
	namespace
	{
		const size_t InvalidNode = std::numeric_limits<size_t>::max();
	}

	void ConstraintGraph::Begin(const std::vector<RigidBodyDynamic*>& Bodies)
	{
		m_Nodes.clear();
		m_ReducedRagdollNodes.clear();
		m_Bodies.clear();
		m_Parents.clear();
		m_Ranks.clear();
		m_Edges.clear();
		m_Islands.clear();
		for (RigidBodyDynamic* Body : Bodies) Node(Body);
	}
	size_t ConstraintGraph::Node(RigidBody* Body)
	{
		RigidBodyDynamic* Dynamic = Body ? Body->CastDynamic() : nullptr;
		if (!Dynamic || Dynamic->Freezing) return InvalidNode;
		const auto Found = m_Nodes.find(Body);
		if (Found != m_Nodes.end()) return Found->second;
		const size_t Index = m_Bodies.size();
		m_Nodes.emplace(Body, Index);
		m_Bodies.push_back(Dynamic);
		m_Parents.push_back(Index);
		m_Ranks.push_back(0);
		if (Body->ReducedRagdoll)
		{
			const auto Owner = m_ReducedRagdollNodes.emplace(Body->ReducedRagdoll, Index);
			if (!Owner.second) Connect(Index, Owner.first->second);
		}
		return Index;
	}
	size_t ConstraintGraph::Root(size_t Index)
	{
		while (m_Parents[Index] != Index)
		{
			m_Parents[Index] = m_Parents[m_Parents[Index]];
			Index = m_Parents[Index];
		}
		return Index;
	}

	void ConstraintGraph::Connect(size_t A, size_t B)
	{
		if (A == InvalidNode || B == InvalidNode) return;
		A = Root(A);
		B = Root(B);
		if (A == B) return;
		if (m_Ranks[A] < m_Ranks[B]) std::swap(A, B);
		m_Parents[B] = A;
		if (m_Ranks[A] == m_Ranks[B]) ++m_Ranks[A];
	}

	void ConstraintGraph::AddContact(RigidBody* A, RigidBody* B, ContactManifold* Contact, size_t Index)
	{
		if (!A || !B || A == B) return;
		const size_t NodeA = Node(A), NodeB = Node(B);
		if (NodeA == InvalidNode && NodeB == InvalidNode) return;
		Connect(NodeA, NodeB);
		m_Edges.push_back({ NodeA, NodeB, Contact, nullptr, Index });
	}

	void ConstraintGraph::AddJoint(PositionConstraint* Joint)
	{
		if (!Joint || !Joint->IsEnabled() || Joint->GetBodyA() == Joint->GetBodyB()) return;
		const size_t A = Node(Joint->GetBodyA()), B = Node(Joint->GetBodyB());
		if (A == InvalidNode && B == InvalidNode) return;
		Connect(A, B);
		m_Edges.push_back({ A, B, nullptr, Joint, InvalidNode });
	}

	void ConstraintGraph::Finish()
	{
		m_Islands.clear();
		std::vector<size_t> Slots(m_Bodies.size(), InvalidNode);
		for (size_t Index = 0; Index < m_Bodies.size(); ++Index)
		{
			const size_t Parent = Root(Index);
			if (Slots[Parent] == InvalidNode)
			{
				Slots[Parent] = m_Islands.size();
				m_Islands.emplace_back();
			}
			m_Islands[Slots[Parent]].Bodies.push_back(m_Bodies[Index]);
		}
		for (const Edge& Constraint : m_Edges)
		{
			const size_t DynamicNode = Constraint.A != InvalidNode ? Constraint.A : Constraint.B;
			ConstraintIsland& Island = m_Islands[Slots[Root(DynamicNode)]];
			if (Constraint.Contact) Island.Contacts.push_back(Constraint.Contact);
			if (Constraint.Joint) Island.Joints.push_back(Constraint.Joint);
			if (Constraint.Index != InvalidNode) Island.ContactIndices.push_back(Constraint.Index);
		}
	}

	void ConstraintGraph::Build(const std::vector<Geometry*>& Geometries, const std::vector<ContactManifold*>& Contacts,
		const std::vector<PositionConstraint*>& Joints, const std::vector<RigidBodyDynamic*>& Bodies)
	{
		Begin(Bodies);
		for (ContactManifold* Contact : Contacts)
		{
			if (!Contact || Contact->NumContactPointCount <= 0 || Contact->indexA < 0 || Contact->indexB < 0
				|| (size_t)Contact->indexA >= Geometries.size() || (size_t)Contact->indexB >= Geometries.size()) continue;
			Geometry* A = Geometries[Contact->indexA];
			Geometry* B = Geometries[Contact->indexB];
			AddContact(A ? A->GetParent<RigidBody>() : nullptr, B ? B->GetParent<RigidBody>() : nullptr, Contact);
		}
		for (PositionConstraint* Joint : Joints) AddJoint(Joint);
		Finish();
	}

	JointVelocityPass::JointVelocityPass(const std::vector<RigidBody*>& Bodies, GeneralizedVelocity* Velocities,
		const std::vector<PositionConstraint*>& Joints) : m_Velocities(Velocities), m_Joints(Joints)
	{
		std::unordered_set<RigidBody*> Endpoints;
		for (PositionConstraint* Joint : Joints)
		{
			if (!Joint || !Joint->IsEnabled()) continue;
			Endpoints.insert(Joint->GetBodyA());
			Endpoints.insert(Joint->GetBodyB());
		}
		if (Endpoints.empty()) return;
		for (size_t Index = 0; Index < Bodies.size(); ++Index)
		{
			RigidBodyDynamic* Body = Bodies[Index] ? Bodies[Index]->CastDynamic() : nullptr;
			if (Body && !Body->Freezing && Endpoints.count(Body)) m_Slots.push_back({ Body, Index });
		}
	}

	float JointVelocityPass::Solve(float Dt)
	{
		for (const Slot& Entry : m_Slots)
		{
			Entry.Body->V = m_Velocities[Entry.Index].v;
			Entry.Body->W = m_Velocities[Entry.Index].w;
		}
		for (PositionConstraint* Joint : m_Joints)
			if (Joint && Joint->IsEnabled()) Joint->SolveVelocity(Dt);
		float Error = 0.0f;
		for (const Slot& Entry : m_Slots)
		{
			GeneralizedVelocity& Velocity = m_Velocities[Entry.Index];
			Error = std::max(Error, (Entry.Body->V - Velocity.v).SquareLength() + (Entry.Body->W - Velocity.w).SquareLength());
			Velocity.v = Entry.Body->V;
			Velocity.w = Entry.Body->W;
		}
		return Error;
	}

	/* Sleeping is a graph decision, not an independent energy clamp. Contacts and
	   enabled joints connect dynamic bodies; immovable boundaries never join two
	   islands through a shared floor. Contact edges survive while both endpoints
	   are inactive, because the broad phase deliberately omits those pairs. Once
	   either endpoint wakes, fresh collision results replace these cached edges.
	   Only body pairs are retained, never borrowed manifolds or joint pointers.

	   Quiet time advances below the low linear/angular speed thresholds, pauses in
	   the hysteresis band, and resets above the high thresholds. Every member must
	   qualify before the island sleeps. Timers use physical seconds and observe
	   every substep; the state transition occurs only at the outer frame boundary.
	   SleepThreshold <= 0 remains a per-body opt-out. Frozen bodies are boundaries.
	   Active drives, compliant XPBD rows, pending loads and moving supports veto
	   sleep conservatively. Gravity alone does not refresh the inactivity timer.

	   Prepare observes edits before integration. New touching contacts propagate
	   wakefulness before the contact solve; a newly hit sleeper receives gravity
	   from the following substep, while its impact response uses physical mass now.
	   Removal and filter invalidation must wake old neighbors before deleting edges.
	   This is a CPU lifecycle layer shared by SI/LCP/PBD/XPBD and their backends.
	*/
	namespace
	{
		RigidBodyDynamic* Movable(RigidBody* Body)
		{
			auto* Dynamic = Body ? Body->CastDynamic() : nullptr;
			return Dynamic && !Dynamic->Freezing && Dynamic->InvMass > 0 ? Dynamic : nullptr;
		}
		bool Finite(const Vector3& Value)
		{ return std::isfinite(Value.x) && std::isfinite(Value.y) && std::isfinite(Value.z); }
		bool SameRotation(const Quaternion& A, const Quaternion& B)
		{
			return (A.x == B.x && A.y == B.y && A.z == B.z && A.w == B.w)
				|| (A.x == -B.x && A.y == -B.y && A.z == -B.z && A.w == -B.w);
		}
	}
	bool IslandSleepingSettings::IsValid() const
	{
		return std::isfinite(QuietTime) && QuietTime > 0 && std::isfinite(LinearThreshold) && LinearThreshold > 0
			&& std::isfinite(AngularThreshold) && AngularThreshold > 0
			&& std::isfinite(HysteresisRatio) && HysteresisRatio > 1
			&& std::isfinite(LinearThreshold * HysteresisRatio) && std::isfinite(AngularThreshold * HysteresisRatio)
			&& std::isfinite(ContactTolerance) && ContactTolerance >= 0;
	}
	void IslandSleeping::Reset() { m_States.clear(); m_Contacts.clear(); m_Stats = IslandSleepingStats(); }
	void IslandSleeping::Configure(const IslandSleepingSettings& Settings)
	{
		if (!Settings.IsValid()) throw std::invalid_argument("Invalid island sleeping settings");
		Reset(); m_Settings = Settings;
	}
	float IslandSleeping::GetQuietTime(const RigidBodyDynamic* Body) const
	{
		const auto Found = m_States.find(const_cast<RigidBodyDynamic*>(Body));
		return Found == m_States.end() ? 0 : Found->second.QuietTime;
	}
	void IslandSleeping::Capture(RigidBody* Body, State& Record)
	{
		Record.Position = Body->X; Record.Rotation = Body->Q;
		Record.LinearVelocity = Body->V; Record.AngularVelocity = Body->W;
		auto* Dynamic = Body->CastDynamic();
		Record.Sleeping = Dynamic && Dynamic->Sleeping; Record.Frozen = Dynamic && Dynamic->Freezing;
		Record.WakeRevision = Dynamic ? Dynamic->GetWakeRevision() : 0;
		Record.Initialized = true;
	}
	bool IslandSleeping::IsActive(RigidBody* Body) const
	{
		const auto Found = m_States.find(Body);
		return Found != m_States.end() && Found->second.Active;
	}
	void IslandSleeping::Activate(RigidBody* Body)
	{
		if (Body) { auto& Record = m_States[Body]; Record.Active = true; Record.QuietTime = 0; }
	}
	void IslandSleeping::Observe(RigidBody* Body)
	{
		if (!Body) return;
		auto& Record = m_States[Body]; auto* Dynamic = Body->CastDynamic();
		if (Record.Initialized && (Record.Position != Body->X || !SameRotation(Record.Rotation, Body->Q)
			|| (Dynamic && (Record.Sleeping != Dynamic->Sleeping || Record.Frozen != Dynamic->Freezing
				|| Record.WakeRevision != Dynamic->GetWakeRevision() || Record.LinearVelocity != Body->V || Record.AngularVelocity != Body->W))))
			Activate(Body);
		if (Dynamic && (Dynamic->ExtForce != Vector3::Zero() || Dynamic->ExtTorque != Vector3::Zero()
			|| (Dynamic->Sleeping && (Body->V != Vector3::Zero() || Body->W != Vector3::Zero())))) Activate(Body);
		if (Body->mRigidType == RigidType::Kinematic && (Body->V != Vector3::Zero() || Body->W != Vector3::Zero())) Activate(Body);
		if (!Record.Initialized)
		{
			if (!Movable(Body)) Activate(Body);
			Capture(Body, Record);
		}
	}
	void IslandSleeping::Prepare(const std::vector<RigidBodyDynamic*>& Bodies, const std::vector<PositionConstraint*>& Joints,
		uint64_t Step, bool Drives, bool SoftJoints)
	{
		m_Step = Step; m_Drives = Drives; m_SoftJoints = SoftJoints;
		for (auto* Body : Bodies) Observe(Body);
		for (const auto& Entry : m_Contacts)
		{
			Observe(Entry.first.A); Observe(Entry.first.B);
			if (!Entry.first.A->CanCollideWith(Entry.first.B))
			{
				if (Movable(Entry.first.A)) Activate(Entry.first.A);
				if (Movable(Entry.first.B)) Activate(Entry.first.B);
			}
		}
		for (auto* Joint : Joints) if (Joint) { Observe(Joint->GetBodyA()); Observe(Joint->GetBodyB()); }
		Propagate(Bodies, Joints);
		for (auto It = m_Contacts.begin(); It != m_Contacts.end();)
			if (!It->first.A->CanCollideWith(It->first.B)) It = m_Contacts.erase(It); else ++It;
	}
	void IslandSleeping::Propagate(const std::vector<RigidBodyDynamic*>& Bodies, const std::vector<PositionConstraint*>& Joints)
	{
		ConstraintGraph Graph; Graph.Begin(Bodies);
		auto Boundary = [&](RigidBody* A, RigidBody* B)
		{
			if (!Movable(A) && m_States[A].Active) Activate(B);
			if (!Movable(B) && m_States[B].Active) Activate(A);
		};
		for (const auto& Entry : m_Contacts)
		{
			Graph.AddContact(Entry.first.A, Entry.first.B);
			Boundary(Entry.first.A, Entry.first.B);
		}
		for (auto* Joint : Joints) if (Joint && Joint->IsEnabled())
		{
			Graph.AddJoint(Joint); Boundary(Joint->GetBodyA(), Joint->GetBodyB());
			if ((m_Drives && Joint->HasActiveDrive()) || (m_SoftJoints
				&& (Joint->GetLinearCompliance() > 0 || Joint->GetAngularCompliance() > 0)))
			{ if (Movable(Joint->GetBodyA())) Activate(Joint->GetBodyA()); if (Movable(Joint->GetBodyB())) Activate(Joint->GetBodyB()); }
		}
		Graph.Finish();
		for (const auto& Island : Graph.GetIslands())
		{
			bool Active = false, Awake = false, Asleep = false;
			for (auto* Body : Island.Bodies)
			{
				Active = Active || m_States[Body].Active;
				Awake = Awake || !Body->Sleeping; Asleep = Asleep || Body->Sleeping;
			}
			if (!Active && !(Awake && Asleep)) continue;
			for (auto* Body : Island.Bodies)
			{
				Activate(Body);
				if (Body->Sleeping) { Body->Wakeup(); ++m_Stats.WakeTransitions; }
			}
		}
	}
	void IslandSleeping::AddSupport(RigidBody* A, RigidBody* B)
	{
		// Query-wheel supports share contact lifetime and wake propagation without
		// manufacturing rigid contact manifolds or retaining transient shape pointers.
		if (!A || !B || A == B || (!Movable(A) && !Movable(B))) return;
		if (std::less<RigidBody*>()(B, A)) std::swap(A, B);
		m_Contacts[{ A, B }] = m_Step;
		for (auto* Body : { A, B }) if (!m_States[Body].Initialized) Capture(Body, m_States[Body]);
	}
	void IslandSleeping::AddContacts(const std::vector<Geometry*>& Geometries, const std::vector<ContactManifold*>& Contacts,
		const std::vector<RigidBodyDynamic*>& Bodies, const std::vector<PositionConstraint*>& Joints)
	{
		for (auto* Contact : Contacts)
		{
			if (!Contact || Contact->indexA < 0 || Contact->indexB < 0 || (size_t)Contact->indexA >= Geometries.size()
				|| (size_t)Contact->indexB >= Geometries.size()) continue;
			auto* A = Geometries[Contact->indexA]; auto* B = Geometries[Contact->indexB];
			if (!A || !B || A->IsTriggerEnabled() || B->IsTriggerEnabled() || !A->IsSimulationEnabled() || !B->IsSimulationEnabled()) continue;
			bool Touching = false;
			for (int I = 0; I < Contact->NumContactPointCount; ++I)
				Touching = Touching || Contact->ContactPoints[I].PenetrationDepth >= -m_Settings.ContactTolerance;
			Pair Key = { A->GetParent<RigidBody>(), B->GetParent<RigidBody>() };
			if (!Touching || !Key.A || !Key.B || Key.A == Key.B || (!Movable(Key.A) && !Movable(Key.B))) continue;
			if (std::less<RigidBody*>()(Key.B, Key.A)) std::swap(Key.A, Key.B);
			m_Contacts[Key] = m_Step;
			for (auto* Body : { Key.A, Key.B }) if (!m_States[Body].Initialized) Capture(Body, m_States[Body]);
		}
		Propagate(Bodies, Joints);
	}
	void IslandSleeping::Finish(const std::vector<RigidBodyDynamic*>& Bodies, const std::vector<PositionConstraint*>& Joints,
		float Dt, bool FinalSubstep, bool SoftContacts)
	{
		// An impact or callback can wake an endpoint after the last discrete solve.
		// Propagate through the old graph before retiring its now-active edges.
		Propagate(Bodies, Joints);
		for (auto It = m_Contacts.begin(); It != m_Contacts.end();)
		{
			auto* A = Movable(It->first.A); auto* B = Movable(It->first.B);
			if (It->second != m_Step && ((A && !A->Sleeping) || (B && !B->Sleeping))) It = m_Contacts.erase(It);
			else ++It;
		}
		Propagate(Bodies, Joints);
		ConstraintGraph Graph; Graph.Begin(Bodies);
		for (const auto& Entry : m_Contacts) Graph.AddContact(Entry.first.A, Entry.first.B);
		for (auto* Joint : Joints) Graph.AddJoint(Joint);
		Graph.Finish();
		m_Stats.IslandCount = (int)Graph.GetIslands().size();
		m_Stats.SleepingIslandCount = m_Stats.SleepingBodyCount = 0;
		for (const auto& Island : Graph.GetIslands())
		{
			bool Ready = true;
			for (auto* Body : Island.Bodies)
			{
				auto& Record = m_States[Body];
				// Pose corrections can move a constraint even when its final velocity is zero.
				const Quaternion RotationDelta = Body->Q * Record.Rotation.Conjugate();
				const float RotationAngle = 2 * std::atan2(Vector3(RotationDelta.x, RotationDelta.y, RotationDelta.z).Length(), std::abs(RotationDelta.w));
				const float Linear = std::max(Body->V.Length(), (Body->X - Record.Position).Length() / Dt);
				const float Angular = std::max(Body->W.Length(), RotationAngle / Dt);
				if (Record.Active || Body->SleepThreshold <= 0 || SoftContacts || !Finite(Body->V) || !Finite(Body->W)
					|| !Finite(Body->X) || !std::isfinite(RotationAngle)
					|| Linear > m_Settings.LinearThreshold * m_Settings.HysteresisRatio
					|| Angular > m_Settings.AngularThreshold * m_Settings.HysteresisRatio) Record.QuietTime = 0;
				else if (Linear <= m_Settings.LinearThreshold && Angular <= m_Settings.AngularThreshold)
					Record.QuietTime = std::min(m_Settings.QuietTime, Record.QuietTime + Dt);
				Ready = Ready && Record.QuietTime >= m_Settings.QuietTime;
			}
			bool Asleep = true;
			for (auto* Body : Island.Bodies)
			{
				if (Ready && FinalSubstep && !Body->Sleeping) { Body->Sleep(); ++m_Stats.SleepTransitions; }
				Asleep = Asleep && Body->Sleeping;
				if (Body->Sleeping) ++m_Stats.SleepingBodyCount;
			}
			if (Asleep) ++m_Stats.SleepingIslandCount;
		}
		for (auto& Entry : m_States) { Capture(Entry.first, Entry.second); Entry.second.Active = false; }
	}
	void IslandSleeping::Forget(RigidBody* Body)
	{
		for (auto It = m_Contacts.begin(); It != m_Contacts.end();)
		{
			if (It->first.A == Body || It->first.B == Body)
			{
				RigidBody* Other = It->first.A == Body ? It->first.B : It->first.A;
				Activate(Other); if (auto* Dynamic = Movable(Other)) Dynamic->Wakeup();
				It = m_Contacts.erase(It);
			}
			else ++It;
		}
		m_States.erase(Body);
	}

	void ContactManifoldIslands::BuildIslands(const std::vector<Geometry*>& geoms, const std::vector<ContactManifold*>& manifolds)
	{
		GraphNE<RigidBody*, ContactManifold*> graph;
		std::vector<int> GeometryBodyNodes(geoms.size(), -1);
		std::unordered_map<RigidBody*, int> BodyNodes;
		BodyNodes.reserve(geoms.size());
		for (size_t GeometryIndex = 0; GeometryIndex < geoms.size(); ++GeometryIndex)
		{
			RigidBody* Body = geoms[GeometryIndex] ? geoms[GeometryIndex]->GetParent<RigidBody>() : nullptr;
			if (!Body)
			{
				continue;
			}

			auto It = BodyNodes.find(Body);
			if (It == BodyNodes.end())
			{
				const int BodyNode = (int)graph.nodes.size();
				BodyNodes.emplace(Body, BodyNode);
				graph.nodes.push_back(Body);
				GeometryBodyNodes[GeometryIndex] = BodyNode;
			}
			else
			{
				GeometryBodyNodes[GeometryIndex] = It->second;
			}
		}
		for (size_t i = 0; i < manifolds.size(); ++i)
		{
			ContactManifold* Manifold = manifolds[i];
			assert(Manifold != nullptr);
			assert(Manifold->indexA >= 0 && (size_t)Manifold->indexA < GeometryBodyNodes.size());
			assert(Manifold->indexB >= 0 && (size_t)Manifold->indexB < GeometryBodyNodes.size());
			const int NodeA = GeometryBodyNodes[(size_t)Manifold->indexA];
			const int NodeB = GeometryBodyNodes[(size_t)Manifold->indexB];
			assert(NodeA >= 0 && NodeB >= 0);
			graph.AddEdge(NodeA, NodeB, Manifold);
		}
		std::vector<bool> separate;
		separate.resize(graph.nodes.size());
		for (size_t i = 0; i < separate.size(); ++i)
		{
			separate[i] = graph.nodes[i]->mRigidType == RigidType::Static;
		}

		graph.BuildEdgeIslands(separate, &islands);

		size_t count = 0;
		for (size_t i = 0; i < islands.size(); ++i)
		{
			count += islands[i].size();
		}
		assert(count == manifolds.size());

		return;
	}
}
