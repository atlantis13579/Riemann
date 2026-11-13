#pragma once

#include <limits>
#include <unordered_map>
#include <vector>

#include "RigidBody.h"

namespace Riemann
{
	class Geometry;
	class RigidBody;
	class RigidBodyDynamic;
	class PositionConstraint;
	class ContactManifold;
	struct GeneralizedVelocity;

	struct ConstraintIsland
	{
		std::vector<RigidBodyDynamic*> Bodies;
		std::vector<ContactManifold*> Contacts;
		std::vector<size_t> ContactIndices;
		std::vector<PositionConstraint*> Joints;
	};

	class ConstraintGraph
	{
	public:
		void Begin(const std::vector<RigidBodyDynamic*>& Bodies = {});
		void AddContact(RigidBody* A, RigidBody* B, ContactManifold* Contact = nullptr,
			size_t Index = std::numeric_limits<size_t>::max());
		void AddJoint(PositionConstraint* Joint);
		void Finish();
		void Build(const std::vector<Geometry*>& Geometries, const std::vector<ContactManifold*>& Contacts,
			const std::vector<PositionConstraint*>& Joints, const std::vector<RigidBodyDynamic*>& Bodies = {});
		std::vector<ConstraintIsland>& GetIslands() { return m_Islands; }
		const std::vector<ConstraintIsland>& GetIslands() const { return m_Islands; }

	private:
		struct Edge
		{
			size_t A, B;
			ContactManifold* Contact;
			PositionConstraint* Joint;
			size_t Index;
		};
		size_t Node(RigidBody* Body);
		size_t Root(size_t Index);
		void Connect(size_t A, size_t B);
		std::unordered_map<RigidBody*, size_t> m_Nodes;
		std::unordered_map<const void*, size_t> m_ReducedRagdollNodes;
		std::vector<RigidBodyDynamic*> m_Bodies;
		std::vector<size_t> m_Parents, m_Ranks;
		std::vector<Edge> m_Edges;
		std::vector<ConstraintIsland> m_Islands;
	};

	class JointVelocityPass
	{
	public:
		JointVelocityPass(const std::vector<RigidBody*>& Bodies, GeneralizedVelocity* Velocities,
			const std::vector<PositionConstraint*>& Joints);
		float Solve(float Dt);
	private:
		struct Slot { RigidBodyDynamic* Body; size_t Index; };
		std::vector<Slot> m_Slots;
		GeneralizedVelocity* m_Velocities;
		const std::vector<PositionConstraint*>& m_Joints;
	};

	struct IslandSleepingSettings
	{
		bool Enabled = false;
		float QuietTime = 0.5f;
		float LinearThreshold = 0.03f;
		float AngularThreshold = 0.03f;
		float HysteresisRatio = 2.0f;
		float ContactTolerance = 0.002f;
		bool IsValid() const;
	};
	struct IslandSleepingStats
	{
		int IslandCount = 0, SleepingIslandCount = 0, SleepingBodyCount = 0;
		int SleepTransitions = 0, WakeTransitions = 0;
	};
	class IslandSleeping
	{
	public:
		void Reset();
		void Configure(const IslandSleepingSettings& Settings);
		const IslandSleepingSettings& GetSettings() const { return m_Settings; }
		const IslandSleepingStats& GetStats() const { return m_Stats; }
		float GetQuietTime(const RigidBodyDynamic* Body) const;
		void BeginFrame() { m_Stats = IslandSleepingStats(); }
		void Prepare(const std::vector<RigidBodyDynamic*>& Bodies, const std::vector<PositionConstraint*>& Joints,
			uint64_t Step, bool Drives, bool SoftJoints);
		void AddContacts(const std::vector<Geometry*>& Geometries, const std::vector<ContactManifold*>& Contacts,
			const std::vector<RigidBodyDynamic*>& Bodies, const std::vector<PositionConstraint*>& Joints);
		void Activate(RigidBody* Body);
		void AddSupport(RigidBody* A, RigidBody* B);
		void Observe(RigidBody* Body);
		bool IsActive(RigidBody* Body) const;
		void Propagate(const std::vector<RigidBodyDynamic*>& Bodies, const std::vector<PositionConstraint*>& Joints);
		void Finish(const std::vector<RigidBodyDynamic*>& Bodies, const std::vector<PositionConstraint*>& Joints,
			float Dt, bool FinalSubstep, bool SoftContacts);
		void Forget(RigidBody* Body);
	private:
		struct State
		{
			Vector3 Position, LinearVelocity, AngularVelocity;
			Quaternion Rotation;
			uint64_t WakeRevision = 0;
			float QuietTime = 0;
			bool Initialized = false, Sleeping = false, Frozen = false, Active = false;
		};
		struct Pair
		{
			RigidBody* A; RigidBody* B;
			bool operator==(const Pair& Other) const { return A == Other.A && B == Other.B; }
		};
		struct PairHash
		{
			size_t operator()(const Pair& Value) const
			{ return std::hash<RigidBody*>()(Value.A) ^ (std::hash<RigidBody*>()(Value.B) << 1); }
		};
		void Capture(RigidBody* Body, State& Record);
		IslandSleepingSettings m_Settings;
		IslandSleepingStats m_Stats;
		std::unordered_map<RigidBody*, State> m_States;
		std::unordered_map<Pair, uint64_t, PairHash> m_Contacts;
		uint64_t m_Step = 0;
		bool m_Drives = true, m_SoftJoints = false;
	};

	class ContactManifoldIslands
	{
	public:
		void BuildIslands(const std::vector<Geometry*>& geoms, const std::vector<ContactManifold*>& manifolds);

		std::vector<std::vector<ContactManifold*>> islands;
	};
}
