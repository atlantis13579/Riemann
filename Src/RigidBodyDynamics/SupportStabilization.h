#pragma once

#include <vector>
#include <unordered_map>

namespace Riemann
{
	class Geometry;
	class RigidBody;
	class ContactManifold;
	class PositionConstraint;

	struct SupportStabilizationSettings
	{
		bool Enabled = false;
		int PositionIterations = 4, VelocityIterations = 4;
		float ParentInverseMassScale = 0.0f;
		float ContactTolerance = 0.001f;
		float MaximumPositionCorrection = 0.1f;
		float MaximumAngularCorrection = 0.2f;
		bool IsValid() const;
	};

	struct SupportStabilizationStats
	{
		int MaximumLevel = 0, ContactPointPasses = 0, JointPasses = 0, ContactBlockFallbacks = 0;
		// Largest artificial change during one substep, excluded from physical load reports.
		float MaximumPositionChange = 0, MaximumLinearVelocityChange = 0, MaximumAngularVelocityChange = 0;
	};

	class SupportGraph
	{
	public:
		struct Edge
		{
			RigidBody* A;
			RigidBody* B;
			ContactManifold* Contact;
			PositionConstraint* Joint;
			int LevelA = -1, LevelB = -1;
		};
		void Build(const std::vector<Geometry*>& Geometries, const std::vector<ContactManifold*>& Contacts,
			const std::vector<PositionConstraint*>& Joints, float ContactTolerance);
		int GetLevel(const RigidBody* Body) const;
		const std::vector<Edge>& GetEdges() const { return m_Edges; }
	private:
		std::unordered_map<const RigidBody*, int> m_Levels;
		std::vector<Edge> m_Edges;
	};

	void StabilizeSupports(const SupportGraph& Graph, const SupportStabilizationSettings& Settings,
		float Dt, SupportStabilizationStats& Stats);
}
