#pragma once

#include <vector>
#include "../Maths/Box3.h"
#include "../CollisionPrimitive/Ray3d.h"

namespace Riemann
{
	struct RayCastOption;
	struct RayCastResult;
	struct IntersectOption;
	struct IntersectResult;
	struct SweepOption;
	struct SweepResult;
	class GeometryBase;

	class DynamicAABBTree
	{
	public:
		static constexpr float kAABBThickness = 0.1f;
		static constexpr float kAABBFattenScale = 4.0f;

		struct Node
		{
			inline bool IsLeaf() const
			{
				return child1 == -1;
			}

			union
			{
				int parent;
				int next;
			};
			Box3 aabb;
			int child1;
			int child2;
			int height;
			void* userData;
			bool moved;
		};

		DynamicAABBTree();
		~DynamicAABBTree();

		int 	Add(const Box3& aabb, void* userData);
		void 	Remove(int nodeId);
		bool 	Update(int nodeId, const Box3& aabb, const Vector3& displacement);

		bool	RayCast(const Ray3d& Ray, const RayCastOption* Option, RayCastResult* Result) const;
		bool	Intersect(const GeometryBase* geometry, const IntersectOption* Option, IntersectResult* Result) const;
		bool	Sweep(const GeometryBase* geometry, const Vector3& Direction, const SweepOption* Option, SweepResult* Result) const;
		bool	Query(const Box3& aabb, std::vector<void*>* Result) const;

		void 	Rebuild();
		bool 	Validate() const;

	private:
		int 	AllocNode();
		void 	FreeNode(int nodeId);
		void 	InsertLeaf(int leaf);
		void 	RemoveLeaf(int leaf);
		int 	Balance(int nodeA);

		int 	GetHeight() const;
		int 	ComputeHeight(int nodeId) const;
		int 	ComputeHeight() const;
		float	CalculateVolumeRatio() const;
		int 	GetMaxBalance() const;

		bool 	ValidateRecursive(int nodeId) const;
		bool 	ValidateAABBs(int nodeId) const;

	private:
		int m_root;
		std::vector<Node> m_nodes;
		int m_nodeCount;
		int m_nodeCapacity;
		int m_freeList;
	};
}