#pragma once

#include <vector>
#include "../Maths/Box3.h"
#include "../CollisionPrimitive/Ray3.h"

namespace Riemann
{
	struct RayCastOption;
	struct RayCastResult;
	struct IntersectOption;
	struct IntersectResult;
	struct SweepOption;
	struct SweepResult;
	class Geometry;

	typedef bool (*DynamicRayCastCallback)(const Ray3& Ray, void* userData, const RayCastOption* Option, RayCastResult* Result, void* Context);
	typedef bool (*DynamicIntersectCallback)(const Geometry* geometry, void* userData, const IntersectOption* Option, IntersectResult* Result, void* Context);
	typedef bool (*DynamicSweepCallback)(const Geometry* geometry, const Vector3& Direction, void* userData, const SweepOption* Option, SweepResult* Result, void* Context);

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

		void	Clear();
		int 	Add(const Box3& aabb, void* userData);
		void 	Remove(int nodeId);
		bool 	Update(int nodeId, const Box3& aabb, const Vector3& displacement);

		bool	RayCast(const Ray3& Ray, const RayCastOption* Option, RayCastResult* Result) const;
		bool	RayCast(const Ray3& Ray, const RayCastOption* Option, RayCastResult* Result, DynamicRayCastCallback Callback, void* Context) const;
		bool	Intersect(const Geometry* geometry, const IntersectOption* Option, IntersectResult* Result) const;
		bool	Intersect(const Geometry* geometry, const IntersectOption* Option, IntersectResult* Result, DynamicIntersectCallback Callback, void* Context) const;
        bool	Sweep(const Geometry* geometry, const Vector3& Direction, const SweepOption* Option, SweepResult* Result) const;
		bool	Sweep(const Geometry* geometry, const Vector3& Direction, const SweepOption* Option, SweepResult* Result, DynamicSweepCallback Callback, void* Context) const;
		bool	Query(const Box3& aabb, std::vector<void*>* Result) const;
		void	CollectAABBs(std::vector<Box3>* aabbs) const;

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
