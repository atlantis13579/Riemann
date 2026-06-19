#pragma once

#include <vector>

#include "../CollisionPrimitive/Ray3.h"
#include "AABBTreeOffline.h"

namespace Riemann
{
	struct RayCastOption;
	struct RayCastResult;
	struct IntersectOption;
	struct IntersectResult;
	struct SweepOption;
	struct SweepResult;
	struct CacheFriendlyAABBTree;
	class Geometry;

	struct TreeStatistics
	{
		int MaxStack;
		int MaxDepth;
		int MaxGeometriesAtLeaf;
		int NumNodes;
		int NumLeafs;
	};

	class AABBTree
	{
	public:
		AABBTree();
		~AABBTree();

	public:
		void	Release();

		void	StaticBuild(AABBTreeBuildData& params);
		void	SetDirty(bool dirty = true);
		bool	IsDirty() const;
		void	Statistic(TreeStatistics& stat);

		bool	RayCast(const Ray3& Ray, Geometry** ObjectCollection, const RayCastOption* Option, RayCastResult* Result) const;
		bool	Intersect(const Geometry* intersect_geometry, Geometry** ObjectCollection, const IntersectOption* Option, IntersectResult* Result) const;
		bool	Sweep(const Geometry* sweep_geometry, Geometry** ObjectCollection, const Vector3& Direction, const SweepOption* Option, SweepResult* Result) const;
		void	CollectAABBs(std::vector<Box3>* aabbs) const;

		int		IntersectPoint(const Vector3& Point) const;
		bool	RayCastBoundingBox(const Ray3& ray, const RayCastOption& Option, RayCastResult* Result) const;

	private:
		void	InitAABBTreeBuild(AABBTreeBuildData& params);

	private:
		int* m_GeometryIndicesBase;
		int	m_NumGeometries;
		bool m_Dirty;

		CacheFriendlyAABBTree* m_AABBTreeInference;
	};
}
