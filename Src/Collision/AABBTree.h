#pragma once

#include "../CollisionPrimitive/Ray3d.h"
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
	class GeometryBase;

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
		void	Statistic(TreeStatistics& stat);

		bool	RayCast(const Ray3d& ray, GeometryBase** ObjectCollection, const RayCastOption* Option, RayCastResult* Result) const;
		bool	Intersect(const GeometryBase* geometry, GeometryBase** ObjectCollection, const IntersectOption* Option, IntersectResult* Result) const;
		bool	Sweep(const GeometryBase* geometry, GeometryBase** ObjectCollection, const Vector3& Direction, const SweepOption* Option, SweepResult* Result) const;

		int		IntersectPoint(const Vector3& Point) const;
		bool	RayCastBoundingBox(const Ray3d& ray, const RayCastOption& Option, RayCastResult* Result) const;

	private:
		void	InitAABBTreeBuild(AABBTreeBuildData& params);

	private:
		int* m_GeometryIndicesBase;
		int							m_NumGeometries;

		CacheFriendlyAABBTree* m_AABBTreeInference;
	};
}