
#include "TriangleMesh.h"
#include "GeometryObject.h"
#include "../CollisionPrimitive/Triangle3d.h"

#include <algorithm>
#include <array>
#include <vector>

struct RTreeNodeNQ
{
	Box3d	bounds;
	int		childPageFirstNodeIndex; // relative to the beginning of all build tree nodes array
	int		leafCount; // -1 for empty nodes, 0 for non-terminal nodes, number of enclosed tris if non-zero (LeafTriangles), also means a terminal node

	struct U {}; // selector struct for uninitialized constructor
	RTreeNodeNQ(U) {} // uninitialized constructor
	RTreeNodeNQ() : bounds(Box3d::Empty()), childPageFirstNodeIndex(-1), leafCount(0) {}
};

static const unsigned int NTRADEOFF = 15;
//	%  -24 -23 -17 -15 -10  -8  -5  -3   0  +3  +3  +5  +7   +8    +9  - % raycast MeshSurface*Random benchmark perf
//  K  717 734 752 777 793 811 824 866 903 939 971 1030 1087 1139 1266 - testzone size in K
//  #    0   1   2   3   4   5   6   7   8   9  10  11  12   13     14 - preset number
static const unsigned int stopAtTrisPerPage[NTRADEOFF] = { 64, 60, 56, 48, 46, 44, 40, 36, 32, 28, 24, 20, 16,  12,    12 };
static const unsigned int stopAtTrisPerLeaf[NTRADEOFF] = { 16, 14, 12, 10,  9,  8,  8,  6,  5,  5,  5,  4,  4,   4,     2 }; // capped at 2 anyway

// auxiliary class for SAH build (SAH = surface area heuristic)
struct Interval
{
	unsigned int start, count;
	Interval(unsigned int s, unsigned int c) : start(s), count(c) {}
};

struct SortBoundsPredicate
{
	unsigned int coordIndex;
	const Box3d* allBounds;
	SortBoundsPredicate(unsigned int coordIndex_, const Box3d* allBounds_) : coordIndex(coordIndex_), allBounds(allBounds_)
	{}

	bool operator()(const unsigned int& idx1, const unsigned int& idx2) const
	{
		// using the bounds center for comparison
		float center1 = allBounds[idx1].Min[coordIndex] + allBounds[idx1].Max[coordIndex];
		float center2 = allBounds[idx2].Min[coordIndex] + allBounds[idx2].Max[coordIndex];
		return (center1 < center2);
	}
};

struct RTreeRemap
{
	unsigned int mNbTris;
	RTreeRemap(unsigned int numTris) : mNbTris(numTris)
	{
	}

	virtual void remap(unsigned int* val, unsigned int start, unsigned int leafCount)
	{
		assert(leafCount > 0);
		assert(leafCount <= 16); // sanity check
		assert(start < mNbTris);
		assert(start + leafCount <= mNbTris);
		assert(val);
		LeafTriangles lt;
		// here we remap from ordered leaf index in the rtree to index in post-remap in triangles
		// this post-remap will happen later
		lt.SetData(leafCount, start);
		*val = lt.Data;
	}
};

static float SAH(const Vector3d& v)
{
	return v.x*v.y + v.y*v.z + v.x*v.z;
}

struct SubSortSAH
{
	unsigned int* permuteStart, * tempPermute;
	const Box3d* allBounds;
	float* metricL;
	float* metricR;
	const unsigned int* xOrder, * yOrder, * zOrder;
	const unsigned int* xRanks, * yRanks, * zRanks;
	unsigned int* tempRanks;
	unsigned int nbTotalBounds;
	unsigned int iTradeOff;

	// precompute various values used during sort
	SubSortSAH(
		unsigned int* permute, const Box3d* allBounds_, unsigned int numBounds,
		const unsigned int* xOrder_, const unsigned int* yOrder_, const unsigned int* zOrder_,
		const unsigned int* xRanks_, const unsigned int* yRanks_, const unsigned int* zRanks_, float sizePerfTradeOff01)
		: permuteStart(permute), allBounds(allBounds_),
		xOrder(xOrder_), yOrder(yOrder_), zOrder(zOrder_),
		xRanks(xRanks_), yRanks(yRanks_), zRanks(zRanks_), nbTotalBounds(numBounds)
	{
		metricL = new float[numBounds];
		metricR = new float[numBounds];
		tempPermute = new unsigned int[numBounds * 2 + 1];
		tempRanks = new unsigned int[numBounds];
		iTradeOff = std::min(unsigned int(std::min<float>(0.0f, sizePerfTradeOff01) * NTRADEOFF), NTRADEOFF - 1);
	}

	~SubSortSAH() // release temporarily used memory
	{
		delete []metricL;
		delete []metricR;
		delete []tempPermute;
		delete []tempRanks;
	}

	////////////////////////////////////////////////////////////////////
	// returns split position for second array start relative to permute ptr
	unsigned int split(unsigned int* permute, unsigned int clusterSize)
	{
		if (clusterSize <= 1)
			return 0;
		if (clusterSize == 2)
			return 1;

		int minCount = clusterSize >= 4 ? 2 : 1;
		int splitStartL = minCount; // range=[startL->endL)
		int splitEndL = int(clusterSize - minCount);
		int splitStartR = int(clusterSize - splitStartL); // range=(endR<-startR], startR > endR
		int splitEndR = int(clusterSize - splitEndL);
		assert(splitEndL - splitStartL == splitStartR - splitEndR);
		assert(splitStartL <= splitEndL);
		assert(splitStartR >= splitEndR);
		assert(splitEndR >= 1);
		assert(splitEndL < int(clusterSize));

		// pick the best axis with some splitting metric
		// axis index is X=0, Y=1, Z=2
		float minMetric[3];
		unsigned int minMetricSplit[3];
		const unsigned int* ranks3[3] = { xRanks, yRanks, zRanks };
		const unsigned int* orders3[3] = { xOrder, yOrder, zOrder };
		for (unsigned int coordIndex = 0; coordIndex <= 2; coordIndex++)
		{
			SortBoundsPredicate sortPredicateLR(coordIndex, allBounds);

			const unsigned int* rank = ranks3[coordIndex];
			const unsigned int* order = orders3[coordIndex];

			// build ranks in tempPermute
			if (clusterSize == nbTotalBounds) // AP: about 4% perf gain from this optimization
			{
				// if this is a full cluster sort, we already have it done
				for (unsigned int i = 0; i < clusterSize; i++)
					tempPermute[i] = order[i];
			}
			else
			{
				// sort the tempRanks
				for (unsigned int i = 0; i < clusterSize; i++)
					tempRanks[i] = rank[permute[i]];
				std::sort(tempRanks, tempRanks + clusterSize);
				for (unsigned int i = 0; i < clusterSize; i++) // convert back from ranks to indices
					tempPermute[i] = order[tempRanks[i]];
			}

			// we consider overlapping intervals for minimum sum of metrics
			// left interval is from splitStartL up to splitEndL
			// right interval is from splitStartR down to splitEndR


			// first compute the array metricL
			Vector3d boundsLmn = allBounds[tempPermute[0]].Min; // init with 0th bound
			Vector3d boundsLmx = allBounds[tempPermute[0]].Max; // init with 0th bound
			int ii;
			for (ii = 1; ii < splitStartL; ii++) // sweep right to include all bounds up to splitStartL-1
			{
				boundsLmn = boundsLmn.Min(allBounds[tempPermute[ii]].Min);
				boundsLmx = boundsLmx.Max(allBounds[tempPermute[ii]].Max);
			}

			unsigned int countL0 = 0;
			for (ii = splitStartL; ii <= splitEndL; ii++) // compute metric for inclusive bounds from splitStartL to splitEndL
			{
				boundsLmn = boundsLmn.Min(allBounds[tempPermute[ii]].Min);
				boundsLmx = boundsLmx.Max(allBounds[tempPermute[ii]].Max);
				metricL[countL0++] = SAH(boundsLmx - boundsLmn);
			}
			// now we have metricL

			// now compute the array metricR
			Vector3d boundsRmn = allBounds[tempPermute[clusterSize - 1]].Min; // init with last bound
			Vector3d boundsRmx = allBounds[tempPermute[clusterSize - 1]].Max; // init with last bound
			for (ii = int(clusterSize - 2); ii > splitStartR; ii--) // include bounds to the left of splitEndR down to splitStartR
			{
				boundsRmn = boundsRmn.Min(allBounds[tempPermute[ii]].Min);
				boundsRmx = boundsRmx.Max(allBounds[tempPermute[ii]].Max);
			}

			unsigned int countR0 = 0;
			for (ii = splitStartR; ii >= splitEndR; ii--) // continue sweeping left, including bounds and recomputing the metric
			{
				boundsRmn = boundsRmn.Min(allBounds[tempPermute[ii]].Min);
				boundsRmx = boundsRmx.Max(allBounds[tempPermute[ii]].Max);
				metricR[countR0++] = SAH(boundsRmx - boundsRmn);
			}

			assert((countL0 == countR0) && (countL0 == unsigned int(splitEndL - splitStartL + 1)));

			// now iterate over splitRange and compute the minimum sum of SAHLeft*countLeft + SAHRight*countRight
			unsigned int minMetricSplitPosition = 0;
			float minMetricLocal = FLT_MAX;
			const int hsI32 = int(clusterSize / 2);
			const int splitRange = (splitEndL - splitStartL + 1);
			for (ii = 0; ii < splitRange; ii++)
			{
				float countL = float(ii + minCount); // need to add minCount since ii iterates over splitRange
				float countR = float(splitRange - ii - 1 + minCount);
				assert(unsigned int(countL + countR) == clusterSize);

				const float metric = (countL * metricL[ii] + countR * metricR[splitRange - ii - 1]);
				const unsigned int splitPos = unsigned int(ii + splitStartL);
				if (metric < minMetricLocal ||
					(metric <= minMetricLocal && // same metric but more even split
						abs(int(splitPos) - hsI32) < abs(int(minMetricSplitPosition) - hsI32)))
				{
					minMetricLocal = metric;
					minMetricSplitPosition = splitPos;
				}
			}

			minMetric[coordIndex] = minMetricLocal;
			minMetricSplit[coordIndex] = minMetricSplitPosition;

			// sum of axis lengths for both left and right AABBs
		}

		unsigned int winIndex = 2;
		if (minMetric[0] <= minMetric[1] && minMetric[0] <= minMetric[2])
			winIndex = 0;
		else if (minMetric[1] <= minMetric[2])
			winIndex = 1;

		const unsigned int* rank = ranks3[winIndex];
		const unsigned int* order = orders3[winIndex];
		if (clusterSize == nbTotalBounds) // AP: about 4% gain from this special case optimization
		{
			// if this is a full cluster sort, we already have it done
			for (unsigned int i = 0; i < clusterSize; i++)
				permute[i] = order[i];
		}
		else
		{
			// sort the tempRanks
			for (unsigned int i = 0; i < clusterSize; i++)
				tempRanks[i] = rank[permute[i]];
			std::sort(tempRanks, tempRanks + clusterSize);
			for (unsigned int i = 0; i < clusterSize; i++)
				permute[i] = order[tempRanks[i]];
		}

		unsigned int splitPoint = minMetricSplit[winIndex];
		if (clusterSize == 3 && splitPoint == 0)
			splitPoint = 1; // special case due to rounding
		return splitPoint;
	}

	// compute surface area for a given split
	float computeSA(const unsigned int* permute, const Interval& split) // both permute and i are relative
	{
		assert(split.count >= 1);
		Vector3d bmn = allBounds[permute[split.start]].Min;
		Vector3d bmx = allBounds[permute[split.start]].Max;
		for (unsigned int i = 1; i < split.count; i++)
		{
			const Box3d& b1 = allBounds[permute[split.start + i]];
			bmn = bmn.Min(b1.Min);
			bmx = bmx.Max(b1.Max);
		}

		return SAH(bmx - bmn);
	}

	////////////////////////////////////////////////////////////////////
	// main SAH sort routine
	void sort4(unsigned int* permute, unsigned int clusterSize,
		std::vector<RTreeNodeNQ>& resultTree, unsigned int& maxLevels, unsigned int level = 0, RTreeNodeNQ* parentNode = NULL)
	{
		if (level == 0)
			maxLevels = 1;
		else
			maxLevels = std::max(maxLevels, level + 1);

		unsigned int splitPos[RTREE_N];
		for (unsigned int j = 0; j < RTREE_N; j++)
			splitPos[j] = j + 1;

		if (clusterSize >= RTREE_N)
		{
			// split into RTREE_N number of regions via RTREE_N-1 subsequent splits
			// each split is represented as a current interval
			// we iterate over currently active intervals and compute it's surface area
			// then we split the interval with maximum surface area
			// AP scaffold: possible optimization - seems like computeSA can be cached for unchanged intervals
			std::vector<Interval> splits;
			splits.push_back(Interval(0, clusterSize));
			for (unsigned int iSplit = 0; iSplit < RTREE_N - 1; iSplit++)
			{
				float maxSAH = -FLT_MAX;
				unsigned int maxSplit = 0xFFFFffff;
				for (unsigned int i = 0; i < splits.size(); i++)
				{
					if (splits[i].count == 1)
						continue;
					float SAH = computeSA(permute, splits[i]) * splits[i].count;
					if (SAH > maxSAH)
					{
						maxSAH = SAH;
						maxSplit = i;
					}
				}
				assert(maxSplit != 0xFFFFffff);

				// maxSplit is now the index of the interval in splits array with maximum surface area
				// we now split it into 2 using the split() function
				Interval old = splits[maxSplit];
				assert(old.count > 1);
				unsigned int splitLocal = split(permute + old.start, old.count); // relative split pos

				assert(splitLocal >= 1);
				assert(old.count - splitLocal >= 1);
				splits.push_back(Interval(old.start, splitLocal));
				splits.push_back(Interval(old.start + splitLocal, old.count - splitLocal));
				splits[maxSplit] =  splits.back();
				splits.pop_back();
				splitPos[iSplit] = old.start + splitLocal;
			}

			// verification code, make sure split counts add up to clusterSize
			assert(splits.size() == RTREE_N);
			unsigned int sum = 0;
			for (unsigned int j = 0; j < RTREE_N; j++)
				sum += splits[j].count;
			assert(sum == clusterSize);
		}
		else // clusterSize < RTREE_N
		{
			// make it so splitCounts based on splitPos add up correctly for small cluster sizes
			for (unsigned int i = clusterSize; i < RTREE_N - 1; i++)
				splitPos[i] = clusterSize;
		}

		// sort splitPos index array using quicksort (just a few values)
		std::sort(splitPos, splitPos + RTREE_N - 1);
		splitPos[RTREE_N - 1] = clusterSize; // splitCount[n] is computed as splitPos[n+1]-splitPos[n], so we need to add this last value

		// now compute splitStarts and splitCounts from splitPos[] array. Also perform a bunch of correctness verification
		unsigned int splitStarts[RTREE_N];
		unsigned int splitCounts[RTREE_N];
		splitStarts[0] = 0;
		splitCounts[0] = splitPos[0];
		unsigned int sumCounts = splitCounts[0];
		for (unsigned int j = 1; j < RTREE_N; j++)
		{
			splitStarts[j] = splitPos[j - 1];
			assert(splitStarts[j - 1] <= splitStarts[j]);
			splitCounts[j] = splitPos[j] - splitPos[j - 1];
			assert(splitCounts[j] > 0 || clusterSize < RTREE_N);
			sumCounts += splitCounts[j];
			assert(splitStarts[j - 1] + splitCounts[j - 1] <= splitStarts[j]);
		}
		assert(sumCounts == clusterSize);
		assert(splitStarts[RTREE_N - 1] + splitCounts[RTREE_N - 1] <= clusterSize);

		// mark this cluster as terminal based on clusterSize <= stopAtTrisPerPage parameter for current iTradeOff user specified preset
		bool terminalClusterByTotalCount = (clusterSize <= stopAtTrisPerPage[iTradeOff]);
		// iterate over splitCounts for the current cluster, if any of counts exceed 16 (which is the maximum supported by LeafTriangles
		// we cannot mark this cluster as terminal (has to be split more)
		for (unsigned int s = 0; s < RTREE_N; s++)
			if (splitCounts[s] > 16) // LeafTriangles doesn't support > 16 tris
				terminalClusterByTotalCount = false;

		// iterate over all the splits
		for (unsigned int s = 0; s < RTREE_N; s++)
		{
			RTreeNodeNQ rtn;
			unsigned int splitCount = splitCounts[s];
			if (splitCount > 0) // splits shouldn't be empty generally
			{
				// sweep left to right and compute min and max SAH for each individual bound in current split
				Box3d b = allBounds[permute[splitStarts[s]]];
				float sahMin = SAH(b.GetExtent());
				float sahMax = sahMin;
				// AP scaffold - looks like this could be optimized (we are recomputing bounds top down)
				for (unsigned int i = 1; i < splitCount; i++)
				{
					unsigned int localIndex = i + splitStarts[s];
					const Box3d& b1 = allBounds[permute[localIndex]];
					float sah1 = SAH(b1.GetExtent());
					sahMin = std::min(sahMin, sah1);
					sahMax = std::max(sahMax, sah1);
					b.Grow(b1);
				}

				rtn.bounds.Min = b.Min;
				rtn.bounds.Max = b.Max;

				// if bounds differ widely (according to some heuristic preset), we continue splitting
				// this is important for a mixed cluster with large and small triangles
				bool okSAH = (sahMax / sahMin < 40.0f);
				if (!okSAH)
					terminalClusterByTotalCount = false; // force splitting this cluster

				bool stopSplitting = // compute the final splitting criterion
					splitCount <= 2 || (okSAH && splitCount <= 3) // stop splitting at 2 nodes or if SAH ratio is OK and splitCount <= 3
					|| terminalClusterByTotalCount || splitCount <= stopAtTrisPerLeaf[iTradeOff];
				if (stopSplitting)
				{
					// this is a terminal page then, mark as such
					// first node index is relative to the top level input array beginning
					rtn.childPageFirstNodeIndex = int(splitStarts[s] + (permute - permuteStart));
					rtn.leafCount = int(splitCount);
					assert(splitCount <= 16); // LeafTriangles doesn't support more
				}
				else
				{
					// this is not a terminal page, we will recompute this later, after we recurse on subpages (label ZZZ)
					rtn.childPageFirstNodeIndex = -1;
					rtn.leafCount = 0;
				}
			}
			else // splitCount == 0 at this point, this is an empty paddding node (with current presets it's very rare)
			{
				assert(splitCount == 0);
				rtn.bounds.SetEmpty();
				rtn.childPageFirstNodeIndex = -1;
				rtn.leafCount = -1;
			}
			resultTree.push_back(rtn); // push the new node into the resultTree array
		}

		if (terminalClusterByTotalCount) // abort recursion if terminal cluster
			return;

		// recurse on subpages
		unsigned int parentIndex = (unsigned int)resultTree.size() - RTREE_N; // save the parentIndex as specified (array can be resized during recursion)
		for (unsigned int s = 0; s < RTREE_N; s++)
		{
			RTreeNodeNQ* sParent = &resultTree[parentIndex + s]; // array can be resized and relocated during recursion
			if (sParent->leafCount == 0) // only split pages that were marked as non-terminal during splitting (see "label ZZZ" above)
			{
				// all child nodes will be pushed inside of this recursive call,
				// so we set the child pointer for parent node to resultTree.size()
				sParent->childPageFirstNodeIndex = int(resultTree.size());
				sort4(permute + splitStarts[s], splitCounts[s], resultTree, maxLevels, level + 1, sParent);
			}
		}
	}
};


static void buildFromBounds(RTree& result, void*& Memory, const Box3d* allBounds, unsigned int numBounds, std::vector<unsigned int>& permute, const Box3d &treeBounds, RTreeRemap *rc)
{
	float sizePerfTradeOff01 = 1.0f;		// 0 - 1
	int hint = 0;

	// start off with an identity permutation
	permute.resize(0);
	permute.reserve(numBounds + 1);
	for (unsigned int j = 0; j < numBounds; j++)
		permute.push_back(j);
	const unsigned int sentinel = 0xABCDEF01;
	permute.push_back(sentinel);

	// load sorted nodes into an RTreeNodeNQ tree representation
	// build the tree structure from sorted nodes
	const unsigned int pageSize = RTREE_N;
	std::vector<RTreeNodeNQ> resultTree;
	resultTree.reserve(numBounds * 2);

	unsigned int maxLevels = 0;

	if (hint == 0) // use high quality SAH build, eSIM_PERFORMANCE
	{
		std::vector<unsigned int> xRanks(numBounds), yRanks(numBounds), zRanks(numBounds), xOrder(numBounds), yOrder(numBounds), zOrder(numBounds);
		memcpy(&xOrder[0], &permute[0], sizeof(xOrder[0]) * numBounds);
		memcpy(&yOrder[0], &permute[0], sizeof(yOrder[0]) * numBounds);
		memcpy(&zOrder[0], &permute[0], sizeof(zOrder[0]) * numBounds);
		// sort by shuffling the permutation, precompute sorted ranks for x,y,z-orders
		std::sort(xOrder.begin(), xOrder.end(), SortBoundsPredicate(0, allBounds));
		for (unsigned int i = 0; i < numBounds; i++) xRanks[xOrder[i]] = i;
		std::sort(yOrder.begin(), yOrder.end(), SortBoundsPredicate(1, allBounds));
		for (unsigned int i = 0; i < numBounds; i++) yRanks[yOrder[i]] = i;
		std::sort(zOrder.begin(), zOrder.end(), SortBoundsPredicate(2, allBounds));
		for (unsigned int i = 0; i < numBounds; i++)
			zRanks[zOrder[i]] = i;

		SubSortSAH ss(&permute[0], allBounds, numBounds, &xOrder[0], &yOrder[0], &zOrder[0], &xRanks[0], &yRanks[0], &zRanks[0], sizePerfTradeOff01);
		ss.sort4(&permute[0], numBounds, resultTree, maxLevels);
	}
	else
	{ // use fast cooking path
		// assert(hint == PxMeshCookingHint::eCOOKING_PERFORMANCE);
		// SubSortQuick ss(permute.begin(), allBounds, numBounds, sizePerfTradeOff01);
		// Box3d discard((Box3d::U()));
		// ss.sort4(permute.begin(), permute.size() - 1, resultTree, maxLevels, discard); // AP scaffold: need to implement build speed/runtime perf slider
	}

	assert(permute[numBounds] == sentinel); // verify we didn't write past the array
	permute.pop_back(); // discard the sentinel value

	assert(RTREE_N * sizeof(RTreeNodeQ) == sizeof(RTreePage)); // needed for nodePtrMultiplier computation to be correct
	const int nodePtrMultiplier = sizeof(RTreeNodeQ); // convert offset as count in qnodes to page ptr

	// Quantize the tree. AP scaffold - might be possible to merge this phase with the page pass below this loop
	std::vector<RTreeNodeQ> qtreeNodes;
	unsigned int firstEmptyIndex = unsigned int(-1);
	unsigned int resultCount = (unsigned int)resultTree.size();
	qtreeNodes.reserve(resultCount);

	for (unsigned int i = 0; i < resultCount; i++) // AP scaffold - eliminate this pass
	{
		RTreeNodeNQ& u = resultTree[i];
		RTreeNodeQ q;
		q.setLeaf(u.leafCount > 0); // set the leaf flag
		if (u.childPageFirstNodeIndex == -1) // empty node?
		{
			if (firstEmptyIndex == unsigned int(-1))
				firstEmptyIndex = (unsigned int)qtreeNodes.size();
			q.minx = q.miny = q.minz = FLT_MAX; // AP scaffold improvement - use empty 1e30 bounds instead and reference a valid leaf
			q.maxx = q.maxy = q.maxz = -FLT_MAX; // that will allow to remove the empty node test from the runtime

			q.ptr = firstEmptyIndex * nodePtrMultiplier;
			assert((q.ptr & 1) == 0);
			q.setLeaf(true); // label empty node as leaf node
		}
		else
		{
			// non-leaf node
			q.minx = u.bounds.Min.x;
			q.miny = u.bounds.Min.y;
			q.minz = u.bounds.Min.z;
			q.maxx = u.bounds.Max.x;
			q.maxy = u.bounds.Max.y;
			q.maxz = u.bounds.Max.z;
			if (u.leafCount > 0)
			{
				q.ptr = unsigned int(u.childPageFirstNodeIndex);
				rc->remap(&q.ptr, q.ptr, unsigned int(u.leafCount));
				assert(q.isLeaf()); // remap is expected to set the isLeaf bit
			}
			else
			{
				// verify that all children bounds are included in the parent bounds
				for (unsigned int s = 0; s < RTREE_N; s++)
				{
					const RTreeNodeNQ& child = resultTree[u.childPageFirstNodeIndex + s];
					assert(child.leafCount == -1 || child.bounds.IsInside(u.bounds));
				}

				q.ptr = unsigned int(u.childPageFirstNodeIndex * nodePtrMultiplier);
				assert(q.ptr % RTREE_N == 0);
				q.setLeaf(false);
			}
		}
		qtreeNodes.push_back(q);
	}

	// build the final rtree image
	result.mInvDiagonal = Vector4d(1.0f);
	assert(qtreeNodes.size() % RTREE_N == 0);
	result.mTotalNodes = (unsigned int)qtreeNodes.size();
	result.mTotalPages = result.mTotalNodes / pageSize;
	Memory = new unsigned char[sizeof(RTreePage) * result.mTotalPages + 127];		// TODO
	result.mPages = (RTreePage*)AlignMemory(Memory, 128);
	result.mBoundsMin = Vector4d(treeBounds.Min, 0.0f);
	result.mBoundsMax = Vector4d(treeBounds.Max, 0.0f);
	result.mDiagonalScaler = (result.mBoundsMax - result.mBoundsMin) / 65535.0f;
	result.mPageSize = pageSize;
	result.mNumLevels = maxLevels;
	assert(result.mTotalNodes % pageSize == 0);
	result.mNumRootPages = 1;

	for (unsigned int j = 0; j < result.mTotalPages; j++)
	{
		RTreePage& page = result.mPages[j];
		for (unsigned int k = 0; k < RTREE_N; k++)
		{
			const RTreeNodeQ& n = qtreeNodes[j * RTREE_N + k];
			page.maxx[k] = n.maxx;
			page.maxy[k] = n.maxy;
			page.maxz[k] = n.maxz;
			page.minx[k] = n.minx;
			page.miny[k] = n.miny;
			page.minz[k] = n.minz;
			page.ptrs[k] = n.ptr;
		}
	}

#if _DEBUG
	result.validate(); // make sure the child bounds are included in the parent and other validation
#endif
}


RTree* TriangleMesh::CreateEmptyRTree()
{
	if (m_Tree == nullptr)
	{
		m_Tree = new RTree;
	}
	else
	{
		m_Tree->release();
	}
	return m_Tree;
}

void* TriangleMesh::AllocMemory(int Size, int Width)
{
	if (m_Memory != nullptr)
	{
		delete[]m_Memory;
	}
	m_Memory = new char[Size + Width - 1];
	return AlignMemory(m_Memory, Width);
}

void TriangleMesh::BuildRTree()
{
	if (NumTriangles == 0)
	{
		return;
	}

	std::vector<Box3d> allBounds;
	allBounds.reserve(NumTriangles);

	Box3d treeBounds = Box3d::Empty();

	for (unsigned int i = 0; i < NumTriangles; ++i)
	{
		unsigned int i0 = Indices[i*3];
		unsigned int i1 = Indices[i*3 + 1];
		unsigned int i2 = Indices[i*3 + 2];

		allBounds.push_back(Box3d(Verties[i0], Verties[i1], Verties[i2]));
		treeBounds.Grow(allBounds.back());
	}

	CreateEmptyRTree();

	std::vector<unsigned int> permute;
	RTreeRemap rc(NumTriangles);
	buildFromBounds(*m_Tree, m_Memory, &allBounds[0], NumTriangles, permute, treeBounds, &rc);
}

template<bool tRayTest>
class RTreeCallbackRaycast : public RTree::CallbackRaycast
{
public:
	const void* mTris;
	const Vector3d* mVerts;
	const Vector3d* mInflate;
	// const SimpleRayTriOverlap rayCollider;
	float maxT;
	RayCastResult closestHit; // recorded closest hit over the whole traversal (only for callback mode eCLOSEST)
	Vector3d cv0, cv1, cv2;	// PT: make sure these aren't last in the class, to safely V4Load them
	unsigned int cis[3];
	bool hadClosestHit;
	const bool closestMode;
	const bool IndicesBit16;
	Vector3d inflateV, rayOriginV, rayDirV;

	RTreeCallbackRaycast(const void* tris, const Vector3d* verts,
		const Vector3d& origin, const Vector3d& dir, float maxT_, bool bit16)
		:
		mTris(tris), mVerts(verts),
		maxT(maxT_), closestMode(true), IndicesBit16(bit16)
	{
		assert(closestHit.hitTime == FLT_MAX);
		hadClosestHit = false;
		rayOriginV = origin;
		rayDirV = dir;
	}

	void getVertIndices(unsigned int triIndex, unsigned int& i0, unsigned int& i1, unsigned int& i2)
	{
		if (IndicesBit16)
		{
			const unsigned short* p = (unsigned short*)mTris + triIndex * 3;
			i0 = p[0]; i1 = p[1]; i2 = p[2];
		}
		else
		{
			const unsigned int* p = (unsigned int*)mTris + triIndex * 3;
			i0 = p[0]; i1 = p[1]; i2 = p[2];
		}
	}

	// result buffer should have room for at least RTREE_N items
	// should return true to continue traversal. If false is returned, traversal is aborted
	// newMaxT serves as both input and output, as input it's the maxT so far
	// set it to a new value (which should be smaller) and it will become the new far clip t
	virtual bool processResults(unsigned int NumTouched, unsigned int* Touched, float& newMaxT) override
	{
		assert(NumTouched > 0);
		// Loop through touched leaves
		RayCastResult tempHit;
		for (unsigned int leaf = 0; leaf < NumTouched; leaf++)
		{
			// Each leaf box has a set of triangles
			LeafTriangles currentLeaf;
			currentLeaf.Data = Touched[leaf];
			unsigned int nbLeafTris = currentLeaf.GetNbTriangles();
			unsigned int baseLeafTriIndex = currentLeaf.GetTriangleIndex();

			for (unsigned int i = 0; i < nbLeafTris; i++)
			{
				unsigned int i0, i1, i2;
				const unsigned int triangleIndex = baseLeafTriIndex + i;
				getVertIndices(triangleIndex, i0, i1, i2);

				const Vector3d& v0 = mVerts[i0], & v1 = mVerts[i1], & v2 = mVerts[i2];
				const unsigned int vinds[3] = { i0, i1, i2 };

				if (tRayTest)
				{
					bool intersect = Triangle3d::RayIntersectTriangle(rayOriginV, rayDirV, v0, v1, v2, &tempHit.hitTime);
					intersect = intersect && tempHit.hitTime <= maxT;
					if (!intersect)
						continue;
				}
				// TODO
				triangleIndex;
				// tempHit.faceIndex = triangleIndex;
				// tempHit.flags = PxHitFlag::ePOSITION;

				// Intersection point is valid if dist < segment's length
				// We know dist>0 so we can use integers
				if (closestMode)
				{
					if (tempHit.hitTime < closestHit.hitTime)
					{
						closestHit = tempHit;
						newMaxT = std::min(tempHit.hitTime, newMaxT);
						cv0 = v0;
						cv1 = v1;
						cv2 = v2;
						cis[0] = vinds[0]; cis[1] = vinds[1]; cis[2] = vinds[2];
						hadClosestHit = true;
					}
				}
				else
				{
					/*
					float shrunkMaxT = newMaxT;
					PxAgain again = outerCallback.processHit(tempHit, v0, v1, v2, shrunkMaxT, vinds);
					if (!again)
						return false;
					if (shrunkMaxT < newMaxT)
					{
						newMaxT = shrunkMaxT;
						maxT = shrunkMaxT;
					}
					*/
				}

				// if (outerCallback.inAnyMode()) // early out if in ANY mode
				//	return false;
			}

		} // for(unsigned int leaf = 0; leaf<NumTouched; leaf++)

		return true;
	}
};

bool TriangleMesh::IntersectRay(const Vector3d& Origin, const Vector3d& Dir, float* t) const
{
	if (m_Tree == nullptr)
	{
		return false;
	}
	RTreeCallbackRaycast<true> cb(GetIndexBuffer(), &Verties[0], Origin, Dir, FLT_MAX, Is16bitIndices());
	m_Tree->traverseRay(Origin, Dir, &cb);
	if (cb.hadClosestHit)
	{
		*t = cb.closestHit.hitTime;
		return true;
	}
	return false;
}

Matrix3d TriangleMesh::GetInertiaTensor(float Mass) const
{
	assert(false);
	return Matrix3d::Zero();
}

Vector3d TriangleMesh::GetSupport(const Vector3d& dir) const
{
	assert(false);
	return Vector3d::Zero();
}
