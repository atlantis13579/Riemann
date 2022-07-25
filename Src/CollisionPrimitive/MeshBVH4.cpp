
#include "MeshBVH4.h"
#include "Mesh.h"

#define INFLATION_EPSILON 5e-4f

void MeshBVH4::ValidateRecursive(void* p, uint32_t Depth, const Box3d& parentBounds, BVHNodeBatch* batch)
{
	static uint32_t validateCounter = 0; // this is to suppress a warning that recursive call has no side effects
	validateCounter++;
	
	uint32_t pageNodeCount = batch->GetNodeCount();
	for (uint32_t j = 0; j < pageNodeCount; j++)
	{
		BVHNode n = batch->GetNode(j);
		if (batch->IsEmpty(j))
			continue;
		Box3d* bounds = (Box3d*)&n.minx;
		assert(bounds->IsInside(parentBounds));
		if (!n.IsLeaf())
		{
			assert((n.Data & 1) == 0);
			BVHNodeBatch* child = reinterpret_cast<BVHNodeBatch*>(size_t(BatchPtr) + n.Data);
			ValidateRecursive(p, Depth + 1, *bounds, child);
		}
		
		if (n.IsLeaf())
		{
			LeafNode currentLeaf(n.Data);
			uint32_t nbLeafTris = currentLeaf.GetNumTriangles();
			uint32_t baseLeafTriIndex = currentLeaf.GetTriangleIndex();
			for (uint32_t i = 0; i < nbLeafTris; i++)
			{
				const uint32_t triangleIndex = baseLeafTriIndex + i;
				Mesh* pm = (Mesh*)p;
				for (int i = 0; i < 3; ++i)
				{
					const Vector3 &vv = pm->GetVertex(triangleIndex, i);
					assert(bounds->IsInside(vv));
				}
				continue;
			}
		}
	}

	Box3d recomputedBounds = batch->ComputeBounds();
	assert((recomputedBounds.Min.x - parentBounds.Min.x) <= INFLATION_EPSILON);
	assert((recomputedBounds.Min.y - parentBounds.Min.y) <= INFLATION_EPSILON);
	assert((recomputedBounds.Min.z - parentBounds.Min.z) <= INFLATION_EPSILON);
	assert((recomputedBounds.Max.x - parentBounds.Max.x) <= INFLATION_EPSILON);
	assert((recomputedBounds.Max.y - parentBounds.Max.y) <= INFLATION_EPSILON);
	assert((recomputedBounds.Max.z - parentBounds.Max.z) <= INFLATION_EPSILON);
}

void		MeshBVH4::Validate(void *p)
{
	for (uint32_t j = 0; j < NumRoots; j++)
	{
		Box3d BV = BatchPtr[j].ComputeBounds();
		ValidateRecursive(p, 0, BV, BatchPtr + j);
	}
}


struct TreeNode
{
	Box3d	bounds;
	int		NextNodeIndex;
	int		LeafCount;
	TreeNode() : bounds(Box3d::Empty()), NextNodeIndex(-1), LeafCount(0) {}
};

static const uint32_t NTRADEOFF = 15;
static const uint32_t stopAtTrisPerPage[NTRADEOFF] = { 64, 60, 56, 48, 46, 44, 40, 36, 32, 28, 24, 20, 16, 12, 12 };
static const uint32_t stopAtTrisPerLeaf[NTRADEOFF] = { 16, 14, 12, 10,  9,  8,  8,  6,  5,  5,  5,  4,  4,  4,  2 };

struct Interval
{
	uint32_t start, count;
	Interval(uint32_t s, uint32_t c) : start(s), count(c) {}
};

struct SortBoundsPredicate
{
	uint32_t coordIndex;
	const Box3d* allBounds;
	SortBoundsPredicate(uint32_t coordIndex_, const Box3d* allBounds_) : coordIndex(coordIndex_), allBounds(allBounds_)
	{}

	bool operator()(const uint32_t& idx1, const uint32_t& idx2) const
	{
		float center1 = allBounds[idx1].Min[coordIndex] + allBounds[idx1].Max[coordIndex];
		float center2 = allBounds[idx2].Min[coordIndex] + allBounds[idx2].Max[coordIndex];
		return (center1 < center2);
	}
};

float SAH(const Vector3& v)
{
	return v.x * v.y + v.y * v.z + v.x * v.z;
}

struct SubSortSAH
{
	uint32_t* permuteStart, * tempPermute;
	const Box3d* allBounds;
	float* metricL;
	float* metricR;
	const uint32_t* xOrder, * yOrder, * zOrder;
	const uint32_t* xRanks, * yRanks, * zRanks;
	uint32_t* tempRanks;
	uint32_t nbTotalBounds;
	uint32_t iTradeOff;

	// precompute various values used during sort
	SubSortSAH(
		uint32_t* permute, const Box3d* allBounds_, uint32_t numBounds,
		const uint32_t* xOrder_, const uint32_t* yOrder_, const uint32_t* zOrder_,
		const uint32_t* xRanks_, const uint32_t* yRanks_, const uint32_t* zRanks_, float sizePerfTradeOff01)
		: permuteStart(permute), allBounds(allBounds_),
		xOrder(xOrder_), yOrder(yOrder_), zOrder(zOrder_),
		xRanks(xRanks_), yRanks(yRanks_), zRanks(zRanks_), nbTotalBounds(numBounds)
	{
		metricL = new float[numBounds];
		metricR = new float[numBounds];
		tempPermute = new uint32_t[numBounds * 2 + 1];
		tempRanks = new uint32_t[numBounds];
		iTradeOff = std::min(uint32_t(std::max<float>(0.0f, sizePerfTradeOff01) * NTRADEOFF), NTRADEOFF - 1);
	}

	~SubSortSAH()
	{
		delete[]metricL;
		delete[]metricR;
		delete[]tempPermute;
		delete[]tempRanks;
	}

	uint32_t Split(uint32_t* Permute, uint32_t ClusterSize)
	{
		if (ClusterSize <= 1)
			return 0;
		if (ClusterSize == 2)
			return 1;

		int minCount = ClusterSize >= 4 ? 2 : 1;
		int splitStartL = minCount;
		int splitEndL = int(ClusterSize - minCount);
		int splitStartR = int(ClusterSize - splitStartL);
		int splitEndR = int(ClusterSize - splitEndL);
		assert(splitEndL - splitStartL == splitStartR - splitEndR);
		assert(splitStartL <= splitEndL);
		assert(splitStartR >= splitEndR);
		assert(splitEndR >= 1);
		assert(splitEndL < int(ClusterSize));

		float minMetric[3];
		uint32_t minMetricSplit[3];
		const uint32_t* ranks3[3] = { xRanks, yRanks, zRanks };
		const uint32_t* orders3[3] = { xOrder, yOrder, zOrder };
		for (uint32_t coordIndex = 0; coordIndex <= 2; coordIndex++)
		{
			const uint32_t* rank = ranks3[coordIndex];
			const uint32_t* order = orders3[coordIndex];

			if (ClusterSize == nbTotalBounds)
			{
				for (uint32_t i = 0; i < ClusterSize; i++)
					tempPermute[i] = order[i];
			}
			else
			{
				for (uint32_t i = 0; i < ClusterSize; i++)
					tempRanks[i] = rank[Permute[i]];
				std::sort(tempRanks, tempRanks + ClusterSize);
				for (uint32_t i = 0; i < ClusterSize; i++)
					tempPermute[i] = order[tempRanks[i]];
			}

			Vector3 boundsLmn = allBounds[tempPermute[0]].Min;
			Vector3 boundsLmx = allBounds[tempPermute[0]].Max;

			for (int ii = 1; ii < splitStartL; ii++)
			{
				boundsLmn = boundsLmn.Min(allBounds[tempPermute[ii]].Min);
				boundsLmx = boundsLmx.Max(allBounds[tempPermute[ii]].Max);
			}

			uint32_t countL0 = 0;
			for (int ii = splitStartL; ii <= splitEndL; ii++)
			{
				boundsLmn = boundsLmn.Min(allBounds[tempPermute[ii]].Min);
				boundsLmx = boundsLmx.Max(allBounds[tempPermute[ii]].Max);
				metricL[countL0++] = SAH(boundsLmx - boundsLmn);
			}

			Vector3 boundsRmn = allBounds[tempPermute[ClusterSize - 1]].Min;
			Vector3 boundsRmx = allBounds[tempPermute[ClusterSize - 1]].Max;
			for (int ii = int(ClusterSize - 2); ii > splitStartR; ii--)
			{
				boundsRmn = boundsRmn.Min(allBounds[tempPermute[ii]].Min);
				boundsRmx = boundsRmx.Max(allBounds[tempPermute[ii]].Max);
			}

			uint32_t countR0 = 0;
			for (int ii = splitStartR; ii >= splitEndR; ii--)
			{
				boundsRmn = boundsRmn.Min(allBounds[tempPermute[ii]].Min);
				boundsRmx = boundsRmx.Max(allBounds[tempPermute[ii]].Max);
				metricR[countR0++] = SAH(boundsRmx - boundsRmn);
			}

			assert((countL0 == countR0) && (countL0 == uint32_t(splitEndL - splitStartL + 1)));

			uint32_t minMetricSplitPosition = 0;
			float minMetricLocal = FLT_MAX;
			const int hsI32 = int(ClusterSize / 2);
			const int splitRange = (splitEndL - splitStartL + 1);
			for (int ii = 0; ii < splitRange; ii++)
			{
				float countL = float(ii + minCount);
				float countR = float(splitRange - ii - 1 + minCount);
				assert(uint32_t(countL + countR) == ClusterSize);

				const float metric = (countL * metricL[ii] + countR * metricR[splitRange - ii - 1]);
				const uint32_t splitPos = uint32_t(ii + splitStartL);
				if (metric < minMetricLocal || (metric <= minMetricLocal && abs((int)splitPos - hsI32) < abs((int)minMetricSplitPosition - hsI32)))
				{
					minMetricLocal = metric;
					minMetricSplitPosition = splitPos;
				}
			}

			minMetric[coordIndex] = minMetricLocal;
			minMetricSplit[coordIndex] = minMetricSplitPosition;
		}

		uint32_t bestIndex = 2;
		if (minMetric[0] <= minMetric[1] && minMetric[0] <= minMetric[2])
			bestIndex = 0;
		else if (minMetric[1] <= minMetric[2])
			bestIndex = 1;

		const uint32_t* rank = ranks3[bestIndex];
		const uint32_t* order = orders3[bestIndex];
		if (ClusterSize == nbTotalBounds)
		{
			for (uint32_t i = 0; i < ClusterSize; i++)
				Permute[i] = order[i];
		}
		else
		{
			for (uint32_t i = 0; i < ClusterSize; i++)
				tempRanks[i] = rank[Permute[i]];
			std::sort(tempRanks, tempRanks + ClusterSize);
			for (uint32_t i = 0; i < ClusterSize; i++)
				Permute[i] = order[tempRanks[i]];
		}

		uint32_t splitPoint = minMetricSplit[bestIndex];
		if (ClusterSize == 3 && splitPoint == 0)
			splitPoint = 1;
		return splitPoint;
	}

	float ComputeSA(const uint32_t* permute, const Interval& split)
	{
		assert(split.count >= 1);
		Vector3 bmn = allBounds[permute[split.start]].Min;
		Vector3 bmx = allBounds[permute[split.start]].Max;
		for (uint32_t i = 1; i < split.count; i++)
		{
			const Box3d& b1 = allBounds[permute[split.start + i]];
			bmn = bmn.Min(b1.Min);
			bmx = bmx.Max(b1.Max);
		}

		return SAH(bmx - bmn);
	}

	void Sort4(uint32_t* Permute, uint32_t ClusterSize, std::vector<TreeNode>& resultTree, uint32_t& OutMaxDepth, uint32_t Depth = 0, TreeNode* parentNode = NULL)
	{
		if (Depth == 0)
			OutMaxDepth = 1;
		else
			OutMaxDepth = std::max(OutMaxDepth, Depth + 1);

		uint32_t splitPos[SIMD_WIDTH];
		for (uint32_t j = 0; j < SIMD_WIDTH; j++)
			splitPos[j] = j + 1;

		if (ClusterSize >= SIMD_WIDTH)
		{
			std::vector<Interval> splits;
			splits.push_back(Interval(0, ClusterSize));
			for (uint32_t iSplit = 0; iSplit < SIMD_WIDTH - 1; iSplit++)
			{
				float maxSAH = -FLT_MAX;
				uint32_t maxSplit = 0xFFFFffff;
				for (uint32_t i = 0; i < splits.size(); i++)
				{
					if (splits[i].count == 1)
						continue;
					float SAH = ComputeSA(Permute, splits[i]) * splits[i].count;
					if (SAH > maxSAH)
					{
						maxSAH = SAH;
						maxSplit = i;
					}
				}
				assert(maxSplit != 0xFFFFffff);

				Interval old = splits[maxSplit];
				assert(old.count > 1);
				uint32_t splitLocal = Split(Permute + old.start, old.count);

				assert(splitLocal >= 1);
				assert(old.count - splitLocal >= 1);
				splits.push_back(Interval(old.start, splitLocal));
				splits.push_back(Interval(old.start + splitLocal, old.count - splitLocal));
				splits[maxSplit] = splits.back();
				splits.pop_back();
				splitPos[iSplit] = old.start + splitLocal;
			}


			assert(splits.size() == SIMD_WIDTH);
			uint32_t sum = 0;
			for (uint32_t j = 0; j < SIMD_WIDTH; j++)
				sum += splits[j].count;
			assert(sum == ClusterSize);
		}
		else
		{
			for (uint32_t i = ClusterSize; i < SIMD_WIDTH - 1; i++)
				splitPos[i] = ClusterSize;
		}

		std::sort(splitPos, splitPos + SIMD_WIDTH - 1);
		splitPos[SIMD_WIDTH - 1] = ClusterSize;

		uint32_t splitStarts[SIMD_WIDTH];
		uint32_t splitCounts[SIMD_WIDTH];
		splitStarts[0] = 0;
		splitCounts[0] = splitPos[0];
		uint32_t sumCounts = splitCounts[0];
		for (uint32_t j = 1; j < SIMD_WIDTH; j++)
		{
			splitStarts[j] = splitPos[j - 1];
			assert(splitStarts[j - 1] <= splitStarts[j]);
			splitCounts[j] = splitPos[j] - splitPos[j - 1];
			assert(splitCounts[j] > 0 || ClusterSize < SIMD_WIDTH);
			sumCounts += splitCounts[j];
			assert(splitStarts[j - 1] + splitCounts[j - 1] <= splitStarts[j]);
		}
		assert(sumCounts == ClusterSize);
		assert(splitStarts[SIMD_WIDTH - 1] + splitCounts[SIMD_WIDTH - 1] <= ClusterSize);

		bool stop_split = (ClusterSize <= stopAtTrisPerPage[iTradeOff]);
		for (uint32_t s = 0; s < SIMD_WIDTH; s++)
		{
			if (splitCounts[s] > 16)
				stop_split = false;
		}

		for (uint32_t s = 0; s < SIMD_WIDTH; s++)
		{
			TreeNode rtn;
			uint32_t splitCount = splitCounts[s];
			if (splitCount > 0)
			{
				Box3d b = allBounds[Permute[splitStarts[s]]];
				float sahMin = SAH(b.Max - b.Min);
				float sahMax = sahMin;
				for (uint32_t i = 1; i < splitCount; i++)
				{
					uint32_t localIndex = i + splitStarts[s];
					const Box3d& b1 = allBounds[Permute[localIndex]];
					float sah1 = SAH(b1.Max - b1.Min);
					sahMin = std::min(sahMin, sah1);
					sahMax = std::max(sahMax, sah1);
					b.Grow(b1);
				}

				rtn.bounds.Min = b.Min;
				rtn.bounds.Max = b.Max;

				bool okSAH = (sahMax / sahMin < 40.0f);
				if (!okSAH)
				{
					stop_split = false;
				}

				bool stopSplitting = splitCount <= 2 ||
					(okSAH && splitCount <= 3) ||
					stop_split ||
					splitCount <= stopAtTrisPerLeaf[iTradeOff];
				if (stopSplitting)
				{
					rtn.NextNodeIndex = int(splitStarts[s] + (Permute - permuteStart));
					rtn.LeafCount = int(splitCount);
					assert(splitCount <= 16);
				}
				else
				{
					rtn.NextNodeIndex = -1;
					rtn.LeafCount = 0;
				}
			}
			else
			{
				assert(splitCount == 0);
				rtn.bounds.SetEmpty();
				rtn.NextNodeIndex = -1;
				rtn.LeafCount = -1;
			}
			resultTree.push_back(rtn);
		}

		if (stop_split)
			return;

		uint32_t parentIndex = (uint32_t)resultTree.size() - SIMD_WIDTH;
		for (uint32_t s = 0; s < SIMD_WIDTH; s++)
		{
			TreeNode* sParent = &resultTree[parentIndex + s];
			if (sParent->LeafCount == 0)
			{
				sParent->NextNodeIndex = int(resultTree.size());
				Sort4(Permute + splitStarts[s], splitCounts[s], resultTree, OutMaxDepth, Depth + 1, sParent);
			}
		}
	}
};


void MeshBVH4::BuildFromBounds(MeshBVH4& bvh, const std::vector<Box3d>& allBounds, std::vector<uint32_t>& Permute, const Box3d& meshBounds)
{
	const uint32_t numBounds = (uint32_t)allBounds.size();
	Permute.resize(numBounds + 1);
	for (uint32_t j = 0; j < numBounds; j++)
		Permute[j] = j;
	Permute[numBounds] = 0x7FFFFFFF;

	std::vector<TreeNode> resultTree;
	resultTree.reserve(numBounds * 2);

	uint32_t maxDepth = 0;

	const bool fast_build = false;
	if (!fast_build)
	{
		std::vector<uint32_t> xRanks(numBounds), yRanks(numBounds), zRanks(numBounds), xOrder(numBounds), yOrder(numBounds), zOrder(numBounds);
		memcpy(&xOrder[0], &Permute[0], sizeof(xOrder[0]) * numBounds);
		memcpy(&yOrder[0], &Permute[0], sizeof(yOrder[0]) * numBounds);
		memcpy(&zOrder[0], &Permute[0], sizeof(zOrder[0]) * numBounds);
		std::sort(xOrder.begin(), xOrder.end(), SortBoundsPredicate(0, &allBounds[0]));
		for (uint32_t i = 0; i < numBounds; i++)
			xRanks[xOrder[i]] = i;
		std::sort(yOrder.begin(), yOrder.end(), SortBoundsPredicate(1, &allBounds[0]));
		for (uint32_t i = 0; i < numBounds; i++)
			yRanks[yOrder[i]] = i;
		std::sort(zOrder.begin(), zOrder.end(), SortBoundsPredicate(2, &allBounds[0]));
		for (uint32_t i = 0; i < numBounds; i++)
			zRanks[zOrder[i]] = i;

		SubSortSAH ss(&Permute[0], &allBounds[0], numBounds, &xOrder[0], &yOrder[0], &zOrder[0], &xRanks[0], &yRanks[0], &zRanks[0], 1.0f);
		ss.Sort4(&Permute[0], numBounds, resultTree, maxDepth);
	}
	else
	{
		// use fast build path
		// assert(hint == PxMeshCookingHint::eCOOKING_PERFORMANCE);
		// SubSortQuick ss(permute.begin(), allBounds, numBounds, sizePerfTradeOff01);
		// Box3d discard((Box3d::U()));
		// ss.sort4(permute.begin(), permute.size() - 1, resultTree, maxLevels, discard); // AP scaffold: need to implement build speed/runtime perf slider
	}

	assert(Permute[numBounds] == 0x7FFFFFFF);
	Permute.pop_back();

	uint32_t nLeafTris = 0;
	uint32_t nLeaves = 0;
	int maxLeafTris = 0;
	for (uint32_t i = 0; i < resultTree.size(); i++)
	{
		int leafCount = resultTree[i].LeafCount;
		if (leafCount > 0)
		{
			nLeaves++;
			nLeafTris += leafCount;
			if (leafCount > maxLeafTris)
				maxLeafTris = leafCount;
		}
	}

	static_assert(SIMD_WIDTH * sizeof(BVHNode) == sizeof(BVHNodeBatch), "sizeof(BVHNodeBatch) is not correct");

	std::vector<BVHNode> Nodes;
	int firstEmptyIndex = -1;
	int Count = (int)resultTree.size();
	Nodes.reserve(Count);

	for (int i = 0; i < Count; i++)
	{
		TreeNode& tr = resultTree[i];
		BVHNode n;
		n.SetLeaf(tr.LeafCount > 0);
		if (tr.NextNodeIndex == -1)
		{
			if (firstEmptyIndex == -1)
			{
				firstEmptyIndex = (int)Nodes.size();
			}
			n.SetEmpty();

			n.Data = firstEmptyIndex * sizeof(BVHNode);
			assert((n.Data & 1) == 0);
			n.SetLeaf(true);
		}
		else
		{
			n.minx = tr.bounds.Min.x;
			n.miny = tr.bounds.Min.y;
			n.minz = tr.bounds.Min.z;
			n.maxx = tr.bounds.Max.x;
			n.maxy = tr.bounds.Max.y;
			n.maxz = tr.bounds.Max.z;
			if (tr.LeafCount > 0)
			{
				n.Data = uint32_t(tr.NextNodeIndex);
				assert(tr.LeafCount > 0);
				assert(tr.LeafCount <= 16);
				assert(n.Data < numBounds);
				assert(n.Data + tr.LeafCount <= numBounds);
				LeafNode lt(tr.LeafCount, n.Data);
				n.Data = lt.Data;
				assert(n.IsLeaf());
			}
			else
			{
				for (uint32_t s = 0; s < SIMD_WIDTH; s++)
				{
					const TreeNode& child = resultTree[tr.NextNodeIndex + s];
					assert(child.LeafCount == -1 || child.bounds.IsInside(tr.bounds));
				}

				n.Data = uint32_t(tr.NextNodeIndex * sizeof(BVHNode));
				assert(n.Data % SIMD_WIDTH == 0);
				n.SetLeaf(false);
			}
		}
		Nodes.push_back(n);
	}

	assert(Nodes.size() % SIMD_WIDTH == 0);
	bvh.NumNodes = (uint32_t)Nodes.size();
	bvh.NumBatches = bvh.NumNodes / SIMD_WIDTH;
	bvh.Memory = new uint8_t[sizeof(BVHNodeBatch) * bvh.NumBatches + 127];
	bvh.BatchPtr = (BVHNodeBatch*)AlignMemory(bvh.Memory, 128);
	bvh.BoundsMin = Vector4(meshBounds.Min, 0.0f);
	bvh.BoundsMax = Vector4(meshBounds.Max, 0.0f);
	bvh.BatchSize = SIMD_WIDTH;
	bvh.MaxDepth = maxDepth;
	assert(bvh.NumNodes % SIMD_WIDTH == 0);
	bvh.NumRoots = 1;
	bvh.Flags |= USER_ALLOCATED;

	for (uint32_t j = 0; j < bvh.NumBatches; j++)
	{
		BVHNodeBatch& page = bvh.BatchPtr[j];
		for (uint32_t k = 0; k < SIMD_WIDTH; k++)
		{
			const BVHNode& n = Nodes[j * SIMD_WIDTH + k];
			page.maxx[k] = n.maxx;
			page.maxy[k] = n.maxy;
			page.maxz[k] = n.maxz;
			page.minx[k] = n.minx;
			page.miny[k] = n.miny;
			page.minz[k] = n.minz;
			page.Data[k] = n.Data;
		}
	}
}

uint32_t BVHNodeBatch::GetNodeCount() const
{
	for (int j = 0; j < SIMD_WIDTH; j++)
		if (minx[j] == FLT_MAX)
			return uint32_t(j);

	return SIMD_WIDTH;
}

/////////////////////////////////////////////////////////////////////////
BVHNode BVHNodeBatch::GetNode(const uint32_t nodeIndex) const
{
	assert(nodeIndex < SIMD_WIDTH);
	BVHNode r;
	r.minx = minx[nodeIndex];
	r.miny = miny[nodeIndex];
	r.minz = minz[nodeIndex];
	r.maxx = maxx[nodeIndex];
	r.maxy = maxy[nodeIndex];
	r.maxz = maxz[nodeIndex];
	r.Data = Data[nodeIndex];
	return r;
}

/////////////////////////////////////////////////////////////////////////
Box3d BVHNodeBatch::ComputeBounds()
{
	float _minx = FLT_MAX, _miny = FLT_MAX, _minz = FLT_MAX, _maxx = -FLT_MAX, _maxy = -FLT_MAX, _maxz = -FLT_MAX;
	for (uint32_t j = 0; j < SIMD_WIDTH; j++)
	{
		if (IsEmpty(j))
			continue;
		_minx = std::min(_minx, minx[j]);
		_miny = std::min(_miny, miny[j]);
		_minz = std::min(_minz, minz[j]);
		_maxx = std::max(_maxx, maxx[j]);
		_maxy = std::max(_maxy, maxy[j]);
		_maxz = std::max(_maxz, maxz[j]);
	}
	return Box3d(Vector3(_minx, _miny, _minz), Vector3(_maxx, _maxy, _maxz));
}

/////////////////////////////////////////////////////////////////////////
void BVHNode::Grow(const BVHNodeBatch& page, int nodeIndex)
{
	assert(nodeIndex < SIMD_WIDTH);
	minx = std::min(minx, page.minx[nodeIndex]);
	miny = std::min(miny, page.miny[nodeIndex]);
	minz = std::min(minz, page.minz[nodeIndex]);
	maxx = std::max(maxx, page.maxx[nodeIndex]);
	maxy = std::max(maxy, page.maxy[nodeIndex]);
	maxz = std::max(maxz, page.maxz[nodeIndex]);
}

void BVHNode::Grow(const BVHNode& node)
{
	minx = std::min(minx, node.minx); miny = std::min(miny, node.miny); minz = std::min(minz, node.minz);
	maxx = std::max(maxx, node.maxx); maxy = std::max(maxy, node.maxy); maxz = std::max(maxz, node.maxz);
}