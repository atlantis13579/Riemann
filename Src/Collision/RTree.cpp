#pragma once

#include "RTree.h"
#include "../Maths/SIMD.h"

/*
bool RTree::load(PxInputStream& stream, uint32_t meshVersion, bool mismatch_)	// PT: 'meshVersion' is the PX_MESH_VERSION from cooked file
{
	PX_UNUSED(meshVersion);

	release();

	PxI8 a, b, c, d;
	readChunk(a, b, c, d, stream);
	if (a != 'R' || b != 'T' || c != 'R' || d != 'E')
		return false;

	bool mismatch;
	uint32_t fileVersion;
	if (!readBigEndianVersionNumber(stream, mismatch_, fileVersion, mismatch))
		return false;

	readFloatBuffer(&mBoundsMin.x, 4, mismatch, stream);
	readFloatBuffer(&mBoundsMax.x, 4, mismatch, stream);
	readFloatBuffer(&mInvDiagonal.x, 4, mismatch, stream);
	readFloatBuffer(&mDiagonalScaler.x, 4, mismatch, stream);
	mPageSize = readDword(mismatch, stream);
	mNumRootPages = readDword(mismatch, stream);
	mNumLevels = readDword(mismatch, stream);
	mTotalNodes = readDword(mismatch, stream);
	mTotalPages = readDword(mismatch, stream);
	uint32_t unused = readDword(mismatch, stream); PX_UNUSED(unused); // backwards compatibility
	mPages = static_cast<RTreePage*>(Ps::AlignedAllocator<128>().allocate(sizeof(RTreePage) * mTotalPages, __FILE__, __LINE__));
	Cm::markSerializedMem(mPages, sizeof(RTreePage) * mTotalPages);
	for (uint32_t j = 0; j < mTotalPages; j++)
	{
		readFloatBuffer(mPages[j].minx, RTREE_N, mismatch, stream);
		readFloatBuffer(mPages[j].miny, RTREE_N, mismatch, stream);
		readFloatBuffer(mPages[j].minz, RTREE_N, mismatch, stream);
		readFloatBuffer(mPages[j].maxx, RTREE_N, mismatch, stream);
		readFloatBuffer(mPages[j].maxy, RTREE_N, mismatch, stream);
		readFloatBuffer(mPages[j].maxz, RTREE_N, mismatch, stream);
		ReadDwordBuffer(mPages[j].ptrs, RTREE_N, mismatch, stream);
	}
	return true;
}
*/

void		RTree::validate()
{
	for (uint32_t j = 0; j < mNumRootPages; j++)
	{
		RTreeNodeQ rootBounds;
		mPages[j].computeBounds(rootBounds);
		validateRecursive(0, rootBounds, mPages + j);
	}
}

#define RTREE_INFLATION_EPSILON 5e-4f

void RTree::validateRecursive(uint32_t level, RTreeNodeQ parentBounds, RTreePage* page)
{
	static uint32_t validateCounter = 0; // this is to suppress a warning that recursive call has no side effects
	validateCounter++;

	RTreeNodeQ n;
	uint32_t pageNodeCount = page->nodeCount();
	for (uint32_t j = 0; j < pageNodeCount; j++)
	{
		page->getNode(j, n);
		if (page->isEmpty(j))
			continue;
		assert(n.minx >= parentBounds.minx); assert(n.miny >= parentBounds.miny); assert(n.minz >= parentBounds.minz);
		assert(n.maxx <= parentBounds.maxx); assert(n.maxy <= parentBounds.maxy); assert(n.maxz <= parentBounds.maxz);
		if (!n.isLeaf())
		{
			assert((n.ptr & 1) == 0);
			RTreePage* childPage = reinterpret_cast<RTreePage*>(size_t(mPages) + n.ptr);
			validateRecursive(level + 1, n, childPage);
		}
	}

	RTreeNodeQ recomputedBounds;
	page->computeBounds(recomputedBounds);
	assert((recomputedBounds.minx - parentBounds.minx) <= RTREE_INFLATION_EPSILON);
	assert((recomputedBounds.miny - parentBounds.miny) <= RTREE_INFLATION_EPSILON);
	assert((recomputedBounds.minz - parentBounds.minz) <= RTREE_INFLATION_EPSILON);
	assert((recomputedBounds.maxx - parentBounds.maxx) <= RTREE_INFLATION_EPSILON);
	assert((recomputedBounds.maxy - parentBounds.maxy) <= RTREE_INFLATION_EPSILON);
	assert((recomputedBounds.maxz - parentBounds.maxz) <= RTREE_INFLATION_EPSILON);
}


const VecU32V signMask = U4LoadXYZW((1 << 31), (1 << 31), (1 << 31), (1 << 31));
const Vec4V epsFloat4 = V4Load(1e-9f);
const Vec4V twos = V4Load(2.0f);


void		RTree::traverseRay(const Vector3d& Origin, const Vector3d& Dir, CallbackRaycast *cb, float maxT) const
{
	const uint32_t maxStack = 128;
	uint32_t stack1[maxStack];
	uint32_t* stack = stack1 + 1;

	assert(mPages);
	assert((uintptr_t(mPages) & 127) == 0);
	assert((uintptr_t(this) & 15) == 0);

	uint8_t* treeNodes8 = (uint8_t*)(mPages);

	Vec4V maxT4;
	maxT4 = V4Load(maxT);
	Vec4V rayP = Vec4V_From_PxVec3_WUndefined(Origin);
	Vec4V rayD = Vec4V_From_PxVec3_WUndefined(Dir);
	VecU32V raySign = V4U32and(VecU32V_ReinterpretFrom_Vec4V(rayD), signMask);
	Vec4V rayDAbs = V4Abs(rayD); // abs value of rayD
	Vec4V rayInvD = Vec4V_ReinterpretFrom_VecU32V(V4U32or(raySign, VecU32V_ReinterpretFrom_Vec4V(V4Max(rayDAbs, epsFloat4)))); // clamp near-zero components up to epsilon
	rayD = rayInvD;

	//rayInvD = V4Recip(rayInvD);
	// Newton-Raphson iteration for reciprocal (see wikipedia):
	// X[n+1] = X[n]*(2-original*X[n]), X[0] = V4RecipFast estimate
	//rayInvD = rayInvD*(twos-rayD*rayInvD);
	rayInvD = V4RecipFast(rayInvD); // initial estimate, not accurate enough
	rayInvD = V4Mul(rayInvD, V4NegMulSub(rayD, rayInvD, twos));

	// P+tD=a; t=(a-P)/D
	// t=(a - p.x)*1/d.x = a/d.x +(- p.x/d.x)
	Vec4V rayPinvD = V4NegMulSub(rayInvD, rayP, V4Zero());
	Vec4V rayInvDsplatX = V4SplatElement<0>(rayInvD);
	Vec4V rayInvDsplatY = V4SplatElement<1>(rayInvD);
	Vec4V rayInvDsplatZ = V4SplatElement<2>(rayInvD);
	Vec4V rayPinvDsplatX = V4SplatElement<0>(rayPinvD);
	Vec4V rayPinvDsplatY = V4SplatElement<1>(rayPinvD);
	Vec4V rayPinvDsplatZ = V4SplatElement<2>(rayPinvD);

	assert(RTREE_N == 4 || RTREE_N == 8);
	assert(mNumRootPages > 0);

	uint32_t stackPtr = 0;
	for (int j = mNumRootPages - 1; j >= 0; j--)
		stack[stackPtr++] = j * sizeof(RTreePage);

	__declspec(align(16)) uint32_t resa[4];

	while (stackPtr)
	{
		uint32_t top = stack[--stackPtr];
		if (top & 1) // isLeaf test
		{
			top--;
			float newMaxT = maxT;
			if (!cb->processResults(1, &top, newMaxT))
				return;

			// shrink the ray if newMaxT is reduced compared to the original maxT
			if (maxT != newMaxT)
			{
				assert(newMaxT < maxT);
				maxT = newMaxT;
				maxT4 = V4Load(newMaxT);
			}
			continue;
		}

		RTreePage* tn = reinterpret_cast<RTreePage*>(treeNodes8 + top);

		// 6i load
		Vec4V minx4a = V4LoadA(tn->minx), miny4a = V4LoadA(tn->miny), minz4a = V4LoadA(tn->minz);
		Vec4V maxx4a = V4LoadA(tn->maxx), maxy4a = V4LoadA(tn->maxy), maxz4a = V4LoadA(tn->maxz);

		// 1i disabled test
		// AP scaffold - optimization opportunity - can save 2 instructions here
		VecU32V ignore4a = V4IsGrtrV32u(minx4a, maxx4a); // 1 if degenerate box (empty slot in the page)

		// P+tD=a; t=(a-P)/D
		// t=(a - p.x)*1/d.x = a/d.x +(- p.x/d.x)
		// 6i
		Vec4V tminxa0 = V4MulAdd(minx4a, rayInvDsplatX, rayPinvDsplatX);
		Vec4V tminya0 = V4MulAdd(miny4a, rayInvDsplatY, rayPinvDsplatY);
		Vec4V tminza0 = V4MulAdd(minz4a, rayInvDsplatZ, rayPinvDsplatZ);
		Vec4V tmaxxa0 = V4MulAdd(maxx4a, rayInvDsplatX, rayPinvDsplatX);
		Vec4V tmaxya0 = V4MulAdd(maxy4a, rayInvDsplatY, rayPinvDsplatY);
		Vec4V tmaxza0 = V4MulAdd(maxz4a, rayInvDsplatZ, rayPinvDsplatZ);

		// test half-spaces
		// P+tD=dN
		// t = (d(N,D)-(P,D))/(D,D) , (D,D)=1

		// compute 4x dot products (N,D) and (P,N) for each AABB in the page

		// 6i
		// now compute tnear and tfar for each pair of planes for each box
		Vec4V tminxa = V4Min(tminxa0, tmaxxa0); Vec4V tmaxxa = V4Max(tminxa0, tmaxxa0);
		Vec4V tminya = V4Min(tminya0, tmaxya0); Vec4V tmaxya = V4Max(tminya0, tmaxya0);
		Vec4V tminza = V4Min(tminza0, tmaxza0); Vec4V tmaxza = V4Max(tminza0, tmaxza0);

		// 8i
		Vec4V maxOfNeasa = V4Max(V4Max(tminxa, tminya), tminza);
		Vec4V minOfFarsa = V4Min(V4Min(tmaxxa, tmaxya), tmaxza);
		ignore4a = V4U32or(ignore4a, V4IsGrtrV32u(epsFloat4, minOfFarsa));  // if tfar is negative, ignore since its a ray, not a line
		// AP scaffold: update the build to eliminate 3 more instructions for ignore4a above
		//VecU32V ignore4a = V4IsGrtrV32u(epsFloat4, minOfFarsa);  // if tfar is negative, ignore since its a ray, not a line
		ignore4a = V4U32or(ignore4a, V4IsGrtrV32u(maxOfNeasa, maxT4));  // if tnear is over maxT, ignore this result

		// 2i
		VecU32V resa4 = V4IsGrtrV32u(maxOfNeasa, minOfFarsa); // if 1 => fail
		resa4 = V4U32or(resa4, ignore4a);

		// 1i
		V4U32StoreAligned(resa4, reinterpret_cast<VecU32V*>(resa));

		uint32_t* ptrs = (reinterpret_cast<RTreePage*>(tn))->ptrs;

		stack[stackPtr] = ptrs[0]; stackPtr += (1 + resa[0]); // AP scaffold TODO: use VecU32add
		stack[stackPtr] = ptrs[1]; stackPtr += (1 + resa[1]);
		stack[stackPtr] = ptrs[2]; stackPtr += (1 + resa[2]);
		stack[stackPtr] = ptrs[3]; stackPtr += (1 + resa[3]);
	}
}


/////////////////////////////////////////////////////////////////////////
uint32_t RTree::computeBottomLevelCount(uint32_t multiplier) const
{
	uint32_t topCount = 0, curCount = mNumRootPages;
	const RTreePage* rightMostPage = &mPages[mNumRootPages - 1];
	assert(rightMostPage);
	for (uint32_t level = 0; level < mNumLevels - 1; level++)
	{
		topCount += curCount;
		uint32_t nc = rightMostPage->nodeCount();
		assert(nc > 0 && nc <= RTREE_N);
		// old version pointer, up to PX_MESH_VERSION 8
		uint32_t ptr = (rightMostPage->ptrs[nc - 1]) * multiplier;
		assert(ptr % sizeof(RTreePage) == 0);
		const RTreePage* rightMostPageNext = mPages + (ptr / sizeof(RTreePage));
		curCount = uint32_t(rightMostPageNext - rightMostPage);
		rightMostPage = rightMostPageNext;
	}

	return mTotalPages - topCount;
}

uint32_t RTreePage::nodeCount() const
{
	for (int j = 0; j < RTREE_N; j++)
		if (minx[j] == FLT_MAX)
			return uint32_t(j);

	return RTREE_N;
}

/////////////////////////////////////////////////////////////////////////
void RTreePage::clearNode(uint32_t nodeIndex)
{
	assert(nodeIndex < RTREE_N);
	minx[nodeIndex] = miny[nodeIndex] = minz[nodeIndex] = FLT_MAX; // initialize empty node with sentinels
	maxx[nodeIndex] = maxy[nodeIndex] = maxz[nodeIndex] = -FLT_MAX;
	ptrs[nodeIndex] = 0;
}

/////////////////////////////////////////////////////////////////////////
void RTreePage::getNode(const uint32_t nodeIndex, RTreeNodeQ& r) const
{
	assert(nodeIndex < RTREE_N);
	r.minx = minx[nodeIndex];
	r.miny = miny[nodeIndex];
	r.minz = minz[nodeIndex];
	r.maxx = maxx[nodeIndex];
	r.maxy = maxy[nodeIndex];
	r.maxz = maxz[nodeIndex];
	r.ptr = ptrs[nodeIndex];
}

/////////////////////////////////////////////////////////////////////////
void RTreePage::setEmpty(uint32_t startIndex)
{
	assert(startIndex < RTREE_N);
	for (uint32_t j = startIndex; j < RTREE_N; j++)
		clearNode(j);
}

/////////////////////////////////////////////////////////////////////////
void RTreePage::computeBounds(RTreeNodeQ& newBounds)
{
	float _minx = FLT_MAX, _miny = FLT_MAX, _minz = FLT_MAX, _maxx = -FLT_MAX, _maxy = -FLT_MAX, _maxz = -FLT_MAX;
	for (uint32_t j = 0; j < RTREE_N; j++)
	{
		if (isEmpty(j))
			continue;
		_minx = std::min(_minx, minx[j]);
		_miny = std::min(_miny, miny[j]);
		_minz = std::min(_minz, minz[j]);
		_maxx = std::max(_maxx, maxx[j]);
		_maxy = std::max(_maxy, maxy[j]);
		_maxz = std::max(_maxz, maxz[j]);
	}
	newBounds.minx = _minx;
	newBounds.miny = _miny;
	newBounds.minz = _minz;
	newBounds.maxx = _maxx;
	newBounds.maxy = _maxy;
	newBounds.maxz = _maxz;
}

/////////////////////////////////////////////////////////////////////////
void RTreePage::adjustChildBounds(uint32_t index, const RTreeNodeQ& adjChild)
{
	assert(index < RTREE_N);
	minx[index] = adjChild.minx;
	miny[index] = adjChild.miny;
	minz[index] = adjChild.minz;
	maxx[index] = adjChild.maxx;
	maxy[index] = adjChild.maxy;
	maxz[index] = adjChild.maxz;
}

/////////////////////////////////////////////////////////////////////////
void RTreePage::growChildBounds(uint32_t index, const RTreeNodeQ& child)
{
	assert(index < RTREE_N);
	minx[index] = std::min(minx[index], child.minx);
	miny[index] = std::min(miny[index], child.miny);
	minz[index] = std::min(minz[index], child.minz);
	maxx[index] = std::max(maxx[index], child.maxx);
	maxy[index] = std::max(maxy[index], child.maxy);
	maxz[index] = std::max(maxz[index], child.maxz);
}

/////////////////////////////////////////////////////////////////////////
void RTreePage::copyNode(uint32_t targetIndex, const RTreePage& sourcePage, uint32_t sourceIndex)
{
	assert(targetIndex < RTREE_N);
	assert(sourceIndex < RTREE_N);
	minx[targetIndex] = sourcePage.minx[sourceIndex];
	miny[targetIndex] = sourcePage.miny[sourceIndex];
	minz[targetIndex] = sourcePage.minz[sourceIndex];
	maxx[targetIndex] = sourcePage.maxx[sourceIndex];
	maxy[targetIndex] = sourcePage.maxy[sourceIndex];
	maxz[targetIndex] = sourcePage.maxz[sourceIndex];
	ptrs[targetIndex] = sourcePage.ptrs[sourceIndex];
}

/////////////////////////////////////////////////////////////////////////
void RTreePage::setNode(uint32_t targetIndex, const RTreeNodeQ& sourceNode)
{
	assert(targetIndex < RTREE_N);
	minx[targetIndex] = sourceNode.minx;
	miny[targetIndex] = sourceNode.miny;
	minz[targetIndex] = sourceNode.minz;
	maxx[targetIndex] = sourceNode.maxx;
	maxy[targetIndex] = sourceNode.maxy;
	maxz[targetIndex] = sourceNode.maxz;
	ptrs[targetIndex] = sourceNode.ptr;
}

/////////////////////////////////////////////////////////////////////////
void RTreeNodeQ::grow(const RTreePage& page, int nodeIndex)
{
	assert(nodeIndex < RTREE_N);
	minx = std::min(minx, page.minx[nodeIndex]);
	miny = std::min(miny, page.miny[nodeIndex]);
	minz = std::min(minz, page.minz[nodeIndex]);
	maxx = std::max(maxx, page.maxx[nodeIndex]);
	maxy = std::max(maxy, page.maxy[nodeIndex]);
	maxz = std::max(maxz, page.maxz[nodeIndex]);
}

void RTreeNodeQ::grow(const RTreeNodeQ& node)
{
	minx = std::min(minx, node.minx); miny = std::min(miny, node.miny); minz = std::min(minz, node.minz);
	maxx = std::max(maxx, node.maxx); maxy = std::max(maxy, node.maxy); maxz = std::max(maxz, node.maxz);
}
