

#include "TriangleMesh.h"
#include "Triangle3d.h"
#include "../Maths/SIMD.h"
#include "../Collision/GeometryQuery.h"

#include <algorithm>
#include <array>
#include <vector>

MeshBVH4* TriangleMesh::CreateEmptyBVH()
{
	if (m_BVH == nullptr)
	{
		m_BVH = new MeshBVH4;
	}
	else
	{
		m_BVH->Release();
	}
	return m_BVH;
}

void TriangleMesh::BuildBVH()
{
	if (NumTriangles == 0)
	{
		return;
	}

	std::vector<Box3d> allBounds;
	allBounds.reserve(NumTriangles);

	Box3d meshBounds = Box3d::Empty();

	for (uint32_t i = 0; i < NumTriangles; ++i)
	{
		allBounds.push_back(Box3d(GetVertex(i, 0), GetVertex(i, 1), GetVertex(i, 2)));
		meshBounds.Grow(allBounds.back());
	}

	CreateEmptyBVH();

	std::vector<uint32_t> Permute;
	MeshBVH4::BuildFromBounds(*m_BVH, allBounds, Permute, meshBounds);

	Reorder(Permute);

#if _DEBUG
	m_BVH->Validate((Mesh*)this);
#endif
}

void	TriangleMesh::GetVertIndices(uint32_t triIndex, uint32_t& i0, uint32_t& i1, uint32_t& i2) const
{
	if (Is16bitIndices())
	{
		const uint16_t* p = GetIndices16() + 3 * triIndex;
		i0 = p[0];
		i1 = p[1];
		i2 = p[2];
	}
	else
	{
		const uint32_t* p = GetIndices32() + 3 * triIndex;
		i0 = p[0];
		i1 = p[1];
		i2 = p[2];
	}
}

bool TriangleMesh::OverlapTri(uint32_t HitNode, const Vector3d& Bmin, const Vector3d& Bmax) const
{
	LeafNode currLeaf(HitNode);
	uint32_t NumLeafTriangles = currLeaf.GetNumTriangles();
	uint32_t BaseTriIndex = currLeaf.GetTriangleIndex();

	for (uint32_t i = 0; i < NumLeafTriangles; i++)
	{
		uint32_t i0, i1, i2;
		const uint32_t triangleIndex = BaseTriIndex + i;
		GetVertIndices(triangleIndex, i0, i1, i2);

		const Vector3d& v0 = Vertices[i0];
		const Vector3d& v1 = Vertices[i1];
		const Vector3d& v2 = Vertices[i2];

		bool intersect = Triangle3d::IntersectAABB(v0, v1, v2, Bmin, Bmax);
		if (intersect)
		{
			return true;
		}
	}

	return false;
}

bool TriangleMesh::IntersectAABB(const Vector3d& Bmin, const Vector3d& Bmax) const
{
	const uint32_t maxStack = 128;
	uint32_t stack1[maxStack];
	uint32_t* stack = stack1 + 1;

	assert(m_BVH->BatchPtr);
	assert((uintptr_t(m_BVH->BatchPtr) & 127) == 0);
	assert((uintptr_t(this) & 15) == 0);

	Vec4 nqMin = Vec4_Load_Vector3d(Bmin);
	Vec4 nqMax = Vec4_Load_Vector3d(Bmax);

	Vec4 nqMinx4 = Vec4_SplatElement<0>(nqMin);
	Vec4 nqMiny4 = Vec4_SplatElement<1>(nqMin);
	Vec4 nqMinz4 = Vec4_SplatElement<2>(nqMin);
	Vec4 nqMaxx4 = Vec4_SplatElement<0>(nqMax);
	Vec4 nqMaxy4 = Vec4_SplatElement<1>(nqMax);
	Vec4 nqMaxz4 = Vec4_SplatElement<2>(nqMax);

	uint8_t* batch_ptr = (uint8_t*)(m_BVH->BatchPtr);
	uint32_t* stackPtr = stack;

	assert(SIMD_WIDTH == 4 || SIMD_WIDTH == 8);
	assert(IsPowerOfTwo(m_BVH->BatchSize));

	for (int j = int(m_BVH->NumRoots - 1); j >= 0; j--)
		*stackPtr++ = j * sizeof(BVHNodeBatch);

	uint32_t cacheTopValid = true;
	uint32_t cacheTop = 0;

	do {
		stackPtr--;
		uint32_t top;
		if (cacheTopValid) // branch is faster than lhs
			top = cacheTop;
		else
			top = stackPtr[0];
		assert(!cacheTopValid || stackPtr[0] == cacheTop);
		BVHNodeBatch* tn = reinterpret_cast<BVHNodeBatch*>(batch_ptr + top);
		const uint32_t* Data = (reinterpret_cast<BVHNodeBatch*>(tn))->Data;

		Vec4 minx4 = Vec4_LoadA(tn->minx);
		Vec4 miny4 = Vec4_LoadA(tn->miny);
		Vec4 minz4 = Vec4_LoadA(tn->minz);
		Vec4 maxx4 = Vec4_LoadA(tn->maxx);
		Vec4 maxy4 = Vec4_LoadA(tn->maxy);
		Vec4 maxz4 = Vec4_LoadA(tn->maxz);

		// AABB/AABB overlap test
		BVec4 res0 = Vec4_Greater(nqMinx4, maxx4);
		BVec4 res1 = Vec4_Greater(nqMiny4, maxy4);
		BVec4 res2 = Vec4_Greater(nqMinz4, maxz4);
		BVec4 res3 = Vec4_Greater(minx4, nqMaxx4);
		BVec4 res4 = Vec4_Greater(miny4, nqMaxy4);
		BVec4 res5 = Vec4_Greater(minz4, nqMaxz4);
		BVec4 resx = BVec4_Or(BVec4_Or(BVec4_Or(res0, res1), BVec4_Or(res2, res3)), BVec4_Or(res4, res5));
		alignas(16) uint32_t resa[SIMD_WIDTH];

		UVec4 res4x = UVec4_Load_BVec4(resx);
		UVec4_StoreA(res4x, resa);

		cacheTopValid = false;
		for (uint32_t i = 0; i < SIMD_WIDTH; i++)
		{
			uint32_t ptr = Data[i] & ~1; // clear the isLeaf bit
			if (resa[i])
				continue;
			if (tn->Data[i] & 1)
			{
				bool overlap = OverlapTri(ptr, Bmin, Bmax);
				if (overlap)
				{
					return true;
				}
			}
			else
			{
				*(stackPtr++) = ptr;
				cacheTop = ptr;
				cacheTopValid = true;
			}
		}
	} while (stackPtr > stack);

	return false;
}

bool	TriangleMesh::RayIntersectTri(uint32_t HitNode, const Vector3d& Origin, const Vector3d& Dir, const TriMeshHitOption& Option, TriMeshHitResult* Result) const
{
	LeafNode currLeaf(HitNode);
	uint32_t NumLeafTriangles = currLeaf.GetNumTriangles();
	uint32_t BaseTriIndex = currLeaf.GetTriangleIndex();
	bool hit = false;

	for (uint32_t i = 0; i < NumLeafTriangles; i++)
	{
		Result->AddTestCount(1);

		uint32_t i0, i1, i2;
		const uint32_t triangleIndex = BaseTriIndex + i;
		GetVertIndices(triangleIndex, i0, i1, i2);

		const Vector3d& v0 = Vertices[i0];
		const Vector3d& v1 = Vertices[i1];
		const Vector3d& v2 = Vertices[i2];

		float t;
		bool intersect = Triangle3d::RayIntersectTriangle(Origin, Dir, v0, v1, v2, &t);
		if (!intersect || t > Option.maxDist)
		{
			continue;
		}

		hit = true;

		if (!Option.hitNearest)
		{
			Result->hitTime = t;
			Result->hitIndex = triangleIndex;
			return true;
		}

		if (t < Result->hitTime)
		{
			Result->hitTime = t;
			Result->hitIndex = triangleIndex;
		}
	}

	return hit;
}

bool TriangleMesh::IntersectRay(const Vector3d& Origin, const Vector3d& Dir, const TriMeshHitOption& Option, TriMeshHitResult* Result) const
{
	if (m_BVH == nullptr)
	{
		return false;
	}

	Result->hitTime = FLT_MAX;

	const UVec4 signMask = UVec4_LoadXYZW((1 << 31), (1 << 31), (1 << 31), (1 << 31));
	const Vec4 epsFloat4 = Vec4_Load(1e-9f);
	const Vec4 twos = Vec4_Load(2.0f);
	const uint32_t maxStack = 128;
	uint32_t stack1[maxStack];
	uint32_t* stack = stack1 + 1;

	assert(m_BVH->BatchPtr);
	assert((uintptr_t(m_BVH->BatchPtr) & 127) == 0);
	// assert((uintptr_t(this) & 15) == 0);

	float maxT = Option.maxDist;

	uint8_t* batch_ptr = (uint8_t*)(m_BVH->BatchPtr);
	Vec4 maxT4;
	maxT4 = Vec4_Load(maxT);
	Vec4 rayP = Vec4_Load_Vector3d(Origin);
	Vec4 rayD = Vec4_Load_Vector3d(Dir);
	UVec4 raySign = UVec4_AND(VecU32V_ReinterpretFrom_Vec4V(rayD), signMask);
	Vec4 rayDAbs = Vec4_Abs(rayD); // abs value of rayD
	Vec4 rayInvD = Vec4V_ReinterpretFrom_VecU32V(UVec4_OR(raySign, VecU32V_ReinterpretFrom_Vec4V(Vec4_Max(rayDAbs, epsFloat4)))); // clamp near-zero components up to epsilon
	rayD = rayInvD;

	//rayInvD = V4Recip(rayInvD);
	// Newton-Raphson iteration for reciprocal (see wikipedia):
	// X[n+1] = X[n]*(2-original*X[n]), X[0] = V4RecipFast estimate
	//rayInvD = rayInvD*(twos-rayD*rayInvD);
	rayInvD = Vec4_RecipFast(rayInvD); // initial estimate, not accurate enough
	rayInvD = Vec4_Mul(rayInvD, Vec4_NegMulSub(rayD, rayInvD, twos));

	// P+tD=a; t=(a-P)/D
	// t=(a - p.x)*1/d.x = a/d.x +(- p.x/d.x)
	Vec4 rayPinvD = Vec4_NegMulSub(rayInvD, rayP, Vec4_Zero());
	Vec4 rayInvDsplatX = Vec4_SplatElement<0>(rayInvD);
	Vec4 rayInvDsplatY = Vec4_SplatElement<1>(rayInvD);
	Vec4 rayInvDsplatZ = Vec4_SplatElement<2>(rayInvD);
	Vec4 rayPinvDsplatX = Vec4_SplatElement<0>(rayPinvD);
	Vec4 rayPinvDsplatY = Vec4_SplatElement<1>(rayPinvD);
	Vec4 rayPinvDsplatZ = Vec4_SplatElement<2>(rayPinvD);

	assert(SIMD_WIDTH == 4 || SIMD_WIDTH == 8);
	assert(m_BVH->NumRoots > 0);

	uint32_t stackPtr = 0;
	for (int j = m_BVH->NumRoots - 1; j >= 0; j--)
		stack[stackPtr++] = j * sizeof(BVHNodeBatch);

	alignas(16) uint32_t resa[4];

	while (stackPtr)
	{
		uint32_t top = stack[--stackPtr];
		if (top & 1)
		{
			top--;
			bool intersect = RayIntersectTri(top, Origin, Dir, Option, Result);
			if (intersect)
			{
				if (!Option.hitNearest)
				{
					return true;
				}

				if (Result->hitTime < maxT)
				{
					maxT = Result->hitTime;
					maxT4 = Vec4_Load(maxT);
				}
			}

			continue;
		}

		Result->AddTestCount(2);

		BVHNodeBatch* tn = reinterpret_cast<BVHNodeBatch*>(batch_ptr + top);

		// 6i load
		Vec4 minx4a = Vec4_LoadA(tn->minx), miny4a = Vec4_LoadA(tn->miny), minz4a = Vec4_LoadA(tn->minz);
		Vec4 maxx4a = Vec4_LoadA(tn->maxx), maxy4a = Vec4_LoadA(tn->maxy), maxz4a = Vec4_LoadA(tn->maxz);

		// 1i disabled test
		// AP scaffold - optimization opportunity - can save 2 instructions here
		UVec4 ignore4a = Vec4_Greater(minx4a, maxx4a); // 1 if degenerate box (empty slot in the page)

		// P+tD=a; t=(a-P)/D
		// t=(a - p.x)*1/d.x = a/d.x +(- p.x/d.x)
		// 6i
		Vec4 tminxa0 = Vec4_MAdd(minx4a, rayInvDsplatX, rayPinvDsplatX);
		Vec4 tminya0 = Vec4_MAdd(miny4a, rayInvDsplatY, rayPinvDsplatY);
		Vec4 tminza0 = Vec4_MAdd(minz4a, rayInvDsplatZ, rayPinvDsplatZ);
		Vec4 tmaxxa0 = Vec4_MAdd(maxx4a, rayInvDsplatX, rayPinvDsplatX);
		Vec4 tmaxya0 = Vec4_MAdd(maxy4a, rayInvDsplatY, rayPinvDsplatY);
		Vec4 tmaxza0 = Vec4_MAdd(maxz4a, rayInvDsplatZ, rayPinvDsplatZ);

		// test half-spaces
		// P+tD=dN
		// t = (d(N,D)-(P,D))/(D,D) , (D,D)=1

		// compute 4x dot products (N,D) and (P,N) for each AABB in the page

		// 6i
		// now compute tnear and tfar for each pair of planes for each box
		Vec4 tminxa = Vec4_Min(tminxa0, tmaxxa0); Vec4 tmaxxa = Vec4_Max(tminxa0, tmaxxa0);
		Vec4 tminya = Vec4_Min(tminya0, tmaxya0); Vec4 tmaxya = Vec4_Max(tminya0, tmaxya0);
		Vec4 tminza = Vec4_Min(tminza0, tmaxza0); Vec4 tmaxza = Vec4_Max(tminza0, tmaxza0);

		// 8i
		Vec4 maxOfNeasa = Vec4_Max(Vec4_Max(tminxa, tminya), tminza);
		Vec4 minOfFarsa = Vec4_Min(Vec4_Min(tmaxxa, tmaxya), tmaxza);
		ignore4a = UVec4_OR(ignore4a, Vec4_Greater(epsFloat4, minOfFarsa));  // if tfar is negative, ignore since its a ray, not a line
		// AP scaffold: update the build to eliminate 3 more instructions for ignore4a above
		//VecU32V ignore4a = Vec4_Greater(epsFloat4, minOfFarsa);  // if tfar is negative, ignore since its a ray, not a line
		ignore4a = UVec4_OR(ignore4a, Vec4_Greater(maxOfNeasa, maxT4));  // if tnear is over maxT, ignore this result

		// 2i
		UVec4 resa4 = Vec4_Greater(maxOfNeasa, minOfFarsa); // if 1 => fail
		resa4 = UVec4_OR(resa4, ignore4a);

		// 1i
		UVec4_StoreAA(resa4, reinterpret_cast<UVec4*>(resa));

		uint32_t* ptrs = (reinterpret_cast<BVHNodeBatch*>(tn))->Data;

		stack[stackPtr] = ptrs[0]; stackPtr += (1 + resa[0]); // AP scaffold TODO: use VecU32add
		stack[stackPtr] = ptrs[1]; stackPtr += (1 + resa[1]);
		stack[stackPtr] = ptrs[2]; stackPtr += (1 + resa[2]);
		stack[stackPtr] = ptrs[3]; stackPtr += (1 + resa[3]);
	}

	return Result->hitTime != FLT_MAX;
}

bool	TriangleMesh::IntersectRay(const Vector3d& Origin, const Vector3d& Dir, float* t) const
{
	TriMeshHitOption Option;
	Option.hitNearest = true;
	Option.maxDist = FLT_MAX;

	TriMeshHitResult Result = { 0 };
	if (IntersectRay(Origin, Dir, Option, &Result))
	{
		*t = Result.hitTime;
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

int TriangleMesh::GetSupportFace(const Vector3d& dir, Vector3d* FacePoints) const
{
	assert(false);
	return 0;
}
