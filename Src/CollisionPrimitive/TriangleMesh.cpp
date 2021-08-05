

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

void* TriangleMesh::AllocMemory(int Size, int Width)
{
	if (m_Memory != nullptr)
	{
		delete[]m_Memory;
	}
	m_Memory = new char[Size + Width - 1];
	return AlignMemory(m_Memory, Width);
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
	MeshBVH4::BuildFromBounds(*m_BVH, m_Memory, allBounds, Permute, meshBounds);

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

bool	TriangleMesh::IntersectTri(uint32_t HitNode, const Vector3d& Origin, const Vector3d& Dir, const TriMeshHitOption& Option, TriMeshHitResult* Result) const
{
	LeafNode currLeaf(HitNode);
	uint32_t NumLeafTriangles = currLeaf.GetNumTriangles();
	uint32_t BaseTriIndex = currLeaf.GetTriangleIndex();
	bool hit = false;

	for (uint32_t i = 0; i < NumLeafTriangles; i++)
	{
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

		if (!Option.hitClosest)
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

bool TriangleMesh::IntersectAABB(const Vector3d& Bmin, const Vector3d& Bmax) const
{
	// TODO
	return BoundingVolume.Intersect(Bmin, Bmax);
}

const VecU32V signMask = U4LoadXYZW((1 << 31), (1 << 31), (1 << 31), (1 << 31));
const Vec4V epsFloat4 = V4Load(1e-9f);
const Vec4V twos = V4Load(2.0f);

bool	TriangleMesh::IntersectRay(const Vector3d& Origin, const Vector3d& Dir, float* t) const
{
	TriMeshHitOption Option;
	Option.hitClosest = true;
	Option.maxDist = FLT_MAX;

	TriMeshHitResult Result;
	if (IntersectRay(Origin, Dir, Option, &Result))
	{
		*t = Result.hitTime;
		return true;
	}
	return false;
}

bool TriangleMesh::IntersectRay(const Vector3d& Origin, const Vector3d& Dir, const TriMeshHitOption& Option, TriMeshHitResult* Result) const
{
	if (m_BVH == nullptr)
	{
		return false;
	}

	Result->hitTime = FLT_MAX;

	const uint32_t maxStack = 128;
	uint32_t stack1[maxStack];
	uint32_t* stack = stack1 + 1;

	assert(m_BVH->BatchPtr);
	assert((uintptr_t(m_BVH->BatchPtr) & 127) == 0);
	assert((uintptr_t(this) & 15) == 0);

	float maxT = Option.maxDist;

	uint8_t* batch_ptr = (uint8_t*)(m_BVH->BatchPtr);
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

	assert(SIMD_WIDTH == 4 || SIMD_WIDTH == 8);
	assert(m_BVH->NumRoots > 0);

	uint32_t stackPtr = 0;
	for (int j = m_BVH->NumRoots - 1; j >= 0; j--)
		stack[stackPtr++] = j * sizeof(BVHNodeBatch);

	__declspec(align(16)) uint32_t resa[4];

	while (stackPtr)
	{
		uint32_t top = stack[--stackPtr];
		if (top & 1)
		{
			top--;
			bool intersect = IntersectTri(top, Origin, Dir, Option, Result);
			if (intersect)
			{
				if (!Option.hitClosest)
				{
					return true;
				}

				if (Result->hitTime < maxT)
				{
					maxT = Result->hitTime;
					maxT4 = V4Load(maxT);
				}
			}

			continue;
		}

		BVHNodeBatch* tn = reinterpret_cast<BVHNodeBatch*>(batch_ptr + top);

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

		uint32_t* ptrs = (reinterpret_cast<BVHNodeBatch*>(tn))->Data;

		stack[stackPtr] = ptrs[0]; stackPtr += (1 + resa[0]); // AP scaffold TODO: use VecU32add
		stack[stackPtr] = ptrs[1]; stackPtr += (1 + resa[1]);
		stack[stackPtr] = ptrs[2]; stackPtr += (1 + resa[2]);
		stack[stackPtr] = ptrs[3]; stackPtr += (1 + resa[3]);
	}
	return Result->hitTime != FLT_MAX;
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
