
#include "TriangleMesh.h"
#include "AxisAlignedBox3d.h"
#include "OrientedBox3d.h"
#include "Sphere3d.h"
#include "Capsule3d.h"
#include "Triangle3d.h"
#include "../Maths/SIMD.h"

#include <algorithm>
#include <array>
#include <vector>

namespace Riemann
{
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
			meshBounds.Encapsulate(allBounds.back());
		}

		CreateEmptyBVH();

		std::vector<uint32_t> Permute;
		MeshBVH4::BuildFromBounds(*m_BVH, allBounds, Permute, meshBounds);

		Reorder(Permute);

#if _DEBUG
		m_BVH->Validate((Mesh*)this);
#endif
	}

	class IntersectTest
	{
	public:
		~IntersectTest() {}

	};

	template <class Shape>
	static bool IntersectTri(const Mesh* mesh, uint32_t HitNode, const Shape& shape)
	{
		LeafNode currLeaf(HitNode);
		uint32_t NumLeafTriangles = currLeaf.GetNumTriangles();
		uint32_t BaseTriIndex = currLeaf.GetTriangleIndex();

		for (uint32_t i = 0; i < NumLeafTriangles; i++)
		{
			uint32_t i0, i1, i2;
			const uint32_t index = BaseTriIndex + i;
			mesh->GetVertIndices(index, i0, i1, i2);

			const Vector3& v0 = mesh->Vertices[i0];
			const Vector3& v1 = mesh->Vertices[i1];
			const Vector3& v2 = mesh->Vertices[i2];

			bool intersect = shape.IntersectTriangle(v0, v1, v2);
			if (intersect)
			{
				return true;
			}
		}

		return false;
	}

	template <class Shape>
	static bool IntersectBVH(const Mesh* mesh, const MeshBVH4* bvh, const Shape& shape)
	{
		Box3d bv = shape.CalculateBoundingVolume();
		const Vector3& Bmin = bv.Min;
		const Vector3& Bmax = bv.Max;

		const uint32_t maxStack = 128;
		uint32_t stack1[maxStack];
		uint32_t* stack = stack1 + 1;

		assert(bvh->BatchPtr);
		assert((uintptr_t(bvh->BatchPtr) & 127) == 0);
		assert((uintptr_t(mesh) & 15) == 0);

		F128 nqMin = F128_Load_Vector3d(Bmin);
		F128 nqMax = F128_Load_Vector3d(Bmax);

		F128 nqMinx4 = F128_SplatElement<0>(nqMin);
		F128 nqMiny4 = F128_SplatElement<1>(nqMin);
		F128 nqMinz4 = F128_SplatElement<2>(nqMin);
		F128 nqMaxx4 = F128_SplatElement<0>(nqMax);
		F128 nqMaxy4 = F128_SplatElement<1>(nqMax);
		F128 nqMaxz4 = F128_SplatElement<2>(nqMax);

		uint8_t* batch_ptr = (uint8_t*)(bvh->BatchPtr);
		uint32_t* stackPtr = stack;

		assert(SIMD_WIDTH == 4 || SIMD_WIDTH == 8);
		assert(Maths::IsPowerOfTwo(bvh->BatchSize));

		for (int j = int(bvh->NumRoots - 1); j >= 0; j--)
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

			F128 minx4 = F128_LoadA(tn->minx);
			F128 miny4 = F128_LoadA(tn->miny);
			F128 minz4 = F128_LoadA(tn->minz);
			F128 maxx4 = F128_LoadA(tn->maxx);
			F128 maxy4 = F128_LoadA(tn->maxy);
			F128 maxz4 = F128_LoadA(tn->maxz);

			// AABB/AABB overlap test
			B128 res0 = F128_Greater(nqMinx4, maxx4);
			B128 res1 = F128_Greater(nqMiny4, maxy4);
			B128 res2 = F128_Greater(nqMinz4, maxz4);
			B128 res3 = F128_Greater(minx4, nqMaxx4);
			B128 res4 = F128_Greater(miny4, nqMaxy4);
			B128 res5 = F128_Greater(minz4, nqMaxz4);
			B128 resx = B128_Or(B128_Or(B128_Or(res0, res1), B128_Or(res2, res3)), B128_Or(res4, res5));
			alignas(16) uint32_t resa[SIMD_WIDTH];

			U128 res4x = U128_Load_B128(resx);
			U128_StoreA(res4x, resa);

			cacheTopValid = false;
			for (uint32_t i = 0; i < SIMD_WIDTH; i++)
			{
				uint32_t ptr = Data[i] & ~1; // clear the isLeaf bit
				if (resa[i])
					continue;
				if (tn->Data[i] & 1)
				{
					bool intersect = IntersectTri(mesh, ptr, shape);
					if (intersect)
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

	bool TriangleMesh::IntersectAABB(const Vector3& Bmin, const Vector3& Bmax) const
	{
		AxisAlignedBox3d shape(Bmin, Bmax);
		return IntersectBVH(this, m_BVH, shape);
	}

	bool TriangleMesh::IntersectOBB(const Vector3& Center, const Vector3& Extent, const Matrix3& rot) const
	{
		OrientedBox3d shape(Center, Extent, rot);
		return IntersectBVH(this, m_BVH, shape);
	}

	bool TriangleMesh::IntersectSphere(const Vector3& Center, float Radius) const
	{
		Sphere3d shape(Center, Radius);
		return IntersectBVH(this, m_BVH, shape);
	}

	bool TriangleMesh::IntersectCapsule(const Vector3& X0, const Vector3& X1, float Radius) const
	{
		Capsule3d shape(X0, X1, Radius);
		return IntersectBVH(this, m_BVH, shape);
	}

	bool	TriangleMesh::RayIntersectTri(uint32_t HitNode, const Vector3& Origin, const Vector3& Direction, const TriMeshHitOption& Option, TriMeshHitResult* Result) const
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

			const Vector3& v0 = Vertices[i0];
			const Vector3& v1 = Vertices[i1];
			const Vector3& v2 = Vertices[i2];

			float t;
			bool intersect = Triangle3d::RayIntersectTriangle(Origin, Direction, v0, v1, v2, &t);
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

	bool TriangleMesh::IntersectRay(const Vector3& Origin, const Vector3& Direction, const TriMeshHitOption& Option, TriMeshHitResult* Result) const
	{
		if (m_BVH == nullptr)
		{
			return false;
		}

		Result->hitTime = FLT_MAX;

		const U128 signMask = U128_LoadXYZW((1 << 31), (1 << 31), (1 << 31), (1 << 31));
		const F128 epsFloat4 = F128_Load(1e-9f);
		const F128 twos = F128_Load(2.0f);
		const uint32_t maxStack = 128;
		uint32_t stack1[maxStack];
		uint32_t* stack = stack1 + 1;

		assert(m_BVH->BatchPtr);
		assert((uintptr_t(m_BVH->BatchPtr) & 127) == 0);
		// assert((uintptr_t(this) & 15) == 0);

		float maxT = Option.maxDist;

		uint8_t* batch_ptr = (uint8_t*)(m_BVH->BatchPtr);
		F128 maxT4;
		maxT4 = F128_Load(maxT);
		F128 rayP = F128_Load_Vector3d(Origin);
		F128 rayD = F128_Load_Vector3d(Direction);
		U128 raySign = U128_AND(U128_ReinterpretFrom_F128(rayD), signMask);
		F128 rayDAbs = F128_Abs(rayD); // abs value of rayD
		F128 rayInvD = F128_ReinterpretFrom_U128(U128_OR(raySign, U128_ReinterpretFrom_F128(F128_Max(rayDAbs, epsFloat4)))); // clamp near-zero components up to epsilon
		rayD = rayInvD;

		//rayInvD = V4Recip(rayInvD);
		// Newton-Raphson iteration for reciprocal (see wikipedia):
		// X[n+1] = X[n]*(2-original*X[n]), X[0] = V4RecipFast estimate
		//rayInvD = rayInvD*(twos-rayD*rayInvD);
		rayInvD = F128_RecipFast(rayInvD); // initial estimate, not accurate enough
		rayInvD = F128_Mul(rayInvD, F128_NegMSub(rayD, rayInvD, twos));

		// P+tD=a; t=(a-P)/D
		// t=(a - p.x)*1/d.x = a/d.x +(- p.x/d.x)
		F128 rayPinvD = F128_NegMSub(rayInvD, rayP, F128_Zero());
		F128 rayInvDsplatX = F128_SplatElement<0>(rayInvD);
		F128 rayInvDsplatY = F128_SplatElement<1>(rayInvD);
		F128 rayInvDsplatZ = F128_SplatElement<2>(rayInvD);
		F128 rayPinvDsplatX = F128_SplatElement<0>(rayPinvD);
		F128 rayPinvDsplatY = F128_SplatElement<1>(rayPinvD);
		F128 rayPinvDsplatZ = F128_SplatElement<2>(rayPinvD);

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
				bool intersect = RayIntersectTri(top, Origin, Direction, Option, Result);
				if (intersect)
				{
					if (!Option.hitNearest)
					{
						return true;
					}

					if (Result->hitTime < maxT)
					{
						maxT = Result->hitTime;
						maxT4 = F128_Load(maxT);
					}
				}

				continue;
			}

			Result->AddTestCount(2);

			BVHNodeBatch* tn = reinterpret_cast<BVHNodeBatch*>(batch_ptr + top);

			// 6i load
			F128 minx4a = F128_LoadA(tn->minx), miny4a = F128_LoadA(tn->miny), minz4a = F128_LoadA(tn->minz);
			F128 maxx4a = F128_LoadA(tn->maxx), maxy4a = F128_LoadA(tn->maxy), maxz4a = F128_LoadA(tn->maxz);

			// 1i disabled test
			// AP scaffold - optimization opportunity - can save 2 instructions here
			U128 ignore4a = U128_Load_B128(F128_Greater(minx4a, maxx4a)); // 1 if degenerate box (empty slot in the page)

			// P+tD=a; t=(a-P)/D
			// t=(a - p.x)*1/d.x = a/d.x +(- p.x/d.x)
			// 6i
			F128 tminxa0 = F128_MAdd(minx4a, rayInvDsplatX, rayPinvDsplatX);
			F128 tminya0 = F128_MAdd(miny4a, rayInvDsplatY, rayPinvDsplatY);
			F128 tminza0 = F128_MAdd(minz4a, rayInvDsplatZ, rayPinvDsplatZ);
			F128 tmaxxa0 = F128_MAdd(maxx4a, rayInvDsplatX, rayPinvDsplatX);
			F128 tmaxya0 = F128_MAdd(maxy4a, rayInvDsplatY, rayPinvDsplatY);
			F128 tmaxza0 = F128_MAdd(maxz4a, rayInvDsplatZ, rayPinvDsplatZ);

			// test half-spaces
			// P+tD=dN
			// t = (d(N,D)-(P,D))/(D,D) , (D,D)=1

			// compute 4x dot products (N,D) and (P,N) for each AABB in the page

			// 6i
			// now compute tnear and tfar for each pair of planes for each box
			F128 tminxa = F128_Min(tminxa0, tmaxxa0);
			F128 tmaxxa = F128_Max(tminxa0, tmaxxa0);
			F128 tminya = F128_Min(tminya0, tmaxya0);
			F128 tmaxya = F128_Max(tminya0, tmaxya0);
			F128 tminza = F128_Min(tminza0, tmaxza0);
			F128 tmaxza = F128_Max(tminza0, tmaxza0);

			// 8i
			F128 maxOfNeasa = F128_Max(F128_Max(tminxa, tminya), tminza);
			F128 minOfFarsa = F128_Min(F128_Min(tmaxxa, tmaxya), tmaxza);
			ignore4a = U128_OR(ignore4a, U128_Load_B128(F128_Greater(epsFloat4, minOfFarsa)));  // if tfar is negative, ignore since its a ray, not a line
			// AP scaffold: update the build to eliminate 3 more instructions for ignore4a above
			//VecU32V ignore4a = F128_Greater(epsFloat4, minOfFarsa);  // if tfar is negative, ignore since its a ray, not a line
			ignore4a = U128_OR(ignore4a, U128_Load_B128(F128_Greater(maxOfNeasa, maxT4)));  // if tnear is over maxT, ignore this result

			// 2i
			U128 resa4 = U128_Load_B128(F128_Greater(maxOfNeasa, minOfFarsa)); // if 1 => fail
			resa4 = U128_OR(resa4, ignore4a);

			// 1i
			U128_StoreAA(resa4, reinterpret_cast<U128*>(resa));

			uint32_t* ptrs = (reinterpret_cast<BVHNodeBatch*>(tn))->Data;

			stack[stackPtr] = ptrs[0]; stackPtr += (1 + resa[0]); // AP scaffold TODO: use VecU32add
			stack[stackPtr] = ptrs[1]; stackPtr += (1 + resa[1]);
			stack[stackPtr] = ptrs[2]; stackPtr += (1 + resa[2]);
			stack[stackPtr] = ptrs[3]; stackPtr += (1 + resa[3]);
		}

		return Result->hitTime != FLT_MAX;
	}

	bool	TriangleMesh::IntersectRay(const Vector3& Origin, const Vector3& Direction, float* t) const
	{
		TriMeshHitOption Option;
		Option.hitNearest = true;
		Option.maxDist = FLT_MAX;

		TriMeshHitResult Result = { 0 };
		if (IntersectRay(Origin, Direction, Option, &Result))
		{
			*t = Result.hitTime;
			return true;
		}
		return false;
	}

	Vector3 TriangleMesh::GetSupport(const Vector3& Direction) const
	{
		assert(false);
		return Vector3::Zero();
	}

	int TriangleMesh::GetSupportFace(const Vector3& Direction, Vector3* FacePoints) const
	{
		assert(false);
		return 0;
	}
}