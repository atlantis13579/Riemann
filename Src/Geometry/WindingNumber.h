#pragma once

#include <assert.h>
#include <vector>
#include <set>
#include "../CollisionPrimitive/Triangle3.h"
#include "../Maths/Maths.h"
#include "DynamicMesh.h"

namespace Riemann
{
	struct FMeshTriInfoCache
	{
		std::vector<Vector3> Centroids;
		std::vector<Vector3> Normals;
		std::vector<float> Areas;

		void GetCache(int TriangleID, Vector3& NormalOut, float& AreaOut, Vector3& CentroidOut) const
		{
			NormalOut = Normals[TriangleID];
			AreaOut = Areas[TriangleID];
			CentroidOut = Centroids[TriangleID];
		}

		void Build(const DynamicMesh& Mesh)
		{
			int nt = Mesh.GetTriangleCount();
			Centroids.resize(nt);
			Normals.resize(nt);
			Areas.resize(nt);

			for (int i = 0; i < nt; ++i)
			{
				Triangle3 Triangle;
				Mesh.GetTriVertices(i, Triangle.v0, Triangle.v1, Triangle.v2);

				Centroids[i] = Triangle.GetCenter();
				Normals[i] = Triangle.GetNormal();
				Areas[i] = Triangle.GetArea();
			}

			return;
		}
	};

	inline void ComputeCoeffs(const DynamicMesh& Mesh,
		const std::set<int>& Triangles,
		const FMeshTriInfoCache& TriCache,
		Vector3& P,
		float& R,
		Vector3& Order1,
		Matrix3& Order2)
	{
		P = Vector3::Zero();
		Order1 = Vector3::Zero();
		Order2 = Matrix3::Zero();
		R = 0;

		Vector3 P0 = Vector3::Zero(), P1 = Vector3::Zero(), P2 = Vector3::Zero();
		float sum_area = 0;
		for (int tid : Triangles)
		{
			float Area = TriCache.Areas[tid];
			sum_area += Area;
			P += Area * TriCache.Centroids[tid];
		}
		P /= sum_area;

		Vector3 n, c;
		float a = 0;
		float RSq = 0;
		for (int tid : Triangles)
		{
			Mesh.GetTriVertices(tid, P0, P1, P2);
			TriCache.GetCache(tid, n, a, c);

			Order1 += a * n;

			Vector3 dcp = c - P;
			Order2 += a * Matrix3::OuterProduct(dcp, n);

			// update max radius R (as squared value in loop)
			float MaxDistSq = Maths::Max((P0 - P).SquareLength(), (P1 - P).SquareLength(), (P2 - P).SquareLength());
			RSq = Maths::Max(RSq, MaxDistSq);
		}
		R = sqrtf(RSq);
	}

	inline float EvaluateOrder1Approx(const Vector3& Center, const Vector3& Order1Coeff, const Vector3& Q)
	{
		Vector3 dpq = (Center - Q);
		float len = dpq.Length();

		return (1.0f / PI_4) * Order1Coeff.Dot(dpq / (len * len * len));
	}

	inline float EvaluateOrder2Approx(const Vector3& Center, const Vector3& Order1Coeff, const Matrix3& Order2Coeff, const Vector3& Q)
	{
		Vector3 dpq = (Center - Q);
		float len = dpq.Length();
		float len3 = len * len * len;
		float fourPi_len3 = 1.0f / (PI_4 * len3);

		float Order1 = fourPi_len3 * Order1Coeff.Dot(dpq);

		// second-order hessian \grad^2(G)
		float c = -3.0f / (PI_4 * len3 * len * len);

		// expanded-out version below avoids extra constructors
		//Matrix3 xqxq(dpq, dpq);
		//Matrix3 hessian(fourPi_len3, fourPi_len3, fourPi_len3) - c * xqxq;
		Matrix3 hessian(
			fourPi_len3 + c * dpq.x * dpq.x, c * dpq.x * dpq.y, c * dpq.x * dpq.z,
			c * dpq.y * dpq.x, fourPi_len3 + c * dpq.y * dpq.y, c * dpq.y * dpq.z,
			c * dpq.z * dpq.x, c * dpq.z * dpq.y, fourPi_len3 + c * dpq.z * dpq.z);

		float Order2 = Order2Coeff.InnerProduct(hessian, true);

		return Order1 + Order2;
	}

	template <class T>
	class Optional
	{
	public:
		T& GetValue()
		{
			return data;
		}

		bool IsSet() const
		{
			return is_set;
		}

		void Set()
		{
			is_set = true;
		}

		void Clear()
		{
			is_set = false;
		}

	private:
		bool	is_set = false;
		T		data;
	};

	class TFastWindingTree
	{
		DynamicMeshAABBTree* Tree;

		struct FWNInfo
		{
			Vector3 Center;
			float R;
			Vector3 Order1Vec;
			Matrix3 Order2Mat;
		};

		std::map<int, FWNInfo> FastWindingCache;

	public:
		float FWNBeta = 2.0f;
		int FWNApproxOrder = 2;
		uint64_t MeshChangeStamp = 0;

		TFastWindingTree(DynamicMeshAABBTree* TreeToRef)
		{
			this->Tree = TreeToRef;
			Build(true);
		}

		void Build(bool force_build)
		{
			if (!Tree->IsValid())
			{
				Tree->Build();
			}
			if (force_build || MeshChangeStamp != Tree->Mesh->GetChangeStamps())
			{
				MeshChangeStamp = Tree->Mesh->GetChangeStamps();
				build_fast_winding_cache();
			}
		}

		DynamicMeshAABBTree* GetTree() const
		{
			return Tree;
		}

		float FastWindingNumber(const Vector3& P)
		{
			Build(false);
			float sum = branch_fast_winding_num(Tree->RootIndex, P);
			return sum;
		}

		float FastWindingNumber(const Vector3& P) const
		{
			assert(Tree->IsValid());
			float sum = branch_fast_winding_num(Tree->RootIndex, P);
			return sum;
		}

		bool IsInside(const Vector3& P, float WindingIsoThreshold = 0.5) const
		{
			return FastWindingNumber(P) > WindingIsoThreshold;
		}

	private:
		static float TriSolidAngle(Vector3 A, Vector3 B, Vector3 C, const Vector3& P)
		{
			// Formula from https://igl.ethz.ch/projects/winding-number/
			A -= P;
			B -= P;
			C -= P;
			float la = A.Length(), lb = B.Length(), lc = C.Length();
			float top = (la * lb * lc) + A.Dot(B) * lc + B.Dot(C) * la + C.Dot(A) * lb;
			float bottom = A.x * (B.y * C.z - C.y * B.z) - A.y * (B.x * C.z - C.x * B.z) + A.z * (B.x * C.y - C.x * B.y);
			// -2 instead of 2 to account for UE winding
			return -2.0f * atan2f(bottom, top);
		}

		float branch_fast_winding_num(int IBox, Vector3 P) const
		{
			float branch_sum = 0;

			int idx = Tree->BoxToIndex[IBox];
			if (idx < Tree->TrianglesEnd)
			{
				int num_tris = Tree->IndexList[idx];
				for (int i = 1; i <= num_tris; ++i)
				{
					Vector3 a, b, c;
					int ti = Tree->IndexList[idx + i];
					Tree->Mesh->GetTriVertices(ti, a, b, c);
					float angle = TriSolidAngle(a, b, c, P);
					branch_sum += angle / PI_4;
				}
			}
			else
			{
				int iChild1 = Tree->IndexList[idx];
				if (iChild1 < 0)
				{
					iChild1 = (-iChild1) - 1;

					bool contained = Tree->GetBoxEps(iChild1).IsInside(P);
					if (contained == false && can_use_fast_winding_cache(iChild1, P))
					{
						branch_sum += evaluate_box_fast_winding_cache(iChild1, P);
					}
					else
					{
						branch_sum += branch_fast_winding_num(iChild1, P);
					}
				}
				else
				{
					iChild1 = iChild1 - 1;
					int iChild2 = Tree->IndexList[idx + 1] - 1;

					bool contained1 = Tree->GetBoxEps(iChild1).IsInside(P);
					if (contained1 == false && can_use_fast_winding_cache(iChild1, P))
					{
						branch_sum += evaluate_box_fast_winding_cache(iChild1, P);
					}
					else
					{
						branch_sum += branch_fast_winding_num(iChild1, P);
					}

					bool contained2 = Tree->GetBoxEps(iChild2).IsInside(P);
					if (contained2 == false && can_use_fast_winding_cache(iChild2, P))
					{
						branch_sum += evaluate_box_fast_winding_cache(iChild2, P);
					}
					else
					{
						branch_sum += branch_fast_winding_num(iChild2, P);
					}
				}
			}

			return branch_sum;
		}

		void build_fast_winding_cache()
		{
			int WINDING_CACHE_THRESH = 1;

			FMeshTriInfoCache TriCache;
			TriCache.Build(*Tree->Mesh);

			FastWindingCache.clear();
			Optional<std::set<int>> root_hash;
			build_fast_winding_cache(Tree->RootIndex, 0, WINDING_CACHE_THRESH, root_hash, TriCache);
		}

		int build_fast_winding_cache(int IBox, int Depth, int TriCountThresh, Optional<std::set<int>>& TriHash, const FMeshTriInfoCache& TriCache)
		{
			TriHash.Clear();

			int idx = Tree->BoxToIndex[IBox];
			if (idx < Tree->TrianglesEnd)
			{
				int num_tris = Tree->IndexList[idx];
				return num_tris;
			}
			else
			{
				int iChild1 = Tree->IndexList[idx];
				if (iChild1 < 0)
				{
					iChild1 = (-iChild1) - 1;
					int num_child_tris = build_fast_winding_cache(iChild1, Depth + 1, TriCountThresh, TriHash, TriCache);
					return num_child_tris;
				}
				else
				{
					iChild1 = iChild1 - 1;
					int iChild2 = Tree->IndexList[idx + 1] - 1;

					Optional<std::set<int>> child2_hash;
					int num_tris_1 = build_fast_winding_cache(iChild1, Depth + 1, TriCountThresh, TriHash, TriCache);
					int num_tris_2 = build_fast_winding_cache(iChild2, Depth + 1, TriCountThresh, child2_hash, TriCache);
					bool build_cache = (num_tris_1 + num_tris_2 > TriCountThresh);

					if (Depth == 0)
					{
						return num_tris_1 + num_tris_2;
					}

					if (TriHash.IsSet() || child2_hash.IsSet() || build_cache)
					{
						if (!TriHash.IsSet() && child2_hash.IsSet())
						{
							collect_triangles(iChild1, child2_hash.GetValue());
							TriHash = child2_hash;
						}
						else
						{
							if (!TriHash.IsSet())
							{
								TriHash.Set();
								collect_triangles(iChild1, TriHash.GetValue());
							}
							if (!child2_hash.IsSet())
							{
								collect_triangles(iChild2, TriHash.GetValue());
							}
							else
							{
								TriHash.GetValue().insert(child2_hash.GetValue().begin(), child2_hash.GetValue().end());
							}
						}
					}
					if (build_cache)
					{
						assert(TriHash.IsSet());
						make_box_fast_winding_cache(IBox, TriHash.GetValue(), TriCache);
					}

					return (num_tris_1 + num_tris_2);
				}
			}
		}

		bool can_use_fast_winding_cache(int IBox, const Vector3& Q) const
		{
			auto it = FastWindingCache.find(IBox);
			if (it == FastWindingCache.end())
			{
				return false;
			}

			const FWNInfo& cacheInfo = it->second;

			float dist_qp = (cacheInfo.Center - Q).Length();
			if (dist_qp > FWNBeta * cacheInfo.R)
			{
				return true;
			}

			return false;
		}

		void make_box_fast_winding_cache(int IBox, const std::set<int>& Triangles, const FMeshTriInfoCache& TriCache)
		{
			assert(FastWindingCache.find(IBox) == FastWindingCache.end());

			FWNInfo cacheInfo;
			ComputeCoeffs(*Tree->Mesh, Triangles, TriCache, cacheInfo.Center, cacheInfo.R, cacheInfo.Order1Vec, cacheInfo.Order2Mat);

			FastWindingCache[IBox] = cacheInfo;
		}

		float evaluate_box_fast_winding_cache(int IBox, const Vector3& Q) const
		{
			auto it = FastWindingCache.find(IBox);
			assert(it != FastWindingCache.end());

			const FWNInfo& cacheInfo = it->second;

			if (FWNApproxOrder == 2)
			{
				return EvaluateOrder2Approx(cacheInfo.Center, cacheInfo.Order1Vec, cacheInfo.Order2Mat, Q);
			}
			else
			{
				return EvaluateOrder1Approx(cacheInfo.Center, cacheInfo.Order1Vec, Q);
			}
		}

		void collect_triangles(int IBox, std::set<int>& Triangles)
		{
			int idx = Tree->BoxToIndex[IBox];
			if (idx < Tree->TrianglesEnd)
			{
				int num_tris = Tree->IndexList[idx];
				for (int i = 1; i <= num_tris; ++i)
				{
					Triangles.insert(Tree->IndexList[idx + i]);
				}
			}
			else
			{
				int iChild1 = Tree->IndexList[idx];
				if (iChild1 < 0)
				{
					collect_triangles((-iChild1) - 1, Triangles);
				}
				else
				{
					collect_triangles(iChild1 - 1, Triangles);
					collect_triangles(Tree->IndexList[idx + 1] - 1, Triangles);
				}
			}
		}
	};
}
