
#include "GeometryAttributes.h"
#include "DynamicMesh.h"

namespace Riemann
{
	std::string ColorAttribName = "ColorAttrib";
	std::string TangentUAttribName = "TangentUAttrib";
	std::string TangentVAttribName = "TangentVAttrib";
	std::string VisibleAttribName = "VisibleAttrib";
	std::string InternalAttribName = "InternalAttrib";

	#define MAX_NUM_UV_CHANNELS			(8)
	std::string UVChannelNames[MAX_NUM_UV_CHANNELS] = {
		"UVAttrib0",
		"UVAttrib1",
		"UVAttrib2",
		"UVAttrib3",
		"UVAttrib4",
		"UVAttrib5",
		"UVAttrib6",
		"UVAttrib7"
	};

	template<typename AttribValueType, int AttribDimension>
	void TDynamicMeshTriangleAttribute<AttribValueType, AttribDimension>::SetNewValue(int NewTriangleID, const AttribValueType* Data)
	{
		AttribValueType default_val = GetDefaultAttributeValue();
		int k = NewTriangleID * AttribDimension;
		for (int i = 0; i < AttribDimension; ++i)
		{
			VectorSetSafe(AttribValues, k + i, Data[i], default_val);
		}
	}

	template<typename RealType, int ElementSize>
	void TDynamicMeshOverlay<RealType, ElementSize>::OnReverseTriOrientation(int TriangleID)
	{
		Index3 Triangle = GetTriangle(TriangleID);
		int i = 3 * TriangleID;
		ElementTriangles[i] = Triangle[1];			// mirrors order in FDynamicMesh3::ReverseTriOrientationInternal
		ElementTriangles[i + 1] = Triangle[0];
		ElementTriangles[i + 2] = Triangle[2];
	}

	template<typename AttribValueType, int AttribDimension>
	void TDynamicMeshTriangleAttribute<AttribValueType, AttribDimension>::SetValue(int TriangleID, const AttribValueType* Data)
	{
		AttribValueType default_val = GetDefaultAttributeValue();
		int k = TriangleID * AttribDimension;
		for (int i = 0; i < AttribDimension; ++i)
		{
			VectorSetSafe(AttribValues, k + i, Data[i], default_val);
		}
	}

	template<typename AttribValueType, int AttribDimension>
	void TDynamicMeshTriangleAttribute<AttribValueType, AttribDimension>::CopyValue(int FromTriangleID, int ToTriangleID)
	{
		AttribValueType default_val = GetDefaultAttributeValue();
		int kA = FromTriangleID * AttribDimension;
		int kB = ToTriangleID * AttribDimension;
		for (int i = 0; i < AttribDimension; ++i)
		{
			AttribValueType Value = AttribValues[kA + i];
			VectorSetSafe(AttribValues, kB + i, Value, default_val);
		}
	}

	template<typename AttribValueType, int AttribDimension>
	bool TDynamicMeshTriangleAttribute<AttribValueType, AttribDimension>::IsBorderEdge(int EdgeID, bool bMeshBoundaryIsBorder /*= true*/) const
	{
		Index2 EdgeTris = ParentMesh->GetEdgeT(EdgeID);
		if (EdgeTris.b == -1)
		{
			return bMeshBoundaryIsBorder;
		}
		int kA = EdgeTris.a * AttribDimension;
		int kB = EdgeTris.b * AttribDimension;
		for (int i = 0; i < AttribDimension; ++i)
		{
			if (AttribValues[kA + i] != AttribValues[kB + i])
			{
				return true;
			}
		}
		assert(false);
		return false;
	}

	template<typename RealType, int ElementSize>
	void TDynamicMeshOverlay<RealType, ElementSize>::ClearElements()
	{
		Elements.clear();
		ElementsRefCounts.Clear();
		ParentVertices.clear();
		InitializeTriangles(ParentMesh->GetTriangleCount());
	}

	template<typename RealType, int ElementSize>
	void TDynamicMeshOverlay<RealType, ElementSize>::OnRemoveTriangle(int TriangleID)
	{
		Index3 Triangle = GetTriangle(TriangleID);
		if (Triangle.a < 0 && Triangle.b < 0 && Triangle.c < 0)
		{
			// if whole triangle has no overlay vertices set, that's OK, just remove nothing
			// (if only *some* of the triangle vertices were < 0, that would be a bug / invalid overlay triangle)
			return;
		}
		InitializeNewTriangle(TriangleID);

		// decrement element refcounts, and free element if it is now unreferenced
		for (int j = 0; j < 3; ++j)
		{
			int elemid = Triangle[j];
			ElementsRefCounts.Decrement(elemid);
			if (ElementsRefCounts.GetRefCount(elemid) == 1)
			{
				ElementsRefCounts.Decrement(elemid);
				ParentVertices[elemid] = -1;
				assert(ElementsRefCounts.IsValid(elemid) == false);
			}
		}
	}

	template<typename RealType, int ElementSize>
	void TDynamicMeshOverlay<RealType, ElementSize>::InitializeNewTriangle(int tid)
	{
		int i = 3 * tid;
		ElementTriangles.resize(i + 3, -1);
		ElementTriangles[i + 2] = -1;
		ElementTriangles[i + 1] = -1;
		ElementTriangles[i] = -1;

		//updateTimeStamp(true);
	}

	template<typename RealType, int ElementSize>
	bool TDynamicMeshOverlay<RealType, ElementSize>::IsSeamEdge(int eid, bool* bIsNonIntersecting) const
	{
		if (bIsNonIntersecting != nullptr)
		{
			*bIsNonIntersecting = false;
		}
		if (ParentMesh->IsEdge(eid) == false)
		{
			return false;
		}

		Index2 et = ParentMesh->GetEdgeT(eid);
		if (et.b == -1)
		{
			if (bIsNonIntersecting != nullptr)
			{
				Index2 ev = ParentMesh->GetEdgeV(eid);
				int CountA = CountVertexElements(ev.a);
				int CountB = CountVertexElements(ev.b);

				// will be false if another seam intersects is adjacent to either end of the seam edge
				*bIsNonIntersecting = (CountA == 1) && (CountB == 1);
			}

			return true;
		}

		Index2 ev = ParentMesh->GetEdgeV(eid);
		int base_a = ev.a, base_b = ev.b;

		bool bASet = IsSetTriangle(et.a), bBSet = IsSetTriangle(et.b);
		if (!bASet || !bBSet) // if either triangle is unset, need different logic for checking if this is a seam
		{
			return (bASet || bBSet); // consider it a seam if only one is unset
		}

		Index3 Triangle0 = GetTriangle(et.a);
		Index3 BaseTriangle0(ParentVertices[Triangle0.a], ParentVertices[Triangle0.b], ParentVertices[Triangle0.c]);
		int idx_base_a0 = BaseTriangle0.IndexOf(base_a);
		int idx_base_b0 = BaseTriangle0.IndexOf(base_b);

		Index3 Triangle1 = GetTriangle(et.b);
		Index3 BaseTriangle1(ParentVertices[Triangle1.a], ParentVertices[Triangle1.b], ParentVertices[Triangle1.c]);
		int idx_base_a1 = BaseTriangle1.IndexOf(base_a);
		int idx_base_b1 = BaseTriangle1.IndexOf(base_b);

		int el_a_tri0 = Triangle0[idx_base_a0];
		int el_b_tri0 = Triangle0[idx_base_b0];
		int el_a_tri1 = Triangle1[idx_base_a1];
		int el_b_tri1 = Triangle1[idx_base_b1];

		bool bIsSeam = !SamePairUnordered(el_a_tri0, el_b_tri0, el_a_tri1, el_b_tri1);

		if (bIsNonIntersecting != nullptr)
		{
			if ((el_a_tri0 == el_a_tri1 || el_a_tri0 == el_b_tri1) || (el_b_tri0 == el_b_tri1 || el_b_tri0 == el_a_tri1))
			{
				// seam edge "intersects" with end of the seam
				*bIsNonIntersecting = false;
			}
			else
			{
				// check that exactly two elements are associated with the vertices at each end of the edge
				int CountA = CountVertexElements(base_a);
				int CountB = CountVertexElements(base_b);

				// will be false if another seam intersects is adjacent to either end of the seam edge
				*bIsNonIntersecting = (CountA == 2) && (CountB == 2);
			}
		}

		return bIsSeam;

		// TODO: this doesn't seem to work but it should, and would be more efficient:
		//   - add ParentMesh->FindTriEdgeIndex(tid,eid)
		//   - SamePairUnordered query could directly index into ElementTriangles[]
		//Index3 TriangleA = GetTriangle(et.a);
		//Index3 TriangleB = GetTriangle(et.b);

		//Index3 BaseTriEdgesA = ParentMesh->GetTriEdges(et.a);
		//int WhichA = (BaseTriEdgesA.a == eid) ? 0 :
		//	((BaseTriEdgesA.b == eid) ? 1 : 2);

		//Index3 BaseTriEdgesB = ParentMesh->GetTriEdges(et.b);
		//int WhichB = (BaseTriEdgesB.a == eid) ? 0 :
		//	((BaseTriEdgesB.b == eid) ? 1 : 2);

		//return SamePairUnordered(
		//	TriangleA[WhichA], TriangleA[(WhichA + 1) % 3],
		//	TriangleB[WhichB], TriangleB[(WhichB + 1) % 3]);
	}


	template<typename RealType, int ElementSize>
	bool TDynamicMeshOverlay<RealType, ElementSize>::IsSeamEndEdge(int eid) const
	{
		if (ParentMesh->IsEdge(eid) == false)
		{
			return false;
		}

		Index2 et = ParentMesh->GetEdgeT(eid);
		if (et.b == -1)
		{
			return false;
		}

		Index2 ev = ParentMesh->GetEdgeV(eid);
		int base_a = ev.a, base_b = ev.b;

		bool bASet = IsSetTriangle(et.a), bBSet = IsSetTriangle(et.b);
		if (!bASet || !bBSet)
		{
			return false;
		}

		Index3 Triangle0 = GetTriangle(et.a);
		Index3 BaseTriangle0(ParentVertices[Triangle0.a], ParentVertices[Triangle0.b], ParentVertices[Triangle0.c]);
		int idx_base_a0 = BaseTriangle0.IndexOf(base_a);
		int idx_base_b0 = BaseTriangle0.IndexOf(base_b);

		Index3 Triangle1 = GetTriangle(et.b);
		Index3 BaseTriangle1(ParentVertices[Triangle1.a], ParentVertices[Triangle1.b], ParentVertices[Triangle1.c]);
		int idx_base_a1 = BaseTriangle1.IndexOf(base_a);
		int idx_base_b1 = BaseTriangle1.IndexOf(base_b);

		int el_a_tri0 = Triangle0[idx_base_a0];
		int el_b_tri0 = Triangle0[idx_base_b0];
		int el_a_tri1 = Triangle1[idx_base_a1];
		int el_b_tri1 = Triangle1[idx_base_b1];

		bool bIsSeam = !SamePairUnordered(el_a_tri0, el_b_tri0, el_a_tri1, el_b_tri1);

		bool bIsSeamEnd = false;
		if (bIsSeam)
		{
			// is only one of elements split?
			if ((el_a_tri0 == el_a_tri1 || el_a_tri0 == el_b_tri1) || (el_b_tri0 == el_b_tri1 || el_b_tri0 == el_a_tri1))
			{
				bIsSeamEnd = true;
			}
		}

		return bIsSeamEnd;
	}

	template<typename RealType, int ElementSize>
	bool TDynamicMeshOverlay<RealType, ElementSize>::HasInteriorSeamEdges() const
	{
		for (int eid = 0; eid < ParentMesh->GetEdgeCount(); ++eid)
		{
			if (!ParentMesh->IsEdgeFast(eid))
			{
				continue;
			}

			Index2 et = ParentMesh->GetEdgeT(eid);
			if (et.b != -1)
			{
				bool bASet = IsSetTriangle(et.a), bBSet = IsSetTriangle(et.b);
				if (bASet != bBSet)
				{
					// seam between triangles with elements and triangles without
					return true;
				}
				else if (!bASet)
				{
					// neither triangle has set elements
					continue;
				}
				Index2 ev = ParentMesh->GetEdgeV(eid);
				int base_a = ev.a, base_b = ev.b;

				Index3 Triangle0 = GetTriangle(et.a);
				Index3 BaseTriangle0(ParentVertices[Triangle0.a], ParentVertices[Triangle0.b], ParentVertices[Triangle0.c]);
				int idx_base_a1 = BaseTriangle0.IndexOf(base_a);
				int idx_base_b1 = BaseTriangle0.IndexOf(base_b);

				Index3 Triangle1 = GetTriangle(et.b);
				Index3 BaseTriangle1(ParentVertices[Triangle1.a], ParentVertices[Triangle1.b], ParentVertices[Triangle1.c]);
				int idx_base_a2 = BaseTriangle1.IndexOf(base_a);
				int idx_base_b2 = BaseTriangle1.IndexOf(base_b);

				if (!SamePairUnordered(Triangle0[idx_base_a1], Triangle0[idx_base_b1], Triangle1[idx_base_a2], Triangle1[idx_base_b2]))
				{
					return true;
				}
			}
		}
		return false;
	}


	template<typename RealType, int ElementSize>
	int TDynamicMeshOverlay<RealType, ElementSize>::SplitElementWithNewParent(int ElementID, int NewParentID, const std::vector<int>& TrianglesToUpdate)
	{
		RealType SourceData[ElementSize];
		GetElement(ElementID, SourceData);
		int NewElID = AppendElement(SourceData);
		for (int TriID : TrianglesToUpdate)
		{
			int ElementTriStart = TriID * 3;
			for (int SubIdx = 0; SubIdx < 3; SubIdx++)
			{
				int CurElID = ElementTriangles[ElementTriStart + SubIdx];
				if (CurElID == ElementID)
				{
					ElementsRefCounts.Decrement(ElementID);
					ElementsRefCounts.Increment(NewElID);
					ElementTriangles[ElementTriStart + SubIdx] = NewElID;
				}
			}
		}
		VectorSetSafe(ParentVertices, NewElID, NewParentID, -1);

		assert(ElementsRefCounts.IsValid(ElementID));

		// An element may have become isolated after changing all of its incident triangles. Delete such an element.
		if (ElementsRefCounts.GetRefCount(ElementID) == 1)
		{
			ElementsRefCounts.Decrement(ElementID);
			ParentVertices[ElementID] = -1;
		}

		return NewElID;
	}


	template<typename RealType, int ElementSize>
	bool TDynamicMeshOverlay<RealType, ElementSize>::IsSeamVertex(int vid, bool bBoundaryIsSeam) const
	{
		// @todo can we do this more efficiently? At minimum we are looking up each triangle twice...
		std::vector<int> VtxEdgesItr = ParentMesh->GetVexEdges(vid);
		for (int edgeid : VtxEdgesItr)
		{
			if (!bBoundaryIsSeam && ParentMesh->IsBoundaryEdge(edgeid))
			{
				continue;
			}
			if (IsSeamEdge(edgeid))
			{
				return true;
			}
		}
		return false;
	}


	template<typename RealType, int ElementSize>
	int TDynamicMeshOverlay<RealType, ElementSize>::AppendElement(RealType ConstantValue)
	{
		int vid = ElementsRefCounts.Allocate();
		int i = ElementSize * vid;
		for (int k = ElementSize - 1; k >= 0; --k)
		{
			VectorSetSafe(Elements, i + k, ConstantValue, (RealType)0);
		}
		VectorSetSafe(ParentVertices, vid, -1, -1);

		//updateTimeStamp(true);
		return vid;
	}


	template<typename RealType, int ElementSize>
	int TDynamicMeshOverlay<RealType, ElementSize>::AppendElement(const RealType* Value)
	{
		int vid = ElementsRefCounts.Allocate();
		int i = ElementSize * vid;
		for (int k = ElementSize - 1; k >= 0; --k)
		{
			VectorSetSafe(Elements, i + k, Value[k], (RealType)0);
		}

		VectorSetSafe(ParentVertices, vid, -1, -1);

		//updateTimeStamp(true);
		return vid;
	}


	template<typename RealType, int ElementSize>
	void TDynamicMeshOverlay<RealType, ElementSize>::SetElementFromLerp(int SetElement, int ElementA, int ElementB, double Alpha)
	{
		int IndexSet = ElementSize * SetElement;
		int IndexA = ElementSize * ElementA;
		int IndexB = ElementSize * ElementB;
		double Beta = ((double)1 - Alpha);
		for (int i = 0; i < ElementSize; ++i)
		{
			double LerpValue = Beta * (double)Elements[IndexA + i] + Alpha * (double)Elements[IndexB + i];
			Elements[IndexSet + i] = (RealType)LerpValue;
		}
	}

	template<typename RealType, int ElementSize>
	void TDynamicMeshOverlay<RealType, ElementSize>::SetElementFromBary(int SetElement, int ElementA, int ElementB, int ElementC, const Vector3& BaryCoords)
	{
		int IndexSet = ElementSize * SetElement;
		int IndexA = ElementSize * ElementA;
		int IndexB = ElementSize * ElementB;
		int IndexC = ElementSize * ElementC;
		for (int i = 0; i < ElementSize; ++i)
		{
			float BaryValue = BaryCoords.x * Elements[IndexA + i] + BaryCoords.y * Elements[IndexB + i] + BaryCoords.z * Elements[IndexC + i];
			Elements[IndexSet + i] = (RealType)BaryValue;
		}
	}

	template<typename RealType, int ElementSize>
	int TDynamicMeshOverlay<RealType, ElementSize>::CountVertexElements(int vid, bool bBruteForce /*= false*/) const
	{
		std::set<int> VertexElements;
		Index3 Triangle;
		if (bBruteForce)
		{
			for (int tid = 0; tid <= ParentMesh->GetTriangleCount(); ++tid)
			{
				if (!ParentMesh->IsTriangleFast(tid))
				{
					continue;
				}

				if (GetTriangleIfValid(tid, Triangle))
				{
					for (int j = 0; j < 3; ++j)
					{
						if (ParentVertices[Triangle[j]] == vid)
						{
							VertexElements.insert(Triangle[j]);
						}
					}
				}
			}
		}
		else
		{
			for (int tid = 0; tid <= ParentMesh->GetTriangleCount(); ++tid)
			{
				if (!ParentMesh->IsTriangleFast(tid))
				{
					continue;
				}
				if (GetTriangleIfValid(tid, Triangle))
				{
					for (int j = 0; j < 3; ++j)
					{
						if (ParentVertices[Triangle[j]] == vid)
						{
							VertexElements.insert(Triangle[j]);
						}
					}
				}
			}
		}

		return (int)VertexElements.size();
	}

	template<typename RealType, int ElementSize>
	EMeshResult TDynamicMeshOverlay<RealType, ElementSize>::SetTriangle(int tid, const Index3& tv, bool bAllowElementFreeing)
	{
		if (IsElement(tv[0]) == false || IsElement(tv[1]) == false || IsElement(tv[2]) == false)
		{
			assert(false);
			return EMeshResult::Failed_NotAVertex;
		}
		if (tv[0] == tv[1] || tv[0] == tv[2] || tv[1] == tv[2])
		{
			assert(false);
			return EMeshResult::Failed_InvalidNeighbourhood;
		}

		InternalSetTriangle(tid, tv, true, bAllowElementFreeing);

		//updateTimeStamp(true);
		return EMeshResult::Ok;
	}

	template<typename RealType, int ElementSize>
	void TDynamicMeshOverlay<RealType, ElementSize>::InternalSetTriangle(int tid, const Index3& tv, bool bUpdateRefCounts, bool bAllowElementFreeing)
	{
		if (ParentMesh == nullptr)
		{
			return;
		}

		// If we have to decrement refcounts, we will do it at the end, because Decrement() frees
		// elements as soon as they lose their last reference, so a Decrement followed by Increment
		// can leave things in an invalid state.
		bool bNeedToDecrement = false;
		Index3 OldTriElements; // only used if need to decrement.

		int i = 3 * tid;

		// See if triangle existed and make it exist if not
		if (ElementTriangles.size() >= i + 3 && bUpdateRefCounts)
		{
			OldTriElements = GetTriangle(tid);
			bNeedToDecrement = (OldTriElements[0] != -1);
		}
		else
		{
			ElementTriangles.resize(i + 3, -1);
		}

		ElementTriangles[i + 2] = tv[2];
		ElementTriangles[i + 1] = tv[1];
		ElementTriangles[i] = tv[0];

		if (bUpdateRefCounts)
		{
			ElementsRefCounts.Increment(tv[0]);
			ElementsRefCounts.Increment(tv[1]);
			ElementsRefCounts.Increment(tv[2]);

			if (bNeedToDecrement)
			{
				for (int j = 0; j < 3; ++j)
				{
					ElementsRefCounts.Decrement(OldTriElements[j]);

					if (bAllowElementFreeing && ElementsRefCounts.GetRefCount(OldTriElements[j]) == 1)
					{
						ElementsRefCounts.Decrement(OldTriElements[j]);
						ParentVertices[OldTriElements[j]] = -1;
					}
				};
			}
		}

		if (tv.a != -1 || tv.b != -1 || tv.c != -1)
		{
			// Set parent vertex IDs
			const Index3 ParentTriangle = ParentMesh->GetTriangle(tid);

			for (int VInd = 0; VInd < 3; ++VInd)
			{
				// Checks that the parent vertices of the elements that we're referencing in the overlay
				// triangle are either not yet set or already point to the vertices of the corresponding
				// mesh triangle (and so will remain unchanged). Remember that the same element is not
				// allowed to be used for multiple vertices.
				assert(ParentVertices[tv[VInd]] == ParentTriangle[VInd] || ParentVertices[tv[VInd]] == -1);

				VectorSetSafe(ParentVertices, tv[VInd], ParentTriangle[VInd], -1);
			}
		}
	}

	template<typename RealType, int ElementSize>
	void TDynamicMeshOverlay<RealType, ElementSize>::OnSplitEdge(const FEdgeSplitInfo& splitInfo)
	{
		int orig_t0 = splitInfo.OriginalTriangles.a;
		int orig_t1 = splitInfo.OriginalTriangles.b;
		int base_a = splitInfo.OriginalVertices.a;
		int base_b = splitInfo.OriginalVertices.b;

		// special handling if either triangle is unset
		bool bT0Set = IsSetTriangle(orig_t0), bT1Set = orig_t1 >= 0 && IsSetTriangle(orig_t1);
		// insert invalid triangle as needed
		if (!bT0Set)
		{
			InitializeNewTriangle(splitInfo.NewTriangles.a);
		}
		if (!bT1Set && splitInfo.NewTriangles.b >= 0)
		{
			// new triangle is invalid
			InitializeNewTriangle(splitInfo.NewTriangles.b);
		}
		// if neither tri was set, nothing else to do
		if (!bT0Set && !bT1Set)
		{
			return;
		}

		// look up current triangle 0, and infer base triangle 0
		Index3 Triangle0(-1, -1, -1);
		int idx_base_a1 = -1, idx_base_b1 = -1;
		int NewElemID = -1;
		if (bT0Set)
		{
			Triangle0 = GetTriangle(orig_t0);
			Index3 BaseTriangle0(ParentVertices[Triangle0.a], ParentVertices[Triangle0.b], ParentVertices[Triangle0.c]);
			idx_base_a1 = BaseTriangle0.IndexOf(base_a);
			idx_base_b1 = BaseTriangle0.IndexOf(base_b);
			int idx_base_c = GetOtherTriIndex(idx_base_a1, idx_base_b1);

			// create new element at lerp position
			NewElemID = AppendElement((RealType)0);
			SetElementFromLerp(NewElemID, Triangle0[idx_base_a1], Triangle0[idx_base_b1], (double)splitInfo.SplitT);

			// rewrite triangle 0
			ElementTriangles[3 * orig_t0 + idx_base_b1] = NewElemID;

			// create new triangle 2 w/ correct winding order
			Index3 NewTriangle2(NewElemID, Triangle0[idx_base_b1], Triangle0[idx_base_c]);  // mirrors DMesh3::SplitEdge [f,b,c]
			InternalSetTriangle(splitInfo.NewTriangles.a, NewTriangle2, false);

			// update ref counts
			ElementsRefCounts.Increment(NewElemID, 2); // for the two tris on the T0 side
			ElementsRefCounts.Increment(Triangle0[idx_base_c]);
		}

		if (orig_t1 == -1)
		{
			return;  // we are done if this is a boundary triangle
		}

		// look up current triangle1 and infer base triangle 1
		if (bT1Set)
		{
			Index3 Triangle1 = GetTriangle(orig_t1);
			Index3 BaseTriangle1(ParentVertices[Triangle1.a], ParentVertices[Triangle1.b], ParentVertices[Triangle1.c]);
			int idx_base_a2 = BaseTriangle1.IndexOf(base_a);
			int idx_base_b2 = BaseTriangle1.IndexOf(base_b);
			int idx_base_d = GetOtherTriIndex(idx_base_a2, idx_base_b2);

			int OtherNewElemID = NewElemID;

			// if we don't have a shared edge, we need to create another new UV for the other side
			bool bHasSharedUVEdge = bT0Set && SamePairUnordered(Triangle0[idx_base_a1], Triangle0[idx_base_b1], Triangle1[idx_base_a2], Triangle1[idx_base_b2]);
			if (bHasSharedUVEdge == false)
			{
				// create new element at lerp position
				OtherNewElemID = AppendElement((RealType)0);
				SetElementFromLerp(OtherNewElemID, Triangle1[idx_base_a2], Triangle1[idx_base_b2], (double)splitInfo.SplitT);
			}

			// rewrite triangle 1
			ElementTriangles[3 * orig_t1 + idx_base_b2] = OtherNewElemID;

			// create new triangle 3 w/ correct winding order
			Index3 NewTriangle3(OtherNewElemID, Triangle1[idx_base_d], Triangle1[idx_base_b2]);  // mirrors DMesh3::SplitEdge [f,d,b]
			InternalSetTriangle(splitInfo.NewTriangles.b, NewTriangle3, false);

			// update ref counts
			ElementsRefCounts.Increment(OtherNewElemID, 2);
			ElementsRefCounts.Increment(Triangle1[idx_base_d]);
		}
	}

	template<typename RealType, int ElementSize>
	void TDynamicMeshOverlay<RealType, ElementSize>::OnFlipEdge(const FEdgeFlipInfo& FlipInfo)
	{
		int orig_t0 = FlipInfo.Triangles.a;
		int orig_t1 = FlipInfo.Triangles.b;
		bool bT0Set = IsSetTriangle(orig_t0), bT1Set = IsSetTriangle(orig_t1);
		if (!bT0Set)
		{
			assert(!bT1Set); // flipping across a set/unset boundary is not allowed?
			return; // nothing to do on the overlay if both triangles are unset
		}

		int base_a = FlipInfo.OriginalVerts.a;
		int base_b = FlipInfo.OriginalVerts.b;
		int base_c = FlipInfo.OpposingVerts.a;
		int base_d = FlipInfo.OpposingVerts.b;

		// look up triangle 0
		Index3 Triangle0 = GetTriangle(orig_t0);
		Index3 BaseTriangle0(ParentVertices[Triangle0.a], ParentVertices[Triangle0.b], ParentVertices[Triangle0.c]);
		int idx_base_a1 = BaseTriangle0.IndexOf(base_a);
		int idx_base_b1 = BaseTriangle0.IndexOf(base_b);
		int idx_base_c = GetOtherTriIndex(idx_base_a1, idx_base_b1);

		// look up triangle 1 (must exist because base mesh would never flip a boundary edge)
		Index3 Triangle1 = GetTriangle(orig_t1);
		Index3 BaseTriangle1(ParentVertices[Triangle1.a], ParentVertices[Triangle1.b], ParentVertices[Triangle1.c]);
		int idx_base_a2 = BaseTriangle1.IndexOf(base_a);
		int idx_base_b2 = BaseTriangle1.IndexOf(base_b);
		int idx_base_d = GetOtherTriIndex(idx_base_a2, idx_base_b2);

		// sanity checks
		assert(idx_base_c == BaseTriangle0.IndexOf(base_c));
		assert(idx_base_d == BaseTriangle1.IndexOf(base_d));

		// we should not have been called on a non-shared edge!!
		bool bHasSharedUVEdge = SamePairUnordered(Triangle0[idx_base_a1], Triangle0[idx_base_b1], Triangle1[idx_base_a2], Triangle1[idx_base_b2]);
		assert(bHasSharedUVEdge);

		int A = Triangle0[idx_base_a1];
		int B = Triangle0[idx_base_b1];
		int C = Triangle0[idx_base_c];
		int D = Triangle1[idx_base_d];

		// set triangles to same index order as in FDynamicMesh::FlipEdge
		int i0 = 3 * orig_t0;
		ElementTriangles[i0] = C; ElementTriangles[i0 + 1] = D; ElementTriangles[i0 + 2] = B;
		int i1 = 3 * orig_t1;
		ElementTriangles[i1] = D; ElementTriangles[i1 + 1] = C; ElementTriangles[i1 + 2] = A;

		// update reference counts
		ElementsRefCounts.Decrement(A);
		ElementsRefCounts.Decrement(B);
		ElementsRefCounts.Increment(C);
		ElementsRefCounts.Increment(D);
	}

	template<typename RealType, int ElementSize>
	void TDynamicMeshOverlay<RealType, ElementSize>::OnPokeTriangle(const FPokeTriangleInfo& PokeInfo)
	{
		if (!IsSetTriangle(PokeInfo.OriginalTriangle))
		{
			InitializeNewTriangle(PokeInfo.NewTriangles.a);
			InitializeNewTriangle(PokeInfo.NewTriangles.b);
			return;
		}

		Index3 Triangle = GetTriangle(PokeInfo.OriginalTriangle);

		// create new element at barycentric position
		int CenterElemID = AppendElement((RealType)0);
		SetElementFromBary(CenterElemID, Triangle[0], Triangle[1], Triangle[2], PokeInfo.BaryCoords);

		// update orig triangle and two new ones. Winding orders here mirror FDynamicMesh3::PokeTriangle
		InternalSetTriangle(PokeInfo.OriginalTriangle, Index3(Triangle[0], Triangle[1], CenterElemID), false);
		InternalSetTriangle(PokeInfo.NewTriangles.a, Index3(Triangle[1], Triangle[2], CenterElemID), false);
		InternalSetTriangle(PokeInfo.NewTriangles.b, Index3(Triangle[2], Triangle[0], CenterElemID), false);

		ElementsRefCounts.Increment(Triangle[0]);
		ElementsRefCounts.Increment(Triangle[1]);
		ElementsRefCounts.Increment(Triangle[2]);
		ElementsRefCounts.Increment(CenterElemID, 3);
	}

	template<typename RealType, int ElementSize>
	void TDynamicMeshOverlay<RealType, ElementSize>::OnMergeEdges(const FMergeEdgesInfo& MergeInfo)
	{
		// MergeEdges just merges vertices. For now we will not also merge UVs. So all we need to
		// do is rewrite the UV parent vertices

		for (int i = 0; i < 2; i++)
		{
			int KeptVID = MergeInfo.KeptVerts[i];
			int RemovedVID = MergeInfo.RemovedVerts[i];
			if (RemovedVID == -1)
			{
				continue;
			}
			// this for loop is very similar to GetVertexElements() but accounts for the base mesh already being updated
			std::vector<int> VtxTrianglesItr = ParentMesh->GetVexTriangles(KeptVID);
			for (int TID : VtxTrianglesItr) // only care about triangles connected to the *new* vertex; these are updated
			{
				if (!IsSetTriangle(TID))
				{
					continue;
				}
				Index3 Triangle = GetTriangle(TID);
				for (int j = 0; j < 3; ++j)
				{
					// though the ParentMesh vertex is NewVertex in the source mesh, it is still OriginalVertex in the ParentVertices array (since that hasn't been updated yet)
					if (Triangle[j] != -1 && ParentVertices[Triangle[j]] == RemovedVID)
					{
						ParentVertices[Triangle[j]] = KeptVID;
					}
				}
			}
		}
	}

	template<typename RealType, int ElementSize>
	void TDynamicMeshOverlay<RealType, ElementSize>::OnCollapseEdge(const FEdgeCollapseInfo& collapseInfo)
	{
		int tid_removed0 = collapseInfo.RemovedTris.a;
		int tid_removed1 = collapseInfo.RemovedTris.b;
		bool bT0Set = IsSetTriangle(tid_removed0), bT1Set = tid_removed1 >= 0 && IsSetTriangle(tid_removed1);

		int vid_base_kept = collapseInfo.KeptVertex;
		int vid_base_removed = collapseInfo.RemovedVertex;


		bool bIsSeam = false;
		bool bIsSeamEnd = false;


		// look up triangle 0
		Index3 Triangle0(-1, -1, -1), BaseTriangle0(-1, -1, -1);
		int idx_removed_tri0 = -1, idx_kept_tri0 = -1;
		if (bT0Set)
		{
			Triangle0 = GetTriangle(tid_removed0);
			BaseTriangle0 = Index3(ParentVertices[Triangle0.a], ParentVertices[Triangle0.b], ParentVertices[Triangle0.c]);
			idx_kept_tri0 = BaseTriangle0.IndexOf(vid_base_kept);
			idx_removed_tri0 = BaseTriangle0.IndexOf(vid_base_removed);
		}

		// look up triangle 1 if this is not a boundary edge
		Index3 Triangle1(-1, -1, -1), BaseTriangle1(-1, -1, -1);
		int idx_removed_tri1 = -1, idx_kept_tri1 = -1;
		if (collapseInfo.bIsBoundary == false && bT1Set)
		{
			Triangle1 = GetTriangle(tid_removed1);
			BaseTriangle1 = Index3(ParentVertices[Triangle1.a], ParentVertices[Triangle1.b], ParentVertices[Triangle1.c]);

			idx_kept_tri1 = BaseTriangle1.IndexOf(vid_base_kept);
			idx_removed_tri1 = BaseTriangle1.IndexOf(vid_base_removed);

			if (bT0Set)
			{
				int el_kept_tri0 = Triangle0[idx_kept_tri0];
				int el_removed_tri0 = Triangle0[idx_removed_tri0];
				int el_kept_tri1 = Triangle1[idx_kept_tri1];
				int el_removed_tri1 = Triangle1[idx_removed_tri1];

				// is this a seam?
				bIsSeam = !SamePairUnordered(el_kept_tri0, el_removed_tri0, el_kept_tri1, el_removed_tri1);

				if (bIsSeam)
				{
					// is only one of elements split?
					if ((el_kept_tri0 == el_kept_tri1 || el_kept_tri0 == el_removed_tri1) || (el_removed_tri0 == el_kept_tri1 || el_removed_tri0 == el_removed_tri1))
					{
						bIsSeamEnd = true;
					}
				}
			}
		}

		// this should be protected against by calling code.
		//assert(!bIsSeamEnd);

		// need to find the elementid for the "kept" and "removed" vertices that are connected by the edges of T0 and T1.
		// If this edge is :
		//       not a seam - just one kept and one removed element.
		//       a seam end - one (two) kept and two (one) removed elements.
		//       a seam     - two kept and two removed elements.
		// The collapse of a seam end must be protected against by the higher-level code.
		//     There is no sensible way to handle the collapse of a seam end.  Retaining two elements would require some arbitrary split
		//     of the removed element and conversely if one element is retained there is no reason to believe a single element value 
		//     would be a good approximation to collapsing the edges on both sides of the seam end.
		int kept_elemid[2] = { -1, -1 };
		int removed_elemid[2] = { -1, -1 };
		bool bFoundRemovedElement[2] = { false,false };
		bool bFoundKeptElement[2] = { false, false };
		if (bT0Set)
		{
			kept_elemid[0] = Triangle0[idx_kept_tri0];
			removed_elemid[0] = Triangle0[idx_removed_tri0];
			bFoundKeptElement[0] = bFoundRemovedElement[0] = true;

			assert(kept_elemid[0] != -1);
			assert(removed_elemid[0] != -1);

		}
		if ((bIsSeam || !bT0Set) && bT1Set)
		{
			kept_elemid[1] = Triangle1[idx_kept_tri1];
			removed_elemid[1] = Triangle1[idx_removed_tri1];
			bFoundKeptElement[1] = bFoundRemovedElement[1] = true;

			assert(kept_elemid[1] != -1);
			assert(removed_elemid[1] != -1);

		}

		// update value of kept elements
		for (int i = 0; i < 2; ++i)
		{
			if (kept_elemid[i] == -1 || removed_elemid[i] == -1)
			{
				continue;
			}

			SetElementFromLerp(kept_elemid[i], kept_elemid[i], removed_elemid[i], (double)collapseInfo.CollapseT);
		}

		// Helper for detaching from elements further below. Technically, the freeing gets done for us if the 
		// triangle unset call is the last detachment (which it should be as long as only elements on a removed 
		// overlay edge are ones that are removed), but it is saner to have it.
		auto DecrementAndFreeIfLast = [this](int elem_id)
		{
			ElementsRefCounts.Decrement(elem_id);
			if (ElementsRefCounts.GetRefCount(elem_id) == 1)
			{
				ElementsRefCounts.Decrement(elem_id);
				ParentVertices[elem_id] = -1;
			}
		};


		// Look for still-existing triangles that have elements linked to the removed vertex and update them.
		// Note that this has to happen even if both triangles were unset, as the removed vertex may have had
		// other elements associated with it, so we need to look at its triangles (which are now attached to
		// vid_base_kept).
		std::vector<int> VtxTrianglesItr = ParentMesh->GetVexTriangles(vid_base_kept);
		for (int onering_tid : VtxTrianglesItr)
		{
			if (!IsSetTriangle(onering_tid))
			{
				continue;
			}
			Index3 elem_tri = GetTriangle(onering_tid);
			for (int j = 0; j < 3; ++j)
			{
				int elem_id = elem_tri[j];
				if (ParentVertices[elem_id] == vid_base_removed)
				{
					if (elem_id == removed_elemid[0])
					{
						ElementTriangles[3 * onering_tid + j] = kept_elemid[0];
						if (bFoundKeptElement[0])
						{
							ElementsRefCounts.Increment(kept_elemid[0]);
						}
						DecrementAndFreeIfLast(elem_id);
					}
					else if (elem_id == removed_elemid[1])
					{
						ElementTriangles[3 * onering_tid + j] = kept_elemid[1];
						if (bFoundKeptElement[1])
						{
							ElementsRefCounts.Increment(kept_elemid[1]);
						}
						DecrementAndFreeIfLast(elem_id);
					}
					else
					{
						// this could happen if a split edge is adjacent to the edge we collapse
						ParentVertices[elem_id] = vid_base_kept;
					}
				}
			}
		}


		// clear the two triangles we removed
		if (bT0Set)
		{
			UnsetTriangle(tid_removed0, true);
		}
		if (collapseInfo.bIsBoundary == false && bT1Set)
		{
			UnsetTriangle(tid_removed1, true);
		}

		// if the edge was split, but still shared one element, this should be protected against in the calling code
		if (removed_elemid[1] == removed_elemid[0])
		{
			removed_elemid[1] = -1;
		}
	}

	template<typename RealType, int ElementSize>
	void TDynamicMeshOverlay<RealType, ElementSize>::OnSplitVertex(const FVertexSplitInfo& SplitInfo, const std::vector<int>& TrianglesToUpdate)
	{
		std::set<int> OutElements;

		// this for loop is very similar to GetVertexElements() but accounts for the base mesh already being updated
		std::vector<int> VtxTrianglesItr = ParentMesh->GetVexTriangles(SplitInfo.NewVertex);
		for (int tid : VtxTrianglesItr) // only care about triangles connected to the *new* vertex; these are updated
		{
			if (!IsSetTriangle(tid))
			{
				continue;
			}
			Index3 Triangle = GetTriangle(tid);
			for (int j = 0; j < 3; ++j)
			{
				// though the ParentMesh vertex is NewVertex in the source mesh, it is still OriginalVertex in the ParentVertices array (since that hasn't been updated yet)
				if (Triangle[j] != -1 && ParentVertices[Triangle[j]] == SplitInfo.OriginalVertex)
				{
					OutElements.insert(Triangle[j]);
				}
			}
		}

		for (int ElementID : OutElements)
		{
			// Note: TrianglesToUpdate will include triangles that don't include the element, but that's ok; it just won't find any elements to update for those
			//			(and this should be cheaper than constructing a new array for every element)
			SplitElementWithNewParent(ElementID, SplitInfo.NewVertex, TrianglesToUpdate);
		}
	}

	DynamicMeshAttributeSet::DynamicMeshAttributeSet(DynamicMesh* Mesh)
	{
		ParentMesh = Mesh;
		SetNumUVLayers(2);
		SetNumNormalLayers(1);
		EnableMaterialID();
	}

	DynamicMeshAttributeSet::DynamicMeshAttributeSet(DynamicMesh* Mesh, int NumUVLayers, int NumNormalLayers)
	{
		ParentMesh = Mesh;
		SetNumUVLayers(NumUVLayers);
		SetNumNormalLayers(NumNormalLayers);
		EnableMaterialID();
	}

	DynamicMeshAttributeSet::~DynamicMeshAttributeSet()
	{
		DisableMaterialID();
		DisablePrimaryColors();
	}

	bool DynamicMeshAttributeSet::IsSeamEdge(int eid) const
	{
		for (const FDynamicMeshUVOverlay& UVLayer : UVLayers)
		{
			if (UVLayer.IsSeamEdge(eid))
			{
				return true;
			}
		}

		for (const FDynamicMeshNormalOverlay& NormalLayer : NormalLayers)
		{
			if (NormalLayer.IsSeamEdge(eid))
			{
				return true;
			}
		}

		if (ColorLayer && ColorLayer->IsSeamEdge(eid))
		{
			return true;
		}

		return false;
	}

	bool DynamicMeshAttributeSet::IsSeamEndEdge(int eid) const
	{
		for (const FDynamicMeshUVOverlay& UVLayer : UVLayers)
		{
			if (UVLayer.IsSeamEndEdge(eid))
			{
				return true;
			}
		}

		for (const FDynamicMeshNormalOverlay& NormalLayer : NormalLayers)
		{
			if (NormalLayer.IsSeamEndEdge(eid))
			{
				return true;
			}
		}

		if (ColorLayer && ColorLayer->IsSeamEndEdge(eid))
		{
			return true;
		}
		return false;
	}

	bool DynamicMeshAttributeSet::IsSeamEdge(int EdgeID, bool& bIsUVSeamOut, bool& bIsNormalSeamOut, bool& bIsColorSeamOut, bool& bIsTangentSeamOut) const
	{
		bIsUVSeamOut = false;
		for (const FDynamicMeshUVOverlay& UVLayer : UVLayers)
		{
			if (UVLayer.IsSeamEdge(EdgeID))
			{
				bIsUVSeamOut = true;
			}
		}

		bIsNormalSeamOut = !NormalLayers.empty() && NormalLayers[0].IsSeamEdge(EdgeID);
		bIsTangentSeamOut = false;
		for (size_t LayerIdx = 1; LayerIdx < NormalLayers.size(); ++LayerIdx)
		{
			const FDynamicMeshNormalOverlay& NormalLayer = NormalLayers[LayerIdx];
			if (NormalLayer.IsSeamEdge(EdgeID))
			{
				bIsTangentSeamOut = true;
			}
		}

		bIsColorSeamOut = false;
		if (ColorLayer && ColorLayer->IsSeamEdge(EdgeID))
		{
			bIsColorSeamOut = true;
		}
		return (bIsUVSeamOut || bIsNormalSeamOut || bIsColorSeamOut || bIsTangentSeamOut);
	}


	bool DynamicMeshAttributeSet::IsSeamVertex(int VID, bool bBoundaryIsSeam) const
	{
		for (const FDynamicMeshUVOverlay& UVLayer : UVLayers)
		{
			if (UVLayer.IsSeamVertex(VID, bBoundaryIsSeam))
			{
				return true;
			}
		}
		for (const FDynamicMeshNormalOverlay& NormalLayer : NormalLayers)
		{
			if (NormalLayer.IsSeamVertex(VID, bBoundaryIsSeam))
			{
				return true;
			}
		}
		if (ColorLayer && ColorLayer->IsSeamVertex(VID, bBoundaryIsSeam))
		{
			return true;
		}
		return false;
	}

	bool DynamicMeshAttributeSet::IsMaterialBoundaryEdge(int EdgeID) const
	{
		if (MaterialIDAttrib == nullptr)
		{
			return false;
		}
		assert(ParentMesh->IsEdge(EdgeID));
		if (ParentMesh->IsEdge(EdgeID))
		{
			const DynamicMesh::Edge Edge = ParentMesh->GetEdge(EdgeID);
			const int Tri0 = Edge.Tri[0];
			const int Tri1 = Edge.Tri[1];
			if ((Tri0 == -1) || (Tri1 == -1))
			{
				return false;
			}
			const int MatID0 = MaterialIDAttrib->GetValue(Tri0);
			const int MatID1 = MaterialIDAttrib->GetValue(Tri1);
			return MatID0 != MatID1;
		}
		return false;
	}

	void DynamicMeshAttributeSet::EnableMaterialID()
	{
		if (HasMaterialID() == false)
		{
			MaterialIDAttrib = new FDynamicMeshMaterialAttribute(ParentMesh);
			MaterialIDAttrib->Initialize(0);
		}
	}

	void DynamicMeshAttributeSet::DisableMaterialID()
	{
		if (MaterialIDAttrib)
		{
			delete MaterialIDAttrib;
			MaterialIDAttrib = nullptr;
		}
	}

	void DynamicMeshAttributeSet::SetNumUVLayers(int Num)
	{
		if ((int)UVLayers.size() == Num)
		{
			return;
		}
		if (Num >= (int)UVLayers.size())
		{
			for (int i = (int)UVLayers.size(); i < Num; ++i)
			{
				FDynamicMeshUVOverlay NewUVLayer(ParentMesh);
				NewUVLayer.InitializeTriangles(ParentMesh->GetTriangleCount());
				UVLayers.push_back(NewUVLayer);
			}
		}
		else
		{
			UVLayers.resize(Num);
		}
		assert(UVLayers.size() == (int)Num);
	}

	void DynamicMeshAttributeSet::SetNumNormalLayers(int Num)
	{
		if ((int)NormalLayers.size() == Num)
		{
			return;
		}
		if (Num >= (int)NormalLayers.size())
		{
			for (int i = (int)NormalLayers.size(); i < Num; ++i)
			{
				FDynamicMeshNormalOverlay NewNormalLayer(ParentMesh);
				NewNormalLayer.InitializeTriangles(ParentMesh->GetTriangleCount());
				NormalLayers.push_back(NewNormalLayer);
			}
		}
		else
		{
			NormalLayers.resize(Num);
		}
		assert((int)NormalLayers.size() == Num);
	}

	void DynamicMeshAttributeSet::EnablePrimaryColors()
	{
		if (HasPrimaryColors() == false)
		{
			ColorLayer = new FDynamicMeshColorOverlay(ParentMesh);
			ColorLayer->InitializeTriangles(ParentMesh->GetTriangleCount());
		}
	}

	void DynamicMeshAttributeSet::OnNewVertex(int VertexID, bool bInserted)
	{
		for (auto pair : GenericAttributes)
		{
			FDynamicMeshAttributeBase *attrib = pair.second;
			attrib->OnNewTriangle(VertexID, bInserted);
		}

		for (FDynamicMeshWeightAttribute& WeightLayer : WeightLayers)
		{
			float NewWeight = 0.0f;
			WeightLayer.SetNewValue(VertexID, &NewWeight);
		}
	}


	void DynamicMeshAttributeSet::OnRemoveVertex(int VertexID)
	{
		for (auto pair : GenericAttributes)
		{
			FDynamicMeshAttributeBase* attrib = pair.second;
			attrib->OnRemoveVertex(VertexID);
		}

		for (FDynamicMeshWeightAttribute& WeightLayer : WeightLayers)
		{
			WeightLayer.OnRemoveVertex(VertexID);
		}
	}

	void DynamicMeshAttributeSet::Initialize(int MaxVertexID, int MaxTriangleID)
	{
		for (FDynamicMeshUVOverlay& UVLayer : UVLayers)
		{
			UVLayer.InitializeTriangles(MaxTriangleID);
		}
		for (FDynamicMeshNormalOverlay& NormalLayer : NormalLayers)
		{
			NormalLayer.InitializeTriangles(MaxTriangleID);
		}
	}

	void DynamicMeshAttributeSet::OnNewTriangle(int TriangleID, bool bInserted)
	{
		for (auto pair : GenericAttributes)
		{
			FDynamicMeshAttributeBase* attrib = pair.second;
			attrib->OnNewTriangle(TriangleID, bInserted);
		}

		for (FDynamicMeshUVOverlay& UVLayer : UVLayers)
		{
			UVLayer.InitializeNewTriangle(TriangleID);
		}
		for (FDynamicMeshNormalOverlay& NormalLayer : NormalLayers)
		{
			NormalLayer.InitializeNewTriangle(TriangleID);
		}
		if (ColorLayer)
		{
			ColorLayer->InitializeNewTriangle(TriangleID);
		}
		if (MaterialIDAttrib)
		{
			int NewValue = 0;
			MaterialIDAttrib->SetNewValue(TriangleID, &NewValue);
		}

		//for (FDynamicMeshPolygroupAttribute& PolygroupLayer : PolygroupLayers)
		//{
		//	int NewGroup = 0;
		//	PolygroupLayer.SetNewValue(TriangleID, &NewGroup);
		//}
	}


	void DynamicMeshAttributeSet::OnRemoveTriangle(int TriangleID)
	{
		for (auto pair : GenericAttributes)
		{
			FDynamicMeshAttributeBase* attrib = pair.second;
			attrib->OnRemoveTriangle(TriangleID);
		}

		for (FDynamicMeshUVOverlay& UVLayer : UVLayers)
		{
			UVLayer.OnRemoveTriangle(TriangleID);
		}
		for (FDynamicMeshNormalOverlay& NormalLayer : NormalLayers)
		{
			NormalLayer.OnRemoveTriangle(TriangleID);
		}
		if (ColorLayer)
		{
			ColorLayer->OnRemoveTriangle(TriangleID);
		}

		// has no effect on MaterialIDAttrib
	}

	void DynamicMeshAttributeSet::OnReverseTriOrientation(int TriangleID)
	{
		for (auto pair : GenericAttributes)
		{
			FDynamicMeshAttributeBase* attrib = pair.second;
			attrib->OnReverseTriOrientation(TriangleID);
		}

		for (FDynamicMeshUVOverlay& UVLayer : UVLayers)
		{
			UVLayer.OnReverseTriOrientation(TriangleID);
		}
		for (FDynamicMeshNormalOverlay& NormalLayer : NormalLayers)
		{
			NormalLayer.OnReverseTriOrientation(TriangleID);
		}
		if (ColorLayer)
		{
			ColorLayer->OnReverseTriOrientation(TriangleID);
		}
		// has no effect on MaterialIDAttrib
	}

	void DynamicMeshAttributeSet::OnSplitEdge(const FEdgeSplitInfo& SplitInfo)
	{
		for (auto pair : GenericAttributes)
		{
			FDynamicMeshAttributeBase* attrib = pair.second;
			attrib->OnSplitEdge(SplitInfo);
		}

		for (FDynamicMeshUVOverlay& UVLayer : UVLayers)
		{
			UVLayer.OnSplitEdge(SplitInfo);
		}
		for (FDynamicMeshNormalOverlay& NormalLayer : NormalLayers)
		{
			NormalLayer.OnSplitEdge(SplitInfo);
		}
		if (ColorLayer)
		{
			ColorLayer->OnSplitEdge(SplitInfo);
		}
		if (MaterialIDAttrib)
		{
			MaterialIDAttrib->OnSplitEdge(SplitInfo);
		}

		//for (FDynamicMeshPolygroupAttribute& PolygroupLayer : PolygroupLayers)
		//{
		//	PolygroupLayer.OnSplitEdge(SplitInfo);
		//}

		for (FDynamicMeshWeightAttribute& WeightLayer : WeightLayers)
		{
			WeightLayer.OnSplitEdge(SplitInfo);
		}
	}

	void DynamicMeshAttributeSet::OnFlipEdge(const FEdgeFlipInfo& flipInfo)
	{
		for (auto pair : GenericAttributes)
		{
			FDynamicMeshAttributeBase* attrib = pair.second;
			attrib->OnFlipEdge(flipInfo);
		}

		for (FDynamicMeshUVOverlay& UVLayer : UVLayers)
		{
			UVLayer.OnFlipEdge(flipInfo);
		}
		for (FDynamicMeshNormalOverlay& NormalLayer : NormalLayers)
		{
			NormalLayer.OnFlipEdge(flipInfo);
		}
		if (ColorLayer)
		{
			ColorLayer->OnFlipEdge(flipInfo);
		}
		if (MaterialIDAttrib)
		{
			MaterialIDAttrib->OnFlipEdge(flipInfo);
		}

		//for (FDynamicMeshPolygroupAttribute& PolygroupLayer : PolygroupLayers)
		//{
		//	PolygroupLayer.OnFlipEdge(flipInfo);
		//}

		for (FDynamicMeshWeightAttribute& WeightLayer : WeightLayers)
		{
			WeightLayer.OnFlipEdge(flipInfo);
		}
	}


	void DynamicMeshAttributeSet::OnCollapseEdge(const FEdgeCollapseInfo& collapseInfo)
	{
		for (auto pair : GenericAttributes)
		{
			FDynamicMeshAttributeBase* attrib = pair.second;
			attrib->OnCollapseEdge(collapseInfo);
		}

		for (FDynamicMeshUVOverlay& UVLayer : UVLayers)
		{
			UVLayer.OnCollapseEdge(collapseInfo);
		}
		for (FDynamicMeshNormalOverlay& NormalLayer : NormalLayers)
		{
			NormalLayer.OnCollapseEdge(collapseInfo);
		}
		if (ColorLayer)
		{
			ColorLayer->OnCollapseEdge(collapseInfo);
		}
		if (MaterialIDAttrib)
		{
			MaterialIDAttrib->OnCollapseEdge(collapseInfo);
		}

		//for (FDynamicMeshPolygroupAttribute& PolygroupLayer : PolygroupLayers)
		//{
		//	PolygroupLayer.OnCollapseEdge(collapseInfo);
		//}

		for (FDynamicMeshWeightAttribute& WeightLayer : WeightLayers)
		{
			WeightLayer.OnCollapseEdge(collapseInfo);
		}

	}

	void DynamicMeshAttributeSet::OnPokeTriangle(const FPokeTriangleInfo& pokeInfo)
	{
		for (auto pair : GenericAttributes)
		{
			FDynamicMeshAttributeBase* attrib = pair.second;
			attrib->OnPokeTriangle(pokeInfo);
		}

		for (FDynamicMeshUVOverlay& UVLayer : UVLayers)
		{
			UVLayer.OnPokeTriangle(pokeInfo);
		}
		for (FDynamicMeshNormalOverlay& NormalLayer : NormalLayers)
		{
			NormalLayer.OnPokeTriangle(pokeInfo);
		}
		if (ColorLayer)
		{
			ColorLayer->OnPokeTriangle(pokeInfo);
		}
		if (MaterialIDAttrib)
		{
			MaterialIDAttrib->OnPokeTriangle(pokeInfo);
		}

		//for (FDynamicMeshPolygroupAttribute& PolygroupLayer : PolygroupLayers)
		//{
		//	PolygroupLayer.OnPokeTriangle(pokeInfo);
		//}

		for (FDynamicMeshWeightAttribute& WeightLayer : WeightLayers)
		{
			WeightLayer.OnPokeTriangle(pokeInfo);
		}
	}

	void DynamicMeshAttributeSet::OnMergeEdges(const FMergeEdgesInfo& mergeInfo)
	{
		for (auto pair : GenericAttributes)
		{
			FDynamicMeshAttributeBase* attrib = pair.second;
			attrib->OnMergeEdges(mergeInfo);
		}

		for (FDynamicMeshUVOverlay& UVLayer : UVLayers)
		{
			UVLayer.OnMergeEdges(mergeInfo);
		}
		for (FDynamicMeshNormalOverlay& NormalLayer : NormalLayers)
		{
			NormalLayer.OnMergeEdges(mergeInfo);
		}
		if (ColorLayer)
		{
			ColorLayer->OnMergeEdges(mergeInfo);
		}
		if (MaterialIDAttrib)
		{
			MaterialIDAttrib->OnMergeEdges(mergeInfo);
		}

		//for (FDynamicMeshPolygroupAttribute& PolygroupLayer : PolygroupLayers)
		//{
		//	PolygroupLayer.OnMergeEdges(mergeInfo);
		//}

		for (FDynamicMeshWeightAttribute& WeightLayer : WeightLayers)
		{
			WeightLayer.OnMergeEdges(mergeInfo);
		}
	}

	void DynamicMeshAttributeSet::OnSplitVertex(const FVertexSplitInfo& SplitInfo, const std::vector<int>& TrianglesToUpdate)
	{
		for (auto pair : GenericAttributes)
		{
			FDynamicMeshAttributeBase* attrib = pair.second;
			attrib->OnSplitVertex(SplitInfo, TrianglesToUpdate);
		}

		for (FDynamicMeshUVOverlay& UVLayer : UVLayers)
		{
			UVLayer.OnSplitVertex(SplitInfo, TrianglesToUpdate);
		}
		for (FDynamicMeshNormalOverlay& NormalLayer : NormalLayers)
		{
			NormalLayer.OnSplitVertex(SplitInfo, TrianglesToUpdate);
		}
		if (ColorLayer)
		{
			ColorLayer->OnSplitVertex(SplitInfo, TrianglesToUpdate);
		}
		if (MaterialIDAttrib)
		{
			MaterialIDAttrib->OnSplitVertex(SplitInfo, TrianglesToUpdate);
		}

		//for (FDynamicMeshPolygroupAttribute& PolygroupLayer : PolygroupLayers)
		//{
		//	PolygroupLayer.OnSplitVertex(SplitInfo, TrianglesToUpdate);
		//}

		for (FDynamicMeshWeightAttribute& WeightLayer : WeightLayers)
		{
			WeightLayer.OnSplitVertex(SplitInfo, TrianglesToUpdate);
		}
	}

	template<typename AttribValueType, int AttribDimension>
	void TDynamicVertexAttribute<AttribValueType, AttribDimension>::SetNewValue(int NewVertexID, const AttribValueType* Data)
	{
		AttribValueType default_val = GetDefaultAttributeValue();
		int k = NewVertexID * AttribDimension;
		for (int i = 0; i < AttribDimension; ++i)
		{
			VectorSetSafe(AttribValues, k + i, Data[i], default_val);
		}
	}

	template<typename AttribValueType, int AttribDimension>
	void TDynamicVertexAttribute<AttribValueType, AttribDimension>::CopyValue(int FromVertexID, int ToVertexID)
	{
		AttribValueType default_val = GetDefaultAttributeValue();
		int kA = FromVertexID * AttribDimension;
		int kB = ToVertexID * AttribDimension;
		for (int i = 0; i < AttribDimension; ++i)
		{
			VectorSetSafe(AttribValues, kB + i, AttribValues[kA + i], default_val);
		}
	}

	// static
	void DynamicMeshAttributeSet::EnableUVChannels(DynamicMesh* Mesh, int NumUVChannels, bool bResetExisting, bool bDisablePrevious)
	{
		Mesh->EnableAttributes();
		if (!(NumUVChannels <= MAX_NUM_UV_CHANNELS))
		{
			NumUVChannels = MAX_NUM_UV_CHANNELS;
		}
		for (int UVIdx = 0; UVIdx < NumUVChannels; UVIdx++)
		{
			if (!bResetExisting && Mesh->Attributes()->HasAttachedAttribute(UVChannelNames[UVIdx]))
			{
				continue;
			}
			Mesh->Attributes()->AttachAttribute(UVChannelNames[UVIdx], new TDynamicMeshVertexAttribute<float, 2>(Mesh));
		}
		if (bDisablePrevious)
		{
			for (int UVIdx = NumUVChannels; UVIdx < MAX_NUM_UV_CHANNELS; UVIdx++)
			{
				Mesh->Attributes()->RemoveAttribute(UVChannelNames[UVIdx]);
			}
		}
	}

	// static
	int DynamicMeshAttributeSet::NumEnabledUVChannels(DynamicMesh* Mesh)
	{
		if (!Mesh->Attributes())
		{
			return 0;
		}
		for (int UVIdx = 0; UVIdx < MAX_NUM_UV_CHANNELS; UVIdx++)
		{
			if (!Mesh->Attributes()->HasAttachedAttribute(UVChannelNames[UVIdx]))
			{
				return UVIdx;
			}
		}
		return MAX_NUM_UV_CHANNELS;
	}

	// static
	void DynamicMeshAttributeSet::Augment(DynamicMesh* Mesh, int NumUVChannels)
	{
		Mesh->EnableVertexNormals(Vector3::UnitZ());
		Mesh->EnableAttributes();
		Mesh->Attributes()->EnableMaterialID();
		Mesh->Attributes()->AttachAttribute(ColorAttribName, new TDynamicMeshVertexAttribute<float, 4>(Mesh));
		Mesh->Attributes()->AttachAttribute(TangentUAttribName, new TDynamicMeshVertexAttribute<float, 3>(Mesh));
		Mesh->Attributes()->AttachAttribute(TangentVAttribName, new TDynamicMeshVertexAttribute<float, 3>(Mesh));
		TDynamicMeshScalarTriangleAttribute<bool>* VisAttrib = new TDynamicMeshScalarTriangleAttribute<bool>(Mesh);
		VisAttrib->Initialize(true);
		Mesh->Attributes()->AttachAttribute(VisibleAttribName, VisAttrib);
		TDynamicMeshScalarTriangleAttribute<bool>* InternalAttrib = new TDynamicMeshScalarTriangleAttribute<bool>(Mesh);
		InternalAttrib->Initialize(true);
		Mesh->Attributes()->AttachAttribute(InternalAttribName, InternalAttrib);

		EnableUVChannels(Mesh, NumUVChannels);
	}

	// static
	void DynamicMeshAttributeSet::AddVertexColorAttribute(DynamicMesh* Mesh)
	{
		Mesh->Attributes()->AttachAttribute(ColorAttribName, new TDynamicMeshVertexAttribute<float, 4>(Mesh));
	}

	// static
	bool DynamicMeshAttributeSet::IsAugmented(const DynamicMesh* Mesh)
	{
		return Mesh->HasAttributes()
			&& Mesh->Attributes()->HasAttachedAttribute(ColorAttribName)
			&& Mesh->Attributes()->HasAttachedAttribute(TangentUAttribName)
			&& Mesh->Attributes()->HasAttachedAttribute(TangentVAttribName)
			&& Mesh->Attributes()->HasAttachedAttribute(VisibleAttribName)
			&& Mesh->Attributes()->HasMaterialID()
			&& Mesh->HasVertexNormals();
	}

	// static
	void DynamicMeshAttributeSet::SetUV(DynamicMesh* Mesh, int VID, Vector2 UV, int UVLayer)
	{
		if (!(UVLayer < MAX_NUM_UV_CHANNELS))
		{
			return;
		}
		TDynamicMeshVertexAttribute<float, 2>* UVs =
			static_cast<TDynamicMeshVertexAttribute<float, 2>*>(Mesh->Attributes()->GetAttachedAttribute(UVChannelNames[UVLayer]));
		if ((UVs))
		{
			UVs->SetValue(VID, UV);
		}
	}

	// static
	void DynamicMeshAttributeSet::SetAllUV(DynamicMesh* Mesh, int VID, Vector2 UV, int NumUVLayers)
	{
		if (!(NumUVLayers <= MAX_NUM_UV_CHANNELS))
		{
			NumUVLayers = MAX_NUM_UV_CHANNELS;
		}
		for (int Layer = 0; Layer < NumUVLayers; Layer++)
		{
			TDynamicMeshVertexAttribute<float, 2>* UVs =
				static_cast<TDynamicMeshVertexAttribute<float, 2>*>(Mesh->Attributes()->GetAttachedAttribute(UVChannelNames[Layer]));
			if (UVs)
			{
				UVs->SetValue(VID, UV);
			}
		}
	}

	// static
	void DynamicMeshAttributeSet::GetUV(const DynamicMesh* Mesh, int VID, Vector2& UV, int UVLayer)
	{
		if (!(UVLayer < MAX_NUM_UV_CHANNELS))
		{
			UV = Vector2::Zero();
			return;
		}
		const TDynamicMeshVertexAttribute<float, 2>* UVs =
			static_cast<const TDynamicMeshVertexAttribute<float, 2>*>(Mesh->Attributes()->GetAttachedAttribute(UVChannelNames[UVLayer]));
		if (UVs)
		{
			UVs->GetValue(VID, UV);
		}
	}

	// static
	void DynamicMeshAttributeSet::SetTangent(DynamicMesh* Mesh, int VID, Vector3 Normal, Vector3 TangentU, Vector3 TangentV)
	{
		assert(IsAugmented(Mesh));
		TDynamicMeshVertexAttribute<float, 3>* Us =
			static_cast<TDynamicMeshVertexAttribute<float, 3>*>(Mesh->Attributes()->GetAttachedAttribute(TangentUAttribName));
		TDynamicMeshVertexAttribute<float, 3>* Vs =
			static_cast<TDynamicMeshVertexAttribute<float, 3>*>(Mesh->Attributes()->GetAttachedAttribute(TangentVAttribName));
		Us->SetValue(VID, TangentU);
		Vs->SetValue(VID, TangentV);
	}

	// static
	void DynamicMeshAttributeSet::GetTangent(const DynamicMesh* Mesh, int VID, Vector3& U, Vector3& V)
	{
		assert(IsAugmented(Mesh));
		const TDynamicMeshVertexAttribute<float, 3>* Us =
			static_cast<const TDynamicMeshVertexAttribute<float, 3>*>(Mesh->Attributes()->GetAttachedAttribute(TangentUAttribName));
		const TDynamicMeshVertexAttribute<float, 3>* Vs =
			static_cast<const TDynamicMeshVertexAttribute<float, 3>*>(Mesh->Attributes()->GetAttachedAttribute(TangentVAttribName));
		// Vector3 Normal = Mesh->GetVertexNormal(VID);
		Us->GetValue(VID, U);
		Vs->GetValue(VID, V);
	}

	// static
	Vector4 DynamicMeshAttributeSet::GetVertexColor(const DynamicMesh* Mesh, int VID)
	{
		assert(IsAugmented(Mesh));
		const TDynamicMeshVertexAttribute<float, 4>* Colors =
			static_cast<const TDynamicMeshVertexAttribute<float, 4>*>(Mesh->Attributes()->GetAttachedAttribute(ColorAttribName));
		Vector4 Color;
		Colors->GetValue(VID, Color);
		return Color;
	}

	// static
	void DynamicMeshAttributeSet::SetVertexColor(DynamicMesh* Mesh, int VID, Vector4 Color)
	{
		assert(IsAugmented(Mesh));
		TDynamicMeshVertexAttribute<float, 4>* Colors =
			static_cast<TDynamicMeshVertexAttribute<float, 4>*>(Mesh->Attributes()->GetAttachedAttribute(ColorAttribName));
		Colors->SetValue(VID, Color);
	}


	template class TDynamicMeshOverlay<float, 2>;
	template class TDynamicMeshOverlay<float, 3>;
	template class TDynamicMeshOverlay<float, 4>;
	template class TDynamicMeshTriangleAttribute<float, 1>;

	template class TDynamicVertexAttribute<float, 1>;
}	// namespace Riemann
