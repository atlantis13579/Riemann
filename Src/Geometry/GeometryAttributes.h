#pragma once

#include <assert.h>
#include <functional>
#include <set>
#include <string>
#include <map>
#include <vector>

#include "GeometryOperation.h"
#include "../Maths/Index2.h"
#include "../Maths/Index3.h"
#include "../Maths/Vector2.h"
#include "../Maths/Vector3.h"
#include "../Maths/Vector4.h"

namespace Riemann
{
	class DynamicMesh;

	class RefCountVector
	{
	public:
		void Decrement(int idx, int count = 1)
		{
			assert(m_refcount[idx] >= count);
			m_refcount[idx] -= count;
		}

		void Increment(int idx, int count = 1)
		{
			m_refcount[idx] += count;
		}

		int GetRefCount(int idx) const
		{
			return m_refcount[idx];
		}

		bool IsValid(int idx) const
		{
			return idx >= 0 && idx < (int)m_refcount.size() && m_refcount[idx] > 0;
		}

		int GetMaxIndex() const
		{
			return (int)m_refcount.size();
		}

		int Allocate()
		{
			int idx = (int)m_refcount.size();
			m_refcount.push_back(1);
			return idx;
		}

		int GetCount() const
		{
			return (int)m_refcount.size();
		}

		void Clear()
		{
			m_refcount.clear();
		}

	private:
		std::vector<int> m_refcount;
	};

	class TDynamicAttributeBase
	{
	public:
		virtual ~TDynamicAttributeBase()
		{
		}

	public:
		const std::string& GetName() const { return Name; }
		void SetName(const std::string& NameIn) { Name = NameIn; }

		virtual void OnNewVertex(int VertexID, bool bInserted) {}
		virtual void OnRemoveVertex(int VertexID) {}
		virtual void OnNewTriangle(int TriangleID, bool bInserted) {}
		virtual void OnRemoveTriangle(int TriangleID) {}
		virtual void OnReverseTriOrientation(int TriangleID) {}
		virtual void OnSplitEdge(const FEdgeSplitInfo& SplitInfo) {}
		virtual void OnFlipEdge(const FEdgeFlipInfo& FlipInfo) {}
		virtual void OnCollapseEdge(const FEdgeCollapseInfo& CollapseInfo) {}
		virtual void OnPokeTriangle(const FPokeTriangleInfo& PokeInfo) {}
		virtual void OnMergeEdges(const FMergeEdgesInfo& MergeInfo) {}
		virtual void OnSplitVertex(const FVertexSplitInfo& SplitInfo, const std::vector<int>& TrianglesToUpdate) {}

	protected:
		std::string Name;
	};

	template<typename RealType, int ElementSize>
	class TDynamicMeshOverlay
	{

	protected:
		DynamicMesh* ParentMesh;
		std::vector<RealType> Elements;
		std::vector<int> ParentVertices;
		std::vector<int> ElementTriangles;
		RefCountVector	ElementsRefCounts;

		friend class DynamicMesh;
		friend class DynamicMeshAttributeSet;

	public:
		TDynamicMeshOverlay()
		{
			ParentMesh = nullptr;
		}

		TDynamicMeshOverlay(DynamicMesh* ParentMeshIn)
		{
			ParentMesh = ParentMeshIn;
		}
	private:
		void Reparent(DynamicMesh* ParentMeshIn)
		{
			ParentMesh = ParentMeshIn;
		}
	public:
		const DynamicMesh* GetParentMesh() const { return ParentMesh; }
		DynamicMesh* GetParentMesh() { return ParentMesh; }

		void InitializeTriangles(int MaxTriangleID)
		{
			ElementTriangles.resize(MaxTriangleID * 3, -1);
		}

		/*
		void Copy(const TDynamicMeshOverlay<RealType, ElementSize>& Copy)
		{
			ElementsRefCounts = FRefCountVector(Copy.ElementsRefCounts);
			Elements = Copy.Elements;
			ParentVertices = Copy.ParentVertices;
			ElementTriangles = Copy.ElementTriangles;
		}

		void CompactCopy(const FCompactMaps& CompactMaps, const TDynamicMeshOverlay<RealType, ElementSize>& Copy)
		{
			ClearElements();

			// map of element IDs
			std::vector<int> MapE; MapE.SetNumUninitialized(Copy.MaxElementID());

			// copy elements across
			RealType Data[ElementSize];
			for (int EID = 0; EID < Copy.MaxElementID(); EID++)
			{
				if (Copy.IsElement(EID))
				{
					Copy.GetElement(EID, Data);
					MapE[EID] = AppendElement(Data);
				}
				else
				{
					MapE[EID] = -1;
				}
			}

			// copy triangles across
			assert(CompactMaps.NumTriangleMappings() == Copy.GetParentMesh()->MaxTriangleID()); // must have valid triangle map
			for (int FromTID : Copy.GetParentMesh()->TriangleIndicesItr())
			{
				if (!Copy.IsSetTriangle(FromTID))
				{
					continue;
				}
				const int ToTID = CompactMaps.GetTriangleMapping(FromTID);
				Index3 FromTriElements = Copy.GetTriangle(FromTID);
				SetTriangle(ToTID, Index3(MapE[FromTriElements.a], MapE[FromTriElements.b], MapE[FromTriElements.c]));
			}
		}

		void CompactInPlace(const FCompactMaps& CompactMaps)
		{
			int iLastE = MaxElementID() - 1, iCurE = 0;
			while (iLastE >= 0 && ElementsRefCounts.IsValidUnsafe(iLastE) == false)
			{
				iLastE--;
			}
			while (iCurE < iLastE && ElementsRefCounts.IsValidUnsafe(iCurE))
			{
				iCurE++;
			}

			std::vector<int> MapE; MapE.SetNumUninitialized(MaxElementID());
			for (int ID = 0; ID < MapE.size(); ID++)
			{
				// mapping is 1:1 by default; sparsely re-mapped below
				MapE[ID] = ID;
				// remap all parents
				if (ParentVertices[ID] >= 0)
				{
					ParentVertices[ID] = CompactMaps.GetVertexMapping(ParentVertices[ID]);
				}
			}

			std::vector<unsigned short>& ERef = ElementsRefCounts.GetRawRefCountsUnsafe();
			RealType Data[ElementSize];
			while (iCurE < iLastE)
			{
				// remap the element data
				GetElement(iLastE, Data);
				SetElement(iCurE, Data);
				ParentVertices[iCurE] = ParentVertices[iLastE];
				ERef[iCurE] = ERef[iLastE];
				ERef[iLastE] = FRefCountVector::INVALID_REF_COUNT;
				MapE[iLastE] = iCurE;

				// move cur forward one, last back one, and  then search for next valid
				iLastE--; iCurE++;
				while (iLastE >= 0 && ElementsRefCounts.IsValidUnsafe(iLastE) == false)
				{
					iLastE--;
				}
				while (iCurE < iLastE && ElementsRefCounts.IsValidUnsafe(iCurE))
				{
					iCurE++;
				}
			}
			ElementsRefCounts.Trim(ElementCount());
			Elements.Resize(ElementCount() * ElementSize);
			ParentVertices.Resize(ElementCount());

			// Remap and compact triangle element indices.
			int MaxNewTID = 0;
			for (int TID = 0, OldMaxTID = ElementTriangles.size() / 3; TID < OldMaxTID; TID++)
			{
				const int OldStart = TID * 3;
				const int NewTID = CompactMaps.GetTriangleMapping(TID);
				if (NewTID == -1)
				{
					// skip if there's no mapping
					continue;
				}

				MaxNewTID = FMath::Max(NewTID, MaxNewTID);

				const int NewStart = NewTID * 3;
				if (ElementTriangles[OldStart] == -1)
				{
					// triangle was not set; copy back InvalidID
					for (int SubIdx = 0; SubIdx < 3; SubIdx++)
					{
						ElementTriangles[NewStart + SubIdx] = -1;
					}
				}
				else
				{
					for (int SubIdx = 0; SubIdx < 3; SubIdx++)
					{
						ElementTriangles[NewStart + SubIdx] = MapE[ElementTriangles[OldStart + SubIdx]];
					}
				}
			}

			assert(ElementTriangles.size() >= (MaxNewTID + 1) * 3);
			ElementTriangles.Resize((MaxNewTID + 1) * 3);

			assert(IsCompact());
		}

		bool IsCompact() const { return ElementsRefCounts.IsDense(); }

		typedef typename FRefCountVector::IndexEnumerable element_iterator;

		element_iterator ElementIndicesItr() const { return ElementsRefCounts.Indices(); }


		void FreeUnusedElements(const TSet<int>* ElementsToCheck = nullptr);


		void CreateFromPredicate(std::function<bool(int ParentVertexIdx, int TriIDA, int TriIDB)> TrisCanShareVertexPredicate, RealType InitElementValue);

		void SplitVerticesWithPredicate(std::function<bool(int ElementIdx, int TriID)> ShouldSplitOutVertex, std::function<void(int ElementIdx, int TriID, RealType* FillVect)> GetNewElementValue);

		bool MergeElement(int SourceElementID, int TargetElementID);

		int SplitElement(int ElementID, const std::vector<const int>& TrianglesToUpdate);


		void SplitBowtiesAtVertex(int Vid, std::vector<int>* NewElementIDs = nullptr);

		void SplitBowties();

		EMeshResult InsertElement(int ElementID, const RealType* Value, bool bUnsafe = false);

		*/

		void SetParentVertex(int ElementIndex, int ParentVertexIndex)
		{
			ParentVertices[ElementIndex] = ParentVertexIndex;
		}

		EMeshResult SetTriangle(int tid, const Index3& tv, bool bAllowElementFreeing = true);

		void ClearElements();

		void ClearElements(const std::vector<int>& Triangles)
		{
			for (int TriID : Triangles)
			{
				UnsetTriangle(TriID);
			}
		}

		int MaxElementID() const { return (int)ElementsRefCounts.GetMaxIndex(); }

		inline bool IsElement(int vID) const { return ElementsRefCounts.IsValid(vID); }

		int ElementCount() const { return (int)ElementsRefCounts.GetCount(); }

		int AppendElement(RealType ConstantValue);

		int AppendElement(const RealType* Value);

		int SplitElementWithNewParent(int ElementID, int NewParentID, const std::vector<int>& TrianglesToUpdate);


		bool IsSetTriangle(int TID) const
		{
			bool bIsSet = ElementTriangles[3 * TID] >= 0;
			// we require that triangle elements either be all set or all unset
			assert(ElementTriangles[3 * TID + 1] >= 0 == bIsSet);
			assert(ElementTriangles[3 * TID + 2] >= 0 == bIsSet);
			return bIsSet;
		}

		void UnsetTriangle(int TriangleID, bool bAllowElementFreeing = true)
		{
			int i = 3 * TriangleID;
			if (ElementTriangles[i] == -1)
			{
				return;
			}
			for (int SubIdx = 0; SubIdx < 3; SubIdx++)
			{
				ElementsRefCounts.Decrement(ElementTriangles[i + SubIdx]);

				if (bAllowElementFreeing && ElementsRefCounts.GetRefCount(ElementTriangles[i + SubIdx]) == 1)
				{
					ElementsRefCounts.Decrement(ElementTriangles[i + SubIdx]);
					ParentVertices[ElementTriangles[i + SubIdx]] = -1;
				}
				ElementTriangles[i + SubIdx] = -1;
			}
		}

		inline void GetElement(int ElementID, RealType* Data) const
		{
			int k = ElementID * ElementSize;
			for (int i = 0; i < ElementSize; ++i)
			{
				Data[i] = Elements[k + i];
			}
		}

		inline int GetParentVertex(int ElementID) const
		{
			return ParentVertices[ElementID];
		}

		inline Index3 GetTriangle(int TriangleID) const
		{
			int i = 3 * TriangleID;
			return Index3(ElementTriangles[i], ElementTriangles[i + 1], ElementTriangles[i + 2]);
		}

		inline bool GetTriangleIfValid(int TriangleID, Index3& TriangleOut) const
		{
			int i = 3 * TriangleID;
			int a = ElementTriangles[i];
			if (a >= 0)
			{
				TriangleOut = Index3(a, ElementTriangles[i + 1], ElementTriangles[i + 2]);
				assert(TriangleOut.b >= 0 && TriangleOut.c >= 0);
				return true;
			}
			return false;
		}

		inline void SetElement(int ElementID, const RealType* Data)
		{
			int k = ElementID * ElementSize;
			for (int i = 0; i < ElementSize; ++i)
			{
				Elements[k + i] = Data[i];
			}
		}

		template<typename AsType>
		void SetElement(int ElementID, const AsType& Data)
		{
			int k = ElementID * ElementSize;
			for (int i = 0; i < ElementSize; ++i)
			{
				Elements[k + i] = Data[i];
			}
		}

		inline bool TriangleHasElement(int TriangleID, int ElementID) const
		{
			int i = 3 * TriangleID;
			return (ElementTriangles[i] == ElementID || ElementTriangles[i + 1] == ElementID || ElementTriangles[i + 2] == ElementID);
		}

		bool IsSeamEdge(int EdgeID, bool* bIsNonIntersectingOut = nullptr) const;

		bool IsSeamEndEdge(int EdgeID) const;

		bool IsSeamVertex(int VertexID, bool bBoundaryIsSeam = true) const;

		bool HasInteriorSeamEdges() const;

		//bool IsBowtieInOverlay(int VertexID) const;

		//bool AreTrianglesConnected(int TriangleID0, int TriangleID1) const;

		//void GetVertexElements(int VertexID, std::vector<int>& OutElements) const;

		//void GetElementTriangles(int ElementID, std::vector<int>& OutTriangles) const;

		int CountVertexElements(int vid, bool bBruteForce = false) const;

		int GetElementIDAtVertex(int TriangleID, int VertexID) const
		{
			Index3 Triangle = GetTriangle(TriangleID);
			for (int IDX = 0; IDX < 3; ++IDX)
			{
				int ElementID = Triangle[IDX];
				if (ParentVertices[ElementID] == VertexID)
				{
					return ElementID;
				}
			}

			assert(false);
			return -1;
		}

		template<typename AsType>
		void GetTriBaryInterpolate(int TriangleID, const AsType* BaryCoords, AsType* DataOut) const
		{
			int TriIndex = 3 * TriangleID;
			int ElemIndex0 = ElementTriangles[TriIndex] * ElementSize;
			int ElemIndex1 = ElementTriangles[TriIndex + 1] * ElementSize;
			int ElemIndex2 = ElementTriangles[TriIndex + 2] * ElementSize;
			const AsType Bary0 = (AsType)BaryCoords[0], Bary1 = (AsType)BaryCoords[1], Bary2 = (AsType)BaryCoords[2];
			for (int i = 0; i < ElementSize; ++i)
			{
				DataOut[i] = Bary0 * (AsType)Elements[ElemIndex0 + i] + Bary1 * (AsType)Elements[ElemIndex1 + i] + Bary2 * (AsType)Elements[ElemIndex2 + i];
			}
		}

	public:
		void InitializeNewTriangle(int tid);

		void OnRemoveTriangle(int TriangleID);

		void OnReverseTriOrientation(int TriangleID);

		void OnSplitEdge(const FEdgeSplitInfo& splitInfo);

		void OnFlipEdge(const FEdgeFlipInfo& FlipInfo);

		void OnCollapseEdge(const FEdgeCollapseInfo& collapseInfo);

		void OnPokeTriangle(const FPokeTriangleInfo& PokeInfo);

		void OnMergeEdges(const FMergeEdgesInfo& MergeInfo);

		void OnSplitVertex(const FVertexSplitInfo& SplitInfo, const std::vector<int>& TrianglesToUpdate);

	protected:
		void SetElementFromLerp(int SetElement, int ElementA, int ElementB, double Alpha);
		void SetElementFromBary(int SetElement, int ElementA, int ElementB, int ElementC, const Vector3& BaryCoords);
		void InternalSetTriangle(int TriangleID, const Index3& TriElements, bool bUpdateRefCounts, bool bAllowElementFreeing = true);
	};

	template<typename RealType, int ElementSize, typename VectorType>
	class TDynamicMeshVectorOverlay : public TDynamicMeshOverlay<RealType, ElementSize>
	{
	public:
		using BaseType = TDynamicMeshOverlay<RealType, ElementSize>;

		TDynamicMeshVectorOverlay()
			: TDynamicMeshOverlay<RealType, ElementSize>()
		{
		}

		TDynamicMeshVectorOverlay(DynamicMesh* parentMesh)
			: TDynamicMeshOverlay<RealType, ElementSize>(parentMesh)
		{
		}

		inline int AppendElement(const VectorType& Value)
		{
			return BaseType::AppendElement((RealType*)&Value);
		}

		inline int AppendElement(const RealType* Value)
		{
			return BaseType::AppendElement(Value);
		}

		inline VectorType GetElement(int ElementID) const
		{
			VectorType V;
			BaseType::GetElement(ElementID, (RealType*)&V);
			return V;
		}

		inline void GetElement(int ElementID, VectorType& V) const
		{
			BaseType::GetElement(ElementID, V);
		}

		inline VectorType GetElementAtVertex(int TriangleID, int VertexID) const
		{
			VectorType V;
			BaseType::GetElementAtVertex(TriangleID, VertexID, V);
			return V;
		}

		inline void GetElementAtVertex(int TriangleID, int VertexID, VectorType& V) const
		{
			BaseType::GetElementAtVertex(TriangleID, VertexID, V);
		}

		inline void GetTriElement(int TriangleID, int TriVertexIndex, VectorType& Value) const
		{
			assert(TriVertexIndex >= 0 && TriVertexIndex <= 2);
			GetElement(BaseType::ElementTriangles[(3 * TriangleID) + TriVertexIndex], Value);
		}

		inline void GetTriElements(int TriangleID, VectorType& A, VectorType& B, VectorType& C) const
		{
			int i = 3 * TriangleID;
			GetElement(BaseType::ElementTriangles[i], A);
			GetElement(BaseType::ElementTriangles[i + 1], B);
			GetElement(BaseType::ElementTriangles[i + 2], C);
		}

		inline void SetElement(int ElementID, const VectorType& Value)
		{
			BaseType::SetElement(ElementID, Value);
		}

		//bool EnumerateVertexElements(
		//	int VertexID,
		//	std::function<bool(int TriangleID, int ElementID, const VectorType& Value)> ProcessFunc,
		//	bool bFindUniqueElements = true) const;
	};

	template<typename AttribValueType, int AttribDimension>
	class TDynamicMeshTriangleAttribute : public TDynamicAttributeBase
	{

	protected:
		DynamicMesh* ParentMesh;

		std::vector<AttribValueType> AttribValues;

		friend class DynamicMesh;
		friend class DynamicMeshAttributeSet;

	public:
		TDynamicMeshTriangleAttribute()
		{
			ParentMesh = nullptr;
		}

		TDynamicMeshTriangleAttribute(DynamicMesh* ParentMeshIn, bool bAutoInit = true)
		{
			ParentMesh = ParentMeshIn;
			if (bAutoInit)
			{
				// Initialize();
			}
		}

		AttribValueType GetDefaultAttributeValue() const { return (AttribValueType)0; }

	private:
		void Reparent(DynamicMesh* ParentMeshIn)
		{
			ParentMesh = ParentMeshIn;
		}

	public:
		const DynamicMesh* GetParentMesh() const { return ParentMesh; }
		DynamicMesh* GetParentMesh() { return ParentMesh; }

		void Initialize(AttribValueType InitialValue = (AttribValueType)0)
		{
			//assert(ParentMesh != nullptr);
			//AttribValues.resize(ParentMesh->GetTriangleCount() * AttribDimension, InitialValue);
		}

		void SetNewValue(int NewTriangleID, const AttribValueType* Data);

		/*
		virtual FDynamicMeshAttributeBase* MakeNew(DynamicMesh* ParentMeshIn) const
		{
			TDynamicMeshTriangleAttribute<AttribValueType, AttribDimension>* Matching = new TDynamicMeshTriangleAttribute<AttribValueType, AttribDimension>(ParentMeshIn);
			Matching->Initialize();
			return Matching;
		}

		virtual FDynamicMeshAttributeBase* MakeCopy(DynamicMesh* ParentMeshIn) const
		{
			TDynamicMeshTriangleAttribute<AttribValueType, AttribDimension>* ToFill = new TDynamicMeshTriangleAttribute<AttribValueType, AttribDimension>(ParentMeshIn);
			ToFill->Copy(*this);
			return ToFill;
		}

		void Copy(const TDynamicMeshTriangleAttribute<AttribValueType, AttribDimension>& Copy)
		{
			CopyParentClassData(Copy);
			AttribValues = Copy.AttribValues;
		}

		virtual FDynamicMeshAttributeBase* MakeCompactCopy(const FCompactMaps& CompactMaps, DynamicMesh* ParentMeshIn) const
		{
			TDynamicMeshTriangleAttribute<AttribValueType, AttribDimension>* ToFill = new TDynamicMeshTriangleAttribute<AttribValueType, AttribDimension>(ParentMeshIn);
			ToFill->Initialize();
			ToFill->CompactCopy(CompactMaps, *this);
			return ToFill;
		}

		void CompactInPlace(const FCompactMaps& CompactMaps)
		{
			for (int TID = 0, NumTID = CompactMaps.NumTriangleMappings(); TID < NumTID; TID++)
			{
				const int ToTID = CompactMaps.GetTriangleMapping(TID);
				if (ToTID == FCompactMaps::InvalidID)
				{
					continue;
				}
				if (ensure(ToTID <= TID))
				{
					CopyValue(TID, ToTID);
				}
			}
			AttribValues.Resize(ParentMesh->MaxTriangleID() * AttribDimension);
		}

		void CompactCopy(const FCompactMaps& CompactMaps, const TDynamicMeshTriangleAttribute<AttribValueType, AttribDimension>& ToCopy)
		{
			CopyParentClassData(ToCopy);
			assert(CompactMaps.NumTriangleMappings() <= int(ToCopy.AttribValues.Num() / AttribDimension));
			AttribValueType Data[AttribDimension];
			for (int TID = 0, NumTID = CompactMaps.NumTriangleMappings(); TID < NumTID; TID++)
			{
				const int ToTID = CompactMaps.GetTriangleMapping(TID);
				if (ToTID == FCompactMaps::InvalidID)
				{
					continue;
				}
				ToCopy.GetValue(TID, Data);
				SetValue(ToTID, Data);
			}
		}

		virtual bool CopyThroughMapping(const TDynamicAttributeBase<DynamicMesh>* Source, const FMeshIndexMappings& Mapping)
		{
			AttribValueType BufferData[AttribDimension];
			int BufferSize = sizeof(BufferData);
			for (const TPair<int, int>& MapTID : Mapping.GetTriangleMap().GetForwardMap())
			{
				if (!(Source->CopyOut(MapTID.Key, BufferData, BufferSize)))
				{
					return false;
				}
				SetValue(MapTID.Value, BufferData);
			}
			return true;
		}
		*/

		bool CopyOut(int RawID, void* Buffer, int BufferSize) const
		{
			if (sizeof(AttribValueType) * AttribDimension != BufferSize)
			{
				return false;
			}
			AttribValueType* BufferData = static_cast<AttribValueType*>(Buffer);
			int k = RawID * AttribDimension;
			for (int i = 0; i < AttribDimension; ++i)
			{
				BufferData[i] = AttribValues[k + i];
			}
			return true;
		}

		bool CopyIn(int RawID, void* Buffer, int BufferSize)
		{
			if (sizeof(AttribValueType) * AttribDimension != BufferSize)
			{
				return false;
			}
			AttribValueType* BufferData = static_cast<AttribValueType*>(Buffer);
			int k = RawID * AttribDimension;
			for (int i = 0; i < AttribDimension; ++i)
			{
				AttribValues[k + i] = BufferData[i];
			}
			return true;
		}

		inline void GetValue(int TriangleID, AttribValueType* Data) const
		{
			int k = TriangleID * AttribDimension;
			for (int i = 0; i < AttribDimension; ++i)
			{
				Data[i] = AttribValues[k + i];
			}
		}

		void SetValue(int TriangleID, const AttribValueType* Data);

		void CopyValue(int FromTriangleID, int ToTriangleID);

		bool IsBorderEdge(int EdgeID, bool bMeshBoundaryIsBorder = true) const;

		inline void ResizeAttribStoreIfNeeded(int TriangleID)
		{
			if (!(TriangleID >= 0))
			{
				return;
			}
			size_t NeededSize = (((size_t)TriangleID + 1) * AttribDimension);
			if (NeededSize > AttribValues.size())
			{
				AttribValues.resize(NeededSize, (AttribValueType)0);
			}
		}

	public:
		virtual void OnSplitEdge(const FEdgeSplitInfo& SplitInfo) override
		{
			CopyValue(SplitInfo.OriginalTriangles.a, SplitInfo.NewTriangles.a);
			if (SplitInfo.OriginalTriangles.b != -1)
			{
				CopyValue(SplitInfo.OriginalTriangles.b, SplitInfo.NewTriangles.b);
			}
		}

		virtual void OnFlipEdge(const FEdgeFlipInfo& FlipInfo) override
		{
			// yikes! triangles did not actually change so we will leave attrib unmodified
		}

		virtual void OnCollapseEdge(const FEdgeCollapseInfo& CollapseInfo) override
		{
			// nothing to do here, triangles were only deleted
		}

		virtual void OnPokeTriangle(const FPokeTriangleInfo& PokeInfo) override
		{
			CopyValue(PokeInfo.OriginalTriangle, PokeInfo.NewTriangles.a);
			CopyValue(PokeInfo.OriginalTriangle, PokeInfo.NewTriangles.b);
		}

		virtual void OnMergeEdges(const FMergeEdgesInfo& MergeInfo) override
		{
			// nothing to do here because triangles did not change
		}

		virtual void OnSplitVertex(const FVertexSplitInfo& SplitInfo, const std::vector<int>& TrianglesToUpdate) override
		{
			// nothing to do here because triangles did not change
		}

		virtual void OnNewTriangle(int TriangleID, bool bInserted) override
		{
			ResizeAttribStoreIfNeeded(TriangleID);
		}
	};


	template<typename RealType>
	class TDynamicMeshScalarTriangleAttribute : public TDynamicMeshTriangleAttribute<RealType, 1>
	{
	public:
		using BaseType = TDynamicMeshTriangleAttribute<RealType, 1>;
		using BaseType::SetNewValue;

		TDynamicMeshScalarTriangleAttribute() : BaseType()
		{
		}

		TDynamicMeshScalarTriangleAttribute(DynamicMesh* ParentMeshIn) : BaseType(ParentMeshIn)
		{
		}

		inline void SetNewValue(int NewTriangleID, RealType Value)
		{
			if (NewTriangleID >= (int)this->AttribValues.size())
			{
				this->AttribValues.resize(NewTriangleID + 1, 0);
			}
			this->AttribValues[NewTriangleID] = Value;
		}

		inline RealType GetValue(int TriangleID) const
		{
			return this->AttribValues[TriangleID];
		}

		inline void SetValue(int TriangleID, RealType Value)
		{
			this->AttribValues[TriangleID] = Value;
		}
	};

	template<typename AttribValueType, int AttribDimension>
	class TDynamicVertexAttribute : public TDynamicAttributeBase
	{
	protected:
		DynamicMesh* Parent;

		std::vector<AttribValueType> AttribValues;

		friend class DynamicMeshAttributeSet;

	public:
		TDynamicVertexAttribute() : Parent(nullptr)
		{
		}

		TDynamicVertexAttribute(DynamicMesh* ParentIn, bool bAutoInit = true) : Parent(ParentIn)
		{
			if (bAutoInit)
			{
				Initialize();
			}
		}

		~TDynamicVertexAttribute()
		{
		}

		AttribValueType GetDefaultAttributeValue() const { return (AttribValueType)0; }

		const DynamicMesh* GetParent() const { return Parent; }
		DynamicMesh* GetParent() { return Parent; }
	private:
		void Reparent(DynamicMesh* NewParent) { Parent = NewParent; }
	public:
		/*
		virtual TDynamicAttributeBase<ParentType>* MakeNew(ParentType* ParentIn) const
		{
			TDynamicVertexAttribute<AttribValueType, AttribDimension, ParentType>* Matching = new TDynamicVertexAttribute<AttribValueType, AttribDimension, ParentType>(ParentIn);
			Matching->Initialize();
			return Matching;
		}
		virtual TDynamicAttributeBase<ParentType>* MakeCopy(ParentType* ParentIn) const
		{
			TDynamicVertexAttribute<AttribValueType, AttribDimension, ParentType>* ToFill = new TDynamicVertexAttribute<AttribValueType, AttribDimension, ParentType>(ParentIn);
			ToFill->Copy(*this);
			return ToFill;
		}

		void Copy(const TDynamicVertexAttribute<AttribValueType, AttribDimension, ParentType>& Copy)
		{
			TDynamicAttributeBase<ParentType>::CopyParentClassData(Copy);
			AttribValues = Copy.AttribValues;
		}

		virtual TDynamicAttributeBase<ParentType>* MakeCompactCopy(const FCompactMaps& CompactMaps, ParentType* ParentTypeIn) const
		{
			TDynamicVertexAttribute<AttribValueType, AttribDimension, ParentType>* ToFill = new TDynamicVertexAttribute<AttribValueType, AttribDimension, ParentType>(ParentTypeIn);
			ToFill->Initialize();
			ToFill->CompactCopy(CompactMaps, *this);
			return ToFill;
		}

		void CompactInPlace(const FCompactMaps& CompactMaps)
		{
			for (int VID = 0, NumVID = CompactMaps.NumVertexMappings(); VID < NumVID; VID++)
			{
				const int ToVID = CompactMaps.GetVertexMapping(VID);
				if (ToVID == FCompactMaps::InvalidID)
				{
					continue;
				}
				if (ensure(ToVID <= VID))
				{
					CopyValue(VID, ToVID);
				}
			}
			AttribValues.Resize(Parent->MaxVertexID() * AttribDimension);
		}

		void CompactCopy(const FCompactMaps& CompactMaps, const TDynamicVertexAttribute<AttribValueType, AttribDimension, ParentType>& ToCopy)
		{
			TDynamicAttributeBase<ParentType>::CopyParentClassData(ToCopy);
			check(CompactMaps.NumVertexMappings() <= int(ToCopy.AttribValues.Num() / AttribDimension));

			AttribValueType Data[AttribDimension];
			for (int VID = 0, NumVID = CompactMaps.NumVertexMappings(); VID < NumVID; VID++)
			{
				int ToVID = CompactMaps.GetVertexMapping(VID);
				if (ToVID == FCompactMaps::InvalidID)
				{
					continue;
				}
				ToCopy.GetValue(VID, Data);
				SetValue(ToVID, Data);
			}
		}

		bool CopyThroughMapping(const TDynamicAttributeBase<DynamicMesh>* Source, const FMeshIndexMappings& Mapping)
		{
			AttribValueType BufferData[AttribDimension];
			int BufferSize = sizeof(BufferData);
			for (const TPair<int, int>& MapVID : Mapping.GetVertexMap().GetForwardMap())
			{
				if (!(Source->CopyOut(MapVID.Key, BufferData, BufferSize)))
				{
					return false;
				}
				SetValue(MapVID.Value, BufferData);
			}
			return true;
		}
		*/

		void Initialize(AttribValueType InitialValue = (AttribValueType)0)
		{
            //assert(Parent != nullptr);
			//AttribValues.resize(0);
			//AttribValues.resize(Parent->GetVertexCount() * AttribDimension, InitialValue);
		}

		void SetNewValue(int NewVertexID, const AttribValueType* Data);

		bool CopyOut(int RawID, void* Buffer, int BufferSize) const
		{
			if (sizeof(AttribValueType) * AttribDimension != BufferSize)
			{
				return false;
			}
			AttribValueType* BufferData = static_cast<AttribValueType*>(Buffer);
			int k = RawID * AttribDimension;
			for (int i = 0; i < AttribDimension; ++i)
			{
				BufferData[i] = AttribValues[k + i];
			}
			return true;
		}

		bool CopyIn(int RawID, void* Buffer, int BufferSize)
		{
			if (sizeof(AttribValueType) * AttribDimension != BufferSize)
			{
				return false;
			}
			AttribValueType* BufferData = static_cast<AttribValueType*>(Buffer);
			int k = RawID * AttribDimension;
			for (int i = 0; i < AttribDimension; ++i)
			{
				AttribValues[k + i] = BufferData[i];
			}
			return true;
		}

		inline void GetValue(int VertexID, AttribValueType* Data) const
		{
			int k = VertexID * AttribDimension;
			for (int i = 0; i < AttribDimension; ++i)
			{
				Data[i] = AttribValues[k + i];
			}
		}

		template<typename AsType>
		void GetValue(int VertexID, AsType& Data) const
		{
			int k = VertexID * AttribDimension;
			for (int i = 0; i < AttribDimension; ++i)
			{
				Data[i] = AttribValues[k + i];
			}
		}

		inline void SetValue(int VertexID, const AttribValueType* Data)
		{
			int k = VertexID * AttribDimension;
			for (int i = 0; i < AttribDimension; ++i)
			{
				AttribValues[k + i] = Data[i];
			}
		}

		template<typename AsType>
		void SetValue(int VertexID, const AsType& Data)
		{
			int k = VertexID * AttribDimension;
			for (int i = 0; i < AttribDimension; ++i)
			{
				AttribValues[k + i] = Data[i];
			}
		}

		void CopyValue(int FromVertexID, int ToVertexID);

		AttribValueType GetDefaultAttributeValue()
		{
			return (AttribValueType)0;
		}

		inline void ResizeAttribStoreIfNeeded(int VertexID)
		{
			if (!(VertexID >= 0))
			{
				return;
			}
			size_t NeededSize = (size_t(VertexID) + 1) * AttribDimension;
			if (NeededSize > AttribValues.size())
			{
				AttribValues.resize(NeededSize, GetDefaultAttributeValue());
			}
		}

	public:
		virtual void OnSplitEdge(const FEdgeSplitInfo& SplitInfo) override
		{
			ResizeAttribStoreIfNeeded(SplitInfo.NewVertex);
			SetAttributeFromLerp(SplitInfo.NewVertex, SplitInfo.OriginalVertices.a, SplitInfo.OriginalVertices.b, SplitInfo.SplitT);
		}

		virtual void OnFlipEdge(const FEdgeFlipInfo& FlipInfo) override
		{
			// vertices unchanged
		}

		virtual void OnCollapseEdge(const FEdgeCollapseInfo& CollapseInfo) override
		{
			SetAttributeFromLerp(CollapseInfo.KeptVertex, CollapseInfo.KeptVertex, CollapseInfo.RemovedVertex, CollapseInfo.CollapseT);
		}

		virtual void OnNewVertex(int VertexID, bool bInserted) override
		{
			ResizeAttribStoreIfNeeded(VertexID);
		}

		virtual void OnPokeTriangle(const FPokeTriangleInfo& PokeInfo) override
		{
			Index3 Tri = PokeInfo.TriVertices;
			ResizeAttribStoreIfNeeded(PokeInfo.NewVertex);
			SetAttributeFromBary(PokeInfo.NewVertex, Tri.a, Tri.b, Tri.c, PokeInfo.BaryCoords);
		}

		virtual void OnMergeEdges(const FMergeEdgesInfo& MergeInfo) override
		{
			if (MergeInfo.RemovedVerts.a != -1)
			{
				SetAttributeFromLerp(MergeInfo.KeptVerts.a, MergeInfo.KeptVerts.a, MergeInfo.RemovedVerts.a, .5);
			}
			if (MergeInfo.RemovedVerts.b != -1)
			{
				SetAttributeFromLerp(MergeInfo.KeptVerts.b, MergeInfo.KeptVerts.b, MergeInfo.RemovedVerts.b, .5);
			}
		}

		virtual void OnSplitVertex(const FVertexSplitInfo& SplitInfo, const std::vector<int>& TrianglesToUpdate) override
		{
			CopyValue(SplitInfo.OriginalVertex, SplitInfo.NewVertex);
		}

	protected:
		virtual void SetAttributeFromLerp(int SetAttribute, int AttributeA, int AttributeB, double Alpha)
		{
			int IndexSet = AttribDimension * SetAttribute;
			int IndexA = AttribDimension * AttributeA;
			int IndexB = AttribDimension * AttributeB;
			double Beta = (1. - Alpha);
			for (int i = 0; i < AttribDimension; ++i)
			{
				AttribValues[IndexSet + i] = AttribValueType(Beta * AttribValues[IndexA + i] + Alpha * AttribValues[IndexB + i]);
			}
		}

		virtual void SetAttributeFromBary(int SetAttribute, int AttributeA, int AttributeB, int AttributeC, const Vector3& BaryCoords)
		{
			int IndexSet = AttribDimension * SetAttribute;
			int IndexA = AttribDimension * AttributeA;
			int IndexB = AttribDimension * AttributeB;
			int IndexC = AttribDimension * AttributeC;
			for (int i = 0; i < AttribDimension; ++i)
			{
				AttribValues[IndexSet + i] = AttribValueType(
					BaryCoords.x * AttribValues[IndexA + i] + BaryCoords.y * AttribValues[IndexB + i] + BaryCoords.z * AttribValues[IndexC + i]);
			}
		}
	};


	typedef TDynamicMeshVectorOverlay<float, 2, Vector2>	FDynamicMeshUVOverlay;
	typedef TDynamicMeshVectorOverlay<float, 3, Vector3>	FDynamicMeshNormalOverlay;
	typedef TDynamicMeshVectorOverlay<float, 4, Vector4>	FDynamicMeshColorOverlay;
	typedef TDynamicMeshScalarTriangleAttribute<int>		FDynamicMeshMaterialAttribute;

	template<typename AttribValueType, int AttribDimension>
	using TDynamicMeshVertexAttribute = TDynamicVertexAttribute<AttribValueType, AttribDimension>;

	typedef TDynamicAttributeBase FDynamicMeshAttributeBase;
	typedef TDynamicMeshVertexAttribute<float, 1> FDynamicMeshWeightAttribute;

	class DynamicMeshAttributeSet
	{
		DynamicMeshAttributeSet(DynamicMesh* Mesh);

		DynamicMeshAttributeSet(DynamicMesh* Mesh, int NumUVLayers, int NumNormalLayers);

		~DynamicMeshAttributeSet();

		const DynamicMesh* GetParentMesh() const { return ParentMesh; }

		DynamicMesh* GetParentMesh() { return ParentMesh; }

	private:
		void Reparent(DynamicMesh* NewParent) { ParentMesh = NewParent; }

	public:

		bool IsSeamEdge(int EdgeID) const;

		bool IsSeamEndEdge(int EdgeID) const;

		bool IsSeamEdge(int EdgeID, bool& bIsUVSeamOut, bool& bIsNormalSeamOut, bool& bIsColorSeamOut, bool& bIsTangentSeamOut) const;

		bool IsSeamVertex(int VertexID, bool bBoundaryIsSeam = true) const;

		bool IsMaterialBoundaryEdge(int EdgeID) const;

		bool HasMaterialID() const
		{
			return !!MaterialIDAttrib;
		}

		FDynamicMeshMaterialAttribute* GetMaterialID()
		{
			return MaterialIDAttrib;
		}

		const FDynamicMeshMaterialAttribute* GetMaterialID() const
		{
			return MaterialIDAttrib;
		}

		void EnableMaterialID();

		void DisableMaterialID();

		int NumUVLayers() const
		{
			return (int)UVLayers.size();
		}

		void SetNumUVLayers(int Num);

		FDynamicMeshUVOverlay* GetUVLayer(int Index)
		{
			if (Index < UVLayers.size() && Index > -1)
			{
				return &UVLayers[Index];
			}
			return nullptr;
		}

		const FDynamicMeshUVOverlay* GetUVLayer(int Index) const
		{
			if (Index < UVLayers.size() && Index > -1)
			{
				return &UVLayers[Index];
			}
			return nullptr;
		}

		FDynamicMeshUVOverlay* PrimaryUV()
		{
			return GetUVLayer(0);
		}
		const FDynamicMeshUVOverlay* PrimaryUV() const
		{
			return GetUVLayer(0);
		}

		int NumNormalLayers() const
		{
			return (int)NormalLayers.size();
		}

		void SetNumNormalLayers(int Num);

		FDynamicMeshNormalOverlay* GetNormalLayer(int Index)
		{
			if (Index < NormalLayers.size() && Index > -1)
			{
				return &NormalLayers[Index];
			}
			return nullptr;
		}

		const FDynamicMeshNormalOverlay* GetNormalLayer(int Index) const
		{
			if (Index < NormalLayers.size() && Index > -1)
			{
				return &NormalLayers[Index];
			}
			return nullptr;
		}

		FDynamicMeshNormalOverlay* PrimaryNormals()
		{
			return GetNormalLayer(0);
		}

		const FDynamicMeshNormalOverlay* PrimaryNormals() const
		{
			return GetNormalLayer(0);
		}

		void EnableTangents()
		{
			SetNumNormalLayers(3);
		}

		void DisableTangents()
		{
			SetNumNormalLayers(1);
		}

		FDynamicMeshNormalOverlay* PrimaryTangents()
		{
			return GetNormalLayer(1);
		}

		const FDynamicMeshNormalOverlay* PrimaryTangents() const
		{
			return GetNormalLayer(1);
		}

		FDynamicMeshNormalOverlay* PrimaryBiTangents()
		{
			return GetNormalLayer(2);
		}

		const FDynamicMeshNormalOverlay* PrimaryBiTangents() const
		{
			return GetNormalLayer(2);
		}

		bool HasTangentSpace() const
		{
			return (PrimaryNormals() != nullptr && PrimaryTangents() != nullptr && PrimaryBiTangents() != nullptr);
		}

		bool HasPrimaryColors() const
		{
			return !!ColorLayer;
		}

		FDynamicMeshColorOverlay* PrimaryColors()
		{
			return ColorLayer;
		}

		const FDynamicMeshColorOverlay* PrimaryColors() const
		{
			return ColorLayer;
		}

		void EnablePrimaryColors();

		void DisablePrimaryColors()
		{
			if (ColorLayer)
			{
				delete ColorLayer;
				ColorLayer = nullptr;
			}
		}

		int NumWeightLayers() const
		{
			return (int)WeightLayers.size();
		}

		void SetNumWeightLayers(int Num)
		{
			if ((int)WeightLayers.size() == Num)
			{
				return;
			}
			if (Num >= (int)WeightLayers.size())
			{
				for (int i = (int)WeightLayers.size(); i < Num; ++i)
				{
					WeightLayers.push_back(FDynamicMeshWeightAttribute(ParentMesh));
				}
			}
			else
			{
				WeightLayers.resize(Num);
			}
			assert(WeightLayers.size() == Num);
		}

		void RemoveWeightLayer(int Index)
		{
			WeightLayers.erase(WeightLayers.begin() + Index);
		}

		FDynamicMeshWeightAttribute* GetWeightLayer(int Index)
		{
			return &WeightLayers[Index];
		}

		const FDynamicMeshWeightAttribute* GetWeightLayer(int Index) const
		{
			return &WeightLayers[Index];
		}

		/*
		* 
		* 
		int NumPolygroupLayers() const;

		void SetNumPolygroupLayers(int Num);

		FDynamicMeshPolygroupAttribute* GetPolygroupLayer(int Index);

		const FDynamicMeshPolygroupAttribute* GetPolygroupLayer(int Index) const;



		void AttachSkinWeightsAttribute(std::string InProfileName, FDynamicMeshVertexSkinWeightsAttribute* InAttribute);

		void RemoveSkinWeightsAttribute(std::string InProfileName);

		bool HasSkinWeightsAttribute(std::string InProfileName) const
		{
			return SkinWeightAttributes->Contains(InProfileName);
		}

		FDynamicMeshVertexSkinWeightsAttribute* GetSkinWeightsAttribute(std::string InProfileName) const
		{
			if (SkinWeightAttributes->Contains(InProfileName))
			{
				return SkinWeightAttributes[InProfileName].Get();
			}
			else
			{
				return nullptr;
			}
		}

		const TMap<std::string, TUniquePtr<FDynamicMeshVertexSkinWeightsAttribute>>& GetSkinWeightsAttributes() const
		{
			return SkinWeightAttributes;
		}

		void CopyBoneAttributes(const FDynamicMeshAttributeSet& Copy);

		void EnableMatchingBoneAttributes(const FDynamicMeshAttributeSet& ToMatch, bool bClearExisting, bool bDiscardExtraAttributes);

		bool IsSameBoneAttributesAs(const FDynamicMeshAttributeSet& Other) const;

		bool CheckBoneValidity(EValidityCheckFailMode FailMode) const;

		bool AppendBonesUnique(const FDynamicMeshAttributeSet& Other);

		void EnableBones(const int InBonesNum);

		void DisableBones();

		int GetNumBones() const;

		bool HasBones() const
		{
			return !!BoneNameAttrib;
		}

		FDynamicMeshBoneNameAttribute* GetBoneNames()
		{
			return BoneNameAttrib.Get();
		}

		const FDynamicMeshBoneNameAttribute* GetBoneNames() const
		{
			return BoneNameAttrib.Get();
		}

		FDynamicMeshBoneParentIndexAttribute* GetBoneParentIndices()
		{
			return BoneParentIndexAttrib.Get();
		}

		const FDynamicMeshBoneParentIndexAttribute* GetBoneParentIndices() const
		{
			return BoneParentIndexAttrib.Get();
		}

		FDynamicMeshBonePoseAttribute* GetBonePoses()
		{
			return BonePoseAttrib.Get();
		}

		const FDynamicMeshBonePoseAttribute* GetBonePoses() const
		{
			return BonePoseAttrib.Get();
		}

		FDynamicMeshBoneColorAttribute* GetBoneColors()
		{
			return BoneColorAttrib.Get();
		}

		const FDynamicMeshBoneColorAttribute* GetBoneColors() const
		{
			return BoneColorAttrib.Get();
		}
		*/

		void AttachAttribute(std::string AttribName, FDynamicMeshAttributeBase* Attribute)
		{
			if (GenericAttributes.find(AttribName) != GenericAttributes.end())
			{
				RemoveAttribute(AttribName);
			}
			GenericAttributes.emplace(AttribName, Attribute);
		}

		void RemoveAttribute(std::string AttribName)
		{
			if (GenericAttributes.find(AttribName) != GenericAttributes.end())
			{
				GenericAttributes.erase(AttribName);
			}
		}

		FDynamicMeshAttributeBase* GetAttachedAttribute(const std::string& AttribName)
		{
			return GenericAttributes[AttribName];
		}

		const FDynamicMeshAttributeBase* GetAttachedAttribute(const std::string& AttribName) const
		{
			auto it = GenericAttributes.find(AttribName);
			if (it != GenericAttributes.end())
			{
				return it->second;
			}
			return nullptr;
		}

		int NumAttachedAttributes() const
		{
			return (int)GenericAttributes.size();
		}

		bool HasAttachedAttribute(const std::string& AttribName) const
		{
			return GenericAttributes.find(AttribName) != GenericAttributes.end();
		}

		const std::map<std::string, FDynamicMeshAttributeBase*>& GetAttachedAttributes() const
		{
			return GenericAttributes;
		}

		static void EnableUVChannels(DynamicMesh* Mesh, int NumUVChannels, bool bResetExisting = false, bool bDisablePrevious = true);
		static int NumEnabledUVChannels(DynamicMesh* Mesh);
		static void AddVertexColorAttribute(DynamicMesh* Mesh);
		static void Augment(DynamicMesh* Mesh, int NumUVChannels);
		static bool IsAugmented(const DynamicMesh* Mesh);
		static void SetUV(DynamicMesh* Mesh, int VID, Vector2 UV, int UVLayer);
		static void SetAllUV(DynamicMesh* Mesh, int VID, Vector2 UV, int NumUVLayers);
		static void GetUV(const DynamicMesh* Mesh, int VID, Vector2& UV, int UVLayer);
		static void SetTangent(DynamicMesh* Mesh, int VID, Vector3 Normal, Vector3 TangentU, Vector3 TangentV);
		static void GetTangent(const DynamicMesh* Mesh, int VID, Vector3& U, Vector3& V);
		static Vector4 GetVertexColor(const DynamicMesh* Mesh, int VID);
		static void SetVertexColor(DynamicMesh* Mesh, int VID, Vector4 Color);

	protected:
		DynamicMesh* ParentMesh;

		FDynamicMeshMaterialAttribute* MaterialIDAttrib = nullptr;

		std::vector<FDynamicMeshUVOverlay> UVLayers;
		std::vector<FDynamicMeshNormalOverlay> NormalLayers;

		FDynamicMeshColorOverlay* ColorLayer;

		std::vector<FDynamicMeshWeightAttribute> WeightLayers;
		// std::vector<FDynamicMeshPolygroupAttribute> PolygroupLayers;

		// using SkinWeightAttributesMap = TMap<std::string, TUniquePtr<FDynamicMeshVertexSkinWeightsAttribute>>;
		// SkinWeightAttributesMap SkinWeightAttributes;

		// Bone attributes
		// TUniquePtr<FDynamicMeshBoneNameAttribute> BoneNameAttrib;
		// TUniquePtr<FDynamicMeshBoneParentIndexAttribute> BoneParentIndexAttrib;
		// TUniquePtr<FDynamicMeshBonePoseAttribute> BonePoseAttrib;
		// TUniquePtr<FDynamicMeshBoneColorAttribute> BoneColorAttrib;

		std::map<std::string, FDynamicMeshAttributeBase*> GenericAttributes;

		std::vector<TDynamicAttributeBase*> RegisteredAttributes;

	protected:
		friend class DynamicMesh;

		void Initialize(int MaxVertexID, int MaxTriangleID);

		void OnNewTriangle(int TriangleID, bool bInserted);
		void OnNewVertex(int VertexID, bool bInserted);
		void OnRemoveTriangle(int TriangleID);
		void OnRemoveVertex(int VertexID);
		void OnReverseTriOrientation(int TriangleID);
		void OnSplitEdge(const FEdgeSplitInfo& splitInfo);
		void OnFlipEdge(const FEdgeFlipInfo& flipInfo);
		void OnCollapseEdge(const FEdgeCollapseInfo& collapseInfo);
		void OnPokeTriangle(const FPokeTriangleInfo& pokeInfo);
		void OnMergeEdges(const FMergeEdgesInfo& mergeInfo);
		void OnSplitVertex(const FVertexSplitInfo& SplitInfo, const std::vector<int>& TrianglesToUpdate);
	};

}	// namespace Riemann
