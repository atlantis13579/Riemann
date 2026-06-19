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
		virtual void OnSplitEdge(const EdgeSplitInfo& SplitInfo) {}
		virtual void OnFlipEdge(const EdgeFlipInfo& FlipInfo) {}
		virtual void OnCollapseEdge(const EdgeCollapseInfo& CollapseInfo) {}
		virtual void OnPokeTriangle(const PokeTriangleInfo& PokeInfo) {}
		virtual void OnMergeEdges(const MergeEdgesInfo& MergeInfo) {}
		virtual void OnSplitVertex(const VertexSplitInfo& SplitInfo, const std::vector<int>& TrianglesToUpdate) {}

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
			int i = 3 * TID;
			if (TID < 0 || i + 2 >= (int)ElementTriangles.size())
			{
				return false;
			}
			bool bIsSet = ElementTriangles[i] >= 0;
			// we require that triangle elements either be all set or all unset
			assert((ElementTriangles[i + 1] >= 0) == bIsSet);
			assert((ElementTriangles[i + 2] >= 0) == bIsSet);
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
			if (TriangleID < 0 || i + 2 >= (int)ElementTriangles.size())
			{
				return false;
			}
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

		int CountVertexElements(int vid, bool bBruteForce = false) const;

		int GetElementIDAtVertex(int TriangleID, int VertexID) const
		{
			Index3 Triangle;
			if (!GetTriangleIfValid(TriangleID, Triangle))
			{
				return -1;
			}
			for (int IDX = 0; IDX < 3; ++IDX)
			{
				int ElementID = Triangle[IDX];
				if (ElementID >= 0 && ElementID < (int)ParentVertices.size() && ParentVertices[ElementID] == VertexID)
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

		void OnSplitEdge(const EdgeSplitInfo& splitInfo);

		void OnFlipEdge(const EdgeFlipInfo& FlipInfo);

		void OnCollapseEdge(const EdgeCollapseInfo& collapseInfo);

		void OnPokeTriangle(const PokeTriangleInfo& PokeInfo);

		void OnMergeEdges(const MergeEdgesInfo& MergeInfo);

		void OnSplitVertex(const VertexSplitInfo& SplitInfo, const std::vector<int>& TrianglesToUpdate);

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
			BaseType::GetElement(ElementID, (RealType*)&V);
		}

		inline VectorType GetElementAtVertex(int TriangleID, int VertexID) const
		{
			VectorType V = VectorType::Zero();
			int ElementID = BaseType::GetElementIDAtVertex(TriangleID, VertexID);
			if (ElementID >= 0)
			{
				BaseType::GetElement(ElementID, (RealType*)&V);
			}
			return V;
		}

		inline void GetElementAtVertex(int TriangleID, int VertexID, VectorType& V) const
		{
			int ElementID = BaseType::GetElementIDAtVertex(TriangleID, VertexID);
			if (ElementID >= 0)
			{
				BaseType::GetElement(ElementID, (RealType*)&V);
			}
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

		TDynamicMeshTriangleAttribute(DynamicMesh* ParentMeshIn, bool bAutoInit = true);

		AttribValueType GetDefaultAttributeValue() const { return (AttribValueType)0; }

	private:
		void Reparent(DynamicMesh* ParentMeshIn)
		{
			ParentMesh = ParentMeshIn;
		}

	public:
		const DynamicMesh* GetParentMesh() const { return ParentMesh; }
		DynamicMesh* GetParentMesh() { return ParentMesh; }

		void Initialize(AttribValueType InitialValue = (AttribValueType)0);

		void SetNewValue(int NewTriangleID, const AttribValueType* Data);

		bool CopyOut(int RawID, void* Buffer, int BufferSize) const
		{
			if (sizeof(AttribValueType) * AttribDimension != BufferSize)
			{
				return false;
			}
			AttribValueType* BufferData = static_cast<AttribValueType*>(Buffer);
			int k = RawID * AttribDimension;
			AttribValueType DefaultValue = GetDefaultAttributeValue();
			for (int i = 0; i < AttribDimension; ++i)
			{
				size_t Index = (size_t)k + i;
				BufferData[i] = Index < AttribValues.size() ? AttribValues[Index] : DefaultValue;
			}
			return true;
		}

		bool CopyIn(int RawID, void* Buffer, int BufferSize)
		{
			if (sizeof(AttribValueType) * AttribDimension != BufferSize)
			{
				return false;
			}
			if (RawID < 0)
			{
				return false;
			}
			AttribValueType* BufferData = static_cast<AttribValueType*>(Buffer);
			ResizeAttribStoreIfNeeded(RawID);
			size_t k = (size_t)RawID * AttribDimension;
			for (int i = 0; i < AttribDimension; ++i)
			{
				AttribValues[k + i] = BufferData[i];
			}
			return true;
		}

		inline void GetValue(int TriangleID, AttribValueType* Data) const
		{
			int k = TriangleID * AttribDimension;
			AttribValueType DefaultValue = GetDefaultAttributeValue();
			for (int i = 0; i < AttribDimension; ++i)
			{
				size_t Index = (size_t)k + i;
				Data[i] = Index < AttribValues.size() ? AttribValues[Index] : DefaultValue;
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
		virtual void OnSplitEdge(const EdgeSplitInfo& SplitInfo) override
		{
			CopyValue(SplitInfo.OriginalTriangles.a, SplitInfo.NewTriangles.a);
			if (SplitInfo.OriginalTriangles.b != -1)
			{
				CopyValue(SplitInfo.OriginalTriangles.b, SplitInfo.NewTriangles.b);
			}
		}

		virtual void OnFlipEdge(const EdgeFlipInfo& FlipInfo) override
		{
			// yikes! triangles did not actually change so we will leave attrib unmodified
		}

		virtual void OnCollapseEdge(const EdgeCollapseInfo& CollapseInfo) override
		{
			// nothing to do here, triangles were only deleted
		}

		virtual void OnPokeTriangle(const PokeTriangleInfo& PokeInfo) override
		{
			CopyValue(PokeInfo.OriginalTriangle, PokeInfo.NewTriangles.a);
			CopyValue(PokeInfo.OriginalTriangle, PokeInfo.NewTriangles.b);
		}

		virtual void OnMergeEdges(const MergeEdgesInfo& MergeInfo) override
		{
			// nothing to do here because triangles did not change
		}

		virtual void OnSplitVertex(const VertexSplitInfo& SplitInfo, const std::vector<int>& TrianglesToUpdate) override
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
			if (NewTriangleID < 0)
			{
				return;
			}
			if (NewTriangleID >= (int)this->AttribValues.size())
			{
				this->AttribValues.resize(NewTriangleID + 1, 0);
			}
			this->AttribValues[NewTriangleID] = Value;
		}

		inline RealType GetValue(int TriangleID) const
		{
			if (TriangleID < 0 || TriangleID >= (int)this->AttribValues.size())
			{
				return this->GetDefaultAttributeValue();
			}
			return this->AttribValues[TriangleID];
		}

		inline void SetValue(int TriangleID, RealType Value)
		{
			if (TriangleID < 0)
			{
				return;
			}
			if (TriangleID >= (int)this->AttribValues.size())
			{
				this->AttribValues.resize(TriangleID + 1, this->GetDefaultAttributeValue());
			}
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
		void Initialize(AttribValueType InitialValue = (AttribValueType)0);

		void SetNewValue(int NewVertexID, const AttribValueType* Data);

		bool CopyOut(int RawID, void* Buffer, int BufferSize) const
		{
			if (sizeof(AttribValueType) * AttribDimension != BufferSize)
			{
				return false;
			}
			AttribValueType* BufferData = static_cast<AttribValueType*>(Buffer);
			int k = RawID * AttribDimension;
			AttribValueType DefaultValue = GetDefaultAttributeValue();
			for (int i = 0; i < AttribDimension; ++i)
			{
				size_t Index = (size_t)k + i;
				BufferData[i] = Index < AttribValues.size() ? AttribValues[Index] : DefaultValue;
			}
			return true;
		}

		bool CopyIn(int RawID, void* Buffer, int BufferSize)
		{
			if (sizeof(AttribValueType) * AttribDimension != BufferSize)
			{
				return false;
			}
			if (RawID < 0)
			{
				return false;
			}
			AttribValueType* BufferData = static_cast<AttribValueType*>(Buffer);
			ResizeAttribStoreIfNeeded(RawID);
			size_t k = (size_t)RawID * AttribDimension;
			for (int i = 0; i < AttribDimension; ++i)
			{
				AttribValues[k + i] = BufferData[i];
			}
			return true;
		}

		inline void GetValue(int VertexID, AttribValueType* Data) const
		{
			int k = VertexID * AttribDimension;
			AttribValueType DefaultValue = GetDefaultAttributeValue();
			for (int i = 0; i < AttribDimension; ++i)
			{
				size_t Index = (size_t)k + i;
				Data[i] = Index < AttribValues.size() ? AttribValues[Index] : DefaultValue;
			}
		}

		template<typename AsType>
		void GetValue(int VertexID, AsType& Data) const
		{
			int k = VertexID * AttribDimension;
			AttribValueType DefaultValue = GetDefaultAttributeValue();
			for (int i = 0; i < AttribDimension; ++i)
			{
				size_t Index = (size_t)k + i;
				Data[i] = Index < AttribValues.size() ? AttribValues[Index] : DefaultValue;
			}
		}

		inline void SetValue(int VertexID, const AttribValueType* Data)
		{
			if (VertexID < 0)
			{
				return;
			}
			ResizeAttribStoreIfNeeded(VertexID);
			size_t k = (size_t)VertexID * AttribDimension;
			for (int i = 0; i < AttribDimension; ++i)
			{
				AttribValues[k + i] = Data[i];
			}
		}

		template<typename AsType>
		void SetValue(int VertexID, const AsType& Data)
		{
			if (VertexID < 0)
			{
				return;
			}
			ResizeAttribStoreIfNeeded(VertexID);
			size_t k = (size_t)VertexID * AttribDimension;
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

		inline AttribValueType GetStoredValue(int Index) const
		{
			if (Index < 0 || Index >= (int)AttribValues.size())
			{
				return GetDefaultAttributeValue();
			}
			return AttribValues[Index];
		}

	public:
		virtual void OnSplitEdge(const EdgeSplitInfo& SplitInfo) override
		{
			ResizeAttribStoreIfNeeded(SplitInfo.NewVertex);
			SetAttributeFromLerp(SplitInfo.NewVertex, SplitInfo.OriginalVertices.a, SplitInfo.OriginalVertices.b, SplitInfo.SplitT);
		}

		virtual void OnFlipEdge(const EdgeFlipInfo& FlipInfo) override
		{
			// vertices unchanged
		}

		virtual void OnCollapseEdge(const EdgeCollapseInfo& CollapseInfo) override
		{
			SetAttributeFromLerp(CollapseInfo.KeptVertex, CollapseInfo.KeptVertex, CollapseInfo.RemovedVertex, CollapseInfo.CollapseT);
		}

		virtual void OnNewVertex(int VertexID, bool bInserted) override
		{
			ResizeAttribStoreIfNeeded(VertexID);
		}

		virtual void OnPokeTriangle(const PokeTriangleInfo& PokeInfo) override
		{
			Index3 Tri = PokeInfo.TriVertices;
			ResizeAttribStoreIfNeeded(PokeInfo.NewVertex);
			SetAttributeFromBary(PokeInfo.NewVertex, Tri.a, Tri.b, Tri.c, PokeInfo.BaryCoords);
		}

		virtual void OnMergeEdges(const MergeEdgesInfo& MergeInfo) override
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

		virtual void OnSplitVertex(const VertexSplitInfo& SplitInfo, const std::vector<int>& TrianglesToUpdate) override
		{
			CopyValue(SplitInfo.OriginalVertex, SplitInfo.NewVertex);
		}

	protected:
		virtual void SetAttributeFromLerp(int SetAttribute, int AttributeA, int AttributeB, double Alpha)
		{
			if (SetAttribute < 0)
			{
				return;
			}
			ResizeAttribStoreIfNeeded(SetAttribute);
			int IndexSet = AttribDimension * SetAttribute;
			int IndexA = AttribDimension * AttributeA;
			int IndexB = AttribDimension * AttributeB;
			double Beta = (1. - Alpha);
			for (int i = 0; i < AttribDimension; ++i)
			{
				AttribValues[IndexSet + i] = AttribValueType(Beta * GetStoredValue(IndexA + i) + Alpha * GetStoredValue(IndexB + i));
			}
		}

		virtual void SetAttributeFromBary(int SetAttribute, int AttributeA, int AttributeB, int AttributeC, const Vector3& BaryCoords)
		{
			if (SetAttribute < 0)
			{
				return;
			}
			ResizeAttribStoreIfNeeded(SetAttribute);
			int IndexSet = AttribDimension * SetAttribute;
			int IndexA = AttribDimension * AttributeA;
			int IndexB = AttribDimension * AttributeB;
			int IndexC = AttribDimension * AttributeC;
			for (int i = 0; i < AttribDimension; ++i)
			{
				AttribValues[IndexSet + i] = AttribValueType(
					BaryCoords.x * GetStoredValue(IndexA + i) + BaryCoords.y * GetStoredValue(IndexB + i) + BaryCoords.z * GetStoredValue(IndexC + i));
			}
		}
	};


	using DynamicMeshUVOverlay = TDynamicMeshVectorOverlay<float, 2, Vector2>;
	using DynamicMeshNormalOverlay = TDynamicMeshVectorOverlay<float, 3, Vector3>;
	using DynamicMeshColorOverlay = TDynamicMeshVectorOverlay<float, 4, Vector4>;
	using DynamicMeshMaterialAttribute = TDynamicMeshScalarTriangleAttribute<int>;

	template<typename AttribValueType, int AttribDimension>
	using TDynamicMeshVertexAttribute = TDynamicVertexAttribute<AttribValueType, AttribDimension>;

	using DynamicMeshAttributeBase = TDynamicAttributeBase;
	using DynamicMeshWeightAttribute = TDynamicMeshVertexAttribute<float, 1>;

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

		DynamicMeshMaterialAttribute* GetMaterialID()
		{
			return MaterialIDAttrib;
		}

		const DynamicMeshMaterialAttribute* GetMaterialID() const
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

		DynamicMeshUVOverlay* GetUVLayer(int Index)
		{
			if (Index < UVLayers.size() && Index > -1)
			{
				return &UVLayers[Index];
			}
			return nullptr;
		}

		const DynamicMeshUVOverlay* GetUVLayer(int Index) const
		{
			if (Index < UVLayers.size() && Index > -1)
			{
				return &UVLayers[Index];
			}
			return nullptr;
		}

		DynamicMeshUVOverlay* PrimaryUV()
		{
			return GetUVLayer(0);
		}
		const DynamicMeshUVOverlay* PrimaryUV() const
		{
			return GetUVLayer(0);
		}

		int NumNormalLayers() const
		{
			return (int)NormalLayers.size();
		}

		void SetNumNormalLayers(int Num);

		DynamicMeshNormalOverlay* GetNormalLayer(int Index)
		{
			if (Index < NormalLayers.size() && Index > -1)
			{
				return &NormalLayers[Index];
			}
			return nullptr;
		}

		const DynamicMeshNormalOverlay* GetNormalLayer(int Index) const
		{
			if (Index < NormalLayers.size() && Index > -1)
			{
				return &NormalLayers[Index];
			}
			return nullptr;
		}

		DynamicMeshNormalOverlay* PrimaryNormals()
		{
			return GetNormalLayer(0);
		}

		const DynamicMeshNormalOverlay* PrimaryNormals() const
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

		DynamicMeshNormalOverlay* PrimaryTangents()
		{
			return GetNormalLayer(1);
		}

		const DynamicMeshNormalOverlay* PrimaryTangents() const
		{
			return GetNormalLayer(1);
		}

		DynamicMeshNormalOverlay* PrimaryBiTangents()
		{
			return GetNormalLayer(2);
		}

		const DynamicMeshNormalOverlay* PrimaryBiTangents() const
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

		DynamicMeshColorOverlay* PrimaryColors()
		{
			return ColorLayer;
		}

		const DynamicMeshColorOverlay* PrimaryColors() const
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
					WeightLayers.push_back(DynamicMeshWeightAttribute(ParentMesh));
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

		DynamicMeshWeightAttribute* GetWeightLayer(int Index)
		{
			return &WeightLayers[Index];
		}

		const DynamicMeshWeightAttribute* GetWeightLayer(int Index) const
		{
			return &WeightLayers[Index];
		}

		void AttachAttribute(std::string AttribName, DynamicMeshAttributeBase* Attribute)
		{
			if (Attribute == nullptr)
			{
				RemoveAttribute(AttribName);
				return;
			}
			Attribute->SetName(AttribName);
			auto Found = GenericAttributes.find(AttribName);
			if (Found != GenericAttributes.end())
			{
				delete Found->second;
				Found->second = Attribute;
				return;
			}
			GenericAttributes.emplace(AttribName, Attribute);
		}

		void RemoveAttribute(std::string AttribName)
		{
			auto Found = GenericAttributes.find(AttribName);
			if (Found != GenericAttributes.end())
			{
				delete Found->second;
				GenericAttributes.erase(Found);
			}
		}

		DynamicMeshAttributeBase* GetAttachedAttribute(const std::string& AttribName)
		{
			auto Found = GenericAttributes.find(AttribName);
			return Found != GenericAttributes.end() ? Found->second : nullptr;
		}

		const DynamicMeshAttributeBase* GetAttachedAttribute(const std::string& AttribName) const
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

		const std::map<std::string, DynamicMeshAttributeBase*>& GetAttachedAttributes() const
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

		DynamicMeshMaterialAttribute* MaterialIDAttrib = nullptr;

		std::vector<DynamicMeshUVOverlay> UVLayers;
		std::vector<DynamicMeshNormalOverlay> NormalLayers;

		DynamicMeshColorOverlay* ColorLayer = nullptr;

		std::vector<DynamicMeshWeightAttribute> WeightLayers;

		std::map<std::string, DynamicMeshAttributeBase*> GenericAttributes;

		std::vector<TDynamicAttributeBase*> RegisteredAttributes;

	protected:
		friend class DynamicMesh;

		void Initialize(int MaxVertexID, int MaxTriangleID);

		void OnNewTriangle(int TriangleID, bool bInserted);
		void OnNewVertex(int VertexID, bool bInserted);
		void OnRemoveTriangle(int TriangleID);
		void OnRemoveVertex(int VertexID);
		void OnReverseTriOrientation(int TriangleID);
		void OnSplitEdge(const EdgeSplitInfo& splitInfo);
		void OnFlipEdge(const EdgeFlipInfo& flipInfo);
		void OnCollapseEdge(const EdgeCollapseInfo& collapseInfo);
		void OnPokeTriangle(const PokeTriangleInfo& pokeInfo);
		void OnMergeEdges(const MergeEdgesInfo& mergeInfo);
		void OnSplitVertex(const VertexSplitInfo& SplitInfo, const std::vector<int>& TrianglesToUpdate);
	};

}	// namespace Riemann
