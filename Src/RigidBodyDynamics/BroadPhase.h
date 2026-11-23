#pragma once

#include <stdint.h>
#include <vector>
#include <functional>
#include <utility>
#include <array>

#include "RigidBody.h"
#include "ContactGeneration.h"

namespace Riemann
{
	class Geometry;
	class CollisionFilter;
	struct BroadPhaseProxy
	{
		BroadPhaseProxy()
			: Handle()
			, Displacement(Vector3::Zero())
			, Version(0)
			, FrameId(0)
			, SyncId(0)
			, StateIndex(-1)
			, PrunerHandle(-1)
			, Moved(false)
			, MovingRigid(false)
			, Active(false)
		{
		}

		GeometryHandle	Handle;
		Box3			WorldBounds;
		Vector3			Displacement;
		uint64_t		Version;
		uint64_t		FrameId;
		uint64_t		SyncId;
		int				StateIndex;
		int				PrunerHandle;
		bool			Moved;
		bool			MovingRigid;
		bool			Active;
	};

	struct OverlapPair
	{
		OverlapPair(int _Geom1, int _Geom2)
		{
			index1 = _Geom1;
			index2 = _Geom2;
		}
		int index1;
		int index2;
	};

	enum class BroadPhaseObjectType : uint8_t { Rigid, Cloth, Deformable, Aerodynamics };
	enum class BroadPhasePairType : uint8_t { Rigid, Cloth, Deformable, ClothRigid, DeformableRigid, Aerodynamics, Count };
	struct BroadPhasePairView
	{
		const std::vector<OverlapPair>& Pairs;
		const std::vector<uint32_t>* Indices;
		size_t Count;
		size_t size() const { return Count; }
		bool empty() const { return Count == 0; }
		const OverlapPair& operator[](size_t Index) const { return Pairs[Indices ? (*Indices)[Index] : Index]; }
		struct Iterator
		{
			const BroadPhasePairView* View;
			size_t Index;
			const OverlapPair& operator*() const { return (*View)[Index]; }
			Iterator& operator++() { ++Index; return *this; }
			bool operator!=(const Iterator& Other) const { return Index != Other.Index; }
		};
		Iterator begin() const { return { this, 0 }; }
		Iterator end() const { return { this, Count }; }
	};
	// Pair views reference the sole broadphase output. Pure rigid worlds need no
	// routing storage; mixed worlds dispatch once and append only new pair indices.
	struct BroadPhaseResult
	{
		std::vector<GeometryWorldState> States;
		std::vector<OverlapPair> Pairs;
		Geometry* GetGeometry(int Index) const { return States[Index].Geom; }
		void Clear();
		void SetObject(int Index, BroadPhaseObjectType Type, void* Owner = nullptr);
		BroadPhaseObjectType GetObjectType(int Index) const { return (size_t)Index < m_Objects.size() ? m_Objects[Index].Type : BroadPhaseObjectType::Rigid; }
		void* GetObjectOwner(int Index) const { return (size_t)Index < m_Objects.size() ? m_Objects[Index].Owner : nullptr; }
		void DispatchPairs(size_t Begin = 0);
		BroadPhasePairView GetPairs(BroadPhasePairType Type) const;
		uint64_t GetRevision() const { return m_Revision; }
	private:
		struct Object { BroadPhaseObjectType Type = BroadPhaseObjectType::Rigid; void* Owner = nullptr; };
		std::vector<Object> m_Objects;
		std::array<std::vector<uint32_t>, (size_t)BroadPhasePairType::Count> m_PairIndices;
		uint64_t m_Revision = 0;
	};
	// Bind world slots to local solver rows once, then use direct array access in
	// every iteration. Appended slots bind incrementally; removed slots stay null.
	template<class Row> class BroadPhaseObjectMap
	{
	public:
		void Invalidate() { m_Source = nullptr; m_Rows.clear(); }
		template<class Lookup> void Update(const BroadPhaseResult& Source, Lookup Find, bool Reset = false)
		{
			if (Reset || m_Source != &Source || m_Revision != Source.GetRevision())
			{
				m_Rows.clear(); m_Source = &Source; m_Revision = Source.GetRevision();
			}
			for (size_t I = m_Rows.size(); I < Source.States.size(); ++I) m_Rows.push_back(Find((int)I));
		}
		const Row& operator[](int Index) const { return m_Rows[Index]; }
	private:
		const BroadPhaseResult* m_Source = nullptr;
		uint64_t m_Revision = 0;
		std::vector<Row> m_Rows;
	};
	using BroadPhaseQuery = std::function<const BroadPhaseResult&()>;

	class BroadPhase
	{
	public:
		virtual ~BroadPhase() {}
		void SetCollisionFilter(CollisionFilter* Filter) { m_Filter = Filter; }
		void SetContactGeneration(const ContactGenerationSettings& Settings, float Dt) { m_ContactSettings = Settings; m_Dt = Dt; }
		void ProduceOverlaps(const std::vector<GeometryWorldState>& states, std::vector<OverlapPair>* overlaps);
		void ProduceOverlaps(GeometryWorldStateSpan states, std::vector<OverlapPair>* overlaps);
		static bool AllowsPair(const GeometryWorldState& A, const GeometryWorldState& B, CollisionFilter* Filter);
		static void BuildStandalone(const std::vector<Geometry*>& Geometries, BroadPhaseResult& Result,
			BroadPhaseObjectType Type = BroadPhaseObjectType::Deformable);

		static BroadPhase* Create_AllPairs();		// Slow, debug purpose
		static BroadPhase* Create_Bruteforce();		// Slow, debug purpose
		static BroadPhase* Create_SAP();
		static BroadPhase* Create_ABP();
		static BroadPhase* Create_MBP();
		static BroadPhase* Create_DynamicAABB();
	protected:
		virtual void ProduceOverlapsImpl(GeometryWorldStateSpan states, std::vector<OverlapPair>* overlaps) = 0;
		CollisionFilter* m_Filter = nullptr;
	private:
		ContactGenerationSettings m_ContactSettings;
		float m_Dt = 0.0f;
		std::vector<GeometryWorldState> m_ExpandedStates;
		std::vector<const GeometryWorldState*> m_ExpandedPointers;
	};
}
