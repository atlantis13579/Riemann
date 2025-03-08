#include <assert.h>

#include "SparseOctree.h"
#include "../CollisionPrimitive/AxisAlignedBox3.h"
#include "../Maths/Maths.h"

namespace Riemann
{

	SparseOctree::SparseOctree(const Vector3& _min, const Vector3& _max, int _max_depth)
	{
		const float expand_factor = 0.25f;
		Min = _min - expand_factor * (_max - _min);
		Vector3 dims = (_max - _min) * (1.0f + 2.0f * expand_factor);
		RootDimension = Maths::Max(dims.x, dims.y, dims.z);
		MaxTreeDepth = std::min(_max_depth, MAX_TREE_DEPTH);
	}

	void SparseOctree::Clear()
	{
		m_Cells.clear();
		m_RootCells.Clear();
		m_Objects.clear();
		m_CellsLookup.clear();
	}

	bool SparseOctree::HasObject(int Id) const
	{
		int cellId = GetCellForObject(Id);
		return cellId >= 0;
	}

	bool SparseOctree::InsertObject(int Id, const Box3& Bounds)
	{
		assert(HasObject(Id) == false);

		SparseOctree::Cell c = FindCurrentContainingCell(Bounds);

		if (c.Level == -1)
		{
			// assert(false);
			InsertSpill(Id, Bounds);
			return false;
		}
		assert(IsFit(c, Bounds));

		if (c.Level == 0 && c.IsExist() == false)
		{
			InsertNewRoot(Id, Bounds, c);
			return true;
		}

		int child = GetChildIndex(c, Bounds.GetCenter());
		if (c.HasChild(child) == false)
		{
			SparseOctree::Cell NewChildCell = c.MakeChild(child);
			if (NewChildCell.Level <= MaxTreeDepth && IsFit(NewChildCell, Bounds))
			{
				InsertNewChildCell(Id, Bounds, c.Id, NewChildCell, child);
				return true;
			}
		}

		InsertToCell(Id, Bounds, c);
		return true;
	}

	bool SparseOctree::RemoveObject(int Id)
	{
		if (HasObject(Id) == false)
		{
			return false;
		}

		int cellId = GetCellForObject(Id);
		if (cellId == SPILL_CELL_ID)
		{
			int RemovedCount = (int)m_SpillObjects.count(Id);
			m_SpillObjects.erase(Id);
			assert(RemovedCount > 0);
			return (RemovedCount > 0);
		}
		else if (cellId == INVALID_CELL_ID)
		{
			return false;
		}

		m_CellsLookup.erase(Id);

		bool bInList = m_Objects[cellId].remove(Id);
		assert(bInList);
		return true;
	}

	bool SparseOctree::ReinsertObject(int Id, const Box3& NewBounds)
	{
		int cellId = INVALID_CELL_ID;

		if (HasObject(Id))
		{
			cellId = GetCellForObject(Id);
			if (cellId != INVALID_CELL_ID && cellId != SPILL_CELL_ID)
			{
				SparseOctree::Cell& CurrentCell = m_Cells[cellId];
				if (IsFit(CurrentCell, NewBounds))
				{
					return false;
				}

				m_CellsLookup.erase(Id);
				m_Objects[cellId].remove(Id);
			}
		}

		// remove object
		if (cellId != INVALID_CELL_ID)
		{
			if (cellId == SPILL_CELL_ID)
			{
				m_SpillObjects.erase(Id);
			}
			else
			{
				m_CellsLookup[Id] = INVALID_CELL_ID;
				m_Objects[Id].remove(cellId);
			}
		}

		return InsertObject(Id, NewBounds);
	}

	void SparseOctree::RangeQuery(const Box3& Bounds, std::vector<int>& Result) const
	{
		for (int Id : m_SpillObjects)
		{
			Result.push_back(Id);
		}

		std::vector<const SparseOctree::Cell*> Queue = InitializeQueryQueue(Bounds);

		while (Queue.size() > 0)
		{
			const SparseOctree::Cell* CurCell = Queue.back();
			Queue.pop_back();

			for (int Id : m_Objects[CurCell->Id])
			{
				Result.push_back(Id);
			}

			for (int k = 0; k < 8; ++k)
			{
				if (CurCell->HasChild(k))
				{
					const SparseOctree::Cell* ChildCell = &m_Cells[CurCell->GetChildID(k)];
					if (GetCellBox(*ChildCell).Intersect(Bounds))
					{
						Queue.push_back(ChildCell);
					}
				}
			}
		}
	}

	int SparseOctree::RaycastCell(const Vector3& Origin, const Vector3& Direction, std::function<float(int, const Vector3&, const Vector3&)> HitObjectDistFunc, float* Distance) const
	{
		float MaxDistance = FLT_MAX;
		int HitObjectID = -1;

		for (int id : m_SpillObjects)
		{
			const float HitDist = HitObjectDistFunc(id, Origin, Direction);
			if (HitDist < MaxDistance)
			{
				MaxDistance = HitDist;
				HitObjectID = id;
			}
		}

		std::vector<const SparseOctree::Cell*> Queue;
		Queue.reserve(64);

		m_RootCells.AllIteration(
			[&](const int& id)
			{
				const SparseOctree::Cell* RootCell = &m_Cells[id];
				const float HitDist = RaycastCell(*RootCell, Origin, Direction);
				if (HitDist < MaxDistance)
				{
					Queue.push_back(&m_Cells[id]);
				}
			});

		while (Queue.size() > 0)
		{
			const SparseOctree::Cell* CurCell = Queue.back();
			Queue.pop_back();

			for (int id : m_Objects[CurCell->Id])
			{
				const float HitDist = HitObjectDistFunc(id, Origin, Direction);
				if (HitDist < MaxDistance)
				{
					MaxDistance = HitDist;
					HitObjectID = id;
				}
			}

			for (int k = 0; k < 8; ++k)
			{
				if (CurCell->HasChild(k))
				{
					const SparseOctree::Cell* ChildCell = &m_Cells[CurCell->GetChildID(k)];
					double RayHitParam = RaycastCell(*ChildCell, Origin, Direction);
					if (RayHitParam < MaxDistance)
					{
						Queue.push_back(ChildCell);
					}
				}
			}
		}

		*Distance = MaxDistance;
		return HitObjectID;
	}

	float SparseOctree::GetCellWidth(int Level) const
	{
		assert(Level <= MAX_TREE_DEPTH);
		float Divisor = (float)((uint64_t)1 << (Level & MAX_TREE_DEPTH));
		float width = RootDimension / Divisor;
		return width;
	}

	Box3 SparseOctree::GetBox(int Level, const Vector3i& Index) const
	{
		float width = GetCellWidth(Level);
		float minx = width * (float)Index.x + Min.x;
		float miny = width * (float)Index.y + Min.y;
		float minz = width * (float)Index.z + Min.z;
		return Box3(Vector3(minx, miny, minz),
					Vector3(minx + width, miny + width, minz + width));
	}

	Box3 SparseOctree::GetCellBox(const Cell& c) const
	{
		Box3 box = GetBox(c.Level, c.Index);
		return box;
	}

	Vector3 SparseOctree::GetCellCenter(const Cell& c) const
	{
		float width = GetCellWidth(c.Level);
		float minx = width * (float)c.Index.x + Min.x;
		float miny = width * (float)c.Index.y + Min.y;
		float minz = width * (float)c.Index.z + Min.z;
		width *= 0.5;
		return Vector3(minx + width, miny + width, minz + width);
	}

	Vector3i SparseOctree::PointToIndex(int Level, const Vector3& Position) const
	{
		float width = GetCellWidth(Level);
		int i = (int)floorf(Position.x / width);
		int j = (int)floorf(Position.y / width);
		int k = (int)floorf(Position.z / width);
		return Vector3i(i, j, k);
	}

	int SparseOctree::GetChildIndex(const Cell& c, const Vector3& Position) const
	{
		Vector3 v = GetCellCenter(c);
		int ChildIndex =
			((Position.x < v.x) ? 0 : 1) +
			((Position.y < v.y) ? 0 : 2) +
			((Position.z < v.z) ? 0 : 4);
		return ChildIndex;
	}

	bool SparseOctree::IsFit(const Cell& c, const Box3& Bounds) const
	{
		Box3 box = GetCellBox(c);
		return box.IsInside(Bounds);
	}

	int SparseOctree::GetCellForObject(int Id) const
	{
		auto it = m_CellsLookup.find(Id);
		if (it == m_CellsLookup.end())
		{
			return -1;
		}
		return it->second;
	}

	SparseOctree::Cell SparseOctree::FindCurrentContainingCell(const Box3& Bounds) const
	{
		// float BoxWidth = Bounds.MaxDim();
		Vector3 BoxCenter = Bounds.GetCenter();

		Vector3i RootIndex = PointToIndex(0, BoxCenter);
		const int* RootId = m_RootCells.Find(RootIndex);
		if (RootId == nullptr)
		{
			SparseOctree::Cell RootCell(0, RootIndex);
			if (IsFit(RootCell, Bounds))
			{
				return RootCell;
			}
			else
			{
				return SparseOctree::Cell(-1, Vector3i::Zero());
			}

		}

		const SparseOctree::Cell* RootCell = &m_Cells[*RootId];
		if (IsFit(*RootCell, Bounds) == false)
		{
			return SparseOctree::Cell(-1, Vector3i::Zero());
		}

		const SparseOctree::Cell* CurrentCell = RootCell;
		do
		{
			int ChildIdx = GetChildIndex(*CurrentCell, BoxCenter);
			if (CurrentCell->HasChild(ChildIdx))
			{
				int ChildCellID = CurrentCell->GetChildID(ChildIdx);
				const SparseOctree::Cell* ChildCell = &m_Cells[ChildCellID];
				if (IsFit(*ChildCell, Bounds))
				{
					CurrentCell = ChildCell;
					continue;
				}
			}

			return *CurrentCell;

		} while (true);

		return SparseOctree::Cell(-1, Vector3i::Zero());
	}

	void SparseOctree::InsertNewRoot(int Id, const Box3& Bounds, Cell NewRootCell)
	{
		assert(m_RootCells.Has(NewRootCell.Index) == false);

		NewRootCell.Id = (int)m_Cells.size();
		m_Cells.push_back(NewRootCell);

		assert(m_CellsLookup.find(Id) == m_CellsLookup.end());
		m_CellsLookup[Id] = NewRootCell.Id;

		int* RootCellElem = m_RootCells.Get(NewRootCell.Index, true, 0);
		*RootCellElem = NewRootCell.Id;

		assert(NewRootCell.Id >= m_Objects.size());
		m_Objects.resize(NewRootCell.Id + 1);
		m_Objects[NewRootCell.Id].push_back(Id);

		assert(IsFit(NewRootCell, Bounds));
	}

	void SparseOctree::InsertToCell(int Id, const Box3& Bounds, const Cell& ExistingCell)
	{
		assert(m_CellsLookup.find(Id) == m_CellsLookup.end());
		m_CellsLookup[Id] = ExistingCell.Id;

		m_Objects[ExistingCell.Id].push_back(Id);

		assert(IsFit(ExistingCell, Bounds));
	}

	void SparseOctree::InsertNewChildCell(int Id, const Box3& Bounds, int ParentCellID, Cell NewChildCell, int ChildIdx)
	{
		NewChildCell.Id = (int)m_Cells.size();
		m_Cells.push_back(NewChildCell);

		assert(m_CellsLookup.find(Id) == m_CellsLookup.end());
		m_CellsLookup[Id] = NewChildCell.Id;

		assert(NewChildCell.Id >= m_Objects.size());
		m_Objects.resize(NewChildCell.Id + 1);
		m_Objects[NewChildCell.Id].push_back(Id);

		SparseOctree::Cell& OrigParentCell = m_Cells[ParentCellID];
		assert(OrigParentCell.HasChild(ChildIdx) == false);
		OrigParentCell.SetChild(ChildIdx, NewChildCell);

		assert(IsFit(NewChildCell, Bounds));
	}

	void SparseOctree::InsertSpill(int ID, const Box3& Bounds)
	{
		m_SpillObjects.insert(ID);
		m_CellsLookup[ID] = SPILL_CELL_ID;
	}

	float SparseOctree::RaycastCell(const SparseOctree::Cell& Cell, const Vector3& Origin, const Vector3& Direction) const
	{
		Box3 box = GetCellBox(Cell);
		float dist = FLT_MAX;
		AxisAlignedBox3 aabb(box.Min, box.Max);
		if (aabb.IntersectRay(Origin, Direction, &dist))
		{
			return dist;
		}
		else
		{
			return FLT_MAX;
		}
	}

	std::vector<const SparseOctree::Cell*> SparseOctree::InitializeQueryQueue(const Vector3& Point) const
	{
		std::vector<const SparseOctree::Cell*> Queue;

		constexpr int MinCountForRangeQuery = 10;
		if (m_RootCells.GetSize() > MinCountForRangeQuery)
		{
			Vector3i RootMinIndex = PointToIndex(0, Point);
			Vector3i RootMaxIndex = PointToIndex(0, Point);
			Vector3i QuerySize = RootMaxIndex - RootMinIndex + Vector3i(1, 1, 1);
			if (m_RootCells.GetSize() > QuerySize.x * QuerySize.y * QuerySize.z)
			{
				m_RootCells.RangeIteration(RootMinIndex, RootMaxIndex,
					[&](int RootCellID)
					{
						Queue.push_back(&m_Cells[RootCellID]);
					});
				return Queue;
			}
		}

		m_RootCells.AllIteration(
			[&](const int& RootCellID)
			{
				const SparseOctree::Cell* RootCell = &m_Cells[RootCellID];
				if (GetCellBox(*RootCell).IsInside(Point))
				{
					Queue.push_back(&m_Cells[RootCellID]);
				}
			});
		return Queue;
	}

	std::vector<const SparseOctree::Cell*> SparseOctree::InitializeQueryQueue(const Box3& Bounds) const
	{
		std::vector<const SparseOctree::Cell*> Queue;

		constexpr int MinCountForRangeQuery = 10;
		if (m_RootCells.GetSize() > MinCountForRangeQuery)
		{
			Vector3i RootMinIndex = PointToIndex(0, Bounds.Min);
			Vector3i RootMaxIndex = PointToIndex(0, Bounds.Max);
			Vector3i QuerySize = RootMaxIndex - RootMinIndex + Vector3i(1, 1, 1);
			if (m_RootCells.GetSize() > QuerySize.x * QuerySize.y * QuerySize.z)
			{
				m_RootCells.RangeIteration(RootMinIndex, RootMaxIndex,
					[&](int RootCellID)
					{
						Queue.push_back(&m_Cells[RootCellID]);
					});
				return Queue;
			}
		}

		m_RootCells.AllIteration(
			[&](const int& RootCellID)
			{
				const SparseOctree::Cell* RootCell = &m_Cells[RootCellID];
				if (GetCellBox(*RootCell).Intersect(Bounds))
				{
					Queue.push_back(&m_Cells[RootCellID]);
				}
			});
		return Queue;
	}

}
