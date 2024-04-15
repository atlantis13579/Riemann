#pragma once

#include <math.h>
#include <functional>
#include <vector>
#include <set>
#include <map>
#include <string>

#include "../Core/ListSet.h"
#include "../Core/StaticArray.h"
#include "../Maths/Vector3.h"
#include "../Maths/Box3.h"

namespace Riemann
{
	template<typename T>
	class SparseGrid3
	{
	protected:
		std::map<Vector3i, T*>	mElements;
		Box3					mBounds;

	public:
		SparseGrid3()
		{
			mBounds = Box3::Empty();
		}

		~SparseGrid3()
		{
			clear();
		}

		bool Has(const Vector3i& Index) const
		{
			auto it = mElements.find(Index);
			return it != mElements.end();
		}

		const T* Get(const Vector3i& Index) const
		{
			auto it = mElements.find(Index);
			return it->second;
		}

		T* Get(const Vector3i& Index, bool alloc)
		{
			auto it = mElements.find(Index);
			if (it != mElements.end())
			{
				return it->second;
			}
			if (alloc)
			{
				return allocate(Index);
			}
			return nullptr;
		}

		bool release(const Vector3& Index)
		{
			auto it = mElements.find(Index);
			if (it != mElements.end())
			{
				delete it->second;
				mElements.erase(Index);
				return true;
			}
			return false;
		}

		void clear()
		{
			for (auto it : mElements)
			{
				delete it.second;
			}
			mElements.clear();
		}

		int size() const
		{
			return mElements.size();
		}

	protected:
		T* allocate(const Vector3i& index)
		{
			T* p = new T();
			mElements.Add(index, p);
			mBounds.Encapsulate(index);
			return p;
		}
	};


	class SparseOctree3
	{
	public:
		struct Cell
		{
			int CellID;
			int Level;

			Vector3i	Index;
			int			Children[8];

			Cell()
				: CellID(-1), Level(0), Index(Vector3i::Zero())
			{
				Children[0] = Children[1] = Children[2] = Children[3] = -1;
				Children[4] = Children[5] = Children[6] = Children[7] = -1;
			}

			Cell(unsigned char LevelIn, const Vector3i& IndexIn)
				: CellID(-1), Level(LevelIn), Index(IndexIn)
			{
				Children[0] = Children[1] = Children[2] = Children[3] = -1;
				Children[4] = Children[5] = Children[6] = Children[7] = -1;
			}

			inline bool IsExistingCell() const
			{
				return CellID != -1;
			}

			inline bool HasChild(int index) const
			{
				return Children[index] != -1;
			}

			inline int GetChildCellID(int index) const
			{
				return Children[index];
			}

			inline Cell MakeChildCell(int index)
			{
				Vector3i IndexOffset(
					((index & 1) != 0) ? 1 : 0,
					((index & 2) != 0) ? 1 : 0,
					((index & 4) != 0) ? 1 : 0);
				return Cell(Level + 1, 2 * Index + IndexOffset);
			}

			inline void SetChild(int index, const Cell& cell)
			{
				Children[index] = cell.CellID;
			}

		};

		float RootDimension = 1000.0;
		float MaxExpandFactor = 0.25;

		static constexpr int MaxSupportedTreeDepth = 0x1F;

		int GetMaxTreeDepth()
		{
			return MaxTreeDepth;
		}

		void SetMaxTreeDepth(int MaxTreeDepthIn)
		{
			if (!((int)MaxTreeDepthIn <= MaxSupportedTreeDepth))
			{
				MaxTreeDepthIn = (int)MaxSupportedTreeDepth;
			}

			MaxTreeDepth = MaxTreeDepthIn;
		}
	protected:
		int MaxTreeDepth = 10;

	public:

		bool ContainsObject(int ObjectID) const;

		void InsertObject(int ObjectID, const Box3& Bounds);

		bool RemoveObject(int ObjectID);

		bool ReinsertObject(int ObjectID, const Box3& NewBounds, int CellIDHint = InvalidCellID);

		bool CheckIfObjectNeedsReinsert(int ObjectID, const Box3& NewBounds, int& CellIDOut) const;

		int FindNearestHitObject(const Vector3& Origin, const Vector3 &Direction,
			std::function<Box3(int)> GetObjectBoundsFunc,
			std::function<float(int, const Vector3&, const Vector3&)> HitObjectDistFunc,
			float MaxDistance = std::numeric_limits<float>::max()) const;

		void ContainmentQuery(const Vector3& Point,
			std::function<void(int)> ObjectIDFunc) const;

		bool ContainmentQueryCancellable(const Vector3& Point,
			std::function<bool(int)> ObjectIDFunc) const;

		void RangeQuery(const Box3& Bounds,
			std::function<void(int)> ObjectIDFunc) const;

		void RangeQuery(const Box3& Bounds,
			std::vector<int>& ObjectIDsOut) const;

		void ParallelRangeQuery(const Box3& Bounds,
			std::vector<int>& ObjectIDsOut) const;

		struct FStatistics
		{
			int Levels;
			std::vector<int> LevelBoxCounts;
			std::vector<int> LevelObjCounts;
			int SpillObjCount;
			std::string ToString() const;
		};

		void ComputeStatistics(FStatistics& StatsOut) const;

	protected:
		static constexpr int InvalidCellID = -1;
		static constexpr int SpillCellID = InvalidCellID - 1;

		// FRefCountVector CellRefCounts;

		std::vector<Cell> Cells;

		ListSet<int> CellObjectLists;
		std::set<int> SpillObjectSet;

		std::vector<int> ObjectIDToCellMap;
		// FDynamicFlagArray ValidObjectIDs;

		SparseGrid3<int> RootCells;

		inline float GetCellWidth(int Level) const
		{
			assert(Level <= MaxSupportedTreeDepth);
			float Divisor = (float)((uint64_t)1 << (Level & MaxSupportedTreeDepth));
			float CellWidth = RootDimension / Divisor;
			return CellWidth;
		}

		Box3 GetBox(int Level, const Vector3i& Index, float ExpandFactor) const
		{
			float CellWidth = GetCellWidth(Level);
			float ExpandDelta = CellWidth * ExpandFactor;
			float MinX = (CellWidth * (float)Index.x) - ExpandDelta;
			float MinY = (CellWidth * (float)Index.y) - ExpandDelta;
			float MinZ = (CellWidth * (float)Index.z) - ExpandDelta;
			CellWidth += 2.0f * ExpandDelta;
			return Box3(
				Vector3(MinX, MinY, MinZ),
				Vector3(MinX + CellWidth, MinY + CellWidth, MinZ + CellWidth));
		}
		inline Box3 GetCellBox(const Cell& Cell, float ExpandFactor = 0) const
		{
			return GetBox(Cell.Level, Cell.Index, ExpandFactor);
		}
		Vector3 GetCellCenter(const Cell& Cell) const
		{
			float CellWidth = GetCellWidth(Cell.Level);
			float MinX = CellWidth * (float)Cell.Index.x;
			float MinY = CellWidth * (float)Cell.Index.y;
			float MinZ = CellWidth * (float)Cell.Index.z;
			CellWidth *= 0.5;
			return Vector3(MinX + CellWidth, MinY + CellWidth, MinZ + CellWidth);
		}

		Vector3i PointToIndex(int Level, const Vector3& Position) const
		{
			float CellWidth = GetCellWidth(Level);
			int i = (int)floorf(Position.x / CellWidth);
			int j = (int)floorf(Position.y / CellWidth);
			int k = (int)floorf(Position.z / CellWidth);
			return Vector3i(i, j, k);
		}

		int ToChildCellIndex(const Cell& Cell, const Vector3& Position) const
		{
			Vector3 Center = GetCellCenter(Cell);
			int ChildIndex =
				((Position.x < Center.x) ? 0 : 1) +
				((Position.y < Center.y) ? 0 : 2) +
				((Position.z < Center.z) ? 0 : 4);
			return ChildIndex;
		}

		bool CanFit(const Cell& Cell, const Box3& Bounds) const
		{
			Box3 CellBox = GetCellBox(Cell, MaxExpandFactor);
			return CellBox.IsInside(Bounds);
		}

		int GetCellForObject(int ObjectID) const
		{
			if (ObjectID >= 0 && static_cast<size_t>(ObjectID) < ObjectIDToCellMap.size())
			{
				return ObjectIDToCellMap[ObjectID];
			}
			return InvalidCellID;
		}

		Cell FindCurrentContainingCell(const Box3& Bounds) const;

		void Insert_Spill(int ObjectID, const Box3& Bounds);
		void Insert_NewRoot(int ObjectID, const Box3& Bounds, Cell NewRootCell);
		void Insert_ToCell(int ObjectID, const Box3& Bounds, const Cell& ExistingCell);
		void Insert_NewChildCell(int ObjectID, const Box3& Bounds, int ParentCellID, Cell NewChildCell, int ChildIdx);

		float FindNearestRayCellIntersection(const Cell& Cell, const Vector3& Origin, const Vector3 &Direction) const;

		void BranchRangeQuery(const Cell* ParentCell,
			const Box3& Bounds,
			std::vector<int>& ObjectIDs) const;

	private:
		/*
		std::vector<const FSparseOctreeCell*, TInlineAllocator<32>> InitializeQueryQueue(const Vector3& Point) const
		{
			std::vector<const FSparseOctreeCell*, TInlineAllocator<32>> Queue;

			constexpr int MinCountForRangeQuery = 10;
			if (RootCells.GetCount() > MinCountForRangeQuery)
			{
				Vector3 RootBoundExpand(RootDimension * MaxExpandFactor);
				Vector3i RootMinIndex = PointToIndex(0, Point - RootBoundExpand);
				Vector3i RootMaxIndex = PointToIndex(0, Point + RootBoundExpand);
				Vector3i QuerySize = RootMaxIndex - RootMinIndex + Vector3i(1, 1, 1);
				if (RootCells.GetCount() > QuerySize.x * QuerySize.y * QuerySize.z)
				{
					RootCells.RangeIteration(RootMinIndex, RootMaxIndex, [&](int RootCellID)
						{
							Queue.Add(&Cells[RootCellID]);
						});
					return Queue;
				}
			}

			RootCells.AllocatedIteration([&](const int* RootCellID)
				{
					const FSparseOctreeCell* RootCell = &Cells[*RootCellID];
					if (GetCellBox(*RootCell, MaxExpandFactor).Contains(Point))
					{
						Queue.Add(&Cells[*RootCellID]);
					}
				});
			return Queue;
		}

		std::vector<const FSparseOctreeCell*, TInlineAllocator<32>> InitializeQueryQueue(const Box3& Bounds) const
		{
			std::vector<const FSparseOctreeCell*, TInlineAllocator<32>> Queue;

			constexpr int MinCountForRangeQuery = 10;
			if (RootCells.GetCount() > MinCountForRangeQuery)
			{
				Vector3 RootBoundExpand(RootDimension * MaxExpandFactor);
				Vector3i RootMinIndex = PointToIndex(0, Bounds.Min - RootBoundExpand);
				Vector3i RootMaxIndex = PointToIndex(0, Bounds.Max + RootBoundExpand);
				Vector3i QuerySize = RootMaxIndex - RootMinIndex + Vector3i(1, 1, 1);
				if (RootCells.GetCount() > QuerySize.x * QuerySize.y * QuerySize.z)
				{
					RootCells.RangeIteration(RootMinIndex, RootMaxIndex, [&](int RootCellID)
						{
							Queue.Add(&Cells[RootCellID]);
						});
					return Queue;
				}
			}

			RootCells.AllocatedIteration([&](const int* RootCellID)
				{
					const FSparseOctreeCell* RootCell = &Cells[*RootCellID];
					if (GetCellBox(*RootCell, MaxExpandFactor).Intersects(Bounds))
					{
						Queue.Add(&Cells[*RootCellID]);
					}
				});
			return Queue;
		}
		*/
	};
}