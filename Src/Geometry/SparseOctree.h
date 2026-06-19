#pragma once

#include <math.h>
#include <functional>
#include <vector>
#include <set>
#include <map>
#include <string>
#include <unordered_map>

#include "../Core/BitSet.h"
#include "../Core/ListSet.h"
#include "../Maths/Vector3.h"
#include "../Maths/Box3.h"

namespace Riemann
{
	template<typename T>
	class SparseGrid3
	{
	protected:
		std::map<Vector3i, T>	m_Elements;

	public:
		SparseGrid3()
		{
		}

		~SparseGrid3()
		{
		}

		void Clear()
		{
			m_Elements.clear();
		}

		bool Has(const Vector3i& Index) const
		{
			auto it = m_Elements.find(Index);
			return it != m_Elements.end();
		}

		const T* Find(const Vector3i& Index) const
		{
			auto it = m_Elements.find(Index);
			if (it != m_Elements.end())
			{
				return &it->second;
			}
			return nullptr;
		}

		T* Get(const Vector3i& Index, bool alloc_if_missing, const T& default_val)
		{
			auto it = m_Elements.find(Index);
			if (it != m_Elements.end())
			{
				return &it->second;
			}
			if (alloc_if_missing)
			{
				m_Elements.emplace(Index, default_val);
				return &m_Elements[Index];
			}
			return nullptr;
		}

		int GetSize() const
		{
			return (int)m_Elements.size();
		}

		void AllIteration(std::function<void(const T &value)> func) const
		{
			for (auto it : m_Elements)
			{
				func(it.second);
			}
		}

		void RangeIteration(const Vector3i& MinIndex, const Vector3i& MaxIndex, std::function<void(const T& value)> func) const
		{
			for (int z = MinIndex.z; z <= MaxIndex.z; ++z)
			for (int y = MinIndex.y; y <= MaxIndex.y; ++y)
			for (int x = MinIndex.x; x <= MaxIndex.x; ++x)
			{
				const T* item = Find(Vector3i(x, y, z));
				if (item)
				{
					func(*item);
				}
			}
		}
	};

	#define MAX_TREE_DEPTH		(31)
	#define	INVALID_CELL_ID		(-1)
	#define	SPILL_CELL_ID		(-2)

	class SparseOctree
	{
	public:
		struct Cell
		{
			int			Id;
			int			Level;
			Vector3i	Index;
			int			Childrens[8];

			Cell() : Id(-1), Level(0), Index(Vector3i::Zero())
			{
				for (int i = 0; i < 8; ++i)
				{
					Childrens[i] = -1;
				}
			}

			Cell(int _level, const Vector3i& _index) : Id(-1), Level(_level), Index(_index)
			{
				for (int i = 0; i < 8; ++i)
				{
					Childrens[i] = -1;
				}
			}

			inline bool IsExist() const
			{
				return Id != -1;
			}

			inline bool HasChild(int index) const
			{
				return Childrens[index] != -1;
			}

			inline int GetChildID(int index) const
			{
				return Childrens[index];
			}

			inline Cell MakeChild(int index)
			{
				Vector3i IndexOffset(
					((index & 1) != 0) ? 1 : 0,
					((index & 2) != 0) ? 1 : 0,
					((index & 4) != 0) ? 1 : 0);
				return Cell(Level + 1, 2 * Index + IndexOffset);
			}

			inline void SetChild(int index, const Cell& cell)
			{
				Childrens[index] = cell.Id;
			}
		};

	public:
		SparseOctree(const Vector3 &_min, const Vector3& _max, int max_tree_depth = 10);
		~SparseOctree() {}

		void	Clear();
		bool	HasObject(int Id) const;
		bool	InsertObject(int Id, const Box3& Bounds);
		bool	RemoveObject(int Id);
		bool	ReinsertObject(int Id, const Box3& NewBounds);
		void	RangeQuery(const Box3& Bounds, std::vector<int>& Results) const;
		int		RaycastCell(const Vector3& Origin, const Vector3& Direction, std::function<float(int, const Vector3&, const Vector3&)> HitObjectDistFunc, float *Distance) const;

	private:
		float	GetCellWidth(int Level) const;
		Box3	GetBox(int Level, const Vector3i& Index) const;
		Box3	GetCellBox(const Cell& c) const;
		Vector3 GetCellCenter(const Cell& c) const;
		int		GetCellForObject(int Id) const;

		Vector3i PointToIndex(int Level, const Vector3& Position) const;

		int GetChildIndex(const Cell& c, const Vector3& Position) const;

		bool IsFit(const Cell& c, const Box3& Bounds) const;

		Cell FindCurrentContainingCell(const Box3& Bounds) const;

		void InsertNewRoot(int Id, const Box3& Bounds, Cell NewRootCell);
		void InsertToCell(int Id, const Box3& Bounds, const Cell& ExistingCell);
		void InsertNewChildCell(int Id, const Box3& Bounds, int ParentCellID, Cell NewChildCell, int ChildIdx);
		void InsertSpill(int ID, const Box3& Bounds);
		float RaycastCell(const Cell& c, const Vector3& Origin, const Vector3 &Direction) const;

		std::vector<const Cell*> InitializeQueryQueue(const Vector3& Point) const;
		std::vector<const Cell*> InitializeQueryQueue(const Box3& Bounds) const;

	private:
		std::vector<Cell>				m_Cells;
		SparseGrid3<int>				m_RootCells;
		ListSet<int>					m_Objects;
		std::set<int>					m_SpillObjects;
		std::unordered_map<int, int>	m_CellsLookup;

		Vector3	Min = Vector3::Zero();
		float	RootDimension = 1000.0f;
		int		MaxTreeDepth = 10;
	};
}