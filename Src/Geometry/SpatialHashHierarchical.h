#pragma once

#include <vector>
#include "../Maths/Box3.h"

namespace Riemann
{
	// Batch-built AABB index. Each item occupies at most eight cells of the
	// smallest power-of-two level that contains its longest side. Queries visit
	// occupied levels and retain exact bounds tests, including touching boxes.
	class SpatialHashHierarchical
	{
	public:
		void Build(const std::vector<Box3>& Bounds, float MinimumCellSize = 0);
		void Query(const Box3& Bounds, std::vector<int>& Indices) const;
		const Box3& GetBounds() const { return m_Bounds; }
	private:
		struct Cell { int X, Y, Z; };
		struct Entry { Cell Position; int Level, Item, Next; };
		struct Level { int Exponent, First = -1, Count = 0; double InverseSize; };
		static int SizeLevel(double Size);
		static size_t Hash(const Cell& Position, int Level);
		Cell CellAt(const Vector3& Position, double InverseSize) const;
		Box3 m_Bounds = Box3(Vector3::Zero(), Vector3::Zero());
		Vector3 m_Origin = Vector3::Zero();
		std::vector<Box3> m_Items;
		std::vector<Cell> m_MinCells;
		std::vector<int> m_Buckets, m_NextItems;
		std::vector<Entry> m_Entries;
		std::vector<Level> m_Levels;
	};
}
