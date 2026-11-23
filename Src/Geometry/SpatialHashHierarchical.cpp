#include "SpatialHashHierarchical.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>

namespace Riemann
{
	namespace
	{
		bool ValidHashBounds(const Box3& Bounds)
		{
			for (int Axis = 0; Axis < 3; ++Axis)
				if (!std::isfinite(Bounds.Min[Axis]) || !std::isfinite(Bounds.Max[Axis]) || Bounds.Min[Axis] > Bounds.Max[Axis]) return false;
			return true;
		}
	}
	int SpatialHashHierarchical::SizeLevel(double Size)
	{
		int Exponent; const double Fraction = std::frexp(Size, &Exponent);
		return Fraction == 0.5 ? Exponent - 1 : Exponent;
	}
	size_t SpatialHashHierarchical::Hash(const Cell& P, int Level)
	{
		uint32_t Value = uint32_t(P.X) * 0x8da6b343u ^ uint32_t(P.Y) * 0xd8163841u
			^ uint32_t(P.Z) * 0xcb1ab31fu ^ uint32_t(Level) * 0x9e3779b9u;
		Value ^= Value >> 16; return Value;
	}
	SpatialHashHierarchical::Cell SpatialHashHierarchical::CellAt(const Vector3& P, double InverseSize) const
	{
		return { (int)std::floor(((double)P.x - m_Origin.x) * InverseSize),
			(int)std::floor(((double)P.y - m_Origin.y) * InverseSize),
			(int)std::floor(((double)P.z - m_Origin.z) * InverseSize) };
	}
	void SpatialHashHierarchical::Build(const std::vector<Box3>& Bounds, float MinimumCellSize)
	{
		if (!std::isfinite(MinimumCellSize) || MinimumCellSize < 0 || Bounds.size() > size_t(std::numeric_limits<int>::max()) / 8)
			throw std::invalid_argument("Invalid hierarchical spatial hash size");
		for (const auto& B : Bounds) if (!ValidHashBounds(B)) throw std::invalid_argument("Invalid hierarchical spatial hash bounds");
		m_Items = Bounds; m_Entries.clear(); m_Levels.clear();
		m_Bounds = Box3(Vector3::Zero(), Vector3::Zero());
		if (Bounds.empty()) return;
		m_Bounds = Bounds[0]; for (const auto& B : Bounds) m_Bounds = Box3(m_Bounds, B);
		m_Origin = m_Bounds.Min;
		double Span = 0; for (int Axis = 0; Axis < 3; ++Axis) Span = std::max(Span, (double)m_Bounds.Max[Axis] - m_Bounds.Min[Axis]);
		// Rebase coordinates and bound their magnitude before converting to int.
		// This also handles zero-size features and very large world coordinates.
		const int MinimumLevel = SizeLevel(std::max({ (double)MinimumCellSize, Span / 1073741824.0, (double)std::numeric_limits<float>::min() }));
		size_t BucketCount = 1; while (BucketCount < Bounds.size() * 2) BucketCount *= 2;
		m_Buckets.assign(BucketCount, -1); m_Entries.reserve(Bounds.size() * 8);
		m_MinCells.resize(Bounds.size()); m_NextItems.resize(Bounds.size());
		for (int I = 0; I < (int)Bounds.size(); ++I)
		{
			const auto& B = Bounds[I]; double Size = 0;
			for (int Axis = 0; Axis < 3; ++Axis) Size = std::max(Size, (double)B.Max[Axis] - B.Min[Axis]);
			const int Exponent = Size > 0 ? std::max(MinimumLevel, SizeLevel(Size)) : MinimumLevel;
			int L = 0; while (L < (int)m_Levels.size() && m_Levels[L].Exponent != Exponent) ++L;
			if (L == (int)m_Levels.size()) { Level New; New.Exponent = Exponent; New.InverseSize = std::ldexp(1.0, -Exponent); m_Levels.push_back(New); }
			auto& Layer = m_Levels[L]; m_NextItems[I] = Layer.First; Layer.First = I; ++Layer.Count;
			const Cell Min = CellAt(B.Min, Layer.InverseSize), Max = CellAt(B.Max, Layer.InverseSize); m_MinCells[I] = Min;
			for (int X = Min.X; X <= Max.X; ++X) for (int Y = Min.Y; Y <= Max.Y; ++Y) for (int Z = Min.Z; Z <= Max.Z; ++Z)
			{
				const Cell P = { X, Y, Z }; const size_t Bucket = Hash(P, L) & (BucketCount - 1);
				m_Entries.push_back({ P, L, I, m_Buckets[Bucket] }); m_Buckets[Bucket] = (int)m_Entries.size() - 1;
			}
		}
	}
	void SpatialHashHierarchical::Query(const Box3& Bounds, std::vector<int>& Indices) const
	{
		if (!ValidHashBounds(Bounds)) throw std::invalid_argument("Invalid hierarchical spatial hash query");
		Indices.clear(); if (m_Items.empty() || !m_Bounds.Intersect(Bounds)) return;
		// Clipping prevents coordinate overflow for queries far larger than the mesh.
		const Box3 Clipped(Bounds.Min.Max(m_Bounds.Min), Bounds.Max.Min(m_Bounds.Max));
		for (int L = 0; L < (int)m_Levels.size(); ++L)
		{
			const auto& Layer = m_Levels[L]; const Cell Min = CellAt(Clipped.Min, Layer.InverseSize), Max = CellAt(Clipped.Max, Layer.InverseSize);
			const double CellCount = (double)(Max.X - Min.X + 1) * (Max.Y - Min.Y + 1) * (Max.Z - Min.Z + 1);
			// A huge swept query must not enumerate billions of empty fine cells.
			if (CellCount > Layer.Count)
			{
				for (int I = Layer.First; I >= 0; I = m_NextItems[I]) if (m_Items[I].Intersect(Bounds)) Indices.push_back(I);
				continue;
			}
			for (int X = Min.X; X <= Max.X; ++X) for (int Y = Min.Y; Y <= Max.Y; ++Y) for (int Z = Min.Z; Z <= Max.Z; ++Z)
			{
				const Cell P = { X, Y, Z };
				for (int E = m_Buckets[Hash(P, L) & (m_Buckets.size() - 1)]; E >= 0; E = m_Entries[E].Next)
				{
					const auto& Entry = m_Entries[E]; const auto& First = m_MinCells[Entry.Item];
					if (Entry.Level != L || Entry.Position.X != X || Entry.Position.Y != Y || Entry.Position.Z != Z) continue;
					// Only the first shared cell owns the result; no per-query visited set.
					if (X != std::max(Min.X, First.X) || Y != std::max(Min.Y, First.Y) || Z != std::max(Min.Z, First.Z)) continue;
					if (m_Items[Entry.Item].Intersect(Bounds)) Indices.push_back(Entry.Item);
				}
			}
		}
		std::sort(Indices.begin(), Indices.end());
	}
}
