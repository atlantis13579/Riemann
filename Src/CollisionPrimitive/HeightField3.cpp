
#include "HeightField3.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include "Capsule3.h"
#include "OrientedBox3.h"
#include "Triangle3.h"
#include "Ray3.h"
#include "Sphere3.h"
#include "../Maths/Maths.h"

namespace Riemann
{
	#define X_INDEX(_x)		((int)((_x - BV.Min.x) * InvDX))
	#define Z_INDEX(_z)		((int)((_z - BV.Min.z) * InvDZ))

	static const float HfCellThickness = 0.0001f;

	namespace
	{
		bool FiniteHeightFieldVector(const Vector3& Value)
		{
			return std::isfinite(Value.x) && std::isfinite(Value.y) && std::isfinite(Value.z);
		}

		double BrushWeight(double DistanceSquared, double Radius, bool LinearFalloff)
		{
			return LinearFalloff ? std::max(0.0, 1.0 - std::sqrt(DistanceSquared) / Radius) : 1.0;
		}
	}

	HeightField3::HeightField3()
		: nX(0), nZ(0), BV(Vector3::Zero(), Vector3::Zero()), DX(0.0f), DZ(0.0f), InvDX(0.0f), InvDZ(0.0f)
		, HeightScale(1.0f), Cells(nullptr), Revision(0), mLastEditBounds(Box3::Empty())
	{
	}

	void HeightField3::Init(const Box3& Bounds, int CountX, int CountZ)
	{
		const Box3 NewBounds = Bounds;
		nX = nZ = 0;
		DX = DZ = InvDX = InvDZ = 0.0f;
		BV = Box3(Vector3::Zero(), Vector3::Zero());
		Cells = nullptr;
		mCells.clear();
		Revision = 0;
		mLastEditBounds = Box3::Empty();
		if (CountX < 2 || CountZ < 2 || (uint64_t)CountX * (uint64_t)CountZ > (uint64_t)std::numeric_limits<int>::max()
			|| !FiniteHeightFieldVector(NewBounds.Min) || !FiniteHeightFieldVector(NewBounds.Max)
			|| NewBounds.Max.x <= NewBounds.Min.x || NewBounds.Max.z <= NewBounds.Min.z || NewBounds.Max.y < NewBounds.Min.y) return;
		const double SpacingX = ((double)NewBounds.Max.x - NewBounds.Min.x) / (CountX - 1);
		const double SpacingZ = ((double)NewBounds.Max.z - NewBounds.Min.z) / (CountZ - 1);
		const double Limit = std::numeric_limits<float>::max();
		if (SpacingX > Limit || SpacingZ > Limit || 1.0 / SpacingX > Limit || 1.0 / SpacingZ > Limit) return;
		nX = (uint32_t)CountX;
		nZ = (uint32_t)CountZ;
		BV = NewBounds;
		DX = (float)SpacingX;
		DZ = (float)SpacingZ;
		InvDX = 1.0f / DX;
		InvDZ = 1.0f / DZ;
	}

	void HeightField3::AllocMemory()
	{
		if (nX < 2 || nZ < 2) return;
		mCells.assign((size_t)nX * nZ, CellInfo{});
		Cells = mCells.data();
		++Revision;
		mLastEditBounds = BV;
	}

	bool HeightField3::IsValid() const
	{
		return Cells != nullptr && nX >= 2 && nZ >= 2 && DX > 0.0f && DZ > 0.0f
			&& (uint64_t)nX * nZ <= (uint64_t)std::numeric_limits<int>::max()
			&& std::isfinite(DX) && std::isfinite(DZ) && std::isfinite(InvDX) && std::isfinite(InvDZ) && std::isfinite(HeightScale)
			&& FiniteHeightFieldVector(BV.Min) && FiniteHeightFieldVector(BV.Max)
			&& std::fabs(HeightScale) > 1.0e-12f && std::fabs(HeightScale) <= std::numeric_limits<float>::max() / 32768.0f;
	}

	bool HeightField3::QuantizeHeight(double Height, int16_t& Sample) const
	{
		if (!IsValid() || !std::isfinite(Height)) return false;
		// Clamp in floating point before conversion, including extremely large inputs.
		const double Value = std::max(-32768.0, std::min(32767.0, Height / HeightScale));
		Sample = (int16_t)std::round(Value);
		return true;
	}

	bool HeightField3::GetBrushRange(float CenterX, float CenterZ, float Radius, int& MinX, int& MinZ, int& MaxX, int& MaxZ) const
	{
		if (!IsValid() || !std::isfinite(CenterX) || !std::isfinite(CenterZ) || !std::isfinite(Radius) || Radius <= 0.0f) return false;
		const double Left = (double)CenterX - Radius, Right = (double)CenterX + Radius;
		const double Bottom = (double)CenterZ - Radius, Top = (double)CenterZ + Radius;
		if (Right < BV.Min.x || Left > BV.Max.x || Top < BV.Min.z || Bottom > BV.Max.z) return false;
		MinX = (int)std::max(0.0, std::min((double)nX - 1, std::ceil((Left - BV.Min.x) / DX)));
		MinZ = (int)std::max(0.0, std::min((double)nZ - 1, std::ceil((Bottom - BV.Min.z) / DZ)));
		MaxX = (int)std::max(0.0, std::min((double)nX - 1, std::floor((Right - BV.Min.x) / DX)));
		MaxZ = (int)std::max(0.0, std::min((double)nZ - 1, std::floor((Top - BV.Min.z) / DZ)));
		return MinX <= MaxX && MinZ <= MaxZ;
	}

	void HeightField3::CompleteEdit(int MinX, int MinZ, int MaxX, int MaxZ, const Box3& PreviousBounds, bool RebuildBounds)
	{
		if (RebuildBounds) RecalculateBounds();
		++Revision;
		// A vertex changes the triangles on either side, including the old support
		// surface when terrain is lowered. Keep a one-cell horizontal halo.
		MinX = std::max(0, MinX - 1);
		MinZ = std::max(0, MinZ - 1);
		MaxX = std::min((int)nX - 1, MaxX + 1);
		MaxZ = std::min((int)nZ - 1, MaxZ + 1);
		float MinHeight = RebuildBounds ? PreviousBounds.Min.y : FLT_MAX;
		float MaxHeight = RebuildBounds ? PreviousBounds.Max.y : -FLT_MAX;
		for (int z = MinZ; z <= MaxZ; ++z)
		for (int x = MinX; x <= MaxX; ++x)
		{
			const float Height = GetHeight(z * nX + x);
			MinHeight = std::min(MinHeight, Height);
			MaxHeight = std::max(MaxHeight, Height);
		}
		mLastEditBounds = Box3(Vector3(BV.Min.x + DX * MinX, MinHeight, BV.Min.z + DZ * MinZ),
			Vector3(BV.Min.x + DX * MaxX, MaxHeight, BV.Min.z + DZ * MaxZ));
	}

	bool HeightField3::SetHeight(int x, int z, float height)
	{
		int16_t Sample;
		return QuantizeHeight(height, Sample) && SetHeightRaw(x, z, Sample);
	}

	bool HeightField3::SetHeightRaw(int x, int z, int16_t heightSample)
	{
		if (!IsValid() || x < 0 || z < 0 || x >= (int)nX || z >= (int)nZ)
		{
			return false;
		}
		CellInfo& cell = Cells[z * nX + x];
		if (cell.HeightSample == heightSample)
		{
			return true;
		}
		const Vector3 PreviousSample(0.0f, GetHeight(z * nX + x), 0.0f);
		const Box3 PreviousBounds(PreviousSample, PreviousSample);
		cell.HeightSample = heightSample;
		CompleteEdit(x, z, x, z, PreviousBounds);
		return true;
	}

	int HeightField3::SetHeightsRect(int X, int Z, int Width, int Height, const std::vector<float>& Heights)
	{
		if (!IsValid() || Width <= 0 || Height <= 0 || (uint64_t)Width * (uint64_t)Height != Heights.size()) return 0;
		for (float Value : Heights) if (!std::isfinite(Value)) return 0;
		const int MinX = (int)std::max<int64_t>(0, X), MinZ = (int)std::max<int64_t>(0, Z);
		const int MaxX = (int)std::min<int64_t>((int64_t)nX - 1, (int64_t)X + Width - 1);
		const int MaxZ = (int)std::min<int64_t>((int64_t)nZ - 1, (int64_t)Z + Height - 1);
		if (MinX > MaxX || MinZ > MaxZ) return 0;
		Box3 PreviousBounds = Box3::Empty();
		int Changed = 0, ChangedMinX = MaxX, ChangedMinZ = MaxZ, ChangedMaxX = MinX, ChangedMaxZ = MinZ;
		for (int z = MinZ; z <= MaxZ; ++z)
		for (int x = MinX; x <= MaxX; ++x)
		{
			int16_t Sample;
			QuantizeHeight(Heights[(size_t)((int64_t)z - Z) * Width + (size_t)((int64_t)x - X)], Sample);
			CellInfo& Cell = Cells[(size_t)z * nX + x];
			if (Cell.HeightSample == Sample) continue;
			PreviousBounds.Encapsulate(Vector3(0.0f, (float)Cell.HeightSample * HeightScale, 0.0f));
			Cell.HeightSample = Sample;
			++Changed;
			ChangedMinX = std::min(ChangedMinX, x); ChangedMinZ = std::min(ChangedMinZ, z);
			ChangedMaxX = std::max(ChangedMaxX, x); ChangedMaxZ = std::max(ChangedMaxZ, z);
		}
		if (Changed) CompleteEdit(ChangedMinX, ChangedMinZ, ChangedMaxX, ChangedMaxZ, PreviousBounds);
		return Changed;
	}

	int HeightField3::ModifyHeightCircle(float centerX, float centerZ, float radius, float heightDelta, bool linearFalloff)
	{
		int MinX, MinZ, MaxX, MaxZ;
		if (!std::isfinite(heightDelta) || heightDelta == 0.0f || !GetBrushRange(centerX, centerZ, radius, MinX, MinZ, MaxX, MaxZ)) return 0;
		Box3 PreviousBounds = Box3::Empty();
		const double radiusSquared = (double)radius * radius;
		int modifiedCount = 0;
		int ChangedMinX = MaxX, ChangedMinZ = MaxZ, ChangedMaxX = MinX, ChangedMaxZ = MinZ;
		for (int z = MinZ; z <= MaxZ; ++z)
		{
			for (int x = MinX; x <= MaxX; ++x)
			{
				const double dx = (double)BV.Min.x + (double)DX * x - centerX;
				const double dz = (double)BV.Min.z + (double)DZ * z - centerZ;
				const double distanceSquared = dx * dx + dz * dz;
				if (distanceSquared > radiusSquared)
				{
					continue;
				}
				const double weight = BrushWeight(distanceSquared, radius, linearFalloff);
				const int index = (int)(z * nX + x);
				int16_t quantized;
				QuantizeHeight((double)GetHeight(index) + heightDelta * weight, quantized);
				if (Cells[index].HeightSample != quantized)
				{
					PreviousBounds.Encapsulate(Vector3(0.0f, GetHeight(index), 0.0f));
					Cells[index].HeightSample = quantized;
					++modifiedCount;
					ChangedMinX = std::min(ChangedMinX, x); ChangedMinZ = std::min(ChangedMinZ, z);
					ChangedMaxX = std::max(ChangedMaxX, x); ChangedMaxZ = std::max(ChangedMaxZ, z);
				}
			}
		}
		if (modifiedCount > 0)
		{
			CompleteEdit(ChangedMinX, ChangedMinZ, ChangedMaxX, ChangedMaxZ, PreviousBounds);
		}
		return modifiedCount;
	}

	int HeightField3::FillHeightCircle(float CenterX, float CenterZ, float Radius, float TargetHeight, float Strength, bool LinearFalloff)
	{
		int MinX, MinZ, MaxX, MaxZ;
		if (!std::isfinite(TargetHeight) || !std::isfinite(Strength) || Strength <= 0.0f || Strength > 1.0f
			|| !GetBrushRange(CenterX, CenterZ, Radius, MinX, MinZ, MaxX, MaxZ)) return 0;
		Box3 PreviousBounds = Box3::Empty();
		int Changed = 0, ChangedMinX = MaxX, ChangedMinZ = MaxZ, ChangedMaxX = MinX, ChangedMaxZ = MinZ;
		for (int z = MinZ; z <= MaxZ; ++z)
		for (int x = MinX; x <= MaxX; ++x)
		{
			const double DeltaX = (double)BV.Min.x + (double)DX * x - CenterX;
			const double DeltaZ = (double)BV.Min.z + (double)DZ * z - CenterZ;
			const double DistanceSquared = DeltaX * DeltaX + DeltaZ * DeltaZ;
			if (DistanceSquared > (double)Radius * Radius) continue;
			const double Weight = Strength * BrushWeight(DistanceSquared, Radius, LinearFalloff);
			CellInfo& Cell = Cells[(size_t)z * nX + x];
			const double Height = (double)Cell.HeightSample * HeightScale;
			int16_t Sample;
			QuantizeHeight(Height + ((double)TargetHeight - Height) * Weight, Sample);
			if (Cell.HeightSample == Sample) continue;
			PreviousBounds.Encapsulate(Vector3(0.0f, (float)Height, 0.0f));
			Cell.HeightSample = Sample;
			++Changed;
			ChangedMinX = std::min(ChangedMinX, x); ChangedMinZ = std::min(ChangedMinZ, z);
			ChangedMaxX = std::max(ChangedMaxX, x); ChangedMaxZ = std::max(ChangedMaxZ, z);
		}
		if (Changed) CompleteEdit(ChangedMinX, ChangedMinZ, ChangedMaxX, ChangedMaxZ, PreviousBounds);
		return Changed;
	}

	int HeightField3::SmoothHeightCircle(float CenterX, float CenterZ, float Radius, float Strength, int Iterations, bool LinearFalloff)
	{
		int MinX, MinZ, MaxX, MaxZ;
		if (!std::isfinite(Strength) || Strength <= 0.0f || Strength > 1.0f || Iterations <= 0
			|| !GetBrushRange(CenterX, CenterZ, Radius, MinX, MinZ, MaxX, MaxZ)) return 0;
		const int ReadMinX = std::max(0, MinX - 1), ReadMinZ = std::max(0, MinZ - 1);
		const int ReadMaxX = std::min((int)nX - 1, MaxX + 1), ReadMaxZ = std::min((int)nZ - 1, MaxZ + 1);
		const size_t Width = (size_t)(ReadMaxX - ReadMinX + 1);
		std::vector<double> Previous(Width * (ReadMaxZ - ReadMinZ + 1));
		for (int z = ReadMinZ; z <= ReadMaxZ; ++z)
		for (int x = ReadMinX; x <= ReadMaxX; ++x)
			Previous[(size_t)(z - ReadMinZ) * Width + x - ReadMinX] = (double)Cells[(size_t)z * nX + x].HeightSample * HeightScale;
		std::vector<double> Current = Previous;
		// Jacobi updates read a snapshot, so the result is independent of traversal.
		// Only the brush and its one-sample read halo need scratch storage.
		for (int Iteration = 0; Iteration < Iterations; ++Iteration)
		{
			for (int z = MinZ; z <= MaxZ; ++z)
			for (int x = MinX; x <= MaxX; ++x)
			{
				const double DeltaX = (double)BV.Min.x + (double)DX * x - CenterX;
				const double DeltaZ = (double)BV.Min.z + (double)DZ * z - CenterZ;
				const double DistanceSquared = DeltaX * DeltaX + DeltaZ * DeltaZ;
				if (DistanceSquared > (double)Radius * Radius) continue;
				double Sum = 0.0;
				int Count = 0;
				for (int zz = std::max(ReadMinZ, z - 1); zz <= std::min(ReadMaxZ, z + 1); ++zz)
				for (int xx = std::max(ReadMinX, x - 1); xx <= std::min(ReadMaxX, x + 1); ++xx)
				{
					Sum += Previous[(size_t)(zz - ReadMinZ) * Width + xx - ReadMinX];
					++Count;
				}
				const size_t Index = (size_t)(z - ReadMinZ) * Width + x - ReadMinX;
				const double Weight = Strength * BrushWeight(DistanceSquared, Radius, LinearFalloff);
				Current[Index] = Previous[Index] + (Sum / Count - Previous[Index]) * Weight;
			}
			Previous.swap(Current);
		}
		Box3 PreviousBounds = Box3::Empty();
		int Changed = 0, ChangedMinX = MaxX, ChangedMinZ = MaxZ, ChangedMaxX = MinX, ChangedMaxZ = MinZ;
		for (int z = MinZ; z <= MaxZ; ++z)
		for (int x = MinX; x <= MaxX; ++x)
		{
			int16_t Sample;
			QuantizeHeight(Previous[(size_t)(z - ReadMinZ) * Width + x - ReadMinX], Sample);
			CellInfo& Cell = Cells[(size_t)z * nX + x];
			if (Cell.HeightSample == Sample) continue;
			PreviousBounds.Encapsulate(Vector3(0.0f, (float)Cell.HeightSample * HeightScale, 0.0f));
			Cell.HeightSample = Sample;
			++Changed;
			ChangedMinX = std::min(ChangedMinX, x); ChangedMinZ = std::min(ChangedMinZ, z);
			ChangedMaxX = std::max(ChangedMaxX, x); ChangedMaxZ = std::max(ChangedMaxZ, z);
		}
		if (Changed) CompleteEdit(ChangedMinX, ChangedMinZ, ChangedMaxX, ChangedMaxZ, PreviousBounds);
		return Changed;
	}

	int HeightField3::DigCrater(float centerX, float centerZ, float radius, float depth, bool linearFalloff)
	{
		return ModifyHeightCircle(centerX, centerZ, radius, -std::abs(depth), linearFalloff);
	}

	bool HeightField3::SetTriangleHole(int cellX, int cellZ, int triangleIndex, bool hole)
	{
		if (!IsValid() || cellX < 0 || cellZ < 0 || cellX >= (int)nX - 1 || cellZ >= (int)nZ - 1 || triangleIndex < 0 || triangleIndex > 1)
		{
			return false;
		}
		CellInfo& cell = Cells[cellZ * nX + cellX];
		uint8_t& flag = triangleIndex == 0 ? cell.Tessellation0 : cell.Tessellation1;
		const uint8_t newFlag = (flag & 0x80) | (hole ? 0x7F : 0x00);
		if (flag != newFlag)
		{
			flag = newFlag;
			CompleteEdit(cellX, cellZ, cellX + 1, cellZ + 1, BV, false);
		}
		return true;
	}

	bool HeightField3::IsTriangleHole(int cellX, int cellZ, int triangleIndex) const
	{
		if (Cells == nullptr || cellX < 0 || cellZ < 0 || cellX >= (int)nX - 1 || cellZ >= (int)nZ - 1 || triangleIndex < 0 || triangleIndex > 1)
		{
			return false;
		}
		const CellInfo& cell = Cells[cellZ * nX + cellX];
		return ((triangleIndex == 0 ? cell.Tessellation0 : cell.Tessellation1) & 0x7F) == 0x7F;
	}

	void HeightField3::RecalculateBounds()
	{
		if (!IsValid())
		{
			return;
		}
		float minHeight = FLT_MAX;
		float maxHeight = -FLT_MAX;
		for (uint32_t i = 0; i < nX * nZ; ++i)
		{
			const float height = GetHeight((int)i);
			minHeight = std::min(minHeight, height);
			maxHeight = std::max(maxHeight, height);
		}
		BV.Min.y = minHeight;
		BV.Max.y = maxHeight;
	}

	bool HeightField3::IntersectRayCell(const Vector3& Origin, const Vector3& Direction, int i, int j, const HeightFieldHitOption& Option, HeightFieldHitResult* Result) const
	{
		float minH, maxH;
		if (!GetHeightRange(i, j, minH, maxH)) return false;

		Result->AddTestCount(1);

		if (fabsf(Direction.y) < 0.000001f)
		{
			if (Origin.y < minH || Origin.y > maxH)
			{
				return false;
			}
		}

		{
			Vector3 Bmin(BV.Min.x + i * DX, minH, BV.Min.z + j * DZ);
			Vector3 Bmax(BV.Min.x + (i + 1) * DX, maxH, BV.Min.z + (j + 1) * DZ);
			float t0, t1;
			if (!Ray3::RayIntersectAABB2(Origin, Direction, Bmin, Bmax, HfCellThickness, Option.maxDist, &t0, &t1))
			{
				return false;
			}
		}

		bool hit = false;
		float min_t = FLT_MAX;

		Vector3 Tris[6];
		int nT = GetCellTriangle(i, j, Tris);
		for (int k = 0; k < nT; k += 3)
		{
			float tt;
			if (Triangle3::RayIntersectTriangle(Origin, Direction, Tris[k], Tris[k + 1], Tris[k + 2], &tt) && tt < min_t)
			{
				min_t = tt;
				if (min_t < Option.maxDist)
				{
					Result->hitTime = min_t;
					Result->cellIndex = j * nX + i;
					hit = true;
				}
			}
		}

		Result->AddTestCount(nT / 3);

		return hit;
	}

	bool HeightField3::IntersectRayY(const Vector3& Origin, const Vector3& Direction, const HeightFieldHitOption& Option, HeightFieldHitResult* Result) const
	{
		const int i = X_INDEX(Origin.x);
		const int j = Z_INDEX(Origin.z);
		if (i < 0 || i >= (int)nX - 1 || j < 0 || j >= (int)nZ - 1)
		{
			return false;
		}

		bool hit = false;
		float min_t = FLT_MAX;

		Vector3 Tris[6];
		int nT = GetCellTriangle(i, j, Tris);
		for (int k = 0; k < nT; k += 3)
		{
			float tt;
			if (Triangle3::RayIntersectTriangle(Origin, Direction, Tris[k], Tris[k + 1], Tris[k + 2], &tt) && tt < min_t)
			{
				min_t = tt;
				if (min_t < Option.maxDist)
				{
					Result->hitTime = min_t;
					Result->cellIndex = j * nX + i;
					hit = true;
				}
			}
		}

		Result->AddTestCount(nT / 3);

		return hit;
	}

	bool HeightField3::IntersectRayBruteForce(const Vector3& Origin, const Vector3& Direction, const HeightFieldHitOption& Option, HeightFieldHitResult* Result) const
	{
		float min_dist = FLT_MAX;
		for (uint32_t i = 0; i < nX - 1; ++i)
		for (uint32_t j = 0; j < nZ - 1; ++j)
		{
			if (IntersectRayCell(Origin, Direction, i, j, Option, Result))
			{
				if (Result->hitTime < min_dist)
				{
					min_dist = Result->hitTime;
				}
			}
		}
		Result->hitTime = min_dist;
		return min_dist != FLT_MAX;
	}

	bool HeightField3::IntersectRay(const Vector3& Origin, const Vector3& Direction, const HeightFieldHitOption& Option, HeightFieldHitResult* Result) const
	{
		if (!IsValid() || Result == nullptr || !FiniteHeightFieldVector(Origin) || !FiniteHeightFieldVector(Direction)) return false;
		if (Origin.y > BV.Max.y && Direction.y >= 0)
		{
			return false;
		}
		else if (Origin.y < BV.Min.y && Direction.y <= 0)
		{
			return false;
		}

		if (Direction.ParallelTo(Vector3::UnitY()))
		{
			// Handle the most common case
			return IntersectRayY(Origin, Direction, Option, Result);
		}

		Result->AddTestCount(1);

		float t0, t1;
		if (!Ray3::RayIntersectAABB2(Origin, Direction, BV.Min, BV.Max, HfCellThickness, Option.maxDist, &t0, &t1))
		{
			return false;
		}

		Vector3 P0 = Origin + Direction * t0;
		Vector3 P1 = Origin + Direction * t1;

		const int istart = X_INDEX(P0.x);
		const int jstart = Z_INDEX(P0.z);
		const int iend = X_INDEX(P1.x);
		const int jend = Z_INDEX(P1.z);
		const int di = Direction.x > 0 ? 1 : (Direction.x < 0 ? -1 : 0);
		const int dj = Direction.z > 0 ? 1 : (Direction.z < 0 ? -1 : 0);

		if (istart < 0 || istart >= (int)nX - 1 || jstart < 0 || jstart >= (int)nZ - 1)
		{
			return false;
		}

		if (iend < 0 || iend >= (int)nX - 1 || jend < 0 || jend >= (int)nZ - 1)
		{
			return false;
		}

		if (istart == iend && jstart == jend)
		{
			return IntersectRayCell(Origin, Direction, istart, jstart, Option, Result);
		}

		int i = istart;
		int j = jstart;

		if (istart != iend && jstart != jend)
		{
			const float minx = BV.Min.x + DX * istart;
			const float maxx = minx + DX;
			const float minz = BV.Min.z + DZ * jstart;
			const float maxz = minz + DZ;

			const float dtx = DX / fabsf(P1.x - P0.x);
			const float dty = DZ / fabsf(P1.z - P0.z);

			float tx = ((P0.x > P1.x) ? (P0.x - minx) : (maxx - P0.x)) / fabsf(P1.x - P0.x);
			float ty = ((P0.z > P1.z) ? (P0.z - minz) : (maxz - P0.z)) / fabsf(P1.z - P0.z);

			while (i != iend || j != jend)
			{
				if (IntersectRayCell(Origin, Direction, i, j, Option, Result))
				{
					return true;
				}

				if (tx <= ty)
				{
					if (i == iend)
						break;
					tx += dtx;
					i += di;
				}
				else
				{
					if (j == jend)
						break;
					ty += dty;
					j += dj;
				}
			}
		}
		else if (istart == iend)
		{
			while (j != jend)
			{
				if (IntersectRayCell(Origin, Direction, i, j, Option, Result))
				{
					return true;
				}
				j += dj;
			}
		}
		else
		{
			// jstart == jend
			while (i != iend)
			{
				if (IntersectRayCell(Origin, Direction, i, j, Option, Result))
				{
					return true;
				}
				i += di;
			}
		}

		return IntersectRayCell(Origin, Direction, iend, jend, Option, Result);
	}


	bool HeightField3::IntersectRay(const Vector3& Origin, const Vector3& Direction, float* t) const
	{
		HeightFieldHitOption Option;
		Option.hitBothSides = true;
		Option.maxDist = FLT_MAX;

		HeightFieldHitResult Result = { 0 };
		if (IntersectRay(Origin, Direction, Option, &Result))
		{
			*t = Result.hitTime;
			return true;
		}
		return false;
	}

	bool HeightField3::IntersectAABB(const Vector3& Bmin, const Vector3& Bmax) const
	{
		Box3 Intersect;
		if (!BV.GetIntersection(Box3(Bmin, Bmax), Intersect))
		{
			return false;
		}

		float minH = Bmin.y;
		float maxH = Bmax.y;

		const int i0 = X_INDEX(Intersect.Min.x);
		const int j0 = Z_INDEX(Intersect.Min.z);
		const int i1 = X_INDEX(Intersect.Max.x);
		const int j1 = Z_INDEX(Intersect.Max.z);
		for (int i = i0; i <= i1; ++i)
		for (int j = j0; j <= j1; ++j)
		{
			float H = GetHeight(i * nZ + j);
			if (minH <= H && H <= maxH)
			{
				return true;
			}
		}

		return true;
	}

	template<class Shape>
	bool IntersectHF(const HeightField3* hf, const Shape& shape)
	{
		const Box3 b = shape.CalculateBoundingVolume();
		const Box3& BV = hf->BV;
		const float InvDX = hf->InvDX;
		const float InvDZ = hf->InvDZ;

		Box3 Intersect;
		if (!BV.GetIntersection(b, Intersect))
		{
			return false;
		}

		const int i0 = X_INDEX(Intersect.Min.x);
		const int j0 = Z_INDEX(Intersect.Min.z);
		const int i1 = X_INDEX(Intersect.Max.x);
		const int j1 = Z_INDEX(Intersect.Max.z);
		for (int i = i0; i <= i1; ++i)
		for (int j = j0; j <= j1; ++j)
		{
			Vector3 Tris[6];
			int n = hf->GetCellTriangle(i, j, Tris);
			for (int k = 0; k < n; k += 3)
			{
				if (shape.IntersectTriangle(Tris[k], Tris[k + 1], Tris[k + 2]))
				{
					return true;
				}
			}
		}

		return false;
	}

	bool HeightField3::IntersectOBB(const Vector3& Center, const Vector3& Extent, const Matrix3& rot) const
	{
		OrientedBox3 obb(Center, Extent, rot);
		return IntersectHF(this, obb);
	}

	bool HeightField3::IntersectSphere(const Vector3& Center, float Radius) const
	{
		Sphere3 sphere(Center, Radius);
		return IntersectHF(this, sphere);
	}

	bool HeightField3::IntersectCapsule(const Vector3& X0, const Vector3& X1, float Radius) const
	{
		Capsule3 capsule(X0, X1, Radius);
		return IntersectHF(this, capsule);
	}

	bool HeightField3::CalculateVolumeProperties(MassParameters* p, float Density) const
	{
		p->Volume = 0.0f;
		p->Mass = 0.0f;
		p->CenterOfMass = BV.GetCenter();
		p->BoundingVolume = BV;
		p->InertiaMat = Matrix3::Identity();
		return true;
	}

	bool HeightField3::GetCellBV(int i, int j, Box3& box) const
	{
		float minH, maxH;
		if (!GetHeightRange(i, j, minH, maxH))
		{
			return false;
		}
		box.Min = Vector3(BV.Min.x + DX * (i + 0), minH, BV.Min.z + DZ * (j + 0));
		box.Max = Vector3(BV.Min.x + DX * (i + 1), maxH, BV.Min.z + DZ * (j + 1));
		return true;
	}

	bool HeightField3::GetHeightRange(int i, int j, float& minH, float& maxH) const
	{
		if (!IsValid() || i < 0 || j < 0 || i >= (int)nX - 1 || j >= (int)nZ - 1) return false;

		bool tessFlag = Cells[i + j * nX].Tessellation0 & 0x80;
		uint32_t i0 = j * nX + i;
		uint32_t i1 = j * nX + i + 1;
		uint32_t i2 = (j + 1) * nX + i;
		uint32_t i3 = (j + 1) * nX + i + 1;
		// i2---i3
		// |    |
		// |    |
		// i0---i1
		uint8_t Hole0 = Cells[i + j * nX].Tessellation0 & 0x7F;
		uint8_t Hole1 = Cells[i + j * nX].Tessellation1 & 0x7F;

		minH = FLT_MAX;
		maxH = -FLT_MAX;

		float Height0 = GetHeight(i0);
		float Height1 = GetHeight(i1);
		float Height2 = GetHeight(i2);
		float Height3 = GetHeight(i3);

		if (Hole0 != 0x7F && Hole1 != 0x7F)
		{
			minH = Maths::Min(Height0, Height1, Height2, Height3);
			maxH = Maths::Max(Height0, Height1, Height2, Height3);
			return true;
		}
		else if (Hole0 != 0x7F)
		{
			minH = Maths::Min(Maths::Min(Height0, Height2), (tessFlag ? Height3 : Height1));
			maxH = Maths::Max(Maths::Max(Height0, Height2), (tessFlag ? Height3 : Height1));
			return true;
		}
		else if (Hole1 != 0x7F)
		{
			minH = Maths::Min(Maths::Min(Height1, Height3), (tessFlag ? Height0 : Height2));
			maxH = Maths::Max(Maths::Max(Height1, Height3), (tessFlag ? Height0 : Height2));
			return true;
		}

		return false;
	}

	int HeightField3::GetCellTriangle(int i, int j, Vector3 Tris[6]) const
	{
		if (!IsValid() || i < 0 || j < 0 || i >= (int)nX - 1 || j >= (int)nZ - 1) return 0;

		bool tessFlag = Cells[i + j * nX].Tessellation0 & 0x80;
		uint32_t i0 = j * nX + i;
		uint32_t i1 = j * nX + i + 1;
		uint32_t i2 = (j + 1) * nX + i;
		uint32_t i3 = (j + 1) * nX + i + 1;
		// i2---i3
		// |    |
		// |    |
		// i0---i1
		uint8_t Hole0 = Cells[i + j * nX].Tessellation0 & 0x7F;
		uint8_t Hole1 = Cells[i + j * nX].Tessellation1 & 0x7F;

		Vector3 Base = Vector3(BV.Min.x + DX * i, 0.0f, BV.Min.z + DZ * j);

		float Height0 = GetHeight(i0);
		float Height1 = GetHeight(i1);
		float Height2 = GetHeight(i2);
		float Height3 = GetHeight(i3);

		int nt = 0;
		if (Hole0 != 0x7F)
		{
			Tris[0] = Base + Vector3(0.0f, Height2, DZ);
			Tris[1] = Base + Vector3(0.0f, Height0, 0.0f);
			Tris[2] = tessFlag ? Base + Vector3(DX, Height3, DZ) : Base + Vector3(DX, Height1, 0.0f);
			nt += 3;
		}
		if (Hole1 != 0x7F)
		{
			Tris[nt + 0] = Base + Vector3(DX, Height3, DZ);
			Tris[nt + 1] = tessFlag ? Base + Vector3(0.0f, Height0, 0.0f) : Base + Vector3(0.0f, Height2, DZ);
			Tris[nt + 2] = Base + Vector3(DX, Height1, 0.0f);
			nt += 3;
		}

		return nt;
	}

	int HeightField3::GetCellTriangle(int i, int j, uint32_t Tris[6]) const
	{
		if (!IsValid() || i < 0 || j < 0 || i >= (int)nX - 1 || j >= (int)nZ - 1) return 0;

		bool tessFlag = Cells[i + j * nX].Tessellation0 & 0x80;
		uint32_t i0 = j * nX + i;
		uint32_t i1 = j * nX + i + 1;
		uint32_t i2 = (j + 1) * nX + i;
		uint32_t i3 = (j + 1) * nX + i + 1;
		// i2---i3
		// |    |
		// |    |
		// i0---i1
		uint8_t Hole0 = Cells[i + j * nX].Tessellation0 & 0x7F;
		uint8_t Hole1 = Cells[i + j * nX].Tessellation1 & 0x7F;

		int nt = 0;
		if (Hole0 != 0x7F)
		{
			Tris[0] = i2;
			Tris[1] = i0;
			Tris[2] = tessFlag ? i3 : i1;
			nt += 3;
		}
		if (Hole1 != 0x7F)
		{
			Tris[nt + 0] = i3;
			Tris[nt + 1] = tessFlag ? i0 : i2;
			Tris[nt + 2] = i1;
			nt += 3;
		}

		return nt;
	}

	void HeightField3::GetMesh(std::vector<Vector3>& Vertices, std::vector<uint16_t>& Indices, std::vector<Vector3>& Normals)
	{
		Vertices.clear();
		Indices.clear();
		Normals.clear();

		if (Cells == nullptr || nX < 2 || nZ < 2)
		{
			return;
		}

		Vertices.resize(nX * nZ);
		for (uint32_t j = 0; j < nZ; j++)
		for (uint32_t i = 0; i < nX; i++)
		{
			const uint32_t idx = j * nX + i;
			Vertices[idx] = Vector3(BV.Min.x + DX * i, GetHeight(idx), BV.Min.z + DZ * j);
		}

		assert(Vertices.size() < 65535);
		Indices.reserve((nZ - 1) * (nX - 1) * 2 * 3);
		uint32_t Tris[6];

		for (uint32_t j = 0; j < (nZ - 1); ++j)
		for (uint32_t i = 0; i < (nX - 1); ++i)
		{
			int nT = GetCellTriangle(i, j, Tris);
			for (int k = 0; k < nT; k += 3)
			{
				Indices.push_back((uint16_t)Tris[k + 0]);
				Indices.push_back((uint16_t)Tris[k + 2]);
				Indices.push_back((uint16_t)Tris[k + 1]);
			}
		}

		std::vector<int> Count;
		Count.resize(Vertices.size(), 0);
		Normals.resize(Vertices.size(), Vector3::Zero());
		const int nTris = (int)Indices.size() / 3;
		for (int i = 0; i < nTris; ++i)
		{
			uint16_t i0 = Indices[3 * i + 0];
			uint16_t i1 = Indices[3 * i + 1];
			uint16_t i2 = Indices[3 * i + 2];
			const Vector3& v0 = Vertices[i0];
			const Vector3& v1 = Vertices[i1];
			const Vector3& v2 = Vertices[i2];
			Vector3 Nor = (v1 - v0).Cross(v2 - v0);
			if (Nor.SafeNormalize() == 0.0f)
			{
				continue;
			}
			Normals[i0] += Nor; Count[i0]++;
			Normals[i1] += Nor; Count[i1]++;
			Normals[i2] += Nor; Count[i2]++;
		}

		for (size_t i = 0; i < Normals.size(); ++i)
		{
			if (Count[i] > 0)
			{
				Normals[i] *= 1.0f / Count[i];
				if (Normals[i].SafeNormalize() > 0.0f)
				{
					continue;
				}
			}
			Normals[i] = Vector3::UnitY();
		}
	}

	void HeightField3::GetWireframe(std::vector<Vector3>& Vertices, std::vector<uint16_t>& Indices)
	{

	}
}
