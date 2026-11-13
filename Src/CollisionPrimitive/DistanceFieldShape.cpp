#include "DistanceFieldShape.h"
#include "StaticMesh.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <tuple>

namespace Riemann
{
	/* A simulation SDF represents the negative region of a closed sampled field.
	   Positive grid boundaries make the finite domain unambiguous. Cooking owns an
	   immutable snapshot; replacing a geometry must also update its volume properties
	   so broad-phase bounds, mass and persistent contact revisions change together.

	   Collision samples are exact zero crossings of grid edges. They are selected
	   through the field hierarchy and refined against the trilinear field, never
	   against the display mesh. Newton projection preserves the analytic gradient
	   magnitude and limits steps in cell units. A medial-axis or escaped projection
	   reports failure; callers may use the globally bounded closest-surface query.

	   Volume and inertia use uniform subcell quadrature of the negative region.
	   The inertia includes each occupied subcell's intrinsic inertia and a parallel
	   axis shift to the computed centroid. These are sampled-volume approximations;
	   increasing grid resolution or MassSamplesPerAxis improves the integration.
	*/
	namespace
	{
		const int Dimensions = 3;
		const int CornerCount = 1 << Dimensions;
		const double CuboidInertiaDivisor = 12.0;
		bool Finite(const Vector3& P) { return std::isfinite(P.x) && std::isfinite(P.y) && std::isfinite(P.z); }
	}
	bool DistanceFieldShapeSettings::IsValid() const
	{
		return ProjectionIterations > 0 && MassSamplesPerAxis > 0 && WitnessRefinementIterations > 0 && MaximumSweepIterations > 0
			&& std::isfinite(PatchNormalCosine) && PatchNormalCosine > 0.0f && PatchNormalCosine <= 1.0f
			&& std::isfinite(MinimumFacingCosine) && MinimumFacingCosine >= 0.0f && MinimumFacingCosine < 1.0f
			&& std::isfinite(ManifoldDepthToleranceCells) && ManifoldDepthToleranceCells >= 0.0f
			&& std::isfinite(RelativeSurfaceTolerance) && RelativeSurfaceTolerance > 0.0f
			&& std::isfinite(RelativeSweepTolerance) && RelativeSweepTolerance > 0.0f
			&& std::isfinite(ProjectionStepCells) && ProjectionStepCells > 0.0f;
	}
	struct DistanceFieldShape::Storage
	{
		RuntimeDistanceField Field;
		DistanceFieldShapeSettings Settings;
		StaticMesh Display;
		MassParameters UnitDensity;
		std::vector<DistanceFieldSurfaceSample> SurfaceSamples;
	};
	bool DistanceFieldShape::Build(const RuntimeDistanceField& Field, const DistanceFieldShapeSettings& Settings, std::string* Error)
	{
		auto Fail = [&](const char* Message) { if (Error) *Error = Message; return false; };
		if (!Settings.IsValid() || !Field.IsValid() || !Field.HasClosedBoundary() || Field.GetSurfaceCellCount() == 0)
			return Fail("A simulation SDF needs valid settings, a zero surface and strictly positive grid boundaries");
		const auto& Grid = *Field.GetField();
		if (!std::isfinite(Field.GetCellSize() * Settings.RelativeSurfaceTolerance)
			|| !std::isfinite(Field.GetCellSize() * Settings.RelativeSweepTolerance)
			|| !std::isfinite(Field.GetCellSize() * Settings.ProjectionStepCells)) return Fail("Invalid scaled SDF tolerances");
		auto Data = std::make_shared<Storage>(); Data->Field = Field; Data->Settings = Settings;
		std::vector<Vector3> Vertices; std::vector<Index3> Triangles;
		if (!Grid.ExtractIsoSurface(0.0f, Vertices, Triangles) || Triangles.empty()) return Fail("SDF display surface extraction failed");
		Data->Display.SetData(Vertices.data(), Triangles.data(), (uint32_t)Vertices.size(), (uint32_t)Triangles.size(), false, true);
		Data->Display.Compact();
		// A trilinear zero surface reaches its coordinate extrema on cell edges:
		// each cross-section is bilinear, whose extrema occur at its corners.
		// Bound those crossings, excluding the positive padding of the cook grid.
		DistanceFieldShape Candidate; Candidate.m_Data = Data;
		std::vector<DistanceFieldSurfaceSample> SurfaceSamples;
		Candidate.GetSurfaceSamples(Field.GetBounds(), SurfaceSamples);
		if (SurfaceSamples.empty()) return Fail("SDF has no bounded zero crossings");
		std::sort(SurfaceSamples.begin(), SurfaceSamples.end(), [](const auto& A, const auto& B)
		{
			return std::tie(A.Position.x, A.Position.y, A.Position.z, A.Feature) < std::tie(B.Position.x, B.Position.y, B.Position.z, B.Feature);
		});
		SurfaceSamples.erase(std::unique(SurfaceSamples.begin(), SurfaceSamples.end(), [](const auto& A, const auto& B)
			{ return A.Position == B.Position; }), SurfaceSamples.end());
		Data->SurfaceSamples = SurfaceSamples;
		Box3 SurfaceBounds(SurfaceSamples.front().Position, SurfaceSamples.front().Position);
		for (const auto& Sample : SurfaceSamples)
		{
			SurfaceBounds.Min = SurfaceBounds.Min.Min(Sample.Position);
			SurfaceBounds.Max = SurfaceBounds.Max.Max(Sample.Position);
		}
		SurfaceBounds.Thicken(Candidate.GetSurfaceTolerance());
		const Vector3 Reference = Field.GetBounds().GetCenter();
		double Volume = 0.0, First[Dimensions] = {}, Second[Dimensions][Dimensions] = {};
		const int Count = Settings.MassSamplesPerAxis;
		for (int Z = 0; Z < Grid.GetCellsZ(); ++Z) for (int Y = 0; Y < Grid.GetCellsY(); ++Y) for (int X = 0; X < Grid.GetCellsX(); ++X)
		{
			float Min, Max; Field.GetCellRange(Index3(X, Y, Z), Min, Max); if (Min >= 0.0f) continue;
			const Vector3 Low = Grid.GridPointToWorld(X, Y, Z), High = Grid.GridPointToWorld(X + 1, Y + 1, Z + 1);
			const int Samples = Max < 0.0f ? 1 : Count;
			const Vector3 Size = (High - Low) / (float)Samples;
			const double CellVolume = (double)Size.x * Size.y * Size.z;
			for (int K = 0; K < Samples; ++K) for (int J = 0; J < Samples; ++J) for (int I = 0; I < Samples; ++I)
			{
				const Vector3 P = Low + Vector3((I + 0.5f) * Size.x, (J + 0.5f) * Size.y, (K + 0.5f) * Size.z);
				DistanceFieldSample Sample; if (!Field.Sample(P, Sample) || Sample.Value >= 0.0f) continue;
				Volume += CellVolume;
				const Vector3 Relative = P - Reference;
				for (int A = 0; A < Dimensions; ++A)
				{
					First[A] += CellVolume * Relative[A];
					for (int B = 0; B < Dimensions; ++B)
						Second[A][B] += CellVolume * ((double)Relative[A] * Relative[B] + (A == B ? (double)Size[A] * Size[A] / CuboidInertiaDivisor : 0.0));
				}
			}
		}
		if (!(Volume > 0.0) || !std::isfinite((float)Volume)) return Fail("SDF has no representable occupied volume");
		MassParameters& Mass = Data->UnitDensity;
		Mass.Volume = Mass.Mass = (float)Volume; Mass.BoundingVolume = SurfaceBounds;
		for (int A = 0; A < Dimensions; ++A)
		{
			Mass.CenterOfMass[A] = Reference[A] + (float)(First[A] / Volume);
			for (int B = 0; B < Dimensions; ++B) Second[A][B] -= First[A] * First[B] / Volume;
		}
		const double Trace = Second[0][0] + Second[1][1] + Second[2][2];
		for (int A = 0; A < Dimensions; ++A) for (int B = 0; B < Dimensions; ++B)
		{
			Mass.InertiaMat[A][B] = (float)((A == B ? Trace : 0.0) - Second[A][B]);
			if (!std::isfinite(Mass.InertiaMat[A][B])) return Fail("SDF inertia exceeds finite float range");
		}
		if (!Finite(Mass.CenterOfMass)) return Fail("Invalid SDF centroid");
		m_Data = std::move(Data); if (Error) Error->clear(); return true;
	}
	const RuntimeDistanceField& DistanceFieldShape::GetField() const
	{
		if (!m_Data) throw std::logic_error("Uncooked distance-field shape");
		return m_Data->Field;
	}
	const DistanceFieldShapeSettings& DistanceFieldShape::GetSettings() const
	{
		if (!m_Data) throw std::logic_error("Uncooked distance-field shape");
		return m_Data->Settings;
	}
	const StaticMesh& DistanceFieldShape::GetDisplayMesh() const
	{
		if (!m_Data) throw std::logic_error("Uncooked distance-field shape");
		return m_Data->Display;
	}
	float DistanceFieldShape::GetSurfaceTolerance() const { return m_Data ? m_Data->Field.GetCellSize() * m_Data->Settings.RelativeSurfaceTolerance : 0.0f; }
	uint32_t DistanceFieldShape::GetSurfaceFeature(const Vector3& Point) const
	{
		if (!m_Data || !Finite(Point)) return UINT32_MAX;
		const auto& Grid = *GetField().GetField();
		const Vector3 P = (Point - Grid.GetBounds().Min) / Grid.GetCellSize();
		const int X = (int)std::max(0.0f, std::min((float)Grid.GetCellsX() - 1, std::floor(P.x)));
		const int Y = (int)std::max(0.0f, std::min((float)Grid.GetCellsY() - 1, std::floor(P.y)));
		const int Z = (int)std::max(0.0f, std::min((float)Grid.GetCellsZ() - 1, std::floor(P.z)));
		return (Z * Grid.GetCellsY() + Y) * Grid.GetCellsX() + X;
	}
	bool DistanceFieldShape::Project(const Vector3& Point, Vector3& Surface, Vector3& Normal, float& SignedDistance) const
	{
		if (!m_Data || !Finite(Point)) return false;
		DistanceFieldSample Initial; if (!GetField().Sample(Point, Initial)) return false;
		Vector3 P = Point;
		for (int Iteration = 0; Iteration < m_Data->Settings.ProjectionIterations; ++Iteration)
		{
			DistanceFieldSample Sample; if (!GetField().Sample(P, Sample)) return false;
			const Vector3& G = Sample.Gradient;
			const double Magnitude = std::sqrt((double)G.x * G.x + (double)G.y * G.y + (double)G.z * G.z);
			if (!(Magnitude > 0.0)) return false;
			const Vector3 UnitGradient((float)(G.x / Magnitude), (float)(G.y / Magnitude), (float)(G.z / Magnitude));
			const double Error = Sample.Value / Magnitude;
			if (std::abs(Error) <= GetSurfaceTolerance())
			{
				Surface = P; Normal = UnitGradient;
				SignedDistance = (Point - P).Length() * (Initial.Value < 0.0f ? -1.0f : 1.0f); return true;
			}
			const float Limit = GetField().GetCellSize() * m_Data->Settings.ProjectionStepCells;
			P -= UnitGradient * (float)std::max(-(double)Limit, std::min((double)Limit, Error));
		}
		return false;
	}
	void DistanceFieldShape::GetSurfaceSamples(const Box3& Bounds, std::vector<DistanceFieldSurfaceSample>& Samples) const
	{
		Samples.clear(); if (!m_Data) return;
		const Box3 Domain = GetField().GetBounds();
		if (!m_Data->SurfaceSamples.empty() && Bounds.Min.x <= Domain.Min.x && Bounds.Min.y <= Domain.Min.y && Bounds.Min.z <= Domain.Min.z
			&& Bounds.Max.x >= Domain.Max.x && Bounds.Max.y >= Domain.Max.y && Bounds.Max.z >= Domain.Max.z)
		{
			Samples = m_Data->SurfaceSamples; return;
		}
		std::vector<Index3> Cells; GetField().QueryCells(Bounds, 0.0f, 0.0f, Cells);
		const auto& Grid = *GetField().GetField();
		for (const Index3& Cell : Cells)
		{
			const uint32_t Feature = (Cell.c * Grid.GetCellsY() + Cell.b) * Grid.GetCellsX() + Cell.a;
			for (int Axis = 0; Axis < Dimensions; ++Axis) for (int Corner = 0; Corner < CornerCount; ++Corner)
			{
				if (Corner & (1 << Axis)) continue;
				Index3 A(Cell.a + (Corner & 1), Cell.b + ((Corner >> 1) & 1), Cell.c + ((Corner >> 2) & 1));
				Index3 B = A; ++B[Axis];
				const double VA = Grid.At(A.a, A.b, A.c), VB = Grid.At(B.a, B.b, B.c);
				if (VA != 0.0 && VB != 0.0 && (VA < 0.0) == (VB < 0.0)) continue;
				const Vector3 Low = Grid.GridPointToWorld(A.a, A.b, A.c), High = Grid.GridPointToWorld(B.a, B.b, B.c);
				const double Weight = VA == VB ? 0.5 : VA / (VA - VB);
				Vector3 P = Low; P[Axis] = (float)(Low[Axis] + ((double)High[Axis] - Low[Axis]) * Weight);
				Samples.push_back({ P, Feature });
			}
		}
	}
	bool DistanceFieldShape::IntersectRay(const Vector3& Origin, const Vector3& Direction, float* Time) const
	{
		if (!m_Data || !Time) return false;
		DistanceFieldRayHit Hit;
		if (!GetField().RayCast(Origin, Direction, std::numeric_limits<float>::max(), Hit)) return false;
		*Time = Hit.Time; return true;
	}
	bool DistanceFieldShape::CalculateVolumeProperties(MassParameters* Properties, float Density) const
	{
		if (!m_Data || !Properties || !std::isfinite(Density) || Density < 0.0f) return false;
		*Properties = m_Data->UnitDensity; Properties->Mass *= Density; Properties->InertiaMat *= Density; return true;
	}
	Vector3 DistanceFieldShape::GetCenter() const { return m_Data ? m_Data->UnitDensity.CenterOfMass : Vector3::Zero(); }
	Vector3 DistanceFieldShape::GetSupport(const Vector3& Direction) const
	{
		Vector3 Best = GetCenter(); float Projection = -std::numeric_limits<float>::max();
		if (m_Data) for (uint32_t I = 0; I < m_Data->Display.NumVertices; ++I)
		{
			const Vector3 P = m_Data->Display.Vertices[I]; const float Value = Direction.Dot(P);
			if (Value > Projection) { Best = P; Projection = Value; }
		}
		return Best;
	}
}
