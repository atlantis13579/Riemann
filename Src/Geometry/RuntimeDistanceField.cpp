#include "RuntimeDistanceField.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <queue>

namespace Riemann
{
	/* Runtime SDF queries operate on the trilinear interpolant of a sampled field,
	   rather than on a reconstructed triangle mesh. The analytic gradient is the
	   derivative of that same interpolant; its magnitude is deliberately preserved.
	   A sampled SDF need not remain an exact Euclidean distance after interpolation.

	   A binary hierarchy stores spatial blocks, corner-value intervals and gradient
	   bounds. A trilinear function is a convex combination of its eight corners, so
	   these intervals conservatively reject blocks that cannot contain a requested
	   level set. Partial derivatives are bilinear; their extrema are bounded by the
	   corresponding grid-edge differences. Copies share an immutable cooked field.
	   Rebuild an instance between queries; existing copies retain the previous cook.

	   Within a cell a ray restricts the trilinear polynomial to a cubic. Splitting
	   at its derivative roots yields monotone intervals, including tangent roots
	   that a sign-change-only test or fixed-step ray march would miss. Ray time is
	   the parameter in origin + direction * time; direction need not have unit length.

	   Closest-surface queries use branch and bound: AABB and affine-slab distances
	   provide lower bounds, while edge zero crossings and projected samples provide
	   feasible upper bounds.
	   Ambiguous cells subdivide until the distance gap meets the requested accuracy.
	   A work limit returns an explicit error bound and Converged=false, never a false
	   claim of exactness. The bound concerns this interpolated field, not the original
	   source mesh. Grid bounds follow the actual rounded sample vertices. Sampling
	   outside that grid fails instead of inventing an extrapolated distance; nearest-surface queries also work outside it and at medial axes.

	   References: Hart, Sphere Tracing (1996), for conservative implicit queries;
	   interval branch and bound for constrained distance minimization.
	*/
	namespace
	{
		const int Dimensions = 3;
		const int CornerCount = 1 << Dimensions;
		const int PolynomialSize = Dimensions + 1;
		const double PolynomialEpsilon = 64 * std::numeric_limits<double>::epsilon();
		const double PositionRoundoffFactor = 8 * std::numeric_limits<float>::epsilon();
		using Corners = std::array<double, CornerCount>;

		bool Finite(const Vector3& P) { return std::isfinite(P.x) && std::isfinite(P.y) && std::isfinite(P.z); }
		bool ValidBounds(const Box3& B) { return Finite(B.Min) && Finite(B.Max) && B.Min.x <= B.Max.x && B.Min.y <= B.Max.y && B.Min.z <= B.Max.z; }
		bool Contains(const Box3& B, const Vector3& P)
		{
			return P.x >= B.Min.x && P.y >= B.Min.y && P.z >= B.Min.z && P.x <= B.Max.x && P.y <= B.Max.y && P.z <= B.Max.z;
		}
		Vector3 Clamp(const Vector3& P, const Box3& B) { return P.Max(B.Min).Min(B.Max); }
		double Length(const Vector3& V) { return std::sqrt((double)V.x * V.x + (double)V.y * V.y + (double)V.z * V.z); }
		double BoundsDistance(const Box3& B, const Vector3& P) { return Length(P - Clamp(P, B)); }
		Vector3 Corner(const Box3& B, int Index)
		{
			return Vector3(Index & 1 ? B.Max.x : B.Min.x, Index & 2 ? B.Max.y : B.Min.y, Index & 4 ? B.Max.z : B.Min.z);
		}
		double Evaluate(const Corners& Values, const double* Position, double* Gradient = nullptr)
		{
			double Value = 0.0;
			if (Gradient) std::fill(Gradient, Gradient + Dimensions, 0.0);
			for (int C = 0; C < CornerCount; ++C)
			{
				double Weights[Dimensions];
				for (int Axis = 0; Axis < Dimensions; ++Axis) Weights[Axis] = C & (1 << Axis) ? Position[Axis] : 1.0 - Position[Axis];
				Value += Values[C] * Weights[0] * Weights[1] * Weights[2];
				if (Gradient)
					for (int Axis = 0; Axis < Dimensions; ++Axis)
						Gradient[Axis] += Values[C] * (C & (1 << Axis) ? 1.0 : -1.0) * Weights[(Axis + 1) % Dimensions] * Weights[(Axis + 2) % Dimensions];
			}
			return Value;
		}
		void Range(const Corners& V, double& Min, double& Max)
		{
			Min = *std::min_element(V.begin(), V.end()); Max = *std::max_element(V.begin(), V.end());
		}
		bool HasZero(const Corners& V)
		{
			double Min, Max; Range(V, Min, Max); return Min <= 0.0 && Max >= 0.0;
		}
		bool RayBounds(const Box3& B, const Vector3& Origin, const Vector3& Direction, double Limit, double& Enter, double& Exit)
		{
			Enter = 0.0; Exit = Limit;
			for (int Axis = 0; Axis < Dimensions; ++Axis)
			{
				if (Direction[Axis] == 0.0f) { if (Origin[Axis] < B.Min[Axis] || Origin[Axis] > B.Max[Axis]) return false; continue; }
				double A = ((double)B.Min[Axis] - Origin[Axis]) / Direction[Axis], Z = ((double)B.Max[Axis] - Origin[Axis]) / Direction[Axis];
				if (A > Z) std::swap(A, Z);
				Enter = std::max(Enter, A); Exit = std::min(Exit, Z);
				if (Enter > Exit) return false;
			}
			return true;
		}
		bool Normal(const Vector3& Gradient, float Minimum, Vector3& Result)
		{
			const double Magnitude = Length(Gradient);
			Result = Magnitude > Minimum ? Gradient / (float)Magnitude : Vector3::Zero();
			return Magnitude > Minimum;
		}
		struct QueueItem
		{
			double Bound;
			int Index;
			bool operator<(const QueueItem& Other) const { return Bound == Other.Bound ? Index > Other.Index : Bound > Other.Bound; }
		};
	}

	bool RuntimeDistanceFieldSettings::IsValid() const
	{
		return (uint64_t)MaximumSearchNodes * ((uint64_t)LeafCellCount + 2) <= (uint64_t)std::numeric_limits<int>::max()
			&& LeafCellCount > 0 && RayRootIterations > 0 && ProjectionIterations >= 0 && MaximumSearchNodes > 0
			&& std::isfinite(RelativeValueTolerance) && RelativeValueTolerance > 0.0f
			&& std::isfinite(RelativeDistanceTolerance) && RelativeDistanceTolerance > 0.0f
			&& std::isfinite(MinimumGradient) && MinimumGradient >= 0.0f;
	}

	struct RuntimeDistanceField::Storage
	{
		struct Node
		{
			Index3 Begin, End;
			Box3 Bounds;
			float Minimum = 0.0f, Maximum = 0.0f, GradientBound = 0.0f;
			int Left = -1, Right = -1;
			bool HasZero() const { return Minimum <= 0.0f && Maximum >= 0.0f; }
		};
		SignedDistanceField3 Field;
		RuntimeDistanceFieldSettings Settings;
		std::vector<Node> Nodes;
		uint64_t SurfaceCellCount = 0;
		bool ClosedBoundary = true;

		Corners Values(const Index3& Cell) const
		{
			Corners V;
			for (int C = 0; C < CornerCount; ++C) V[C] = Field.At(Cell.a + (C & 1), Cell.b + ((C >> 1) & 1), Cell.c + ((C >> 2) & 1));
			return V;
		}
		Box3 CellBounds(const Index3& Cell) const { return Box3(Field.GridPointToWorld(Cell.a, Cell.b, Cell.c), Field.GridPointToWorld(Cell.a + 1, Cell.b + 1, Cell.c + 1)); }
		bool ValidCell(const Index3& C) const { return C.a >= 0 && C.b >= 0 && C.c >= 0 && C.a < Field.GetCellsX() && C.b < Field.GetCellsY() && C.c < Field.GetCellsZ(); }
		double EvaluateCell(const Index3& Cell, const Vector3& Position, double* Gradient = nullptr) const
		{
			const Box3 Bounds = CellBounds(Cell);
			double Coordinate[Dimensions];
			for (int Axis = 0; Axis < Dimensions; ++Axis) Coordinate[Axis] = ((double)Position[Axis] - Bounds.Min[Axis]) / ((double)Bounds.Max[Axis] - Bounds.Min[Axis]);
			const double Value = Evaluate(Values(Cell), Coordinate, Gradient);
			if (Gradient) for (int Axis = 0; Axis < Dimensions; ++Axis) Gradient[Axis] /= (double)Bounds.Max[Axis] - Bounds.Min[Axis];
			return Value;
		}
		DistanceFieldSample SampleCell(const Index3& Cell, const Vector3& Position) const
		{
			double Gradient[Dimensions];
			DistanceFieldSample Sample; Sample.Value = (float)EvaluateCell(Cell, Position, Gradient);
			for (int Axis = 0; Axis < Dimensions; ++Axis) Sample.Gradient[Axis] = (float)Gradient[Axis];
			return Sample;
		}
		Index3 Locate(const Vector3& P) const
		{
			Index3 Cell(0, 0, 0);
			const int Counts[Dimensions] = { Field.GetCellsX(), Field.GetCellsY(), Field.GetCellsZ() };
			for (int Axis = 0; Axis < Dimensions; ++Axis)
			{
				Cell[Axis] = std::max(0, std::min(Counts[Axis] - 1, (int)std::floor(((double)P[Axis] - Field.GetBounds().Min[Axis]) / Field.GetCellSize())));
				// World grid vertices are rounded floats. Keep sampling in the cell
				// bounded by those vertices, including either side of a shared face.
				while (Cell[Axis] > 0 && P[Axis] < CellBounds(Cell).Min[Axis]) --Cell[Axis];
				while (Cell[Axis] + 1 < Counts[Axis] && P[Axis] >= CellBounds(Cell).Max[Axis]) ++Cell[Axis];
			}
			return Cell;
		}
		int BuildNode(const Index3& Begin, const Index3& End)
		{
			const int Index = (int)Nodes.size(); Nodes.emplace_back();
			Node N; N.Begin = Begin; N.End = End;
			N.Bounds = Box3(Field.GridPointToWorld(Begin.a, Begin.b, Begin.c), Field.GridPointToWorld(End.a, End.b, End.c));
			int Axis = 0;
			for (int Dim = 1; Dim < Dimensions; ++Dim) if (End[Dim] - Begin[Dim] > End[Axis] - Begin[Axis]) Axis = Dim;
			const uint64_t Count = (uint64_t)(End.a - Begin.a) * (End.b - Begin.b) * (End.c - Begin.c);
			if (Count > (uint64_t)Settings.LeafCellCount)
			{
				Index3 Middle = Begin; Middle[Axis] += (End[Axis] - Begin[Axis]) / 2;
				Index3 LeftEnd = End; LeftEnd[Axis] = Middle[Axis];
				N.Left = BuildNode(Begin, LeftEnd); N.Right = BuildNode(Middle, End);
				N.Minimum = std::min(Nodes[N.Left].Minimum, Nodes[N.Right].Minimum);
				N.Maximum = std::max(Nodes[N.Left].Maximum, Nodes[N.Right].Maximum);
				N.GradientBound = std::max(Nodes[N.Left].GradientBound, Nodes[N.Right].GradientBound);
			}
			else
			{
				N.Minimum = std::numeric_limits<float>::max(); N.Maximum = -N.Minimum;
				for (int Z = Begin.c; Z < End.c; ++Z) for (int Y = Begin.b; Y < End.b; ++Y) for (int X = Begin.a; X < End.a; ++X)
				{
					const Index3 Cell(X, Y, Z);
					const Corners V = Values(Cell);
					const Box3 CellBox = CellBounds(Cell);
					double Min, Max; Range(V, Min, Max);
					N.Minimum = std::min(N.Minimum, (float)Min); N.Maximum = std::max(N.Maximum, (float)Max);
					if (Min <= 0.0 && Max >= 0.0) ++SurfaceCellCount;
					double GradientSquared = 0.0;
					for (int Dim = 0; Dim < Dimensions; ++Dim)
					{
						double Derivative = 0.0;
						for (int C = 0; C < CornerCount; ++C) if (!(C & (1 << Dim))) Derivative = std::max(Derivative, std::abs(V[C + (1 << Dim)] - V[C]) / ((double)CellBox.Max[Dim] - CellBox.Min[Dim]));
						GradientSquared += Derivative * Derivative;
					}
					N.GradientBound = std::max(N.GradientBound, std::nextafter((float)std::sqrt(GradientSquared), std::numeric_limits<float>::infinity()));
				}
			}
			Nodes[Index] = N;
			return Index;
		}
	};

	bool RuntimeDistanceField::Build(const SignedDistanceField3& Field, const RuntimeDistanceFieldSettings& Settings, std::string* Error)
	{
		auto Fail = [&](const char* Message) { if (Error) *Error = Message; return false; };
		if (!Settings.IsValid()) return Fail("Invalid runtime distance-field settings");
		if (!Field.IsValid() || !ValidBounds(Field.GetBounds()) || !std::isfinite(Field.GetCellSize())) return Fail("Invalid distance-field grid");
		const Index3 Counts(Field.GetCellsX(), Field.GetCellsY(), Field.GetCellsZ());
		if (2 * (uint64_t)Counts.a * Counts.b * Counts.c > (uint64_t)std::numeric_limits<int>::max()) return Fail("Grid hierarchy exceeds index capacity");
		for (int Axis = 0; Axis < Dimensions; ++Axis)
		{
			const float CoordinateScale = std::max(std::abs(Field.GetBounds().Min[Axis]), std::abs(Field.GetBounds().Max[Axis]));
			if (Field.GetCellSize() <= std::numeric_limits<float>::epsilon() * CoordinateScale) return Fail("Grid spacing is below position precision");
		}
		if (!std::isfinite(Field.GetCellSize() * Settings.RelativeDistanceTolerance)
			|| !std::isfinite(Field.GetCellSize() * Settings.RelativeValueTolerance)) return Fail("Grid tolerance exceeds finite float range");
		const Vector3 Last = Field.GridPointToWorld(Counts.a, Counts.b, Counts.c);
		const double ExtentTolerance = (double)Field.GetCellSize() * Settings.RelativeValueTolerance;
		if (!Finite(Last) || Length(Last - Field.GetBounds().Max) > ExtentTolerance) return Fail("Grid bounds do not match cell spacing and dimensions");
		auto Data = std::make_shared<Storage>();
		Data->Settings = Settings; Data->Field = Field;
		for (int Z = 0; Z <= Counts.c; ++Z) for (int Y = 0; Y <= Counts.b; ++Y) for (int X = 0; X <= Counts.a; ++X)
		{
			const float Value = Field.At(X, Y, Z);
			if (!std::isfinite(Value)) return Fail("Distance-field samples must be finite");
			if ((X == 0 || Y == 0 || Z == 0 || X == Counts.a || Y == Counts.b || Z == Counts.c) && Value <= 0.0f) Data->ClosedBoundary = false;
		}
		Data->BuildNode(Index3(0, 0, 0), Counts);
		if (!std::isfinite(Data->Nodes[0].GradientBound)) return Fail("Distance-field gradient exceeds finite float range");
		m_Data = std::move(Data);
		if (Error) Error->clear();
		return true;
	}
	bool RuntimeDistanceField::HasClosedBoundary() const { return m_Data && m_Data->ClosedBoundary; }
	const SignedDistanceField3* RuntimeDistanceField::GetField() const { return m_Data ? &m_Data->Field : nullptr; }
	Box3 RuntimeDistanceField::GetBounds() const { return m_Data ? m_Data->Nodes[0].Bounds : Box3::Empty(); }
	float RuntimeDistanceField::GetCellSize() const { return m_Data ? m_Data->Field.GetCellSize() : 0.0f; }
	float RuntimeDistanceField::GetGradientBound() const { return m_Data ? m_Data->Nodes[0].GradientBound : 0.0f; }
	size_t RuntimeDistanceField::GetNodeCount() const { return m_Data ? m_Data->Nodes.size() : 0; }
	uint64_t RuntimeDistanceField::GetSurfaceCellCount() const { return m_Data ? m_Data->SurfaceCellCount : 0; }
	bool RuntimeDistanceField::Sample(const Vector3& Position, DistanceFieldSample& Result) const
	{
		if (!m_Data || !Finite(Position) || !Contains(GetBounds(), Position)) return false;
		Result = m_Data->SampleCell(m_Data->Locate(Position), Position);
		return true;
	}
	bool RuntimeDistanceField::GetCellRange(const Index3& Cell, float& Minimum, float& Maximum) const
	{
		if (!m_Data || !m_Data->ValidCell(Cell)) return false;
		double Min, Max; Range(m_Data->Values(Cell), Min, Max); Minimum = (float)Min; Maximum = (float)Max;
		return true;
	}
	void RuntimeDistanceField::QueryCells(const Box3& Bounds, float MinimumValue, float MaximumValue,
		std::vector<Index3>& Cells, DistanceFieldQueryStats* Stats) const
	{
		Cells.clear(); if (Stats) *Stats = DistanceFieldQueryStats();
		if (!m_Data || !ValidBounds(Bounds) || std::isnan(MinimumValue) || std::isnan(MaximumValue) || MinimumValue > MaximumValue) return;
		std::vector<int> Stack(1, 0);
		while (!Stack.empty())
		{
			const Storage::Node& N = m_Data->Nodes[Stack.back()]; Stack.pop_back();
			if (Stats) ++Stats->NodeVisits;
			if (!N.Bounds.Intersect(Bounds) || N.Minimum > MaximumValue || N.Maximum < MinimumValue) continue;
			if (N.Left >= 0) { Stack.push_back(N.Right); Stack.push_back(N.Left); continue; }
			for (int Z = N.Begin.c; Z < N.End.c; ++Z) for (int Y = N.Begin.b; Y < N.End.b; ++Y) for (int X = N.Begin.a; X < N.End.a; ++X)
			{
				if (Stats) ++Stats->CellTests;
				const Index3 C(X, Y, Z);
				if (!m_Data->CellBounds(C).Intersect(Bounds)) continue;
				double Min, Max; Range(m_Data->Values(C), Min, Max);
				if (Min <= MaximumValue && Max >= MinimumValue) Cells.push_back(C);
			}
		}
	}

	namespace
	{
		double Polynomial(const double* C, double T) { return ((C[3] * T + C[2]) * T + C[1]) * T + C[0]; }
		bool RayCell(const Corners& V, const Box3& Bounds, const Vector3& Origin, const Vector3& Direction,
			double Enter, double Exit, int Iterations, double& Time)
		{
			double Coefficients[PolynomialSize] = {};
			for (int Corner = 0; Corner < CornerCount; ++Corner)
			{
				double Product[PolynomialSize] = { V[Corner], 0.0, 0.0, 0.0 };
				for (int Axis = 0; Axis < Dimensions; ++Axis)
				{
					const double Width = (double)Bounds.Max[Axis] - Bounds.Min[Axis];
					const double Start = ((double)Origin[Axis] + Enter * Direction[Axis] - Bounds.Min[Axis]) / Width;
					const double Delta = (Exit - Enter) * Direction[Axis] / Width;
					const double A = Corner & (1 << Axis) ? Start : 1.0 - Start, B = Corner & (1 << Axis) ? Delta : -Delta;
					for (int Order = Axis + 1; Order >= 0; --Order) Product[Order] = Product[Order] * A + (Order > 0 ? Product[Order - 1] * B : 0.0);
				}
				for (int Order = 0; Order < PolynomialSize; ++Order) Coefficients[Order] += Product[Order];
			}
			double CoefficientScale = 0.0;
			for (double Coefficient : Coefficients) CoefficientScale = std::max(CoefficientScale, std::abs(Coefficient));
			const double ValueTolerance = PolynomialEpsilon * CoefficientScale;
			std::vector<double> Cuts = { 0.0, 1.0 };
			auto AddCut = [&](double T) { if (T > 0.0 && T < 1.0) Cuts.push_back(T); };
			const double A = 3 * Coefficients[3], B = 2 * Coefficients[2], C = Coefficients[1];
			const double Scale = std::max(std::abs(A), std::max(std::abs(B), std::abs(C)));
			if (std::abs(A) <= PolynomialEpsilon * Scale)
			{
				if (std::abs(B) > PolynomialEpsilon * Scale) AddCut(-C / B);
			}
			else
			{
				const double Discriminant = B * B - 4 * A * C;
				if (Discriminant >= 0.0)
				{
					const double Q = -0.5 * (B + std::copysign(std::sqrt(Discriminant), B));
					if (Q != 0.0) { AddCut(Q / A); AddCut(C / Q); } else AddCut(-B / (2 * A));
				}
			}
			std::sort(Cuts.begin(), Cuts.end());
			for (size_t Index = 0; Index < Cuts.size(); ++Index)
			{
				double Low = Cuts[Index], LowValue = Polynomial(Coefficients, Low);
				if (std::abs(LowValue) <= ValueTolerance) { Time = Enter + Low * (Exit - Enter); return true; }
				if (Index + 1 == Cuts.size()) break;
				double High = Cuts[Index + 1], HighValue = Polynomial(Coefficients, High);
				if ((LowValue < 0.0) == (HighValue < 0.0)) continue;
				for (int Iteration = 0; Iteration < Iterations; ++Iteration)
				{
					const double Middle = (Low + High) / 2, Value = Polynomial(Coefficients, Middle);
					if (std::abs(Value) <= ValueTolerance) { Low = High = Middle; break; }
					if ((Value < 0.0) == (LowValue < 0.0)) { Low = Middle; LowValue = Value; } else High = Middle;
				}
				Time = Enter + (Low + High) / 2 * (Exit - Enter); return true;
			}
			return false;
		}
	}

	bool RuntimeDistanceField::RayCast(const Vector3& Origin, const Vector3& Direction, float MaxTime,
		DistanceFieldRayHit& Hit, bool InitialOverlap, DistanceFieldQueryStats* Stats) const
	{
		if (Stats) *Stats = DistanceFieldQueryStats();
		if (!m_Data || !Finite(Origin) || !Finite(Direction) || Length(Direction) == 0.0 || !std::isfinite(MaxTime) || MaxTime < 0.0f) return false;
		DistanceFieldSample Start;
		if (InitialOverlap && Sample(Origin, Start) && Start.Value <= 0.0f)
		{
			DistanceFieldRayHit Result; Result.Position = Origin; Result.InitialOverlap = Start.Value < 0.0f; Result.Cell = m_Data->Locate(Origin);
			Result.NormalValid = Normal(Start.Gradient, m_Data->Settings.MinimumGradient, Result.Normal); Hit = Result; return true;
		}
		double Enter, Exit;
		if (!m_Data->Nodes[0].HasZero() || !RayBounds(GetBounds(), Origin, Direction, MaxTime, Enter, Exit)) return false;
		std::priority_queue<QueueItem> Queue; Queue.push({ Enter, 0 });
		double Closest = MaxTime;
		bool Found = false;
		DistanceFieldRayHit Result;
		while (!Queue.empty())
		{
			const QueueItem Item = Queue.top(); Queue.pop();
			if (Item.Bound > Closest) break;
			const Storage::Node& N = m_Data->Nodes[Item.Index];
			if (Stats) ++Stats->NodeVisits;
			if (N.Left >= 0)
			{
				for (int Child : { N.Left, N.Right })
					if (m_Data->Nodes[Child].HasZero() && RayBounds(m_Data->Nodes[Child].Bounds, Origin, Direction, Closest, Enter, Exit)) Queue.push({ Enter, Child });
				continue;
			}
			for (int Z = N.Begin.c; Z < N.End.c; ++Z) for (int Y = N.Begin.b; Y < N.End.b; ++Y) for (int X = N.Begin.a; X < N.End.a; ++X)
			{
				const Index3 Cell(X, Y, Z); const Box3 Bounds = m_Data->CellBounds(Cell);
				if (!RayBounds(Bounds, Origin, Direction, Closest, Enter, Exit)) continue;
				const Corners V = m_Data->Values(Cell); if (!HasZero(V)) continue;
				if (Stats) ++Stats->CellTests;
				double Time;
				if (!RayCell(V, Bounds, Origin, Direction, Enter, Exit, m_Data->Settings.RayRootIterations, Time)) continue;
				Closest = Time; Found = true; Result.Time = (float)Time; Result.Position = Origin + Direction * Result.Time; Result.Cell = Cell;
				Result.NormalValid = Normal(m_Data->SampleCell(Cell, Result.Position).Gradient, m_Data->Settings.MinimumGradient, Result.Normal);
			}
		}
		if (Found) Hit = Result;
		return Found;
	}

	bool RuntimeDistanceField::ClosestSurfacePoint(const Vector3& Position, DistanceFieldSurfacePoint& Result,
		float MaxDistance, DistanceFieldQueryStats* Stats, float DistanceTolerance) const
	{
		if (Stats) *Stats = DistanceFieldQueryStats();
		Result = DistanceFieldSurfacePoint();
		if (!m_Data || !Finite(Position) || !std::isfinite(MaxDistance) || MaxDistance < 0.0f || !std::isfinite(DistanceTolerance) || DistanceTolerance < 0.0f) return false;
		if (!m_Data->Nodes[0].HasZero()) { Result.Converged = true; Result.Status = DistanceFieldSearchStatus::Complete; return false; }
		struct Region { Box3 Bounds; Corners Values; Index3 Cell; };
		auto RegionDistance = [&](const Region& R)
		{
			const double Center[Dimensions] = { 0.5, 0.5, 0.5 };
			double Gradient[Dimensions]; const double Value = Evaluate(R.Values, Center, Gradient);
			double Affine = Value, MagnitudeSquared = 0.0;
			for (int Axis = 0; Axis < Dimensions; ++Axis)
			{
				const double Width = (double)R.Bounds.Max[Axis] - R.Bounds.Min[Axis];
				Affine += Gradient[Axis] * (((double)Position[Axis] - R.Bounds.Min[Axis]) / Width - 0.5);
				MagnitudeSquared += Gradient[Axis] * Gradient[Axis] / (Width * Width);
			}
			double Minimum = std::numeric_limits<double>::max(), Maximum = -Minimum;
			for (int C = 0; C < CornerCount; ++C)
			{
				double Residual = R.Values[C] - Value;
				for (int Axis = 0; Axis < Dimensions; ++Axis) Residual -= Gradient[Axis] * (C & (1 << Axis) ? 0.5 : -0.5);
				Minimum = std::min(Minimum, Residual); Maximum = std::max(Maximum, Residual);
			}
			// Trilinear minus affine remains trilinear: corner residuals bound it
			// everywhere. The resulting plane slab tightly encloses the zero set.
			const double BoxDistance = BoundsDistance(R.Bounds, Position);
			if (MagnitudeSquared == 0.0) return BoxDistance;
			double N[Dimensions], ClampedValue = Affine;
			std::array<double, 2 * Dimensions> Breakpoints;
			int Count = 0;
			for (int Axis = 0; Axis < Dimensions; ++Axis)
			{
				N[Axis] = Gradient[Axis] / ((double)R.Bounds.Max[Axis] - R.Bounds.Min[Axis]);
				ClampedValue += N[Axis] * (std::max((double)R.Bounds.Min[Axis], std::min((double)R.Bounds.Max[Axis], (double)Position[Axis])) - Position[Axis]);
				if (N[Axis] == 0.0) continue;
				Breakpoints[Count++] = ((double)Position[Axis] - R.Bounds.Min[Axis]) / N[Axis];
				Breakpoints[Count++] = ((double)Position[Axis] - R.Bounds.Max[Axis]) / N[Axis];
			}
			if (ClampedValue >= -Maximum && ClampedValue <= -Minimum) return BoxDistance;
			const double Target = ClampedValue < -Maximum ? -Maximum : -Minimum;
			auto Constraint = [&](double Lambda, double& SquaredDistance)
			{
				double Value = Affine - Target; SquaredDistance = 0.0;
				for (int Axis = 0; Axis < Dimensions; ++Axis)
				{
					const double P = std::max((double)R.Bounds.Min[Axis], std::min((double)R.Bounds.Max[Axis], Position[Axis] - Lambda * N[Axis]));
					const double Delta = P - Position[Axis]; Value += N[Axis] * Delta; SquaredDistance += Delta * Delta;
				}
				return Value;
			};
			// Box-constrained projection onto the nearest slab plane is piecewise
			// linear in its Lagrange multiplier. Its dual value is a lower bound
			// even if floating-point rounding leaves a small constraint residual.
			std::sort(Breakpoints.begin(), Breakpoints.begin() + Count);
			double SquaredDistance, Lambda = Breakpoints[0];
			double Previous = Constraint(Lambda, SquaredDistance);
			for (int Index = 1; Index < Count; ++Index)
			{
				const double Next = Constraint(Breakpoints[Index], SquaredDistance);
				if (Previous >= 0.0 && Next <= 0.0)
				{
					Lambda = Previous == Next ? Breakpoints[Index] : Breakpoints[Index - 1]
						+ (Breakpoints[Index] - Breakpoints[Index - 1]) * Previous / (Previous - Next);
					break;
				}
				Previous = Next;
			}
			const double Residual = Constraint(Lambda, SquaredDistance);
			const double Dual = SquaredDistance + 2 * Lambda * Residual;
			return std::max(BoxDistance, std::sqrt(std::max(0.0, Dual * (1.0 - PolynomialEpsilon))));
		};
		std::vector<Region> Regions;
		std::priority_queue<QueueItem> Queue;
		Queue.push({ BoundsDistance(GetBounds(), Position), 0 });
		double Best = MaxDistance, TerminalBound = MaxDistance;
		const double Tolerance = DistanceTolerance > 0.0f ? DistanceTolerance : GetCellSize() * m_Data->Settings.RelativeDistanceTolerance;
		const double Roundoff = PositionRoundoffFactor * std::max(Length(Position), std::max(Length(GetBounds().Min), Length(GetBounds().Max)));
		const double SearchTolerance = Tolerance > Roundoff ? Tolerance - Roundoff : Tolerance;
		bool Found = false;
		DistanceFieldSurfacePoint Candidate;
		auto Accept = [&](const Vector3& Point, const Index3& Cell)
		{
			const double Distance = Length(Point - Position);
			if (Distance > Best || (Found && Distance == Best)) return;
			Best = Distance; Found = true; Candidate.Position = Point; Candidate.Distance = (float)Distance; Candidate.Cell = Cell;
		};
		auto Seed = [&](const Region& R)
		{
			for (int Axis = 0; Axis < Dimensions; ++Axis) for (int C = 0; C < CornerCount; ++C)
			{
				if (C & (1 << Axis)) continue;
				const int D = C + (1 << Axis);
				if (R.Values[C] == 0.0) Accept(Corner(R.Bounds, C), R.Cell);
				if (R.Values[D] == 0.0) Accept(Corner(R.Bounds, D), R.Cell);
				if ((R.Values[C] < 0.0) == (R.Values[D] < 0.0)) continue;
				const double Weight = R.Values[C] / (R.Values[C] - R.Values[D]);
				Accept(Corner(R.Bounds, C) * (float)(1.0 - Weight) + Corner(R.Bounds, D) * (float)Weight, R.Cell);
			}
			Vector3 Point = Clamp(Position, R.Bounds);
			for (int Iteration = 0; Iteration < m_Data->Settings.ProjectionIterations; ++Iteration)
			{
				const DistanceFieldSample Sample = m_Data->SampleCell(R.Cell, Point);
				const double Magnitude = Length(Sample.Gradient);
				if (Magnitude <= m_Data->Settings.MinimumGradient) break;
				// Solve a coordinate line exactly before accepting a projected seed.
				// A small residual alone gives no distance bound near a vanishing gradient.
				for (int Axis = 0; Axis < Dimensions; ++Axis)
				{
					Vector3 A = Point, B = Point; A[Axis] = R.Bounds.Min[Axis]; B[Axis] = R.Bounds.Max[Axis];
					const double VA = m_Data->EvaluateCell(R.Cell, A), VB = m_Data->EvaluateCell(R.Cell, B);
					if (VA == 0.0) Accept(A, R.Cell);
					if (VB == 0.0) Accept(B, R.Cell);
					if ((VA < 0.0) != (VB < 0.0))
					{
						Vector3 Root = Point; Root[Axis] = (float)(A[Axis] + ((double)B[Axis] - A[Axis]) * VA / (VA - VB));
						Accept(Root, R.Cell);
					}
				}
				if (std::abs(Sample.Value) <= GetCellSize() * m_Data->Settings.RelativeValueTolerance) break;
				const Vector3 Next = Clamp(Point - Sample.Gradient * (float)(Sample.Value / (Magnitude * Magnitude)), R.Bounds);
				if (Next == Point) break;
				Point = Next;
			}
		};
		uint32_t Work = 0;
		while (!Queue.empty() && Work++ < m_Data->Settings.MaximumSearchNodes)
		{
			const QueueItem Item = Queue.top();
			if (Found && Best - Item.Bound <= SearchTolerance) break;
			Queue.pop();
			if (Item.Bound > Best) continue;
			if (Item.Index >= 0)
			{
				const Storage::Node& N = m_Data->Nodes[Item.Index]; if (Stats) ++Stats->NodeVisits;
				if (N.Left >= 0)
				{
					for (int Child : { N.Left, N.Right }) if (m_Data->Nodes[Child].HasZero()) Queue.push({ BoundsDistance(m_Data->Nodes[Child].Bounds, Position), Child });
				}
				else for (int Z = N.Begin.c; Z < N.End.c; ++Z) for (int Y = N.Begin.b; Y < N.End.b; ++Y) for (int X = N.Begin.a; X < N.End.a; ++X)
				{
					if (Stats) ++Stats->CellTests;
					const Index3 Cell(X, Y, Z); Region R = { m_Data->CellBounds(Cell), m_Data->Values(Cell), Cell };
					if (!HasZero(R.Values)) continue;
					Seed(R); Regions.push_back(R); Queue.push({ RegionDistance(R), -(int)Regions.size() });
				}
				continue;
			}
			const Region R = Regions[-Item.Index - 1];
			const Vector3 Size = R.Bounds.Max - R.Bounds.Min;
			int Axis = 0; for (int Dim = 1; Dim < Dimensions; ++Dim) if (Size[Dim] > Size[Axis]) Axis = Dim;
			const float Middle = (float)(((double)R.Bounds.Min[Axis] + R.Bounds.Max[Axis]) / 2);
			if (Middle <= R.Bounds.Min[Axis] || Middle >= R.Bounds.Max[Axis]) { TerminalBound = std::min(TerminalBound, Item.Bound); continue; }
			if (Stats) ++Stats->Subdivisions;
			for (int Side = 0; Side < 2; ++Side)
			{
				Region Child = R;
				if (Side == 0) Child.Bounds.Max[Axis] = Middle; else Child.Bounds.Min[Axis] = Middle;
				for (int C = 0; C < CornerCount; ++C)
				{
					const Vector3 Point = Corner(Child.Bounds, C); double Coordinate[Dimensions];
					for (int Dim = 0; Dim < Dimensions; ++Dim) Coordinate[Dim] = ((double)Point[Dim] - R.Bounds.Min[Dim]) / Size[Dim];
					Child.Values[C] = Evaluate(R.Values, Coordinate);
				}
				if (!HasZero(Child.Values) || RegionDistance(Child) > Best) continue;
				Seed(Child); Regions.push_back(Child); Queue.push({ RegionDistance(Child), -(int)Regions.size() });
			}
		}
		if (!Found)
		{
			Result.Converged = Queue.empty();
			Result.Status = Queue.empty() ? DistanceFieldSearchStatus::Complete : DistanceFieldSearchStatus::WorkLimit;
			Result.ErrorBound = Queue.empty() ? 0.0f : std::numeric_limits<float>::infinity();
			return false;
		}
		const double LowerBound = std::min(TerminalBound, Queue.empty() ? Best : Queue.top().Bound);
		Candidate.ErrorBound = std::nextafter((float)(std::max(0.0, Best - LowerBound) + Roundoff), std::numeric_limits<float>::infinity());
		Candidate.Converged = Candidate.ErrorBound <= Tolerance;
		Candidate.Status = Candidate.Converged ? DistanceFieldSearchStatus::Complete
			: (Queue.empty() || Roundoff >= Tolerance ? DistanceFieldSearchStatus::PrecisionLimit : DistanceFieldSearchStatus::WorkLimit);
		Candidate.NormalValid = Normal(m_Data->SampleCell(Candidate.Cell, Candidate.Position).Gradient, m_Data->Settings.MinimumGradient, Candidate.Normal);
		Result = Candidate; return true;
	}
	/* A target oracle returns Euclidean distance upper bounds and their errors.
	   Distance to a closed set is 1-Lipschitz, so a center distance minus a box's
	   radius bounds every point in that box. Convex targets can additionally expose
	   a supporting plane. Its minimum over the box is often much tighter.

	   Inside a source cell, an affine approximation plus the exact corner residual
	   interval encloses the trilinear zero set in a slab. Intersect its coordinate
	   bounds with the cell before evaluating the target bound. This keeps parallel
	   flat surfaces efficient without replacing either surface with display triangles.
	   Returned lower/upper distances remain separate when a work/precision limit is
	   reached: conservative advancement must use the lower bound, never the witness
	   distance. All oracle quantities use the source field's coordinate system.
	   A positive per-query tolerance overrides the cook's default distance error.
	   Nested oracles propagate their work/precision status: exhausting a target's
	   search budget must stop advancement, never become a certified empty target.
	*/
	double DistanceFieldDistanceTarget::LowerBound(const Box3& Bounds) const
	{
		DistanceFieldTargetPoint Sample;
		if (!Evaluate(Bounds.GetCenter(), Sample)) return 0.0;
		double Bound = std::max(0.0, (double)Sample.Distance - Sample.ErrorBound - Length(Bounds.GetExtent()));
		if (Sample.SupportingPlane)
		{
			const double Magnitude = Length(Sample.Normal);
			if (Magnitude > 0.0)
			{
				double Plane = 0.0;
				for (int Axis = 0; Axis < Dimensions; ++Axis)
					Plane += ((Sample.Normal[Axis] >= 0.0f ? Bounds.Min[Axis] : Bounds.Max[Axis]) - (double)Sample.Point[Axis]) * Sample.Normal[Axis] / Magnitude;
				Bound = std::max(Bound, Plane);
			}
		}
		return Bound;
	}

	bool RuntimeDistanceField::DistanceToTarget(const DistanceFieldDistanceTarget& Target,
		DistanceFieldTargetDistance& Result, DistanceFieldQueryStats* Stats, float DistanceTolerance) const
	{
		Result = DistanceFieldTargetDistance(); if (Stats) *Stats = DistanceFieldQueryStats();
		if (!m_Data || !std::isfinite(DistanceTolerance) || DistanceTolerance < 0.0f) return false;
		if (!m_Data->Nodes[0].HasZero()) { Result.Converged = true; Result.Status = DistanceFieldSearchStatus::Complete; return false; }
		struct Region { Box3 Bounds; Corners Values; Index3 Cell; };
		const double Roundoff = PositionRoundoffFactor * std::max(Length(GetBounds().Min), Length(GetBounds().Max));
		const double Tolerance = DistanceTolerance > 0.0f ? DistanceTolerance : GetCellSize() * m_Data->Settings.RelativeDistanceTolerance;
		auto Bound = [&](const Box3& Bounds) { return std::max(0.0, Target.LowerBound(Bounds) - Roundoff); };
		auto ZeroBounds = [&](const Region& R)
		{
			const double Center[Dimensions] = { 0.5, 0.5, 0.5 };
			double G[Dimensions]; const double Value = Evaluate(R.Values, Center, G);
			double Minimum = std::numeric_limits<double>::max(), Maximum = -Minimum;
			for (int C = 0; C < CornerCount; ++C)
			{
				double Residual = R.Values[C] - Value;
				for (int Axis = 0; Axis < Dimensions; ++Axis) Residual -= G[Axis] * (C & (1 << Axis) ? 0.5 : -0.5);
				Minimum = std::min(Minimum, Residual); Maximum = std::max(Maximum, Residual);
			}
			Box3 Bounds = R.Bounds;
			for (int Axis = 0; Axis < Dimensions; ++Axis)
			{
				if (G[Axis] == 0.0) continue;
				double Other = 0.0;
				for (int Dim = 0; Dim < Dimensions; ++Dim) if (Dim != Axis) Other += std::abs(G[Dim]) / 2;
				double Low = (-Maximum - Value - Other) / G[Axis] + 0.5;
				double High = (-Minimum - Value + Other) / G[Axis] + 0.5;
				if (Low > High) std::swap(Low, High);
				const double Width = (double)R.Bounds.Max[Axis] - R.Bounds.Min[Axis];
				const float MinimumPosition = std::nextafter((float)(R.Bounds.Min[Axis] + Width * Low), -std::numeric_limits<float>::infinity());
				const float MaximumPosition = std::nextafter((float)(R.Bounds.Min[Axis] + Width * High), std::numeric_limits<float>::infinity());
				Bounds.Min[Axis] = std::max(Bounds.Min[Axis], MinimumPosition);
				Bounds.Max[Axis] = std::min(Bounds.Max[Axis], MaximumPosition);
			}
			return ValidBounds(Bounds) ? Bounds : R.Bounds;
		};
		std::vector<Region> Regions; std::priority_queue<QueueItem> Queue;
		Queue.push({ Bound(GetBounds()), 0 });
		double Best = std::numeric_limits<double>::infinity(), Terminal = Best;
		bool Found = false, Invalid = false;
		auto Accept = [&](const Vector3& Point, const Index3& Cell)
		{
			DistanceFieldTargetPoint Sample;
			if (!Target.Evaluate(Point, Sample) || !Finite(Sample.Point) || !Finite(Sample.Normal)
				|| !std::isfinite(Sample.Distance) || Sample.Distance < 0.0f || !std::isfinite(Sample.ErrorBound) || Sample.ErrorBound < 0.0f)
			{ Invalid = true; return; }
			if (Sample.Distance >= Best) return;
			Best = Sample.Distance; Found = true; Result.Position = Point; Result.TargetPoint = Sample.Point;
			Result.Normal = Sample.Normal; Result.Cell = Cell;
		};
		auto Seed = [&](const Region& R)
		{
			for (int Axis = 0; Axis < Dimensions; ++Axis) for (int C = 0; C < CornerCount; ++C)
			{
				if (C & (1 << Axis)) continue;
				const int D = C + (1 << Axis);
				if (R.Values[C] == 0.0) Accept(Corner(R.Bounds, C), R.Cell);
				if (R.Values[D] == 0.0) Accept(Corner(R.Bounds, D), R.Cell);
				if ((R.Values[C] < 0.0) == (R.Values[D] < 0.0)) continue;
				const double Weight = R.Values[C] / (R.Values[C] - R.Values[D]);
				Vector3 Point = Corner(R.Bounds, C);
				Point[Axis] = (float)(Point[Axis] + ((double)Corner(R.Bounds, D)[Axis] - Point[Axis]) * Weight);
				Accept(Point, R.Cell);
			}
			// The target witness supplies useful interior coordinate lines even when
			// the nearest pair is far from the cell's twelve edge crossings.
			DistanceFieldTargetPoint Sample;
			if (!Target.Evaluate(R.Bounds.GetCenter(), Sample)) { Invalid = true; return; }
			Vector3 Point = Clamp(Sample.Point, R.Bounds);
			for (int Axis = 0; Axis < Dimensions; ++Axis)
			{
				Vector3 A = Point, B = Point; A[Axis] = R.Bounds.Min[Axis]; B[Axis] = R.Bounds.Max[Axis];
				const double VA = m_Data->EvaluateCell(R.Cell, A), VB = m_Data->EvaluateCell(R.Cell, B);
				if (VA == 0.0) Accept(A, R.Cell);
				if (VB == 0.0) Accept(B, R.Cell);
				if ((VA < 0.0) != (VB < 0.0))
				{
					Vector3 Root = Point; Root[Axis] = (float)(A[Axis] + ((double)B[Axis] - A[Axis]) * VA / (VA - VB)); Accept(Root, R.Cell);
				}
			}
		};
		uint32_t Work = 0;
		while (!Queue.empty() && Work++ < m_Data->Settings.MaximumSearchNodes && !Invalid && Target.GetStatus() == DistanceFieldSearchStatus::Complete)
		{
			const QueueItem Item = Queue.top();
			if (Found && Best - Item.Bound + Roundoff <= Tolerance) break;
			Queue.pop(); if (Item.Bound > Best) continue;
			if (Item.Index >= 0)
			{
				const auto& N = m_Data->Nodes[Item.Index]; if (Stats) ++Stats->NodeVisits;
				if (N.Left >= 0)
				{
					for (int Child : { N.Left, N.Right }) if (m_Data->Nodes[Child].HasZero()) Queue.push({ Bound(m_Data->Nodes[Child].Bounds), Child });
				}
				else for (int Z = N.Begin.c; Z < N.End.c; ++Z) for (int Y = N.Begin.b; Y < N.End.b; ++Y) for (int X = N.Begin.a; X < N.End.a; ++X)
				{
					if (Stats) ++Stats->CellTests;
					const Index3 Cell(X, Y, Z); Region R = { m_Data->CellBounds(Cell), m_Data->Values(Cell), Cell };
					if (!HasZero(R.Values)) continue;
					const double Lower = Bound(ZeroBounds(R)); if (Lower > Best) continue;
					Seed(R); Regions.push_back(R); Queue.push({ Lower, -(int)Regions.size() });
				}
				continue;
			}
			const Region R = Regions[-Item.Index - 1]; const Vector3 Size = R.Bounds.Max - R.Bounds.Min;
			int Axis = 0; for (int Dim = 1; Dim < Dimensions; ++Dim) if (Size[Dim] > Size[Axis]) Axis = Dim;
			const float Middle = (float)(((double)R.Bounds.Min[Axis] + R.Bounds.Max[Axis]) / 2);
			if (Middle <= R.Bounds.Min[Axis] || Middle >= R.Bounds.Max[Axis]) { Terminal = std::min(Terminal, Item.Bound); continue; }
			if (Stats) ++Stats->Subdivisions;
			for (int Side = 0; Side < 2; ++Side)
			{
				Region Child = R; if (Side == 0) Child.Bounds.Max[Axis] = Middle; else Child.Bounds.Min[Axis] = Middle;
				for (int C = 0; C < CornerCount; ++C)
				{
					const Vector3 Point = Corner(Child.Bounds, C); double Coordinate[Dimensions];
					for (int Dim = 0; Dim < Dimensions; ++Dim) Coordinate[Dim] = ((double)Point[Dim] - R.Bounds.Min[Dim]) / Size[Dim];
					Child.Values[C] = Evaluate(R.Values, Coordinate);
				}
				if (!HasZero(Child.Values)) continue;
				const double Lower = Bound(ZeroBounds(Child)); if (Lower > Best) continue;
				Seed(Child); Regions.push_back(Child); Queue.push({ Lower, -(int)Regions.size() });
			}
		}
		if (Invalid) { Result = DistanceFieldTargetDistance(); return false; }
		const double Lower = std::min(Terminal, Queue.empty() ? Best : Queue.top().Bound);
		Result.LowerBound = (float)std::max(0.0, std::min(Best, Lower));
		if (Result.LowerBound > 0.0f) Result.LowerBound = std::nextafter(Result.LowerBound, 0.0f);
		Result.UpperBound = Found ? std::nextafter((float)(Best + Roundoff), std::numeric_limits<float>::infinity()) : std::numeric_limits<float>::infinity();
		Result.Converged = Found && Result.UpperBound - Result.LowerBound <= Tolerance;
		Result.Status = Result.Converged ? DistanceFieldSearchStatus::Complete
			: (Queue.empty() || Roundoff >= Tolerance ? DistanceFieldSearchStatus::PrecisionLimit : DistanceFieldSearchStatus::WorkLimit);
		if (!Result.Converged && Target.GetStatus() != DistanceFieldSearchStatus::Complete) Result.Status = Target.GetStatus();
		return Found;
	}

}
