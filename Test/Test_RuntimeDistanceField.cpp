#include "Test.h"
#include <algorithm>
#include <future>
#include <limits>
#include <random>
#include "../Src/Geometry/RuntimeDistanceField.h"
#include "../Src/CollisionPrimitive/StaticMesh.h"

namespace
{
	using namespace Riemann;
	const float ValueTolerance = 2.0e-5f;
	template<class Function>
	SignedDistanceField3 MakeField(float Radius, int Cells, Function Value)
	{
		SignedDistanceField3 Field;
		const float Spacing = 2 * Radius / Cells;
		EXPECT(Field.Reset(Box3(Vector3(-Radius), Vector3(Radius)), Cells, Cells, Cells, Spacing));
		for (int Z = 0; Z <= Cells; ++Z) for (int Y = 0; Y <= Cells; ++Y) for (int X = 0; X <= Cells; ++X)
			Field.At(X, Y, Z) = Value(Field.GridPointToWorld(X, Y, Z));
		return Field;
	}
	float Multilinear(const Vector3& P)
	{
		return 1 + 2 * P.x - 3 * P.y + 0.5f * P.z + P.x * P.y + 2 * P.y * P.z - 0.25f * P.x * P.z + P.x * P.y * P.z;
	}
	Vector3 MultilinearGradient(const Vector3& P)
	{
		return Vector3(2 + P.y - 0.25f * P.z + P.y * P.z, -3 + P.x + 2 * P.z + P.x * P.z, 0.5f + 2 * P.y - 0.25f * P.x + P.x * P.y);
	}
	class SphereDistanceTarget : public DistanceFieldDistanceTarget
	{
	public:
		Vector3 Center; float Radius;
		SphereDistanceTarget(const Vector3& Position, float Size) : Center(Position), Radius(Size) {}
		bool Evaluate(const Vector3& Position, DistanceFieldTargetPoint& Result) const override
		{
			const Vector3 Delta = Position - Center; const float Length = Delta.Length();
			Result.Normal = Length > 0.0f ? Delta / Length : Vector3::UnitX();
			Result.Point = Center + Result.Normal * Radius; Result.Distance = std::max(0.0f, Length - Radius);
			Result.SupportingPlane = true; return true;
		}
	};
	void TestTargetDistance()
	{
		for (float Scale : { 0.001f, 1.0f, 1000.0f })
		{
			RuntimeDistanceField Field;
			EXPECT(Field.Build(MakeField(1.5f * Scale, 24, [=](const Vector3& P) { return P.Length() - Scale; })));
			SphereDistanceTarget Target(Vector3(2 * Scale, 0, 0), 0.25f * Scale);
			DistanceFieldTargetDistance Result; DistanceFieldQueryStats Stats;
			EXPECT(Field.DistanceToTarget(Target, Result, &Stats));
			printf("SDF target scale=%g lower=%g upper=%g nodes=%llu subdivisions=%llu\n", Scale, Result.LowerBound / Scale, Result.UpperBound / Scale,
				(unsigned long long)Stats.NodeVisits, (unsigned long long)Stats.Subdivisions);
			EXPECT(Result.Converged && Result.Status == DistanceFieldSearchStatus::Complete);
			EXPECT(Result.LowerBound <= 0.75f * Scale && Result.UpperBound >= 0.75f * Scale);
			EXPECT_NEAR(Result.UpperBound / Scale, 0.75f, 0.002f);
			EXPECT(Result.Normal.x < -0.99f);
			DistanceFieldSample AtSurface; EXPECT(Field.Sample(Result.Position, AtSurface)); EXPECT(std::abs(AtSurface.Value / Scale) < 0.0001f);
			Target.Center = Vector3(Scale, 0, 0);
			EXPECT(Field.DistanceToTarget(Target, Result)); EXPECT(Result.UpperBound / Scale < 0.001f);
		}
		RuntimeDistanceField Plane;
		EXPECT(Plane.Build(MakeField(1.0f, 16, [](const Vector3& P) { return P.x * 1.0e-9f; })));
		SphereDistanceTarget Target(Vector3(2, 0.17f, -0.31f), 0.25f);
		DistanceFieldTargetDistance Result; DistanceFieldQueryStats Stats;
		EXPECT(Plane.DistanceToTarget(Target, Result, &Stats)); EXPECT(Result.Converged);
		EXPECT(Result.LowerBound <= 1.75f && Result.UpperBound >= 1.75f);
		EXPECT_NEAR(Result.UpperBound, 1.75f, 0.002f); EXPECT(Stats.Subdivisions < 1000);
		RuntimeDistanceFieldSettings Limited; Limited.MaximumSearchNodes = 1;
		RuntimeDistanceField Short; EXPECT(Short.Build(*Plane.GetField(), Limited));
		Short.DistanceToTarget(Target, Result);
		EXPECT(!Result.Converged && Result.Status == DistanceFieldSearchStatus::WorkLimit);
		EXPECT(Result.LowerBound <= 1.75f && Result.UpperBound >= 1.75f);
	}

	void TestSamplingAndOwnership()
	{
		SignedDistanceField3 Source = MakeField(1.0f, 8, Multilinear);
		RuntimeDistanceField Runtime;
		EXPECT(Runtime.Build(Source));
		EXPECT(Runtime.IsValid() && !Runtime.HasClosedBoundary());
		RuntimeDistanceField Copy = Runtime;
		std::mt19937 Random(2026); std::uniform_real_distribution<float> Coordinate(-1.0f, 1.0f);
		for (int Index = 0; Index < 1000; ++Index)
		{
			const Vector3 Point(Coordinate(Random), Coordinate(Random), Coordinate(Random));
			DistanceFieldSample Sample;
			EXPECT(Runtime.Sample(Point, Sample));
			EXPECT_NEAR(Sample.Value, Multilinear(Point), ValueTolerance);
			EXPECT((Sample.Gradient - MultilinearGradient(Point)).Length() < ValueTolerance);
			EXPECT(Sample.Gradient.Length() <= Runtime.GetGradientBound() + ValueTolerance);
		}
		for (const Vector3& Point : { Vector3(-1), Vector3(1), Vector3(-1, 0.5f, 1), Vector3(0, 1, 0) })
		{
			DistanceFieldSample Sample; EXPECT(Runtime.Sample(Point, Sample));
			EXPECT_NEAR(Sample.Value, Multilinear(Point), ValueTolerance);
			EXPECT((Sample.Gradient - MultilinearGradient(Point)).Length() < ValueTolerance);
		}
		DistanceFieldSample Sample; Sample.Value = 123;
		EXPECT(!Runtime.Sample(Vector3(1.001f, 0, 0), Sample)); EXPECT(Sample.Value == 123);
		EXPECT(!Runtime.Sample(Vector3(std::numeric_limits<float>::quiet_NaN()), Sample));
		Source.At(4, 4, 4) = -999;
		EXPECT(Runtime.Sample(Vector3::Zero(), Sample)); EXPECT_NEAR(Sample.Value, 1.0f, ValueTolerance);
		EXPECT(Runtime.Build(MakeField(1.0f, 4, [](const Vector3&) { return 3.0f; })));
		EXPECT(Copy.Sample(Vector3::Zero(), Sample)); EXPECT_NEAR(Sample.Value, 1.0f, ValueTolerance);
		EXPECT(Runtime.GetSurfaceCellCount() == 0);
		DistanceFieldRayHit Hit; EXPECT(!Runtime.RayCast(Vector3(-2, 0, 0), Vector3::UnitX(), 4.0f, Hit));
		DistanceFieldSurfacePoint Surface; EXPECT(!Runtime.ClosestSurfacePoint(Vector3::Zero(), Surface));
		EXPECT(Surface.Converged && Surface.Status == DistanceFieldSearchStatus::Complete);
		Runtime.Clear(); EXPECT(!Runtime.IsValid() && !Runtime.Sample(Vector3::Zero(), Sample));
	}
	void TestTranslatedGridBoundaries()
	{
		const int Cells = 16;
		const float Spacing = 0.1f;
		const Vector3 Origin(100.13f, -72.71f, 35.33f);
		SignedDistanceField3 Field;
		EXPECT(Field.Reset(Box3(Origin, Origin + Vector3(Cells * Spacing)), Cells, Cells, Cells, Spacing));
		std::mt19937 Random(47); std::uniform_real_distribution<float> Value(-2.0f, 2.0f);
		for (int Z = 0; Z <= Cells; ++Z) for (int Y = 0; Y <= Cells; ++Y) for (int X = 0; X <= Cells; ++X)
			Field.At(X, Y, Z) = Value(Random);
		RuntimeDistanceField Runtime; EXPECT(Runtime.Build(Field));
		for (int Z = 1; Z < Cells; ++Z) for (int Y = 1; Y < Cells; ++Y) for (int X = 1; X < Cells; ++X)
		{
			const Vector3 Vertex = Field.GridPointToWorld(X, Y, Z);
			DistanceFieldSample Sample; EXPECT(Runtime.Sample(Vertex, Sample));
			EXPECT(Sample.Value == Field.At(X, Y, Z));
			const Index3 At(X, Y, Z);
			for (int Axis = 0; Axis < 3; ++Axis) for (int Side : { -1, 1 })
			{
				Vector3 Point = Vertex;
				Point[Axis] = std::nextafter(Vertex[Axis], Side < 0 ? -std::numeric_limits<float>::infinity() : std::numeric_limits<float>::infinity());
				Index3 Neighbor = At; Neighbor[Axis] += Side;
				const Vector3 Endpoint = Field.GridPointToWorld(Neighbor.a, Neighbor.b, Neighbor.c);
				const double Weight = ((double)Point[Axis] - Vertex[Axis]) / ((double)Endpoint[Axis] - Vertex[Axis]);
				const float Expected = (float)((1.0 - Weight) * Field.At(X, Y, Z) + Weight * Field.At(Neighbor.a, Neighbor.b, Neighbor.c));
				EXPECT(Runtime.Sample(Point, Sample));
				EXPECT_NEAR(Sample.Value, Expected, std::numeric_limits<float>::epsilon());
			}
		}
	}
	void TestHierarchy()
	{
		const int Cells = 48;
		SignedDistanceField3 Source = MakeField(1.5f, Cells, [](const Vector3& P) { return P.Length() - 1.0f; });
		RuntimeDistanceField Runtime; EXPECT(Runtime.Build(Source));
		EXPECT(Runtime.HasClosedBoundary() && Runtime.GetNodeCount() > 1 && Runtime.GetSurfaceCellCount() > 0);
		for (float Range : { 0.0f, 0.1f, 0.3f })
		{
			const Box3 Bounds(Vector3(0.8f, -0.15f, -0.15f), Vector3(1.2f, 0.15f, 0.15f));
			std::vector<Index3> Found; DistanceFieldQueryStats Stats;
			Runtime.QueryCells(Bounds, -Range, Range, Found, &Stats);
			std::vector<int> Expected, Actual;
			for (const Index3& C : Found) Actual.push_back((C.c * Cells + C.b) * Cells + C.a);
			for (int Z = 0; Z < Cells; ++Z) for (int Y = 0; Y < Cells; ++Y) for (int X = 0; X < Cells; ++X)
			{
				const Box3 B(Source.GridPointToWorld(X, Y, Z), Source.GridPointToWorld(X + 1, Y + 1, Z + 1));
				if (!B.Intersect(Bounds)) continue;
				float Min = std::numeric_limits<float>::max(), Max = -Min;
				for (int DZ = 0; DZ <= 1; ++DZ) for (int DY = 0; DY <= 1; ++DY) for (int DX = 0; DX <= 1; ++DX)
				{
					const float V = Source.At(X + DX, Y + DY, Z + DZ); Min = std::min(Min, V); Max = std::max(Max, V);
				}
				if (Min <= Range && Max >= -Range) Expected.push_back((Z * Cells + Y) * Cells + X);
			}
			std::sort(Actual.begin(), Actual.end()); std::sort(Expected.begin(), Expected.end());
			EXPECT(Actual == Expected && !Actual.empty());
			EXPECT(Stats.CellTests < (uint64_t)Cells * Cells * Cells / 20);
			printf("SDF range=%g selected=%zu cell tests=%llu full grid=%d\n", Range, Actual.size(), (unsigned long long)Stats.CellTests, Cells * Cells * Cells);
		}
	}
	void TestRayRoots()
	{
		RuntimeDistanceField Runtime;
		// All three roots occur in one cell; a march that only checks its ends misses two.
		SignedDistanceField3 Source; EXPECT(Source.Reset(Box3(Vector3::Zero(), Vector3(1)), 1, 1, 1, 1.0f));
		for (int Z = 0; Z <= 1; ++Z) for (int Y = 0; Y <= 1; ++Y) for (int X = 0; X <= 1; ++X)
			Source.At(X, Y, Z) = (X - 0.2f) * (Y - 0.5f) * (Z - 0.8f);
		EXPECT(Runtime.Build(Source));
		DistanceFieldRayHit Hit;
		for (float Start : { 0.0f, 0.3f, 0.6f })
		{
			const float Target = Start == 0.0f ? 0.2f : (Start < 0.5f ? 0.5f : 0.8f);
			EXPECT(Runtime.RayCast(Vector3(Start), Vector3(1), 1.0f - Start, Hit, false));
			EXPECT_NEAR(Hit.Time, Target - Start, 0.0002f);
			EXPECT(Hit.NormalValid);
			EXPECT(!Runtime.RayCast(Vector3(Start), Vector3(1), Target - Start - 0.01f, Hit, false));
		}
		for (int Z = 0; Z <= 1; ++Z) for (int Y = 0; Y <= 1; ++Y) for (int X = 0; X <= 1; ++X)
			Source.At(X, Y, Z) = (X - 0.5f) * (Y - 0.5f);
		EXPECT(Runtime.Build(Source));
		EXPECT(Runtime.RayCast(Vector3(0, 0, 0.5f), Vector3(1, 1, 0), 1.0f, Hit));
		EXPECT_NEAR(Hit.Time, 0.5f, ValueTolerance);
		EXPECT(!Hit.NormalValid);
		EXPECT(!Runtime.RayCast(Vector3(0, 0, 0.5f), Vector3(1, 1, 0), 0.49f, Hit));
		for (int Z = 0; Z <= 1; ++Z) for (int Y = 0; Y <= 1; ++Y) for (int X = 0; X <= 1; ++X)
			Source.At(X, Y, Z) = (X - 0.5f) * (Y - 0.5f) + 1.0e-6f;
		EXPECT(Runtime.Build(Source));
		EXPECT(!Runtime.RayCast(Vector3(0, 0, 0.5f), Vector3(1, 1, 0), 1.0f, Hit));
		for (int Z = 0; Z <= 1; ++Z) for (int Y = 0; Y <= 1; ++Y) for (int X = 0; X <= 1; ++X)
			Source.At(X, Y, Z) = (X - 0.5f) * 1.0e-9f;
		EXPECT(Runtime.Build(Source));
		EXPECT(Runtime.RayCast(Vector3(1, 0.5f, 0.5f), -Vector3::UnitX(), 1.0f, Hit));
		EXPECT_NEAR(Hit.Time, 0.5f, ValueTolerance);
		EXPECT(Runtime.Build(MakeField(1.0f, 8, [](const Vector3& P) { return P.x - 1.0f; })));
		EXPECT(Runtime.RayCast(Vector3(2, 0, 0), Vector3(-2, 0, 0), 0.5f, Hit));
		EXPECT_NEAR(Hit.Time, 0.5f, ValueTolerance); EXPECT(Hit.Normal.x > 0.99f);
		EXPECT(Runtime.RayCast(Vector3::Zero(), Vector3::UnitX(), 2.0f, Hit));
		EXPECT(Hit.InitialOverlap && Hit.Time == 0.0f);
		EXPECT(Runtime.RayCast(Vector3::Zero(), Vector3::UnitX(), 2.0f, Hit, false));
		EXPECT_NEAR(Hit.Time, 1.0f, ValueTolerance);
		EXPECT(!Runtime.RayCast(Vector3::Zero(), Vector3::Zero(), 2.0f, Hit));
	}
	void TestSphereQueries()
	{
		for (float Scale : { 0.001f, 1.0f, 1000.0f })
		{
			RuntimeDistanceField Runtime;
			EXPECT(Runtime.Build(MakeField(1.5f * Scale, 48, [&](const Vector3& P) { return P.Length() - Scale; })));
			for (float Y : { 0.0f, 0.3f, 0.8f })
			{
				DistanceFieldRayHit Hit; DistanceFieldQueryStats Stats;
				EXPECT(Runtime.RayCast(Vector3(-2, Y, 0) * Scale, Vector3::UnitX(), 4 * Scale, Hit, true, &Stats));
				EXPECT_NEAR(Hit.Time / Scale, 2 - std::sqrt(1 - Y * Y), 0.004f);
				EXPECT(Hit.NormalValid && Hit.Normal.Dot(Hit.Position.Unit()) > 0.995f);
				EXPECT(Stats.CellTests < 100);
			}
			for (const Vector3& P : { Vector3(2, 0, 0), Vector3(1.1f, 0.3f, -0.2f), Vector3(0.2f, 0, 0), Vector3::Zero() })
			{
				DistanceFieldSurfacePoint Surface; DistanceFieldQueryStats Stats;
				EXPECT(Runtime.ClosestSurfacePoint(P * Scale, Surface, 4 * Scale, &Stats));
				EXPECT(Surface.Converged && Surface.Status == DistanceFieldSearchStatus::Complete);
				EXPECT_NEAR(Surface.Distance / Scale, std::abs(P.Length() - 1.0f), 0.004f);
				DistanceFieldSample Sample; EXPECT(Runtime.Sample(Surface.Position, Sample));
				EXPECT(std::abs(Sample.Value) / Scale < 2.0e-5f);
				EXPECT(Surface.NormalValid && Surface.Normal.Dot(Surface.Position.Unit()) > 0.995f);
				printf("SDF closest scale=%g query=%g distance=%g error=%g nodes=%llu splits=%llu converged=%d\n", Scale,
					P.Length(), Surface.Distance / Scale, Surface.ErrorBound / Scale, (unsigned long long)Stats.NodeVisits, (unsigned long long)Stats.Subdivisions, Surface.Converged);
			}
			DistanceFieldSurfacePoint Miss;
			EXPECT(!Runtime.ClosestSurfacePoint(Vector3(2 * Scale, 0, 0), Miss, 0.9f * Scale));
			EXPECT(Miss.Converged);
		}
	}
	void TestDistanceBounds()
	{
		RuntimeDistanceField Runtime;
		const auto Plane = MakeField(2.0f, 8, [](const Vector3& P) { return P.x + P.y + P.z; });
		EXPECT(Runtime.Build(Plane));
		std::mt19937 Random(1207); std::uniform_real_distribution<float> Coordinate(-1.0f, 1.0f);
		for (int Index = 0; Index < 100; ++Index)
		{
			const Vector3 Point(Coordinate(Random), Coordinate(Random), Coordinate(Random));
			DistanceFieldSurfacePoint Surface;
			EXPECT(Runtime.ClosestSurfacePoint(Point, Surface));
			const float Exact = std::abs(Point.x + Point.y + Point.z) / std::sqrt(3.0f);
			EXPECT(Surface.Converged);
			EXPECT(std::abs(Surface.Distance - Exact) <= Surface.ErrorBound + ValueTolerance);
			EXPECT(std::abs(Surface.Position.x + Surface.Position.y + Surface.Position.z) < ValueTolerance);
		}
		RuntimeDistanceFieldSettings Strict; Strict.RelativeDistanceTolerance = 1.0e-10f;
		EXPECT(Runtime.Build(Plane, Strict));
		DistanceFieldSurfacePoint Surface;
		EXPECT(Runtime.ClosestSurfacePoint(Vector3(1), Surface));
		EXPECT(!Surface.Converged && Surface.Status == DistanceFieldSearchStatus::PrecisionLimit);
	}
	void TestConcavityAndBudgets()
	{
		const auto Shell = MakeField(1.5f, 48, [](const Vector3& P) { return std::max(0.5f - P.Length(), P.Length() - 1.0f); });
		RuntimeDistanceField Runtime; EXPECT(Runtime.Build(Shell));
		DistanceFieldSurfacePoint Surface;
		EXPECT(Runtime.ClosestSurfacePoint(Vector3::Zero(), Surface));
		EXPECT_NEAR(Surface.Distance, 0.5f, 0.004f); EXPECT(Surface.Converged);
		EXPECT(Surface.Normal.Dot(Surface.Position.Unit()) < -0.99f);
		DistanceFieldRayHit Hit;
		EXPECT(Runtime.RayCast(Vector3::Zero(), Vector3::UnitX(), 2.0f, Hit));
		EXPECT_NEAR(Hit.Time, 0.5f, ValueTolerance); EXPECT(!Hit.InitialOverlap && Hit.Normal.x < -0.99f);
		RuntimeDistanceFieldSettings Budget; Budget.MaximumSearchNodes = 1;
		EXPECT(Runtime.Build(Shell, Budget));
		Runtime.ClosestSurfacePoint(Vector3::Zero(), Surface);
		EXPECT(!Surface.Converged && Surface.Status == DistanceFieldSearchStatus::WorkLimit);
		const auto Torus = MakeField(1.5f, 48, [](const Vector3& P)
		{
			const float Ring = std::sqrt(P.x * P.x + P.z * P.z) - 0.9f;
			return std::sqrt(Ring * Ring + P.y * P.y) - 0.25f;
		});
		EXPECT(Runtime.Build(Torus));
		EXPECT(!Runtime.RayCast(Vector3(0, 2, 0), -Vector3::UnitY(), 4.0f, Hit));
		EXPECT(Runtime.RayCast(Vector3(0.9f, 2, 0), -Vector3::UnitY(), 4.0f, Hit));
		EXPECT_NEAR(Hit.Time, 1.75f, 0.006f);
		DistanceFieldQueryStats Stats;
		EXPECT(Runtime.ClosestSurfacePoint(Vector3::Zero(), Surface, 4.0f, &Stats));
		printf("SDF torus distance=%g error=%g nodes=%llu splits=%llu converged=%d\n", Surface.Distance, Surface.ErrorBound,
			(unsigned long long)Stats.NodeVisits, (unsigned long long)Stats.Subdivisions, Surface.Converged);
		EXPECT_NEAR(Surface.Distance, 0.65f, 0.006f); EXPECT(Surface.Converged);
	}
	void TestValidationAndMeshCooking()
	{
		SignedDistanceField3 Field;
		EXPECT(!Field.Reset(Box3(Vector3(-1), Vector3(1)), std::numeric_limits<int>::max(), 1, 1, 1.0f));
		EXPECT(!Field.Reset(Box3(Vector3(-1), Vector3(1)), 2, 2, 2, std::numeric_limits<float>::infinity()));
		Field = MakeField(1.0f, 8, [](const Vector3& P) { return P.y; });
		RuntimeDistanceField Runtime; EXPECT(Runtime.Build(Field));
		const auto* Original = Runtime.GetField();
		Field.At(0, 0, 0) = std::numeric_limits<float>::quiet_NaN();
		std::string Error; EXPECT(!Runtime.Build(Field, RuntimeDistanceFieldSettings(), &Error));
		EXPECT(!Error.empty() && Runtime.GetField() == Original);
		EXPECT(Field.Reset(Box3(Vector3(-1), Vector3(1)), 2, 2, 2, 0.5f));
		EXPECT(!Runtime.Build(Field, RuntimeDistanceFieldSettings(), &Error));
		// Accepted metadata roundoff must not permit extrapolation outside the cells.
		SignedDistanceField3 RoundedBounds;
		const float AboveOne = std::nextafter(1.0f, 2.0f);
		EXPECT(RoundedBounds.Reset(Box3(Vector3(-1), Vector3(AboveOne)), 2, 2, 2, 1.0f));
		RuntimeDistanceField RoundedRuntime; EXPECT(RoundedRuntime.Build(RoundedBounds));
		EXPECT(RoundedRuntime.GetBounds().Max == Vector3(1));
		DistanceFieldSample RoundedSample;
		EXPECT(RoundedRuntime.Sample(Vector3(1), RoundedSample));
		EXPECT(!RoundedRuntime.Sample(Vector3(AboveOne), RoundedSample));
		RuntimeDistanceFieldSettings Settings; Settings.RelativeValueTolerance = -1;
		EXPECT(!Runtime.Build(*Original, Settings, &Error));
		EXPECT(Runtime.GetField() == Original);

		StaticMesh Mesh; Mesh.AddAABB(Vector3(-0.5f), Vector3(0.5f)); Mesh.Compact();
		std::vector<uint32_t> Indices;
		for (uint32_t T = 0; T < Mesh.NumTriangles; ++T)
		{
			uint32_t A, B, C; Mesh.GetVertIndices(T, A, B, C);
			// Winding-number cooking requires a consistently oriented source surface.
			if ((Mesh.Vertices[B] - Mesh.Vertices[A]).Cross(Mesh.Vertices[C] - Mesh.Vertices[A]).Dot(Mesh.Vertices[A]) < 0.0f) std::swap(B, C);
			Indices.push_back(A); Indices.push_back(B); Indices.push_back(C);
		}
		SignedDistanceFieldBuildOptions Options; Options.Resolution = 24;
		EXPECT(SignedDistanceField3::BuildFromTriangleSoup(Mesh.Vertices, Mesh.NumVertices, Indices.data(), Mesh.NumTriangles, Field, Options));
		EXPECT(Runtime.Build(Field)); EXPECT(Runtime.HasClosedBoundary());
		DistanceFieldRayHit Hit; EXPECT(Runtime.RayCast(Vector3(0, 2, 0), -Vector3::UnitY(), 4.0f, Hit));
		EXPECT_NEAR(Hit.Time, 1.5f, 0.01f);
		std::vector<std::future<bool>> Jobs;
		for (int Worker = 0; Worker < 4; ++Worker) Jobs.push_back(std::async(std::launch::async, [Runtime]()
		{
			for (int Step = 0; Step < 100; ++Step)
			{
				DistanceFieldRayHit Hit; DistanceFieldSample Sample;
				if (!Runtime.RayCast(Vector3(0, 2, 0), -Vector3::UnitY(), 4.0f, Hit)
					|| !Runtime.Sample(Vector3::Zero(), Sample) || Sample.Value >= 0.0f || std::abs(Hit.Time - 1.5f) > 0.01f) return false;
			}
			return true;
		}));
		for (auto& Job : Jobs) EXPECT(Job.get());
	}
}

void TestRuntimeDistanceField()
{
	printf("Running runtime SDF interpolation, hierarchy, cubic roots, closest surfaces and cooking\n");
	TestTargetDistance(); TestSamplingAndOwnership(); TestTranslatedGridBoundaries(); TestHierarchy(); TestRayRoots(); TestSphereQueries(); TestDistanceBounds(); TestConcavityAndBudgets(); TestValidationAndMeshCooking();
}
