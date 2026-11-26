#include "Test.h"
#include "../Src/Collision/SurfaceCollision.h"
#include "../Src/Maths/Quaternion.h"
#include <algorithm>
#include <array>
#include <stdexcept>
#include <chrono>
#include "../Src/Deformation/SurfaceContactConstraint.h"

namespace
{
	using namespace Riemann;
	SurfaceMotion Feature(std::initializer_list<Vector3> Points, const Vector3& Displacement = Vector3::Zero())
	{
		SurfaceMotion M; M.Count = (int)Points.size(); int I = 0;
		for (const auto& P : Points) { M.Start[I] = P; M.End[I++] = P + Displacement; } return M;
	}
	SurfaceMotion Triangle() { return Feature({ Vector3(0, 0, 0), Vector3(1, 0, 0), Vector3(0, 1, 0) }); }
	Vector3 Evaluate(const SurfaceMotion& M, const Vector3& Weights, double Time)
	{
		Vector3 P;
		for (int Axis = 0; Axis < 3; ++Axis)
		{
			double Coordinate = 0;
			for (int I = 0; I < M.Count; ++I) Coordinate += ((double)M.Start[I][Axis] + ((double)M.End[I][Axis] - M.Start[I][Axis]) * Time) * Weights[I];
			P[Axis] = (float)Coordinate;
		}
		return P;
	}
	void CheckWitness(const SurfaceMotion& A, const SurfaceMotion& B, const SurfaceContact& C, bool UnitNormal = true)
	{
		EXPECT(std::isfinite(C.Distance) && C.Distance >= 0 && C.Time >= 0 && C.Time <= 1);
		for (int I = 0; I < 3; ++I)
		{
			EXPECT(C.WeightsA[I] >= 0 && C.WeightsB[I] >= 0);
			if (I >= A.Count) EXPECT(C.WeightsA[I] == 0); if (I >= B.Count) EXPECT(C.WeightsB[I] == 0);
		}
		EXPECT_NEAR(C.WeightsA.x + C.WeightsA.y + C.WeightsA.z, 1, 2e-6f);
		EXPECT_NEAR(C.WeightsB.x + C.WeightsB.y + C.WeightsB.z, 1, 2e-6f);
		EXPECT_NEAR((Evaluate(A, C.WeightsA, C.Time) - C.PointA).Length(), 0, 3e-5f);
		EXPECT_NEAR((Evaluate(B, C.WeightsB, C.Time) - C.PointB).Length(), 0, 3e-5f);
		EXPECT_NEAR((C.PointA - C.PointB).Length(), C.Distance, 3e-5);
		if (UnitNormal) EXPECT_NEAR(C.Normal.Length(), 1, 2e-6f);
	}
	void DistanceFeatures()
	{
		auto T = Triangle(); auto P = Feature({ Vector3(0.2f, 0.3f, 2) });
		auto C = SurfaceCollision::Closest(P.Start, P.Count, T.Start, T.Count); CheckWitness(P, T, C);
		EXPECT_NEAR(C.Distance, 2, 1e-12); EXPECT_NEAR((C.WeightsB - Vector3(0.5f, 0.2f, 0.3f)).Length(), 0, 1e-6f);
		P = Feature({ Vector3(2, 0, 0) }); C = SurfaceCollision::Closest(P.Start, 1, T.Start, 3); CheckWitness(P, T, C);
		EXPECT_NEAR(C.Distance, 1, 1e-12); EXPECT(C.WeightsB == Vector3(0, 1, 0));
		auto A = Feature({ Vector3(-1, 0, 0), Vector3(1, 0, 0) }); auto B = Feature({ Vector3(0, -1, 1), Vector3(0, 1, 1) });
		C = SurfaceCollision::Closest(A.Start, 2, B.Start, 2); CheckWitness(A, B, C);
		EXPECT_NEAR(C.Distance, 1, 1e-12); EXPECT_NEAR(C.WeightsA.x, 0.5f, 1e-6f); EXPECT_NEAR(C.WeightsB.x, 0.5f, 1e-6f);
		for (float Angle : { 1e-3f, 1e-8f, 1e-16f })
		{
			B = Feature({ Vector3(-1, Angle, 0), Vector3(1, -Angle, 0) }); C = SurfaceCollision::Closest(A.Start, 2, B.Start, 2);
			CheckWitness(A, B, C, false); EXPECT_NEAR(C.Distance, 0, 1e-20); EXPECT_NEAR(C.WeightsA.x, 0.5f, 1e-6f);
		}
		A = Feature({ Vector3(0.2f, 0.2f, -1), Vector3(0.2f, 0.2f, 1) }); C = SurfaceCollision::Closest(A.Start, 2, T.Start, 3);
		CheckWitness(A, T, C, false); EXPECT_NEAR(C.Distance, 0, 1e-12); EXPECT_NEAR(C.WeightsA.x, 0.5f, 1e-6f);
		// Neither endpoint lies near the face; this is the capsule/large-triangle
		// case that vertex-only cloth collision cannot detect.
		EXPECT_NEAR((C.WeightsB - Vector3(0.6f, 0.2f, 0.2f)).Length(), 0, 1e-6f);
		B = Feature({ Vector3(0.2f, 0.2f, 0), Vector3(0.3f, 0.2f, 0), Vector3(0.2f, 0.3f, 0) });
		C = SurfaceCollision::Closest(T.Start, 3, B.Start, 3); CheckWitness(T, B, C, false); EXPECT_NEAR(C.Distance, 0, 1e-12);
		B = Feature({ Vector3(0.2f, 0.2f, -1), Vector3(0.2f, 0.2f, 1), Vector3(2, 2, 1) });
		C = SurfaceCollision::Closest(T.Start, 3, B.Start, 3); CheckWitness(T, B, C, false); EXPECT_NEAR(C.Distance, 0, 1e-12);
		// Current triangles can collapse even though their rest mesh is valid.
		B = Feature({ Vector3(-1, 0, 1), Vector3(0, 0, 1), Vector3(1, 0, 1) });
		C = SurfaceCollision::Closest(T.Start, 3, B.Start, 3); CheckWitness(T, B, C); EXPECT_NEAR(C.Distance, 1, 1e-12);
		A = Feature({ Vector3(0, 0, 0), Vector3(0, 0, 0) }); B = Feature({ Vector3(0, 2, 0), Vector3(0, 2, 0), Vector3(0, 2, 0) });
		C = SurfaceCollision::Closest(A.Start, 2, B.Start, 3); CheckWitness(A, B, C); EXPECT_NEAR(C.Distance, 2, 1e-12);
	}
	void ContinuousFeatures()
	{
		SurfaceCollisionSettings S; S.Thickness = 0.1; SurfaceContact C; const auto T = Triangle();
		auto P = Feature({ Vector3(0.2f, 0.2f, 1) }, Vector3(0, 0, -2));
		EXPECT(SurfaceCollision::Sweep(P, T, S, C)); CheckWitness(P, T, C); EXPECT(!C.Conservative && !C.InitialOverlap);
		EXPECT_NEAR(C.Time, 0.45, 2e-6); EXPECT(C.Normal.z > 0.99f);
		P = Feature({ Vector3(0.2f, 0.2f, 1000) }, Vector3(0, 0, -2000));
		EXPECT(SurfaceCollision::Sweep(P, T, S, C)); CheckWitness(P, T, C); EXPECT(!C.Conservative);
		EXPECT_NEAR(C.Time, (1000 - S.Thickness) / 2000, 1e-8);
		auto A = Feature({ Vector3(-2, 0, 1), Vector3(2, 0, 1) }, Vector3(0, 0, -2));
		auto B = Feature({ Vector3(0, -2, 0), Vector3(0, 2, 0) });
		EXPECT(SurfaceCollision::Sweep(A, B, S, C)); CheckWitness(A, B, C); EXPECT_NEAR(C.Time, 0.45, 3e-6);
		EXPECT_NEAR(C.WeightsA.x, 0.5f, 1e-6f); EXPECT_NEAR(C.WeightsB.x, 0.5f, 1e-6f);
		A = Feature({ Vector3(-2, -2, 0), Vector3(2, -2, 0), Vector3(0, 2, 0) }, Vector3(0, 0, 5));
		B = Feature({ Vector3(-1, 0, 3), Vector3(1, 0, 3) }); S.Thickness = 0.2;
		EXPECT(SurfaceCollision::Sweep(A, B, S, C)); CheckWitness(A, B, C); EXPECT_NEAR(C.Time, 2.8 / 5, 3e-6); EXPECT(C.Normal.z < -0.99f);
		// Coplanar entry has no oriented-volume sign change.
		P = Feature({ Vector3(-1, 0.2f, 0) }, Vector3(1.8f, 0, 0)); S.Thickness = 0;
		EXPECT(SurfaceCollision::Sweep(P, T, S, C)); CheckWitness(P, T, C); EXPECT_NEAR(C.Time, 1 / 1.8, 3e-6); EXPECT(C.Normal.x < -0.99f);
		P = Feature({ Vector3(0.2f, 0.2f, 1) }, Vector3(0, 0, -1));
		EXPECT(SurfaceCollision::Sweep(P, T, S, C)); CheckWitness(P, T, C); EXPECT_NEAR(C.Time, 1, 3e-6);
		P = Feature({ Vector3(0.2f, 0.2f, 0) });
		EXPECT(SurfaceCollision::Sweep(P, T, S, C, -Vector3::UnitZ())); CheckWitness(P, T, C); EXPECT(C.InitialOverlap); EXPECT(C.Normal.z == -1);
		P = Feature({ Vector3(3, 3, 1) }, Vector3(0, 0, -2)); EXPECT(!SurfaceCollision::Sweep(P, T, S, C)); EXPECT(!std::isfinite(C.Distance));
	}
	void GrazingBudgetAndCovariance()
	{
		SurfaceCollisionSettings S; S.Thickness = 0.1; S.RelativeTolerance = 0; SurfaceContact C;
		auto A = Feature({ Vector3(-1, 0.1f, 0) }, Vector3(2, 0, 0)), B = Feature({ Vector3::Zero() });
		EXPECT(SurfaceCollision::Sweep(A, B, S, C)); CheckWitness(A, B, C); EXPECT(C.Time < 0.5);
		if (C.Conservative) EXPECT(C.Iterations == S.MaxIterations);
		S.MaxIterations = 4096; EXPECT(SurfaceCollision::Sweep(A, B, S, C)); EXPECT(!C.Conservative); EXPECT_NEAR(C.Time, 0.5, 0.001);
		A.Start[0].y = A.End[0].y = 0.101f; EXPECT(!SurfaceCollision::Sweep(A, B, S, C));
		S.MaxIterations = 1; A = Feature({ Vector3(0, 0, 2) }, Vector3(0, 0, -4));
		EXPECT(SurfaceCollision::Sweep(A, B, S, C)); EXPECT(C.Conservative && C.Time == 0 && C.Iterations == 1);
		S = SurfaceCollisionSettings(); S.Thickness = 0.1; A = Feature({ Vector3(0.2f, 0.2f, 1) }, Vector3(0, 0, -2)); B = Triangle();
		EXPECT(SurfaceCollision::Sweep(A, B, S, C)); const auto Original = C;
		Quaternion Rotation; Rotation.FromRotationAxis(Vector3(1, 2, 3), 0.7f); const Vector3 Translation(13, -29, 17), CommonMotion(1, 2, -3);
		for (float Scale : { 0.001f, 1.0f, 1000.0f })
		{
			auto AA = A, BB = B;
			for (auto* M : { &AA, &BB }) for (int I = 0; I < M->Count; ++I)
			{
				M->Start[I] = (Rotation * M->Start[I] + Translation) * Scale;
				M->End[I] = (Rotation * M->End[I] + Translation + CommonMotion) * Scale;
			}
			auto Scaled = S; Scaled.Thickness *= Scale; Scaled.DistanceTolerance *= Scale;
			EXPECT(SurfaceCollision::Sweep(AA, BB, Scaled, C)); EXPECT_NEAR(C.Time, Original.Time, 4e-6);
			EXPECT_NEAR((C.Normal - Rotation * Original.Normal).Length(), 0, 5e-5f);
			SurfaceContact Reverse; EXPECT(SurfaceCollision::Sweep(BB, AA, Scaled, Reverse));
			EXPECT_NEAR(C.Time, Reverse.Time, 1e-12); EXPECT_NEAR((C.Normal + Reverse.Normal).Length(), 0, 1e-6f);
		}
	}
	using CandidateKey = std::array<int, 3>;
	std::vector<CandidateKey> Keys(const std::vector<SurfaceCandidate>& Candidates, bool Sort = false)
	{
		std::vector<CandidateKey> Result;
		for (const auto& C : Candidates) Result.push_back({ (int)C.Type, C.A, C.B });
		if (Sort) std::sort(Result.begin(), Result.end()); return Result;
	}
	Box3 VertexBounds(const std::vector<Vector3>& Start, const std::vector<Vector3>& End, int I, float Margin)
	{
		return Box3(Start[I].Min(End[I]) - Vector3(Margin), Start[I].Max(End[I]) + Vector3(Margin));
	}
	std::vector<Index3> GridFaces(int Width, int Height)
	{
		std::vector<Index3> Faces;
		for (int Y = 1; Y < Height; ++Y) for (int X = 1; X < Width; ++X)
		{
			const int A = (Y - 1) * Width + X - 1, B = A + 1, C = A + Width, D = C + 1;
			Faces.emplace_back(A, B, C); Faces.emplace_back(B, D, C);
		}
		return Faces;
	}
	void SurfaceIndexAgainstBruteForce(SurfaceAcceleration Acceleration)
	{
		const int Width = 5, Height = 4, Count = Width * Height;
		const auto FacesA = GridFaces(Width, Height), FacesB = GridFaces(Height, Width);
		SurfaceCollisionMesh A(Count, FacesA, {}, Acceleration), B(Count, FacesB, {}, Acceleration);
		std::vector<Vector3> StartA(Count), EndA(Count), StartB(Count), EndB(Count);
		uint32_t State = 91723;
		auto Random = [&]() { State = 1664525 * State + 1013904223; return (float)(State >> 8) / (float)(1 << 24); };
		auto RandomVector = [&]() { return Vector3(Random(), Random(), Random()) - Vector3(0.5f); };
		const float MarginA = 0.02f, MarginB = 0.03f;
		for (int Frame = 0; Frame < 24; ++Frame)
		{
			for (int I = 0; I < Count; ++I)
			{
				StartA[I] = Vector3((float)(I % Width), (float)(I / Width), 0) + RandomVector();
				StartB[I] = Vector3((float)(I % Height), (float)(I / Height), 0.25f) + RandomVector();
				EndA[I] = StartA[I] + RandomVector() * 2; EndB[I] = StartB[I] + RandomVector() * 2;
			}
			A.Update(StartA, EndA, MarginA); B.Update(StartB, EndB, MarginB);
			std::vector<Box3> VA, VB, TA, TB, EA, EB;
			for (int I = 0; I < Count; ++I) { VA.push_back(VertexBounds(StartA, EndA, I, MarginA)); VB.push_back(VertexBounds(StartB, EndB, I, MarginB)); }
			for (const auto& T : FacesA) TA.emplace_back(Box3(VA[T.a], VA[T.b]), VA[T.c]);
			for (const auto& T : FacesB) TB.emplace_back(Box3(VB[T.a], VB[T.b]), VB[T.c]);
			for (const auto& E : A.GetEdges()) EA.emplace_back(VA[E.A], VA[E.B]);
			for (const auto& E : B.GetEdges()) EB.emplace_back(VB[E.A], VB[E.B]);
			std::vector<CandidateKey> Expected;
			for (size_t I = 0; I < VA.size(); ++I) for (size_t J = 0; J < TB.size(); ++J)
				if (VA[I].Intersect(TB[J])) Expected.push_back({ (int)SurfacePairType::VertexTriangle, (int)I, (int)J });
			for (size_t I = 0; I < TA.size(); ++I) for (size_t J = 0; J < VB.size(); ++J)
				if (TA[I].Intersect(VB[J])) Expected.push_back({ (int)SurfacePairType::TriangleVertex, (int)I, (int)J });
			for (size_t I = 0; I < EA.size(); ++I) for (size_t J = 0; J < EB.size(); ++J)
				if (EA[I].Intersect(EB[J])) Expected.push_back({ (int)SurfacePairType::EdgeEdge, (int)I, (int)J });
			std::sort(Expected.begin(), Expected.end());
			std::vector<SurfaceCandidate> Actual; A.FindCandidates(B, Actual); const auto Sorted = Keys(Actual, true);
			EXPECT(Sorted == Expected); EXPECT(std::adjacent_find(Sorted.begin(), Sorted.end()) == Sorted.end());
			// A fresh tree and a repeatedly refitted tree must produce the same order.
			SurfaceCollisionMesh FreshA(Count, FacesA, {}, Acceleration), FreshB(Count, FacesB, {}, Acceleration);
			FreshA.Update(StartA, EndA, MarginA); FreshB.Update(StartB, EndB, MarginB);
			std::vector<SurfaceCandidate> Fresh; FreshA.FindCandidates(FreshB, Fresh); EXPECT(Keys(Fresh) == Keys(Actual));
			SurfaceCollisionMesh Reference(Count, FacesA); Reference.Update(StartA, EndA, MarginA);
			for (bool ExcludeNeighbors : { false, true })
			{
				A.FindSelfCandidates(Actual, ExcludeNeighbors); Reference.FindSelfCandidates(Fresh, ExcludeNeighbors);
				EXPECT(Keys(Actual) == Keys(Fresh));
			}
			Reference.FindCandidates(B, Actual); EXPECT(Keys(Actual, true) == Expected);
		}
	}
	void SurfaceIndexTopologyAndValidation(SurfaceAcceleration Acceleration)
	{
		std::vector<SurfaceCandidate> Candidates; std::vector<int> Hits;
		const std::vector<Index3> Quad = { Index3(0, 1, 2), Index3(1, 3, 2) };
		SurfaceCollisionMesh Connected(4, Quad, {}, Acceleration); const std::vector<Vector3> Collapsed(4, Vector3::Zero());
		auto Reject = [](auto Function) { bool Failed = false; try { Function(); } catch (const std::exception&) { Failed = true; } EXPECT(Failed); };
		EXPECT(!Connected.IsReady()); Reject([&] { Connected.GetVertexBounds(0); });
		Reject([&] { Connected.FindSelfCandidates(Candidates); });
		Connected.Update(Collapsed, Collapsed, 0); Connected.FindSelfCandidates(Candidates); EXPECT(Candidates.empty());
		Connected.FindSelfCandidates(Candidates, false); EXPECT(Candidates.size() == 4);
		const auto Expected = Keys(Candidates); Connected.FindCandidates(Connected, Candidates); EXPECT(Candidates.empty());
		const Box3 Saved = Connected.GetVertexBounds(0);
		auto Bad = Collapsed; Bad[2].x = std::numeric_limits<float>::quiet_NaN();
		Reject([&] { Connected.Update(Collapsed, Bad, 0); });
		Reject([&] { Connected.Update(Collapsed, Collapsed, -1); });
		Reject([&] { Connected.Update({}, {}, 0); });
		EXPECT(Connected.GetVertexBounds(0) == Saved);
		Connected.FindSelfCandidates(Candidates, false); EXPECT(Keys(Candidates) == Expected);
		Reject([&] { SurfaceCollisionMesh Invalid(3, { Index3(0, 1, 3) }); });
		Reject([&] { SurfaceCollisionMesh Invalid(3, { Index3(0, 1, 1) }); });
		Reject([&] { SurfaceCollisionMesh Invalid(3, { Index3(0, 1, 2), Index3(2, 1, 0) }); });
		Reject([&] { SurfaceCollisionMesh Invalid(0, {}); });
		Reject([&] { Connected.QueryTriangles(Box3(Vector3(1), Vector3(-1)), Hits); });
		Reject([&] { DynamicAABBTree Invalid(-1); });
		Reject([&] { DynamicAABBTree Invalid(std::numeric_limits<float>::infinity()); });
		// Disconnected panels may have identical positions without sharing topology.
		SurfaceCollisionMesh Disconnected(6, { Index3(0, 1, 2), Index3(3, 4, 5) }, {}, Acceleration);
		const std::vector<Vector3> Six(6, Vector3::Zero()); Disconnected.Update(Six, Six, 0);
		Disconnected.FindSelfCandidates(Candidates); EXPECT(Candidates.size() == 15);
		SurfaceCollisionMesh First(3, { Index3(0, 1, 2) }, {}, Acceleration), Second(3, { Index3(0, 1, 2) }, {}, Acceleration);
		const std::vector<Vector3> Three(3, Vector3::Zero()); First.Update(Three, Three, 0); Second.Update(Three, Three, 0);
		First.FindCandidates(Second, Candidates); EXPECT(Candidates.size() == 15);
		// No inherited rigid-body padding may bridge this gap between tiny surfaces.
		const std::vector<Vector3> Raised(3, Vector3(0, 0, 0.001f)); Second.Update(Raised, Raised, 0.00001f);
		First.FindCandidates(Second, Candidates); EXPECT(Candidates.empty());
		First.Update(Raised, std::vector<Vector3>(3, Vector3(0, 0, -0.001f)), 0);
		First.FindCandidates(Second, Candidates); EXPECT(Candidates.size() == 15);
	}
	void SurfaceIndexSeparatedPanels(SurfaceAcceleration Acceleration)
	{
		const int PanelCount = 1024; std::vector<Vector3> Positions; std::vector<Index3> Faces;
		for (int I = 0; I < PanelCount; ++I)
		{
			const float X = (float)I * 4; const int Base = (int)Positions.size();
			Positions.emplace_back(X, 0, 0); Positions.emplace_back(X + 1, 0, 0); Positions.emplace_back(X, 1, 0);
			Faces.emplace_back(Base, Base + 1, Base + 2);
		}
		SurfaceCollisionMesh Mesh((int)Positions.size(), Faces, {}, Acceleration); Mesh.Update(Positions, Positions, 0.01f);
		std::vector<SurfaceCandidate> Candidates; Mesh.FindSelfCandidates(Candidates); EXPECT(Candidates.empty());
		// Move one isolated panel onto another, retaining all other leaves.
		for (int I = 0; I < 3; ++I) Positions[I] = Positions[I + 3];
		Mesh.Update(Positions, Positions, 0.01f); Mesh.FindSelfCandidates(Candidates);
		EXPECT(Candidates.size() >= 6 && Candidates.size() <= 15);
		for (const auto& C : Candidates) EXPECT(C.A < 6 && C.B < 6);
	}
	void HierarchicalHashQueries()
	{
		SpatialHashHierarchical Hash; std::vector<Box3> Boxes; std::vector<int> Hits;
		uint32_t State = 13839; auto Random = [&]() { State = 1664525 * State + 1013904223; return (float)(State >> 8) / (float)(1 << 24); };
		for (int I = 0; I < 256; ++I)
		{
			const float Size = std::ldexp(1.0f, I % 18 - 10);
			const Vector3 P(8 * Random() - 4, 8 * Random() - 4, 8 * Random() - 4);
			Boxes.emplace_back(P, P + Vector3(Size, Size * Random(), Size * Random()));
		}
		// Exact cell boundaries, collapsed features, negative coordinates and
		// mixed scales must all retain inclusive AABB overlap semantics.
		Boxes.emplace_back(Vector3(-2), Vector3(-1)); Boxes.emplace_back(Vector3(-1), Vector3::Zero());
		Boxes.emplace_back(Vector3::Zero(), Vector3::Zero());
		for (int Pass = 0; Pass < 3; ++Pass)
		{
			Hash.Build(Boxes);
			auto Check = [&](const Box3& Q)
			{
				Hash.Query(Q, Hits); std::vector<int> Expected;
				for (size_t I = 0; I < Boxes.size(); ++I) if (Boxes[I].Intersect(Q)) Expected.push_back((int)I);
				EXPECT(Hits == Expected);
			};
			for (const auto& Box : Boxes) { Check(Box); Check(Box3(Box.Min, Box.Min)); Check(Box3(Box.Max, Box.Max)); }
			Check(Box3(Vector3(-1e30f), Vector3(1e30f)));
			Check(Box3(Vector3(1e35f), Vector3(2e35f)));
			for (auto& Box : Boxes) { Box.Min = Box.Min * 8 + Vector3(-100); Box.Max = Box.Max * 8 + Vector3(-100); }
		}
		Hash.Build({ Box3(Vector3(-1e30f), Vector3(-1e30f)), Box3(Vector3(1e30f), Vector3(1e30f)) });
		Hash.Query(Box3(Vector3(-1e30f), Vector3(1e30f)), Hits); EXPECT(Hits == std::vector<int>({ 0, 1 }));
		Hash.Build({}); Hash.Query(Box3(Vector3(-1), Vector3(1)), Hits); EXPECT(Hits.empty());
	}
	void SurfaceIndexBenchmark()
	{
		const int Width = 25, Height = 21; const auto Faces = GridFaces(Width, Height);
		SurfaceCollisionMesh Tree(Width * Height, Faces), Hash(Width * Height, Faces, {}, SurfaceAcceleration::HierarchicalHash);
		std::vector<Vector3> P(Width * Height), Previous; std::vector<SurfaceCandidate> A, B;
		double TreeTime = 0, HashTime = 0;
		for (int Frame = 0; Frame < 60; ++Frame)
		{
			for (int I = 0; I < Width * Height; ++I)
			{
				const float X = (I % Width) * 0.4f, Y = (I / Width) * 0.4f;
				P[I] = Vector3(X, Y, 0.3f * std::sin(X + Frame * 0.1f) * std::cos(Y));
			}
			if (Previous.empty()) Previous = P;
			const auto Start = std::chrono::steady_clock::now(); Tree.Update(Previous, P, 0.1f); Tree.FindSelfCandidates(A);
			const auto Middle = std::chrono::steady_clock::now(); Hash.Update(Previous, P, 0.1f); Hash.FindSelfCandidates(B);
			const auto End = std::chrono::steady_clock::now();
			TreeTime += std::chrono::duration<double, std::milli>(Middle - Start).count();
			HashTime += std::chrono::duration<double, std::milli>(End - Middle).count();
			EXPECT(Keys(A) == Keys(B)); Previous = P;
		}
		printf("Surface index update + self query: tree %.3f ms, hash %.3f ms\n", TreeTime / 60, HashTime / 60);
	}
	void RefinedSweeps()
	{
		SurfaceCollisionSettings S; S.Thickness = 0.1; S.RelativeTolerance = 0; S.MaxIterations = 1; SurfaceContact C;
		const auto T = Triangle(); auto P = Feature({ Vector3(0.2f, 0.2f, 1) }, Vector3(0, 0, -2));
		EXPECT(SurfaceCollision::SweepRefined(P, T, S, 0, C)); EXPECT(C.Conservative && C.Time == 0);
		EXPECT(SurfaceCollision::SweepRefined(P, T, S, 64, C)); EXPECT(!C.Conservative); CheckWitness(P, T, C);
		EXPECT_NEAR(C.Time, 0.45, 2e-6);
		// Search left intervals first even though the endpoints are separated.
		P = Feature({ Vector3(-1, 0, 0) }, Vector3(2, 0, 0)); const auto Point = Feature({ Vector3::Zero() });
		EXPECT(SurfaceCollision::SweepRefined(P, Point, S, 64, C)); EXPECT(!C.Conservative); EXPECT_NEAR(C.Time, 0.45, 2e-6);
		P = Feature({ Vector3(-1, 0.2f, 0) }, Vector3(2, 0, 0));
		EXPECT(!SurfaceCollision::SweepRefined(P, Point, S, 128, C));
		// Interval endpoints stay in double precision; float slicing can collapse
		// a short unresolved interval and accidentally certify the wrong trajectory.
		S.MaxIterations = 4; P = Feature({ Vector3(0.2f, 0.2f, 1000000) }, Vector3(0, 0, -2000000));
		EXPECT(SurfaceCollision::SweepRefined(P, T, S, 64, C)); EXPECT(!C.Conservative);
		EXPECT_NEAR(C.Time, (1000000 - S.Thickness) / 2000000, 1e-11);
		auto Reject = [](auto Function) { bool Failed = false; try { Function(); } catch (const std::invalid_argument&) { Failed = true; } EXPECT(Failed); };
		Reject([&] { SurfaceCollision::SweepRefined(P, T, S, -1, C); });
		P.Count = 4; Reject([&] { SurfaceCollision::SweepRefined(P, T, S, 64, C); });
	}
	void SampledNoMissAndValidation()
	{
		uint32_t State = 24681357; auto Random = [&]() { State = 1664525 * State + 1013904223; return (float)(State >> 8) / (float)(1 << 24); };
		auto RandomVector = [&]() { return Vector3(2 * Random() - 1, 2 * Random() - 1, 2 * Random() - 1); };
		SurfaceCollisionSettings S; S.Thickness = 0.08; int SampledImpacts = 0, Conservative = 0;
		for (int Trial = 0; Trial < 240; ++Trial)
		{
			SurfaceMotion A, B; A.Count = 1 + Trial % 3; B.Count = 1 + (Trial / 3) % 3;
			for (auto* M : { &A, &B }) for (int I = 0; I < M->Count; ++I) { M->Start[I] = RandomVector(); M->End[I] = RandomVector(); }
			SurfaceContact Contact; const bool Hit = SurfaceCollision::Sweep(A, B, S, Contact);
			if (Hit) { CheckWitness(A, B, Contact); Conservative += Contact.Conservative; }
			for (int Sample = 0; Sample <= 64; ++Sample)
			{
				const float Time = (float)Sample / 64; Vector3 PA[3], PB[3];
				for (int I = 0; I < A.Count; ++I) PA[I] = A.Start[I] + (A.End[I] - A.Start[I]) * Time;
				for (int I = 0; I < B.Count; ++I) PB[I] = B.Start[I] + (B.End[I] - B.Start[I]) * Time;
				const auto Discrete = SurfaceCollision::Closest(PA, A.Count, PB, B.Count);
				if (Discrete.Distance < S.Thickness) { ++SampledImpacts; EXPECT(Hit); if (Hit) EXPECT(Contact.Time <= Time + 1e-6); break; }
			}
		}
		printf("Surface CCD sampled impacts=%d conservative budget returns=%d\n", SampledImpacts, Conservative); EXPECT(SampledImpacts > 30);
		auto Reject = [](auto Function) { bool Rejected = false; try { Function(); } catch (const std::invalid_argument&) { Rejected = true; } EXPECT(Rejected); };
		auto A = Triangle(), B = Triangle(); SurfaceContact C;
		Reject([&] { SurfaceCollision::Closest(A.Start, 0, B.Start, 3); });
		A.Start[0].x = std::numeric_limits<float>::quiet_NaN(); Reject([&] { SurfaceCollision::Sweep(A, B, S, C); });
		A = Triangle(); S.MaxIterations = 0; Reject([&] { SurfaceCollision::Sweep(A, B, S, C); });
		S = SurfaceCollisionSettings(); S.AdvanceFraction = 1; EXPECT(!S.IsValid());
		S = SurfaceCollisionSettings(); S.Thickness = -1; EXPECT(!S.IsValid());
	}
}
void TestSurfaceCollision()
{
	DistanceFeatures(); ContinuousFeatures(); GrazingBudgetAndCovariance(); SampledNoMissAndValidation();
	RefinedSweeps();
	for (auto Acceleration : { SurfaceAcceleration::DynamicTree, SurfaceAcceleration::HierarchicalHash })
	{
		SurfaceIndexAgainstBruteForce(Acceleration); SurfaceIndexTopologyAndValidation(Acceleration); SurfaceIndexSeparatedPanels(Acceleration);
	}
	HierarchicalHashQueries(); SurfaceIndexBenchmark();
}

namespace Test_SurfaceContact_Cases
{
namespace
{
	using namespace Riemann;
	SurfaceContact Witness(const Vector3& Normal = Vector3::UnitZ())
	{
		SurfaceContact C; C.Normal = Normal; C.WeightsA = C.WeightsB = Vector3(1, 0, 0); C.Distance = 0; return C;
	}
	void WeightedResponse()
	{
		for (float MassScale : { 0.01f, 1.0f, 1000.0f })
		{
			Vector3 Point(0.2f, 0.3f, -0.2f), Face[] = { Vector3(0, 0, 0), Vector3(1, 0, 0), Vector3(0, 1, 0) };
			SurfaceContactParticle A[] = { { &Point, MassScale } }, B[] = { { &Face[0], 2 * MassScale }, { &Face[1], 3 * MassScale }, { &Face[2], 4 * MassScale } };
			auto C = Witness(); C.WeightsB = Vector3(0.5f, 0.2f, 0.3f); C.PointA = C.PointB = Vector3(0.2f, 0.3f, 0);
			SurfaceContactMaterial Material; Material.StaticFriction = Material.DynamicFriction = 0;
			SurfaceContactConstraint Row(A, 1, B, 3, C, 0.1, Material);
			const Vector3 MomentumBefore = Point + Face[0] / 2 + Face[1] / 3 + Face[2] / 4;
			EXPECT_NEAR(Row.GetEffectiveInverseMass(), 1.98 * MassScale, 2e-6 * MassScale);
			EXPECT(Row.Solve(0.01f)); EXPECT_NEAR(Row.GetNormalMultiplier(), 0.3 / (1.98 * MassScale), 1e-6 / MassScale);
			const Vector3 Weighted = Face[0] * 0.5f + Face[1] * 0.2f + Face[2] * 0.3f;
			EXPECT_NEAR(Point.z - Weighted.z, 0.1f, 1e-6f);
			EXPECT_NEAR((Point + Face[0] / 2 + Face[1] / 3 + Face[2] / 4 - MomentumBefore).Length(), 0, 1e-6f);
			const Vector3 Once = Point; for (int I = 0; I < 20; ++I) Row.Solve(0.01f);
			EXPECT_NEAR((Point - Once).Length(), 0, 1e-6f);
		}
	}
	void ComplianceAndFixedParticles()
	{
		for (float Dt : { 0.01f, 0.02f })
		{
			Vector3 Point(0, 0, -0.2f), Fixed = Vector3::Zero();
			SurfaceContactParticle A{ &Point, 2 }, B{ &Fixed, 0 }; auto C = Witness();
			SurfaceContactMaterial M; M.Compliance = 0.001; M.StaticFriction = M.DynamicFriction = 0;
			SurfaceContactConstraint Row(&A, 1, &B, 1, C, 0.1, M); Row.Solve(Dt);
			const double Alpha = M.Compliance / ((double)Dt * Dt), Expected = 0.3 / (2 + Alpha);
			EXPECT_NEAR(Row.GetNormalMultiplier(), Expected, 1e-8); EXPECT_NEAR(Point.z, -0.2 + 2 * Expected, 1e-7);
			EXPECT(Fixed == Vector3::Zero()); EXPECT_NEAR(Point.z - 0.1 + Alpha * Row.GetNormalMultiplier(), 0, 1e-7);
		}
		Vector3 Point(0, 0, -1), Fixed = Vector3::Zero(); SurfaceContactParticle A{ &Point, 0 }, B{ &Fixed, 0 };
		SurfaceContactConstraint Row(&A, 1, &B, 1, Witness(), 0); EXPECT(!Row.Solve(0.01f)); EXPECT(Point.z == -1);
	}
	void CoulombFriction()
	{
		for (const Vector3& Travel : { Vector3(0.05f, 0, 0), Vector3(0.6f, 0.8f, 0) })
		{
			Vector3 Point = Travel + Vector3(0, 0, -0.1f), Fixed = Vector3::Zero();
			SurfaceContactParticle A{ &Point, 1 }, B{ &Fixed, 0 };
			SurfaceContactMaterial M; M.StaticFriction = 1; M.DynamicFriction = 0.5;
			SurfaceContactConstraint Row(&A, 1, &B, 1, Witness(), 0, M);
			Row.Solve(0.01f); EXPECT_NEAR(Point.z, 0, 1e-7f);
			const bool Sliding = Travel.Length() > 0.1f; EXPECT(Row.IsSliding() == Sliding);
			const Vector3 Expected = Sliding ? Travel * 0.95f : Vector3::Zero();
			EXPECT_NEAR((Point - Expected).Length(), 0, 1e-6f);
			const Vector3 Once = Point;
			for (int I = 0; I < 30; ++I) Row.Solve(0.01f);
			EXPECT_NEAR((Point - Once).Length(), 0, 1e-6f);
			const auto Tangent = Row.GetTangentMultiplier();
			EXPECT_NEAR(std::hypot(std::hypot(Tangent.x, Tangent.y), Tangent.z), 0.05, 1e-7);
			// Release the old friction correction when another row removes normal load.
			Point.z = 1; Row.Solve(0.01f); EXPECT(Row.GetNormalMultiplier() == 0);
			EXPECT_NEAR((Vector3(Point.x, Point.y, 0) - Travel).Length(), 0, 1e-6f);
			EXPECT(Row.GetTangentMultiplier().SquareLength() == 0);
		}
		// Motion of a zero-mass collider contributes to relative tangential travel.
		Vector3 Point(0, 0, -0.1f), Platform(0.05f, 0, 0); SurfaceContactParticle A{ &Point, 1 }, B{ &Platform, 0 };
		SurfaceContactMaterial M; M.StaticFriction = M.DynamicFriction = 1;
		SurfaceContactConstraint Row(&A, 1, &B, 1, Witness(), 0, M); Row.Solve(0.01f);
		EXPECT_NEAR((Point - Platform).Length(), 0, 1e-6f); EXPECT(Platform == Vector3(0.05f, 0, 0));
	}
	void SharedDegreesOfFreedom()
	{
		Vector3 Shared = Vector3::Zero(), PointA(2, 0, 0), PointB = Vector3::Zero();
		SurfaceContactParticle A[] = { { &Shared, 1 }, { &PointA, 1 } }, B[] = { { &Shared, 1 }, { &PointB, 1 } };
		auto C = Witness(Vector3::UnitY()); C.WeightsA = Vector3(0.5f, 0.5f, 0); C.WeightsB = Vector3(0.25f, 0.75f, 0);
		C.PointA = Vector3(1, 0, 0); SurfaceContactConstraint Row(A, 2, B, 2, C, 0.1);
		EXPECT_NEAR(Row.GetEffectiveInverseMass(), 0.875, 1e-12); Row.Solve(0.01f);
		EXPECT_NEAR((Shared + PointA + PointB - Vector3(2, 0, 0)).Length(), 0, 1e-6f);
		EXPECT_NEAR(Shared.y * 0.25f + PointA.y * 0.5f - PointB.y * 0.75f, 0.1f, 1e-6f);
		SurfaceContactConstraint Cancelled(A, 2, A, 2, C, 0.1);
		// Different weights still describe a nonzero gradient on the same segment.
		EXPECT_NEAR(Cancelled.GetEffectiveInverseMass(), 0.125, 1e-12);
		C.WeightsB = C.WeightsA; SurfaceContactConstraint Identical(A, 2, A, 2, C, 0.1);
		EXPECT(Identical.GetEffectiveInverseMass() == 0); EXPECT(!Identical.Solve(0.01f));
	}
	void CovarianceAndTwoWayFriction()
	{
		Vector3 A(0.6f, 0.8f, -0.2f), B = Vector3::Zero();
		const Vector3 Before = A / 2 + B / 3;
		SurfaceContactParticle PA{ &A, 2 }, PB{ &B, 3 }; SurfaceContactMaterial Material;
		Material.StaticFriction = 0.8; Material.DynamicFriction = 0.5;
		SurfaceContactConstraint Row(&PA, 1, &PB, 1, Witness(), 0, Material); Row.Solve(0.02f);
		EXPECT(Row.IsSliding()); EXPECT_NEAR((A / 2 + B / 3 - Before).Length(), 0, 1e-6f);
		EXPECT_NEAR((A - B - Vector3(0.54f, 0.72f, 0)).Length(), 0, 1e-6f);
		Quaternion Rotation; Rotation.FromRotationAxis(Vector3(1, 2, 3), 0.7f);
		for (float Scale : { 0.001f, 1.0f, 1000.0f })
		{
			const Vector3 Translation = Vector3(13, -17, 29) * Scale;
			Vector3 AA = Rotation * Vector3(0.6f, 0.8f, -0.2f) * Scale + Translation, BB = Translation;
			SurfaceContactParticle PAA{ &AA, 2 }, PBB{ &BB, 3 }; auto C = Witness(Rotation * Vector3::UnitZ());
			C.PointA = C.PointB = Translation;
			SurfaceContactConstraint Rotated(&PAA, 1, &PBB, 1, C, 0, Material); Rotated.Solve(0.02f);
			EXPECT_NEAR((AA - (Rotation * A * Scale + Translation)).Length(), 0, 5e-6 * Scale);
			EXPECT_NEAR((BB - (Rotation * B * Scale + Translation)).Length(), 0, 5e-6 * Scale);
			EXPECT_NEAR(Rotated.GetNormalMultiplier(), Row.GetNormalMultiplier() * Scale, 2e-6 * Scale);
		}
	}
	void NonlinearRefresh()
	{
		Vector3 Position(0, 0, -0.2f), Fixed = Vector3::Zero(); SurfaceContactParticle A{ &Position, 1 }, B{ &Fixed, 0 };
		auto C = Witness(); SurfaceContactMaterial M; M.Compliance = 0.01; M.StaticFriction = M.DynamicFriction = 0;
		SurfaceContactConstraint Soft(&A, 1, &B, 1, C, 0, M); Soft.Solve(0.1f);
		const Vector3 Equilibrium = Position; const double Lambda = Soft.GetNormalMultiplier();
		for (int I = 0; I < 12; ++I) { Soft.Refresh(&A, 1, &B, 1, C, 0); Soft.Solve(0.1f); }
		EXPECT_NEAR((Position - Equilibrium).Length(), 0, 1e-6f); EXPECT_NEAR(Soft.GetNormalMultiplier(), Lambda, 1e-6);
		Position = Vector3(0.1f, 0, -0.2f); M.Compliance = 0; M.StaticFriction = 1; M.DynamicFriction = 0.5;
		SurfaceContactConstraint Row(&A, 1, &B, 1, C, 0, M); Row.Solve(0.1f);
		Quaternion Q; Q.FromRotationAxis(Vector3::UnitY(), 0.6f); C.Normal = Q * Vector3::UnitZ(); Position = Q * Position;
		const Vector3 ExpectedTangent = Q * Vector3(-0.1f, 0, 0), Before = Position;
		Row.Refresh(&A, 1, &B, 1, C, 0); Row.Solve(0.1f); const auto Tangent = Row.GetTangentMultiplier();
		EXPECT_NEAR(Tangent.x, ExpectedTangent.x, 1e-6); EXPECT_NEAR(Tangent.z, ExpectedTangent.z, 1e-6);
		EXPECT_NEAR((Position - Before).Length(), 0, 1e-6f); EXPECT_NEAR(Row.GetNormalMultiplier(), 0.2, 1e-6);
		C.Conservative = true; bool Rejected = false;
		try { Row.Refresh(&A, 1, &B, 1, C, 0); } catch (const std::invalid_argument&) { Rejected = true; }
		EXPECT(Rejected); EXPECT_NEAR(Row.GetNormalMultiplier(), 0.2, 1e-6);
	}
	void ConflictingFrictionAndSeparation()
	{
		Vector3 Position(-0.2f, 0, -0.2f), Fixed = Vector3::Zero();
		SurfaceContactParticle A{ &Position, 1 }, B{ &Fixed, 0 };
		auto Z = Witness(), X = Witness(Vector3::UnitX());
		Z.PointA = Vector3(-0.2f, 0, 0); X.PointA = Vector3(0, 0, -0.2f);
		SurfaceContactMaterial Material; Material.StaticFriction = Material.DynamicFriction = 10;
		SurfaceContactConstraint RowZ(&A, 1, &B, 1, Z, 0, Material), RowX(&A, 1, &B, 1, X, 0, Material);
		for (int I = 0; I < 8; ++I) { RowZ.Solve(0.1f); RowX.Solve(0.1f); }
		EXPECT(RowZ.GetNormalGap() < -0.1);
		const auto Tangent = RowZ.GetTangentMultiplier(); const double Normal = RowZ.GetNormalMultiplier();
		EXPECT(RowZ.ProjectSeparation()); RowX.ProjectSeparation();
		EXPECT(RowZ.GetNormalGap() >= -1e-6 && RowX.GetNormalGap() >= -1e-6);
		EXPECT(RowZ.GetTangentMultiplier() == Tangent); EXPECT(RowZ.GetNormalMultiplier() > Normal);
		Position += Vector3(1, 0, 1); const Vector3 Away = Position;
		EXPECT(!RowZ.ProjectSeparation() && !RowX.ProjectSeparation()); EXPECT(Position == Away);

		Vector3 P(0, 0, -0.2f), Q = Vector3::Zero();
		SurfaceContactParticle PA{ &P, 2 }, PB{ &Q, 3 }; const Vector3 Center = P / 2 + Q / 3;
		SurfaceContactConstraint Pair(&PA, 1, &PB, 1, Witness(), 0.1, Material); EXPECT(Pair.ProjectSeparation());
		EXPECT_NEAR(P.z - Q.z, 0.1, 1e-6); EXPECT_NEAR((P / 2 + Q / 3 - Center).Length(), 0, 1e-6);
		PA.InverseMass = PB.InverseMass = 0; P.z = -0.2f; Q.SetZero();
		SurfaceContactConstraint Immovable(&PA, 1, &PB, 1, Witness(), 0.1, Material);
		EXPECT(!Immovable.ProjectSeparation()); EXPECT(Immovable.GetNormalGap() < -0.2);
	}
	void ContinuousResponseAndValidation()
	{
		SurfaceMotion Point, Face; Point.Start[0] = Vector3(-1, 0, 1); Point.End[0] = Vector3(1, 0, -1);
		Face.Count = 3; Face.Start[0] = Vector3(-10, -10, 0); Face.Start[1] = Vector3(10, -10, 0); Face.Start[2] = Vector3(0, 10, 0);
		for (int I = 0; I < 3; ++I) Face.End[I] = Face.Start[I];
		SurfaceContact Contact; SurfaceCollisionSettings S; EXPECT(SurfaceCollision::Sweep(Point, Face, S, Contact));
		EXPECT(!Contact.Conservative); SurfaceContactParticle A{ &Point.End[0], 1 }, B[3];
		for (int I = 0; I < 3; ++I) B[I] = { &Face.End[I], 0 };
		SurfaceContactMaterial M; M.StaticFriction = 1.5; M.DynamicFriction = 0.5;
		SurfaceContactConstraint Row(&A, 1, B, 3, Contact, 0, M); Row.Solve(0.01f);
		EXPECT_NEAR(Point.End[0].z, 0, 1e-6f); EXPECT_NEAR(Point.End[0].x, Contact.PointA.x, 5e-5f); EXPECT(!Row.IsSliding());
		auto Reject = [](auto Function) { bool Failed = false; try { Function(); } catch (const std::exception&) { Failed = true; } EXPECT(Failed); };
		Contact.Conservative = true; Reject([&] { SurfaceContactConstraint Bad(&A, 1, B, 3, Contact, 0); });
		Contact.Conservative = false; Contact.Normal.SetZero(); Reject([&] { SurfaceContactConstraint Bad(&A, 1, B, 3, Contact, 0); });
		Contact = Witness(); Contact.WeightsA.x = 0.5f; Reject([&] { SurfaceContactConstraint Bad(&A, 1, B, 1, Contact, 0); });
		Contact = Witness(); Reject([&] { SurfaceContactConstraint Bad(nullptr, 1, B, 1, Contact, 0); });
		Reject([&] { Row.Solve(0); }); Reject([&] { Row.Solve(std::numeric_limits<float>::infinity()); });
		M.DynamicFriction = 2; EXPECT(!M.IsValid()); M = SurfaceContactMaterial(); M.Compliance = -1; EXPECT(!M.IsValid());
		SurfaceContactParticle Conflict = A; Conflict.InverseMass = 2;
		Reject([&] { SurfaceContactConstraint Bad(&A, 1, &Conflict, 1, Contact, 0); });
		Point.End[0].x = std::numeric_limits<float>::quiet_NaN(); Reject([&] { Row.Solve(0.01f); });
	}
}
void TestSurfaceContact()
{
	WeightedResponse(); ComplianceAndFixedParticles(); CoulombFriction(); SharedDegreesOfFreedom(); CovarianceAndTwoWayFriction(); NonlinearRefresh(); ConflictingFrictionAndSeparation(); ContinuousResponseAndValidation();
}
}

void TestSurfaceContact() { Test_SurfaceContact_Cases::TestSurfaceContact(); }
