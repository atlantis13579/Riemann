#include "TetrahedralAsset.h"
#include "../Collision/DynamicAABBTree.h"
#include "../Collision/SurfaceCollision.h"
#include <algorithm>
#include <climits>
#include <cmath>
#include <limits>
#include <map>
#include <set>
#include <stdexcept>

namespace Riemann
{
	/* Preparation consumes tetrahedral connectivity from an importer or procedural
	   author. It normalizes signed orientation, validates element quality/materials,
	   derives the external boundary by canceling opposite interior faces, and lumps
	   density-weighted mass onto vertices. Surface edge incidence and vertex links
	   must form a closed manifold. SAT rejects positive-volume element overlaps;
	   exact face/edge/vertex touching is handled separately by topology validation.
	   No welding or silent connectivity repair changes material/attachment indices.

	   The prepared asset is immutable shared data. Rest geometry, materials, mass,
	   collision boundary, BVHs and render bindings can serve independent instances.
	   Instance particles, forces, plastic history and nonlinear multipliers remain
	   owned by DeformableBody. Imported surface-to-volume tetrahedralization is a
	   separate producer of this input, rather than a hidden approximation here.
	   TODO: integrate a constrained tetrahedral mesher for arbitrary surface input.

	   Rendering uses four affine barycentric coordinates in a containing element.
	   Optional extrapolation selects the closest boundary triangle's owning element
	   and evaluates the ORIGINAL render point in that element, not its projection.
	   Thus a rotated/scaled/affinely deformed volume reproduces the render surface
	   without a world-space rest offset. Distance and weight limits bound unstable
	   extrapolation near thin elements. Vertex normals come from the deformed render
	   triangles. Simulation collision always uses the volume boundary instead.

	   Native AABB trees select candidate elements/faces. Tests and barycentric math
	   use double precision with relative coordinates, and deterministic feature
	   ordering resolves ties independently of tree balancing. Preparation either
	   publishes a complete asset or throws before any live body is changed. */
	namespace
	{
		using DVector = DeformationGradient::DVector;
		constexpr int FaceVertices[4][3] = { { 1, 2, 3 }, { 0, 3, 2 }, { 0, 1, 3 }, { 0, 2, 1 } };
		constexpr int EdgeVertices[6][2] = { { 0, 1 }, { 0, 2 }, { 0, 3 }, { 1, 2 }, { 1, 3 }, { 2, 3 } };
		DVector Double(const Vector3& P) { return DVector(P.x, P.y, P.z); }
		double Length(const DVector& P) { return std::hypot(std::hypot(P.x, P.y), P.z); }
		bool Finite(const Vector3& P) { return std::isfinite(P.x) && std::isfinite(P.y) && std::isfinite(P.z); }
		bool Positive(double X) { return std::isfinite(X) && X > 0; }
		std::array<int, 3> Cycle(const Index3& F)
		{
			int Start = 0; for (int I = 1; I < 3; ++I) if (F[I] < F[Start]) Start = I;
			return { F[Start], F[(Start + 1) % 3], F[(Start + 2) % 3] };
		}
		bool Overlap(const Vector3 A[4], const Vector3 B[4], double Tolerance)
		{
			DVector PA[4], PB[4]; for (int I = 0; I < 4; ++I) { PA[I] = Double(A[I]) - Double(A[0]); PB[I] = Double(B[I]) - Double(A[0]); }
			auto Separated = [&](DVector Axis)
			{
				const double Size = Length(Axis); if (!(Size > 0)) return false; Axis = Axis / Size;
				double MinA = PA[0].Dot(Axis), MaxA = MinA, MinB = PB[0].Dot(Axis), MaxB = MinB;
				for (int I = 1; I < 4; ++I)
				{ const double X = PA[I].Dot(Axis), Y = PB[I].Dot(Axis); MinA = std::min(MinA, X); MaxA = std::max(MaxA, X); MinB = std::min(MinB, Y); MaxB = std::max(MaxB, Y); }
				return std::min(MaxA, MaxB) - std::max(MinA, MinB) <= Tolerance;
			};
			for (const auto& F : FaceVertices)
				if (Separated((PA[F[1]] - PA[F[0]]).Cross(PA[F[2]] - PA[F[0]])) || Separated((PB[F[1]] - PB[F[0]]).Cross(PB[F[2]] - PB[F[0]]))) return false;
			for (const auto& EA : EdgeVertices) for (const auto& EB : EdgeVertices)
				if (Separated((PA[EA[1]] - PA[EA[0]]).Cross(PB[EB[1]] - PB[EB[0]]))) return false;
			return true;
		}
		void ValidateSurface(const std::vector<Vector3>& Positions, const std::vector<Index3>& Faces, double Tolerance)
		{
			for (const auto& P : Positions) if (!Finite(P)) throw std::invalid_argument("Non-finite asset render position");
			std::set<std::array<int, 3>> Seen;
			for (const auto& F : Faces)
			{
				std::array<int, 3> Key = { F.a, F.b, F.c }; std::sort(Key.begin(), Key.end());
				if (Key[0] < 0 || (size_t)Key[2] >= Positions.size() || Key[0] == Key[1] || Key[1] == Key[2] || !Seen.insert(Key).second)
					throw std::invalid_argument("Invalid asset render triangle");
				const auto A = Double(Positions[F.b]) - Double(Positions[F.a]), B = Double(Positions[F.c]) - Double(Positions[F.a]);
				const double Scale = std::max(A.SquareLength(), std::max(B.SquareLength(), (A - B).SquareLength()));
				if (Length(A.Cross(B)) <= Tolerance * Scale) throw std::invalid_argument("Degenerate asset render triangle");
			}
		}
	}
	bool TetrahedralAssetSettings::IsValid() const
	{
		return ElementSettings.IsValid() && Positive(MinimumQuality) && MinimumQuality <= 1 && Positive(MinimumEdgeLength)
			&& Positive(RelativeOverlapTolerance) && RelativeOverlapTolerance < 1 && Positive(BarycentricTolerance) && BarycentricTolerance < 1
			&& Positive(RelativeRenderAreaTolerance) && RelativeRenderAreaTolerance < 1
			&& Positive(MaximumExtrapolationRatio) && Positive(MaximumExtrapolationWeight) && MaximumExtrapolationWeight >= 1;
	}
	struct TetrahedralAsset::Storage
	{
		TetrahedralAssetSettings Settings;
		TetrahedralAssetStats Stats;
		std::vector<Vector3> Positions, RenderPositions;
		std::vector<TetrahedralAssetElement> Elements;
		std::vector<TetrahedralMaterial> Materials;
		std::vector<TetrahedralRestShape> Shapes;
		std::vector<std::array<int, 2>> Edges;
		std::vector<Index3> Boundary, RenderTriangles;
		std::vector<float> InverseMasses;
		std::vector<bool> Pins;
		std::vector<TetrahedralRenderBinding> Bindings;
		struct Entry { int Index = 0; };
		std::vector<Entry> ElementEntries, BoundaryEntries;
		std::vector<int> BoundaryOwners;
		DynamicAABBTree ElementTree, BoundaryTree;
		double MaximumEdge = 0;
		Storage() : ElementTree(0), BoundaryTree(0) {}
		std::array<double, 4> Weights(int Element, const Vector3& P) const
		{
			const auto& E = Elements[Element]; const auto D = Double(P) - Double(Positions[E.Vertices[0]]); std::array<double, 4> W = {};
			W[0] = 1; for (int I = 1; I < 4; ++I) { W[I] = Shapes[Element].Gradients[I].Dot(D); W[0] -= W[I]; } return W;
		}
		TetrahedralRenderBinding Bind(const Vector3& P, bool& Extrapolated) const
		{
			if (!Finite(P)) throw std::invalid_argument("Non-finite render binding position");
			const float Margin = (float)(Settings.BarycentricTolerance * MaximumEdge); const Vector3 Extent(Margin);
			if (!Finite(P - Extent) || !Finite(P + Extent)) throw std::overflow_error("Asset binding bounds overflow");
			std::vector<void*> Hits; ElementTree.Query(Box3(P - Extent, P + Extent), &Hits);
			TetrahedralRenderBinding Best; double BestInterior = -std::numeric_limits<double>::infinity();
			for (const auto* Hit : Hits)
			{
				const int I = static_cast<const Entry*>(Hit)->Index; const auto W = Weights(I, P); const double Interior = *std::min_element(W.begin(), W.end());
				if (Interior < -Settings.BarycentricTolerance) continue;
				if (Best.Element < 0 || Interior > BestInterior || (Interior == BestInterior && I < Best.Element)) { Best.Element = I; Best.Weights = W; BestInterior = Interior; }
			}
			Extrapolated = false; if (Best.Element >= 0) return Best;
			if (!Settings.AllowExtrapolation) throw std::invalid_argument("Render vertex is outside the tetrahedral volume");
			const float Radius = (float)(Settings.MaximumExtrapolationRatio * MaximumEdge); const Vector3 Search(Radius);
			if (!Finite(P - Search) || !Finite(P + Search)) throw std::overflow_error("Asset extrapolation bounds overflow");
			Hits.clear(); BoundaryTree.Query(Box3(P - Search, P + Search), &Hits);
			double Distance = std::numeric_limits<double>::infinity(); int BestFace = -1;
			for (const auto* Hit : Hits)
			{
				const int I = static_cast<const Entry*>(Hit)->Index; const auto& Face = Boundary[I]; const int Owner = BoundaryOwners[I];
				Vector3 Triangle[] = { Positions[Face.a], Positions[Face.b], Positions[Face.c] };
				const double D = SurfaceCollision::Closest(&P, 1, Triangle, 3).Distance;
				if (D > Settings.MaximumExtrapolationRatio * Shapes[Owner].CharacteristicLength) continue;
				const auto W = Weights(Owner, P); double Magnitude = 0; for (double X : W) Magnitude = std::max(Magnitude, std::abs(X));
				if (Magnitude > Settings.MaximumExtrapolationWeight) continue;
				if (D < Distance || (D == Distance && I < BestFace)) { Distance = D; BestFace = I; Best.Element = Owner; Best.Weights = W; }
			}
			if (Best.Element < 0) throw std::invalid_argument("Render vertex exceeds the extrapolation limits");
			Extrapolated = true; return Best;
		}
	};
	TetrahedralAsset TetrahedralAsset::Prepare(const TetrahedralAssetInput& Input, const TetrahedralAssetSettings& Settings)
	{
		if (!Settings.IsValid() || Input.Positions.size() < 4 || Input.Elements.empty() || Input.Materials.empty()
			|| Input.Positions.size() > INT_MAX || Input.Elements.size() > INT_MAX || Input.Materials.size() > INT_MAX || Input.RenderPositions.size() > INT_MAX || Input.RenderTriangles.size() > INT_MAX)
			throw std::invalid_argument("Invalid tetrahedral asset input");
		auto D = std::make_shared<Storage>(); D->Settings = Settings; D->Positions = Input.Positions; D->Elements = Input.Elements; D->Materials = Input.Materials;
		for (const auto& P : D->Positions) if (!Finite(P)) throw std::invalid_argument("Non-finite asset simulation position");
		for (const auto& M : D->Materials) if (!M.IsValid()) throw std::invalid_argument("Invalid asset material");
		D->Pins.assign(D->Positions.size(), false); for (int I : Input.PinnedVertices)
		{ if (I < 0 || (size_t)I >= D->Pins.size()) throw std::invalid_argument("Invalid asset pin index"); D->Pins[I] = true; }
		struct Face { Index3 Vertices; int Owner = -1, Count = 0; };
		std::map<std::array<int, 3>, Face> Faces; std::set<std::array<int, 4>> Elements; std::set<std::array<int, 2>> Edges;
		std::vector<double> Masses(D->Positions.size(), 0); std::vector<int> VertexOwners(D->Positions.size(), -1), Parents(D->Elements.size());
		auto Root = [&](int I) { while (Parents[I] != I) { Parents[I] = Parents[Parents[I]]; I = Parents[I]; } return I; };
		for (size_t I = 0; I < Parents.size(); ++I) Parents[I] = (int)I;
		D->Shapes.reserve(D->Elements.size()); D->ElementEntries.resize(D->Elements.size());
		for (size_t I = 0; I < D->Elements.size(); ++I)
		{
			auto& E = D->Elements[I]; auto Key = E.Vertices; std::sort(Key.begin(), Key.end());
			if (Key[0] < 0 || (size_t)Key[3] >= D->Positions.size() || std::adjacent_find(Key.begin(), Key.end()) != Key.end()
				|| !Elements.insert(Key).second || E.Material < 0 || (size_t)E.Material >= D->Materials.size()) throw std::invalid_argument("Invalid asset tetrahedron");
			Vector3 P[4]; for (int J = 0; J < 4; ++J) P[J] = D->Positions[E.Vertices[J]];
			if ((Double(P[1]) - Double(P[0])).Dot((Double(P[2]) - Double(P[0])).Cross(Double(P[3]) - Double(P[0]))) < 0)
			{ std::swap(E.Vertices[1], E.Vertices[2]); std::swap(P[1], P[2]); ++D->Stats.ReorientedElements; }
			const auto Shape = TetrahedralRestShape::Create(P, Settings.ElementSettings); double Longest = 0;
			for (const auto& Pair : EdgeVertices)
			{
				const double EdgeLength = Length(Double(P[Pair[0]]) - Double(P[Pair[1]]));
				if (EdgeLength <= Settings.MinimumEdgeLength) throw std::invalid_argument("Asset edge is below simulation precision"); Longest = std::max(Longest, EdgeLength);
				std::array<int, 2> Edge = { E.Vertices[Pair[0]], E.Vertices[Pair[1]] }; std::sort(Edge.begin(), Edge.end()); Edges.insert(Edge);
			}
			const double Quality = 6 * std::sqrt(2.0) * Shape.Volume / (Longest * Longest * Longest);
			if (Quality < Settings.MinimumQuality || !std::isfinite((float)Shape.Volume) || (float)Shape.Volume <= 0) throw std::invalid_argument("Asset tetrahedron quality or volume is invalid");
			D->Stats.MinimumQuality = std::min(D->Stats.MinimumQuality, Quality); D->Stats.TotalVolume += Shape.Volume; D->MaximumEdge = std::max(D->MaximumEdge, Longest);
			const double Mass = Shape.Volume * D->Materials[E.Material].Density / 4;
			if (!Positive(Mass)) throw std::overflow_error("Asset element mass overflow");
			for (int J : E.Vertices) { Masses[J] += Mass; if (VertexOwners[J] < 0) VertexOwners[J] = (int)I; }
			D->Shapes.push_back(Shape); D->ElementEntries[I].Index = (int)I; D->ElementTree.Add(Box3(P, 4), &D->ElementEntries[I]);
			for (const auto& F : FaceVertices)
			{
				const Index3 Triangle(E.Vertices[F[0]], E.Vertices[F[1]], E.Vertices[F[2]]); std::array<int, 3> FaceKey = { Triangle.a, Triangle.b, Triangle.c }; std::sort(FaceKey.begin(), FaceKey.end());
				auto& Record = Faces[FaceKey];
				if (!Record.Count) { Record.Vertices = Triangle; Record.Owner = (int)I; }
				else
				{
					if (Record.Count > 1 || Cycle(Record.Vertices) == Cycle(Triangle)) throw std::invalid_argument("Non-manifold or overlapping tetrahedral face");
					Parents[Root((int)I)] = Root(Record.Owner);
				}
				++Record.Count;
			}
		}
		for (size_t I = 0; I < D->Elements.size(); ++I)
		{
			Vector3 A[4]; for (int K = 0; K < 4; ++K) A[K] = D->Positions[D->Elements[I].Vertices[K]];
			std::vector<void*> Hits; D->ElementTree.Query(Box3(A, 4), &Hits);
			for (const auto* H : Hits)
			{
				const int J = static_cast<const Storage::Entry*>(H)->Index; if (J <= (int)I) continue;
				Vector3 B[4]; for (int K = 0; K < 4; ++K) B[K] = D->Positions[D->Elements[J].Vertices[K]];
				const double Tolerance = Settings.RelativeOverlapTolerance * std::min(D->Shapes[I].CharacteristicLength, D->Shapes[J].CharacteristicLength);
				if (Overlap(A, B, Tolerance)) throw std::invalid_argument("Asset tetrahedra have overlapping interiors");
			}
		}
		std::set<int> Components; for (size_t I = 0; I < Parents.size(); ++I) Components.insert(Root((int)I)); D->Stats.ConnectedComponents = Components.size();
		D->Edges.assign(Edges.begin(), Edges.end());
		for (const auto& Pair : Faces) if (Pair.second.Count == 1) { D->Boundary.push_back(Pair.second.Vertices); D->BoundaryOwners.push_back(Pair.second.Owner); }
		if (D->Boundary.size() > INT_MAX || D->Edges.size() > INT_MAX) throw std::overflow_error("Prepared asset topology exceeds index range");
		if (D->Boundary.empty()) throw std::invalid_argument("Asset has no external boundary");
		std::map<std::array<int, 2>, std::pair<int, int>> SurfaceEdges; std::vector<std::vector<std::array<int, 2>>> Links(D->Positions.size());
		for (const auto& F : D->Boundary) for (int I = 0; I < 3; ++I)
		{
			const int A = F[I], B = F[(I + 1) % 3]; std::array<int, 2> E = { std::min(A, B), std::max(A, B) };
			auto& Edge = SurfaceEdges[E]; ++Edge.first; Edge.second += A < B ? 1 : -1;
			Links[A].push_back({ B, F[(I + 2) % 3] });
		}
		for (const auto& E : SurfaceEdges) if (E.second.first != 2 || E.second.second != 0) throw std::invalid_argument("Asset boundary has a non-manifold edge");
		for (const auto& Link : Links) if (!Link.empty())
		{
			std::map<int, std::vector<int>> Graph; for (const auto& E : Link) { Graph[E[0]].push_back(E[1]); Graph[E[1]].push_back(E[0]); }
			std::set<int> Seen; std::vector<int> Stack = { Graph.begin()->first };
			while (!Stack.empty()) { const int I = Stack.back(); Stack.pop_back(); if (!Seen.insert(I).second) continue; for (int J : Graph[I]) Stack.push_back(J); }
			if (Seen.size() != Graph.size()) throw std::invalid_argument("Asset boundary has a disconnected vertex link");
			for (const auto& Node : Graph) if (Node.second.size() != 2) throw std::invalid_argument("Asset boundary vertex link is not a cycle");
		}
		D->InverseMasses.resize(Masses.size());
		for (size_t I = 0; I < Masses.size(); ++I)
		{
			if (VertexOwners[I] < 0) throw std::invalid_argument("Asset contains an unused simulation vertex");
			D->InverseMasses[I] = (float)(1 / Masses[I]); if (!Positive(Masses[I]) || !Positive(D->InverseMasses[I])) throw std::overflow_error("Asset nodal mass is not representable");
			D->Stats.TotalMass += Masses[I];
		}
		if (!Positive(D->Stats.TotalMass) || !Positive(D->Stats.TotalVolume)) throw std::overflow_error("Asset totals overflow");
		D->BoundaryEntries.resize(D->Boundary.size());
		for (size_t I = 0; I < D->Boundary.size(); ++I)
		{
			const auto& F = D->Boundary[I]; Vector3 P[] = { D->Positions[F.a], D->Positions[F.b], D->Positions[F.c] };
			D->BoundaryEntries[I].Index = (int)I; D->BoundaryTree.Add(Box3(P, 3), &D->BoundaryEntries[I]);
		}
		if (Input.RenderPositions.empty() && Input.RenderTriangles.empty())
		{
			D->RenderPositions = D->Positions; D->RenderTriangles = D->Boundary;
			for (size_t I = 0; I < D->Positions.size(); ++I)
			{
				TetrahedralRenderBinding B; B.Element = VertexOwners[I]; const auto& E = D->Elements[B.Element];
				for (int J = 0; J < 4; ++J) if (E.Vertices[J] == (int)I) B.Weights[J] = 1; D->Bindings.push_back(B);
			}
		}
		else
		{
			if (Input.RenderPositions.empty() || Input.RenderTriangles.empty()) throw std::invalid_argument("Incomplete asset render mesh");
			ValidateSurface(Input.RenderPositions, Input.RenderTriangles, Settings.RelativeRenderAreaTolerance);
			D->RenderPositions = Input.RenderPositions; D->RenderTriangles = Input.RenderTriangles;
			for (const auto& P : D->RenderPositions) { bool Outside; D->Bindings.push_back(D->Bind(P, Outside)); if (Outside) ++D->Stats.ExtrapolatedVertices; }
		}
		TetrahedralAsset Asset; Asset.m_Data = std::move(D); return Asset;
	}
	const TetrahedralAsset::Storage& TetrahedralAsset::Data() const { if (!m_Data) throw std::logic_error("Unprepared tetrahedral asset"); return *m_Data; }
	const std::vector<Vector3>& TetrahedralAsset::GetPositions() const { return Data().Positions; }
	const std::vector<TetrahedralAssetElement>& TetrahedralAsset::GetElements() const { return Data().Elements; }
	const std::vector<TetrahedralMaterial>& TetrahedralAsset::GetMaterials() const { return Data().Materials; }
	const std::vector<TetrahedralRestShape>& TetrahedralAsset::GetRestShapes() const { return Data().Shapes; }
	const std::vector<std::array<int, 2>>& TetrahedralAsset::GetEdges() const { return Data().Edges; }
	const std::vector<Index3>& TetrahedralAsset::GetBoundary() const { return Data().Boundary; }
	const std::vector<float>& TetrahedralAsset::GetInverseMasses() const { return Data().InverseMasses; }
	const std::vector<bool>& TetrahedralAsset::GetPins() const { return Data().Pins; }
	const std::vector<Index3>& TetrahedralAsset::GetRenderTriangles() const { return Data().RenderTriangles; }
	const std::vector<Vector3>& TetrahedralAsset::GetRenderPositions() const { return Data().RenderPositions; }
	const std::vector<TetrahedralRenderBinding>& TetrahedralAsset::GetRenderBindings() const { return Data().Bindings; }
	const TetrahedralAssetStats& TetrahedralAsset::GetStats() const { return Data().Stats; }
	const TetrahedralAssetSettings& TetrahedralAsset::GetSettings() const { return Data().Settings; }
	std::vector<TetrahedralRenderBinding> TetrahedralAsset::BindPoints(const std::vector<Vector3>& Points) const
	{
		std::vector<TetrahedralRenderBinding> Bindings; Bindings.reserve(Points.size());
		for (const auto& P : Points) { bool Outside; Bindings.push_back(Data().Bind(P, Outside)); } return Bindings;
	}
}
