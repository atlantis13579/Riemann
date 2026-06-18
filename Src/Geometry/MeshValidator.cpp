#include "MeshValidator.h"

#include <algorithm>
#include <cmath>
#include <cfloat>
#include <cstdint>
#include <limits>
#include <unordered_map>

#include "DynamicMesh.h"
#include "../CollisionPrimitive/StaticMesh.h"
#include "../CollisionPrimitive/Triangle3.h"

namespace Riemann
{
	namespace
	{
		struct EdgeUse
		{
			int Triangle = -1;
			uint32_t From = 0;
			uint32_t To = 0;
			int Direction = 0;
		};

		struct EdgeKey
		{
			uint32_t A = 0;
			uint32_t B = 0;

			EdgeKey() {}
			EdgeKey(uint32_t InA, uint32_t InB)
			{
				A = std::min(InA, InB);
				B = std::max(InA, InB);
			}

			bool operator==(const EdgeKey& Other) const
			{
				return A == Other.A && B == Other.B;
			}
		};

		struct EdgeKeyHash
		{
			size_t operator()(const EdgeKey& Key) const
			{
				return (size_t(Key.A) * 73856093u) ^ (size_t(Key.B) * 19349663u);
			}
		};

		struct TriangleKey
		{
			uint32_t V[3]{ 0, 0, 0 };

			TriangleKey() {}
			TriangleKey(uint32_t A, uint32_t B, uint32_t C)
			{
				V[0] = A;
				V[1] = B;
				V[2] = C;
				std::sort(V, V + 3);
			}

			bool operator==(const TriangleKey& Other) const
			{
				return V[0] == Other.V[0] && V[1] == Other.V[1] && V[2] == Other.V[2];
			}
		};

		struct TriangleKeyHash
		{
			size_t operator()(const TriangleKey& Key) const
			{
				return (size_t(Key.V[0]) * 73856093u) ^ (size_t(Key.V[1]) * 19349663u) ^ (size_t(Key.V[2]) * 83492791u);
			}
		};

		struct CellKey
		{
			int64_t X = 0;
			int64_t Y = 0;
			int64_t Z = 0;

			bool operator==(const CellKey& Other) const
			{
				return X == Other.X && Y == Other.Y && Z == Other.Z;
			}
		};

		struct CellKeyHash
		{
			size_t operator()(const CellKey& Key) const
			{
				uint64_t X = (uint64_t)Key.X;
				uint64_t Y = (uint64_t)Key.Y;
				uint64_t Z = (uint64_t)Key.Z;
				return size_t((X * 73856093ull) ^ (Y * 19349663ull) ^ (Z * 83492791ull));
			}
		};

		struct CleanTriangle
		{
			int OriginalId = -1;
			uint32_t V[3]{ 0, 0, 0 };
			Vector3 P[3];
			Box3 Bounds = Box3::Empty();
		};

		class DisjointSet
		{
		public:
			void Reset(size_t Count)
			{
				Parent.resize(Count);
				Rank.assign(Count, 0);
				for (size_t i = 0; i < Count; ++i)
				{
					Parent[i] = (int)i;
				}
			}

			int Find(int X)
			{
				int P = Parent[X];
				if (P != X)
				{
					Parent[X] = Find(P);
				}
				return Parent[X];
			}

			void Union(int A, int B)
			{
				A = Find(A);
				B = Find(B);
				if (A == B)
				{
					return;
				}
				if (Rank[A] < Rank[B])
				{
					std::swap(A, B);
				}
				Parent[B] = A;
				if (Rank[A] == Rank[B])
				{
					Rank[A]++;
				}
			}

		private:
			std::vector<int> Parent;
			std::vector<uint8_t> Rank;
		};

		static bool IsFinite(const Vector3& V)
		{
			return std::isfinite(V.x) && std::isfinite(V.y) && std::isfinite(V.z);
		}

		static double DotCross(const Vector3& A, const Vector3& B, const Vector3& C)
		{
			const double X = double(B.y) * double(C.z) - double(B.z) * double(C.y);
			const double Y = double(B.z) * double(C.x) - double(B.x) * double(C.z);
			const double Z = double(B.x) * double(C.y) - double(B.y) * double(C.x);
			return double(A.x) * X + double(A.y) * Y + double(A.z) * Z;
		}

		static double TriangleArea(const Vector3& A, const Vector3& B, const Vector3& C)
		{
			const double ABx = double(B.x) - double(A.x);
			const double ABy = double(B.y) - double(A.y);
			const double ABz = double(B.z) - double(A.z);
			const double ACx = double(C.x) - double(A.x);
			const double ACy = double(C.y) - double(A.y);
			const double ACz = double(C.z) - double(A.z);
			const double CX = ABy * ACz - ABz * ACy;
			const double CY = ABz * ACx - ABx * ACz;
			const double CZ = ABx * ACy - ABy * ACx;
			return 0.5 * std::sqrt(CX * CX + CY * CY + CZ * CZ);
		}

		static double SqrDistance(const Vector3& A, const Vector3& B)
		{
			const double X = double(A.x) - double(B.x);
			const double Y = double(A.y) - double(B.y);
			const double Z = double(A.z) - double(B.z);
			return X * X + Y * Y + Z * Z;
		}

		static void AddSample(std::vector<int>& Samples, int Value, int MaxSamples)
		{
			if ((int)Samples.size() < MaxSamples)
			{
				Samples.push_back(Value);
			}
		}

		template <class T>
		static void AddSample(std::vector<T>& Samples, const T& Value, int MaxSamples)
		{
			if ((int)Samples.size() < MaxSamples)
			{
				Samples.push_back(Value);
			}
		}

		static CellKey MakeCell(const Vector3& P, float Tolerance)
		{
			const double Inv = 1.0 / double(Tolerance);
			return CellKey{
				(int64_t)std::floor(double(P.x) * Inv),
				(int64_t)std::floor(double(P.y) * Inv),
				(int64_t)std::floor(double(P.z) * Inv)
			};
		}

		static void BuildVertexRemap(const Vector3* Vertices, uint32_t NumVertices, const std::vector<uint8_t>& ValidVertex,
			float WeldTolerance, std::vector<uint32_t>& Remap, MeshValidationReport& Report, int MaxSamples)
		{
			Remap.resize(NumVertices);
			for (uint32_t i = 0; i < NumVertices; ++i)
			{
				Remap[i] = i;
			}

			if (WeldTolerance <= 0.0f)
			{
				Report.WeldedVertexCount = NumVertices;
				return;
			}

			const double TolSqr = double(WeldTolerance) * double(WeldTolerance);
			std::unordered_map<CellKey, std::vector<uint32_t>, CellKeyHash> Grid;
			uint32_t UniqueCount = 0;

			for (uint32_t i = 0; i < NumVertices; ++i)
			{
				if (!ValidVertex[i])
				{
					continue;
				}

				const CellKey Cell = MakeCell(Vertices[i], WeldTolerance);
				uint32_t Found = i;
				bool bFound = false;

				for (int DX = -1; DX <= 1 && !bFound; ++DX)
				{
					for (int DY = -1; DY <= 1 && !bFound; ++DY)
					{
						for (int DZ = -1; DZ <= 1 && !bFound; ++DZ)
						{
							CellKey Neighbor{ Cell.X + DX, Cell.Y + DY, Cell.Z + DZ };
							auto It = Grid.find(Neighbor);
							if (It == Grid.end())
							{
								continue;
							}
							for (uint32_t Candidate : It->second)
							{
								if (SqrDistance(Vertices[i], Vertices[Candidate]) <= TolSqr)
								{
									Found = Candidate;
									bFound = true;
									break;
								}
							}
						}
					}
				}

				if (bFound)
				{
					Remap[i] = Remap[Found];
					Report.DuplicateVertexCount++;
					AddSample(Report.DuplicateVertexSamples, (int)i, MaxSamples);
				}
				else
				{
					Remap[i] = i;
					Grid[Cell].push_back(i);
					UniqueCount++;
				}
			}

			Report.WeldedVertexCount = UniqueCount + Report.InvalidVertexCount;
		}

		static bool SharesVertex(const CleanTriangle& A, const CleanTriangle& B)
		{
			for (int i = 0; i < 3; ++i)
			{
				for (int j = 0; j < 3; ++j)
				{
					if (A.V[i] == B.V[j])
					{
						return true;
					}
				}
			}
			return false;
		}

		static bool BoundsOverlap(const Box3& A, const Box3& B, float Epsilon)
		{
			return A.Min.x <= B.Max.x + Epsilon && A.Max.x + Epsilon >= B.Min.x &&
				A.Min.y <= B.Max.y + Epsilon && A.Max.y + Epsilon >= B.Min.y &&
				A.Min.z <= B.Max.z + Epsilon && A.Max.z + Epsilon >= B.Min.z;
		}

		static void CheckSelfIntersections(const std::vector<CleanTriangle>& Triangles,
			const MeshValidationOptions& Options, MeshValidationReport& Report)
		{
			std::vector<int> Order(Triangles.size());
			for (size_t i = 0; i < Triangles.size(); ++i)
			{
				Order[i] = (int)i;
			}

			std::sort(Order.begin(), Order.end(), [&](int A, int B)
			{
				return Triangles[A].Bounds.Min.x < Triangles[B].Bounds.Min.x;
			});

			uint64_t TestCount = 0;
			const float BoundsEps = std::max(Options.WeldTolerance, 1e-6f);
			for (size_t I = 0; I < Order.size(); ++I)
			{
				const CleanTriangle& A = Triangles[Order[I]];
				for (size_t J = I + 1; J < Order.size(); ++J)
				{
					const CleanTriangle& B = Triangles[Order[J]];
					if (B.Bounds.Min.x > A.Bounds.Max.x + BoundsEps)
					{
						break;
					}
					if (!BoundsOverlap(A.Bounds, B.Bounds, BoundsEps))
					{
						continue;
					}
					if (Options.IgnoreAdjacentTriangleIntersections && SharesVertex(A, B))
					{
						continue;
					}

					if (++TestCount > Options.MaxSelfIntersectionTests)
					{
						Report.SelfIntersectionTestLimitReached = true;
						return;
					}

					Triangle3 TA(A.P[0], A.P[1], A.P[2]);
					Triangle3::Triangle3IntersectionResult Result;
					if (TA.IntersectTriangle(Triangle3(B.P[0], B.P[1], B.P[2]), Result))
					{
						Report.SelfIntersectionCount++;
						AddSample(Report.SelfIntersectionSamples,
							MeshValidationTriangleIssue{ A.OriginalId, B.OriginalId }, Options.MaxIssueSamples);
					}
				}
			}
		}
	}

	bool MeshValidationReport::HasValidData() const
	{
		return InvalidVertexCount == 0 && InvalidIndexCount == 0 && TriangleWithInvalidVertexCount == 0 && DegenerateTriangleCount == 0;
	}

	bool MeshValidationReport::IsClosed() const
	{
		return BoundaryEdgeCount == 0 && NonManifoldEdgeCount == 0;
	}

	bool MeshValidationReport::IsOrientable() const
	{
		return SameDirectionEdgeCount == 0;
	}

	bool MeshValidationReport::IsTwoManifold() const
	{
		return NonManifoldEdgeCount == 0 && NonManifoldVertexCount == 0;
	}

	bool MeshValidationReport::IsSolidCandidate() const
	{
		return HasValidData() && IsClosed() && IsOrientable() && IsTwoManifold() &&
			SelfIntersectionCount == 0 && !SelfIntersectionTestLimitReached && AbsoluteVolume > std::numeric_limits<double>::epsilon();
	}

	MeshValidationReport MeshValidator::Validate(const Vector3* Vertices, uint32_t NumVertices,
		const uint32_t* Indices, uint32_t NumTriangles, const MeshValidationOptions& Options)
	{
		MeshValidationReport Report;
		Report.VertexCount = NumVertices;
		Report.TriangleCount = NumTriangles;

		if (Vertices == nullptr || (Indices == nullptr && NumTriangles > 0))
		{
			Report.InvalidIndexCount = (int)NumTriangles;
			return Report;
		}

		std::vector<uint8_t> ValidVertex(NumVertices, 1);
		for (uint32_t I = 0; I < NumVertices; ++I)
		{
			if (!IsFinite(Vertices[I]))
			{
				ValidVertex[I] = 0;
				Report.InvalidVertexCount++;
				AddSample(Report.InvalidVertexSamples, (int)I, Options.MaxIssueSamples);
			}
			else
			{
				if (Report.SurfaceArea == 0.0 && Report.SignedVolume == 0.0 && I == 0)
				{
					Report.Bounds = Box3(Vertices[I], Vertices[I]);
				}
				else
				{
					Report.Bounds.Encapsulate(Vertices[I]);
				}
			}
		}

		std::vector<uint32_t> Remap;
		BuildVertexRemap(Vertices, NumVertices, ValidVertex, Options.WeldTolerance, Remap, Report, Options.MaxIssueSamples);

		std::vector<CleanTriangle> CleanTriangles;
		CleanTriangles.reserve(NumTriangles);

		std::unordered_map<TriangleKey, int, TriangleKeyHash> TriangleSet;
		std::unordered_map<EdgeKey, std::vector<EdgeUse>, EdgeKeyHash> EdgeMap;
		std::unordered_map<uint32_t, std::vector<int>> VertexTriangles;

		for (uint32_t T = 0; T < NumTriangles; ++T)
		{
			const uint32_t I0 = Indices[3 * T + 0];
			const uint32_t I1 = Indices[3 * T + 1];
			const uint32_t I2 = Indices[3 * T + 2];

			if (I0 >= NumVertices || I1 >= NumVertices || I2 >= NumVertices)
			{
				Report.InvalidIndexCount++;
				AddSample(Report.InvalidIndexTriangleSamples, (int)T, Options.MaxIssueSamples);
				continue;
			}

			if (!ValidVertex[I0] || !ValidVertex[I1] || !ValidVertex[I2])
			{
				Report.TriangleWithInvalidVertexCount++;
				AddSample(Report.InvalidIndexTriangleSamples, (int)T, Options.MaxIssueSamples);
				continue;
			}

			const uint32_t V[3] = { Remap[I0], Remap[I1], Remap[I2] };
			if (V[0] == V[1] || V[1] == V[2] || V[2] == V[0])
			{
				Report.DegenerateTriangleCount++;
				AddSample(Report.DegenerateTriangleSamples, (int)T, Options.MaxIssueSamples);
				continue;
			}

			const Vector3 P[3] = { Vertices[I0], Vertices[I1], Vertices[I2] };
			const double Area = TriangleArea(P[0], P[1], P[2]);
			if (Area <= double(Options.AreaEpsilon))
			{
				Report.DegenerateTriangleCount++;
				AddSample(Report.DegenerateTriangleSamples, (int)T, Options.MaxIssueSamples);
				continue;
			}

			TriangleKey TriKey(V[0], V[1], V[2]);
			auto TriIt = TriangleSet.find(TriKey);
			if (TriIt != TriangleSet.end())
			{
				Report.DuplicateTriangleCount++;
				AddSample(Report.DuplicateTriangleSamples, (int)T, Options.MaxIssueSamples);
			}
			else
			{
				TriangleSet.emplace(TriKey, (int)T);
			}

			CleanTriangle Clean;
			Clean.OriginalId = (int)T;
			for (int K = 0; K < 3; ++K)
			{
				Clean.V[K] = V[K];
				Clean.P[K] = P[K];
			}
			Clean.Bounds = Box3(P[0], P[1], P[2]);

			const int CleanId = (int)CleanTriangles.size();
			CleanTriangles.push_back(Clean);

			Report.ValidTriangleCount++;
			Report.SurfaceArea += Area;
			Report.SignedVolume += DotCross(P[0], P[1], P[2]) / 6.0;

			for (int K = 0; K < 3; ++K)
			{
				VertexTriangles[V[K]].push_back(CleanId);
			}

			for (int K = 0; K < 3; ++K)
			{
				const uint32_t A = V[K];
				const uint32_t B = V[(K + 1) % 3];
				EdgeKey Key(A, B);
				EdgeUse Use;
				Use.Triangle = CleanId;
				Use.From = A;
				Use.To = B;
				Use.Direction = (A == Key.A && B == Key.B) ? 1 : -1;
				EdgeMap[Key].push_back(Use);
			}
		}

		Report.AbsoluteVolume = std::abs(Report.SignedVolume);

		DisjointSet ComponentSet;
		ComponentSet.Reset(CleanTriangles.size());

		for (const auto& Pair : EdgeMap)
		{
			const EdgeKey& Key = Pair.first;
			const std::vector<EdgeUse>& Uses = Pair.second;
			if (Uses.size() == 1)
			{
				Report.BoundaryEdgeCount++;
				AddSample(Report.BoundaryEdgeSamples,
					MeshValidationEdgeIssue{ Key.A, Key.B, (int)Uses.size(), CleanTriangles[Uses[0].Triangle].OriginalId, -1 }, Options.MaxIssueSamples);
			}
			else if (Uses.size() > 2)
			{
				Report.NonManifoldEdgeCount++;
				AddSample(Report.NonManifoldEdgeSamples,
					MeshValidationEdgeIssue{ Key.A, Key.B, (int)Uses.size(), CleanTriangles[Uses[0].Triangle].OriginalId,
						CleanTriangles[Uses[1].Triangle].OriginalId }, Options.MaxIssueSamples);
			}
			else
			{
				if (Uses[0].Direction == Uses[1].Direction)
				{
					Report.SameDirectionEdgeCount++;
					AddSample(Report.SameDirectionEdgeSamples,
						MeshValidationEdgeIssue{ Key.A, Key.B, 2, CleanTriangles[Uses[0].Triangle].OriginalId,
							CleanTriangles[Uses[1].Triangle].OriginalId }, Options.MaxIssueSamples);
				}
			}

			for (size_t I = 1; I < Uses.size(); ++I)
			{
				ComponentSet.Union(Uses[0].Triangle, Uses[I].Triangle);
			}
		}

		std::unordered_map<int, int> ComponentIndex;
		std::vector<double> ComponentVolumes;
		for (size_t I = 0; I < CleanTriangles.size(); ++I)
		{
			const int Root = ComponentSet.Find((int)I);
			auto It = ComponentIndex.find(Root);
			if (It == ComponentIndex.end())
			{
				const int NewIndex = (int)ComponentVolumes.size();
				ComponentIndex.emplace(Root, NewIndex);
				ComponentVolumes.push_back(0.0);
				It = ComponentIndex.find(Root);
			}
			const CleanTriangle& Tri = CleanTriangles[I];
			ComponentVolumes[It->second] += DotCross(Tri.P[0], Tri.P[1], Tri.P[2]) / 6.0;
		}
		Report.ConnectedComponentCount = (int)ComponentVolumes.size();
		Report.ComponentSignedVolumes = ComponentVolumes;
		const double VolumeEps = 1e-12;
		for (double Volume : ComponentVolumes)
		{
			if (Volume > VolumeEps)
			{
				Report.PositiveVolumeComponentCount++;
			}
			else if (Volume < -VolumeEps)
			{
				Report.NegativeVolumeComponentCount++;
			}
			else
			{
				Report.NearZeroVolumeComponentCount++;
			}
		}

		for (const auto& Pair : VertexTriangles)
		{
			const uint32_t Vertex = Pair.first;
			const std::vector<int>& Incidents = Pair.second;
			if (Incidents.size() <= 1)
			{
				continue;
			}

			std::unordered_map<int, int> LocalIndex;
			for (size_t I = 0; I < Incidents.size(); ++I)
			{
				LocalIndex.emplace(Incidents[I], (int)I);
			}

			DisjointSet LocalSet;
			LocalSet.Reset(Incidents.size());
			for (const auto& EdgePair : EdgeMap)
			{
				if (EdgePair.first.A != Vertex && EdgePair.first.B != Vertex)
				{
					continue;
				}
				const std::vector<EdgeUse>& Uses = EdgePair.second;
				for (size_t I = 1; I < Uses.size(); ++I)
				{
					auto It0 = LocalIndex.find(Uses[0].Triangle);
					auto It1 = LocalIndex.find(Uses[I].Triangle);
					if (It0 != LocalIndex.end() && It1 != LocalIndex.end())
					{
						LocalSet.Union(It0->second, It1->second);
					}
				}
			}

			std::unordered_map<int, int> LocalComponents;
			for (size_t I = 0; I < Incidents.size(); ++I)
			{
				LocalComponents[LocalSet.Find((int)I)]++;
			}

			if (LocalComponents.size() > 1)
			{
				Report.NonManifoldVertexCount++;
				AddSample(Report.NonManifoldVertexSamples,
					MeshValidationVertexIssue{ Vertex, (int)LocalComponents.size() }, Options.MaxIssueSamples);
			}
		}

		if (Options.CheckSelfIntersections)
		{
			CheckSelfIntersections(CleanTriangles, Options, Report);
		}

		return Report;
	}

	MeshValidationReport MeshValidator::Validate(const std::vector<Vector3>& Vertices,
		const std::vector<Index3>& Triangles, const MeshValidationOptions& Options)
	{
		std::vector<uint32_t> Indices;
		Indices.reserve(Triangles.size() * 3);
		for (const Index3& Tri : Triangles)
		{
			Indices.push_back((uint32_t)Tri.a);
			Indices.push_back((uint32_t)Tri.b);
			Indices.push_back((uint32_t)Tri.c);
		}
		return Validate(Vertices.data(), (uint32_t)Vertices.size(), Indices.data(), (uint32_t)Triangles.size(), Options);
	}

	MeshValidationReport MeshValidator::Validate(const StaticMesh& Mesh, const MeshValidationOptions& Options)
	{
		std::vector<uint32_t> Indices;
		Indices.reserve((size_t)Mesh.GetTriangleCount() * 3);
		for (uint32_t T = 0; T < Mesh.GetTriangleCount(); ++T)
		{
			uint32_t I0, I1, I2;
			Mesh.GetVertIndices(T, I0, I1, I2);
			Indices.push_back(I0);
			Indices.push_back(I1);
			Indices.push_back(I2);
		}
		return Validate(Mesh.Vertices, Mesh.GetVertexCount(), Indices.data(), Mesh.GetTriangleCount(), Options);
	}

	MeshValidationReport MeshValidator::Validate(const DynamicMesh& Mesh, const MeshValidationOptions& Options)
	{
		std::vector<Vector3> Vertices;
		Vertices.reserve(Mesh.GetVertexCount());
		for (int I = 0; I < Mesh.GetVertexCount(); ++I)
		{
			Vertices.push_back(Mesh.GetVertex(I));
		}

		std::vector<Index3> Triangles;
		Triangles.reserve(Mesh.GetTriangleCount());
		for (int T = 0; T < Mesh.GetTriangleCount(); ++T)
		{
			if (Mesh.IsTriangle(T))
			{
				Triangles.push_back(Mesh.GetTriangle(T));
			}
		}
		return Validate(Vertices, Triangles, Options);
	}
}

