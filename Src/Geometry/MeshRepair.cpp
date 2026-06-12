#include "MeshRepair.h"

#include <algorithm>
#include <cmath>
#include <cfloat>
#include <cstdint>
#include <limits>
#include <queue>
#include <unordered_map>
#include <vector>

#include "DynamicMesh.h"
#include "../CollisionPrimitive/StaticMesh.h"

namespace Riemann
{
	namespace
	{
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

		struct EdgeUse
		{
			int Triangle = -1;
			uint32_t From = 0;
			uint32_t To = 0;
			int Direction = 0;
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
				return (size_t(Key.V[0]) * 73856093u) ^
					(size_t(Key.V[1]) * 19349663u) ^
					(size_t(Key.V[2]) * 83492791u);
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
				const uint64_t X = (uint64_t)Key.X;
				const uint64_t Y = (uint64_t)Key.Y;
				const uint64_t Z = (uint64_t)Key.Z;
				return size_t((X * 73856093ull) ^ (Y * 19349663ull) ^ (Z * 83492791ull));
			}
		};

		struct RepairTriangle
		{
			uint32_t V[3]{ 0, 0, 0 };
			int SourceTriangle = -1;
			bool Removed = false;
		};

		class DisjointSet
		{
		public:
			void Reset(size_t Count)
			{
				Parent.resize(Count);
				Rank.assign(Count, 0);
				for (size_t I = 0; I < Count; ++I)
				{
					Parent[I] = (int)I;
				}
			}

			int Find(int X)
			{
				const int P = Parent[X];
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

		static double SqrDistance(const Vector3& A, const Vector3& B)
		{
			const double X = double(A.x) - double(B.x);
			const double Y = double(A.y) - double(B.y);
			const double Z = double(A.z) - double(B.z);
			return X * X + Y * Y + Z * Z;
		}

		static double TriangleArea(const Vector3& A, const Vector3& B, const Vector3& C)
		{
			const double ABx = double(B.x) - double(A.x);
			const double ABy = double(B.y) - double(A.y);
			const double ABz = double(B.z) - double(A.z);
			const double ACx = double(C.x) - double(A.x);
			const double ACy = double(C.y) - double(A.y);
			const double ACz = double(C.z) - double(A.z);
			const double X = ABy * ACz - ABz * ACy;
			const double Y = ABz * ACx - ABx * ACz;
			const double Z = ABx * ACy - ABy * ACx;
			return 0.5 * std::sqrt(X * X + Y * Y + Z * Z);
		}

		static double DotCross(const Vector3& A, const Vector3& B, const Vector3& C)
		{
			const double X = double(B.y) * double(C.z) - double(B.z) * double(C.y);
			const double Y = double(B.z) * double(C.x) - double(B.x) * double(C.z);
			const double Z = double(B.x) * double(C.y) - double(B.y) * double(C.x);
			return double(A.x) * X + double(A.y) * Y + double(A.z) * Z;
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

		static uint32_t InvalidVertex()
		{
			return std::numeric_limits<uint32_t>::max();
		}

		static void BuildWeldedVertices(const Vector3* Vertices, uint32_t NumVertices, float WeldTolerance,
			std::vector<Vector3>& OutVertices, std::vector<uint32_t>& Remap)
		{
			OutVertices.clear();
			Remap.assign(NumVertices, InvalidVertex());

			if (Vertices == nullptr)
			{
				return;
			}

			if (WeldTolerance <= 0.0f)
			{
				OutVertices.reserve(NumVertices);
				for (uint32_t I = 0; I < NumVertices; ++I)
				{
					if (!IsFinite(Vertices[I]))
					{
						continue;
					}
					Remap[I] = (uint32_t)OutVertices.size();
					OutVertices.push_back(Vertices[I]);
				}
				return;
			}

			const double TolSqr = double(WeldTolerance) * double(WeldTolerance);
			std::unordered_map<CellKey, std::vector<uint32_t>, CellKeyHash> Grid;
			OutVertices.reserve(NumVertices);

			for (uint32_t I = 0; I < NumVertices; ++I)
			{
				if (!IsFinite(Vertices[I]))
				{
					continue;
				}

				const CellKey Cell = MakeCell(Vertices[I], WeldTolerance);
				uint32_t Found = InvalidVertex();
				for (int DX = -1; DX <= 1 && Found == InvalidVertex(); ++DX)
				{
					for (int DY = -1; DY <= 1 && Found == InvalidVertex(); ++DY)
					{
						for (int DZ = -1; DZ <= 1 && Found == InvalidVertex(); ++DZ)
						{
							const CellKey Neighbor{ Cell.X + DX, Cell.Y + DY, Cell.Z + DZ };
							auto It = Grid.find(Neighbor);
							if (It == Grid.end())
							{
								continue;
							}
							for (uint32_t Candidate : It->second)
							{
								if (SqrDistance(Vertices[I], OutVertices[Candidate]) <= TolSqr)
								{
									Found = Candidate;
									break;
								}
							}
						}
					}
				}

				if (Found == InvalidVertex())
				{
					Found = (uint32_t)OutVertices.size();
					OutVertices.push_back(Vertices[I]);
					Grid[Cell].push_back(Found);
				}
				Remap[I] = Found;
			}
		}

		static void BuildEdgeMap(const std::vector<RepairTriangle>& Triangles,
			std::unordered_map<EdgeKey, std::vector<EdgeUse>, EdgeKeyHash>& EdgeMap)
		{
			EdgeMap.clear();
			EdgeMap.reserve(Triangles.size() * 3);
			for (size_t T = 0; T < Triangles.size(); ++T)
			{
				const RepairTriangle& Tri = Triangles[T];
				if (Tri.Removed)
				{
					continue;
				}

				for (int K = 0; K < 3; ++K)
				{
					const uint32_t A = Tri.V[K];
					const uint32_t B = Tri.V[(K + 1) % 3];
					const EdgeKey Key(A, B);
					EdgeUse Use;
					Use.Triangle = (int)T;
					Use.From = A;
					Use.To = B;
					Use.Direction = (A == Key.A && B == Key.B) ? 1 : -1;
					EdgeMap[Key].push_back(Use);
				}
			}
		}

		static void BuildTriangleSoup(const Vector3* Vertices, uint32_t NumVertices,
			const uint32_t* Indices, uint32_t NumTriangles,
			const MeshTopologyRepairOptions& Options,
			std::vector<Vector3>& OutVertices, std::vector<RepairTriangle>& OutTriangles,
			MeshTopologyRepairReport& Report)
		{
			std::vector<uint32_t> Remap;
			BuildWeldedVertices(Vertices, NumVertices, Options.WeldTolerance, OutVertices, Remap);

			std::unordered_map<TriangleKey, int, TriangleKeyHash> TriangleSet;
			TriangleSet.reserve(NumTriangles);
			OutTriangles.clear();
			OutTriangles.reserve(NumTriangles);

			for (uint32_t T = 0; T < NumTriangles; ++T)
			{
				const uint32_t I0 = Indices[3 * T + 0];
				const uint32_t I1 = Indices[3 * T + 1];
				const uint32_t I2 = Indices[3 * T + 2];

				if (I0 >= NumVertices || I1 >= NumVertices || I2 >= NumVertices ||
					Remap[I0] == InvalidVertex() || Remap[I1] == InvalidVertex() || Remap[I2] == InvalidVertex())
				{
					Report.RemovedInvalidTriangles++;
					continue;
				}

				const uint32_t V[3] = { Remap[I0], Remap[I1], Remap[I2] };
				if (V[0] == V[1] || V[1] == V[2] || V[2] == V[0])
				{
					Report.RemovedDegenerateTriangles++;
					continue;
				}

				if (TriangleArea(OutVertices[V[0]], OutVertices[V[1]], OutVertices[V[2]]) <= double(Options.AreaEpsilon))
				{
					Report.RemovedDegenerateTriangles++;
					continue;
				}

				const TriangleKey Key(V[0], V[1], V[2]);
				if (Options.RemoveDuplicateTriangles && TriangleSet.find(Key) != TriangleSet.end())
				{
					Report.RemovedDuplicateTriangles++;
					continue;
				}

				TriangleSet.emplace(Key, (int)OutTriangles.size());
				RepairTriangle Tri;
				Tri.V[0] = V[0];
				Tri.V[1] = V[1];
				Tri.V[2] = V[2];
				Tri.SourceTriangle = (int)T;
				OutTriangles.push_back(Tri);
			}
		}

		static int RemoveExtraNonManifoldEdgeTriangles(std::vector<RepairTriangle>& Triangles)
		{
			std::unordered_map<EdgeKey, std::vector<EdgeUse>, EdgeKeyHash> EdgeMap;
			BuildEdgeMap(Triangles, EdgeMap);

			int Removed = 0;
			for (const auto& Pair : EdgeMap)
			{
				const std::vector<EdgeUse>& Uses = Pair.second;
				if (Uses.size() <= 2)
				{
					continue;
				}
				for (size_t I = 2; I < Uses.size(); ++I)
				{
					RepairTriangle& Tri = Triangles[Uses[I].Triangle];
					if (!Tri.Removed)
					{
						Tri.Removed = true;
						Removed++;
					}
				}
			}
			return Removed;
		}

		static int FixTriangleOrientation(std::vector<RepairTriangle>& Triangles)
		{
			std::unordered_map<EdgeKey, std::vector<EdgeUse>, EdgeKeyHash> EdgeMap;
			BuildEdgeMap(Triangles, EdgeMap);

			std::vector<int> Sign(Triangles.size(), 0);
			std::queue<int> Queue;
			for (size_t Start = 0; Start < Triangles.size(); ++Start)
			{
				if (Triangles[Start].Removed || Sign[Start] != 0)
				{
					continue;
				}

				Sign[Start] = 1;
				Queue.push((int)Start);
				while (!Queue.empty())
				{
					const int T = Queue.front();
					Queue.pop();

					const RepairTriangle& Tri = Triangles[T];
					for (int K = 0; K < 3; ++K)
					{
						const uint32_t A = Tri.V[K];
						const uint32_t B = Tri.V[(K + 1) % 3];
						const EdgeKey Key(A, B);
						auto It = EdgeMap.find(Key);
						if (It == EdgeMap.end() || It->second.size() != 2)
						{
							continue;
						}

						const EdgeUse& U0 = It->second[0];
						const EdgeUse& U1 = It->second[1];
						const EdgeUse& Self = (U0.Triangle == T) ? U0 : U1;
						const EdgeUse& Other = (U0.Triangle == T) ? U1 : U0;
						const int Desired = -Sign[T] * Self.Direction * Other.Direction;
						if (Sign[Other.Triangle] == 0)
						{
							Sign[Other.Triangle] = Desired;
							Queue.push(Other.Triangle);
						}
					}
				}
			}

			int Flipped = 0;
			for (size_t T = 0; T < Triangles.size(); ++T)
			{
				if (!Triangles[T].Removed && Sign[T] < 0)
				{
					std::swap(Triangles[T].V[1], Triangles[T].V[2]);
					Flipped++;
				}
			}
			return Flipped;
		}

		static int SplitNonManifoldVertices(std::vector<Vector3>& Vertices, std::vector<RepairTriangle>& Triangles)
		{
			std::unordered_map<EdgeKey, std::vector<EdgeUse>, EdgeKeyHash> EdgeMap;
			BuildEdgeMap(Triangles, EdgeMap);

			std::vector<std::vector<int>> VertexTriangles(Vertices.size());
			for (size_t T = 0; T < Triangles.size(); ++T)
			{
				if (Triangles[T].Removed)
				{
					continue;
				}
				for (int K = 0; K < 3; ++K)
				{
					VertexTriangles[Triangles[T].V[K]].push_back((int)T);
				}
			}

			int SplitCount = 0;
			for (uint32_t V = 0; V < (uint32_t)VertexTriangles.size(); ++V)
			{
				const std::vector<int>& Incidents = VertexTriangles[V];
				if (Incidents.size() <= 1)
				{
					continue;
				}

				std::unordered_map<int, int> LocalIndex;
				LocalIndex.reserve(Incidents.size());
				for (size_t I = 0; I < Incidents.size(); ++I)
				{
					LocalIndex.emplace(Incidents[I], (int)I);
				}

				DisjointSet LocalSet;
				LocalSet.Reset(Incidents.size());
				for (const auto& Pair : EdgeMap)
				{
					if (Pair.first.A != V && Pair.first.B != V)
					{
						continue;
					}
					const std::vector<EdgeUse>& Uses = Pair.second;
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

				std::unordered_map<int, uint32_t> ComponentVertex;
				for (size_t I = 0; I < Incidents.size(); ++I)
				{
					const int Root = LocalSet.Find((int)I);
					if (ComponentVertex.find(Root) == ComponentVertex.end())
					{
						if (ComponentVertex.empty())
						{
							ComponentVertex.emplace(Root, V);
						}
						else
						{
							const uint32_t NewVertex = (uint32_t)Vertices.size();
							Vertices.push_back(Vertices[V]);
							ComponentVertex.emplace(Root, NewVertex);
							SplitCount++;
						}
					}
				}

				for (size_t I = 0; I < Incidents.size(); ++I)
				{
					RepairTriangle& Tri = Triangles[Incidents[I]];
					const int Root = LocalSet.Find((int)I);
					const uint32_t NewVertex = ComponentVertex[Root];
					if (NewVertex == V)
					{
						continue;
					}
					for (int K = 0; K < 3; ++K)
					{
						if (Tri.V[K] == V)
						{
							Tri.V[K] = NewVertex;
						}
					}
				}
			}

			return SplitCount;
		}

		static int FillBoundaryHoles(std::vector<Vector3>& Vertices, std::vector<RepairTriangle>& Triangles,
			int MaxHoleEdges, int& AddedTriangles)
		{
			struct BoundaryEdge
			{
				uint32_t From = 0;
				uint32_t To = 0;
			};

			std::unordered_map<EdgeKey, std::vector<EdgeUse>, EdgeKeyHash> EdgeMap;
			BuildEdgeMap(Triangles, EdgeMap);

			std::vector<BoundaryEdge> BoundaryEdges;
			std::unordered_map<uint32_t, std::vector<int>> FromEdge;
			for (const auto& Pair : EdgeMap)
			{
				if (Pair.second.size() != 1)
				{
					continue;
				}
				const EdgeUse& Use = Pair.second[0];
				const int Id = (int)BoundaryEdges.size();
				BoundaryEdges.push_back(BoundaryEdge{ Use.From, Use.To });
				FromEdge[Use.From].push_back(Id);
			}

			std::vector<uint8_t> Used(BoundaryEdges.size(), 0);
			int FilledHoles = 0;
			AddedTriangles = 0;

			for (size_t Start = 0; Start < BoundaryEdges.size(); ++Start)
			{
				if (Used[Start])
				{
					continue;
				}

				std::vector<uint32_t> Loop;
				int EdgeId = (int)Start;
				bool bClosed = false;
				bool bTooLarge = false;

				while (EdgeId >= 0 && !Used[EdgeId])
				{
					const BoundaryEdge& Edge = BoundaryEdges[EdgeId];
					Used[EdgeId] = 1;

					if (Loop.empty())
					{
						Loop.push_back(Edge.From);
					}
					Loop.push_back(Edge.To);

					if (Loop.size() > 1 && Edge.To == Loop[0])
					{
						bClosed = true;
						break;
					}

					if (MaxHoleEdges > 0 && (int)Loop.size() > MaxHoleEdges + 1)
					{
						bTooLarge = true;
						break;
					}

					EdgeId = -1;
					auto It = FromEdge.find(Edge.To);
					if (It != FromEdge.end())
					{
						for (int Candidate : It->second)
						{
							if (!Used[Candidate])
							{
								EdgeId = Candidate;
								break;
							}
						}
					}
				}

				if (!bClosed || bTooLarge || Loop.size() < 4)
				{
					continue;
				}

				Loop.pop_back();
				if (Loop.size() < 3)
				{
					continue;
				}

				Vector3 Center = Vector3::Zero();
				for (uint32_t VertexId : Loop)
				{
					Center += Vertices[VertexId];
				}
				Center /= (float)Loop.size();

				const uint32_t CenterId = (uint32_t)Vertices.size();
				Vertices.push_back(Center);
				for (size_t I = 0; I < Loop.size(); ++I)
				{
					const uint32_t A = Loop[I];
					const uint32_t B = Loop[(I + 1) % Loop.size()];
					if (A == B || A == CenterId || B == CenterId)
					{
						continue;
					}

					RepairTriangle Tri;
					Tri.V[0] = B;
					Tri.V[1] = A;
					Tri.V[2] = CenterId;
					Tri.SourceTriangle = -1;
					Triangles.push_back(Tri);
					AddedTriangles++;
				}
				FilledHoles++;
			}

			return FilledHoles;
		}

		static int RemoveComponents(std::vector<Vector3>& Vertices, std::vector<RepairTriangle>& Triangles,
			const MeshTopologyRepairOptions& Options)
		{
			(void)Vertices;
			if (!Options.KeepLargestComponent && Options.MinComponentTriangleCount <= 0 && Options.MinComponentAbsVolume <= 0.0)
			{
				return 0;
			}

			int ActiveCount = 0;
			for (const RepairTriangle& Tri : Triangles)
			{
				if (!Tri.Removed)
				{
					ActiveCount++;
				}
			}
			if (ActiveCount == 0)
			{
				return 0;
			}

			std::vector<int> ActiveToLocal(Triangles.size(), -1);
			std::vector<int> LocalToTri;
			LocalToTri.reserve(ActiveCount);
			for (size_t T = 0; T < Triangles.size(); ++T)
			{
				if (!Triangles[T].Removed)
				{
					ActiveToLocal[T] = (int)LocalToTri.size();
					LocalToTri.push_back((int)T);
				}
			}

			DisjointSet Set;
			Set.Reset(LocalToTri.size());

			std::unordered_map<EdgeKey, std::vector<EdgeUse>, EdgeKeyHash> EdgeMap;
			BuildEdgeMap(Triangles, EdgeMap);
			for (const auto& Pair : EdgeMap)
			{
				const std::vector<EdgeUse>& Uses = Pair.second;
				for (size_t I = 1; I < Uses.size(); ++I)
				{
					const int A = ActiveToLocal[Uses[0].Triangle];
					const int B = ActiveToLocal[Uses[I].Triangle];
					if (A >= 0 && B >= 0)
					{
						Set.Union(A, B);
					}
				}
			}

			struct ComponentInfo
			{
				int Count = 0;
				double Volume = 0.0;
			};

			std::unordered_map<int, int> RootToComponent;
			std::vector<ComponentInfo> Components;
			std::vector<int> LocalComponent(LocalToTri.size(), -1);
			for (size_t I = 0; I < LocalToTri.size(); ++I)
			{
				const int Root = Set.Find((int)I);
				auto It = RootToComponent.find(Root);
				if (It == RootToComponent.end())
				{
					const int NewComponent = (int)Components.size();
					RootToComponent.emplace(Root, NewComponent);
					Components.push_back(ComponentInfo());
					It = RootToComponent.find(Root);
				}

				const int Component = It->second;
				LocalComponent[I] = Component;
				RepairTriangle& Tri = Triangles[LocalToTri[I]];
				Components[Component].Count++;
				Components[Component].Volume += DotCross(Vertices[Tri.V[0]], Vertices[Tri.V[1]], Vertices[Tri.V[2]]) / 6.0;
			}

			int LargestComponent = -1;
			for (size_t I = 0; I < Components.size(); ++I)
			{
				if (LargestComponent < 0 || Components[I].Count > Components[LargestComponent].Count)
				{
					LargestComponent = (int)I;
				}
			}

			int Removed = 0;
			for (size_t I = 0; I < LocalToTri.size(); ++I)
			{
				const int Component = LocalComponent[I];
				bool bRemove = false;
				if (Options.KeepLargestComponent && Component != LargestComponent)
				{
					bRemove = true;
				}
				if (Options.MinComponentTriangleCount > 0 && Components[Component].Count < Options.MinComponentTriangleCount)
				{
					bRemove = true;
				}
				if (Options.MinComponentAbsVolume > 0.0 && std::abs(Components[Component].Volume) < Options.MinComponentAbsVolume)
				{
					bRemove = true;
				}
				if (bRemove)
				{
					Triangles[LocalToTri[I]].Removed = true;
					Removed++;
				}
			}

			return Removed;
		}

		static int FlipToPositiveVolume(const std::vector<Vector3>& Vertices, std::vector<RepairTriangle>& Triangles)
		{
			double Volume = 0.0;
			int Active = 0;
			for (const RepairTriangle& Tri : Triangles)
			{
				if (!Tri.Removed)
				{
					Volume += DotCross(Vertices[Tri.V[0]], Vertices[Tri.V[1]], Vertices[Tri.V[2]]) / 6.0;
					Active++;
				}
			}
			if (Volume >= 0.0)
			{
				return 0;
			}
			for (RepairTriangle& Tri : Triangles)
			{
				if (!Tri.Removed)
				{
					std::swap(Tri.V[1], Tri.V[2]);
				}
			}
			return Active;
		}

		static void CompactSoup(const std::vector<Vector3>& Vertices, const std::vector<RepairTriangle>& Triangles,
			std::vector<Vector3>& OutVertices, std::vector<Index3>& OutTriangles)
		{
			std::vector<int> VertexMap(Vertices.size(), -1);
			OutVertices.clear();
			OutTriangles.clear();

			for (const RepairTriangle& Tri : Triangles)
			{
				if (Tri.Removed)
				{
					continue;
				}

				uint32_t V[3] = { Tri.V[0], Tri.V[1], Tri.V[2] };
				int NewV[3]{ -1, -1, -1 };
				for (int K = 0; K < 3; ++K)
				{
					if (VertexMap[V[K]] < 0)
					{
						VertexMap[V[K]] = (int)OutVertices.size();
						OutVertices.push_back(Vertices[V[K]]);
					}
					NewV[K] = VertexMap[V[K]];
				}
				if (NewV[0] != NewV[1] && NewV[1] != NewV[2] && NewV[2] != NewV[0])
				{
					OutTriangles.push_back(Index3(NewV[0], NewV[1], NewV[2]));
				}
			}
		}

		static bool BuildDynamicMesh(const std::vector<Vector3>& Vertices, const std::vector<Index3>& Triangles,
			DynamicMesh& OutMesh, MeshTopologyRepairReport& Report)
		{
			OutMesh.Clear();
			for (const Vector3& Vertex : Vertices)
			{
				OutMesh.AppendVertex(Vertex);
			}

			int Skipped = 0;
			for (size_t I = 0; I < Triangles.size(); ++I)
			{
				const Index3& Tri = Triangles[I];
				if (OutMesh.AppendTriangle(Tri) < 0)
				{
					Skipped++;
				}
			}
			Report.RemovedNonManifoldTriangles += Skipped;
			OutMesh.BuildBounds();
			return OutMesh.GetTriangleCount() > 0;
		}

		static bool RepairTopologyInternal(const Vector3* Vertices, uint32_t NumVertices,
			const uint32_t* Indices, uint32_t NumTriangles,
			DynamicMesh& OutMesh,
			const MeshTopologyRepairOptions& Options,
			MeshTopologyRepairReport& Report)
		{
			MeshValidationOptions ValidationOptions;
			ValidationOptions.AreaEpsilon = Options.AreaEpsilon;
			ValidationOptions.WeldTolerance = Options.WeldTolerance;
			ValidationOptions.CheckSelfIntersections = Options.ValidateSelfIntersections;
			Report.Before = MeshValidator::Validate(Vertices, NumVertices, Indices, NumTriangles, ValidationOptions);

			if (Vertices == nullptr || (Indices == nullptr && NumTriangles > 0))
			{
				OutMesh.Clear();
				Report.After = MeshValidator::Validate(OutMesh, ValidationOptions);
				return false;
			}

			std::vector<Vector3> WorkVertices;
			std::vector<RepairTriangle> WorkTriangles;
			BuildTriangleSoup(Vertices, NumVertices, Indices, NumTriangles, Options, WorkVertices, WorkTriangles, Report);

			if (Options.RemoveExtraNonManifoldEdgeTriangles)
			{
				Report.RemovedNonManifoldTriangles += RemoveExtraNonManifoldEdgeTriangles(WorkTriangles);
			}

			if (Options.FixOrientation)
			{
				Report.FlippedTriangles += FixTriangleOrientation(WorkTriangles);
			}

			if (Options.SplitNonManifoldVertices)
			{
				Report.SplitVertices += SplitNonManifoldVertices(WorkVertices, WorkTriangles);
			}

			if (Options.FixOrientation)
			{
				Report.FlippedTriangles += FixTriangleOrientation(WorkTriangles);
			}

			if (Options.FillHoles)
			{
				int Added = 0;
				Report.FilledHoleCount += FillBoundaryHoles(WorkVertices, WorkTriangles, Options.MaxHoleEdges, Added);
				Report.AddedFillTriangles += Added;
				if (Options.FixOrientation)
				{
					Report.FlippedTriangles += FixTriangleOrientation(WorkTriangles);
				}
			}

			Report.RemovedComponentTriangles += RemoveComponents(WorkVertices, WorkTriangles, Options);

			if (Options.FlipToPositiveVolume)
			{
				Report.FlippedTriangles += FlipToPositiveVolume(WorkVertices, WorkTriangles);
			}

			std::vector<Vector3> CompactVertices;
			std::vector<Index3> CompactTriangles;
			CompactSoup(WorkVertices, WorkTriangles, CompactVertices, CompactTriangles);
			const bool bBuilt = BuildDynamicMesh(CompactVertices, CompactTriangles, OutMesh, Report);
			Report.After = MeshValidator::Validate(OutMesh, ValidationOptions);
			return bBuilt;
		}
	}

	bool MeshRepair::RepairTopology(const Vector3* Vertices, uint32_t NumVertices,
		const uint32_t* Indices, uint32_t NumTriangles,
		DynamicMesh& OutMesh,
		const MeshTopologyRepairOptions& Options,
		MeshTopologyRepairReport* Report)
	{
		MeshTopologyRepairReport LocalReport;
		const bool bResult = RepairTopologyInternal(Vertices, NumVertices, Indices, NumTriangles, OutMesh, Options, LocalReport);
		if (Report != nullptr)
		{
			*Report = LocalReport;
		}
		return bResult;
	}

	bool MeshRepair::RepairTopology(const std::vector<Vector3>& Vertices,
		const std::vector<Index3>& Triangles,
		DynamicMesh& OutMesh,
		const MeshTopologyRepairOptions& Options,
		MeshTopologyRepairReport* Report)
	{
		std::vector<uint32_t> Indices;
		Indices.reserve(Triangles.size() * 3);
		for (const Index3& Tri : Triangles)
		{
			Indices.push_back((uint32_t)Tri.a);
			Indices.push_back((uint32_t)Tri.b);
			Indices.push_back((uint32_t)Tri.c);
		}
		return RepairTopology(Vertices.data(), (uint32_t)Vertices.size(), Indices.data(), (uint32_t)Triangles.size(),
			OutMesh, Options, Report);
	}

	bool MeshRepair::RepairTopology(const StaticMesh& Mesh,
		DynamicMesh& OutMesh,
		const MeshTopologyRepairOptions& Options,
		MeshTopologyRepairReport* Report)
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
		return RepairTopology(Mesh.Vertices, Mesh.GetVertexCount(), Indices.data(), Mesh.GetTriangleCount(),
			OutMesh, Options, Report);
	}

	bool MeshRepair::RepairTopology(const DynamicMesh& Mesh,
		DynamicMesh& OutMesh,
		const MeshTopologyRepairOptions& Options,
		MeshTopologyRepairReport* Report)
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
		return RepairTopology(Vertices, Triangles, OutMesh, Options, Report);
	}
}
