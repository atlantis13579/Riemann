#include "MeshDistance.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "../CollisionPrimitive/StaticMesh.h"
#include "../CollisionPrimitive/Triangle3.h"
#include "DynamicMesh.h"
#include "MeshValidator.h"

namespace Riemann
{
	namespace
	{
		struct DistanceTriangle
		{
			Vector3 A = Vector3::Zero();
			Vector3 B = Vector3::Zero();
			Vector3 C = Vector3::Zero();
			Vector3 Center = Vector3::Zero();
			Box3 Bounds = Box3::Empty();
			float Area = 0.0f;
			int OriginalIndex = -1;
		};

		struct DistanceBVHNode
		{
			Box3 Bounds = Box3::Empty();
			int Left = -1;
			int Right = -1;
			int Start = 0;
			int Count = 0;

			bool IsLeaf() const
			{
				return Left < 0 && Right < 0;
			}
		};

		static int ClampInt(int Value, int MinValue, int MaxValue)
		{
			return std::min(std::max(Value, MinValue), MaxValue);
		}

		static bool IsFinite(const Vector3& V)
		{
			return std::isfinite(V.x) && std::isfinite(V.y) && std::isfinite(V.z);
		}

		static float SqrDistanceToBox(const Box3& Box, const Vector3& P)
		{
			float Sqr = 0.0f;
			for (int Axis = 0; Axis < 3; ++Axis)
			{
				const float V = P[Axis];
				if (V < Box.Min[Axis])
				{
					const float D = Box.Min[Axis] - V;
					Sqr += D * D;
				}
				else if (V > Box.Max[Axis])
				{
					const float D = V - Box.Max[Axis];
					Sqr += D * D;
				}
			}
			return Sqr;
		}

		static std::vector<uint32_t> FlattenTriangles(const std::vector<Index3>& Triangles)
		{
			std::vector<uint32_t> Indices;
			Indices.reserve(Triangles.size() * 3);
			for (const Index3& Tri : Triangles)
			{
				Indices.push_back((uint32_t)Tri.a);
				Indices.push_back((uint32_t)Tri.b);
				Indices.push_back((uint32_t)Tri.c);
			}
			return Indices;
		}

		static std::vector<uint32_t> FlattenStaticMesh(const StaticMesh& Mesh)
		{
			std::vector<uint32_t> Indices;
			Indices.reserve((size_t)Mesh.GetTriangleCount() * 3);
			for (uint32_t TriIndex = 0; TriIndex < Mesh.GetTriangleCount(); ++TriIndex)
			{
				uint32_t I0 = 0;
				uint32_t I1 = 0;
				uint32_t I2 = 0;
				Mesh.GetVertIndices(TriIndex, I0, I1, I2);
				Indices.push_back(I0);
				Indices.push_back(I1);
				Indices.push_back(I2);
			}
			return Indices;
		}

		static void ExtractDynamicMesh(const DynamicMesh& Mesh, std::vector<Vector3>& Vertices, std::vector<uint32_t>& Indices)
		{
			Vertices.clear();
			Indices.clear();

			const int NumVertices = Mesh.GetVertexCount();
			Vertices.reserve((size_t)NumVertices);
			for (int VertexIndex = 0; VertexIndex < NumVertices; ++VertexIndex)
			{
				Vertices.push_back(Mesh.GetVertex(VertexIndex));
			}

			const int NumTriangles = Mesh.GetTriangleCount();
			Indices.reserve((size_t)NumTriangles * 3);
			for (int TriIndex = 0; TriIndex < NumTriangles; ++TriIndex)
			{
				if (!Mesh.IsTriangle(TriIndex))
				{
					continue;
				}

				const Index3 Tri = Mesh.GetTriangle(TriIndex);
				Indices.push_back((uint32_t)Tri.a);
				Indices.push_back((uint32_t)Tri.b);
				Indices.push_back((uint32_t)Tri.c);
			}
		}

		static std::vector<DistanceTriangle> BuildDistanceTriangles(
			const Vector3* Vertices,
			uint32_t NumVertices,
			const uint32_t* Indices,
			uint32_t NumTriangles,
			float DegenerateAreaEpsilon)
		{
			std::vector<DistanceTriangle> Triangles;
			if (Vertices == nullptr || Indices == nullptr || NumVertices == 0 || NumTriangles == 0)
			{
				return Triangles;
			}

			Triangles.reserve(NumTriangles);
			for (uint32_t TriIndex = 0; TriIndex < NumTriangles; ++TriIndex)
			{
				const uint32_t I0 = Indices[3 * TriIndex + 0];
				const uint32_t I1 = Indices[3 * TriIndex + 1];
				const uint32_t I2 = Indices[3 * TriIndex + 2];
				if (I0 >= NumVertices || I1 >= NumVertices || I2 >= NumVertices)
				{
					continue;
				}

				const Vector3& A = Vertices[I0];
				const Vector3& B = Vertices[I1];
				const Vector3& C = Vertices[I2];
				if (!IsFinite(A) || !IsFinite(B) || !IsFinite(C))
				{
					continue;
				}

				const float Area = Triangle3::CalculateArea3D(A, B, C);
				if (!(Area > DegenerateAreaEpsilon))
				{
					continue;
				}

				DistanceTriangle Tri;
				Tri.A = A;
				Tri.B = B;
				Tri.C = C;
				Tri.Center = (A + B + C) / 3.0f;
				Tri.Bounds = Box3(A, B, C);
				Tri.Area = Area;
				Tri.OriginalIndex = (int)TriIndex;
				Triangles.push_back(Tri);
			}

			return Triangles;
		}

		class DistanceBVH
		{
		public:
			bool Build(const std::vector<DistanceTriangle>& InTriangles, int InLeafTriangleCount)
			{
				Triangles = InTriangles;
				LeafTriangleCount = ClampInt(InLeafTriangleCount, 1, 64);
				Indices.resize(Triangles.size());
				for (size_t Index = 0; Index < Indices.size(); ++Index)
				{
					Indices[Index] = (int)Index;
				}

				Nodes.clear();
				if (Triangles.empty())
				{
					return false;
				}

				RootIndex = BuildNode(0, (int)Indices.size());
				return RootIndex >= 0;
			}

			bool FindNearest(const Vector3& Point, float& SqrDistance, Vector3& ClosestPoint, int& TriangleIndex) const
			{
				if (RootIndex < 0)
				{
					return false;
				}

				SqrDistance = std::numeric_limits<float>::max();
				ClosestPoint = Vector3::Zero();
				TriangleIndex = -1;
				FindNearestNode(RootIndex, Point, SqrDistance, ClosestPoint, TriangleIndex);
				return TriangleIndex >= 0;
			}

			uint32_t GetTriangleCount() const
			{
				return (uint32_t)Triangles.size();
			}

		private:
			int BuildNode(int Start, int Count)
			{
				const int NodeIndex = (int)Nodes.size();
				Nodes.push_back(DistanceBVHNode());

				Box3 Bounds = Triangles[Indices[Start]].Bounds;
				Box3 CenterBounds(Triangles[Indices[Start]].Center, Triangles[Indices[Start]].Center);
				for (int Offset = 1; Offset < Count; ++Offset)
				{
					const DistanceTriangle& Tri = Triangles[Indices[Start + Offset]];
					Bounds.Encapsulate(Tri.Bounds);
					CenterBounds.Encapsulate(Tri.Center);
				}

				Nodes[NodeIndex].Bounds = Bounds;
				Nodes[NodeIndex].Start = Start;
				Nodes[NodeIndex].Count = Count;

				if (Count <= LeafTriangleCount)
				{
					return NodeIndex;
				}

				const int Axis = CenterBounds.GetSize().LargestAxis();
				const int Mid = Start + Count / 2;
				std::nth_element(
					Indices.begin() + Start,
					Indices.begin() + Mid,
					Indices.begin() + Start + Count,
					[this, Axis](int Lhs, int Rhs)
					{
						return Triangles[Lhs].Center[Axis] < Triangles[Rhs].Center[Axis];
					});

				const int Left = BuildNode(Start, Mid - Start);
				const int Right = BuildNode(Mid, Start + Count - Mid);
				Nodes[NodeIndex].Left = Left;
				Nodes[NodeIndex].Right = Right;
				Nodes[NodeIndex].Start = 0;
				Nodes[NodeIndex].Count = 0;
				return NodeIndex;
			}

			void FindNearestNode(int NodeIndex, const Vector3& Point, float& BestSqrDistance, Vector3& BestPoint, int& BestTriangleIndex) const
			{
				const DistanceBVHNode& Node = Nodes[NodeIndex];
				if (SqrDistanceToBox(Node.Bounds, Point) > BestSqrDistance)
				{
					return;
				}

				if (Node.IsLeaf())
				{
					for (int Offset = 0; Offset < Node.Count; ++Offset)
					{
						const DistanceTriangle& Tri = Triangles[Indices[Node.Start + Offset]];
						const float SqrDistance = Triangle3::SqrDistancePointToTriangle(Point, Tri.A, Tri.B, Tri.C);
						if (SqrDistance < BestSqrDistance)
						{
							BestSqrDistance = SqrDistance;
							BestPoint = Triangle3::ClosestPointOnTriangleToPoint(Point, Tri.A, Tri.B, Tri.C);
							BestTriangleIndex = Tri.OriginalIndex;
						}
					}
					return;
				}

				const float LeftDistance = SqrDistanceToBox(Nodes[Node.Left].Bounds, Point);
				const float RightDistance = SqrDistanceToBox(Nodes[Node.Right].Bounds, Point);

				const int First = LeftDistance <= RightDistance ? Node.Left : Node.Right;
				const int Second = LeftDistance <= RightDistance ? Node.Right : Node.Left;
				FindNearestNode(First, Point, BestSqrDistance, BestPoint, BestTriangleIndex);
				FindNearestNode(Second, Point, BestSqrDistance, BestPoint, BestTriangleIndex);
			}

		private:
			std::vector<DistanceTriangle> Triangles;
			std::vector<int> Indices;
			std::vector<DistanceBVHNode> Nodes;
			int LeafTriangleCount = 16;
			int RootIndex = -1;
		};

		static int ComputeTriangleResolution(const DistanceTriangle& Tri, double TotalArea, const MeshSurfaceDistanceOptions& Options)
		{
			int Resolution = std::max(1, Options.SamplesPerTriangleEdge);
			if (Options.TargetSampleCount > 0 && TotalArea > 0.0)
			{
				const double DesiredSamples = Options.TargetSampleCount * ((double)Tri.Area / TotalArea);
				const int DynamicResolution = (int)std::ceil((std::sqrt(8.0 * DesiredSamples + 1.0) - 3.0) * 0.5);
				Resolution = std::max(Resolution, DynamicResolution);
			}

			const int MaxResolution = std::max(1, Options.MaxSamplesPerTriangleEdge);
			return ClampInt(Resolution, 1, MaxResolution);
		}

		static int BarycentricGridSampleCount(int Resolution, bool IncludeCentroid)
		{
			const int GridCount = (Resolution + 1) * (Resolution + 2) / 2;
			return IncludeCentroid ? GridCount + 1 : GridCount;
		}

		static void AccumulateSample(
			MeshDistanceStats& Stats,
			const DistanceBVH& TargetBVH,
			const Vector3& Point,
			int SourceTriangleIndex,
			double Weight)
		{
			float SqrDistance = 0.0f;
			Vector3 ClosestPoint = Vector3::Zero();
			int TargetTriangleIndex = -1;
			if (!TargetBVH.FindNearest(Point, SqrDistance, ClosestPoint, TargetTriangleIndex))
			{
				return;
			}

			const double Distance = std::sqrt((double)SqrDistance);
			Stats.SampleCount++;
			Stats.TotalWeight += Weight;
			Stats.MeanDistance += Distance * Weight;
			Stats.MeanSquaredDistance += (double)SqrDistance * Weight;
			if (Distance > Stats.MaxDistance)
			{
				Stats.MaxDistance = Distance;
				Stats.SourcePointAtMax = Point;
				Stats.TargetPointAtMax = ClosestPoint;
				Stats.SourceTriangleAtMax = SourceTriangleIndex;
				Stats.TargetTriangleAtMax = TargetTriangleIndex;
			}
		}

		static void FinalizeStats(MeshDistanceStats& Stats)
		{
			if (Stats.SampleCount == 0 || !(Stats.TotalWeight > 0.0))
			{
				Stats.bValid = false;
				return;
			}

			Stats.MeanDistance /= Stats.TotalWeight;
			Stats.MeanSquaredDistance /= Stats.TotalWeight;
			Stats.RmsDistance = std::sqrt(Stats.MeanSquaredDistance);
			Stats.bValid = true;
		}

		static void FillVolumeInfoFromValidation(MeshVolumeInfo& Info, const MeshValidationReport& Report)
		{
			Info.VertexCount = Report.VertexCount;
			Info.TriangleCount = Report.TriangleCount;
			Info.ValidTriangleCount = Report.ValidTriangleCount;
			Info.InvalidIndexCount = Report.InvalidIndexCount;
			Info.DegenerateTriangleCount = Report.DegenerateTriangleCount;
			Info.BoundaryEdgeCount = Report.BoundaryEdgeCount;
			Info.NonManifoldEdgeCount = Report.NonManifoldEdgeCount;
			Info.SameDirectionEdgeCount = Report.SameDirectionEdgeCount;
			Info.NonManifoldVertexCount = Report.NonManifoldVertexCount;
			Info.SelfIntersectionCount = Report.SelfIntersectionCount;
			Info.SignedVolume = Report.SignedVolume;
			Info.AbsoluteVolume = Report.AbsoluteVolume;
			Info.SurfaceArea = Report.SurfaceArea;
			Info.Bounds = Report.Bounds;
			Info.bHasValidSurface = Report.HasValidData();
			Info.bIsClosed = Report.IsClosed();
			Info.bIsOrientable = Report.IsOrientable();
			Info.bIsTwoManifold = Report.IsTwoManifold();
			Info.bIsSolidCandidate = Report.IsSolidCandidate();
		}

		static MeshVolumeInfo ComputeBestEffortVolume(
			const Vector3* Vertices,
			uint32_t NumVertices,
			const uint32_t* Indices,
			uint32_t NumTriangles,
			float AreaEpsilon)
		{
			MeshVolumeInfo Info;
			Info.VertexCount = NumVertices;
			Info.TriangleCount = NumTriangles;

			if (Vertices == nullptr || Indices == nullptr || NumVertices == 0 || NumTriangles == 0)
			{
				return Info;
			}

			bool bHasBounds = false;
			for (uint32_t TriIndex = 0; TriIndex < NumTriangles; ++TriIndex)
			{
				const uint32_t I0 = Indices[3 * TriIndex + 0];
				const uint32_t I1 = Indices[3 * TriIndex + 1];
				const uint32_t I2 = Indices[3 * TriIndex + 2];
				if (I0 >= NumVertices || I1 >= NumVertices || I2 >= NumVertices)
				{
					Info.InvalidIndexCount++;
					continue;
				}

				const Vector3& A = Vertices[I0];
				const Vector3& B = Vertices[I1];
				const Vector3& C = Vertices[I2];
				if (!IsFinite(A) || !IsFinite(B) || !IsFinite(C))
				{
					Info.InvalidIndexCount++;
					continue;
				}

				const float Area = Triangle3::CalculateArea3D(A, B, C);
				if (!(Area > AreaEpsilon))
				{
					Info.DegenerateTriangleCount++;
					continue;
				}

				Info.ValidTriangleCount++;
				Info.SurfaceArea += Area;
				Info.SignedVolume += (double)A.Dot(B.Cross(C)) / 6.0;

				if (!bHasBounds)
				{
					Info.Bounds = Box3(A, B, C);
					bHasBounds = true;
				}
				else
				{
					Info.Bounds.Encapsulate(A, B, C);
				}
			}

			Info.AbsoluteVolume = std::abs(Info.SignedVolume);
			Info.bHasValidSurface = Info.ValidTriangleCount > 0;
			return Info;
		}

		static MeshSurfaceDistanceReport BuildSurfaceReport(const MeshDistanceStats& AToB, const MeshDistanceStats& BToA)
		{
			MeshSurfaceDistanceReport Report;
			Report.AToB = AToB;
			Report.BToA = BToA;
			Report.bValid = AToB.bValid && BToA.bValid;
			if (!Report.bValid)
			{
				return Report;
			}

			Report.OneSidedHausdorffAB = AToB.MaxDistance;
			Report.OneSidedHausdorffBA = BToA.MaxDistance;
			Report.HausdorffDistance = std::max(AToB.MaxDistance, BToA.MaxDistance);

			const double TotalWeight = AToB.TotalWeight + BToA.TotalWeight;
			if (TotalWeight > 0.0)
			{
				Report.MeanBoundaryDistance =
					(AToB.MeanDistance * AToB.TotalWeight + BToA.MeanDistance * BToA.TotalWeight) / TotalWeight;
				const double MeanSquared =
					(AToB.MeanSquaredDistance * AToB.TotalWeight + BToA.MeanSquaredDistance * BToA.TotalWeight) / TotalWeight;
				Report.RmsBoundaryDistance = std::sqrt(MeanSquared);
			}

			Report.ChamferDistance = AToB.MeanDistance + BToA.MeanDistance;
			Report.MeanSquaredChamferDistance = AToB.MeanSquaredDistance + BToA.MeanSquaredDistance;
			return Report;
		}

		static const Vector3* GetStaticMeshVertices(const StaticMesh& Mesh)
		{
			if (Mesh.Vertices != nullptr)
			{
				return Mesh.Vertices;
			}
			return Mesh.mVertices.empty() ? nullptr : Mesh.mVertices.data();
		}
	}

	MeshDistanceStats MeshDistance::ComputeOneSidedSurfaceDistance(
		const Vector3* SourceVertices,
		uint32_t NumSourceVertices,
		const uint32_t* SourceIndices,
		uint32_t NumSourceTriangles,
		const Vector3* TargetVertices,
		uint32_t NumTargetVertices,
		const uint32_t* TargetIndices,
		uint32_t NumTargetTriangles,
		const MeshSurfaceDistanceOptions& Options)
	{
		MeshDistanceStats Stats;

		const std::vector<DistanceTriangle> SourceTriangles = BuildDistanceTriangles(
			SourceVertices, NumSourceVertices, SourceIndices, NumSourceTriangles, Options.DegenerateAreaEpsilon);
		const std::vector<DistanceTriangle> TargetTriangles = BuildDistanceTriangles(
			TargetVertices, NumTargetVertices, TargetIndices, NumTargetTriangles, Options.DegenerateAreaEpsilon);

		Stats.SourceTriangleCount = (uint32_t)SourceTriangles.size();
		Stats.TargetTriangleCount = (uint32_t)TargetTriangles.size();
		if (SourceTriangles.empty() || TargetTriangles.empty())
		{
			return Stats;
		}

		DistanceBVH TargetBVH;
		if (!TargetBVH.Build(TargetTriangles, Options.BVHLeafTriangleCount))
		{
			return Stats;
		}

		double TotalArea = 0.0;
		for (const DistanceTriangle& Tri : SourceTriangles)
		{
			TotalArea += Tri.Area;
		}

		for (const DistanceTriangle& Tri : SourceTriangles)
		{
			const int Resolution = ComputeTriangleResolution(Tri, TotalArea, Options);
			const int SampleCount = BarycentricGridSampleCount(Resolution, Options.IncludeTriangleCentroids);
			const double Weight = Options.AreaWeighted ? ((double)Tri.Area / SampleCount) : 1.0;

			for (int I = 0; I <= Resolution; ++I)
			{
				for (int J = 0; J <= Resolution - I; ++J)
				{
					const int K = Resolution - I - J;
					const float U = (float)I / (float)Resolution;
					const float V = (float)J / (float)Resolution;
					const float W = (float)K / (float)Resolution;
					const Vector3 Point = Tri.A * U + Tri.B * V + Tri.C * W;
					AccumulateSample(Stats, TargetBVH, Point, Tri.OriginalIndex, Weight);
				}
			}

			if (Options.IncludeTriangleCentroids)
			{
				AccumulateSample(Stats, TargetBVH, Tri.Center, Tri.OriginalIndex, Weight);
			}
		}

		FinalizeStats(Stats);
		return Stats;
	}

	MeshSurfaceDistanceReport MeshDistance::ComputeSurfaceDistance(
		const Vector3* VerticesA,
		uint32_t NumVerticesA,
		const uint32_t* IndicesA,
		uint32_t NumTrianglesA,
		const Vector3* VerticesB,
		uint32_t NumVerticesB,
		const uint32_t* IndicesB,
		uint32_t NumTrianglesB,
		const MeshSurfaceDistanceOptions& Options)
	{
		const MeshDistanceStats AToB = ComputeOneSidedSurfaceDistance(
			VerticesA, NumVerticesA, IndicesA, NumTrianglesA,
			VerticesB, NumVerticesB, IndicesB, NumTrianglesB, Options);
		const MeshDistanceStats BToA = ComputeOneSidedSurfaceDistance(
			VerticesB, NumVerticesB, IndicesB, NumTrianglesB,
			VerticesA, NumVerticesA, IndicesA, NumTrianglesA, Options);
		return BuildSurfaceReport(AToB, BToA);
	}

	MeshSurfaceDistanceReport MeshDistance::ComputeBoundaryDistance(
		const Vector3* VerticesA,
		uint32_t NumVerticesA,
		const uint32_t* IndicesA,
		uint32_t NumTrianglesA,
		const Vector3* VerticesB,
		uint32_t NumVerticesB,
		const uint32_t* IndicesB,
		uint32_t NumTrianglesB,
		const MeshSurfaceDistanceOptions& Options)
	{
		return ComputeSurfaceDistance(
			VerticesA, NumVerticesA, IndicesA, NumTrianglesA,
			VerticesB, NumVerticesB, IndicesB, NumTrianglesB, Options);
	}

	double MeshDistance::ComputeHausdorffDistance(
		const Vector3* VerticesA,
		uint32_t NumVerticesA,
		const uint32_t* IndicesA,
		uint32_t NumTrianglesA,
		const Vector3* VerticesB,
		uint32_t NumVerticesB,
		const uint32_t* IndicesB,
		uint32_t NumTrianglesB,
		const MeshSurfaceDistanceOptions& Options)
	{
		return ComputeSurfaceDistance(
			VerticesA, NumVerticesA, IndicesA, NumTrianglesA,
			VerticesB, NumVerticesB, IndicesB, NumTrianglesB, Options).HausdorffDistance;
	}

	MeshVolumeInfo MeshDistance::ComputeVolume(
		const Vector3* Vertices,
		uint32_t NumVertices,
		const uint32_t* Indices,
		uint32_t NumTriangles,
		const MeshVolumeOptions& Options)
	{
		if (!Options.ValidateSolid)
		{
			return ComputeBestEffortVolume(Vertices, NumVertices, Indices, NumTriangles, Options.AreaEpsilon);
		}

		MeshValidationOptions ValidationOptions;
		ValidationOptions.AreaEpsilon = Options.AreaEpsilon;
		ValidationOptions.WeldTolerance = Options.WeldTolerance;
		ValidationOptions.CheckSelfIntersections = Options.CheckSelfIntersections;

		MeshVolumeInfo Info;
		const MeshValidationReport Report = MeshValidator::Validate(
			Vertices, NumVertices, Indices, NumTriangles, ValidationOptions);
		FillVolumeInfoFromValidation(Info, Report);
		return Info;
	}

	MeshVolumeDistanceReport MeshDistance::ComputeVolumeDistance(
		const Vector3* VerticesA,
		uint32_t NumVerticesA,
		const uint32_t* IndicesA,
		uint32_t NumTrianglesA,
		const Vector3* VerticesB,
		uint32_t NumVerticesB,
		const uint32_t* IndicesB,
		uint32_t NumTrianglesB,
		const MeshVolumeOptions& Options)
	{
		MeshVolumeDistanceReport Report;
		Report.A = ComputeVolume(VerticesA, NumVerticesA, IndicesA, NumTrianglesA, Options);
		Report.B = ComputeVolume(VerticesB, NumVerticesB, IndicesB, NumTrianglesB, Options);
		Report.SignedVolumeDifference = Report.A.SignedVolume - Report.B.SignedVolume;
		Report.AbsoluteVolumeDifference = std::abs(Report.A.AbsoluteVolume - Report.B.AbsoluteVolume);

		const double Denominator = std::max(std::max(Report.A.AbsoluteVolume, Report.B.AbsoluteVolume), 1e-12);
		Report.RelativeAbsoluteVolumeDifference = Report.AbsoluteVolumeDifference / Denominator;
		return Report;
	}

	MeshDistanceReport MeshDistance::ComputeAll(
		const Vector3* VerticesA,
		uint32_t NumVerticesA,
		const uint32_t* IndicesA,
		uint32_t NumTrianglesA,
		const Vector3* VerticesB,
		uint32_t NumVerticesB,
		const uint32_t* IndicesB,
		uint32_t NumTrianglesB,
		const MeshSurfaceDistanceOptions& SurfaceOptions,
		const MeshVolumeOptions& VolumeOptions)
	{
		MeshDistanceReport Report;
		Report.Surface = ComputeSurfaceDistance(
			VerticesA, NumVerticesA, IndicesA, NumTrianglesA,
			VerticesB, NumVerticesB, IndicesB, NumTrianglesB, SurfaceOptions);
		Report.Volume = ComputeVolumeDistance(
			VerticesA, NumVerticesA, IndicesA, NumTrianglesA,
			VerticesB, NumVerticesB, IndicesB, NumTrianglesB, VolumeOptions);
		return Report;
	}

	MeshDistanceStats MeshDistance::ComputeOneSidedSurfaceDistance(
		const std::vector<Vector3>& SourceVertices,
		const std::vector<Index3>& SourceTriangles,
		const std::vector<Vector3>& TargetVertices,
		const std::vector<Index3>& TargetTriangles,
		const MeshSurfaceDistanceOptions& Options)
	{
		const std::vector<uint32_t> SourceIndices = FlattenTriangles(SourceTriangles);
		const std::vector<uint32_t> TargetIndices = FlattenTriangles(TargetTriangles);
		return ComputeOneSidedSurfaceDistance(
			SourceVertices.empty() ? nullptr : SourceVertices.data(), (uint32_t)SourceVertices.size(),
			SourceIndices.empty() ? nullptr : SourceIndices.data(), (uint32_t)SourceTriangles.size(),
			TargetVertices.empty() ? nullptr : TargetVertices.data(), (uint32_t)TargetVertices.size(),
			TargetIndices.empty() ? nullptr : TargetIndices.data(), (uint32_t)TargetTriangles.size(),
			Options);
	}

	MeshSurfaceDistanceReport MeshDistance::ComputeSurfaceDistance(
		const std::vector<Vector3>& VerticesA,
		const std::vector<Index3>& TrianglesA,
		const std::vector<Vector3>& VerticesB,
		const std::vector<Index3>& TrianglesB,
		const MeshSurfaceDistanceOptions& Options)
	{
		const std::vector<uint32_t> IndicesA = FlattenTriangles(TrianglesA);
		const std::vector<uint32_t> IndicesB = FlattenTriangles(TrianglesB);
		return ComputeSurfaceDistance(
			VerticesA.empty() ? nullptr : VerticesA.data(), (uint32_t)VerticesA.size(),
			IndicesA.empty() ? nullptr : IndicesA.data(), (uint32_t)TrianglesA.size(),
			VerticesB.empty() ? nullptr : VerticesB.data(), (uint32_t)VerticesB.size(),
			IndicesB.empty() ? nullptr : IndicesB.data(), (uint32_t)TrianglesB.size(),
			Options);
	}

	MeshSurfaceDistanceReport MeshDistance::ComputeBoundaryDistance(
		const std::vector<Vector3>& VerticesA,
		const std::vector<Index3>& TrianglesA,
		const std::vector<Vector3>& VerticesB,
		const std::vector<Index3>& TrianglesB,
		const MeshSurfaceDistanceOptions& Options)
	{
		return ComputeSurfaceDistance(VerticesA, TrianglesA, VerticesB, TrianglesB, Options);
	}

	double MeshDistance::ComputeHausdorffDistance(
		const std::vector<Vector3>& VerticesA,
		const std::vector<Index3>& TrianglesA,
		const std::vector<Vector3>& VerticesB,
		const std::vector<Index3>& TrianglesB,
		const MeshSurfaceDistanceOptions& Options)
	{
		return ComputeSurfaceDistance(VerticesA, TrianglesA, VerticesB, TrianglesB, Options).HausdorffDistance;
	}

	MeshVolumeInfo MeshDistance::ComputeVolume(
		const std::vector<Vector3>& Vertices,
		const std::vector<Index3>& Triangles,
		const MeshVolumeOptions& Options)
	{
		const std::vector<uint32_t> Indices = FlattenTriangles(Triangles);
		return ComputeVolume(
			Vertices.empty() ? nullptr : Vertices.data(), (uint32_t)Vertices.size(),
			Indices.empty() ? nullptr : Indices.data(), (uint32_t)Triangles.size(),
			Options);
	}

	MeshVolumeDistanceReport MeshDistance::ComputeVolumeDistance(
		const std::vector<Vector3>& VerticesA,
		const std::vector<Index3>& TrianglesA,
		const std::vector<Vector3>& VerticesB,
		const std::vector<Index3>& TrianglesB,
		const MeshVolumeOptions& Options)
	{
		const std::vector<uint32_t> IndicesA = FlattenTriangles(TrianglesA);
		const std::vector<uint32_t> IndicesB = FlattenTriangles(TrianglesB);
		return ComputeVolumeDistance(
			VerticesA.empty() ? nullptr : VerticesA.data(), (uint32_t)VerticesA.size(),
			IndicesA.empty() ? nullptr : IndicesA.data(), (uint32_t)TrianglesA.size(),
			VerticesB.empty() ? nullptr : VerticesB.data(), (uint32_t)VerticesB.size(),
			IndicesB.empty() ? nullptr : IndicesB.data(), (uint32_t)TrianglesB.size(),
			Options);
	}

	MeshDistanceStats MeshDistance::ComputeOneSidedSurfaceDistance(
		const StaticMesh& SourceMesh,
		const StaticMesh& TargetMesh,
		const MeshSurfaceDistanceOptions& Options)
	{
		const std::vector<uint32_t> SourceIndices = FlattenStaticMesh(SourceMesh);
		const std::vector<uint32_t> TargetIndices = FlattenStaticMesh(TargetMesh);
		return ComputeOneSidedSurfaceDistance(
			GetStaticMeshVertices(SourceMesh), SourceMesh.GetVertexCount(),
			SourceIndices.empty() ? nullptr : SourceIndices.data(), SourceMesh.GetTriangleCount(),
			GetStaticMeshVertices(TargetMesh), TargetMesh.GetVertexCount(),
			TargetIndices.empty() ? nullptr : TargetIndices.data(), TargetMesh.GetTriangleCount(),
			Options);
	}

	MeshSurfaceDistanceReport MeshDistance::ComputeSurfaceDistance(
		const StaticMesh& MeshA,
		const StaticMesh& MeshB,
		const MeshSurfaceDistanceOptions& Options)
	{
		const std::vector<uint32_t> IndicesA = FlattenStaticMesh(MeshA);
		const std::vector<uint32_t> IndicesB = FlattenStaticMesh(MeshB);
		return ComputeSurfaceDistance(
			GetStaticMeshVertices(MeshA), MeshA.GetVertexCount(),
			IndicesA.empty() ? nullptr : IndicesA.data(), MeshA.GetTriangleCount(),
			GetStaticMeshVertices(MeshB), MeshB.GetVertexCount(),
			IndicesB.empty() ? nullptr : IndicesB.data(), MeshB.GetTriangleCount(),
			Options);
	}

	MeshSurfaceDistanceReport MeshDistance::ComputeBoundaryDistance(
		const StaticMesh& MeshA,
		const StaticMesh& MeshB,
		const MeshSurfaceDistanceOptions& Options)
	{
		return ComputeSurfaceDistance(MeshA, MeshB, Options);
	}

	double MeshDistance::ComputeHausdorffDistance(
		const StaticMesh& MeshA,
		const StaticMesh& MeshB,
		const MeshSurfaceDistanceOptions& Options)
	{
		return ComputeSurfaceDistance(MeshA, MeshB, Options).HausdorffDistance;
	}

	MeshVolumeInfo MeshDistance::ComputeVolume(const StaticMesh& Mesh, const MeshVolumeOptions& Options)
	{
		const std::vector<uint32_t> Indices = FlattenStaticMesh(Mesh);
		return ComputeVolume(
			GetStaticMeshVertices(Mesh), Mesh.GetVertexCount(),
			Indices.empty() ? nullptr : Indices.data(), Mesh.GetTriangleCount(),
			Options);
	}

	MeshVolumeDistanceReport MeshDistance::ComputeVolumeDistance(
		const StaticMesh& MeshA,
		const StaticMesh& MeshB,
		const MeshVolumeOptions& Options)
	{
		const std::vector<uint32_t> IndicesA = FlattenStaticMesh(MeshA);
		const std::vector<uint32_t> IndicesB = FlattenStaticMesh(MeshB);
		return ComputeVolumeDistance(
			GetStaticMeshVertices(MeshA), MeshA.GetVertexCount(),
			IndicesA.empty() ? nullptr : IndicesA.data(), MeshA.GetTriangleCount(),
			GetStaticMeshVertices(MeshB), MeshB.GetVertexCount(),
			IndicesB.empty() ? nullptr : IndicesB.data(), MeshB.GetTriangleCount(),
			Options);
	}

	MeshDistanceStats MeshDistance::ComputeOneSidedSurfaceDistance(
		const DynamicMesh& SourceMesh,
		const DynamicMesh& TargetMesh,
		const MeshSurfaceDistanceOptions& Options)
	{
		std::vector<Vector3> SourceVertices;
		std::vector<uint32_t> SourceIndices;
		std::vector<Vector3> TargetVertices;
		std::vector<uint32_t> TargetIndices;
		ExtractDynamicMesh(SourceMesh, SourceVertices, SourceIndices);
		ExtractDynamicMesh(TargetMesh, TargetVertices, TargetIndices);

		return ComputeOneSidedSurfaceDistance(
			SourceVertices.empty() ? nullptr : SourceVertices.data(), (uint32_t)SourceVertices.size(),
			SourceIndices.empty() ? nullptr : SourceIndices.data(), (uint32_t)(SourceIndices.size() / 3),
			TargetVertices.empty() ? nullptr : TargetVertices.data(), (uint32_t)TargetVertices.size(),
			TargetIndices.empty() ? nullptr : TargetIndices.data(), (uint32_t)(TargetIndices.size() / 3),
			Options);
	}

	MeshSurfaceDistanceReport MeshDistance::ComputeSurfaceDistance(
		const DynamicMesh& MeshA,
		const DynamicMesh& MeshB,
		const MeshSurfaceDistanceOptions& Options)
	{
		std::vector<Vector3> VerticesA;
		std::vector<uint32_t> IndicesA;
		std::vector<Vector3> VerticesB;
		std::vector<uint32_t> IndicesB;
		ExtractDynamicMesh(MeshA, VerticesA, IndicesA);
		ExtractDynamicMesh(MeshB, VerticesB, IndicesB);

		return ComputeSurfaceDistance(
			VerticesA.empty() ? nullptr : VerticesA.data(), (uint32_t)VerticesA.size(),
			IndicesA.empty() ? nullptr : IndicesA.data(), (uint32_t)(IndicesA.size() / 3),
			VerticesB.empty() ? nullptr : VerticesB.data(), (uint32_t)VerticesB.size(),
			IndicesB.empty() ? nullptr : IndicesB.data(), (uint32_t)(IndicesB.size() / 3),
			Options);
	}

	MeshSurfaceDistanceReport MeshDistance::ComputeBoundaryDistance(
		const DynamicMesh& MeshA,
		const DynamicMesh& MeshB,
		const MeshSurfaceDistanceOptions& Options)
	{
		return ComputeSurfaceDistance(MeshA, MeshB, Options);
	}

	double MeshDistance::ComputeHausdorffDistance(
		const DynamicMesh& MeshA,
		const DynamicMesh& MeshB,
		const MeshSurfaceDistanceOptions& Options)
	{
		return ComputeSurfaceDistance(MeshA, MeshB, Options).HausdorffDistance;
	}

	MeshVolumeInfo MeshDistance::ComputeVolume(const DynamicMesh& Mesh, const MeshVolumeOptions& Options)
	{
		std::vector<Vector3> Vertices;
		std::vector<uint32_t> Indices;
		ExtractDynamicMesh(Mesh, Vertices, Indices);
		return ComputeVolume(
			Vertices.empty() ? nullptr : Vertices.data(), (uint32_t)Vertices.size(),
			Indices.empty() ? nullptr : Indices.data(), (uint32_t)(Indices.size() / 3),
			Options);
	}

	MeshVolumeDistanceReport MeshDistance::ComputeVolumeDistance(
		const DynamicMesh& MeshA,
		const DynamicMesh& MeshB,
		const MeshVolumeOptions& Options)
	{
		std::vector<Vector3> VerticesA;
		std::vector<uint32_t> IndicesA;
		std::vector<Vector3> VerticesB;
		std::vector<uint32_t> IndicesB;
		ExtractDynamicMesh(MeshA, VerticesA, IndicesA);
		ExtractDynamicMesh(MeshB, VerticesB, IndicesB);

		return ComputeVolumeDistance(
			VerticesA.empty() ? nullptr : VerticesA.data(), (uint32_t)VerticesA.size(),
			IndicesA.empty() ? nullptr : IndicesA.data(), (uint32_t)(IndicesA.size() / 3),
			VerticesB.empty() ? nullptr : VerticesB.data(), (uint32_t)VerticesB.size(),
			IndicesB.empty() ? nullptr : IndicesB.data(), (uint32_t)(IndicesB.size() / 3),
			Options);
	}

	void HausdorffDistance::setSamplesPerTriangleEdge(int SamplesPerEdge)
	{
		Options.SamplesPerTriangleEdge = std::max(1, SamplesPerEdge);
	}

	void HausdorffDistance::setTargetSampleCount(int SampleCount)
	{
		Options.TargetSampleCount = std::max(0, SampleCount);
	}

	float HausdorffDistance::computeOneSided(const StaticMesh* MeshA, const StaticMesh* MeshB)
	{
		if (MeshA == nullptr || MeshB == nullptr)
		{
			return 0.0f;
		}

		return (float)MeshDistance::ComputeOneSidedSurfaceDistance(*MeshA, *MeshB, Options).MaxDistance;
	}

	float HausdorffDistance::computeSymmetric(const StaticMesh* MeshA, const StaticMesh* MeshB)
	{
		if (MeshA == nullptr || MeshB == nullptr)
		{
			return 0.0f;
		}

		return (float)MeshDistance::ComputeSurfaceDistance(*MeshA, *MeshB, Options).HausdorffDistance;
	}

	float HausdorffDistance::computeWithKDTree(const StaticMesh* MeshA, const StaticMesh* MeshB)
	{
		return computeOneSided(MeshA, MeshB);
	}
}
