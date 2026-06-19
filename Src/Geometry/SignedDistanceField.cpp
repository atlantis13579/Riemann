#include "SignedDistanceField.h"

#include <algorithm>
#include <cmath>
#include <cfloat>
#include <cstdint>
#include <limits>
#include <vector>

#include "DynamicMesh.h"
#include "../CollisionPrimitive/StaticMesh.h"
#include "../CollisionPrimitive/Triangle3.h"
#include "../Maths/Maths.h"

namespace Riemann
{
	namespace
	{
		struct Mat3
		{
			float M[3][3];

			void LoadZero()
			{
				for (int R = 0; R < 3; ++R)
				{
					for (int C = 0; C < 3; ++C)
					{
						M[R][C] = 0.0f;
					}
				}
			}
		};

		struct SoupTriangle
		{
			Vector3 V[3];
			Vector3 Centroid = Vector3::Zero();
			Vector3 Normal = Vector3::Zero();
			float Area = 0.0f;
			Box3 Bounds = Box3::Empty();
		};

		struct SoupBVHNode
		{
			Box3 Bounds = Box3::Empty();
			int Left = -1;
			int Right = -1;
			int Start = 0;
			int Count = 0;
			Vector3 Center = Vector3::Zero();
			float Radius = 0.0f;
			Vector3 Order1 = Vector3::Zero();
			Mat3 Order2;

			bool IsLeaf() const
			{
				return Left < 0 && Right < 0;
			}
		};

		static bool IsFinite(const Vector3& V)
		{
			return std::isfinite(V.x) && std::isfinite(V.y) && std::isfinite(V.z);
		}

		static float TriangleAreaAndNormal(const Vector3& A, const Vector3& B, const Vector3& C, Vector3& Normal)
		{
			const Vector3 N = (C - A).Cross(B - A);
			const float Len = N.Length();
			if (Len <= std::numeric_limits<float>::epsilon())
			{
				Normal = Vector3::Zero();
				return 0.0f;
			}
			Normal = N / Len;
			return 0.5f * Len;
		}

		static float BoundsDistanceSqr(const Box3& Bounds, const Vector3& P)
		{
			float DistSqr = 0.0f;
			for (int Axis = 0; Axis < 3; ++Axis)
			{
				float V = P[Axis];
				if (V < Bounds.Min[Axis])
				{
					const float D = Bounds.Min[Axis] - V;
					DistSqr += D * D;
				}
				else if (V > Bounds.Max[Axis])
				{
					const float D = V - Bounds.Max[Axis];
					DistSqr += D * D;
				}
			}
			return DistSqr;
		}

		static void AddScaledOuterProduct(Mat3& Mat, const Vector3& A, const Vector3& B, float Scale)
		{
			Mat.M[0][0] += Scale * A.x * B.x;
			Mat.M[0][1] += Scale * A.x * B.y;
			Mat.M[0][2] += Scale * A.x * B.z;
			Mat.M[1][0] += Scale * A.y * B.x;
			Mat.M[1][1] += Scale * A.y * B.y;
			Mat.M[1][2] += Scale * A.y * B.z;
			Mat.M[2][0] += Scale * A.z * B.x;
			Mat.M[2][1] += Scale * A.z * B.y;
			Mat.M[2][2] += Scale * A.z * B.z;
		}

		static float InnerProduct(const Mat3& A, const Mat3& B)
		{
			float Sum = 0.0f;
			for (int R = 0; R < 3; ++R)
			{
				for (int C = 0; C < 3; ++C)
				{
					Sum += A.M[R][C] * B.M[R][C];
				}
			}
			return Sum;
		}

		static float EvaluateOrder2Approx(const SoupBVHNode& Node, const Vector3& Q)
		{
			const Vector3 DPQ = Node.Center - Q;
			const float Len = DPQ.Length();
			if (Len <= std::numeric_limits<float>::epsilon())
			{
				return 0.0f;
			}

			const float Len3 = Len * Len * Len;
			const float FourPiLen3 = 1.0f / (PI_4 * Len3);
			const float Order1 = FourPiLen3 * Node.Order1.Dot(DPQ);
			const float C = -3.0f / (PI_4 * Len3 * Len * Len);

			Mat3 Hessian;
			Hessian.M[0][0] = FourPiLen3 + C * DPQ.x * DPQ.x;
			Hessian.M[0][1] = C * DPQ.x * DPQ.y;
			Hessian.M[0][2] = C * DPQ.x * DPQ.z;
			Hessian.M[1][0] = C * DPQ.y * DPQ.x;
			Hessian.M[1][1] = FourPiLen3 + C * DPQ.y * DPQ.y;
			Hessian.M[1][2] = C * DPQ.y * DPQ.z;
			Hessian.M[2][0] = C * DPQ.z * DPQ.x;
			Hessian.M[2][1] = C * DPQ.z * DPQ.y;
			Hessian.M[2][2] = FourPiLen3 + C * DPQ.z * DPQ.z;

			return Order1 + InnerProduct(Node.Order2, Hessian);
		}

		static float TriangleSolidAngle(Vector3 A, Vector3 B, Vector3 C, const Vector3& P)
		{
			A -= P;
			B -= P;
			C -= P;
			const float LA = A.Length();
			const float LB = B.Length();
			const float LC = C.Length();
			if (LA <= std::numeric_limits<float>::epsilon() ||
				LB <= std::numeric_limits<float>::epsilon() ||
				LC <= std::numeric_limits<float>::epsilon())
			{
				return 0.0f;
			}

			const float Top = (LA * LB * LC) + A.Dot(B) * LC + B.Dot(C) * LA + C.Dot(A) * LB;
			const float Bottom = A.x * (B.y * C.z - C.y * B.z) -
				A.y * (B.x * C.z - C.x * B.z) +
				A.z * (B.x * C.y - C.x * B.y);
			return -2.0f * atan2f(Bottom, Top);
		}

		class SoupBVH
		{
		public:
			bool Build(const std::vector<SoupTriangle>& InTriangles, int InLeafTriangleCount,
				bool bInUseFastWinding, float InFastWindingBeta)
			{
				Triangles = &InTriangles;
				LeafTriangleCount = std::max(1, InLeafTriangleCount);
				bUseFastWinding = bInUseFastWinding;
				FastWindingBeta = std::max(1.0f, InFastWindingBeta);
				Nodes.clear();
				Indices.resize(InTriangles.size());
				for (size_t I = 0; I < InTriangles.size(); ++I)
				{
					Indices[I] = (int)I;
				}
				if (Indices.empty())
				{
					Root = -1;
					return false;
				}
				Root = BuildNode(0, (int)Indices.size());
				return Root >= 0;
			}

			float NearestDistanceSqr(const Vector3& P) const
			{
				if (Root < 0)
				{
					return std::numeric_limits<float>::max();
				}
				float Best = std::numeric_limits<float>::max();
				NearestDistanceSqr(Root, P, Best);
				return Best;
			}

			float WindingNumber(const Vector3& P) const
			{
				if (Root < 0)
				{
					return 0.0f;
				}
				return WindingNumber(Root, P);
			}

		private:
			int BuildNode(int Start, int Count)
			{
				const int NodeId = (int)Nodes.size();
				Nodes.push_back(SoupBVHNode());
				Nodes[NodeId].Start = Start;
				Nodes[NodeId].Count = Count;
				Nodes[NodeId].Order2.LoadZero();

				Nodes[NodeId].Bounds = (*Triangles)[Indices[Start]].Bounds;
				Vector3 CentroidMin = (*Triangles)[Indices[Start]].Centroid;
				Vector3 CentroidMax = CentroidMin;
				for (int I = Start; I < Start + Count; ++I)
				{
					const SoupTriangle& Tri = (*Triangles)[Indices[I]];
					Nodes[NodeId].Bounds.Encapsulate(Tri.Bounds);
					CentroidMin = CentroidMin.Min(Tri.Centroid);
					CentroidMax = CentroidMax.Max(Tri.Centroid);
				}

				if (Count > LeafTriangleCount)
				{
					const Vector3 CentroidExtent = CentroidMax - CentroidMin;
					const int Axis = CentroidExtent.LargestAxis();
					const int Mid = Start + Count / 2;
					std::nth_element(Indices.begin() + Start, Indices.begin() + Mid, Indices.begin() + Start + Count,
						[&](int A, int B)
					{
						return (*Triangles)[A].Centroid[Axis] < (*Triangles)[B].Centroid[Axis];
					});
					const int Left = BuildNode(Start, Mid - Start);
					const int Right = BuildNode(Mid, Start + Count - Mid);
					Nodes[NodeId].Left = Left;
					Nodes[NodeId].Right = Right;
					Nodes[NodeId].Count = 0;
				}

				ComputeCoefficients(Nodes[NodeId], Start, Count);
				return NodeId;
			}

			void ComputeCoefficients(SoupBVHNode& Node, int Start, int Count) const
			{
				float AreaSum = 0.0f;
				Vector3 Center = Vector3::Zero();
				for (int I = Start; I < Start + Count; ++I)
				{
					const SoupTriangle& Tri = (*Triangles)[Indices[I]];
					AreaSum += Tri.Area;
					Center += Tri.Area * Tri.Centroid;
				}

				if (AreaSum > std::numeric_limits<float>::epsilon())
				{
					Center /= AreaSum;
				}
				else
				{
					Center = Node.Bounds.GetCenter();
				}

				Node.Center = Center;
				Node.Radius = 0.0f;
				Node.Order1 = Vector3::Zero();
				Node.Order2.LoadZero();

				float RadiusSqr = 0.0f;
				for (int I = Start; I < Start + Count; ++I)
				{
					const SoupTriangle& Tri = (*Triangles)[Indices[I]];
					Node.Order1 += Tri.Area * Tri.Normal;
					AddScaledOuterProduct(Node.Order2, Tri.Centroid - Center, Tri.Normal, Tri.Area);
					for (int K = 0; K < 3; ++K)
					{
						RadiusSqr = std::max(RadiusSqr, (Tri.V[K] - Center).SquareLength());
					}
				}
				Node.Radius = std::sqrt(RadiusSqr);
			}

			void NearestDistanceSqr(int NodeId, const Vector3& P, float& Best) const
			{
				const SoupBVHNode& Node = Nodes[NodeId];
				if (BoundsDistanceSqr(Node.Bounds, P) > Best)
				{
					return;
				}

				if (Node.IsLeaf())
				{
					for (int I = Node.Start; I < Node.Start + Node.Count; ++I)
					{
						const SoupTriangle& Tri = (*Triangles)[Indices[I]];
						Best = std::min(Best, Triangle3::SqrDistancePointToTriangle(P, Tri.V[0], Tri.V[1], Tri.V[2]));
					}
					return;
				}

				const float DistLeft = BoundsDistanceSqr(Nodes[Node.Left].Bounds, P);
				const float DistRight = BoundsDistanceSqr(Nodes[Node.Right].Bounds, P);
				if (DistLeft < DistRight)
				{
					if (DistLeft <= Best)
					{
						NearestDistanceSqr(Node.Left, P, Best);
					}
					if (DistRight <= Best)
					{
						NearestDistanceSqr(Node.Right, P, Best);
					}
				}
				else
				{
					if (DistRight <= Best)
					{
						NearestDistanceSqr(Node.Right, P, Best);
					}
					if (DistLeft <= Best)
					{
						NearestDistanceSqr(Node.Left, P, Best);
					}
				}
			}

			float WindingNumber(int NodeId, const Vector3& P) const
			{
				const SoupBVHNode& Node = Nodes[NodeId];
				if (bUseFastWinding && !Node.IsLeaf() && CanUseFastWinding(Node, P))
				{
					return EvaluateOrder2Approx(Node, P);
				}

				if (Node.IsLeaf())
				{
					float Sum = 0.0f;
					for (int I = Node.Start; I < Node.Start + Node.Count; ++I)
					{
						const SoupTriangle& Tri = (*Triangles)[Indices[I]];
						Sum += TriangleSolidAngle(Tri.V[0], Tri.V[1], Tri.V[2], P) / PI_4;
					}
					return Sum;
				}

				return WindingNumber(Node.Left, P) + WindingNumber(Node.Right, P);
			}

			bool CanUseFastWinding(const SoupBVHNode& Node, const Vector3& P) const
			{
				if (Node.Radius <= std::numeric_limits<float>::epsilon() || Node.Bounds.IsInside(P))
				{
					return false;
				}
				return (Node.Center - P).Length() > FastWindingBeta * Node.Radius;
			}

		private:
			const std::vector<SoupTriangle>* Triangles = nullptr;
			std::vector<int> Indices;
			std::vector<SoupBVHNode> Nodes;
			int Root = -1;
			int LeafTriangleCount = 16;
			bool bUseFastWinding = true;
			float FastWindingBeta = 2.0f;
		};

		static bool BuildSoupTriangles(const Vector3* Vertices, uint32_t NumVertices,
			const uint32_t* Indices, uint32_t NumTriangles,
			std::vector<SoupTriangle>& OutTriangles, Box3& OutBounds)
		{
			OutTriangles.clear();
			OutTriangles.reserve(NumTriangles);
			bool bHasBounds = false;

			for (uint32_t T = 0; T < NumTriangles; ++T)
			{
				const uint32_t I0 = Indices[3 * T + 0];
				const uint32_t I1 = Indices[3 * T + 1];
				const uint32_t I2 = Indices[3 * T + 2];
				if (I0 >= NumVertices || I1 >= NumVertices || I2 >= NumVertices)
				{
					continue;
				}

				const Vector3 A = Vertices[I0];
				const Vector3 B = Vertices[I1];
				const Vector3 C = Vertices[I2];
				if (!IsFinite(A) || !IsFinite(B) || !IsFinite(C))
				{
					continue;
				}

				Vector3 Normal = Vector3::Zero();
				const float Area = TriangleAreaAndNormal(A, B, C, Normal);
				if (Area <= std::numeric_limits<float>::epsilon())
				{
					continue;
				}

				SoupTriangle Tri;
				Tri.V[0] = A;
				Tri.V[1] = B;
				Tri.V[2] = C;
				Tri.Centroid = (A + B + C) / 3.0f;
				Tri.Normal = Normal;
				Tri.Area = Area;
				Tri.Bounds = Box3(A, B, C);
				OutTriangles.push_back(Tri);

				if (!bHasBounds)
				{
					OutBounds = Tri.Bounds;
					bHasBounds = true;
				}
				else
				{
					OutBounds.Encapsulate(Tri.Bounds);
				}
			}

			return !OutTriangles.empty();
		}

		static int CeilToInt(float Value)
		{
			return (int)std::ceil(Value);
		}

		static float Max3(float A, float B, float C)
		{
			return std::max(A, std::max(B, C));
		}

		static void BuildRawDynamicMesh(const std::vector<Vector3>& Vertices, const std::vector<Index3>& Triangles,
			DynamicMesh& OutMesh)
		{
			OutMesh.Clear();
			for (const Vector3& Vertex : Vertices)
			{
				OutMesh.AppendVertex(Vertex);
			}
			for (const Index3& Tri : Triangles)
			{
				OutMesh.AppendTriangle(Tri);
			}
			OutMesh.BuildBounds();
		}

	}

	void SignedDistanceField3::Clear()
	{
		Bounds = Box3::Empty();
		CellsX = CellsY = CellsZ = 0;
		CellSize = 0.0f;
		Values.clear();
	}

	bool SignedDistanceField3::Reset(const Box3& InBounds, int InCellsX, int InCellsY, int InCellsZ, float InCellSize)
	{
		Clear();
		if (InCellsX <= 0 || InCellsY <= 0 || InCellsZ <= 0 || InCellSize <= 0.0f)
		{
			return false;
		}
		Bounds = InBounds;
		CellsX = InCellsX;
		CellsY = InCellsY;
		CellsZ = InCellsZ;
		CellSize = InCellSize;
		const uint64_t Count = (uint64_t)(CellsX + 1) * (uint64_t)(CellsY + 1) * (uint64_t)(CellsZ + 1);
		if (Count > (uint64_t)std::numeric_limits<int>::max())
		{
			Clear();
			return false;
		}
		Values.assign((size_t)Count, 0.0f);
		return true;
	}

	bool SignedDistanceField3::IsValid() const
	{
		return CellsX > 0 && CellsY > 0 && CellsZ > 0 && CellSize > 0.0f && !Values.empty();
	}

	int SignedDistanceField3::ValueIndex(int X, int Y, int Z) const
	{
		return (Z * (CellsY + 1) + Y) * (CellsX + 1) + X;
	}

	float& SignedDistanceField3::At(int X, int Y, int Z)
	{
		return Values[ValueIndex(X, Y, Z)];
	}

	float SignedDistanceField3::At(int X, int Y, int Z) const
	{
		return Values[ValueIndex(X, Y, Z)];
	}

	Vector3 SignedDistanceField3::GridPointToWorld(int X, int Y, int Z) const
	{
		return Bounds.Min + Vector3((float)X * CellSize, (float)Y * CellSize, (float)Z * CellSize);
	}

	bool SignedDistanceField3::BuildFromTriangleSoup(const Vector3* Vertices, uint32_t NumVertices,
		const uint32_t* Indices, uint32_t NumTriangles,
		SignedDistanceField3& OutField,
		const SignedDistanceFieldBuildOptions& Options,
		SignedDistanceFieldBuildReport* Report)
	{
		SignedDistanceFieldBuildReport LocalReport;
		OutField.Clear();

		if (Vertices == nullptr || (Indices == nullptr && NumTriangles > 0) || NumVertices == 0 || NumTriangles == 0)
		{
			if (Report != nullptr)
			{
				*Report = LocalReport;
			}
			return false;
		}

		std::vector<SoupTriangle> SoupTriangles;
		Box3 InputBounds = Box3::Empty();
		if (!BuildSoupTriangles(Vertices, NumVertices, Indices, NumTriangles, SoupTriangles, InputBounds))
		{
			if (Report != nullptr)
			{
				*Report = LocalReport;
			}
			return false;
		}

		float CellSize = Options.VoxelSize;
		const float MaxDim = std::max(InputBounds.MaxDim(), 1e-6f);
		if (CellSize <= 0.0f)
		{
			const int Resolution = std::max(1, Options.Resolution);
			CellSize = MaxDim / (float)Resolution;
		}
		CellSize = std::max(CellSize, 1e-6f);

		Box3 GridBounds = InputBounds;
		GridBounds.Thicken(std::max(0, Options.PaddingCells) * CellSize);
		Vector3 GridSize = GridBounds.GetSize();

		const int MaxResolution = Options.MaxResolution > 0 ? Options.MaxResolution : std::numeric_limits<int>::max();
		int CellsX = std::max(1, CeilToInt(GridSize.x / CellSize));
		int CellsY = std::max(1, CeilToInt(GridSize.y / CellSize));
		int CellsZ = std::max(1, CeilToInt(GridSize.z / CellSize));
		const int CurrentMax = std::max(CellsX, std::max(CellsY, CellsZ));
		if (CurrentMax > MaxResolution)
		{
			CellSize = Max3(GridSize.x, GridSize.y, GridSize.z) / (float)MaxResolution;
			CellSize = std::max(CellSize, 1e-6f);
			CellsX = std::max(1, CeilToInt(GridSize.x / CellSize));
			CellsY = std::max(1, CeilToInt(GridSize.y / CellSize));
			CellsZ = std::max(1, CeilToInt(GridSize.z / CellSize));
		}

		GridBounds.Max = GridBounds.Min + Vector3(CellsX * CellSize, CellsY * CellSize, CellsZ * CellSize);
		if (!OutField.Reset(GridBounds, CellsX, CellsY, CellsZ, CellSize))
		{
			if (Report != nullptr)
			{
				*Report = LocalReport;
			}
			return false;
		}

		SoupBVH BVH;
		if (!BVH.Build(SoupTriangles, Options.LeafTriangleCount, Options.UseFastWinding, Options.FastWindingBeta))
		{
			if (Report != nullptr)
			{
				*Report = LocalReport;
			}
			return false;
		}

		float MinValue = std::numeric_limits<float>::max();
		float MaxValue = -std::numeric_limits<float>::max();
		for (int Z = 0; Z <= CellsZ; ++Z)
		{
			for (int Y = 0; Y <= CellsY; ++Y)
			{
				for (int X = 0; X <= CellsX; ++X)
				{
					const Vector3 P = OutField.GridPointToWorld(X, Y, Z);
					const float Distance = std::sqrt(std::max(0.0f, BVH.NearestDistanceSqr(P)));
					const float Winding = BVH.WindingNumber(P);
					const float InsideValue = Options.UseAbsoluteWinding ? std::abs(Winding) : Winding;
					const float Value = InsideValue > Options.WindingThreshold ? -Distance : Distance;
					OutField.At(X, Y, Z) = Value;
					MinValue = std::min(MinValue, Value);
					MaxValue = std::max(MaxValue, Value);
				}
			}
		}

		LocalReport.InputBounds = InputBounds;
		LocalReport.GridBounds = GridBounds;
		LocalReport.CellSize = CellSize;
		LocalReport.CellsX = CellsX;
		LocalReport.CellsY = CellsY;
		LocalReport.CellsZ = CellsZ;
		LocalReport.SampleCount = OutField.GetSampleCount();
		LocalReport.MinValue = MinValue;
		LocalReport.MaxValue = MaxValue;
		if (Report != nullptr)
		{
			*Report = LocalReport;
		}
		return true;
	}

	bool SignedDistanceField3::BuildFromTriangleSoup(const std::vector<Vector3>& Vertices,
		const std::vector<Index3>& Triangles,
		SignedDistanceField3& OutField,
		const SignedDistanceFieldBuildOptions& Options,
		SignedDistanceFieldBuildReport* Report)
	{
		std::vector<uint32_t> Indices;
		Indices.reserve(Triangles.size() * 3);
		for (const Index3& Tri : Triangles)
		{
			Indices.push_back((uint32_t)Tri.a);
			Indices.push_back((uint32_t)Tri.b);
			Indices.push_back((uint32_t)Tri.c);
		}
		return BuildFromTriangleSoup(Vertices.data(), (uint32_t)Vertices.size(), Indices.data(), (uint32_t)Triangles.size(),
			OutField, Options, Report);
	}

	bool SignedDistanceField3::ExtractIsoSurface(float IsoValue,
		std::vector<Vector3>& OutVertices, std::vector<Index3>& OutTriangles) const
	{
		OutVertices.clear();
		OutTriangles.clear();
		if (!IsValid())
		{
			return false;
		}

		static const int CornerOffset[8][3] =
		{
			{ 0, 0, 0 }, { 1, 0, 0 }, { 0, 1, 0 }, { 1, 1, 0 },
			{ 0, 0, 1 }, { 1, 0, 1 }, { 0, 1, 1 }, { 1, 1, 1 }
		};
		static const int EdgeCorners[12][2] =
		{
			{ 0, 1 }, { 0, 2 }, { 1, 3 }, { 2, 3 },
			{ 4, 5 }, { 4, 6 }, { 5, 7 }, { 6, 7 },
			{ 0, 4 }, { 1, 5 }, { 2, 6 }, { 3, 7 }
		};

		auto CellIndex = [&](int X, int Y, int Z) -> int
		{
			return (Z * CellsY + Y) * CellsX + X;
		};

		auto AddTriangle = [&](int A, int B, int C)
		{
			if (A != B && B != C && C != A)
			{
				OutTriangles.push_back(Index3(A, B, C));
			}
		};

		auto AddQuad = [&](const std::vector<int>& CellVertices, int C0, int C1, int C2, int C3, bool bFlip)
		{
			if (C0 < 0 || C1 < 0 || C2 < 0 || C3 < 0)
			{
				return;
			}
			const int V0 = CellVertices[C0];
			const int V1 = CellVertices[C1];
			const int V2 = CellVertices[C2];
			const int V3 = CellVertices[C3];
			if (V0 < 0 || V1 < 0 || V2 < 0 || V3 < 0)
			{
				return;
			}
			if (bFlip)
			{
				AddTriangle(V0, V2, V1);
				AddTriangle(V0, V3, V2);
			}
			else
			{
				AddTriangle(V0, V1, V2);
				AddTriangle(V0, V2, V3);
			}
		};

		std::vector<int> CellVertices((size_t)CellsX * (size_t)CellsY * (size_t)CellsZ, -1);
		for (int Z = 0; Z < CellsZ; ++Z)
		{
			for (int Y = 0; Y < CellsY; ++Y)
			{
				for (int X = 0; X < CellsX; ++X)
				{
					Vector3 P[8];
					float V[8];
					bool bHasInside = false;
					bool bHasOutside = false;
					for (int C = 0; C < 8; ++C)
					{
						const int GX = X + CornerOffset[C][0];
						const int GY = Y + CornerOffset[C][1];
						const int GZ = Z + CornerOffset[C][2];
						P[C] = GridPointToWorld(GX, GY, GZ);
						V[C] = At(GX, GY, GZ);
						bHasInside = bHasInside || V[C] < IsoValue;
						bHasOutside = bHasOutside || V[C] >= IsoValue;
					}

					if (!bHasInside || !bHasOutside)
					{
						continue;
					}

					Vector3 Vertex = Vector3::Zero();
					int CrossingCount = 0;
					for (int E = 0; E < 12; ++E)
					{
						const int A = EdgeCorners[E][0];
						const int B = EdgeCorners[E][1];
						const bool bAInside = V[A] < IsoValue;
						const bool bBInside = V[B] < IsoValue;
						if (bAInside == bBInside)
						{
							continue;
						}

						float T = 0.5f;
						const float Denom = V[B] - V[A];
						if (std::abs(Denom) > std::numeric_limits<float>::epsilon())
						{
							T = (IsoValue - V[A]) / Denom;
							T = std::max(0.0f, std::min(1.0f, T));
						}
						Vertex += P[A] + (P[B] - P[A]) * T;
						CrossingCount++;
					}

					if (CrossingCount > 0)
					{
						Vertex /= (float)CrossingCount;
						const int NewVertex = (int)OutVertices.size();
						OutVertices.push_back(Vertex);
						CellVertices[CellIndex(X, Y, Z)] = NewVertex;
					}
				}
			}
		}

		for (int Z = 1; Z < CellsZ; ++Z)
		{
			for (int Y = 1; Y < CellsY; ++Y)
			{
				for (int X = 0; X < CellsX; ++X)
				{
					const bool bAInside = At(X, Y, Z) < IsoValue;
					const bool bBInside = At(X + 1, Y, Z) < IsoValue;
					if (bAInside != bBInside)
					{
						AddQuad(CellVertices,
							CellIndex(X, Y - 1, Z - 1),
							CellIndex(X, Y, Z - 1),
							CellIndex(X, Y, Z),
							CellIndex(X, Y - 1, Z),
							bAInside);
					}
				}
			}
		}

		for (int Z = 1; Z < CellsZ; ++Z)
		{
			for (int Y = 0; Y < CellsY; ++Y)
			{
				for (int X = 1; X < CellsX; ++X)
				{
					const bool bAInside = At(X, Y, Z) < IsoValue;
					const bool bBInside = At(X, Y + 1, Z) < IsoValue;
					if (bAInside != bBInside)
					{
						AddQuad(CellVertices,
							CellIndex(X - 1, Y, Z - 1),
							CellIndex(X, Y, Z - 1),
							CellIndex(X, Y, Z),
							CellIndex(X - 1, Y, Z),
							bAInside);
					}
				}
			}
		}

		for (int Z = 0; Z < CellsZ; ++Z)
		{
			for (int Y = 1; Y < CellsY; ++Y)
			{
				for (int X = 1; X < CellsX; ++X)
				{
					const bool bAInside = At(X, Y, Z) < IsoValue;
					const bool bBInside = At(X, Y, Z + 1) < IsoValue;
					if (bAInside != bBInside)
					{
						AddQuad(CellVertices,
							CellIndex(X - 1, Y - 1, Z),
							CellIndex(X, Y - 1, Z),
							CellIndex(X, Y, Z),
							CellIndex(X - 1, Y, Z),
							bAInside);
					}
				}
			}
		}

		return !OutTriangles.empty();
	}

	bool SignedDistanceField3::ExtractIsoSurface(float IsoValue, DynamicMesh& OutMesh,
		const MeshTopologyRepairOptions* RepairOptions,
		MeshTopologyRepairReport* RepairReport) const
	{
		std::vector<Vector3> Vertices;
		std::vector<Index3> Triangles;
		if (!ExtractIsoSurface(IsoValue, Vertices, Triangles))
		{
			OutMesh.Clear();
			return false;
		}

		MeshTopologyRepairOptions LocalOptions;
		if (RepairOptions != nullptr)
		{
			LocalOptions = *RepairOptions;
		}
		LocalOptions.WeldTolerance = std::max(LocalOptions.WeldTolerance, CellSize * 1e-5f);
		LocalOptions.AreaEpsilon = std::max(LocalOptions.AreaEpsilon, CellSize * CellSize * 1e-10f);
		LocalOptions.ValidateSelfIntersections = false;
		return MeshRepair::RepairTopology(Vertices, Triangles, OutMesh, LocalOptions, RepairReport);
	}

	bool MeshSDFReconstructor::ReconstructSolid(const Vector3* Vertices, uint32_t NumVertices,
		const uint32_t* Indices, uint32_t NumTriangles,
		DynamicMesh& OutMesh,
		const SignedDistanceFieldBuildOptions& Options,
		SignedDistanceFieldBuildReport* Report)
	{
		SignedDistanceFieldBuildReport LocalReport;
		SignedDistanceField3 Field;
		if (!SignedDistanceField3::BuildFromTriangleSoup(Vertices, NumVertices, Indices, NumTriangles, Field, Options, &LocalReport))
		{
			OutMesh.Clear();
			if (Report != nullptr)
			{
				*Report = LocalReport;
			}
			return false;
		}

		std::vector<Vector3> ExtractedVertices;
		std::vector<Index3> ExtractedTriangles;
		if (!Field.ExtractIsoSurface(0.0f, ExtractedVertices, ExtractedTriangles))
		{
			OutMesh.Clear();
			if (Report != nullptr)
			{
				*Report = LocalReport;
			}
			return false;
		}

		LocalReport.ExtractedVertexCount = (uint32_t)ExtractedVertices.size();
		LocalReport.ExtractedTriangleCount = (uint32_t)ExtractedTriangles.size();
		if (Options.RepairExtractedMesh)
		{
			MeshTopologyRepairOptions RepairOptions = Options.RepairOptions;
			RepairOptions.WeldTolerance = std::max(RepairOptions.WeldTolerance, Field.GetCellSize() * 1e-5f);
			RepairOptions.AreaEpsilon = std::max(RepairOptions.AreaEpsilon, Field.GetCellSize() * Field.GetCellSize() * 1e-10f);
			RepairOptions.ValidateSelfIntersections = false;
			MeshRepair::RepairTopology(ExtractedVertices, ExtractedTriangles, OutMesh, RepairOptions, &LocalReport.OutputRepair);
		}
		else
		{
			BuildRawDynamicMesh(ExtractedVertices, ExtractedTriangles, OutMesh);
		}

		MeshValidationOptions ValidationOptions;
		ValidationOptions.WeldTolerance = std::max(Options.RepairOptions.WeldTolerance, Field.GetCellSize() * 1e-5f);
		ValidationOptions.AreaEpsilon = std::max(Options.RepairOptions.AreaEpsilon, Field.GetCellSize() * Field.GetCellSize() * 1e-10f);
		LocalReport.OutputValidation = MeshValidator::Validate(OutMesh, ValidationOptions);
		if (Report != nullptr)
		{
			*Report = LocalReport;
		}
		return OutMesh.GetTriangleCount() > 0;
	}

	bool MeshSDFReconstructor::ReconstructSolid(const std::vector<Vector3>& Vertices,
		const std::vector<Index3>& Triangles,
		DynamicMesh& OutMesh,
		const SignedDistanceFieldBuildOptions& Options,
		SignedDistanceFieldBuildReport* Report)
	{
		std::vector<uint32_t> Indices;
		Indices.reserve(Triangles.size() * 3);
		for (const Index3& Tri : Triangles)
		{
			Indices.push_back((uint32_t)Tri.a);
			Indices.push_back((uint32_t)Tri.b);
			Indices.push_back((uint32_t)Tri.c);
		}
		return ReconstructSolid(Vertices.data(), (uint32_t)Vertices.size(), Indices.data(), (uint32_t)Triangles.size(),
			OutMesh, Options, Report);
	}

	bool MeshSDFReconstructor::ReconstructSolid(const StaticMesh& Mesh,
		DynamicMesh& OutMesh,
		const SignedDistanceFieldBuildOptions& Options,
		SignedDistanceFieldBuildReport* Report)
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
		return ReconstructSolid(Mesh.Vertices, Mesh.GetVertexCount(), Indices.data(), Mesh.GetTriangleCount(),
			OutMesh, Options, Report);
	}

	bool MeshSDFReconstructor::ReconstructSolid(const DynamicMesh& Mesh,
		DynamicMesh& OutMesh,
		const SignedDistanceFieldBuildOptions& Options,
		SignedDistanceFieldBuildReport* Report)
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

		return ReconstructSolid(Vertices, Triangles, OutMesh, Options, Report);
	}
}
