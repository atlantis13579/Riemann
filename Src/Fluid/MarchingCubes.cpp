#include "MarchingCubes.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace Riemann
{
	namespace
	{
		const int CornerOffset[8][3] =
		{
			{ 0, 0, 0 }, { 1, 0, 0 }, { 1, 0, 1 }, { 0, 0, 1 },
			{ 0, 1, 0 }, { 1, 1, 0 }, { 1, 1, 1 }, { 0, 1, 1 },
		};

		const int EdgeCorners[12][2] =
		{
			{ 0, 1 }, { 1, 2 }, { 2, 3 }, { 3, 0 },
			{ 4, 5 }, { 5, 6 }, { 6, 7 }, { 7, 4 },
			{ 0, 4 }, { 1, 5 }, { 2, 6 }, { 3, 7 },
		};

		// Each nibble is an edge number; F terminates the row. Packing keeps the
		// standard 256 by 16 marching-cubes topology table compact and immutable.
		const char PackedTriangleTable[] =
			"FFFFFFFFFFFFFFFF083FFFFFFFFFFFFF019FFFFFFFFFFFFF183981FFFFFFFFFF12AFFFFFFFFFFFFF08312AFFFFFFFFFF92A029FFFFFFFFFF2832A8A98FFFFFFF"
			"3B2FFFFFFFFFFFFF0B28B0FFFFFFFFFF19023BFFFFFFFFFF1B219B98BFFFFFFF3A1BA3FFFFFFFFFF0A108A8BAFFFFFFF3903B9BA9FFFFFFF98AA8BFFFFFFFFFF"
			"478FFFFFFFFFFFFF430734FFFFFFFFFF019847FFFFFFFFFF419471731FFFFFFF12A847FFFFFFFFFF34730412AFFFFFFF92A902847FFFFFFF2A9297273794FFFF"
			"8473B2FFFFFFFFFFB47B24204FFFFFFF90184723BFFFFFFF47B94B9B2921FFFF3A13BA784FFFFFFF1BA14B1047B4FFFF47890B9BAB03FFFF47B4B99BAFFFFFFF"
			"954FFFFFFFFFFFFF954083FFFFFFFFFF054150FFFFFFFFFF854835315FFFFFFF12A954FFFFFFFFFF30812A495FFFFFFF52A542402FFFFFFF2A5325354348FFFF"
			"95423BFFFFFFFFFF0B208B495FFFFFFF05401523BFFFFFFF21525828B485FFFFA3BA13954FFFFFFF4950818A18BAFFFF54050B5BAB03FFFF54858AA8BFFFFFFF"
			"978579FFFFFFFFFF930953573FFFFFFF078017157FFFFFFF153357FFFFFFFFFF978957A12FFFFFFFA12950530573FFFF802825857A52FFFF2A5253357FFFFFFF"
			"7957893B2FFFFFFF95797292027BFFFF23B018178157FFFFB21B17715FFFFFFF958857A13A3BFFFF5705097B010ABA0FBA0B03A50807570FBA57B5FFFFFFFFFF"
			"A65FFFFFFFFFFFFF0835A6FFFFFFFFFF9015A6FFFFFFFFFF1831985A6FFFFFFF165261FFFFFFFFFF165126308FFFFFFF965906026FFFFFFF598582526328FFFF"
			"23BA65FFFFFFFFFFB08B20A65FFFFFFF01923B5A6FFFFFFF5A61929B298BFFFF63B653513FFFFFFF08B0B50515B6FFFF3B6036065059FFFF65969BB98FFFFFFF"
			"5A6478FFFFFFFFFF43047365AFFFFFFF1905A6847FFFFFFFA65197173794FFFF612651478FFFFFFF125526304347FFFF847905065026FFFF739794329596269F"
			"3B2784A65FFFFFFF5A647242027BFFFF01947823B5A6FFFF9219B294B7B45A6F8473B53515B6FFFF51B5B610B7B404BF059065036B63847F65969B4797B9FFFF"
			"A4964AFFFFFFFFFF4A649A083FFFFFFFA01A60640FFFFFFF83181686461AFFFF149124264FFFFFFF308129249264FFFF024426FFFFFFFFFF832824426FFFFFFF"
			"A49A64B23FFFFFFF08228B49A4A6FFFF3B201606461AFFFF64161A48121B8B1F964936913B63FFFF8B1810B61914641F3B6360064FFFFFFF648B68FFFFFFFFFF"
			"7A678A89AFFFFFFF0730A709A67AFFFFA671A7178180FFFFA67A71173FFFFFFF126168189867FFFF269291679093739F780706602FFFFFFF732672FFFFFFFFFF"
			"23BA68A89867FFFF20727B09767A9A7F1801781A767A23BFB21B17A61671FFFF896867916B63136F091B67FFFFFFFFFF7807063B0B60FFFF7B6FFFFFFFFFFFFF"
			"76BFFFFFFFFFFFFF308B76FFFFFFFFFF019B76FFFFFFFFFF819831B76FFFFFFFA126B7FFFFFFFFFF12A3086B7FFFFFFF2902A96B7FFFFFFF6B72A3A83A98FFFF"
			"723627FFFFFFFFFF708760620FFFFFFF276237019FFFFFFF162186198876FFFFA76A17137FFFFFFFA7617A187108FFFF03707A0A96A7FFFF76A7A88A9FFFFFFF"
			"684B86FFFFFFFFFF36B306046FFFFFFF86B846901FFFFFFF946963931B36FFFF6846B82A1FFFFFFF12A30B06B046FFFF4B846B0292A9FFFFA93A32943B36463F"
			"823842462FFFFFFF042462FFFFFFFFFF190234246438FFFF194142246FFFFFFF8138618466A1FFFFA10A06604FFFFFFF4634386A3039A93FA946A4FFFFFFFFFF"
			"49576BFFFFFFFFFF083495B76FFFFFFF50154076BFFFFFFFB76834354315FFFF954A1276BFFFFFFF6B712A083495FFFF76B54A42A402FFFF348354325A52B76F"
			"723762549FFFFFFF954086062687FFFF362376150540FFFF628687218485158F954A16176137FFFF16A176107870954F40A4A503A6A737AF76A7A854A48AFFFF"
			"6956B9B89FFFFFFF36B063056095FFFF0B805B01556BFFFF6B3635531FFFFFFF12A95B9B8B56FFFF0B306B09656912AFB85B56805A52025F6B36352A3A53FFFF"
			"589528562382FFFF956960062FFFFFFF158180568382628F156216FFFFFFFFFF13616A386569896FA10A06950560FFFF03856AFFFFFFFFFFA56FFFFFFFFFFFFF"
			"B5A75BFFFFFFFFFFB5AB75830FFFFFFF5B75AB190FFFFFFFA75AB7981831FFFFB12B71751FFFFFFF08312717572BFFFF9759279022B7FFFF75272B592328982F"
			"25A235375FFFFFFF820852875A25FFFF9015A35373A2FFFF982921872A25752F135375FFFFFFFFFF087071175FFFFFFF903935537FFFFFFF987597FFFFFFFFFF"
			"5845A8AB8FFFFFFF5045B05ABB30FFFF01984A8ABA45FFFFAB4A45B34941314F2512852B8458FFFF04B0B345B2B151BF0250592B5458B85F9452B3FFFFFFFFFF"
			"25A352345384FFFF5A2524420FFFFFFF3A235A385458019F5A2524192942FFFF845853351FFFFFFF045105FFFFFFFFFF845853905035FFFF945FFFFFFFFFFFFF"
			"4B749B9ABFFFFFFF0834979B79ABFFFF1AB1B414074BFFFF3143481A474BAB4F4B79B492B912FFFF9749B791B2B1083FB74B42240FFFFFFFB74B42834324FFFF"
			"29A279237749FFFF9A7974A27870207F37A3A274A1A040AF1A2874FFFFFFFFFF491417713FFFFFFF491417081871FFFF403743FFFFFFFFFF487FFFFFFFFFFFFF"
			"9A8AB8FFFFFFFFFF30939BB9AFFFFFFF01A0A88ABFFFFFFF31AB3AFFFFFFFFFF12B1B99B8FFFFFFF30939B1292B9FFFF02B80BFFFFFFFFFF32BFFFFFFFFFFFFF"
			"23828AA89FFFFFFF9A2092FFFFFFFFFF23828A0181A8FFFF1A2FFFFFFFFFFFFF138918FFFFFFFFFF091FFFFFFFFFFFFF038FFFFFFFFFFFFFFFFFFFFFFFFFFFFF";
		static_assert(sizeof(PackedTriangleTable) == 4097, "Invalid marching-cubes topology table");

		int DecodeTriangleEdge(int CaseIndex, int EntryIndex)
		{
			const char Value = PackedTriangleTable[CaseIndex * 16 + EntryIndex];
			if (Value == 'F') return -1;
			return Value <= '9' ? Value - '0' : Value - 'A' + 10;
		}

		bool IsFiniteVector(const Vector3& Value)
		{
			return std::isfinite(Value.x) && std::isfinite(Value.y) && std::isfinite(Value.z);
		}
	}

	void FluidSurfaceMesh::Clear()
	{
		Positions.clear();
		Normals.clear();
		Indices.clear();
	}

	MarchingCubesInput::MarchingCubesInput()
		: Origin(Vector3::Zero())
		, Spacing(Vector3::One())
		, Resolution(0, 0, 0)
		, Values(nullptr)
		, ValueCount(0)
		, IsoValue(0.0f)
		, HighValuesInside(true)
	{
	}

	bool MarchingCubes::Extract(const MarchingCubesInput& Input, FluidSurfaceMesh* Mesh)
	{
		if (Mesh == nullptr) return false;
		Mesh->Clear();
		if (Input.Values == nullptr || !IsFiniteVector(Input.Origin) || !IsFiniteVector(Input.Spacing)
			|| !std::isfinite(Input.IsoValue) || Input.Spacing.x <= 0.0f
			|| Input.Spacing.y <= 0.0f || Input.Spacing.z <= 0.0f
			|| Input.Resolution.x < 2 || Input.Resolution.y < 2 || Input.Resolution.z < 2)
		{
			return false;
		}

		const size_t NX = (size_t)Input.Resolution.x;
		const size_t NY = (size_t)Input.Resolution.y;
		const size_t NZ = (size_t)Input.Resolution.z;
		if (NX > std::numeric_limits<size_t>::max() / NY
			|| NX * NY > std::numeric_limits<size_t>::max() / NZ)
		{
			return false;
		}
		const size_t SampleCount = NX * NY * NZ;
		// This includes the ghost layer around every grid accepted by FluidSimulation.
		const size_t MaxSampleCount = (size_t)32 * 1024 * 1024;
		if (SampleCount > MaxSampleCount) return false;
		if (Input.ValueCount < SampleCount) return false;
		for (size_t Index = 0; Index < SampleCount; ++Index)
		{
			if (!std::isfinite(Input.Values[Index])) return false;
		}
		const double FloatMaximum = (double)std::numeric_limits<float>::max();
		const double FarX = (double)Input.Origin.x + (double)(NX - 1) * (double)Input.Spacing.x;
		const double FarY = (double)Input.Origin.y + (double)(NY - 1) * (double)Input.Spacing.y;
		const double FarZ = (double)Input.Origin.z + (double)(NZ - 1) * (double)Input.Spacing.z;
		if (!std::isfinite(FarX) || !std::isfinite(FarY) || !std::isfinite(FarZ)
			|| std::fabs(FarX) > FloatMaximum || std::fabs(FarY) > FloatMaximum
			|| std::fabs(FarZ) > FloatMaximum)
		{
			return false;
		}

		auto SampleIndex = [NX, NY](int X, int Y, int Z) -> size_t
		{
			return (size_t)X + NX * ((size_t)Y + NY * (size_t)Z);
		};
		std::vector<Vector3> Gradients(SampleCount, Vector3::Zero());
		for (int Z = 0; Z < Input.Resolution.z; ++Z)
		for (int Y = 0; Y < Input.Resolution.y; ++Y)
		for (int X = 0; X < Input.Resolution.x; ++X)
		{
			const int X0 = std::max(0, X - 1), X1 = std::min(Input.Resolution.x - 1, X + 1);
			const int Y0 = std::max(0, Y - 1), Y1 = std::min(Input.Resolution.y - 1, Y + 1);
			const int Z0 = std::max(0, Z - 1), Z1 = std::min(Input.Resolution.z - 1, Z + 1);
			const double GradientX = ((double)Input.Values[SampleIndex(X1, Y, Z)]
				- (double)Input.Values[SampleIndex(X0, Y, Z)])
				/ ((double)(X1 - X0) * (double)Input.Spacing.x);
			const double GradientY = ((double)Input.Values[SampleIndex(X, Y1, Z)]
				- (double)Input.Values[SampleIndex(X, Y0, Z)])
				/ ((double)(Y1 - Y0) * (double)Input.Spacing.y);
			const double GradientZ = ((double)Input.Values[SampleIndex(X, Y, Z1)]
				- (double)Input.Values[SampleIndex(X, Y, Z0)])
				/ ((double)(Z1 - Z0) * (double)Input.Spacing.z);
			if (!std::isfinite(GradientX) || !std::isfinite(GradientY) || !std::isfinite(GradientZ)
				|| std::fabs(GradientX) > FloatMaximum || std::fabs(GradientY) > FloatMaximum
				|| std::fabs(GradientZ) > FloatMaximum)
			{
				Mesh->Clear();
				return false;
			}
			Gradients[SampleIndex(X, Y, Z)] = Vector3(
				(float)GradientX, (float)GradientY, (float)GradientZ);
		}

		const size_t XEdgeCount = (NX - 1) * NY * NZ;
		const size_t YEdgeCount = NX * (NY - 1) * NZ;
		const size_t ZEdgeCount = NX * NY * (NZ - 1);
		std::vector<int32_t> XEdges(XEdgeCount, -1);
		std::vector<int32_t> YEdges(YEdgeCount, -1);
		std::vector<int32_t> ZEdges(ZEdgeCount, -1);
		std::vector<Vector3> FaceNormals;

		auto EdgeVertex = [&](int CellX, int CellY, int CellZ, int Edge) -> int32_t
		{
			const int CornerA = EdgeCorners[Edge][0], CornerB = EdgeCorners[Edge][1];
			int AX = CellX + CornerOffset[CornerA][0];
			int AY = CellY + CornerOffset[CornerA][1];
			int AZ = CellZ + CornerOffset[CornerA][2];
			int BX = CellX + CornerOffset[CornerB][0];
			int BY = CellY + CornerOffset[CornerB][1];
			int BZ = CellZ + CornerOffset[CornerB][2];
			const int Axis = AX != BX ? 0 : (AY != BY ? 1 : 2);
			if (BX < AX || BY < AY || BZ < AZ)
			{
				std::swap(AX, BX); std::swap(AY, BY); std::swap(AZ, BZ);
			}
			int32_t* Cached = nullptr;
			if (Axis == 0) Cached = &XEdges[(size_t)AX + (NX - 1) * ((size_t)AY + NY * (size_t)AZ)];
			if (Axis == 1) Cached = &YEdges[(size_t)AX + NX * ((size_t)AY + (NY - 1) * (size_t)AZ)];
			if (Axis == 2) Cached = &ZEdges[(size_t)AX + NX * ((size_t)AY + NY * (size_t)AZ)];
			if (*Cached >= 0) return *Cached;

			const size_t IndexA = SampleIndex(AX, AY, AZ), IndexB = SampleIndex(BX, BY, BZ);
			const double ValueA = (double)Input.Values[IndexA], ValueB = (double)Input.Values[IndexB];
			const double Denominator = ValueB - ValueA;
			double T = std::fabs(Denominator) > (double)std::numeric_limits<float>::epsilon()
				? ((double)Input.IsoValue - ValueA) / Denominator : 0.5;
			T = std::max(1.0e-4, std::min(1.0 - 1.0e-4, T));
			const double GridX = (double)AX + (double)(BX - AX) * T;
			const double GridY = (double)AY + (double)(BY - AY) * T;
			const double GridZ = (double)AZ + (double)(BZ - AZ) * T;
			Mesh->Positions.push_back(Vector3(
				(float)((double)Input.Origin.x + GridX * (double)Input.Spacing.x),
				(float)((double)Input.Origin.y + GridY * (double)Input.Spacing.y),
				(float)((double)Input.Origin.z + GridZ * (double)Input.Spacing.z)));
			const double NormalSign = Input.HighValuesInside ? -1.0 : 1.0;
			const double NormalX = NormalSign * ((double)Gradients[IndexA].x
				+ ((double)Gradients[IndexB].x - (double)Gradients[IndexA].x) * T);
			const double NormalY = NormalSign * ((double)Gradients[IndexA].y
				+ ((double)Gradients[IndexB].y - (double)Gradients[IndexA].y) * T);
			const double NormalZ = NormalSign * ((double)Gradients[IndexA].z
				+ ((double)Gradients[IndexB].z - (double)Gradients[IndexA].z) * T);
			const double NormalLengthSquared = NormalX * NormalX + NormalY * NormalY + NormalZ * NormalZ;
			const double NormalEpsilon = (double)std::numeric_limits<float>::epsilon();
			Vector3 Normal = Vector3::Zero();
			if (NormalLengthSquared > NormalEpsilon * NormalEpsilon
				&& std::isfinite(NormalLengthSquared))
			{
				const double InverseLength = 1.0 / std::sqrt(NormalLengthSquared);
				Normal = Vector3((float)(NormalX * InverseLength), (float)(NormalY * InverseLength),
					(float)(NormalZ * InverseLength));
			}
			Mesh->Normals.push_back(Normal);
			FaceNormals.push_back(Vector3::Zero());
			*Cached = (int32_t)Mesh->Positions.size() - 1;
			return *Cached;
		};

		const double MaxSpacing = (double)std::max(Input.Spacing.x,
			std::max(Input.Spacing.y, Input.Spacing.z));
		const double SpacingSquared = MaxSpacing * MaxSpacing;
		const double MinimumAreaSquared = 1.0e-30 * SpacingSquared * SpacingSquared;
		if (!std::isfinite(MinimumAreaSquared)) return false;
		for (int Z = 0; Z + 1 < Input.Resolution.z; ++Z)
		for (int Y = 0; Y + 1 < Input.Resolution.y; ++Y)
		for (int X = 0; X + 1 < Input.Resolution.x; ++X)
		{
			int CaseIndex = 0;
			for (int Corner = 0; Corner < 8; ++Corner)
			{
				const float Value = Input.Values[SampleIndex(X + CornerOffset[Corner][0],
					Y + CornerOffset[Corner][1], Z + CornerOffset[Corner][2])];
				const bool Inside = Input.HighValuesInside ? Value >= Input.IsoValue : Value < Input.IsoValue;
				if (Inside) CaseIndex |= 1 << Corner;
			}
			for (int Entry = 0; Entry < 16; Entry += 3)
			{
				const int EdgeA = DecodeTriangleEdge(CaseIndex, Entry);
				if (EdgeA < 0) break;
				const int EdgeB = DecodeTriangleEdge(CaseIndex, Entry + 1);
				const int EdgeC = DecodeTriangleEdge(CaseIndex, Entry + 2);
				if (EdgeB < 0 || EdgeC < 0 || EdgeA > 11 || EdgeB > 11 || EdgeC > 11) return false;
				uint32_t A = (uint32_t)EdgeVertex(X, Y, Z, EdgeA);
				uint32_t B = (uint32_t)EdgeVertex(X, Y, Z, EdgeB);
				uint32_t C = (uint32_t)EdgeVertex(X, Y, Z, EdgeC);
				if (A == B || A == C || B == C) continue;
				const Vector3 AB = Mesh->Positions[B] - Mesh->Positions[A];
				const Vector3 AC = Mesh->Positions[C] - Mesh->Positions[A];
				const double FaceX = (double)AB.y * (double)AC.z - (double)AB.z * (double)AC.y;
				const double FaceY = (double)AB.z * (double)AC.x - (double)AB.x * (double)AC.z;
				const double FaceZ = (double)AB.x * (double)AC.y - (double)AB.y * (double)AC.x;
				const double FaceLengthSquared = FaceX * FaceX + FaceY * FaceY + FaceZ * FaceZ;
				if (!std::isfinite(FaceLengthSquared)) return false;
				if (FaceLengthSquared <= MinimumAreaSquared) continue;
				const double InverseFaceLength = 1.0 / std::sqrt(FaceLengthSquared);
				Vector3 FaceNormal((float)(FaceX * InverseFaceLength),
					(float)(FaceY * InverseFaceLength), (float)(FaceZ * InverseFaceLength));
				const Vector3 ExpectedNormal = Mesh->Normals[A] + Mesh->Normals[B] + Mesh->Normals[C];
				if (ExpectedNormal.SquareLength() > 1.0e-12f && FaceNormal.Dot(ExpectedNormal) < 0.0f)
				{
					std::swap(B, C);
					FaceNormal = -FaceNormal;
				}
				Mesh->Indices.push_back(A); Mesh->Indices.push_back(B); Mesh->Indices.push_back(C);
				FaceNormals[A] += FaceNormal; FaceNormals[B] += FaceNormal; FaceNormals[C] += FaceNormal;
			}
		}

		for (size_t Index = 0; Index < Mesh->Normals.size(); ++Index)
		{
			if (Mesh->Normals[Index].SafeNormalize() <= std::numeric_limits<float>::epsilon())
			{
				Mesh->Normals[Index] = FaceNormals[Index].SafeUnit();
			}
		}
		return Mesh->Positions.size() <= (size_t)std::numeric_limits<uint32_t>::max();
	}
}
