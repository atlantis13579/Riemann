#include "Test.h"

#include <algorithm>
#include <cstring>
#include <iterator>
#include <limits>
#include <memory>
#include <vector>

#include "../Src/CollisionPrimitive/StaticMesh.h"
#include "../Src/CollisionPrimitive/TriangleMesh.h"
#include "../Src/Geometry/VoxelField.h"

using namespace Riemann;

namespace
{
	struct VoxelMeshReference
	{
		int Width, Height, Depth;
		Vector3 Origin;
		float Spacing, HeightSpacing;
		std::vector<unsigned char> Occupied;

		VoxelMeshReference(int width, int height, int depth, const Vector3& origin = Vector3::Zero(), float spacing = 1.0f, float heightSpacing = 1.0f)
			: Width(width), Height(height), Depth(depth), Origin(origin), Spacing(spacing), HeightSpacing(heightSpacing)
			, Occupied((size_t)width * height * depth, 0)
		{
		}

		void Initialize(VoxelField& field) const
		{
			field.MakeEmpty(Box3(Origin, Origin + Vector3(Width * Spacing, Height * HeightSpacing, Depth * Spacing)),
				Width, Height, Depth, Spacing, HeightSpacing);
		}

		bool Contains(int x, int y, int z) const
		{
			return x >= 0 && y >= 0 && z >= 0 && x < Width && y < Height && z < Depth
				&& Occupied[((size_t)z * Height + y) * Width + x] != 0;
		}

		bool Contains(const Vector3& position) const
		{
			const Vector3 local = position - Origin;
			return Contains((int)std::floor(local.x / Spacing), (int)std::floor(local.y / HeightSpacing), (int)std::floor(local.z / Spacing));
		}

		void AddSpan(VoxelField& field, int x, int z, int low, int high)
		{
			EXPECT(field.AddVoxel(z * Width + x, (uint16_t)low, (uint16_t)high, 0.0f));
			for (int y = low; y <= high; ++y) Occupied[((size_t)z * Height + y) * Width + x] = 1;
		}

		void Check(const StaticMesh& mesh) const
		{
			double expectedArea = 0.0, expectedVolume = 0.0;
			for (int z = 0; z < Depth; ++z)
			for (int y = 0; y < Height; ++y)
			for (int x = 0; x < Width; ++x)
			{
				if (!Contains(x, y, z)) continue;
				expectedVolume += (double)Spacing * Spacing * HeightSpacing;
				if (!Contains(x - 1, y, z)) expectedArea += (double)Spacing * HeightSpacing;
				if (!Contains(x + 1, y, z)) expectedArea += (double)Spacing * HeightSpacing;
				if (!Contains(x, y - 1, z)) expectedArea += (double)Spacing * Spacing;
				if (!Contains(x, y + 1, z)) expectedArea += (double)Spacing * Spacing;
				if (!Contains(x, y, z - 1)) expectedArea += (double)Spacing * HeightSpacing;
				if (!Contains(x, y, z + 1)) expectedArea += (double)Spacing * HeightSpacing;
			}
			EXPECT(mesh.mNormals.size() == mesh.NumVertices);
			double actualArea = 0.0, signedVolume = 0.0;
			const float offset = std::min(Spacing, HeightSpacing) * 0.001f;
			for (uint32_t i = 0; i < mesh.NumTriangles; ++i)
			{
				uint32_t indices[3];
				mesh.GetVertIndices(i, indices[0], indices[1], indices[2]);
				const bool valid = indices[0] < mesh.NumVertices && indices[1] < mesh.NumVertices && indices[2] < mesh.NumVertices;
				EXPECT(valid);
				if (!valid) continue;
				const Vector3 a = mesh.Vertices[indices[0]], b = mesh.Vertices[indices[1]], c = mesh.Vertices[indices[2]];
				const Vector3 cross = (b - a).Cross(c - a);
				const float twiceArea = cross.Length();
				EXPECT(std::isfinite(twiceArea) && twiceArea > 0.0f);
				if (!(twiceArea > 0.0f)) continue;
				const Vector3 normal = cross / twiceArea;
				const Vector3 center = (a + b + c) / 3.0f;
				// Boundary triangles have occupied material behind their outward normal only.
				EXPECT(Contains(center - normal * offset));
				EXPECT_FALSE(Contains(center + normal * offset));
				for (uint32_t index : indices)
				{
					if (index >= mesh.mNormals.size()) continue;
					EXPECT_NEAR(mesh.mNormals[index].Length(), 1.0f, 1e-5f);
					EXPECT(mesh.mNormals[index].Dot(normal) > 0.999f);
				}
				actualArea += 0.5 * twiceArea;
				signedVolume += (double)(a - Origin).Dot((b - Origin).Cross(c - Origin)) / 6.0;
			}
			EXPECT_NEAR(actualArea, expectedArea, 1e-5 * std::max(1.0, expectedArea));
			EXPECT_NEAR(signedVolume, expectedVolume, 1e-5 * std::max(1.0, expectedVolume));
			EXPECT((expectedVolume == 0.0) == (mesh.NumTriangles == 0));
		}
	};

	void TestVoxelBoundaryGeometry()
	{
		VoxelField field;
		VoxelMeshReference reference(3, 8, 2, Vector3(4, -3, 7), 2.0f, 0.5f);
		reference.Initialize(field);
		StaticMesh mesh;
		EXPECT(field.GenerateMesh(mesh));
		reference.Check(mesh);
		reference.AddSpan(field, 0, 0, 0, 5);
		reference.AddSpan(field, 1, 0, 6, 6);
		reference.AddSpan(field, 1, 0, 2, 3);
		reference.AddSpan(field, 2, 1, 1, 4);
		EXPECT(field.Verify());
		EXPECT(field.GenerateMesh(mesh));
		reference.Check(mesh);
		EXPECT_SAME(mesh.BoundingVolume.Min, Vector3(4, -3, 7));
		EXPECT_SAME(mesh.BoundingVolume.Max, Vector3(10, 0.5f, 11));

		VoxelMeshReference cropped = reference;
		std::fill(cropped.Occupied.begin(), cropped.Occupied.end(), 0);
		for (int y = 0; y < reference.Height; ++y)
			cropped.Occupied[(size_t)y * reference.Width + 1] = reference.Contains(1, y, 0) ? 1 : 0;
		EXPECT(field.GenerateMesh(mesh, 1, 1, -20, 0));
		cropped.Check(mesh);
		std::unique_ptr<StaticMesh> debug(field.CreateDebugMesh(1, 1, 0, 0));
		EXPECT(debug != nullptr);
		if (debug) cropped.Check(*debug);
		EXPECT(field.GenerateMesh(mesh, 20, 30, 0, 1));
		EXPECT(mesh.NumVertices == 0 && mesh.NumTriangles == 0);
		EXPECT_FALSE(field.GenerateMesh(mesh, 2, 1, 0, 1));
		EXPECT(mesh.NumVertices == 0 && mesh.NumTriangles == 0);

		VoxelField single;
		VoxelMeshReference unit(1, 1, 1, Vector3(3, -2, 4), 2.0f, 0.5f);
		unit.Initialize(single);
		unit.AddSpan(single, 0, 0, 0, 0);
		TriangleMesh collider;
		EXPECT(single.GenerateMesh(collider));
		unit.Check(collider);
		EXPECT(collider.NumTriangles == 12);
		collider.BuildBVH();
		EXPECT(collider.GetBVH() != nullptr);
		TriMeshHitOption option;
		option.hitNearest = true;
		option.hitBothSides = false;
		option.maxDist = 20.0f;
		TriMeshHitResult result = {};
		EXPECT(collider.IntersectRay(Vector3(3.6f, 2.0f, 4.7f), -Vector3::UnitY(), option, &result));
		EXPECT_NEAR(result.hitTime, 3.5f, 1e-5f);
		Vector3 a, b, c;
		if (collider.GetTriangle(result.hitIndex, &a, &b, &c))
			EXPECT((b - a).Cross(c - a).Dot(Vector3::UnitY()) > 0.0f);
	}

	void TestVoxelRasterizationBounds()
	{
		VoxelField field;
		VoxelMeshReference reference(4, 4, 4);
		reference.Initialize(field);
		field.VoxelizationCube(Box3(Vector3(-4, -4, -4), Vector3(-2, -2, -2)));
		field.VoxelizationYPlane(10.0f);
		StaticMesh mesh;
		EXPECT(field.GenerateMesh(mesh));
		reference.Check(mesh);
		EXPECT(field.VoxelizationCube(Box3(Vector3(-2, -2, -2), Vector3(0.25f, 0.25f, 0.25f))));
		reference.Occupied[0] = 1;
		EXPECT(field.VoxelizationCube(Box3(Vector3(2.1f, 2.1f, 2.1f), Vector3(9, 9, 9))));
		for (int z = 2; z < 4; ++z)
		for (int y = 2; y < 4; ++y)
		for (int x = 2; x < 4; ++x)
			reference.Occupied[((size_t)z * 4 + y) * 4 + x] = 1;
		EXPECT(field.GenerateMesh(mesh));
		reference.Check(mesh);
		EXPECT_FALSE(field.AddVoxel(-1, 0, 0, 0));
		EXPECT_FALSE(field.AddVoxel(16, 0, 0, 0));
		EXPECT_FALSE(field.AddVoxel(0, 2, 1, 0));
		EXPECT_FALSE(field.AddVoxel(0, 0, 4, 0));

		VoxelField triangleField;
		VoxelMeshReference triangleReference(2, 2, 2);
		triangleReference.Initialize(triangleField);
		VoxelizationInfo info = {};
		info.VoxelSize = info.VoxelHeight = 1.0f;
		info.BV = triangleField.GetBoundingVolume();
		EXPECT(triangleField.VoxelizationTri(Vector3(0.1f, 0.25f, 0.1f),
			Vector3(0.9f, 0.25f, 0.1f), Vector3(0.1f, 0.25f, 0.9f), info));
		triangleReference.Occupied[0] = 1;
		EXPECT(triangleField.GenerateMesh(mesh));
		triangleReference.Check(mesh);
	}

	void TestLargeVoxelMesh()
	{
		VoxelField field;
		VoxelMeshReference reference(80, 1, 80);
		reference.Initialize(field);
		for (int z = 0; z < 80; ++z)
		for (int x = 0; x < 80; ++x)
			if (((x + z) & 1) == 0) reference.AddSpan(field, x, z, 0, 0);
		StaticMesh mesh;
		EXPECT(field.GenerateMesh(mesh));
		EXPECT(mesh.NumVertices > 65535);
		EXPECT_FALSE(mesh.Is16bitIndices());
		uint32_t largest = 0;
		for (uint32_t i = 0; i < mesh.NumTriangles; ++i)
		{
			uint32_t a, b, c;
			mesh.GetVertIndices(i, a, b, c);
			largest = std::max(largest, std::max(a, std::max(b, c)));
		}
		EXPECT(largest > 65535 && largest < mesh.NumVertices);
		reference.Check(mesh);
	}

	std::vector<uint32_t> VoxelContents(VoxelField& field)
	{
		std::vector<uint32_t> values;
		field.TraversalField([&](int index, Voxel* voxel)
		{
			for (; voxel; voxel = voxel->next)
			{
				values.push_back((uint32_t)index);
				values.push_back(voxel->data);
				values.push_back(voxel->ymin);
				values.push_back(voxel->ymax);
			}
		});
		return values;
	}

	template<typename T>
	void WriteVoxelFileValue(std::vector<char>& bytes, size_t offset, const T& value)
	{
		EXPECT(offset + sizeof(value) <= bytes.size());
		if (offset + sizeof(value) <= bytes.size()) std::memcpy(bytes.data() + offset, &value, sizeof(value));
	}

	void TestVoxelSerialization()
	{
		VoxelField field;
		VoxelMeshReference reference(3, 8, 2, Vector3(-2, 5, 3), 0.5f, 0.25f);
		reference.Initialize(field);
		reference.AddSpan(field, 0, 0, 0, 0);
		reference.AddSpan(field, 0, 0, 3, 4);
		reference.AddSpan(field, 2, 1, 1, 6);
		uint32_t data = 17;
		field.TraversalVoxel([&](Voxel* voxel) { voxel->data = data; data += 31; return true; });
		const std::vector<uint32_t> original = VoxelContents(field);
		const std::string path = TestOutputPath("voxel_mesh_roundtrip.vxl");
		const std::string badPath = TestOutputPath("voxel_mesh_invalid.vxl");
		EXPECT(field.SerializeTo(path.c_str()));
		VoxelField restored;
		EXPECT(restored.SerializeFrom(path.c_str()));
		EXPECT(restored.Verify());
		EXPECT(VoxelContents(restored) == original);
		EXPECT(restored.GetSizeX() == 3 && restored.GetSizeY() == 8 && restored.GetSizeZ() == 2);
		EXPECT_NEAR(restored.GetVoxelSize(), 0.5f, 1e-6f);
		EXPECT_NEAR(restored.GetVoxelHeight(), 0.25f, 1e-6f);
		EXPECT(restored.GetBoundingVolume() == field.GetBoundingVolume());
		StaticMesh mesh;
		EXPECT(restored.GenerateMesh(mesh));
		reference.Check(mesh);
		std::ifstream input(path.c_str(), std::ios::binary);
		const std::vector<char> valid((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
		EXPECT(valid.size() == 60 + 3 * sizeof(VoxelFast) + 2 * 12);
		if (valid.size() < 108) return;

		auto reject = [&](const std::vector<char>& bytes)
		{
			{
				std::ofstream output(badPath.c_str(), std::ios::binary | std::ios::trunc);
				if (!bytes.empty()) output.write(bytes.data(), (std::streamsize)bytes.size());
				EXPECT(output.good());
			}
			EXPECT_FALSE(restored.SerializeFrom(badPath.c_str()));
			EXPECT(VoxelContents(restored) == original);
			EXPECT(restored.GetBoundingVolume() == field.GetBoundingVolume());
		};
		reject({});
		reject(std::vector<char>(valid.begin(), valid.begin() + 20));
		reject(std::vector<char>(valid.begin(), valid.end() - 1));
		std::vector<char> bytes = valid;
		bytes.push_back(0);
		reject(bytes);
		bytes = valid; WriteVoxelFileValue(bytes, 0, uint32_t(0x32F0587A)); reject(bytes);
		bytes = valid; WriteVoxelFileValue(bytes, 4, uint32_t(3)); reject(bytes);
		bytes = valid; WriteVoxelFileValue(bytes, 8, std::numeric_limits<uint32_t>::max()); reject(bytes);
		bytes = valid; WriteVoxelFileValue(bytes, 16, int32_t(-1)); reject(bytes);
		bytes = valid; WriteVoxelFileValue(bytes, 32, std::numeric_limits<float>::quiet_NaN()); reject(bytes);
		bytes = valid; WriteVoxelFileValue(bytes, 64, uint16_t(2)); reject(bytes);
		bytes = valid; WriteVoxelFileValue(bytes, 66, uint16_t(8)); reject(bytes);
		bytes = valid; WriteVoxelFileValue(bytes, 72, uint16_t(0)); reject(bytes);
		const size_t fieldsOffset = 60 + 3 * sizeof(VoxelFast);
		bytes = valid; WriteVoxelFileValue(bytes, fieldsOffset, uint32_t(6)); reject(bytes);
		bytes = valid; WriteVoxelFileValue(bytes, fieldsOffset + 12, uint32_t(0)); reject(bytes);
		bytes = valid; WriteVoxelFileValue(bytes, fieldsOffset + 4, uint32_t(3)); reject(bytes);
		bytes = valid; WriteVoxelFileValue(bytes, fieldsOffset + 8, uint32_t(0)); reject(bytes);
		bytes = valid; WriteVoxelFileValue(bytes, fieldsOffset + 8, uint32_t(4)); reject(bytes);

		VoxelField empty;
		reference.Initialize(empty);
		EXPECT(empty.SerializeTo(path.c_str()));
		EXPECT(restored.SerializeFrom(path.c_str()));
		EXPECT(restored.Verify());
		EXPECT(VoxelContents(restored).empty());
		EXPECT(restored.GenerateMesh(mesh));
		EXPECT(mesh.NumTriangles == 0);
	}

	void TestVoxelMeshIndexStorage()
	{
		StaticMesh empty;
		empty.Compact();
		EXPECT(empty.NumVertices == 0 && empty.NumTriangles == 0);
		EXPECT(empty.Vertices == nullptr && empty.Indices == nullptr);

		StaticMesh mesh;
		for (int i = 0; i < 70000; ++i) mesh.AddVertex(Vector3((float)i, 0.0f, 0.0f));
		mesh.AddTriangle(0, 65536, 69999);
		mesh.Compact();
		uint32_t a, b, c;
		mesh.GetVertIndices(0, a, b, c);
		EXPECT_FALSE(mesh.Is16bitIndices());
		EXPECT(a == 0 && b == 65536 && c == 69999);

		mesh.Clear();
		for (int i = 0; i < 3; ++i) mesh.AddVertex(Vector3((float)i, 0.0f, 0.0f));
		mesh.AddTriangle(0, 1, 2);
		mesh.Compact();
		EXPECT(mesh.Is16bitIndices());
		for (int i = 3; i < 70000; ++i) mesh.AddVertex(Vector3((float)i, 0.0f, 0.0f));
		mesh.AddTriangle(65536, 65537, 69999);
		EXPECT_FALSE(mesh.Is16bitIndices());
		mesh.GetVertIndices(0, a, b, c);
		EXPECT(a == 0 && b == 1 && c == 2);
		mesh.GetVertIndices(1, a, b, c);
		EXPECT(a == 65536 && b == 65537 && c == 69999);
		mesh.Compact();
		mesh.GetVertIndices(0, a, b, c);
		EXPECT(a == 0 && b == 1 && c == 2);
		mesh.GetVertIndices(1, a, b, c);
		EXPECT(a == 65536 && b == 65537 && c == 69999);
	}
}

void TestVoxelMesh()
{
	printf("Running TestVoxelMesh\n");
	TestVoxelMeshIndexStorage();
	TestVoxelBoundaryGeometry();
	TestVoxelRasterizationBounds();
	TestLargeVoxelMesh();
	TestVoxelSerialization();
}
