#include "MeshRenderer.h"

#include <stdint.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <unordered_set>
#include <vector>

#include "../Src/Collision/GeometryObject.h"
#include "../Src/CollisionPrimitive/AxisAlignedBox3.h"
#include "../Src/CollisionPrimitive/Capsule3.h"
#include "../Src/CollisionPrimitive/ConvexMesh.h"
#include "../Src/CollisionPrimitive/Cylinder3.h"
#include "../Src/CollisionPrimitive/HeightField3.h"
#include "../Src/CollisionPrimitive/Plane3.h"
#include "../Src/CollisionPrimitive/Sphere3.h"
#include "../Src/CollisionPrimitive/StaticMesh.h"
#include "../Src/Fluid/AerodynamicsSimulation.h"
#include "../Src/Fluid/FluidSimulation.h"
#include "../Src/Fluid/MarchingCubes.h"
#include "../Src/RigidBodyDynamics/RigidBody.h"

namespace Riemann
{
	namespace
	{
		void AppendIndices(const std::vector<uint16_t>& src, std::vector<uint32_t>* dst)
		{
			dst->reserve(src.size());
			for (uint16_t index : src)
			{
				dst->push_back(static_cast<uint32_t>(index));
			}
		}

		void BuildTriangleWireframeIndices(const std::vector<uint32_t>& triangleIndices,
			std::vector<uint32_t>* wireframeIndices)
		{
			if (wireframeIndices == nullptr)
			{
				return;
			}

			wireframeIndices->clear();
			if (triangleIndices.size() % 3 != 0)
			{
				return;
			}

			std::unordered_set<uint64_t> uniqueEdges;
			uniqueEdges.reserve(triangleIndices.size());
			auto appendEdge = [&uniqueEdges, wireframeIndices](uint32_t first, uint32_t second)
			{
				if (first == second)
				{
					return;
				}

				const uint32_t minimum = std::min(first, second);
				const uint32_t maximum = std::max(first, second);
				const uint64_t edgeKey = (static_cast<uint64_t>(minimum) << 32) | maximum;
				if (uniqueEdges.insert(edgeKey).second)
				{
					wireframeIndices->push_back(first);
					wireframeIndices->push_back(second);
				}
			};

			for (size_t index = 0; index < triangleIndices.size(); index += 3)
			{
				const uint32_t first = triangleIndices[index];
				const uint32_t second = triangleIndices[index + 1];
				const uint32_t third = triangleIndices[index + 2];
				appendEdge(first, second);
				appendEdge(second, third);
				appendEdge(third, first);
			}
		}

		RenderMeshDesc MakeMeshDesc(
			const std::string& id,
			const Transform& transform,
			const std::vector<Vertex1>& vertices,
			const std::vector<uint32_t>& indices,
			const Vector4& color,
			RenderPrimitiveTopology topology,
			bool castShadow)
		{
			RenderMeshDesc mesh;
			mesh.Id = id;
			mesh.WorldTransform = transform;
			mesh.Vertices = vertices;
			mesh.Indices = indices;
			mesh.Color = color;
			mesh.Topology = topology;
			mesh.CastShadow = castShadow && color.w >= 0.999f;
			return mesh;
		}

		Transform GetGeometryWorldTransform(Geometry* geometry)
		{
			if (geometry == nullptr)
			{
				return Transform::Identity();
			}

			RigidBody* body = geometry->GetParent<RigidBody>();
			return body ? body->GetGeometryTransform(geometry) : *geometry->GetTransform();
		}

		void BuildPlaneMesh(Geometry* geometry, const std::string& id, const Vector4& color, std::vector<RenderMeshDesc>* meshes)
		{
			Plane3* shape = geometry->GetShapeObj<Plane3>();
			if (shape == nullptr)
			{
				return;
			}

			std::vector<Vector3> positions;
			std::vector<uint16_t> indices16;
			std::vector<Vector3> normals;
			shape->GetMesh(positions, indices16, normals);

			std::vector<Vertex1> vertices;
			vertices.reserve(positions.size());
			for (size_t i = 0; i < positions.size(); ++i)
			{
				const Vector3 normal = i < normals.size() ? normals[i] : Vector3::UnitY();
				vertices.push_back(Vertex1(positions[i], normal));
			}

			std::vector<uint32_t> indices;
			AppendIndices(indices16, &indices);
			RenderMeshDesc mesh = MakeMeshDesc(id, GetGeometryWorldTransform(geometry), vertices, indices,
				color, RenderPrimitiveTopology::Triangles, true);
			BuildTriangleWireframeIndices(indices, &mesh.WireframeIndices);
			meshes->push_back(std::move(mesh));
		}

		template <class TShape>
		void BuildGeometryMeshImpl(Geometry* geometry, const std::string& id, const Vector4& color, std::vector<RenderMeshDesc>* meshes)
		{
			TShape* shape = geometry->GetShapeObj<TShape>();
			if (shape == nullptr)
			{
				return;
			}

			std::vector<Vector3> positions;
			std::vector<uint16_t> indices16;
			std::vector<Vector3> normals;
			shape->GetMesh(positions, indices16, normals);

			std::vector<Vertex1> vertices;
			vertices.reserve(positions.size());
			for (size_t i = 0; i < positions.size(); ++i)
			{
				const Vector3 normal = i < normals.size() ? normals[i] : Vector3::UnitY();
				vertices.push_back(Vertex1(positions[i], normal));
			}

			std::vector<uint32_t> indices;
			AppendIndices(indices16, &indices);
			RenderMeshDesc mesh = MakeMeshDesc(id, GetGeometryWorldTransform(geometry), vertices, indices,
				color, RenderPrimitiveTopology::Triangles, true);
			BuildTriangleWireframeIndices(indices, &mesh.WireframeIndices);
			meshes->push_back(std::move(mesh));
		}

		void BuildBoxMesh(Geometry* geometry, const std::string& id, const Vector4& color,
			std::vector<RenderMeshDesc>* meshes)
		{
			AxisAlignedBox3* shape = geometry->GetShapeObj<AxisAlignedBox3>();
			if (shape == nullptr)
			{
				return;
			}

			BuildGeometryMeshImpl<AxisAlignedBox3>(geometry, id, color, meshes);
			RenderMeshDesc& mesh = meshes->back();
			std::vector<Vector3> wirePositions;
			std::vector<uint16_t> wireIndices;
			shape->GetWireframe(wirePositions, wireIndices);
			mesh.WireframeIndices.clear();
			mesh.WireframeIndices.reserve(wireIndices.size());
			for (uint16_t wireIndex : wireIndices)
			{
				const Vector3& wirePosition = wirePositions[wireIndex];
				uint32_t meshIndex = static_cast<uint32_t>(mesh.Vertices.size());
				for (uint32_t index = 0; index < mesh.Vertices.size(); ++index)
				{
					if ((mesh.Vertices[index].Pos - wirePosition).SquareLength() <= 1.0e-12f)
					{
						meshIndex = index;
						break;
					}
				}
				if (meshIndex == mesh.Vertices.size())
				{
					mesh.WireframeIndices.clear();
					return;
				}
				mesh.WireframeIndices.push_back(meshIndex);
			}
		}

		void AppendAabbWireMesh(const AxisAlignedBox3& aabb, const Transform& transform, const std::string& id, const Vector4& color, std::vector<RenderMeshDesc>* meshes)
		{
			std::vector<Vector3> positions;
			std::vector<uint16_t> indices16;
			AxisAlignedBox3 wireBox = aabb;
			wireBox.GetWireframe(positions, indices16);

			std::vector<Vertex1> vertices;
			vertices.reserve(positions.size());
			for (const Vector3& position : positions)
			{
				vertices.push_back(Vertex1(position, Vector3::UnitY()));
			}

			std::vector<uint32_t> indices;
			AppendIndices(indices16, &indices);
			meshes->push_back(MakeMeshDesc(id, transform, vertices, indices, color, RenderPrimitiveTopology::Lines, false));
		}

		void AppendStaticMeshIndices(StaticMesh* mesh, std::vector<uint32_t>* indices)
		{
			const uint32_t indexCount = mesh->GetTriangleCount() * 3;
			indices->reserve(indexCount);
			if (mesh->Is16bitIndices())
			{
				const uint16_t* src = mesh->GetIndices16();
				for (uint32_t i = 0; i < indexCount; ++i)
				{
					indices->push_back(static_cast<uint32_t>(src[i]));
				}
			}
			else
			{
				const uint32_t* src = mesh->GetIndices32();
				for (uint32_t i = 0; i < indexCount; ++i)
				{
					indices->push_back(src[i]);
				}
			}
		}

		bool IsFiniteColor(const Vector4& color)
		{
			return std::isfinite(color.x) && std::isfinite(color.y)
				&& std::isfinite(color.z) && std::isfinite(color.w);
		}

		void InitializeSurfaceMeshDesc(const std::string& id, const Vector3& center,
			const Vector4& color, bool unlit, RenderBlendMode blendMode, RenderMeshDesc* mesh)
		{
			*mesh = RenderMeshDesc();
			mesh->Id = id;
			mesh->WorldTransform = Transform(center);
			mesh->Color = color;
			mesh->Topology = RenderPrimitiveTopology::Triangles;
			mesh->FillMode = RenderFillMode::Solid;
			mesh->BlendMode = blendMode;
			mesh->Unlit = unlit;
			mesh->CastShadow = false;
		}

		bool ExtractSurfaceMeshDesc(const MarchingCubesInput& input, const std::string& id,
			const Vector3& center, const Vector4& color, bool unlit,
			RenderBlendMode blendMode, RenderMeshDesc* mesh)
		{
			InitializeSurfaceMeshDesc(id, center, color, unlit, blendMode, mesh);
			FluidSurfaceMesh surface;
			if (!MarchingCubes::Extract(input, &surface)) return false;
			if (surface.Indices.empty()) return true;
			mesh->Vertices.reserve(surface.Positions.size());
			for (size_t index = 0; index < surface.Positions.size(); ++index)
			{
				mesh->Vertices.push_back(Vertex1(surface.Positions[index] - center,
					surface.Normals[index]));
			}
			mesh->Indices = std::move(surface.Indices);
			return true;
		}

		void SetGasGhostSamples(std::vector<float>* density, const Vector3i& resolution,
			float ambientDensity, float isoValue, bool highValuesInside)
		{
			const size_t NX = (size_t)resolution.x;
			const size_t NY = (size_t)resolution.y;
			auto Index = [NX, NY](int X, int Y, int Z) -> size_t
			{
				return (size_t)X + NX * ((size_t)Y + NY * (size_t)Z);
			};
			const float lowerOutside = std::nextafter(isoValue,
				-std::numeric_limits<float>::infinity());
			const float upperOutside = std::nextafter(isoValue,
				std::numeric_limits<float>::infinity());
			for (int Z = 0; Z < resolution.z; ++Z)
			for (int Y = 0; Y < resolution.y; ++Y)
			for (int X = 0; X < resolution.x; ++X)
			{
				if (X > 0 && X + 1 < resolution.x && Y > 0 && Y + 1 < resolution.y
					&& Z > 0 && Z + 1 < resolution.z)
				{
					continue;
				}
				const int interiorX = std::max(1, std::min(resolution.x - 2, X));
				const int interiorY = std::max(1, std::min(resolution.y - 2, Y));
				const int interiorZ = std::max(1, std::min(resolution.z - 2, Z));
				const float interiorDensity = (*density)[Index(interiorX, interiorY, interiorZ)];
				const bool inside = highValuesInside ? interiorDensity > isoValue
					: interiorDensity < isoValue;
				if (!inside)
				{
					(*density)[Index(X, Y, Z)] = ambientDensity;
					continue;
				}
				const float mirroredDensity = 2.0f * isoValue - interiorDensity;
				(*density)[Index(X, Y, Z)] = highValuesInside
					? std::min(mirroredDensity, lowerOutside)
					: std::max(mirroredDensity, upperOutside);
			}
		}
	}

	GasDensityVisualizationSettings::GasDensityVisualizationSettings()
		: RelativeThreshold(0.08f)
		, CompressedColor(1.0f, 0.23f, 0.04f, 0.20f)
		, RarefiedColor(0.03f, 0.42f, 1.0f, 0.16f)
	{
	}

	void BuildTriMeshMeshes(StaticMesh* mesh, const Transform& transform, const std::string& id, const Vector4& color, bool renderBounds, std::vector<RenderMeshDesc>* meshes)
	{
		if (mesh == nullptr || meshes == nullptr || mesh->GetVertexCount() == 0 || mesh->GetTriangleCount() == 0)
		{
			return;
		}

		mesh->CalculateWeightAverageNormals();

		std::vector<Vertex1> vertices;
		vertices.reserve(mesh->GetVertexCount());
		for (uint32_t i = 0; i < mesh->GetVertexCount(); ++i)
		{
			const Vector3 normal = i < mesh->mNormals.size() ? mesh->mNormals[i] : Vector3::UnitY();
			vertices.push_back(Vertex1(mesh->Vertices[i], normal));
		}

		std::vector<uint32_t> indices;
		AppendStaticMeshIndices(mesh, &indices);
		RenderMeshDesc renderMesh = MakeMeshDesc(id, transform, vertices, indices, color,
			RenderPrimitiveTopology::Triangles, true);
		BuildTriangleWireframeIndices(indices, &renderMesh.WireframeIndices);
		meshes->push_back(std::move(renderMesh));

		if (renderBounds)
		{
			AppendAabbWireMesh(AxisAlignedBox3(mesh->BoundingVolume.Min, mesh->BoundingVolume.Max), transform, id + ".bounds", Vector4(1.0f, 1.0f, 1.0f, 1.0f), meshes);
		}
	}

	void BuildAabbWireMesh(const Box3& aabb, const std::string& id, const Vector4& color, std::vector<RenderMeshDesc>* meshes)
	{
		if (meshes == nullptr)
		{
			return;
		}

		AppendAabbWireMesh(AxisAlignedBox3(aabb.Min, aabb.Max), Transform(), id, color, meshes);
	}

	bool BuildWaterSurfaceMesh(const FluidSimulation& simulation, const std::string& id,
		RenderMeshDesc* mesh, float isoDensityRatio, const Vector4& color)
	{
		if (mesh == nullptr || id.empty() || !simulation.IsInitialized()
			|| !std::isfinite(isoDensityRatio) || isoDensityRatio <= 0.0f)
		{
			return false;
		}
		std::vector<float> cellDensity;
		if (!simulation.BuildDensityField(&cellDensity)) return false;
		const FluidSimulationSettings& settings = simulation.GetSettings();
		const Vector3i paddedResolution(settings.Resolution.x + 2,
			settings.Resolution.y + 2, settings.Resolution.z + 2);
		const size_t NX = (size_t)paddedResolution.x;
		const size_t NY = (size_t)paddedResolution.y;
		const size_t NZ = (size_t)paddedResolution.z;
		if (NX > std::numeric_limits<size_t>::max() / NY
			|| NX * NY > std::numeric_limits<size_t>::max() / NZ)
		{
			return false;
		}
		std::vector<float> paddedDensity(NX * NY * NZ, 0.0f);
		auto PaddedIndex = [NX, NY](int X, int Y, int Z) -> size_t
		{
			return (size_t)X + NX * ((size_t)Y + NY * (size_t)Z);
		};
		for (int Z = 0; Z < settings.Resolution.z; ++Z)
		for (int Y = 0; Y < settings.Resolution.y; ++Y)
		for (int X = 0; X < settings.Resolution.x; ++X)
		{
			const size_t Source = (size_t)X + (size_t)settings.Resolution.x
				* ((size_t)Y + (size_t)settings.Resolution.y * (size_t)Z);
			paddedDensity[PaddedIndex(X + 1, Y + 1, Z + 1)] = cellDensity[Source];
		}

		MarchingCubesInput input;
		input.Origin = settings.BoundsMin - simulation.GetCellSize() * 0.5f;
		input.Spacing = simulation.GetCellSize();
		input.Resolution = paddedResolution;
		input.Values = paddedDensity.data();
		input.ValueCount = paddedDensity.size();
		input.IsoValue = settings.RestDensity * isoDensityRatio;
		input.HighValuesInside = true;
		const Vector3 surfaceCenter = settings.BoundsMin
			+ (settings.BoundsMax - settings.BoundsMin) * 0.5f;
		return ExtractSurfaceMeshDesc(input, id, surfaceCenter, color, false,
			RenderBlendMode::Alpha, mesh);
	}

	bool BuildGasDensityMeshes(const AerodynamicsSimulation& simulation, const std::string& idPrefix,
		std::vector<RenderMeshDesc>* meshes, const GasDensityVisualizationSettings& settings)
	{
		if (meshes == nullptr || idPrefix.empty() || !simulation.IsInitialized()
			|| !std::isfinite(settings.RelativeThreshold) || settings.RelativeThreshold <= 0.0f
			|| settings.RelativeThreshold >= 1.0f || !IsFiniteColor(settings.CompressedColor)
			|| !IsFiniteColor(settings.RarefiedColor) || settings.CompressedColor.w <= 0.0f
			|| settings.CompressedColor.w > 1.0f || settings.RarefiedColor.w <= 0.0f
			|| settings.RarefiedColor.w > 1.0f)
		{
			return false;
		}

		std::vector<float> cellDensity;
		if (!simulation.BuildDensityField(&cellDensity)) return false;
		const AerodynamicsSimulationSettings& simulationSettings = simulation.GetSettings();
		const size_t expectedCellCount = (size_t)simulationSettings.Resolution.x
			* (size_t)simulationSettings.Resolution.y * (size_t)simulationSettings.Resolution.z;
		if (cellDensity.size() != expectedCellCount) return false;
		float minimumDensity = std::numeric_limits<float>::max();
		float maximumDensity = 0.0f;
		for (float density : cellDensity)
		{
			if (!std::isfinite(density) || density < 0.0f) return false;
			minimumDensity = std::min(minimumDensity, density);
			maximumDensity = std::max(maximumDensity, density);
		}

		const Vector3i paddedResolution(simulationSettings.Resolution.x + 2,
			simulationSettings.Resolution.y + 2, simulationSettings.Resolution.z + 2);
		const size_t NX = (size_t)paddedResolution.x;
		const size_t NY = (size_t)paddedResolution.y;
		const size_t NZ = (size_t)paddedResolution.z;
		if (NX > std::numeric_limits<size_t>::max() / NY
			|| NX * NY > std::numeric_limits<size_t>::max() / NZ)
		{
			return false;
		}
		std::vector<float> paddedDensity(NX * NY * NZ, simulationSettings.AmbientDensity);
		auto PaddedIndex = [NX, NY](int X, int Y, int Z) -> size_t
		{
			return (size_t)X + NX * ((size_t)Y + NY * (size_t)Z);
		};
		for (int Z = 0; Z < simulationSettings.Resolution.z; ++Z)
		for (int Y = 0; Y < simulationSettings.Resolution.y; ++Y)
		for (int X = 0; X < simulationSettings.Resolution.x; ++X)
		{
			const size_t source = (size_t)X + (size_t)simulationSettings.Resolution.x
				* ((size_t)Y + (size_t)simulationSettings.Resolution.y * (size_t)Z);
			paddedDensity[PaddedIndex(X + 1, Y + 1, Z + 1)] = cellDensity[source];
		}

		MarchingCubesInput input;
		input.Origin = simulationSettings.BoundsMin - simulation.GetCellSize() * 0.5f;
		input.Spacing = simulation.GetCellSize();
		input.Resolution = paddedResolution;
		input.Values = paddedDensity.data();
		input.ValueCount = paddedDensity.size();
		const Vector3 center = simulationSettings.BoundsMin
			+ (simulationSettings.BoundsMax - simulationSettings.BoundsMin) * 0.5f;
		std::vector<RenderMeshDesc> result(2);

		const float compressedIso = simulationSettings.AmbientDensity
			* (1.0f + settings.RelativeThreshold);
		const float compressedOutside = std::nextafter(compressedIso,
			-std::numeric_limits<float>::infinity());
		for (int Z = 1; Z + 1 < paddedResolution.z; ++Z)
		for (int Y = 1; Y + 1 < paddedResolution.y; ++Y)
		for (int X = 1; X + 1 < paddedResolution.x; ++X)
		{
			float& density = paddedDensity[PaddedIndex(X, Y, Z)];
			if (density == compressedIso) density = compressedOutside;
		}
		SetGasGhostSamples(&paddedDensity, paddedResolution, simulationSettings.AmbientDensity,
			compressedIso, true);
		input.IsoValue = compressedIso;
		input.HighValuesInside = true;
		if (maximumDensity > compressedIso)
		{
			if (!ExtractSurfaceMeshDesc(input, idPrefix + ".compressed", center,
				settings.CompressedColor, true, RenderBlendMode::Additive, &result[0])) return false;
		}
		else
		{
			InitializeSurfaceMeshDesc(idPrefix + ".compressed", center,
				settings.CompressedColor, true, RenderBlendMode::Additive, &result[0]);
		}

		const float rarefiedIso = simulationSettings.AmbientDensity
			* (1.0f - settings.RelativeThreshold);
		SetGasGhostSamples(&paddedDensity, paddedResolution, simulationSettings.AmbientDensity,
			rarefiedIso, false);
		input.IsoValue = rarefiedIso;
		input.HighValuesInside = false;
		if (minimumDensity < rarefiedIso)
		{
			if (!ExtractSurfaceMeshDesc(input, idPrefix + ".rarefied", center,
				settings.RarefiedColor, true, RenderBlendMode::Additive, &result[1])) return false;
		}
		else
		{
			InitializeSurfaceMeshDesc(idPrefix + ".rarefied", center,
				settings.RarefiedColor, true, RenderBlendMode::Additive, &result[1]);
		}

		*meshes = std::move(result);
		return true;
	}

	void BuildGeometryMeshes(Geometry* geometry, const std::string& id, const Vector4& color, bool renderBounds, std::vector<RenderMeshDesc>* meshes)
	{
		if (geometry == nullptr || meshes == nullptr)
		{
			return;
		}

		if (geometry->GetShapeType() == PrimitiveType::TRIANGLE_MESH)
		{
			BuildTriMeshMeshes(geometry->GetShapeObj<StaticMesh>(), GetGeometryWorldTransform(geometry), id, color, renderBounds, meshes);
		}
		else if (geometry->GetShapeType() == PrimitiveType::CONVEX_MESH)
		{
			BuildGeometryMeshImpl<ConvexMesh>(geometry, id, color, meshes);
		}
		else if (geometry->GetShapeType() == PrimitiveType::BOX)
		{
			BuildBoxMesh(geometry, id, color, meshes);
		}
		else if (geometry->GetShapeType() == PrimitiveType::PLANE)
		{
			BuildPlaneMesh(geometry, id, color, meshes);
		}
		else if (geometry->GetShapeType() == PrimitiveType::SPHERE)
		{
			BuildGeometryMeshImpl<Sphere3>(geometry, id, color, meshes);
		}
		else if (geometry->GetShapeType() == PrimitiveType::CAPSULE)
		{
			BuildGeometryMeshImpl<Capsule3>(geometry, id, color, meshes);
		}
		else if (geometry->GetShapeType() == PrimitiveType::CYLINDER)
		{
			BuildGeometryMeshImpl<Cylinder3>(geometry, id, color, meshes);
		}
		else if (geometry->GetShapeType() == PrimitiveType::HEIGHTFIELD)
		{
			BuildGeometryMeshImpl<HeightField3>(geometry, id, color, meshes);
		}
	}
}
