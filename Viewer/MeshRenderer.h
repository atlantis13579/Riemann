#pragma once

#include <string>
#include <vector>

#include "../Renderer/RiemannRenderer.h"
#include "../Src/Maths/Box3.h"

namespace Riemann
{
	class Geometry;
	class StaticMesh;
	class FluidSimulation;
	class AerodynamicsSimulation;

	struct GasDensityVisualizationSettings
	{
		GasDensityVisualizationSettings();

		float RelativeThreshold;
		Vector4 CompressedColor;
		Vector4 RarefiedColor;
	};

	void BuildGeometryMeshes(Geometry* geometry, const std::string& id, const Vector4& color, bool renderBounds, std::vector<RenderMeshDesc>* meshes);
	void BuildTriMeshMeshes(StaticMesh* mesh, const Transform& transform, const std::string& id, const Vector4& color, bool renderBounds, std::vector<RenderMeshDesc>* meshes);
	void BuildAabbWireMesh(const Box3& aabb, const std::string& id, const Vector4& color, std::vector<RenderMeshDesc>* meshes);
	bool BuildWaterSurfaceMesh(const FluidSimulation& simulation, const std::string& id,
		RenderMeshDesc* mesh, float isoDensityRatio = 0.5f,
		const Vector4& color = Vector4(0.08f, 0.38f, 0.82f, 0.68f));
	bool BuildGasDensityMeshes(const AerodynamicsSimulation& simulation, const std::string& idPrefix,
		std::vector<RenderMeshDesc>* meshes,
		const GasDensityVisualizationSettings& settings = GasDensityVisualizationSettings());
}
