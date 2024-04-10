#pragma once

namespace Riemann
{
	class StaticMesh;
}

namespace Riemann
{
	class Renderer;
	class GeometryBase;
	class StaticMesh;

	void AddGeometry(Renderer* renderer, GeometryBase* geom);
	void AddTriMesh(Renderer* renderer, StaticMesh* mesh, void* Trans, bool RenderBV);
}