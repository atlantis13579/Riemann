#pragma once

namespace Riemann
{
	class StaticMesh;
}

namespace Riemann
{
	class Renderer;
	class Geometry;
	class StaticMesh;

	void AddGeometry(Renderer* renderer, Geometry* geom);
	void AddTriMesh(Renderer* renderer, StaticMesh* mesh, void* Trans, bool RenderBV);
}