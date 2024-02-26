#pragma once

namespace Riemann
{
	class Renderer;
	class Geometry;
	class Mesh;

	void AddGeometry(Renderer* renderer, Geometry* geom);
	void AddTriMesh(Renderer* renderer, Mesh* mesh, void* Trans, bool RenderBV);
}