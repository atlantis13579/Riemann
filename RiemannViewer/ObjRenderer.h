#pragma once

namespace Riemann
{
	class Renderer;
	class GeometryBase;
	class Mesh;

	void AddGeometry(Renderer* renderer, GeometryBase* geom);
	void AddTriMesh(Renderer* renderer, Mesh* mesh, void* Trans, bool RenderBV);
}