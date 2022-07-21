#pragma once

class Renderer;
class Geometry;
class Mesh;
class Transform;

void AddGeometry(Renderer* renderer, Geometry* geom);
void AddTriMesh(Renderer* renderer, Mesh* mesh, Transform* Trans, bool RenderBV);