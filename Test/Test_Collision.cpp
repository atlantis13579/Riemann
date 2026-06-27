
#include "Test.h"

#include <vector>

#include "../Src/CollisionPrimitive/AxisAlignedBox3.h"
#include "../Src/CollisionPrimitive/Plane3.h"
#include "../Src/CollisionPrimitive/Sphere3.h"
#include "../Src/CollisionPrimitive/Ray3.h"
#include "../Src/CollisionPrimitive/OrientedBox3.h"
#include "../Src/CollisionPrimitive/Triangle3.h"
#include "../Src/CollisionPrimitive/Cylinder3.h"
#include "../Src/CollisionPrimitive/Capsule3.h"
#include "../Src/CollisionPrimitive/ConvexMesh.h"
#include "../Src/CollisionPrimitive/HeightField3.h"
#include "../Src/CollisionPrimitive/StaticMesh.h"
#include "../Src/CollisionPrimitive/MeshBVH4.h"
#include "../Src/CollisionPrimitive/TriangleMesh.h"
#include "../Src/CollisionPrimitive/GJK.h"
#include "../Src/Collision/AABBTree.h"
#include "../Src/Collision/DynamicAABBTree.h"
#include "../Src/Collision/GeometryQuery.h"
#include "../Src/Collision/SAP.h"
#include "../Src/Collision/SAP_Incremental.h"
#include "../Src/Collision/GeometryDifference.h"
#include "../Src/Collision/GeometryIntersection.h"
#include "../Src/Collision/EPAPenetration.h"
#include "../Src/RigidBodyDynamics/BroadPhase.h"
#include "../Src/RigidBodyDynamics/PhysicsWorld.h"
#include "../Src/RigidBodyDynamics/RigidBody.h"
#include "../Src/Maths/Maths.h"

using namespace Riemann;

namespace
{
	struct CollisionSample
	{
		const char* Name;
		PrimitiveType Type;
		Geometry* Geom;
	};

	struct RaySpec
	{
		Vector3 Origin;
		Vector3 Direction;
	};

	Geometry* CreateConvexCube(const Vector3& Center)
	{
		static Vector3 Vertices[] =
		{
			Vector3(-0.5f, -0.5f, -0.5f),
			Vector3( 0.5f, -0.5f, -0.5f),
			Vector3( 0.5f,  0.5f, -0.5f),
			Vector3(-0.5f,  0.5f, -0.5f),
			Vector3(-0.5f, -0.5f,  0.5f),
			Vector3( 0.5f, -0.5f,  0.5f),
			Vector3( 0.5f,  0.5f,  0.5f),
			Vector3(-0.5f,  0.5f,  0.5f),
		};
		static ConvexMeshFace Faces[] =
		{
			ConvexMeshFace(Plane3(Vector3(-1.0f,  0.0f,  0.0f), -0.5f), 4,  0),
			ConvexMeshFace(Plane3(Vector3( 1.0f,  0.0f,  0.0f), -0.5f), 4,  4),
			ConvexMeshFace(Plane3(Vector3( 0.0f, -1.0f,  0.0f), -0.5f), 4,  8),
			ConvexMeshFace(Plane3(Vector3( 0.0f,  1.0f,  0.0f), -0.5f), 4, 12),
			ConvexMeshFace(Plane3(Vector3( 0.0f,  0.0f, -1.0f), -0.5f), 4, 16),
			ConvexMeshFace(Plane3(Vector3( 0.0f,  0.0f,  1.0f), -0.5f), 4, 20),
		};
		static uint8_t Indices[] =
		{
			0, 3, 7, 4,
			1, 5, 6, 2,
			0, 4, 5, 1,
			3, 2, 6, 7,
			0, 1, 2, 3,
			4, 7, 6, 5,
		};

		Geometry* geom = GeometryFactory::CreateConvexMesh();
		ConvexMesh* convex = geom->GetShapeObj<ConvexMesh>();
		convex->SetConvexData(Vertices, 8, Faces, 6, nullptr, 0, Indices, 24, true);
		geom->SetShapeBounds(convex->Bounds);
		geom->SetTransform(Center, Quaternion::One());
		return geom;
	}

	Geometry* CreateTriangleMeshBox(const Vector3& Center)
	{
		Geometry* geom = GeometryFactory::CreateTriangleMesh();
		TriangleMesh* mesh = geom->GetShapeObj<TriangleMesh>();
		mesh->AddAABB(Vector3(-0.5f), Vector3(0.5f));
		mesh->Compact();
		mesh->BuildBVH();
		geom->SetShapeBounds(mesh->BoundingVolume);
		geom->SetTransform(Center, Quaternion::One());
		return geom;
	}

	Geometry* CreateHeightFieldPatch(const Vector3& Center)
	{
		Geometry* geom = GeometryFactory::CreateHeightField(Box3(Vector3(-1.0f, -0.1f, -1.0f), Vector3(1.0f, 0.1f, 1.0f)), 3, 3);
		HeightField3* hf = geom->GetShapeObj<HeightField3>();
		hf->AllocMemory();
		geom->SetShapeBounds(hf->BV);
		geom->SetTransform(Center, Quaternion::One());
		return geom;
	}

	void CreateCollisionSamples(const Vector3& Center, std::vector<CollisionSample>& Samples)
	{
		Samples.emplace_back(CollisionSample{ "box", PrimitiveType::BOX, GeometryFactory::CreateOBB(Center, Vector3(0.5f), Quaternion::One()) });
		Samples.emplace_back(CollisionSample{ "plane", PrimitiveType::PLANE, GeometryFactory::CreatePlane(Center, Vector3::UnitY()) });
		Samples.emplace_back(CollisionSample{ "sphere", PrimitiveType::SPHERE, GeometryFactory::CreateSphere(Center, 0.5f) });
		Samples.emplace_back(CollisionSample{ "capsule", PrimitiveType::CAPSULE, GeometryFactory::CreateCapsule(Center + Vector3(0.0f, -0.5f, 0.0f), Center + Vector3(0.0f, 0.5f, 0.0f), 0.35f) });
		Samples.emplace_back(CollisionSample{ "cylinder", PrimitiveType::CYLINDER, GeometryFactory::CreateCylinder(Center + Vector3(0.0f, -0.5f, 0.0f), Center + Vector3(0.0f, 0.5f, 0.0f), 0.35f) });
		Samples.emplace_back(CollisionSample{ "heightfield", PrimitiveType::HEIGHTFIELD, CreateHeightFieldPatch(Center) });
		Samples.emplace_back(CollisionSample{ "convex", PrimitiveType::CONVEX_MESH, CreateConvexCube(Center) });
		Samples.emplace_back(CollisionSample{ "triangle_mesh", PrimitiveType::TRIANGLE_MESH, CreateTriangleMeshBox(Center) });
	}

	void DeleteCollisionSamples(std::vector<CollisionSample>& Samples)
	{
		for (CollisionSample& Sample : Samples)
		{
			GeometryFactory::DeleteGeometry(Sample.Geom);
			Sample.Geom = nullptr;
		}
		Samples.clear();
	}

	bool SameTypePair(PrimitiveType A, PrimitiveType B, PrimitiveType X, PrimitiveType Y)
	{
		return (A == X && B == Y) || (A == Y && B == X);
	}

	bool ExpectIntersectHit(PrimitiveType A, PrimitiveType B)
	{
		if (SameTypePair(A, B, PrimitiveType::PLANE, PrimitiveType::HEIGHTFIELD)) return false;
		if (SameTypePair(A, B, PrimitiveType::PLANE, PrimitiveType::TRIANGLE_MESH)) return false;
		if (SameTypePair(A, B, PrimitiveType::CYLINDER, PrimitiveType::HEIGHTFIELD)) return false;
		if (SameTypePair(A, B, PrimitiveType::CYLINDER, PrimitiveType::TRIANGLE_MESH)) return false;
		if (SameTypePair(A, B, PrimitiveType::CONVEX_MESH, PrimitiveType::HEIGHTFIELD)) return false;
		if (SameTypePair(A, B, PrimitiveType::CONVEX_MESH, PrimitiveType::TRIANGLE_MESH)) return false;
		if (SameTypePair(A, B, PrimitiveType::HEIGHTFIELD, PrimitiveType::HEIGHTFIELD)) return false;
		if (SameTypePair(A, B, PrimitiveType::HEIGHTFIELD, PrimitiveType::TRIANGLE_MESH)) return false;
		if (SameTypePair(A, B, PrimitiveType::TRIANGLE_MESH, PrimitiveType::TRIANGLE_MESH)) return false;
		return true;
	}

	bool ExpectSweepHit(PrimitiveType Type)
	{
		return Type == PrimitiveType::BOX ||
			Type == PrimitiveType::SPHERE ||
			Type == PrimitiveType::CAPSULE ||
			Type == PrimitiveType::CYLINDER;
	}

	RaySpec GetRaySpec(PrimitiveType Type)
	{
		if (Type == PrimitiveType::PLANE || Type == PrimitiveType::HEIGHTFIELD)
		{
			return RaySpec{ Vector3(0.0f, 3.0f, 0.0f), -Vector3::UnitY() };
		}
		if (Type == PrimitiveType::CONVEX_MESH)
		{
			return RaySpec{ Vector3(-1.0f, 0.0f, 0.0f), Vector3::UnitX() };
		}
		return RaySpec{ Vector3(-3.0f, 0.0f, 0.0f), Vector3::UnitX() };
	}

	RaySpec GetMissRaySpec(PrimitiveType Type)
	{
		if (Type == PrimitiveType::PLANE)
		{
			return RaySpec{ Vector3(0.0f, 3.0f, 0.0f), Vector3::UnitY() };
		}
		if (Type == PrimitiveType::HEIGHTFIELD)
		{
			return RaySpec{ Vector3(3.0f, 3.0f, 0.0f), -Vector3::UnitY() };
		}
		return RaySpec{ Vector3(-3.0f, 2.5f, 0.0f), Vector3::UnitX() };
	}

	bool CanRunFiniteBoundaryCase(PrimitiveType A, PrimitiveType B)
	{
		if (SameTypePair(A, B, PrimitiveType::CAPSULE, PrimitiveType::TRIANGLE_MESH))
		{
			return false;
		}
		return ExpectIntersectHit(A, B) &&
			A != PrimitiveType::PLANE &&
			B != PrimitiveType::PLANE &&
			A != PrimitiveType::HEIGHTFIELD &&
			B != PrimitiveType::HEIGHTFIELD;
	}
}

void TestPlane()
{
	printf("Running TestPlane\n");

	Plane3 plane0, plane1, plane2;
	bool success;
	Vector3 p;

	plane0 = Plane3(Vector3(6.51947312e-06f, -0.0494406559f, -0.998777092f), -7.89754963f);
	plane1 = Plane3(Vector3(0.0f, 0.0f, -1.0f), -8.10573006f);
	plane2 = Plane3(Vector3(7.3720279e-05f, -0.559060335f, -0.829126954f), -4.48326111f);
	success = Plane3::IntersectPlanes(plane0, plane1, plane2, &p);
	EXPECT(!success);

	plane0 = Plane3(Vector3::UnitX(), 0.0f);
	plane1 = Plane3(Vector3::UnitY(), 0.0f);
	plane2 = Plane3(Vector3::UnitZ(), 0.0f);

	success = Plane3::IntersectPlanes(plane0, plane1, plane2, &p);
	EXPECT(success);
	EXPECT((p - Vector3::Zero()).SquareLength() < 1e-6f);

	plane0.Shift(0.1f);
	plane1.Shift(0.1f);
	plane2.Shift(0.1f);

	success = Plane3::IntersectPlanes(plane0, plane1, plane2, &p);
	EXPECT(success);
	EXPECT((p - Vector3(0.1f, 0.1f, 0.1f)).SquareLength() < 1e-6f);
	return;
}

void TestTriangle()
{
	printf("Running TestTriangle\n");

	Triangle3 tri1(Vector3::Zero(), Vector3::UnitX(), Vector3::UnitY());
	Vector3 p = Vector3::One();
	Vector3 pp = tri1.GetCenter();

	Vector3 bc2 = tri1.BarycentricCoods(p);
	Triangle3::PointDistanceQueryResult bc3 = tri1.PointDistanceQuery(pp);

	Vector3 p2 = tri1.BarycentricCoodsComposion(bc2);
	Vector3 p3 = tri1.BarycentricCoodsComposion(bc3.BaryCoords);

	(void)p2;
	
	EXPECT((p3 - pp).SquareLength() < 1e-6f);

	EXPECT(!tri1.IntersectPoint(p));
	EXPECT(tri1.IntersectPoint(tri1.v0));
	EXPECT(tri1.IntersectPoint(pp));
	EXPECT(!tri1.IntersectPoint(Vector3(0.1f, 0.1f, -0.1f)));
	EXPECT(!tri1.IntersectPoint(Vector3(0.1f, 0.1f, 0.1f)));
	EXPECT(tri1.IntersectPoint(Vector3(0.1f, 0.1f, 0.0f)));
	EXPECT(tri1.IntersectSegment(Vector3(0.1f, 0.1f, 0.1f), Vector3(0.1f, 0.1f, -0.1f)));
	EXPECT(!tri1.IntersectSegment(Vector3(0.1f, 0.1f, 0.1f), Vector3(0.1f, 0.1f, 0.2f)));
	EXPECT(!tri1.IntersectSegment(Vector3(0.1f, -0.1f, 0.1f), Vector3(0.1f, -0.1f, -0.1f)));

	Triangle3 tri2(Vector3(5.09916496f, 8.30379868f, 4.52991295f), Vector3(5.08997154f, 8.29810333f, 4.52174377f), Vector3(5.09456825f, 8.300951f, 4.52582836f));
	Vector3 length = tri2.GetSideLength();
	Vector3 normal = tri2.GetNormal();
    (void)length;
    (void)normal;
	EXPECT(tri2.IsDegenerate());
	EXPECT(Triangle3::IsColinear(tri2.v0, tri2.v1, tri2.v2));

	Triangle3 tri3(Vector3(-0.0680909976f, 0.00185490400f, 0.0297227483f), Vector3(-0.0693324953f, -0.00129310042f, 0.0196812488f), Vector3(-0.0677610636f, -0.00000000f, 0.0282528419f));
	p = Vector3(-0.0688225254f, -0.00000000f, 0.0238059796f);
	bc2 = tri1.BarycentricCoods(p);
	float dist = tri1.DistanceToPoint(p);
	EXPECT(fabsf(dist) > 0.001f);
	EXPECT(!tri1.IntersectPoint(p));
	return;
}

void TestSphere()
{
	printf("Running TestSphere\n");

	Vector3 a = Vector3(0, 0, 0);
	Vector3 b = Vector3(0, 1, 0);
	Vector3 c = Vector3(1, 0, 0);
	Vector3 d = Vector3(1, 1, 0);
	Sphere3 s(a, b, c, d);

	EXPECT(s.IntersectPoint(a));
	EXPECT(s.IntersectPoint(b));
	EXPECT(s.IntersectPoint(b));
	EXPECT(s.IntersectPoint(c));
	
	int n = 1000;
	std::vector<Vector3> points;
	for (int i = 0 ;i < n; ++i)
	{
		points.push_back(10.0f * Vector3::Random());
	}
	Sphere3 s1 = Sphere3::ComputeBoundingSphere_Welzl(points.data(), n);
	for (int i = 0 ;i < n; ++i)
	{
		EXPECT(s1.IntersectPoint(points[i]));
	}
	
	return;
}

void TestDynamicAABB()
{
	printf("Running TestDynamicAABB\n");
	
	DynamicAABBTree tree;
	
	struct Actor
	{
		Actor(Geometry*_geom, int _id)
		{
			p = _geom;
			id = _id;
		}
		Geometry*p;
		int		id;
	};
	
	std::vector<Actor> geoms;
	for (int i = 0; i < 128; ++i)
	{
		Geometry* obb = GeometryFactory::CreateOBB(Vector3::Random() * 10.0f, Vector3::One(), Quaternion::One());
		int id = tree.Add(obb->GetBounds(), obb);
		EXPECT(tree.Validate());
		geoms.emplace_back(obb, id);
		
	}
	
	for (int k = 0; k < 1000; ++k)
	{
		for (size_t i = 0; i < geoms.size(); ++i)
		{
			int r = Maths::RandomInt(0, 5);
			if (geoms[i].id == -1)
			{
				if (r == 0)
				{
					geoms[i].id = tree.Add(geoms[i].p->GetBounds(), geoms[i].p);
				}
			}
			else
			{
				if (r == 0)
				{
					tree.Remove(geoms[i].id);
					geoms[i].id = -1;
				}
				else
				{
					geoms[i].p->SetPosition(Vector3::Random() * 10.0f);
					geoms[i].p->UpdateBounds();
					tree.Update(geoms[i].id, geoms[i].p->GetBounds(), Vector3::UnitY());
				}
			}
			
			EXPECT(tree.Validate());
		}
	}
	
	tree.Rebuild();
	
	return;
}

void TestSupport()
{
	printf("Running TestSupport\n");
	Geometry* obb2 = GeometryFactory::CreateOBB(Vector3(0.0f, 0.0, 0.0f), Vector3::One(), Quaternion::One());
	
	Vector3 support;

	support = obb2->GetSupport(Vector3::UnitX());
	EXPECT(fabsf(support.x - 1.0f) < 0.1f);
	
	support = obb2->GetSupport(-Vector3::UnitX());
	EXPECT(fabsf(support.x + 1.0f) < 0.1f);
	
	support = obb2->GetSupport(Vector3::UnitY());
	EXPECT(fabsf(support.y - 1.0f) < 0.1f);
	
	obb2->SetPosition(Vector3(5.0f, 0.0f, 0.0f));
	support = obb2->GetSupport(Vector3::UnitX());
	EXPECT(fabsf(support.x - 6.0f) < 0.1f);
	
	Quaternion quat;
	quat.FromRotationAxis(Vector3::UnitZ(), PI_OVER_4);
	obb2->SetRotation(quat);
	support = obb2->GetSupport(Vector3::UnitX());
	EXPECT(fabsf(support.x - (5.0f + SQRT_2)) < 0.1f);
	support = obb2->GetSupport(Vector3::UnitY());
	EXPECT(fabsf(support.y - SQRT_2) < 0.1f);
	return;
}

void TestGJK()
{
	printf("Running TestGJK\n");
	
	Geometry* plane1 = GeometryFactory::CreatePlane(Vector3(0.0f, 0.0f, 0.0f), Vector3::UnitY());
	Geometry* obb1 = GeometryFactory::CreateOBB(Vector3(0.0f, 0.0, 0.0f), Vector3::One(), Quaternion::One());
	Geometry* obb2 = GeometryFactory::CreateOBB(Vector3(0.0f, 0.0, 0.0f), Vector3::One(), Quaternion::One());
	EXPECT(GJK_Solve(obb1, obb2));

	obb2->SetPosition(Vector3(0.5f, 0.0f, 0.0f));
	EXPECT(GJK_Solve(obb1, obb2));
	
	obb2->SetPosition(Vector3(0.0f, 2.1f, 0.0f));
	EXPECT(!GJK_Solve(obb1, obb2));
	
	EXPECT(GJK_Solve(obb1, plane1));
	EXPECT(GJK_Solve(plane1, obb1));
	EXPECT(!GJK_Solve(plane1, obb2));
	EXPECT(!GJK_Solve(obb2, plane1));
	
	Quaternion quat;
	quat.FromRotationAxis(Vector3::UnitX(), PI_OVER_4);
	obb2->SetRotation(quat);
	EXPECT(GJK_Solve(obb2, obb1));
	EXPECT(GJK_Solve(obb1, obb2));
	
	obb2->SetPosition(Vector3(0.0f, 1.1f, 0.0f));
	EXPECT(GJK_Solve(obb2, plane1));
	EXPECT(GJK_Solve(plane1, obb2));
	
	Geometry* sp1 = GeometryFactory::CreateSphere(Vector3(0.0f, 0.0, 0.0f), 2.0f);
	Geometry* sp2 = GeometryFactory::CreateSphere(Vector3(0.0f, 0.0, 0.0f), 2.0f);
	EXPECT(GJK_Solve(sp1, sp2));
	EXPECT(GJK_Solve(sp1, plane1));
	
	sp2->SetPosition(Vector3(0.0f, 5.0f, 0.0f));
	EXPECT(!GJK_Solve(sp1, sp2));
	EXPECT(!GJK_Solve(plane1, sp2));

	float dist = GJK_Solve_Distance(sp1, sp2);
	EXPECT(fabsf(dist - 1.0f) < 0.01f);

	return;
}

void TestGJKRaycast()
{
	printf("Running TestGJKRaycast\n");

	float t = -1.0f;
	Geometry* obb1 = GeometryFactory::CreateOBB(Vector3(2.0f, 1.0, 0.0f), Vector3(1.0f, 1.0f, 1.0f), Quaternion::One());
	EXPECT(GJK_Solve_Raycast(Vector3(-5.0f, 1.0f, 0.0f), Vector3(1.0f, 0.0f, 0.0f), obb1, &t) && Maths::FloatEqual(t, 6.0f));
	EXPECT(!GJK_Solve_Raycast(Vector3(0.0f, 2.01f, 0.0f), Vector3(1.0f, 0.0f, 0.0f), obb1, &t));
	EXPECT(GJK_Solve_Raycast(Vector3(0.0f, 1.99f, 0.0f), Vector3(1.0f, 0.0f, 0.0f), obb1, &t));
	return;
}

void TestEPA()
{
	printf("Running TestEPA\n");

	// GeometryBase* plane1 = GeometryFactory::CreatePlane(Vector3(0.0f, -5.0f, 0.0f), Vector3::UnitY(), 1.0f);
	Geometry* plane1 = GeometryFactory::CreateOBB(Vector3(0.0f, -5.0f, 0.0f), Vector3(100.0f, 1.0f, 100.0f), Quaternion::One());
	Geometry* obb1 = GeometryFactory::CreateOBB(Vector3(0.0f, 0.0, 0.0f), Vector3::One() * 1.0f, Quaternion::One());
	obb1->SetPosition(Vector3(0.0f, -3.7f, 0.0f));

	GJKIntersection gjk;
	GJK_status gjk_status;

	EPAPenetration epa;
	EPA_status epa_status;

	GeometryDifference shape(plane1, obb1);

	gjk_status = gjk.Solve(&shape);
	EXPECT(gjk_status == GJK_status::Intersect);

	epa_status = epa.Solve(gjk.result);
	EXPECT(epa_status == EPA_status::AccuraryReached);

	obb1->SetPosition(Vector3(20.0f, -3.7f, 0.0f));
	gjk_status = gjk.Solve(&shape);
	EXPECT(gjk_status == GJK_status::Intersect);
	epa_status = epa.Solve(gjk.result);
	EXPECT(epa_status == EPA_status::AccuraryReached);

	obb1->SetPosition(Vector3(-20.0f, -3.7f, 0.0f));
	gjk_status = gjk.Solve(&shape);
	EXPECT(gjk_status == GJK_status::Intersect);
	epa_status = epa.Solve(gjk.result);
	EXPECT(epa_status == EPA_status::AccuraryReached);
	
	Geometry *sp0 = GeometryFactory::CreateSphere(Vector3(1.0f, 1.0f, 0.0f), 1.5f);
	Geometry *sp1 = GeometryFactory::CreateSphere(Vector3(3.0f, 1.0f, 0.0f), 1.0f);
	shape = GeometryDifference(sp0, sp1);
	EXPECT(gjk.Solve(&shape) == GJK_status::Intersect);
	EXPECT(epa.Solve(gjk.result) != EPA_status::Failed);
	
	Vector3 normal;
	float depth;
	EXPECT(sp0->Penetration(sp1, &normal, &depth));
	
	EXPECT(normal.Dot(epa.penetration_normal) > 0.99f);
	EXPECT(fabsf(depth - epa.penetration_depth) < 0.001f);
	
	return;
}

void TestRTree1()
{
	printf("Running TestRTree1\n");
	TriangleMesh mesh;
	mesh.AddAABB(Vector3(-1, -1, -1), Vector3(1, 1, 1));
	mesh.AddAABB(Vector3(-0.5f, -0.5f, -0.5f), Vector3(0.5f, 0.5f, 0.5f));
	mesh.Compact();
	mesh.BuildBVH();
	float t;
	bool success;
	success = mesh.IntersectRay(Vector3(0.0f, 10.0f, 0.0f), -Vector3::UnitY(), &t);
	EXPECT(success);
	EXPECT(Maths::FloatEqual(t, 9.0f));

	success = mesh.IntersectRay(Vector3(2.0f, 10.0f, 0.0f), -Vector3::UnitY(), &t);
	EXPECT(!success);

	success = mesh.IntersectRay(Vector3(0.0f, 0.75f, 0.0f), Vector3::UnitY(), &t);
	EXPECT(success);
	EXPECT(Maths::FloatEqual(t, 0.25f));

	success = mesh.IntersectRay(Vector3(0.0f, 0.0f, 0.0f), Vector3(1, 1, 1).Unit(), &t);
	EXPECT(success);
	EXPECT(Maths::FloatEqual(t, sqrtf(3.0f) * 0.5f));

	return;
}

void TestAABB()
{
	printf("Running TestAABB\n");
	Box3 box1(Vector3(0, 0, 0), Vector3(10, 10, 10));
	Box3 box2(Vector3(4, 4, 4), Vector3(5, 5, 5));
	bool intersect1 = box1.Intersect(box2);
	bool intersect2 = box2.Intersect(box1);
	EXPECT(intersect1 && intersect2);
}

void TestRaycast()
{
	printf("Running TestRaycast\n");

	Vector3 center(-4.19f, 0.43f, -0.46f);
	Vector3 dir(-0.08f, -0.14f, 0.99f);
	AxisAlignedBox3 aabb(Vector3(-4.30f, -0.81f, 1.58f), Vector3(-2.70f, 0.80f, 2.59f));

	float t;
	bool success1 = aabb.IntersectRay(center, dir, &t);
	// Vector3 p1 = center + dir * t;
	EXPECT(!success1);

	bool success2 = Ray3::RayIntersectAABB(center, dir, aabb.Min, aabb.Max, &t);
	// Vector3 p2 = center + dir * t;
	EXPECT(!success2);

	return;
}

void TestBuildOBB()
{
	printf("Running TestBuildOBB\n");
	
	AxisAlignedBox3 aabb(Vector3(-10.0f, -2.5f, -1.0f), Vector3(10.0f, 2.5f, 1.0f));
	Matrix3 mat;
	Vector3 center(1, 1, 2);
	
	mat.LoadRotateZ(PI_OVER_6);
	std::vector<Vector3> verties, normals;
	std::vector<uint16_t> Indices;
	
	aabb.GetMesh(verties, Indices, normals);
	for (size_t i = 0; i < verties.size(); ++i)
	{
		verties[i] = mat * verties[i] + center;
	}
	
	OrientedBox3 obb = OrientedBox3::ComputeBoundingOBB_PCA(verties.data(), (int)verties.size());
	
	EXPECT(fabsf(obb.Center.x - center.x) < 0.1f);
	EXPECT(fabsf(obb.Center.y - center.y) < 0.1f);
	EXPECT(fabsf(obb.Center.z - center.z) < 0.1f);
	
	Vector3 dir = obb.Rotation.Column(0);
	float dp = dir.Dot(Vector3::UnitX());
	EXPECT(dp >= 0.8f);

	return;
}

void TestOBB()
{
	printf("Running TestOBB\n");

	{
		OrientedBox3 obb1(Vector3(0, 0, 0), Vector3(1, 1, 1), Matrix3::Identity());
		OrientedBox3 obb2(Vector3(1, 1, 1), Vector3(5, 4, 4), Matrix3::Identity());
		obb1.Rotation.LoadRotateY(PI_OVER_3);
		obb2.Rotation.LoadRotateX(PI_OVER_3);
		bool intersect = obb1.IntersectOBB(obb2);
		EXPECT(intersect);

		obb2.Center.x = 10.0f;

		intersect = obb1.IntersectOBB(obb2);
		EXPECT(!intersect);
	}

	{
		Geometry* obb1 = GeometryFactory::CreateOBB(Vector3(0.0f, 0.0, 0.0f), Vector3::One(), Quaternion::One());
		Geometry* obb2 = GeometryFactory::CreateOBB(Vector3(2.5f, 0.0, 0.0f), Vector3::One(), Quaternion::One());

		EXPECT(!obb1->Intersect(obb2));

		Quaternion quat;
		quat.FromRotateZ(PI_OVER_4);
		obb2->SetRotation(quat);
		EXPECT(!obb1->Intersect(obb2));

		obb1->SetRotation(quat);
		EXPECT(obb1->Intersect(obb2));
	}

	return;
}

void TestIntersect()
{
	printf("Running TestIntersect\n");
	Geometry* plane1 = GeometryFactory::CreatePlane(Vector3(0.0f, 0.0f, 0.0f), Vector3::UnitY());
	Geometry* obb1 = GeometryFactory::CreateOBB(Vector3(0.0f, 0.0, 0.0f), Vector3::One(), Quaternion::One());
	Geometry* obb2 = GeometryFactory::CreateOBB(Vector3(0.0f, 0.0, 0.0f), Vector3::One(), Quaternion::One());
	EXPECT(obb1->Intersect(obb2));
	EXPECT(obb2->Intersect(obb1));

	obb2->SetPosition(Vector3(0.5f, 0.0f, 0.0f));
	EXPECT(obb1->Intersect(obb2));
	EXPECT(obb2->Intersect(obb1));
	
	obb2->SetPosition(Vector3(0.0f, 2.1f, 0.0f));
	EXPECT(!obb1->Intersect(obb2));
	EXPECT(!obb2->Intersect(obb1));
	
	EXPECT(obb1->Intersect(plane1));
	EXPECT(plane1->Intersect(obb1));
	EXPECT(!obb2->Intersect(plane1));
	EXPECT(!plane1->Intersect(obb2));
	
	Quaternion quat;
	quat.FromRotationAxis(Vector3::UnitX(), PI_OVER_4);
	obb2->SetRotation(quat);
	EXPECT(obb1->Intersect(obb2));
	EXPECT(obb2->Intersect(obb1));
	
	obb2->SetPosition(Vector3(0.0f, 1.1f, 0.0f));
	EXPECT(plane1->Intersect(obb2));
	EXPECT(obb2->Intersect(plane1));
	
	Geometry* sp1 = GeometryFactory::CreateSphere(Vector3(0.0f, 0.0, 0.0f), 2.0f);
	Geometry* sp2 = GeometryFactory::CreateSphere(Vector3(1.0f, 0.0, 0.0f), 2.0f);
	EXPECT(sp1->Intersect(sp2));
	EXPECT(sp2->Intersect(sp1));
	EXPECT(sp1->Intersect(plane1));
	EXPECT(plane1->Intersect(sp1));
	
	sp2->SetPosition(Vector3(0.0f, 5.0f, 0.0f));
	EXPECT(!sp1->Intersect(sp2));
	EXPECT(!sp2->Intersect(sp1));
	EXPECT(!plane1->Intersect(sp2));
	EXPECT(!sp2->Intersect(plane1));
}

void TestGeometryRaycastTypes()
{
	printf("Running TestGeometryRaycastTypes\n");

	std::vector<CollisionSample> samples;
	CreateCollisionSamples(Vector3::Zero(), samples);

	RayCastOption option;
	option.HitBothSides = true;
	option.MaxDist = 100.0f;

	for (const CollisionSample& sample : samples)
	{
		EXPECT(GeometryIntersection::GetRayCastFunc(sample.Type) != nullptr);

		RaySpec spec = GetRaySpec(sample.Type);
		RayCastResult result;
		const bool hit = sample.Geom->RayCast(spec.Origin, spec.Direction, &option, &result);
		if (!hit || !(result.hitTime >= 0.0f && result.hitTime < option.MaxDist))
		{
			printf("Raycast mismatch: %s hit=%d t=%f\n", sample.Name, hit ? 1 : 0, result.hitTime);
		}
		EXPECT(hit);
		EXPECT(result.hitTime >= 0.0f && result.hitTime < option.MaxDist);

		RayCastOption clippedOption = option;
		clippedOption.MaxDist = result.hitTime * 0.5f;
		RayCastResult clippedResult;
		const bool clippedHit = sample.Geom->RayCast(spec.Origin, spec.Direction, &clippedOption, &clippedResult);
		if (clippedHit)
		{
			printf("Raycast MaxDist boundary mismatch: %s t=%f max=%f\n", sample.Name, clippedResult.hitTime, clippedOption.MaxDist);
		}
		EXPECT(!clippedHit);

		RaySpec missSpec = GetMissRaySpec(sample.Type);
		RayCastResult missResult;
		const bool missHit = sample.Geom->RayCast(missSpec.Origin, missSpec.Direction, &option, &missResult);
		if (missHit)
		{
			printf("Raycast miss mismatch: %s t=%f\n", sample.Name, missResult.hitTime);
		}
		EXPECT(!missHit);
	}

	DeleteCollisionSamples(samples);
}

void TestGeometryIntersectMatrix()
{
	printf("Running TestGeometryIntersectMatrix\n");

	std::vector<CollisionSample> lhsSamples;
	std::vector<CollisionSample> overlapSamples;
	std::vector<CollisionSample> separatedSamples;
	CreateCollisionSamples(Vector3::Zero(), lhsSamples);
	CreateCollisionSamples(Vector3::Zero(), overlapSamples);
	CreateCollisionSamples(Vector3(0.0f, 10.0f, 0.0f), separatedSamples);

	for (const CollisionSample& lhs : lhsSamples)
	{
		for (size_t rhsIndex = 0; rhsIndex < overlapSamples.size(); ++rhsIndex)
		{
			const CollisionSample& rhs = overlapSamples[rhsIndex];
			IntersectFunc func = GeometryIntersection::GetIntersectFunc(lhs.Type, rhs.Type);
			IntersectFunc reverseFunc = GeometryIntersection::GetIntersectFunc(rhs.Type, lhs.Type);
			EXPECT(func != nullptr || reverseFunc != nullptr);

			const bool hit = lhs.Geom->Intersect(rhs.Geom);
			const bool expected = ExpectIntersectHit(lhs.Type, rhs.Type);
			if (hit != expected)
			{
				printf("Intersect mismatch: %s vs %s hit=%d expected=%d\n", lhs.Name, rhs.Name, hit ? 1 : 0, expected ? 1 : 0);
			}
			EXPECT(hit == expected);

			const CollisionSample& separated = separatedSamples[rhsIndex];
			const bool separatedHit = lhs.Geom->Intersect(separated.Geom);
			if (separatedHit)
			{
				printf("Intersect separated mismatch: %s vs %s\n", lhs.Name, separated.Name);
			}
			EXPECT(!separatedHit);
		}
	}

	const float boundaryEps = 0.05f;
	for (const CollisionSample& lhs : lhsSamples)
	{
		for (CollisionSample& rhs : overlapSamples)
		{
			if (!CanRunFiniteBoundaryCase(lhs.Type, rhs.Type))
			{
				continue;
			}

			const Box3& lhsBox = lhs.Geom->GetBounds();
			const Box3& rhsLocalBox = rhs.Geom->GetShapeBounds();
			const float touchY = lhsBox.Max.y - rhsLocalBox.Min.y;

			rhs.Geom->SetPosition(Vector3(0.0f, touchY + boundaryEps, 0.0f));
			const bool nearMiss = lhs.Geom->Intersect(rhs.Geom);
			if (nearMiss)
			{
				printf("Intersect boundary gap mismatch: %s vs %s\n", lhs.Name, rhs.Name);
			}
			EXPECT(!nearMiss);

			rhs.Geom->SetPosition(Vector3(0.0f, touchY - boundaryEps, 0.0f));
			const bool nearHit = lhs.Geom->Intersect(rhs.Geom);
			if (!nearHit)
			{
				printf("Intersect boundary overlap mismatch: %s vs %s\n", lhs.Name, rhs.Name);
			}
			EXPECT(nearHit);

			rhs.Geom->SetPosition(Vector3::Zero());
		}
	}

	DeleteCollisionSamples(separatedSamples);
	DeleteCollisionSamples(overlapSamples);
	DeleteCollisionSamples(lhsSamples);
}

void TestGeometrySweepTypes()
{
	printf("Running TestGeometrySweepTypes\n");

	std::vector<CollisionSample> samples;
	CreateCollisionSamples(Vector3::Zero(), samples);
	Geometry* target = GeometryFactory::CreatePlane(Vector3::Zero(), Vector3::UnitY());

	for (const CollisionSample& sample : samples)
	{
		EXPECT(GeometryIntersection::GetSweepFunc(sample.Type, target->GetShapeType()) != nullptr);

		Vector3 position = Vector3::Zero();
		Vector3 normal = Vector3::Zero();
		float t = -1.0f;
		const bool expected = ExpectSweepHit(sample.Type);
		sample.Geom->SetPosition(Vector3(0.0f, 5.0f, 0.0f));
		const bool hit = sample.Geom->Sweep(-Vector3::UnitY(), target, &position, &normal, &t);
		if (hit != expected)
		{
			printf("Sweep mismatch: %s hit=%d expected=%d t=%f normal2=%f\n", sample.Name, hit ? 1 : 0, expected ? 1 : 0, t, normal.SquareLength());
		}
		EXPECT(hit == expected);
		if (expected)
		{
			EXPECT(t >= 0.0f && t < 100.0f);
			EXPECT(normal.SquareLength() > 0.0f);

			position = Vector3::Zero();
			normal = Vector3::Zero();
			t = -1.0f;
			const bool awayHit = sample.Geom->Sweep(Vector3::UnitY(), target, &position, &normal, &t);
			if (awayHit)
			{
				printf("Sweep away mismatch: %s t=%f\n", sample.Name, t);
			}
			EXPECT(!awayHit);

			const float touchY = -sample.Geom->GetShapeBounds().Min.y;
			sample.Geom->SetPosition(Vector3(0.0f, touchY, 0.0f));
			position = Vector3::Zero();
			normal = Vector3::Zero();
			t = -1.0f;
			const bool touchHit = sample.Geom->Sweep(-Vector3::UnitY(), target, &position, &normal, &t);
			if (!touchHit || fabsf(t) > 0.001f)
			{
				printf("Sweep touch mismatch: %s hit=%d t=%f\n", sample.Name, touchHit ? 1 : 0, t);
			}
			EXPECT(touchHit);
			EXPECT(fabsf(t) <= 0.001f);
			EXPECT(normal.SquareLength() > 0.0f);
		}
	}

	GeometryFactory::DeleteGeometry(target);
	DeleteCollisionSamples(samples);
}

void TestRayAABB()
{
	printf("Running TestRayAABB\n");
	Vector3 Origin(-100, 0, 50);
	Vector3 Dir(1, 0, 0);
	float t0 = 0, t1 = 0;
	bool success;
	EXPECT(Ray3::RayIntersectAABB2(Origin, Dir, Vector3::Zero(), Vector3(100, 100, 100), 0.00001f, 100000.0f, &t0, &t1));

	// Vector3 InPos = Origin + Dir * t0;
	// Vector3 OutPos = Origin + Dir * t1;

	// int x0 = (int)(InPos.x / 1.0f);
	// int x1 = (int)(OutPos.x / 1.0f);

	Triangle3 Tri(Vector3(0, 1, 0), Vector3(0, 0, 0), Vector3(1, 0, 0));
	success = Tri.IntersectAABB(Vector3(-1, -1, -1), Vector3(1, 1, 1));
	EXPECT(success);

	success = Tri.IntersectAABB(Vector3(1, 1, 1), Vector3(2, 2, 2));
	EXPECT(!success);

	return;
}

void TestAABBTree()
{
	printf("Running TestAABBTree\n");
	std::vector<Box3> boxes;
	boxes.emplace_back(Vector3(0, 0, 0), Vector3(1, 1, 1));
	boxes.emplace_back(Vector3(0, 0, 0), Vector3(1, 1, 3));
	boxes.emplace_back(Vector3(1, 1, 1), Vector3(1, 1, 2));

	AABBTree tree;

	AABBTreeBuildData param(&boxes[0], (int)boxes.size(), 1);

	tree.Release();
	tree.StaticBuild(param);

	int p = tree.IntersectPoint(Vector3(-1.0f, -1.0f, -1.0f));
	EXPECT(p == -1);
	p = tree.IntersectPoint(Vector3(0.5f, 0.5f, 0.5f));
	EXPECT(p >= 0);
	p = tree.IntersectPoint(Vector3(0.5f, 0.5f, 2.5f));
	EXPECT(p == 1);

	RayCastOption Option;
	RayCastResult Result;
	Ray3 ray(Vector3(0.5f, 0.5f, 100.0f), Vector3(0.0f, 0.0f, -1.0f));
	bool hit = tree.RayCastBoundingBox(ray, Option, &Result);
	EXPECT(hit);

	ray.Origin = Vector3(0.5f, 0.5f, 0.5f);
	hit = tree.RayCastBoundingBox(ray, Option, &Result);
	EXPECT(hit);

	for (int i = 0; i < 10000; ++i)
	{
		Vector3 point1 = Vector3::Random() * 100.0f;
		Vector3 point2 = Vector3::Random() * 100.0f;
		boxes.emplace_back(point1, point1 + point2);
	}

	AABBTreeBuildData param2(&boxes[0], (int)boxes.size(), 1);
	param2.splitter = SplitHeuristic::TreeBalance;
	
	tree.Release();
	tree.StaticBuild(param2);
	TreeStatistics stat;
	tree.Statistic(stat);

	for (int i = 0; i < 10000; ++i)
	{
		Vector3 point = Vector3::Random() * 100.0f;
		p = tree.IntersectPoint(point);

		ray.Origin = point;
		hit = tree.RayCastBoundingBox(ray, Option, &Result);
		if (hit)
		{
			Box3 bb = boxes[hit];
			// float t;
			// EXPECT(ray.IntersectAABB(bb.mMin, bb.mMax, &t));		TODO
		}
	}

	return;
}

void TestGeometryQuery()
{
	printf("Running TestGeometryQuery\n");
	GeometryQuery scene;
	
	std::vector<Geometry*> objs;
	objs.emplace_back(GeometryFactory::CreatePlane(Vector3(0.0f, 0.0f, 0.0f), Vector3::UnitZ()));
	objs.emplace_back(GeometryFactory::CreatePlane(Vector3(0.0f, 0.0f, 10.0f), Vector3::UnitZ()));
	objs.emplace_back(GeometryFactory::CreateOBB(Vector3::Zero(), Vector3(1, 1, 1)));
	scene.BuildStaticGeometry(objs, 1);

	RayCastResult result;
	RayCastOption option;
	scene.RayCastQuery(Vector3(0.2f, 0.2f, 5.0f), Vector3(0.2f, 0.2f, -1.0f), option , &result);
	EXPECT(result.hit);
	EXPECT(fabsf(result.hitTimeMin - 4.0f) < 0.001f);

	scene.RayCastQuery(Vector3(0.0f, 0.0f, -5.0f), Vector3(0.0f, 0.0f, -1.0f), option, &result);
	EXPECT(!result.hit);

	scene.RayCastQuery(Vector3(0.0f, 0.0f, 15.0f), Vector3(0.0f, 0.0f, -1.0f), option, &result);
	EXPECT(result.hit);
	EXPECT(fabsf(result.hitTimeMin - 5.0f) < 0.001f);

	option.Type = RayCastOption::RayCastType::RAYCAST_PENETRATE;
	scene.RayCastQuery(Vector3(0.0f, 0.0f, 15.0f), Vector3(0.0f, 0.0f, -1.0f), option, &result);
	EXPECT(result.hit);
	EXPECT(result.hitGeometries.size() == 3);

	return;
}



class BVProxy2 : public SAP::BoundingVolumeProxy
{
public:
	BVProxy2(std::vector<Box3>* objs)
	{
		m_objs = objs;
	}

	virtual int	 GetBoundingVolumeCount() const override
	{
		return (int)m_objs->size();
	}

	virtual float* GetBoundingVolumeCoordinate(int bv_i, bool left, int axis) const override
	{
		const Box3& box = m_objs->at(bv_i);
		float* p = (float*)&box;
		return left ? p + axis : p + 3 + axis;
	}

	virtual bool	Overlaps(int bv_i, int bv_j) const override
	{
		const Box3& box1 = m_objs->at(bv_i);
		const Box3& box2 = m_objs->at(bv_j);
		return box1.Intersect(box2);
	}

	virtual uint64_t	CalculateBoundingVolumeHash() const override
	{
		return 0;
	}

	std::vector<Box3>* m_objs;
};

void TestSAP()
{
	printf("Running TestSAP\n");
	std::set<OverlapKey> overlaps;
	std::vector<Box3> boxes;
	boxes.emplace_back(Vector3(0, 0, 0), Vector3(2, 2, 2));
	boxes.emplace_back(Vector3(1, 1, 1), Vector3(3, 3, 3));
	boxes.emplace_back(Vector3(0, 0, 0), Vector3(15, 15, 15));

	BVProxy2 P(&boxes);
	SAP sap(&P, { 0, 1, 2 });

	sap.Prune(&overlaps);
	EXPECT(overlaps.size() == 3);

	boxes[2] = Box3(Vector3(10, 10, 10), Vector3(15, 15, 15));
	sap.Prune(&overlaps);
	EXPECT(overlaps.size() == 1);

	for (int i = 0; i < 200; ++i)
	{
		Vector3 point1 = Vector3::Random() * 100.0f;
		Vector3 point2 = Vector3::Random() * 100.0f;
		boxes.emplace_back(point1, point1 + point2);
	}
	sap.Prune(&overlaps);

	for (size_t i = 0; i < boxes.size(); ++i)
	for (size_t j = 0; j < boxes.size(); ++j)
	{
		if (i == j) continue;
		OverlapKey key = SAP::PackOverlapKey((int)i, (int)j);
		if (boxes[i].Intersect(boxes[j]))
		{
			EXPECT(overlaps.count(key) == 1);
		}
		else
		{
			EXPECT(overlaps.count(key) == 0);
		}
	}

	return;
}

class BVProxy : public SAP::BoundingVolumeProxy
{
public:
	BVProxy(std::vector<Geometry*>* objs)
	{
		m_objs = objs;
	}

	virtual int	 GetBoundingVolumeCount() const override
	{
		return (int)m_objs->size();
	}

	virtual float* GetBoundingVolumeCoordinate(int bv_i, bool left, int axis) const override
	{
		const Box3& box = m_objs->at(bv_i)->GetBounds();
		float* p = (float*)&box;
		return left ? p + axis : p + 3 + axis;
	}

	virtual bool	Overlaps(int bv_i, int bv_j) const override
	{
		const Box3& box1 = m_objs->at(bv_i)->GetBounds();
		const Box3& box2 = m_objs->at(bv_j)->GetBounds();
		return box1.Intersect(box2);
	}

	virtual uint64_t	CalculateBoundingVolumeHash() const override
	{
		return 0;
	}

	std::vector<Geometry*>* m_objs;
};

void TestSAPInc()
{
	printf("Running TestSAPInc\n");
	std::vector<Geometry*> boxes;
	boxes.emplace_back(GeometryFactory::CreateOBB(Vector3(0.5f, 0.5f, 0.5f), Vector3(0.5f, 0.5f, 0.5f)));
	boxes.emplace_back(GeometryFactory::CreateOBB(Vector3(2.5f, 2.5f, 2.5f), Vector3(0.5f, 0.5f, 0.5f)));
	boxes.emplace_back(GeometryFactory::CreateOBB(Vector3(15, 15, 15), Vector3(5, 5, 5)));

	BVProxy P(&boxes);
	IncrementalSAP sap(&P, { 0, 1, 2 });

	std::set<OverlapKey> overlaps;

	sap.IncrementalPrune(&overlaps);
	EXPECT(overlaps.size() == 0);

	boxes[2]->SetPosition(boxes[2]->GetPosition() + Vector3(-10, -10, -10));
	boxes[2]->UpdateBounds();
	sap.IncrementalPrune(&overlaps);
	EXPECT(overlaps.size() == 2);

	sap.IncrementalPrune(&overlaps);
	EXPECT(overlaps.size() == 2);

	boxes[2]->SetPosition(boxes[2]->GetPosition() + Vector3(10, 10, 10));
	boxes[2]->UpdateBounds();
	sap.IncrementalPrune(&overlaps);
	EXPECT(overlaps.size() == 0);

	for (int i = 0; i < 100; ++i)
	{
		Vector3 point1 = Vector3::Random() * 100.0f;
		Vector3 point2 = Vector3::Random() * 100.0f;
		boxes.emplace_back(GeometryFactory::CreateOBB(point1, point2));
	}
	sap.SetDirty();

	for (int k = 0; k < 10; ++k)
	{
		boxes[2]->SetPosition(boxes[2]->GetPosition() + Vector3::Random() * 20.0f);
		boxes[2]->UpdateBounds();
		sap.IncrementalPrune(&overlaps);

		for (size_t i = 0; i < boxes.size(); ++i)
		for (size_t j = 0; j < boxes.size(); ++j)
		{
			if (i == j) continue;
			OverlapKey key = SAP::PackOverlapKey((int)i, (int)j);
			if (boxes[i]->GetBounds().Intersect(boxes[j]->GetBounds()))
			{
				EXPECT(overlaps.count(key) == 1);
			}
			else
			{
				EXPECT(overlaps.count(key) == 0);
			}
		}
	}

	std::vector<Box3> degenerateBoxes;
	degenerateBoxes.emplace_back(Vector3(-1.0f, -1.0f, -1.0f), Vector3(1.0f, 1.0f, 1.0f));
	degenerateBoxes.emplace_back(Vector3(1.0f, -1.0f, -1.0f), Vector3(1.0f, 1.0f, 1.0f));
	degenerateBoxes.emplace_back(Vector3(3.0f, -1.0f, -1.0f), Vector3(3.0f, 1.0f, 1.0f));

	BVProxy2 DegenerateProxy(&degenerateBoxes);
	IncrementalSAP degenerateSap(&DegenerateProxy, { 0, 1, 2 });
	overlaps.clear();
	degenerateSap.IncrementalPrune(&overlaps);
	EXPECT(overlaps.count(SAP::PackOverlapKey(0, 1)) == 1);
	EXPECT(overlaps.count(SAP::PackOverlapKey(0, 2)) == 0);
	EXPECT(overlaps.count(SAP::PackOverlapKey(1, 2)) == 0);

	degenerateBoxes[2] = Box3(Vector3(1.0f, -1.0f, -1.0f), Vector3(1.0f, 1.0f, 1.0f));
	degenerateSap.IncrementalPrune(&overlaps);
	EXPECT(overlaps.count(SAP::PackOverlapKey(0, 1)) == 1);
	EXPECT(overlaps.count(SAP::PackOverlapKey(0, 2)) == 1);
	EXPECT(overlaps.count(SAP::PackOverlapKey(1, 2)) == 1);


	return;
}

static std::set<OverlapKey> ToBroadPhasePairSet(const std::vector<OverlapPair>& pairs)
{
	std::set<OverlapKey> result;
	for (const OverlapPair& pair : pairs)
	{
		result.insert(SAP::PackOverlapKey(pair.index1, pair.index2));
	}
	return result;
}

static GeometryWorldState MakeGeometryWorldState(Geometry* geom)
{
	GeometryWorldState state;
	state.Geom = geom;
	state.FrameId = 1;
	state.Version = 1;
	state.Moved = true;
	state.WorldBounds = Box3::Empty();
	if (geom == nullptr)
	{
		return state;
	}

	state.Body = geom->GetParent<RigidBody>();
	if (state.Body)
	{
		state.BodyToWorld = Transform(state.Body->BodyLocalToWorld(Vector3::Zero()), state.Body->Q);
		state.ShapeToWorld = state.Body->GetGeometryTransform(geom);
		state.WorldBounds = Box3::Transform(geom->GetShapeBounds(), state.ShapeToWorld.pos, state.ShapeToWorld.quat);
		state.MovingRigid = state.Body->mRigidType != RigidType::Static;
	}
	else
	{
		state.BodyToWorld = Transform::Identity();
		state.ShapeToWorld = *geom->GetTransform();
		state.WorldBounds = geom->GetBounds();
		state.MovingRigid = false;
	}
	return state;
}

static std::vector<GeometryWorldState> MakeGeometryWorldStates(const std::vector<Geometry*>& geoms)
{
	std::vector<GeometryWorldState> states;
	states.reserve(geoms.size());
	for (Geometry* geom : geoms)
	{
		GeometryWorldState state = MakeGeometryWorldState(geom);
		state.Handle = GeometryHandle((uint32_t)states.size(), 1);
		states.push_back(state);
	}
	return states;
}

void TestPhysXBroadPhasePorts()
{
	printf("Running TestPhysXBroadPhasePorts\n");
	std::vector<RigidBody*> bodies;
	std::vector<Geometry*> geoms;

	for (int i = 0; i < 650; ++i)
	{
		const float x = (float)(i % 25) * 0.75f;
		const float y = (float)((i / 25) % 13) * 0.75f;
		const float z = (float)(i / 325) * 0.75f;
		const Vector3 center(x, y, z);

		RigidBodyParam param;
		param.rigidType = (i % 3) == 0 ? RigidType::Dynamic : RigidType::Static;
		RigidBody* body = RigidBody::CreateRigidBody(param, Transform(center));
		body->AddGeometry(GeometryFactory::CreateOBB(center, Vector3(0.42f)));
		bodies.push_back(body);
		body->GetGeometries(&geoms);
	}

	RigidBodyParam dynamicParam;
	dynamicParam.rigidType = RigidType::Dynamic;
	RigidBody* largeBody = RigidBody::CreateRigidBody(dynamicParam, Transform(Vector3(9.0f, 4.0f, 0.4f)));
	largeBody->AddGeometry(GeometryFactory::CreateOBB(Vector3(9.0f, 4.0f, 0.4f), Vector3(2.5f, 2.0f, 0.8f)));
	bodies.push_back(largeBody);
	largeBody->GetGeometries(&geoms);

	std::vector<GeometryWorldState> states = MakeGeometryWorldStates(geoms);

	BroadPhase* bruteforce = BroadPhase::Create_Bruteforce();
	BroadPhase* sap = BroadPhase::Create_SAP();
	BroadPhase* abp = BroadPhase::Create_ABP();
	BroadPhase* mbp = BroadPhase::Create_MBP();
	BroadPhase* dynamicAABB = BroadPhase::Create_DynamicAABB();

	std::vector<OverlapPair> brutePairs;
	std::vector<OverlapPair> sapPairs;
	std::vector<OverlapPair> abpPairs;
	std::vector<OverlapPair> mbpPairs;
	std::vector<OverlapPair> dynamicAABBPairs;
	bruteforce->ProduceOverlaps(states, &brutePairs);
	sap->ProduceOverlaps(states, &sapPairs);
	abp->ProduceOverlaps(states, &abpPairs);
	mbp->ProduceOverlaps(states, &mbpPairs);
	dynamicAABB->ProduceOverlaps(states, &dynamicAABBPairs);

	const std::set<OverlapKey> bruteSet = ToBroadPhasePairSet(brutePairs);
	EXPECT(ToBroadPhasePairSet(sapPairs) == bruteSet);
	EXPECT(ToBroadPhasePairSet(abpPairs) == bruteSet);
	EXPECT(ToBroadPhasePairSet(mbpPairs) == bruteSet);
	EXPECT(ToBroadPhasePairSet(dynamicAABBPairs) == bruteSet);

	delete bruteforce;
	delete sap;
	delete abp;
	delete mbp;
	delete dynamicAABB;

	for (RigidBody* body : bodies)
	{
		body->ReleaseGeometries();
		delete body;
	}
}

void TestPhysicsWorldSceneQueryTrees()
{
	printf("Running TestPhysicsWorldSceneQueryTrees\n");

	{
		PhysicsWorldParam param;
		param.broadphase = BroadPhaseSolver::DynamicAABB;
		param.sceneQueryType = SceneQueryType::StaticAABB;
		PhysicsWorld world(param);
		EXPECT(world.GetGeometryQuery()->GetDynamicTree() == nullptr);

		RigidBodyParam bodyParam;
		bodyParam.rigidType = RigidType::Dynamic;
		Geometry* geom = GeometryFactory::CreateSphere(Vector3::Zero(), 1.0f);
		RigidBody* body = world.CreateRigidBody(geom, bodyParam);
		EXPECT(body != nullptr);
		const GeometryQueryProxy* proxy = world.GetGeometryQuery()->GetProxy(geom);
		EXPECT(proxy != nullptr);
		EXPECT(proxy->PrunerHandle == -1);
		world.Simulate();
		EXPECT(world.GetGeometryQuery()->GetDynamicTree() == nullptr);

		EXPECT(world.RemoveRigidBody(body));
		body->ReleaseGeometries();
		delete body;
	}

	{
		PhysicsWorldParam param;
		param.broadphase = BroadPhaseSolver::DynamicAABB;
		param.sceneQueryType = SceneQueryType::DynamicAABB;
		PhysicsWorld world(param);
		EXPECT(world.GetGeometryQuery()->GetDynamicTree() != nullptr);

		RigidBodyParam bodyParam;
		bodyParam.rigidType = RigidType::Dynamic;
		Geometry* geom = GeometryFactory::CreateSphere(Vector3::Zero(), 1.0f);
		RigidBody* body = world.CreateRigidBody(geom, bodyParam);
		EXPECT(body != nullptr);
		const GeometryQueryProxy* proxy = world.GetGeometryQuery()->GetProxy(geom);
		EXPECT(proxy != nullptr);
		EXPECT(proxy->PrunerHandle >= 0);
		world.Simulate();
		proxy = world.GetGeometryQuery()->GetProxy(geom);
		EXPECT(proxy != nullptr);
		EXPECT(proxy->PrunerHandle >= 0);

		EXPECT(world.RemoveRigidBody(body));
		EXPECT(world.GetGeometryQuery()->GetProxy(geom) == nullptr);
		body->ReleaseGeometries();
		delete body;
	}

	{
		PhysicsWorldParam param;
		param.sceneQueryType = SceneQueryType::DynamicAABB;
		PhysicsWorld world(param);

		RigidBodyParam bodyParam;
		bodyParam.rigidType = RigidType::Static;
		Geometry* geom = GeometryFactory::CreateSphere(Vector3::Zero(), 1.0f);
		RigidBody* body = world.CreateRigidBody(geom, bodyParam);
		EXPECT(body != nullptr);
		const GeometryQueryProxy* proxy = world.GetGeometryQuery()->GetProxy(geom);
		EXPECT(proxy != nullptr);
		EXPECT(proxy->PrunerHandle >= 0);

		RayCastOption option;
		option.Type = RayCastOption::RAYCAST_NEAREST;
		RayCastResult result;
		const bool hit = world.GetGeometryQuery()->RayCastQuery(Vector3(0.0f, 0.0f, 5.0f), Vector3(0.0f, 0.0f, -1.0f), option, &result);
		EXPECT(hit);
		EXPECT(result.hit);
		EXPECT(result.hitGeom == geom);

		EXPECT(world.RemoveRigidBody(body));
		body->ReleaseGeometries();
		delete body;
	}

	{
		PhysicsWorldParam param;
		param.sceneQueryType = SceneQueryType::DynamicAABB;
		PhysicsWorld world(param);

		RigidBodyParam bodyParam;
		bodyParam.rigidType = RigidType::Static;
		Geometry* geom = GeometryFactory::CreateSphere(Vector3::Zero(), 1.0f);
		geom->SetQueryEnabled(false);
		RigidBody* body = world.CreateRigidBody(geom, bodyParam);
		EXPECT(body != nullptr);
		EXPECT(world.GetGeometryQuery()->GetProxy(geom) == nullptr);

		std::vector<Box3> queryAABBs;
		world.GetGeometryQuery()->CollectAABBs(&queryAABBs);
		EXPECT(queryAABBs.empty());

		RayCastOption option;
		RayCastResult result;
		const bool hit = world.GetGeometryQuery()->RayCastQuery(Vector3(0.0f, 0.0f, 5.0f), Vector3(0.0f, 0.0f, -1.0f), option, &result);
		EXPECT(!hit);
		EXPECT(!result.hit);

		EXPECT(world.RemoveRigidBody(body));
		body->ReleaseGeometries();
		delete body;
	}

	{
		RigidBodyParam dynamicParam;
		dynamicParam.rigidType = RigidType::Dynamic;
		Geometry* dynamicGeom = GeometryFactory::CreateSphere(Vector3::Zero(), 1.0f);
		RigidBody* dynamicBody = RigidBody::CreateRigidBody(dynamicParam, Transform());
		dynamicBody->AddGeometry(dynamicGeom);

		RigidBodyParam staticParam;
		staticParam.rigidType = RigidType::Static;
		Geometry* sensorGeom = GeometryFactory::CreateSphere(Vector3::Zero(), 1.0f);
		sensorGeom->SetSimulationEnabled(false);
		RigidBody* staticBody = RigidBody::CreateRigidBody(staticParam, Transform());
		staticBody->AddGeometry(sensorGeom);

		std::vector<Geometry*> geoms;
		geoms.push_back(dynamicGeom);
		geoms.push_back(sensorGeom);

		BroadPhase* broadphase = BroadPhase::Create_Bruteforce();
		std::vector<GeometryWorldState> states = MakeGeometryWorldStates(geoms);
		std::vector<OverlapPair> overlaps;
		broadphase->ProduceOverlaps(states, &overlaps);
		EXPECT(overlaps.empty());
		delete broadphase;

		GeometryQuery query;
		query.CreateDynamicGeometry();
		EXPECT(query.AddGeometry(sensorGeom));

		RayCastOption option;
		RayCastResult result;
		const bool hit = query.RayCastQuery(Vector3(0.0f, 0.0f, 5.0f), Vector3(0.0f, 0.0f, -1.0f), option, &result);
		EXPECT(hit);
		EXPECT(result.hitGeom == sensorGeom);

		staticBody->ReleaseGeometries();
		dynamicBody->ReleaseGeometries();
		delete staticBody;
		delete dynamicBody;
	}
}

void TestPhysicsWorldSceneQueryRigidBodyShapeTransform()
{
	printf("Running TestPhysicsWorldSceneQueryRigidBodyShapeTransform\n");

	for (int i = 0; i < 2; ++i)
	{
		PhysicsWorldParam param;
		param.sceneQueryType = i == 0 ? SceneQueryType::StaticAABB : SceneQueryType::DynamicAABB;
		PhysicsWorld world(param);

		RigidBodyParam bodyParam;
		bodyParam.rigidType = RigidType::Static;
		Geometry* geom = GeometryFactory::CreateSphere(Vector3(11.0f, 0.0f, 0.0f), 1.0f);
		std::vector<Geometry*> geoms;
		geoms.push_back(geom);

		RigidBody* body = world.CreateRigidBody(geoms, bodyParam, Transform(Vector3(10.0f, 0.0f, 0.0f), Quaternion::One()));
		EXPECT(body != nullptr);
		EXPECT(geom->GetPosition() == Vector3(1.0f, 0.0f, 0.0f));

		RayCastOption option;
		option.Type = RayCastOption::RAYCAST_NEAREST;
		option.HitBothSides = true;

		RayCastResult worldResult;
		const bool worldHit = world.GetGeometryQuery()->RayCastQuery(Vector3(11.0f, 0.0f, 5.0f), Vector3(0.0f, 0.0f, -1.0f), option, &worldResult);
		EXPECT(worldHit);
		EXPECT(worldResult.hitGeom == geom);

		RayCastResult localResult;
		const bool localHit = world.GetGeometryQuery()->RayCastQuery(Vector3(1.0f, 0.0f, 5.0f), Vector3(0.0f, 0.0f, -1.0f), option, &localResult);
		EXPECT(!localHit);

		EXPECT(world.RemoveRigidBody(body));
		body->ReleaseGeometries();
		delete body;
	}
}


void TestCollision()
{
	TestTriangle();
	TestRaycast();
	TestPlane();
	TestSphere();
	TestBuildOBB();
	TestOBB();
	TestIntersect();
	TestGeometryRaycastTypes();
	TestGeometryIntersectMatrix();
	TestGeometrySweepTypes();
	TestDynamicAABB();
	TestSupport();
	TestGJK();
	TestGJKRaycast();
	TestEPA();
	TestAABB();
	TestRayAABB();
	TestRTree1();
	TestAABBTree();
	TestGeometryQuery();
	TestSAP();
	TestSAPInc();
	TestPhysXBroadPhasePorts();
	TestPhysicsWorldSceneQueryTrees();
	TestPhysicsWorldSceneQueryRigidBodyShapeTransform();
	return;
}
