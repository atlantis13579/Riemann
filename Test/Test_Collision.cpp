
#include "Test.h"

#include <chrono>
#include <cfloat>
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
#include "../Src/RigidBodyDynamics/LCPSolver.h"
#include "../Src/RigidBodyDynamics/PhysicsWorld.h"
#include "../Src/RigidBodyDynamics/RigidBody.h"
#include "../Src/Deformation/Rope.h"
#include "../Src/Deformation/Cloth.h"
#include "../Src/RigidBodyDynamics/Ragdoll.h"
#include "../Src/Maths/Maths.h"

using namespace Riemann;

namespace
{
	float GetGeometryMinimumY(const RigidBody* body, const Geometry* geometry)
	{
		const Transform transform = body->GetGeometryTransform(geometry);
		switch (geometry->GetShapeType())
		{
		case PrimitiveType::BOX:
		{
			const AxisAlignedBox3* box = geometry->GetShapeObj<AxisAlignedBox3>();
			const Vector3 center = transform.LocalToWorld(box->GetCenter());
			const Vector3 extent = box->GetExtent();
			const Matrix3 rotation = transform.quat.ToRotationMatrix3();
			const float verticalRadius = std::abs(rotation(1, 0)) * extent.x
				+ std::abs(rotation(1, 1)) * extent.y + std::abs(rotation(1, 2)) * extent.z;
			return center.y - verticalRadius;
		}
		case PrimitiveType::SPHERE:
		{
			const Sphere3* sphere = geometry->GetShapeObj<Sphere3>();
			return transform.LocalToWorld(sphere->Center).y - sphere->Radius;
		}
		case PrimitiveType::CAPSULE:
		{
			const Capsule3* capsule = geometry->GetShapeObj<Capsule3>();
			return std::min(transform.LocalToWorld(capsule->X0).y,
				transform.LocalToWorld(capsule->X1).y) - capsule->Radius;
		}
		default:
			return body->GetGeometryBounds(geometry).Min.y;
		}
	}

	float GetMaximumRagdollAnchorError(const Ragdoll* ragdoll)
	{
		float maximumError = 0.0f;
		for (PositionConstraint* constraint : ragdoll->GetConstraints())
		{
			PointConstraint* pointConstraint = dynamic_cast<PointConstraint*>(constraint);
			if (pointConstraint)
			{
				maximumError = std::max(maximumError,
					(pointConstraint->GetWorldAnchorB() - pointConstraint->GetWorldAnchorA()).Length());
			}
		}
		return maximumError;
	}

	float EstimateMinimumSeparationDistance(const RigidBody* body, const Geometry* geometry,
		const RigidBody* obstacleBody, const Geometry* obstacleGeometry)
	{
		const Transform shapeTransform = body->GetGeometryTransform(geometry);
		const Transform obstacleTransform = obstacleBody->GetGeometryTransform(obstacleGeometry);
		if (!geometry->Intersect(shapeTransform, obstacleGeometry, obstacleTransform))
		{
			return 0.0f;
		}
		const Vector3 axes[6] = {
			obstacleTransform.LocalToWorldDirection(Vector3::UnitX()),
			-obstacleTransform.LocalToWorldDirection(Vector3::UnitX()),
			obstacleTransform.LocalToWorldDirection(Vector3::UnitY()),
			-obstacleTransform.LocalToWorldDirection(Vector3::UnitY()),
			obstacleTransform.LocalToWorldDirection(Vector3::UnitZ()),
			-obstacleTransform.LocalToWorldDirection(Vector3::UnitZ())
		};
		float minimumDistance = FLT_MAX;
		for (const Vector3& axis : axes)
		{
			const Vector3 direction = axis.SafeUnit();
			float lower = 0.0f;
			float upper = 0.005f;
			Transform testTransform = shapeTransform;
			testTransform.pos += direction * upper;
			while (upper < 1.0f && geometry->Intersect(testTransform, obstacleGeometry, obstacleTransform))
			{
				lower = upper;
				upper *= 2.0f;
				testTransform = shapeTransform;
				testTransform.pos += direction * upper;
			}
			if (geometry->Intersect(testTransform, obstacleGeometry, obstacleTransform))
			{
				continue;
			}
			for (int iteration = 0; iteration < 10; ++iteration)
			{
				const float middle = (lower + upper) * 0.5f;
				testTransform = shapeTransform;
				testTransform.pos += direction * middle;
				if (geometry->Intersect(testTransform, obstacleGeometry, obstacleTransform))
				{
					lower = middle;
				}
				else
				{
					upper = middle;
				}
			}
			minimumDistance = std::min(minimumDistance, upper);
		}
		return minimumDistance;
	}

	void ExpectRagdollAngularLimits(const Ragdoll* ragdoll)
	{
		for (PositionConstraint* constraint : ragdoll->GetConstraints())
		{
			if (constraint->GetType() == PositionConstraintType::Cone)
			{
				ConeConstraint* cone = static_cast<ConeConstraint*>(constraint);
				EXPECT(cone->GetCurrentAngle() <= cone->GetMaxAngle() + 1.0f * PI_OVER_180);
			}
			else if (constraint->GetType() == PositionConstraintType::Hinge)
			{
				HingeConstraint* hinge = static_cast<HingeConstraint*>(constraint);
				EXPECT(hinge->GetCurrentAngle() >= hinge->GetMinAngle() - 1.0f * PI_OVER_180);
				EXPECT(hinge->GetCurrentAngle() <= hinge->GetMaxAngle() + 1.0f * PI_OVER_180);
			}
		}
	}

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

	{
		Geometry* translatedMesh = CreateTriangleMeshBox(Vector3(3.0f, 0.0f, 2.0f));
		Geometry* translatedSphere = GeometryFactory::CreateSphere(Vector3(3.0f, 3.0f, 2.0f), 0.5f);
		Vector3 localPosition;
		Vector3 localNormal;
		float hitTime = FLT_MAX;
		EXPECT(translatedSphere->Sweep(-Vector3::UnitY(), translatedMesh, &localPosition, &localNormal, &hitTime));
		EXPECT(std::abs(hitTime - 2.0f) < 0.01f);
		const Vector3 worldPosition = translatedSphere->GetTransform()->LocalToWorld(localPosition);
		const Vector3 worldNormal = translatedSphere->GetTransform()->LocalToWorldDirection(localNormal);
		EXPECT((worldPosition - Vector3(3.0f, 0.5f, 2.0f)).Length() < 0.01f);
		EXPECT(worldNormal.y > 0.99f);
		GeometryFactory::DeleteGeometry(translatedSphere);
		GeometryFactory::DeleteGeometry(translatedMesh);
	}

	{
		Geometry* translatedHeightField = CreateHeightFieldPatch(Vector3(-3.0f, 1.0f, -2.0f));
		Geometry* translatedCapsule = GeometryFactory::CreateCapsule(
			Vector3(-3.0f, 3.5f, -2.0f), Vector3(-3.0f, 4.5f, -2.0f), 0.25f);
		Vector3 localPosition;
		Vector3 localNormal;
		float hitTime = FLT_MAX;
		EXPECT(translatedCapsule->Sweep(-Vector3::UnitY(), translatedHeightField, &localPosition, &localNormal, &hitTime));
		EXPECT(std::abs(hitTime - 2.25f) < 0.01f);
		const Vector3 worldPosition = translatedCapsule->GetTransform()->LocalToWorld(localPosition);
		const Vector3 worldNormal = translatedCapsule->GetTransform()->LocalToWorldDirection(localNormal);
		EXPECT((worldPosition - Vector3(-3.0f, 1.0f, -2.0f)).Length() < 0.01f);
		EXPECT(worldNormal.y > 0.99f);
		GeometryFactory::DeleteGeometry(translatedCapsule);
		GeometryFactory::DeleteGeometry(translatedHeightField);
	}
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

		std::vector<GeometryWorldState> states = MakeGeometryWorldStates(geoms);
		BroadPhase* broadphases[] = { BroadPhase::Create_Bruteforce(), BroadPhase::Create_AllPairs() };
		for (BroadPhase* broadphase : broadphases)
		{
			std::vector<OverlapPair> overlaps;
			broadphase->ProduceOverlaps(states, &overlaps);
			EXPECT(overlaps.empty());

			sensorGeom->SetTriggerEnabled(true);
			broadphase->ProduceOverlaps(states, &overlaps);
			EXPECT(overlaps.size() == 1);
			sensorGeom->SetTriggerEnabled(false);

			sensorGeom->SetSimulationEnabled(true);
			broadphase->ProduceOverlaps(states, &overlaps);
			EXPECT(overlaps.size() == 1);
			sensorGeom->SetSimulationEnabled(false);
			delete broadphase;
		}

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

void TestGlobalLCPResetWarmStartState()
{
	printf("Running TestGlobalLCPResetWarmStartState\n");

	PhysicsWorldParam param;
	param.gravityAcc = Vector3(0.0f, -9.8f, 0.0f);
	param.broadphase = BroadPhaseSolver::SAP;
	param.resolutionSolver = ResolutionPhaseSolver::LCPGlobal;
	param.solverBackend = SolverBackend::CPU;
	param.contactSolverIterations = 32;

	auto CreateContactScene = [](PhysicsWorld& World) -> RigidBodyDynamic*
	{
		RigidBodyParam groundParam;
		groundParam.rigidType = RigidType::Static;
		RigidBody* ground = World.CreateRigidBody(
			GeometryFactory::CreatePlane(Vector3::Zero(), Vector3::UnitY()), groundParam);
		EXPECT(ground != nullptr);

		RigidBodyParam bodyParam;
		bodyParam.rigidType = RigidType::Dynamic;
		bodyParam.linearVelocity = Vector3(0.0f, -1.0f, 0.0f);
		bodyParam.sleepThreshold = 0.0f;
		RigidBody* body = World.CreateRigidBody(GeometryFactory::CreateOBB(
			Vector3(0.0f, 0.49f, 0.0f), Vector3(0.5f)), bodyParam);
		EXPECT(body != nullptr && body->CastDynamic() != nullptr);
		return body ? body->CastDynamic() : nullptr;
	};

	PhysicsWorld resetWorld(param);
	RigidBodyDynamic* primedBody = CreateContactScene(resetWorld);
	EXPECT(primedBody != nullptr);
	for (int frame = 0; frame < 30; ++frame)
	{
		resetWorld.Simulate();
	}
	resetWorld.Reset();
	RigidBodyDynamic* resetBody = CreateContactScene(resetWorld);
	EXPECT(resetBody != nullptr);
	resetWorld.Simulate();

	PhysicsWorld freshWorld(param);
	RigidBodyDynamic* freshBody = CreateContactScene(freshWorld);
	EXPECT(freshBody != nullptr);
	freshWorld.Simulate();

	if (resetBody && freshBody)
	{
		EXPECT_NEAR(resetBody->X.x, freshBody->X.x, 1.0e-6f);
		EXPECT_NEAR(resetBody->X.y, freshBody->X.y, 1.0e-6f);
		EXPECT_NEAR(resetBody->X.z, freshBody->X.z, 1.0e-6f);
		EXPECT_NEAR(resetBody->V.x, freshBody->V.x, 1.0e-6f);
		EXPECT_NEAR(resetBody->V.y, freshBody->V.y, 1.0e-6f);
		EXPECT_NEAR(resetBody->V.z, freshBody->V.z, 1.0e-6f);
		EXPECT_NEAR(resetBody->W.Length(), freshBody->W.Length(), 1.0e-6f);
	}
}

void TestSleepingBodyWakesForAppliedLoads()
{
	printf("Running TestSleepingBodyWakesForAppliedLoads\n");

	PhysicsWorldParam worldParam;
	worldParam.gravityAcc = Vector3::Zero();
	PhysicsWorld world(worldParam);

	RigidBodyParam bodyParam;
	bodyParam.rigidType = RigidType::Dynamic;
	bodyParam.disableGravity = true;
	bodyParam.sleepThreshold = 0.0f;
	RigidBody* rigidBody = world.CreateRigidBody(
		GeometryFactory::CreateOBB(Vector3::Zero(), Vector3(0.5f)), bodyParam);
	EXPECT(rigidBody != nullptr && rigidBody->CastDynamic() != nullptr);
	if (!rigidBody || !rigidBody->CastDynamic())
	{
		return;
	}

	RigidBodyDynamic* body = rigidBody->CastDynamic();
	body->Sleep();
	EXPECT(body->Sleeping);
	body->ApplyForce(Vector3::Zero());
	EXPECT(body->Sleeping);
	body->ApplyForce(Vector3(1.0e-6f, 0.0f, 0.0f));
	EXPECT(!body->Sleeping);
	body->ExtForce.SetZero();
	body->Sleep();
	body->ApplyTorgue(Vector3::UnitX(), Vector3::UnitX());
	EXPECT(body->Sleeping);
	body->ApplyForce(Vector3(1.0f, 0.0f, 0.0f));
	EXPECT(!body->Sleeping);
	world.Simulate();
	EXPECT(body->V.x > 0.0f);

	body->Sleep();
	EXPECT(body->Sleeping);
	body->ApplyTorgue(Vector3(0.0f, 0.0f, 1.0f));
	EXPECT(!body->Sleeping);
	world.Simulate();
	EXPECT(body->W.z > 0.0f);

	auto RunCallbackLoadTest = [](ResolutionPhaseSolver Solver, bool ApplyFromTrigger)
	{
		RigidBodyDynamic* callbackBody = nullptr;
		bool callbackAppliedLoad = false;
		auto ApplyLoad = [&](RigidBody* BodyA, RigidBody* BodyB)
		{
			if (!callbackAppliedLoad && callbackBody
				&& (BodyA == callbackBody || BodyB == callbackBody))
			{
				callbackBody->ApplyForce(Vector3(10.0f, 0.0f, 0.0f));
				callbackAppliedLoad = true;
			}
		};

		PhysicsWorldParam callbackWorldParam;
		callbackWorldParam.gravityAcc = Vector3::Zero();
		callbackWorldParam.resolutionSolver = Solver;
		callbackWorldParam.solverBackend = SolverBackend::CPU;
		callbackWorldParam.onContact = [&](const PhysicsContactInfo& contact)
		{
			if (!ApplyFromTrigger)
			{
				ApplyLoad(contact.BodyA, contact.BodyB);
			}
		};
		callbackWorldParam.onTrigger = [&](const PhysicsTriggerInfo& trigger)
		{
			if (ApplyFromTrigger)
			{
				ApplyLoad(trigger.BodyA, trigger.BodyB);
			}
		};
		PhysicsWorld callbackWorld(callbackWorldParam);
		RigidBodyParam groundParam;
		groundParam.rigidType = RigidType::Static;
		EXPECT(callbackWorld.CreateRigidBody(
			GeometryFactory::CreatePlane(Vector3::Zero(), Vector3::UnitY()), groundParam) != nullptr);
		if (ApplyFromTrigger)
		{
			Geometry* triggerGeometry = GeometryFactory::CreateOBB(
				Vector3(0.0f, 0.49f, 0.0f), Vector3(0.75f));
			triggerGeometry->SetTriggerEnabled(true);
			EXPECT(callbackWorld.CreateRigidBody(triggerGeometry, groundParam) != nullptr);
		}
		RigidBodyParam callbackBodyParam;
		callbackBodyParam.rigidType = RigidType::Dynamic;
		callbackBodyParam.disableGravity = true;
		callbackBodyParam.sleepThreshold = 1000.0f;
		RigidBody* callbackRigidBody = callbackWorld.CreateRigidBody(
			GeometryFactory::CreateOBB(Vector3(0.0f, 0.49f, 0.0f), Vector3(0.5f)),
			callbackBodyParam);
		callbackBody = callbackRigidBody ? callbackRigidBody->CastDynamic() : nullptr;
		EXPECT(callbackBody != nullptr);
		if (callbackBody)
		{
			callbackWorld.Simulate();
			EXPECT(callbackAppliedLoad);
			EXPECT(!callbackBody->Sleeping);
			EXPECT(callbackBody->ExtForce.x > 0.0f);
			callbackBody->SleepThreshold = 0.0f;
			callbackWorld.Simulate();
			EXPECT(callbackBody->V.x > 0.0f);
		}
	};

	RunCallbackLoadTest(ResolutionPhaseSolver::SequentialImpulse, false);
	RunCallbackLoadTest(ResolutionPhaseSolver::LCPGlobal, false);
	RunCallbackLoadTest(ResolutionPhaseSolver::SequentialImpulse, true);
}

void TestFiveBoxStackStability()
{
	printf("Running TestFiveBoxStackStability\n");

	PhysicsWorldParam param;
	param.gravityAcc = Vector3(0.0f, -9.8f, 0.0f);
	param.broadphase = BroadPhaseSolver::SAP;
	param.resolutionSolver = ResolutionPhaseSolver::LCPGlobal;
	param.solverBackend = SolverBackend::CPU;
	param.contactSolverIterations = 32;
	PhysicsWorld world(param);

	RigidBodyParam groundParam;
	groundParam.rigidType = RigidType::Static;
	Geometry* ground = GeometryFactory::CreatePlane(Vector3(0.0f, -5.0f, 0.0f), Vector3::UnitY());
	EXPECT(world.CreateRigidBody(ground, groundParam) != nullptr);

	std::vector<RigidBodyDynamic*> boxes;
	for (int i = 0; i < 5; ++i)
	{
		RigidBodyParam bodyParam;
		bodyParam.rigidType = RigidType::Dynamic;
		Geometry* box = GeometryFactory::CreateOBB(Vector3(0.0f, -4.0f + 2.0f * i, 0.0f), Vector3::One());
		RigidBody* body = world.CreateRigidBody(box, bodyParam);
		EXPECT(body != nullptr && body->CastDynamic() != nullptr);
		if (body && body->CastDynamic())
		{
			boxes.push_back(body->CastDynamic());
		}
	}

	for (int frame = 0; frame < 1200; ++frame)
	{
		world.Simulate();
	}

	EXPECT(boxes.size() == 5);
	for (size_t i = 0; i < boxes.size(); ++i)
	{
		const RigidBodyDynamic* body = boxes[i];
		const float expectedY = -4.0f + 2.0f * (float)i;
		const Vector3 up = body->Q * Vector3::UnitY();
		printf("  stack[%zu] pos=(%.3f, %.3f, %.3f) speed=%.4f upY=%.4f sleeping=%d\n",
			i, body->X.x, body->X.y, body->X.z, body->V.Length(), up.y, body->Sleeping ? 1 : 0);
		EXPECT(std::abs(body->X.x) < 0.25f);
		EXPECT(std::abs(body->X.z) < 0.25f);
		EXPECT(std::abs(body->X.y - expectedY) < 0.25f);
		EXPECT(up.y > 0.98f);
		EXPECT(body->V.Length() < 0.1f);
	}
}

void TestGlobalLCPContactSolver()
{
	printf("Running TestGlobalLCPContactSolver\n");

	const SolverBackend Backends[] = {
		SolverBackend::CPU,
		SolverBackend::SIMD,
		SolverBackend::ComputeShader,
#if defined(RIEMANN_ENABLE_CUDA)
		SolverBackend::CUDA,
#endif
	};
	const char* BackendNames[] = {
		"CPU",
		"SIMD",
		"Compute",
#if defined(RIEMANN_ENABLE_CUDA)
		"CUDA",
#endif
	};
	const int BackendCount = (int)(sizeof(Backends) / sizeof(Backends[0]));
	std::vector<float> FinalHeight((size_t)BackendCount, 0.0f);
	std::vector<float> FinalBoxX((size_t)BackendCount, 0.0f);
	std::vector<bool> Completed((size_t)BackendCount, false);
	for (int BackendIndex = 0; BackendIndex < BackendCount; ++BackendIndex)
	{
		const SolverBackend Backend = Backends[BackendIndex];
		const bool BackendAvailable = LCPSolver::IsBackendAvailable(Backend);
		if (Backend == SolverBackend::ComputeShader && !BackendAvailable)
		{
			printf("  LCP %s skipped: backend unavailable\n", BackendNames[BackendIndex]);
			continue;
		}

		int ContactCount = 0;
		float MaximumNormalImpulse = 0.0f;
		PhysicsWorldParam Param;
		Param.gravityAcc = Vector3(0.0f, -9.8f, 0.0f);
		Param.resolutionSolver = ResolutionPhaseSolver::LCPGlobal;
		Param.solverBackend = Backend;
		Param.contactSolverIterations = 48;
		Param.contactSolverTolerance = 1.0e-6f;
		Param.onContact = [&ContactCount, &MaximumNormalImpulse](const PhysicsContactInfo& ContactInfo)
		{
			++ContactCount;
			for (int ContactIndex = 0; ContactIndex < ContactInfo.NumContactPoints; ++ContactIndex)
			{
				const float NormalImpulse = ContactInfo.ContactPoints[ContactIndex].NormalImpulse;
				EXPECT(NormalImpulse >= 0.0f);
				MaximumNormalImpulse = std::max(MaximumNormalImpulse, NormalImpulse);
			}
		};
		PhysicsWorld World(Param);
		EXPECT(World.GetResolutionSolver() == ResolutionPhaseSolver::LCPGlobal);
		const SolverBackend ExpectedBackend = BackendAvailable ? Backend : SolverBackend::CPU;
		EXPECT(World.GetSolverBackend() == ExpectedBackend);

		RigidBodyParam GroundParam;
		GroundParam.rigidType = RigidType::Static;
		EXPECT(World.CreateRigidBody(
			GeometryFactory::CreatePlane(Vector3::Zero(), Vector3::UnitY()), GroundParam) != nullptr);

		RigidBodyParam SphereParam;
		SphereParam.rigidType = RigidType::Dynamic;
		SphereParam.linearVelocity = Vector3(0.0f, -0.5f, 0.0f);
		SphereParam.sleepThreshold = 0.0f;
		RigidBodyDynamic* Sphere = World.CreateRigidBody(
			GeometryFactory::CreateSphere(Vector3(0.0f, 2.0f, 0.0f), 0.5f), SphereParam)->CastDynamic();
		EXPECT(Sphere != nullptr);

		RigidBodyParam BoxParam;
		BoxParam.rigidType = RigidType::Dynamic;
		BoxParam.linearVelocity = Vector3(1.5f, -0.25f, 0.0f);
		BoxParam.sleepThreshold = 0.0f;
		RigidBodyDynamic* SlidingBox = World.CreateRigidBody(GeometryFactory::CreateOBB(
			Vector3(-3.0f, 0.55f, 0.0f), Vector3(0.5f, 0.5f, 0.5f)), BoxParam)->CastDynamic();
		EXPECT(SlidingBox != nullptr);
		for (int Frame = 0; Frame < 360; ++Frame)
		{
			World.Simulate();
		}
		EXPECT(World.GetSolverBackend() == ExpectedBackend);

		EXPECT(ContactCount > 0);
		EXPECT(MaximumNormalImpulse > 0.0f);
		EXPECT(std::isfinite(Sphere->X.y) && std::isfinite(Sphere->V.y));
		EXPECT(Sphere->X.y >= 0.45f && Sphere->X.y <= 0.65f);
		EXPECT(std::fabs(Sphere->V.y) < 0.2f);
		EXPECT(std::isfinite(SlidingBox->X.x) && std::isfinite(SlidingBox->X.y));
		EXPECT(SlidingBox->X.y >= 0.4f);
		EXPECT(std::fabs(SlidingBox->V.x) < 0.3f);
		EXPECT(SlidingBox->X.x < -1.5f);
		FinalHeight[(size_t)BackendIndex] = Sphere->X.y;
		FinalBoxX[(size_t)BackendIndex] = SlidingBox->X.x;
		Completed[(size_t)BackendIndex] = true;
	}
	EXPECT(Completed[0]);
	for (int BackendIndex = 1; BackendIndex < BackendCount; ++BackendIndex)
	{
		if (!Completed[(size_t)BackendIndex])
		{
			continue;
		}
		EXPECT_NEAR(FinalHeight[0], FinalHeight[(size_t)BackendIndex], 1.0e-3f);
		EXPECT_NEAR(FinalBoxX[0], FinalBoxX[(size_t)BackendIndex], 1.0e-3f);
	}
}

static void TestBoxBoxSATContactBasis()
{
	printf("Running TestBoxBoxSATContactBasis\n");

	const IntegrateMethod integrationMethods[] = {
		IntegrateMethod::ExplicitEuler,
		IntegrateMethod::SymplecticEuler,
	};
	float finalVerticalVelocity[2] = { 0.0f, 0.0f };
	for (int methodIndex = 0; methodIndex < 2; ++methodIndex)
	{
		int contactPointCount = 0;
		PhysicsWorldParam param;
		param.gravityAcc = Vector3(0.0f, -9.8f, 0.0f);
		param.broadphase = BroadPhaseSolver::SAP;
		param.resolutionSolver = ResolutionPhaseSolver::LCPGlobal;
		param.solverBackend = SolverBackend::CPU;
		param.integrateMethod = integrationMethods[methodIndex];
		param.onContact = [&](const PhysicsContactInfo& info)
		{
			for (int pointIndex = 0; pointIndex < info.NumContactPoints; ++pointIndex)
			{
				const PhysicsContactPoint& point = info.ContactPoints[pointIndex];
				++contactPointCount;
				EXPECT(std::abs(point.Normal.y) > 0.9999f);
				EXPECT_NEAR(point.Normal.Length(), 1.0f, 1.0e-5f);
				EXPECT_NEAR(point.Tangent.Length(), 1.0f, 1.0e-5f);
				EXPECT_NEAR(point.Binormal.Length(), 1.0f, 1.0e-5f);
				EXPECT_NEAR(point.Normal.Dot(point.Tangent), 0.0f, 1.0e-5f);
				EXPECT_NEAR(point.Normal.Dot(point.Binormal), 0.0f, 1.0e-5f);
				EXPECT_NEAR(point.Tangent.Dot(point.Binormal), 0.0f, 1.0e-5f);
			}
		};
		PhysicsWorld world(param);

		RigidBodyParam staticParam;
		staticParam.rigidType = RigidType::Static;
		Geometry* lowerBox = GeometryFactory::CreateOBB(Vector3::Zero(), Vector3::One());
		EXPECT(world.CreateRigidBody(lowerBox, staticParam) != nullptr);

		RigidBodyParam dynamicParam;
		dynamicParam.rigidType = RigidType::Dynamic;
		dynamicParam.sleepThreshold = 0.0f;
		Geometry* upperBox = GeometryFactory::CreateOBB(
			Vector3(0.0f, 1.99f, 0.0f), Vector3::One());
		RigidBody* upperBody = world.CreateRigidBody(upperBox, dynamicParam);
		RigidBodyDynamic* dynamicBody = upperBody ? upperBody->CastDynamic() : nullptr;
		EXPECT(dynamicBody != nullptr);

		world.Simulate();
		EXPECT(contactPointCount >= 1);
		EXPECT_NEAR(dynamicBody->V.x, 0.0f, 1.0e-5f);
		EXPECT_NEAR(dynamicBody->V.z, 0.0f, 1.0e-5f);
		EXPECT_NEAR(dynamicBody->W.Length(), 0.0f, 1.0e-5f);
		EXPECT(dynamicBody->V.y > -0.05f);
		finalVerticalVelocity[methodIndex] = dynamicBody->V.y;
	}
	EXPECT_NEAR(finalVerticalVelocity[0], finalVerticalVelocity[1], 1.0e-5f);
}

void TestFiveByFiveBoxStackGridStability()
{
	printf("Running TestFiveByFiveBoxStackGridStability\n");

	struct StackMetrics
	{
		double ElapsedMilliseconds = 0.0;
		float MaximumHorizontalError = 0.0f;
		float MaximumVerticalError = 0.0f;
		float MinimumUp = 1.0f;
		float MaximumSpeed = 0.0f;
		int SleepingCount = 0;
		int FirstContactFrame = -1;
		int FirstContactPointCount = 0;
		int FirstHorizontalContactPointCount = 0;
		float FirstMaximumNormalHorizontal = 0.0f;
		float FirstMaximumNormalLengthError = 0.0f;
	};
	struct StackedBody
	{
		RigidBodyDynamic* Body;
		Vector3 ExpectedPosition;
	};

	const int stackWidth = 5;
	const int stackHeight = 5;
	const int stackBodyCount = stackWidth * stackHeight * stackWidth;
	const float horizontalSpacing = 2.1f;
	const float verticalSpacing = 1.898f;
	const float bottomCenterY = -4.052f;
	const float horizontalStart = -0.5f * horizontalSpacing * (stackWidth - 1);
	auto RunStack = [&](SolverBackend backend, const char* backendName,
		const char* phaseName, int frameCount, bool allowSleep, StackMetrics* output)
	{
		*output = StackMetrics();
		StackMetrics& metrics = *output;
		int simulationFrame = 0;
		PhysicsWorldParam param;
		param.gravityAcc = Vector3(0.0f, -9.8f, 0.0f);
		param.broadphase = BroadPhaseSolver::SAP;
		param.resolutionSolver = ResolutionPhaseSolver::LCPGlobal;
		param.solverBackend = backend;
		param.contactSolverIterations = 32;
		param.onContact = [&](const PhysicsContactInfo& info)
		{
			if (metrics.FirstContactFrame < 0)
			{
				metrics.FirstContactFrame = simulationFrame;
			}
			if (simulationFrame != metrics.FirstContactFrame)
			{
				return;
			}
			for (int pointIndex = 0; pointIndex < info.NumContactPoints; ++pointIndex)
			{
				const Vector3& normal = info.ContactPoints[pointIndex].Normal;
				++metrics.FirstContactPointCount;
				metrics.FirstHorizontalContactPointCount += std::abs(normal.y) < 0.9f ? 1 : 0;
				metrics.FirstMaximumNormalHorizontal = std::max(
					metrics.FirstMaximumNormalHorizontal,
					std::sqrt(normal.x * normal.x + normal.z * normal.z));
				metrics.FirstMaximumNormalLengthError = std::max(
					metrics.FirstMaximumNormalLengthError,
					std::abs(normal.Length() - 1.0f));
			}
		};
		PhysicsWorld world(param);
		EXPECT(world.GetSolverBackend() == backend);

		RigidBodyParam groundParam;
		groundParam.rigidType = RigidType::Static;
		Geometry* ground = GeometryFactory::CreatePlane(
			Vector3(0.0f, -5.0f, 0.0f), Vector3::UnitY());
		EXPECT(world.CreateRigidBody(ground, groundParam) != nullptr);

		std::vector<StackedBody> boxes;
		boxes.reserve((size_t)stackBodyCount);
		for (int z = 0; z < stackWidth; ++z)
		{
			for (int y = 0; y < stackHeight; ++y)
			{
				for (int x = 0; x < stackWidth; ++x)
				{
					const Vector3 position(horizontalStart + horizontalSpacing * x,
						bottomCenterY + verticalSpacing * y,
						horizontalStart + horizontalSpacing * z);
					RigidBodyParam bodyParam;
					bodyParam.rigidType = RigidType::Dynamic;
					if (!allowSleep)
					{
						bodyParam.sleepThreshold = 0.0f;
					}
					Geometry* box = GeometryFactory::CreateOBB(
						position, Vector3(0.95f, 0.95f, 0.95f));
					RigidBody* body = world.CreateRigidBody(box, bodyParam);
					EXPECT(body != nullptr && body->CastDynamic() != nullptr);
					if (body && body->CastDynamic())
					{
						boxes.push_back({ body->CastDynamic(), position });
					}
				}
			}
		}

		// Exclude first-use allocation, CUDA context setup, and broadphase cache
		// population from the steady-state backend comparison.
		simulationFrame = 0;
		world.Simulate();
		const auto begin = std::chrono::steady_clock::now();
		for (int frame = 0; frame < frameCount; ++frame)
		{
			simulationFrame = frame + 1;
			world.Simulate();
		}
		EXPECT(world.GetSolverBackend() == backend);
		metrics.ElapsedMilliseconds =
			std::chrono::duration<double, std::milli>(
				std::chrono::steady_clock::now() - begin).count();

		EXPECT(boxes.size() == (size_t)stackBodyCount);
		for (const StackedBody& stacked : boxes)
		{
			const RigidBodyDynamic* body = stacked.Body;
			const Vector3 up = body->Q * Vector3::UnitY();
			metrics.MaximumHorizontalError = std::max(metrics.MaximumHorizontalError,
				std::max(std::abs(body->X.x - stacked.ExpectedPosition.x),
					std::abs(body->X.z - stacked.ExpectedPosition.z)));
			metrics.MaximumVerticalError = std::max(metrics.MaximumVerticalError,
				std::abs(body->X.y - stacked.ExpectedPosition.y));
			metrics.MinimumUp = std::min(metrics.MinimumUp, up.y);
			metrics.MaximumSpeed = std::max(metrics.MaximumSpeed, body->V.Length());
			metrics.SleepingCount += body->Sleeping ? 1 : 0;
		}
		printf("  LCP %-4s %-7s %d boxes: %.2f ms, %.3f ms/frame, maxXZ=%.4f maxY=%.4f minUp=%.4f maxSpeed=%.4f sleeping=%d\n",
			backendName, phaseName, stackBodyCount, metrics.ElapsedMilliseconds,
			metrics.ElapsedMilliseconds / frameCount, metrics.MaximumHorizontalError,
			metrics.MaximumVerticalError, metrics.MinimumUp, metrics.MaximumSpeed,
			metrics.SleepingCount);
		printf("    firstContactFrame=%d contacts=%d horizontal=%d normalMaxXZ=%.6f normalLengthError=%.6f\n",
			metrics.FirstContactFrame, metrics.FirstContactPointCount,
			metrics.FirstHorizontalContactPointCount,
			metrics.FirstMaximumNormalHorizontal,
			metrics.FirstMaximumNormalLengthError);

		EXPECT(metrics.FirstContactFrame == 0);
		EXPECT(metrics.FirstContactPointCount > 0);
		EXPECT(metrics.FirstHorizontalContactPointCount == 0);
		EXPECT(metrics.FirstMaximumNormalHorizontal < 1.0e-4f);
		EXPECT(metrics.FirstMaximumNormalLengthError < 1.0e-4f);
		EXPECT(metrics.MaximumHorizontalError < (allowSleep ? 0.3f : 0.5f));
		EXPECT(metrics.MaximumVerticalError < 0.3f);
		EXPECT(metrics.MinimumUp > 0.98f);
		EXPECT(metrics.MaximumSpeed < (allowSleep ? 0.1f : 0.5f));
		EXPECT(metrics.SleepingCount == (allowSleep ? stackBodyCount : 0));
	};

	StackMetrics settledMetrics;
	RunStack(SolverBackend::CPU, "CPU", "settled", 600, true, &settledMetrics);

	const SolverBackend Backends[] = {
		SolverBackend::CPU,
		SolverBackend::SIMD,
		SolverBackend::ComputeShader,
#if defined(RIEMANN_ENABLE_CUDA)
		SolverBackend::CUDA,
#endif
	};
	const char* BackendNames[] = {
		"CPU",
		"SIMD",
		"Compute",
#if defined(RIEMANN_ENABLE_CUDA)
		"CUDA",
#endif
	};
	const int BackendCount = (int)(sizeof(Backends) / sizeof(Backends[0]));
	std::vector<StackMetrics> backendMetrics((size_t)BackendCount);
	std::vector<bool> validMetrics((size_t)BackendCount, false);
	for (int backendIndex = 0; backendIndex < BackendCount; ++backendIndex)
	{
		if (!LCPSolver::IsBackendAvailable(Backends[backendIndex]))
		{
			printf("  LCP %s skipped: backend unavailable\n", BackendNames[backendIndex]);
			continue;
		}

		RunStack(Backends[backendIndex], BackendNames[backendIndex],
			"active", 120, false, &backendMetrics[(size_t)backendIndex]);
		validMetrics[(size_t)backendIndex] = true;
	}

	EXPECT(validMetrics[0]);
	for (int backendIndex = 1; backendIndex < BackendCount; ++backendIndex)
	{
		if (!validMetrics[(size_t)backendIndex])
		{
			continue;
		}
		EXPECT_NEAR(backendMetrics[0].MaximumHorizontalError,
			backendMetrics[(size_t)backendIndex].MaximumHorizontalError, 0.05f);
		EXPECT_NEAR(backendMetrics[0].MaximumVerticalError,
			backendMetrics[(size_t)backendIndex].MaximumVerticalError, 0.05f);
		EXPECT_NEAR(backendMetrics[0].MinimumUp,
			backendMetrics[(size_t)backendIndex].MinimumUp, 0.01f);
	}
}

static void TestContactRestitution()
{
	printf("Running TestContactRestitution\n");

	PhysicsWorldParam worldParam;
	worldParam.gravityAcc = Vector3(0.0f, -9.8f, 0.0f);
	PhysicsWorld world(worldParam);
	RigidBodyParam groundParam;
	groundParam.rigidType = RigidType::Static;
	RigidBody* groundBody = world.CreateRigidBody(
		GeometryFactory::CreatePlane(Vector3::Zero(), Vector3::UnitY()), groundParam);
	EXPECT(groundBody != nullptr);

	RigidBodyParam bodyParam;
	bodyParam.rigidType = RigidType::Dynamic;
	bodyParam.linearVelocity = Vector3(0.0f, -0.5f, 0.0f);
	RigidBodyDynamic* body = world.CreateRigidBody(
		GeometryFactory::CreateSphere(Vector3(0.0f, 3.0f, 0.0f), 0.5f), bodyParam)->CastDynamic();
	EXPECT(body != nullptr);
	if (body == nullptr)
	{
		return;
	}
	body->SetRestitution(0.55f);
	EXPECT(std::abs(body->GetRestitution() - 0.55f) < 1.0e-5f);
	EXPECT(groundBody->GetRestitution() == 0.0f);
	EXPECT(body->mMotionType == MorionType::Discrete);

	float maximumReboundSpeed = 0.0f;
	bool approachedGround = false;
	for (int frame = 0; frame < 240; ++frame)
	{
		world.Simulate();
		approachedGround = approachedGround || body->X.y < 0.75f;
		if (approachedGround)
		{
			maximumReboundSpeed = std::max(maximumReboundSpeed, body->V.y);
		}
	}
	EXPECT(approachedGround);
	EXPECT(maximumReboundSpeed > 1.0f);
	EXPECT(body->X.y >= 0.45f);
}

void TestCompoundNonConvexRigidBody()
{
	printf("Running TestCompoundNonConvexRigidBody\n");
	TestContactRestitution();

	int externalContactCount = 0;
	int selfContactCount = 0;
	PhysicsWorldParam param;
	param.gravityAcc = Vector3(0.0f, -9.8f, 0.0f);
	param.broadphase = BroadPhaseSolver::SAP;
	param.onContact = [&externalContactCount, &selfContactCount](const PhysicsContactInfo& contact)
	{
		if (contact.BodyA == contact.BodyB)
		{
			++selfContactCount;
		}
		else
		{
			++externalContactCount;
		}
	};
	PhysicsWorld world(param);

	RigidBodyParam groundParam;
	groundParam.rigidType = RigidType::Static;
	Geometry* ground = GeometryFactory::CreateOBB(Vector3(0.0f, -0.5f, 0.0f), Vector3(8.0f, 0.5f, 8.0f));
	EXPECT(world.CreateRigidBody(ground, groundParam) != nullptr);

	std::vector<Geometry*> shapes;
	shapes.push_back(GeometryFactory::CreateSphere(Vector3(-2.0f, 4.0f, 0.0f), 1.0f));
	shapes.push_back(GeometryFactory::CreateSphere(Vector3(2.0f, 4.0f, 0.0f), 1.0f));
	shapes.push_back(GeometryFactory::CreateOBB(Vector3(0.0f, 4.0f, 0.0f), Vector3(2.0f, 0.25f, 0.25f)));

	RigidBodyParam bodyParam;
	bodyParam.rigidType = RigidType::Dynamic;
	RigidBodyDynamic* body = world.CreateRigidBody(shapes, bodyParam, Transform(Vector3(0.0f, 4.0f, 0.0f)))->CastDynamic();
	EXPECT(body != nullptr);
	if (body == nullptr)
	{
		return;
	}

	EXPECT(body->GetNumGeometries() == 3);
	EXPECT(body->FindGeometryHandle(shapes[0]).IsValid());
	EXPECT(body->FindGeometryHandle(shapes[1]).IsValid());
	EXPECT(body->FindGeometryHandle(shapes[2]).IsValid());
	EXPECT(body->CM.SquareLength() < 1e-5f);
	EXPECT(body->InvMass > 0.0f && body->InvMass < 0.2f);
	EXPECT(body->InvInertia(1, 1) < body->InvInertia(0, 0));
	EXPECT_SAME(shapes[0]->GetPosition(), Vector3(-2.0f, 0.0f, 0.0f));
	EXPECT_SAME(shapes[1]->GetPosition(), Vector3(2.0f, 0.0f, 0.0f));

	Quaternion quarterTurn;
	quarterTurn.FromRotationAxis(Vector3::UnitY(), PI_OVER_2);
	body->Q = quarterTurn;
	EXPECT_SAME(body->GetGeometryTransform(shapes[0]).pos, Vector3(0.0f, 4.0f, 2.0f));
	EXPECT_SAME(body->GetGeometryTransform(shapes[1]).pos, Vector3(0.0f, 4.0f, -2.0f));

	for (int frame = 0; frame < 360; ++frame)
	{
		world.Simulate();
	}

	EXPECT(selfContactCount == 0);
	EXPECT(externalContactCount > 0);
	EXPECT(body->X.y > 0.2f);
	EXPECT(body->V.y > -1.0f);
}

void TestTriggerSystem()
{
	printf("Running TestTriggerSystem\n");

	{
		Geometry* geometry = GeometryFactory::CreateSphere(Vector3::Zero(), 1.0f);
		EXPECT(!geometry->IsTriggerEnabled());
		EXPECT(!geometry->IsTriggerCCDEnabled());
		GeometryFactory::DeleteGeometry(geometry);
	}

	{
		int triggerCount = 0;
		int contactCount = 0;
		PhysicsTriggerInfo lastTrigger = {};
		PhysicsWorldParam param;
		param.onTrigger = [&](const PhysicsTriggerInfo& info)
		{
			++triggerCount;
			lastTrigger = info;
		};
		param.onContact = [&](const PhysicsContactInfo&)
		{
			++contactCount;
		};
		PhysicsWorld world(param);

		Geometry* trigger = GeometryFactory::CreateOBB(Vector3::Zero(), Vector3(2.0f));
		trigger->SetTriggerEnabled(true);
		RigidBodyParam staticParam;
		staticParam.rigidType = RigidType::Static;
		RigidBody* triggerBody = world.CreateRigidBody(trigger, staticParam);

		RigidBodyParam dynamicParam;
		dynamicParam.rigidType = RigidType::Dynamic;
		dynamicParam.linearVelocity = Vector3(1.0f, 0.0f, 0.0f);
		Geometry* sphere = GeometryFactory::CreateSphere(Vector3::Zero(), 0.5f);
		RigidBodyDynamic* movingBody = world.CreateRigidBody(sphere, dynamicParam)->CastDynamic();
		world.Simulate();

		EXPECT(triggerCount == 1);
		EXPECT(contactCount == 0);
		EXPECT(!lastTrigger.IsCCD);
		EXPECT((lastTrigger.GeomA == trigger && lastTrigger.GeomB == sphere)
			|| (lastTrigger.GeomA == sphere && lastTrigger.GeomB == trigger));
		EXPECT((lastTrigger.BodyA == triggerBody && lastTrigger.BodyB == movingBody)
			|| (lastTrigger.BodyA == movingBody && lastTrigger.BodyB == triggerBody));
		EXPECT(std::abs(movingBody->V.x - 1.0f) < 1.0e-4f);
	}

	{
		int ccdTriggerCount = 0;
		std::vector<Geometry*> hitTriggers;
		PhysicsWorldParam param;
		param.onTrigger = [&](const PhysicsTriggerInfo& info)
		{
			if (info.IsCCD)
			{
				++ccdTriggerCount;
				EXPECT(info.Fraction > 0.0f && info.Fraction < 1.0f);
				hitTriggers.push_back(info.GeomA->IsTriggerEnabled() ? info.GeomA : info.GeomB);
			}
		};
		PhysicsWorld world(param);
		RigidBodyParam staticParam;
		staticParam.rigidType = RigidType::Static;
		std::vector<Geometry*> enabledTriggers;
		for (float x : { 0.0f, 2.0f })
		{
			Geometry* trigger = GeometryFactory::CreateOBB(Vector3(x, 0.0f, 0.0f), Vector3(0.05f, 2.0f, 2.0f));
			trigger->SetTriggerEnabled(true);
			trigger->SetTriggerCCDEnabled(true);
			enabledTriggers.push_back(trigger);
			world.CreateRigidBody(trigger, staticParam);
		}
		Geometry* disabledCCDTrigger = GeometryFactory::CreateOBB(
			Vector3(4.0f, 0.0f, 0.0f), Vector3(0.05f, 2.0f, 2.0f));
		disabledCCDTrigger->SetTriggerEnabled(true);
		world.CreateRigidBody(disabledCCDTrigger, staticParam);

		RigidBodyParam movingParam;
		movingParam.rigidType = RigidType::Dynamic;
		movingParam.motionType = MorionType::Discrete;
		movingParam.linearVelocity = Vector3(600.0f, 0.0f, 0.0f);
		RigidBodyDynamic* sphere = world.CreateRigidBody(GeometryFactory::CreateSphere(
			Vector3(-5.0f, 0.0f, 0.0f), 0.5f), movingParam)->CastDynamic();
		world.Simulate();

		EXPECT(ccdTriggerCount == 2);
		EXPECT(hitTriggers.size() == 2);
		for (Geometry* trigger : enabledTriggers)
		{
			EXPECT(std::find(hitTriggers.begin(), hitTriggers.end(), trigger) != hitTriggers.end());
		}
		EXPECT(std::find(hitTriggers.begin(), hitTriggers.end(), disabledCCDTrigger) == hitTriggers.end());
		EXPECT(world.GetLastCCDHitCount() == 0);
		EXPECT(sphere->X.x > 4.5f);
		EXPECT(sphere->V.x > 500.0f);
	}
}

void TestContinuousCollisionDetection()
{
	printf("Running TestContinuousCollisionDetection\n");
	TestTriggerSystem();

	{
		PhysicsWorldParam param;
		PhysicsWorld world(param);
		RigidBodyParam wallParam;
		wallParam.rigidType = RigidType::Static;
		EXPECT(world.CreateRigidBody(GeometryFactory::CreateOBB(Vector3::Zero(), Vector3(0.05f, 5.0f, 5.0f)), wallParam) != nullptr);

		RigidBodyParam sphereParam;
		sphereParam.rigidType = RigidType::Dynamic;
		sphereParam.linearVelocity = Vector3(600.0f, 0.0f, 0.0f);
		RigidBodyDynamic* discrete = world.CreateRigidBody(GeometryFactory::CreateSphere(Vector3(-5.0f, 3.0f, 0.0f), 0.5f), sphereParam)->CastDynamic();
		world.Simulate();
		EXPECT(discrete->X.x > 1.0f);
		EXPECT(world.GetLastCCDHitCount() == 0);
	}

	{
		PhysicsWorldParam param;
		PhysicsWorld world(param);
		RigidBodyParam wallParam;
		wallParam.rigidType = RigidType::Static;
		EXPECT(world.CreateRigidBody(GeometryFactory::CreateOBB(Vector3::Zero(), Vector3(0.05f, 5.0f, 5.0f)), wallParam) != nullptr);

		RigidBodyParam bodyParam;
		bodyParam.rigidType = RigidType::Dynamic;
		bodyParam.motionType = MorionType::Continuous;
		bodyParam.linearVelocity = Vector3(600.0f, 0.0f, 0.0f);
		RigidBodyDynamic* sphere = world.CreateRigidBody(GeometryFactory::CreateSphere(Vector3(-5.0f, 0.0f, 0.0f), 0.5f), bodyParam)->CastDynamic();
		world.Simulate();
		EXPECT(world.GetLastCCDHitCount() == 1);
		EXPECT(sphere->HadCCDContact);
		EXPECT(sphere->X.x < -0.5f && sphere->X.x > -0.7f);
		EXPECT(std::abs(sphere->V.x) < 0.01f);
	}

	{
		PhysicsWorldParam param;
		PhysicsWorld world(param);
		RigidBodyParam wallParam;
		wallParam.rigidType = RigidType::Static;
		EXPECT(world.CreateRigidBody(GeometryFactory::CreatePlane(Vector3::Zero(), -Vector3::UnitX()), wallParam) != nullptr);

		RigidBodyParam bodyParam;
		bodyParam.rigidType = RigidType::Dynamic;
		bodyParam.motionType = MorionType::Continuous;
		bodyParam.linearVelocity = Vector3(480.0f, 0.0f, 0.0f);
		Geometry* capsuleShape = GeometryFactory::CreateCapsule(Vector3(-4.0f, -1.0f, 0.0f), Vector3(-4.0f, 1.0f, 0.0f), 0.4f);
		RigidBodyDynamic* capsule = world.CreateRigidBody(capsuleShape, bodyParam)->CastDynamic();
		world.Simulate();
		EXPECT(world.GetLastCCDHitCount() == 1);
		EXPECT(capsule->X.x <= -0.4f);
		EXPECT(std::abs(capsule->V.x) < 0.01f);
	}

	{
		PhysicsWorldParam param;
		PhysicsWorld world(param);
		RigidBodyParam wallParam;
		wallParam.rigidType = RigidType::Static;
		EXPECT(world.CreateRigidBody(GeometryFactory::CreatePlane(Vector3::Zero(), -Vector3::UnitX()), wallParam) != nullptr);

		RigidBodyParam bodyParam;
		bodyParam.rigidType = RigidType::Dynamic;
		bodyParam.motionType = MorionType::Continuous;
		bodyParam.angularVelocity = Vector3(0.0f, 0.0f, 120.0f);
		Geometry* capsuleGeometry = GeometryFactory::CreateCapsule(
			Vector3(-0.5f, -1.0f, 0.0f), Vector3(-0.5f, 1.0f, 0.0f), 0.1f);
		RigidBodyDynamic* capsuleBody = world.CreateRigidBody(capsuleGeometry, bodyParam)->CastDynamic();
		world.Simulate();
		EXPECT(world.GetLastCCDHitCount() == 1);
		EXPECT(capsuleBody->HadCCDContact);
		EXPECT(capsuleBody->W.Length() > 0.1f && capsuleBody->W.Length() < 120.0f);
		EXPECT(capsuleBody->V.x < -0.1f);
		const Capsule3* capsule = capsuleGeometry->GetShapeObj<Capsule3>();
		const Transform capsuleTransform = capsuleBody->GetGeometryTransform(capsuleGeometry);
		const Vector3 contactPoint = capsuleTransform.LocalToWorld(capsule->GetSupport(
			capsuleTransform.WorldToLocalDirection(Vector3::UnitX())));
		const Vector3 contactVelocity = capsuleBody->V + capsuleBody->W.Cross(contactPoint - capsuleBody->X);
		EXPECT(contactVelocity.Dot(-Vector3::UnitX()) > -1.0f);
		const float maximumX = std::max(capsuleTransform.LocalToWorld(capsule->X0).x,
			capsuleTransform.LocalToWorld(capsule->X1).x) + capsule->Radius;
		EXPECT(maximumX < 0.02f);
	}

	{
		PhysicsWorldParam param;
		PhysicsWorld world(param);
		Geometry* wallGeometry = GeometryFactory::CreateTriangleMesh();
		TriangleMesh* wallMesh = wallGeometry->GetShapeObj<TriangleMesh>();
		wallMesh->AddAABB(Vector3(0.0f, -3.0f, -3.0f), Vector3(0.02f, 3.0f, 3.0f));
		wallMesh->Compact();
		wallMesh->BuildBVH();
		wallGeometry->SetShapeBounds(wallMesh->BoundingVolume);
		RigidBodyParam wallParam;
		wallParam.rigidType = RigidType::Static;
		world.CreateRigidBody(wallGeometry, wallParam);

		RigidBodyParam bodyParam;
		bodyParam.rigidType = RigidType::Dynamic;
		bodyParam.motionType = MorionType::Continuous;
		bodyParam.linearVelocity = Vector3(20.0f, 0.0f, 0.0f);
		bodyParam.angularVelocity = Vector3(0.0f, 0.0f, 90.0f);
		Geometry* capsuleGeometry = GeometryFactory::CreateCapsule(
			Vector3(-0.8f, -1.0f, 0.0f), Vector3(-0.8f, 1.0f, 0.0f), 0.1f);
		RigidBodyDynamic* capsuleBody = world.CreateRigidBody(capsuleGeometry, bodyParam)->CastDynamic();
		world.Simulate();
		EXPECT(capsuleBody->HadCCDContact);
		EXPECT(world.GetLastCCDHitCount() > 0);
		const Capsule3* capsule = capsuleGeometry->GetShapeObj<Capsule3>();
		const Transform capsuleTransform = capsuleBody->GetGeometryTransform(capsuleGeometry);
		const float maximumX = std::max(capsuleTransform.LocalToWorld(capsule->X0).x,
			capsuleTransform.LocalToWorld(capsule->X1).x) + capsule->Radius;
		EXPECT(maximumX < 0.04f);
		EXPECT(capsuleBody->V.x < 20.0f);
	}

	{
		PhysicsWorldParam param;
		PhysicsWorld world(param);
		RigidBodyParam wallParam;
		wallParam.rigidType = RigidType::Static;
		world.CreateRigidBody(GeometryFactory::CreatePlane(Vector3::Zero(), -Vector3::UnitX()), wallParam);
		RigidBodyParam movingParam;
		movingParam.rigidType = RigidType::Dynamic;
		movingParam.motionType = MorionType::Continuous;
		movingParam.angularVelocity = Vector3(0.0f, 0.0f, 240.0f);
		RigidBodyParam linkedParam = movingParam;
		linkedParam.angularVelocity = Vector3::Zero();
		RigidBodyDynamic* movingBody = world.CreateRigidBody(GeometryFactory::CreateCapsule(
			Vector3(-0.5f, -1.0f, 0.0f), Vector3(-0.5f, 1.0f, 0.0f), 0.1f), movingParam)->CastDynamic();
		RigidBodyDynamic* linkedBody = world.CreateRigidBody(GeometryFactory::CreateCapsule(
			Vector3(-0.5f, -1.0f, 0.0f), Vector3(-0.5f, 1.0f, 0.0f), 0.1f), linkedParam)->CastDynamic();
		movingBody->IgnoreCollisionWith(linkedBody);
		EXPECT(world.CreateFixedConstraint(movingBody, linkedBody) != nullptr);
		world.Simulate();
		EXPECT(linkedBody->HadCCDContact);
		EXPECT(linkedBody->CCDMotionFraction < 1.0f);
	}

	{
		PhysicsWorldParam param;
		PhysicsWorld world(param);
		RigidBodyParam targetParam;
		targetParam.rigidType = RigidType::Dynamic;
		targetParam.sleepThreshold = 0.0f;
		RigidBodyDynamic* target = world.CreateRigidBody(GeometryFactory::CreateOBB(
			Vector3(0.05f, 0.0f, 0.0f), Vector3(0.05f, 3.0f, 3.0f)), targetParam)->CastDynamic();
		RigidBodyParam movingParam;
		movingParam.rigidType = RigidType::Dynamic;
		movingParam.motionType = MorionType::Continuous;
		movingParam.angularVelocity = Vector3(0.0f, 0.0f, 120.0f);
		RigidBodyDynamic* capsule = world.CreateRigidBody(GeometryFactory::CreateCapsule(
			Vector3(-0.5f, -1.0f, 0.0f), Vector3(-0.5f, 1.0f, 0.0f), 0.1f), movingParam)->CastDynamic();
		world.Simulate();
		EXPECT(capsule->HadCCDContact);
		EXPECT(target->V.x > 1.0e-4f);
		EXPECT(capsule->V.x < -1.0e-4f);
	}

	{
		PhysicsWorldParam param;
		PhysicsWorld world(param);
		RigidBodyParam wallParam;
		wallParam.rigidType = RigidType::Static;
		EXPECT(world.CreateRigidBody(GeometryFactory::CreateOBB(Vector3::Zero(), Vector3(0.05f, 6.0f, 6.0f)), wallParam) != nullptr);

		std::vector<Geometry*> shapes;
		shapes.push_back(GeometryFactory::CreateSphere(Vector3(-5.0f, -1.5f, 0.0f), 0.4f));
		shapes.push_back(GeometryFactory::CreateSphere(Vector3(-5.0f, 1.5f, 0.0f), 0.4f));
		RigidBodyParam bodyParam;
		bodyParam.rigidType = RigidType::Dynamic;
		bodyParam.motionType = MorionType::Continuous;
		bodyParam.linearVelocity = Vector3(600.0f, 0.0f, 0.0f);
		RigidBodyDynamic* compound = world.CreateRigidBody(shapes, bodyParam, Transform(Vector3(-5.0f, 0.0f, 0.0f)))->CastDynamic();
		world.Simulate();
		EXPECT(world.GetLastCCDHitCount() == 1);
		EXPECT(compound->X.x < -0.4f);
		EXPECT(compound->GetNumGeometries() == 2);
	}

	{
		PhysicsWorldParam param;
		PhysicsWorld world(param);
		Geometry* meshGeometry = GeometryFactory::CreateTriangleMesh();
		TriangleMesh* triangleMesh = meshGeometry->GetShapeObj<TriangleMesh>();
		triangleMesh->AddAABB(Vector3(-5.0f, -0.2f, -5.0f), Vector3(5.0f, 0.0f, 5.0f));
		triangleMesh->Compact();
		triangleMesh->BuildBVH();
		meshGeometry->SetShapeBounds(triangleMesh->BoundingVolume);
		RigidBodyParam meshParam;
		meshParam.rigidType = RigidType::Static;
		EXPECT(world.CreateRigidBody(meshGeometry, meshParam) != nullptr);

		RigidBodyParam bodyParam;
		bodyParam.rigidType = RigidType::Dynamic;
		bodyParam.motionType = MorionType::Continuous;
		bodyParam.linearVelocity = Vector3(0.0f, -600.0f, 0.0f);
		RigidBodyDynamic* sphere = world.CreateRigidBody(
			GeometryFactory::CreateSphere(Vector3(0.0f, 5.0f, 0.0f), 0.5f), bodyParam)->CastDynamic();
		world.Simulate();
		EXPECT(world.GetLastCCDHitCount() == 1);
		EXPECT(sphere->X.y > 0.45f);
		EXPECT(sphere->V.y > -0.1f);
	}

	{
		PhysicsWorldParam param;
		PhysicsWorld world(param);
		RigidBodyParam bodyParam;
		bodyParam.rigidType = RigidType::Dynamic;
		bodyParam.motionType = MorionType::Continuous;
		bodyParam.linearVelocity = Vector3(360.0f, 0.0f, 0.0f);
		RigidBodyDynamic* bodyA = world.CreateRigidBody(GeometryFactory::CreateSphere(Vector3(-5.0f, 0.0f, 0.0f), 0.5f), bodyParam)->CastDynamic();
		bodyParam.linearVelocity = Vector3(-360.0f, 0.0f, 0.0f);
		RigidBodyDynamic* bodyB = world.CreateRigidBody(GeometryFactory::CreateSphere(Vector3(5.0f, 0.0f, 0.0f), 0.5f), bodyParam)->CastDynamic();
		world.Simulate();
		EXPECT(world.GetLastCCDHitCount() >= 1);
		EXPECT(bodyA->X.x < bodyB->X.x);
		EXPECT(bodyA->V.x <= 0.1f);
		EXPECT(bodyB->V.x >= -0.1f);
	}

	{
		PhysicsWorldParam param;
		param.gravityAcc = Vector3(0.0f, -9.8f, 0.0f);
		PhysicsWorld world(param);
		RigidBodyParam staticParam;
		staticParam.rigidType = RigidType::Static;
		world.CreateRigidBody(GeometryFactory::CreatePlane(Vector3::Zero(), Vector3::UnitY()), staticParam);
		Quaternion fulcrumRotation;
		fulcrumRotation.FromRotationAxis(Vector3::UnitZ(), 40.0f * PI_OVER_180);
		world.CreateRigidBody(GeometryFactory::CreateOBB(
			Vector3(3.0f, 0.0f, 0.0f), Vector3::One(), fulcrumRotation), staticParam);

		RigidBodyParam dynamicParam;
		dynamicParam.rigidType = RigidType::Dynamic;
		world.CreateRigidBody(GeometryFactory::CreateOBB(
			Vector3(2.0f, 1.7f, 0.0f), Vector3(5.0f, 0.25f, 1.0f)), dynamicParam);
		world.CreateRigidBody(GeometryFactory::CreateSphere(
			Vector3(-1.0f, 4.0f, 0.0f), 0.5f), dynamicParam);
		RigidBodyParam weightParam = dynamicParam;
		weightParam.invMass = 0.5f;
		world.CreateRigidBody(GeometryFactory::CreateOBB(
			Vector3(-2.0f, 4.0f, 0.0f), Vector3(0.5f)), weightParam);
		RigidBodyParam rollerParam = dynamicParam;
		rollerParam.invMass = 0.01f;
		RigidBodyDynamic* roller = world.CreateRigidBody(GeometryFactory::CreateCylinder(
			Vector3(6.0f, 10.0f, -2.0f), Vector3(6.0f, 10.0f, 2.0f), 0.5f), rollerParam)->CastDynamic();
		float minimumY = roller->X.y;
		for (int frame = 0; frame < 360; ++frame)
		{
			world.Simulate();
			minimumY = std::min(minimumY, roller->X.y);
		}
		EXPECT(roller->mMotionType == MorionType::Discrete);
		EXPECT(minimumY >= 0.45f);
		EXPECT(roller->X.y >= 0.45f);
		EXPECT(std::abs(roller->X.z) < 0.5f);
		EXPECT(roller->X.x < 8.0f);
	}
}

void TestPositionConstraints()
{
	printf("Running TestPositionConstraints\n");

	auto createBody = [](PhysicsWorld& world, const Vector3& position, RigidType type) -> RigidBody*
	{
		Geometry* shape = GeometryFactory::CreateSphere(position, 0.2f);
		shape->SetSimulationEnabled(false);
		RigidBodyParam bodyParam;
		bodyParam.rigidType = type;
		bodyParam.disableGravity = true;
		return world.CreateRigidBody(shape, bodyParam);
	};

	{
		PhysicsWorldParam param;
		PhysicsWorld world(param);
		RigidBody* anchor = createBody(world, Vector3::Zero(), RigidType::Static);
		RigidBodyDynamic* body = createBody(world, Vector3(2.0f, 0.0f, 0.0f), RigidType::Dynamic)->CastDynamic();
		FixedConstraint* constraint = world.CreateFixedConstraint(anchor, body);
		EXPECT(constraint != nullptr && constraint->GetType() == PositionConstraintType::Fixed);
		body->X = Vector3(5.0f, 3.0f, -1.0f);
		body->Q.FromRotationAxis(Vector3::UnitZ(), 1.0f);
		world.Simulate();
		EXPECT_SAME(body->X, Vector3(2.0f, 0.0f, 0.0f));
		EXPECT((body->Q * Vector3::UnitX()).Dot(Vector3::UnitX()) > 0.999f);
		constraint->SetEnabled(false);
		body->X = Vector3(5.0f, 0.0f, 0.0f);
		world.Simulate();
		EXPECT_NEAR(body->X.x, 5.0f, 1e-4f);
		constraint->SetEnabled(true);
		world.Simulate();
		EXPECT_NEAR(body->X.x, 2.0f, 1e-4f);
	}

	{
		PhysicsWorldParam param;
		PhysicsWorld world(param);
		RigidBody* anchor = createBody(world, Vector3::Zero(), RigidType::Static);
		RigidBodyDynamic* body = createBody(world, Vector3(2.0f, 0.0f, 0.0f), RigidType::Dynamic)->CastDynamic();
		PointConstraint* constraint = world.CreatePointConstraint(anchor, body, Vector3(1.0f, 0.0f, 0.0f));
		body->X += Vector3(0.0f, 3.0f, 1.0f);
		body->V = Vector3(0.0f, 8.0f, 2.0f);
		world.Simulate();
		EXPECT((constraint->GetWorldAnchorB() - constraint->GetWorldAnchorA()).Length() < 1e-4f);
		const Vector3 anchorVelocity = body->V + body->W.Cross(constraint->GetWorldAnchorB() - body->X);
		EXPECT(anchorVelocity.Length() < 1e-4f);
		EXPECT(body->V.Length() > 0.1f);
	}

	{
		PhysicsWorldParam param;
		PhysicsWorld world(param);
		RigidBody* anchor = createBody(world, Vector3::Zero(), RigidType::Static);
		RigidBodyDynamic* body = createBody(world, Vector3(2.5f, 0.0f, 0.0f), RigidType::Dynamic)->CastDynamic();
		DistanceConstraint* constraint = world.CreateDistanceConstraint(anchor, body, anchor->X, body->X, 2.0f, 3.0f);
		body->X = Vector3(8.0f, 0.0f, 0.0f);
		world.Simulate();
		EXPECT_NEAR(constraint->GetCurrentDistance(), 3.0f, 1e-4f);
		body->X = Vector3(0.5f, 0.0f, 0.0f);
		world.Simulate();
		EXPECT_NEAR(constraint->GetCurrentDistance(), 2.0f, 1e-4f);
		constraint->SetDistanceLimits(1.0f, 1.5f);
		body->X = Vector3(8.0f, 0.0f, 0.0f);
		world.Simulate();
		EXPECT_NEAR(constraint->GetCurrentDistance(), 1.5f, 1e-4f);
		EXPECT_NEAR(constraint->GetMinDistance(), 1.0f, 1e-6f);
	}

	{
		PhysicsWorldParam param;
		PhysicsWorld world(param);
		RigidBody* anchor = createBody(world, Vector3::Zero(), RigidType::Static);
		RigidBodyDynamic* body = createBody(world, Vector3::Zero(), RigidType::Dynamic)->CastDynamic();
		HingeConstraint* constraint = world.CreateHingeConstraint(anchor, body, Vector3::Zero(), Vector3::UnitY(), -0.4f, 0.4f);
		body->X = Vector3(1.0f, 2.0f, 3.0f);
		body->Q.FromRotationAxis(Vector3::UnitY(), 1.2f);
		world.Simulate();
		EXPECT(body->X.Length() < 1e-4f);
		EXPECT(std::abs(constraint->GetCurrentAngle()) <= 0.401f);
		EXPECT((body->Q * Vector3::UnitY()).Dot(Vector3::UnitY()) > 0.999f);
		constraint->SetAngleLimits(-0.1f, 0.1f);
		body->Q.FromRotationAxis(Vector3::UnitY(), 0.8f);
		world.Simulate();
		EXPECT(std::abs(constraint->GetCurrentAngle()) <= 0.101f);
	}

	{
		PhysicsWorldParam param;
		PhysicsWorld world(param);
		RigidBodyDynamic* bodyA = createBody(world, Vector3::Zero(), RigidType::Dynamic)->CastDynamic();
		RigidBodyDynamic* bodyB = createBody(world, Vector3::Zero(), RigidType::Dynamic)->CastDynamic();
		HingeConstraint* constraint = world.CreateHingeConstraint(
			bodyA, bodyB, Vector3::Zero(), Vector3::UnitY(), -0.5f, 0.5f);
		bodyA->SleepThreshold = 0.0f;
		bodyB->SleepThreshold = 0.0f;
		bodyA->W = Vector3::UnitX();
		bodyB->W = Vector3::UnitX();
		world.Simulate();
		EXPECT(constraint != nullptr);
		EXPECT(bodyA->W.x > 0.9f);
		EXPECT(bodyB->W.x > 0.9f);
		EXPECT((bodyB->W - bodyA->W).Length() < 1e-3f);
	}

	{
		PhysicsWorldParam param;
		PhysicsWorld world(param);
		RigidBody* anchor = createBody(world, Vector3::Zero(), RigidType::Static);
		RigidBodyDynamic* body = createBody(world, Vector3::Zero(), RigidType::Dynamic)->CastDynamic();
		SliderConstraint* constraint = world.CreateSliderConstraint(anchor, body, Vector3::Zero(), Vector3::UnitX(), -2.0f, 2.0f);
		body->X = Vector3(5.0f, 3.0f, -2.0f);
		body->Q.FromRotationAxis(Vector3::UnitZ(), 0.8f);
		world.Simulate();
		EXPECT_NEAR(constraint->GetCurrentPosition(), 2.0f, 1e-4f);
		EXPECT(std::abs(body->X.y) < 1e-4f && std::abs(body->X.z) < 1e-4f);
		EXPECT((body->Q * Vector3::UnitX()).Dot(Vector3::UnitX()) > 0.999f);
		constraint->SetDistanceLimits(-0.5f, 0.5f);
		body->X = Vector3(-4.0f, 2.0f, 1.0f);
		body->V = Vector3(-10.0f, 0.0f, 0.0f);
		world.Simulate();
		EXPECT_NEAR(constraint->GetCurrentPosition(), -0.5f, 1e-4f);
		EXPECT(std::abs(body->V.x) < 1e-4f);
	}

	{
		PhysicsWorldParam param;
		PhysicsWorld world(param);
		RigidBody* anchor = createBody(world, Vector3::Zero(), RigidType::Static);
		RigidBodyDynamic* body = createBody(world, Vector3::Zero(), RigidType::Dynamic)->CastDynamic();
		ConeConstraint* constraint = world.CreateConeConstraint(anchor, body, Vector3::Zero(), Vector3::UnitY(), Vector3::UnitY(), 0.35f);
		body->Q.FromRotationAxis(Vector3::UnitZ(), 1.1f);
		world.Simulate();
		EXPECT(constraint->GetCurrentAngle() <= 0.351f);
		constraint->SetMaxAngle(0.15f);
		body->Q.FromRotationAxis(Vector3::UnitZ(), 0.8f);
		world.Simulate();
		EXPECT(constraint->GetCurrentAngle() <= 0.151f);
	}

	{
		PhysicsWorldParam param;
		PhysicsWorld world(param);
		RigidBody* anchor = createBody(world, Vector3::Zero(), RigidType::Static);
		RigidBodyDynamic* body = createBody(world, Vector3::Zero(), RigidType::Dynamic)->CastDynamic();
		SixDOFConstraintSettings settings;
		settings.LinearMin = Vector3(-1.0f, 0.0f, 0.0f);
		settings.LinearMax = Vector3(1.0f, 0.0f, 0.0f);
		SixDOFConstraint* constraint = world.CreateSixDOFConstraint(anchor, body, Vector3::Zero(), settings);
		body->X = Vector3(4.0f, 2.0f, -3.0f);
		body->Q.FromRotationAxis(Vector3::UnitZ(), 0.7f);
		world.Simulate();
		EXPECT(constraint != nullptr);
		EXPECT_NEAR(body->X.x, 1.0f, 1e-4f);
		EXPECT(std::abs(body->X.y) < 1e-4f && std::abs(body->X.z) < 1e-4f);
		EXPECT((body->Q * Vector3::UnitX()).Dot(Vector3::UnitX()) > 0.999f);
		body->X = Vector3::Zero();
		body->V = Vector3(1.0f, 2.0f, -3.0f);
		world.Simulate();
		EXPECT(body->V.x > 0.9f);
		EXPECT(std::abs(body->V.y) < 1e-4f && std::abs(body->V.z) < 1e-4f);
		SixDOFConstraintSettings updatedSettings = constraint->GetSettings();
		updatedSettings.LinearMin = Vector3(-0.25f, 0.0f, 0.0f);
		updatedSettings.LinearMax = Vector3(0.25f, 0.0f, 0.0f);
		constraint->SetSettings(updatedSettings);
		body->X = Vector3(3.0f, 0.0f, 0.0f);
		world.Simulate();
		EXPECT_NEAR(body->X.x, 0.25f, 1e-4f);
		EXPECT(world.GetConstraintCount() == 1);
		EXPECT(world.RemoveConstraint(constraint));
		EXPECT(world.GetConstraintCount() == 0);
	}

	{
		PhysicsWorldParam param;
		PhysicsWorld world(param);
		RigidBodyDynamic* bodyA = createBody(world, Vector3(-1.0f, 0.0f, 0.0f), RigidType::Dynamic)->CastDynamic();
		RigidBodyDynamic* bodyB = createBody(world, Vector3(1.0f, 0.0f, 0.0f), RigidType::Dynamic)->CastDynamic();
		PointConstraint* constraint = world.CreatePointConstraint(bodyA, bodyB, Vector3::Zero());
		bodyA->X = Vector3(-4.0f, 0.0f, 0.0f);
		bodyB->X = Vector3(2.0f, 0.0f, 0.0f);
		world.Simulate();
		EXPECT((constraint->GetWorldAnchorB() - constraint->GetWorldAnchorA()).Length() < 1e-4f);
		EXPECT_NEAR(bodyA->X.x, -2.0f, 1e-4f);
		EXPECT_NEAR(bodyB->X.x, 0.0f, 1e-4f);
		EXPECT(world.RemoveRigidBody(bodyA));
		EXPECT(world.GetConstraintCount() == 0);
	}

	{
		PhysicsWorldParam param;
		param.gravityAcc = Vector3(0.0f, -9.8f, 0.0f);
		PhysicsWorld unconstrainedWorld(param);
		PhysicsWorld constrainedWorld(param);
		auto createFallingBox = [](PhysicsWorld& world, RigidBody** floorBody) -> RigidBodyDynamic*
		{
			RigidBodyParam staticParam;
			staticParam.rigidType = RigidType::Static;
			RigidBody* floor = world.CreateRigidBody(
				GeometryFactory::CreatePlane(Vector3::Zero(), Vector3::UnitY()), staticParam);
			if (floorBody)
			{
				*floorBody = floor;
			}
			RigidBodyParam dynamicParam;
			dynamicParam.rigidType = RigidType::Dynamic;
			dynamicParam.sleepThreshold = 0.0f;
			return world.CreateRigidBody(GeometryFactory::CreateOBB(
				Vector3(0.0f, 3.0f, 0.0f), Vector3(0.5f, 0.5f, 0.5f)), dynamicParam)->CastDynamic();
		};
		RigidBodyDynamic* referenceBox = createFallingBox(unconstrainedWorld, nullptr);
		RigidBody* sharedFloor = nullptr;
		RigidBodyDynamic* isolatedBox = createFallingBox(constrainedWorld, &sharedFloor);
		RigidBody* remoteBody = createBody(constrainedWorld, Vector3(102.0f, 0.0f, 0.0f), RigidType::Dynamic);
		EXPECT(constrainedWorld.CreatePointConstraint(sharedFloor, remoteBody, remoteBody->X) != nullptr);
		for (int frame = 0; frame < 180; ++frame)
		{
			unconstrainedWorld.Simulate();
			constrainedWorld.Simulate();
		}
		EXPECT((referenceBox->X - isolatedBox->X).Length() < 1e-4f);
		EXPECT((referenceBox->V - isolatedBox->V).Length() < 1e-4f);
	}

	{
		PhysicsWorldParam param;
		param.resolutionSolver = ResolutionPhaseSolver::LCPGlobal;
		param.solverBackend = SolverBackend::CPU;
		PhysicsWorld world(param);
		RigidBodyParam staticParam;
		staticParam.rigidType = RigidType::Static;
		RigidBody* floor = world.CreateRigidBody(
			GeometryFactory::CreatePlane(Vector3::Zero(), Vector3::UnitY()), staticParam);
		RigidBodyParam dynamicParam;
		dynamicParam.rigidType = RigidType::Dynamic;
		dynamicParam.sleepThreshold = 0.0f;
		RigidBodyDynamic* crate = world.CreateRigidBody(GeometryFactory::CreateOBB(
			Vector3(0.0f, 0.46f, 0.0f), Vector3(0.5f, 0.5f, 0.5f)), dynamicParam)->CastDynamic();
		RigidBodyDynamic* constrainedBox = world.CreateRigidBody(GeometryFactory::CreateOBB(
			Vector3(0.0f, 1.34f, 0.0f), Vector3(0.5f, 0.5f, 0.5f)), dynamicParam)->CastDynamic();
		EXPECT(world.CreateDistanceConstraint(floor, constrainedBox,
			constrainedBox->X, constrainedBox->X, 0.0f, 100.0f) != nullptr);
		world.Simulate();
		EXPECT(GetGeometryMinimumY(crate, crate->Geometries()[0]) > -0.02f);
		EXPECT(GetGeometryMinimumY(constrainedBox, constrainedBox->Geometries()[0])
			> GetGeometryMinimumY(crate, crate->Geometries()[0]) + 0.85f);
	}
}

void TestRopeSystem()
{
	printf("Running TestRopeSystem\n");

	{
		RopeSettings settings;
		settings.SegmentCount = 12;
		settings.SolverIterations = 20;
		settings.ParticleRadius = 0.05f;
		RopeSystem rope(settings, Vector3::Zero(), Vector3(6.0f, 0.0f, 0.0f));
		EXPECT(rope.GetParticleCount() == 13);
		EXPECT(rope.GetRenderGeometries().size() == 13);
		EXPECT(rope.PinParticle(0, Vector3::Zero()));
		EXPECT_FALSE(rope.PinParticle(-1, Vector3::Zero()));
		for (int frame = 0; frame < 180; ++frame)
		{
			rope.Simulate(1.0f / 60.0f);
		}
		EXPECT_SAME(rope.GetParticle(0).Position, Vector3::Zero());
		EXPECT(rope.GetParticle(rope.GetParticleCount() - 1).Position.y < -0.5f);
		for (int i = 0; i < rope.GetSegmentCount(); ++i)
		{
			const float length = (rope.GetParticle(i + 1).Position - rope.GetParticle(i).Position).Length();
			EXPECT_NEAR(length, rope.GetRestLength(), 0.03f);
			EXPECT_SAME(rope.GetRenderGeometries()[(size_t)i]->GetPosition(), rope.GetParticle(i).Position);
		}
	}

	{
		PhysicsWorldParam param;
		PhysicsWorld world(param);
		Geometry* boxShape = GeometryFactory::CreateOBB(Vector3(4.0f, 0.0f, 0.0f), Vector3(0.5f, 0.5f, 0.5f));
		boxShape->SetSimulationEnabled(false);
		RigidBodyParam bodyParam;
		bodyParam.rigidType = RigidType::Dynamic;
		bodyParam.disableGravity = true;
		RigidBodyDynamic* box = world.CreateRigidBody(boxShape, bodyParam)->CastDynamic();

		RopeSettings settings;
		settings.SegmentCount = 10;
		settings.SolverIterations = 24;
		settings.RestLength = 0.28f;
		settings.Gravity = Vector3::Zero();
		RopeSystem* rope = world.CreateRope(settings, Vector3::Zero(), box->X);
		EXPECT(rope != nullptr);
		EXPECT(world.GetRopeCount() == 1);
		EXPECT(rope->PinParticle(0, Vector3::Zero()));
		EXPECT(rope->AttachRigidBody(rope->GetParticleCount() - 1, box, box->X));
		const float initialX = box->X.x;
		for (int frame = 0; frame < 30; ++frame)
		{
			world.Simulate();
		}
		EXPECT(box->X.x < initialX - 0.5f);
		const Vector3 ropeEnd = rope->GetParticle(rope->GetParticleCount() - 1).Position;
		EXPECT((ropeEnd - box->X).Length() < 0.05f);
		EXPECT(rope->DetachRigidBody(rope->GetParticleCount() - 1));
		EXPECT_FALSE(rope->DetachRigidBody(rope->GetParticleCount() - 1));
		EXPECT(world.RemoveRope(rope));
		EXPECT(world.GetRopeCount() == 0);
	}

	{
		PhysicsWorldParam param;
		PhysicsWorld world(param);
		RigidBodyParam wallParam;
		wallParam.rigidType = RigidType::Static;
		EXPECT(world.CreateRigidBody(GeometryFactory::CreateOBB(
			Vector3::Zero(), Vector3(0.05f, 4.0f, 4.0f)), wallParam) != nullptr);

		RigidBodyParam bodyParam;
		bodyParam.rigidType = RigidType::Dynamic;
		bodyParam.motionType = MorionType::Continuous;
		bodyParam.disableGravity = true;
		RigidBodyDynamic* box = world.CreateRigidBody(GeometryFactory::CreateOBB(
			Vector3(-2.0f, 0.0f, 0.0f), Vector3(0.4f, 0.4f, 0.4f)), bodyParam)->CastDynamic();

		RopeSettings settings;
		settings.SegmentCount = 8;
		settings.SolverIterations = 32;
		settings.RestLength = 0.25f;
		settings.Gravity = Vector3::Zero();
		RopeSystem* rope = world.CreateRope(settings, Vector3(-4.0f, 0.0f, 0.0f), box->X);
		EXPECT(rope->PinParticle(0, Vector3(4.0f, 0.0f, 0.0f)));
		EXPECT(rope->AttachRigidBody(rope->GetParticleCount() - 1, box, box->X));
		world.Simulate();
		EXPECT(box->HadCCDContact);
		EXPECT(world.GetLastCCDHitCount() > 0);
		EXPECT(box->X.x < -0.44f);
	}
}

void TestClothSystem()
{
	printf("Running TestClothSystem\n");

	{
		ClothSettings settings;
		settings.Rows = 9;
		settings.Columns = 9;
		settings.Width = 4.0f;
		settings.Height = 4.0f;
		settings.SolverIterations = 16;
		ClothSystem cloth(settings, Vector3::Zero(), Vector3::UnitX(), Vector3::UnitZ());
		const Vector3 topLeft = cloth.GetParticle(0, 0).Position;
		const Vector3 topRight = cloth.GetParticle(0, 8).Position;
		EXPECT(cloth.PinParticle(0, 0, topLeft));
		EXPECT(cloth.PinParticle(0, 8, topRight));
		EXPECT_FALSE(cloth.PinParticle(-1, 0, Vector3::Zero()));
		for (int frame = 0; frame < 120; ++frame)
		{
			cloth.Simulate(1.0f / 60.0f);
		}
		EXPECT_SAME(cloth.GetParticle(0, 0).Position, topLeft);
		EXPECT_SAME(cloth.GetParticle(0, 8).Position, topRight);
		EXPECT(cloth.GetParticle(4, 4).Position.y < -0.2f);
		const float horizontalLength = (cloth.GetParticle(4, 5).Position - cloth.GetParticle(4, 4).Position).Length();
		EXPECT_NEAR(horizontalLength, 0.5f, 0.08f);
		Geometry* renderGeometry = cloth.GetRenderGeometry();
		EXPECT(renderGeometry != nullptr && renderGeometry->GetShapeType() == PrimitiveType::TRIANGLE_MESH);
		TriangleMesh* renderMesh = renderGeometry ? renderGeometry->GetShapeObj<TriangleMesh>() : nullptr;
		EXPECT(renderMesh != nullptr && renderMesh->NumVertices == 81 && renderMesh->NumTriangles == 128);
		EXPECT(renderMesh == nullptr || (renderMesh->Vertices[40] - cloth.GetParticle(4, 4).Position).Length() < 1e-6f);
		if (renderMesh != nullptr)
		{
			renderMesh->CalculateWeightAverageNormals();
			EXPECT(renderMesh->mNormals.size() == renderMesh->NumVertices);
			cloth.Simulate(1.0f / 60.0f);
			EXPECT(renderMesh->mNormals.empty());
		}
	}

	{
		PhysicsWorldParam param;
		PhysicsWorld world(param);
		Geometry* sphereShape = GeometryFactory::CreateSphere(Vector3::Zero(), 1.0f);
		sphereShape->SetSimulationEnabled(false);
		RigidBodyParam bodyParam;
		bodyParam.rigidType = RigidType::Static;
		RigidBody* sphere = world.CreateRigidBody(sphereShape, bodyParam);

		ClothSettings settings;
		settings.Rows = 5;
		settings.Columns = 5;
		settings.Width = 4.0f;
		settings.Height = 4.0f;
		settings.Gravity = Vector3::Zero();
		ClothSystem* cloth = world.CreateCloth(settings, Vector3::Zero(), Vector3::UnitX(), Vector3::UnitY());
		EXPECT(cloth->AddSphereCollider(sphere, 1.0f, Vector3::Zero()));
		world.Simulate();
		EXPECT((cloth->GetParticle(2, 2).Position - sphere->X).Length() >= 1.0f + settings.Thickness - 1e-4f);
		EXPECT(cloth->RemoveSphereCollider(sphere));
		EXPECT_FALSE(cloth->RemoveSphereCollider(sphere));
		EXPECT(world.RemoveCloth(cloth));
		EXPECT(world.GetClothCount() == 0);
	}

	{
		PhysicsWorldParam param;
		PhysicsWorld world(param);
		Geometry* sphereShape = GeometryFactory::CreateSphere(Vector3(0.0f, 0.0f, -3.0f), 0.8f);
		sphereShape->SetSimulationEnabled(false);
		RigidBodyParam bodyParam;
		bodyParam.rigidType = RigidType::Dynamic;
		bodyParam.disableGravity = true;
		bodyParam.linearVelocity = Vector3(0.0f, 0.0f, 6.0f);
		RigidBodyDynamic* sphere = world.CreateRigidBody(sphereShape, bodyParam)->CastDynamic();

		ClothSettings settings;
		settings.Rows = 11;
		settings.Columns = 11;
		settings.Width = 6.0f;
		settings.Height = 6.0f;
		settings.Gravity = Vector3::Zero();
		settings.SolverIterations = 16;
		ClothSystem* cloth = world.CreateCloth(settings, Vector3::Zero(), Vector3::UnitX(), Vector3::UnitY());
		cloth->PinParticle(0, 0, cloth->GetParticle(0, 0).Position);
		cloth->PinParticle(0, 10, cloth->GetParticle(0, 10).Position);
		EXPECT(cloth->AddSphereCollider(sphere, 0.8f, sphere->X));
		float maximumCenterDisplacement = 0.0f;
		for (int frame = 0; frame < 90; ++frame)
		{
			world.Simulate();
			maximumCenterDisplacement = std::max(maximumCenterDisplacement, std::abs(cloth->GetParticle(5, 5).Position.z));
		}
		EXPECT(sphere->X.z > 4.0f);
		EXPECT(maximumCenterDisplacement > 0.25f);
	}
}

void TestRagdollSystem()
{
	printf("Running TestRagdollSystem\n");

	int selfContactCount = 0;
	int environmentContactCount = 0;
	PhysicsWorldParam param;
	param.gravityAcc = Vector3(0.0f, -9.8f, 0.0f);
	param.onContact = [&selfContactCount, &environmentContactCount](const PhysicsContactInfo& contact)
	{
		if (contact.BodyA && contact.BodyB && contact.BodyA->CollisionGroup != nullptr
			&& contact.BodyA->CollisionGroup == contact.BodyB->CollisionGroup)
		{
			++selfContactCount;
		}
		else
		{
			++environmentContactCount;
		}
	};
	PhysicsWorld world(param);

	RigidBodyParam groundParam;
	groundParam.rigidType = RigidType::Static;
	Geometry* ground = GeometryFactory::CreatePlane(Vector3::Zero(), Vector3::UnitY());
	EXPECT(world.CreateRigidBody(ground, groundParam) != nullptr);

	RagdollSettings settings;
	settings.Scale = 1.0f;
	Ragdoll* ragdoll = world.CreateRagdoll(settings, Transform(Vector3(0.0f, 6.0f, 0.0f)));
	EXPECT(ragdoll != nullptr && ragdoll->IsValid());
	if (ragdoll == nullptr)
	{
		return;
	}
	EXPECT(ragdoll->GetBodyCount() == (size_t)RagdollBone::Count);
	EXPECT(ragdoll->GetConstraintCount() == 10);
	EXPECT(world.GetConstraintCount() == 10);
	EXPECT(world.GetRagdollCount() == 1);

	int coneCount = 0;
	int hingeCount = 0;
	for (PositionConstraint* constraint : ragdoll->GetConstraints())
	{
		coneCount += constraint->GetType() == PositionConstraintType::Cone ? 1 : 0;
		hingeCount += constraint->GetType() == PositionConstraintType::Hinge ? 1 : 0;
	}
	EXPECT(coneCount == 6);
	EXPECT(hingeCount == 4);

	RigidBodyDynamic* pelvis = ragdoll->GetBone(RagdollBone::Pelvis);
	const float initialPelvisHeight = pelvis->X.y;
	for (int frame = 0; frame < 300; ++frame)
	{
		world.Simulate();
	}
	EXPECT(pelvis->X.y < initialPelvisHeight - 1.0f);
	EXPECT(pelvis->X.y > -0.1f);
	EXPECT(selfContactCount > 0);
	EXPECT(environmentContactCount > 0);
	EXPECT(GetMaximumRagdollAnchorError(ragdoll) < 0.03f);
	ExpectRagdollAngularLimits(ragdoll);
	for (RigidBodyDynamic* body : ragdoll->GetBodies())
	{
		EXPECT(std::isfinite(body->X.x) && std::isfinite(body->X.y) && std::isfinite(body->X.z));
		EXPECT((body->X - pelvis->X).Length() < 5.0f);
		EXPECT(body->CollisionGroup == ragdoll);
		EXPECT(body->EnableCollisionWithinGroup);
		EXPECT(body->mMotionType == MorionType::Discrete);
		for (Geometry* geometry : body->Geometries())
		{
			EXPECT(GetGeometryMinimumY(body, geometry) > -0.05f);
		}
	}
	for (PositionConstraint* constraint : ragdoll->GetConstraints())
	{
		EXPECT(constraint->GetBodyA()->IsCollisionIgnoredWith(constraint->GetBodyB()));
	}
	EXPECT(pelvis->CanCollideWith(ragdoll->GetBone(RagdollBone::Head)));

	EXPECT(world.RemoveRagdoll(ragdoll));
	EXPECT(world.GetRagdollCount() == 0);
	EXPECT(world.GetConstraintCount() == 0);
	std::vector<Geometry*> remainingGeometries;
	world.GetAllGeometries(&remainingGeometries);
	EXPECT(remainingGeometries.size() == 1);

	{
		int triangleMeshContactCount = 0;
		PhysicsWorldParam meshParam;
		meshParam.gravityAcc = Vector3(0.0f, -9.8f, 0.0f);
		meshParam.onContact = [&triangleMeshContactCount](const PhysicsContactInfo& contact)
		{
			if ((contact.GeomA && contact.GeomA->GetShapeType() == PrimitiveType::TRIANGLE_MESH)
				|| (contact.GeomB && contact.GeomB->GetShapeType() == PrimitiveType::TRIANGLE_MESH))
			{
				++triangleMeshContactCount;
			}
		};
		PhysicsWorld meshWorld(meshParam);
		Geometry* meshGeometry = GeometryFactory::CreateTriangleMesh();
		TriangleMesh* triangleMesh = meshGeometry->GetShapeObj<TriangleMesh>();
		triangleMesh->AddAABB(Vector3(-5.0f, -1.0f, -5.0f), Vector3(5.0f, 0.0f, 5.0f));
		triangleMesh->Compact();
		triangleMesh->BuildBVH();
		meshGeometry->SetShapeBounds(triangleMesh->BoundingVolume);
		RigidBodyParam meshBodyParam;
		meshBodyParam.rigidType = RigidType::Static;
		EXPECT(meshWorld.CreateRigidBody(meshGeometry, meshBodyParam) != nullptr);
		Ragdoll* meshRagdoll = meshWorld.CreateRagdoll(settings, Transform(Vector3(0.0f, 5.0f, 0.0f)));
		EXPECT(meshRagdoll != nullptr);
		for (int frame = 0; frame < 240; ++frame)
		{
			meshWorld.Simulate();
		}
		EXPECT(triangleMeshContactCount > 0);
		EXPECT(meshRagdoll->GetBone(RagdollBone::Pelvis)->X.y > -0.1f);
		EXPECT(GetMaximumRagdollAnchorError(meshRagdoll) < 0.03f);
		ExpectRagdollAngularLimits(meshRagdoll);
		for (RigidBodyDynamic* body : meshRagdoll->GetBodies())
		{
			for (Geometry* geometry : body->Geometries())
			{
				EXPECT(GetGeometryMinimumY(body, geometry) > -0.05f);
			}
		}
	}

	{
		PhysicsWorldParam freeFallParam;
		PhysicsWorld freeFallWorld(freeFallParam);
		RagdollSettings freeFallSettings;
		freeFallSettings.DisableSelfCollision = true;
		freeFallSettings.JointVelocity = 5.0f;
		Ragdoll* freeFallRagdoll = freeFallWorld.CreateRagdoll(freeFallSettings, Transform::Identity());
		EXPECT(freeFallRagdoll != nullptr);
		RigidBodyDynamic* upperArm = freeFallRagdoll->GetBone(RagdollBone::LeftUpperArm);
		RigidBodyDynamic* lowerArm = freeFallRagdoll->GetBone(RagdollBone::LeftLowerArm);
		const Vector3 initialDirection = upperArm->WorldToBodyLocalDirection((lowerArm->X - upperArm->X).SafeUnit());
		float maximumAnchorError = 0.0f;
		for (int frame = 0; frame < 60; ++frame)
		{
			freeFallWorld.Simulate();
			for (PositionConstraint* constraint : freeFallRagdoll->GetConstraints())
			{
				PointConstraint* point = dynamic_cast<PointConstraint*>(constraint);
				if (point)
				{
					maximumAnchorError = std::max(maximumAnchorError,
						(point->GetWorldAnchorB() - point->GetWorldAnchorA()).Length());
				}
			}
		}
		const Vector3 finalDirection = upperArm->WorldToBodyLocalDirection((lowerArm->X - upperArm->X).SafeUnit());
		EXPECT(initialDirection.Dot(finalDirection) < 0.98f);
		EXPECT(maximumAnchorError < 0.03f);
	}

	{
		int stairContactCount = 0;
		PhysicsWorldParam stairParam;
		stairParam.gravityAcc = Vector3(0.0f, -9.8f, 0.0f);
		stairParam.onContact = [&stairContactCount](const PhysicsContactInfo& contact)
		{
			if ((contact.GeomA && contact.GeomA->GetShapeType() == PrimitiveType::TRIANGLE_MESH)
				|| (contact.GeomB && contact.GeomB->GetShapeType() == PrimitiveType::TRIANGLE_MESH))
			{
				++stairContactCount;
			}
		};
		PhysicsWorld stairWorld(stairParam);
		Geometry* stairGeometry = GeometryFactory::CreateTriangleMesh();
		TriangleMesh* stairMesh = stairGeometry->GetShapeObj<TriangleMesh>();
		const int stepCount = 10;
		const float stepWidth = 1.2f;
		const float stepHeight = 0.55f;
		const float stairBaseY = -2.0f;
		const float totalWidth = stepCount * stepWidth;
		for (int step = 0; step < stepCount; ++step)
		{
			const float minX = -totalWidth * 0.5f + step * stepWidth;
			stairMesh->AddAABB(Vector3(minX, stairBaseY, -3.5f),
				Vector3(minX + stepWidth, stairBaseY + (step + 1) * stepHeight, 3.5f));
		}
		stairMesh->Compact();
		stairMesh->BuildBVH();
		stairGeometry->SetShapeBounds(stairMesh->BoundingVolume);
		RigidBodyParam stairBodyParam;
		stairBodyParam.rigidType = RigidType::Static;
		RigidBody* stairBody = stairWorld.CreateRigidBody(stairGeometry, stairBodyParam);
		EXPECT(stairBody != nullptr);

		RagdollSettings stairSettings;
		stairSettings.Scale = 1.15f;
		stairSettings.EnableCCD = false;
		stairSettings.DisableSelfCollision = false;
		Quaternion stairRotation;
		stairRotation.FromRotationAxis(Vector3::UnitZ(), 75.0f * PI_OVER_180);
		Ragdoll* stairRagdoll = stairWorld.CreateRagdoll(stairSettings,
			Transform(Vector3(1.5f, 12.0f, 0.0f), stairRotation));
		EXPECT(stairRagdoll != nullptr);
		const float initialPelvisX = stairRagdoll->GetBone(RagdollBone::Pelvis)->X.x;
		int ccdHitCount = 0;
		for (int frame = 0; frame < 360; ++frame)
		{
			stairWorld.Simulate();
			ccdHitCount += stairWorld.GetLastCCDHitCount();
		}
		EXPECT(stairContactCount > 0);
		EXPECT(ccdHitCount == 0);
		EXPECT(GetMaximumRagdollAnchorError(stairRagdoll) < 0.03f);
		ExpectRagdollAngularLimits(stairRagdoll);
		RigidBodyDynamic* stairPelvis = stairRagdoll->GetBone(RagdollBone::Pelvis);
		EXPECT(stairPelvis->X.x < initialPelvisX - 1.0f);
		const int pelvisStep = std::max(0, std::min(stepCount - 1,
			(int)((stairPelvis->X.x + totalWidth * 0.5f) / stepWidth)));
		const float pelvisStepTop = stairBaseY + (pelvisStep + 1) * stepHeight;
		EXPECT(GetGeometryMinimumY(stairPelvis, stairPelvis->Geometries()[0]) > pelvisStepTop - 0.05f);
		for (RigidBodyDynamic* body : stairRagdoll->GetBodies())
		{
			EXPECT(std::isfinite(body->X.x) && std::isfinite(body->X.y) && std::isfinite(body->X.z));
			for (Geometry* geometry : body->Geometries())
			{
				EXPECT(GetGeometryMinimumY(body, geometry) > stairBaseY - 0.08f);
				EXPECT(EstimateMinimumSeparationDistance(body, geometry, stairBody, stairGeometry) < 0.03f);
				const float centerX = body->GetGeometryTransform(geometry).pos.x;
				if (centerX >= -totalWidth * 0.5f && centerX <= totalWidth * 0.5f)
				{
					const int shapeStep = std::max(0, std::min(stepCount - 1,
						(int)((centerX + totalWidth * 0.5f) / stepWidth)));
					const float shapeStepTop = stairBaseY + (shapeStep + 1) * stepHeight;
					EXPECT(GetGeometryMinimumY(body, geometry) > shapeStepTop - 0.05f);
				}
			}
		}
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
	TestGlobalLCPResetWarmStartState();
	TestSleepingBodyWakesForAppliedLoads();
	TestBoxBoxSATContactBasis();
	TestGlobalLCPContactSolver();
	TestFiveBoxStackStability();
	return;
}
