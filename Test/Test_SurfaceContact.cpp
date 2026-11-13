#include "Test.h"
#include "../Src/Deformation/SurfaceContactConstraint.h"
#include "../Src/Maths/Quaternion.h"
#include <stdexcept>

namespace
{
	using namespace Riemann;
	SurfaceContact Witness(const Vector3& Normal = Vector3::UnitZ())
	{
		SurfaceContact C; C.Normal = Normal; C.WeightsA = C.WeightsB = Vector3(1, 0, 0); C.Distance = 0; return C;
	}
	void WeightedResponse()
	{
		for (float MassScale : { 0.01f, 1.0f, 1000.0f })
		{
			Vector3 Point(0.2f, 0.3f, -0.2f), Face[] = { Vector3(0, 0, 0), Vector3(1, 0, 0), Vector3(0, 1, 0) };
			SurfaceContactParticle A[] = { { &Point, MassScale } }, B[] = { { &Face[0], 2 * MassScale }, { &Face[1], 3 * MassScale }, { &Face[2], 4 * MassScale } };
			auto C = Witness(); C.WeightsB = Vector3(0.5f, 0.2f, 0.3f); C.PointA = C.PointB = Vector3(0.2f, 0.3f, 0);
			SurfaceContactMaterial Material; Material.StaticFriction = Material.DynamicFriction = 0;
			SurfaceContactConstraint Row(A, 1, B, 3, C, 0.1, Material);
			const Vector3 MomentumBefore = Point + Face[0] / 2 + Face[1] / 3 + Face[2] / 4;
			EXPECT_NEAR(Row.GetEffectiveInverseMass(), 1.98 * MassScale, 2e-6 * MassScale);
			EXPECT(Row.Solve(0.01f)); EXPECT_NEAR(Row.GetNormalMultiplier(), 0.3 / (1.98 * MassScale), 1e-6 / MassScale);
			const Vector3 Weighted = Face[0] * 0.5f + Face[1] * 0.2f + Face[2] * 0.3f;
			EXPECT_NEAR(Point.z - Weighted.z, 0.1f, 1e-6f);
			EXPECT_NEAR((Point + Face[0] / 2 + Face[1] / 3 + Face[2] / 4 - MomentumBefore).Length(), 0, 1e-6f);
			const Vector3 Once = Point; for (int I = 0; I < 20; ++I) Row.Solve(0.01f);
			EXPECT_NEAR((Point - Once).Length(), 0, 1e-6f);
		}
	}
	void ComplianceAndFixedParticles()
	{
		for (float Dt : { 0.01f, 0.02f })
		{
			Vector3 Point(0, 0, -0.2f), Fixed = Vector3::Zero();
			SurfaceContactParticle A{ &Point, 2 }, B{ &Fixed, 0 }; auto C = Witness();
			SurfaceContactMaterial M; M.Compliance = 0.001; M.StaticFriction = M.DynamicFriction = 0;
			SurfaceContactConstraint Row(&A, 1, &B, 1, C, 0.1, M); Row.Solve(Dt);
			const double Alpha = M.Compliance / ((double)Dt * Dt), Expected = 0.3 / (2 + Alpha);
			EXPECT_NEAR(Row.GetNormalMultiplier(), Expected, 1e-8); EXPECT_NEAR(Point.z, -0.2 + 2 * Expected, 1e-7);
			EXPECT(Fixed == Vector3::Zero()); EXPECT_NEAR(Point.z - 0.1 + Alpha * Row.GetNormalMultiplier(), 0, 1e-7);
		}
		Vector3 Point(0, 0, -1), Fixed = Vector3::Zero(); SurfaceContactParticle A{ &Point, 0 }, B{ &Fixed, 0 };
		SurfaceContactConstraint Row(&A, 1, &B, 1, Witness(), 0); EXPECT(!Row.Solve(0.01f)); EXPECT(Point.z == -1);
	}
	void CoulombFriction()
	{
		for (const Vector3& Travel : { Vector3(0.05f, 0, 0), Vector3(0.6f, 0.8f, 0) })
		{
			Vector3 Point = Travel + Vector3(0, 0, -0.1f), Fixed = Vector3::Zero();
			SurfaceContactParticle A{ &Point, 1 }, B{ &Fixed, 0 };
			SurfaceContactMaterial M; M.StaticFriction = 1; M.DynamicFriction = 0.5;
			SurfaceContactConstraint Row(&A, 1, &B, 1, Witness(), 0, M);
			Row.Solve(0.01f); EXPECT_NEAR(Point.z, 0, 1e-7f);
			const bool Sliding = Travel.Length() > 0.1f; EXPECT(Row.IsSliding() == Sliding);
			const Vector3 Expected = Sliding ? Travel * 0.95f : Vector3::Zero();
			EXPECT_NEAR((Point - Expected).Length(), 0, 1e-6f);
			const Vector3 Once = Point;
			for (int I = 0; I < 30; ++I) Row.Solve(0.01f);
			EXPECT_NEAR((Point - Once).Length(), 0, 1e-6f);
			const auto Tangent = Row.GetTangentMultiplier();
			EXPECT_NEAR(std::hypot(std::hypot(Tangent.x, Tangent.y), Tangent.z), 0.05, 1e-7);
			// Release the old friction correction when another row removes normal load.
			Point.z = 1; Row.Solve(0.01f); EXPECT(Row.GetNormalMultiplier() == 0);
			EXPECT_NEAR((Vector3(Point.x, Point.y, 0) - Travel).Length(), 0, 1e-6f);
			EXPECT(Row.GetTangentMultiplier().SquareLength() == 0);
		}
		// Motion of a zero-mass collider contributes to relative tangential travel.
		Vector3 Point(0, 0, -0.1f), Platform(0.05f, 0, 0); SurfaceContactParticle A{ &Point, 1 }, B{ &Platform, 0 };
		SurfaceContactMaterial M; M.StaticFriction = M.DynamicFriction = 1;
		SurfaceContactConstraint Row(&A, 1, &B, 1, Witness(), 0, M); Row.Solve(0.01f);
		EXPECT_NEAR((Point - Platform).Length(), 0, 1e-6f); EXPECT(Platform == Vector3(0.05f, 0, 0));
	}
	void SharedDegreesOfFreedom()
	{
		Vector3 Shared = Vector3::Zero(), PointA(2, 0, 0), PointB = Vector3::Zero();
		SurfaceContactParticle A[] = { { &Shared, 1 }, { &PointA, 1 } }, B[] = { { &Shared, 1 }, { &PointB, 1 } };
		auto C = Witness(Vector3::UnitY()); C.WeightsA = Vector3(0.5f, 0.5f, 0); C.WeightsB = Vector3(0.25f, 0.75f, 0);
		C.PointA = Vector3(1, 0, 0); SurfaceContactConstraint Row(A, 2, B, 2, C, 0.1);
		EXPECT_NEAR(Row.GetEffectiveInverseMass(), 0.875, 1e-12); Row.Solve(0.01f);
		EXPECT_NEAR((Shared + PointA + PointB - Vector3(2, 0, 0)).Length(), 0, 1e-6f);
		EXPECT_NEAR(Shared.y * 0.25f + PointA.y * 0.5f - PointB.y * 0.75f, 0.1f, 1e-6f);
		SurfaceContactConstraint Cancelled(A, 2, A, 2, C, 0.1);
		// Different weights still describe a nonzero gradient on the same segment.
		EXPECT_NEAR(Cancelled.GetEffectiveInverseMass(), 0.125, 1e-12);
		C.WeightsB = C.WeightsA; SurfaceContactConstraint Identical(A, 2, A, 2, C, 0.1);
		EXPECT(Identical.GetEffectiveInverseMass() == 0); EXPECT(!Identical.Solve(0.01f));
	}
	void CovarianceAndTwoWayFriction()
	{
		Vector3 A(0.6f, 0.8f, -0.2f), B = Vector3::Zero();
		const Vector3 Before = A / 2 + B / 3;
		SurfaceContactParticle PA{ &A, 2 }, PB{ &B, 3 }; SurfaceContactMaterial Material;
		Material.StaticFriction = 0.8; Material.DynamicFriction = 0.5;
		SurfaceContactConstraint Row(&PA, 1, &PB, 1, Witness(), 0, Material); Row.Solve(0.02f);
		EXPECT(Row.IsSliding()); EXPECT_NEAR((A / 2 + B / 3 - Before).Length(), 0, 1e-6f);
		EXPECT_NEAR((A - B - Vector3(0.54f, 0.72f, 0)).Length(), 0, 1e-6f);
		Quaternion Rotation; Rotation.FromRotationAxis(Vector3(1, 2, 3), 0.7f);
		for (float Scale : { 0.001f, 1.0f, 1000.0f })
		{
			const Vector3 Translation = Vector3(13, -17, 29) * Scale;
			Vector3 AA = Rotation * Vector3(0.6f, 0.8f, -0.2f) * Scale + Translation, BB = Translation;
			SurfaceContactParticle PAA{ &AA, 2 }, PBB{ &BB, 3 }; auto C = Witness(Rotation * Vector3::UnitZ());
			C.PointA = C.PointB = Translation;
			SurfaceContactConstraint Rotated(&PAA, 1, &PBB, 1, C, 0, Material); Rotated.Solve(0.02f);
			EXPECT_NEAR((AA - (Rotation * A * Scale + Translation)).Length(), 0, 5e-6 * Scale);
			EXPECT_NEAR((BB - (Rotation * B * Scale + Translation)).Length(), 0, 5e-6 * Scale);
			EXPECT_NEAR(Rotated.GetNormalMultiplier(), Row.GetNormalMultiplier() * Scale, 2e-6 * Scale);
		}
	}
	void NonlinearRefresh()
	{
		Vector3 Position(0, 0, -0.2f), Fixed = Vector3::Zero(); SurfaceContactParticle A{ &Position, 1 }, B{ &Fixed, 0 };
		auto C = Witness(); SurfaceContactMaterial M; M.Compliance = 0.01; M.StaticFriction = M.DynamicFriction = 0;
		SurfaceContactConstraint Soft(&A, 1, &B, 1, C, 0, M); Soft.Solve(0.1f);
		const Vector3 Equilibrium = Position; const double Lambda = Soft.GetNormalMultiplier();
		for (int I = 0; I < 12; ++I) { Soft.Refresh(&A, 1, &B, 1, C, 0); Soft.Solve(0.1f); }
		EXPECT_NEAR((Position - Equilibrium).Length(), 0, 1e-6f); EXPECT_NEAR(Soft.GetNormalMultiplier(), Lambda, 1e-6);
		Position = Vector3(0.1f, 0, -0.2f); M.Compliance = 0; M.StaticFriction = 1; M.DynamicFriction = 0.5;
		SurfaceContactConstraint Row(&A, 1, &B, 1, C, 0, M); Row.Solve(0.1f);
		Quaternion Q; Q.FromRotationAxis(Vector3::UnitY(), 0.6f); C.Normal = Q * Vector3::UnitZ(); Position = Q * Position;
		const Vector3 ExpectedTangent = Q * Vector3(-0.1f, 0, 0), Before = Position;
		Row.Refresh(&A, 1, &B, 1, C, 0); Row.Solve(0.1f); const auto Tangent = Row.GetTangentMultiplier();
		EXPECT_NEAR(Tangent.x, ExpectedTangent.x, 1e-6); EXPECT_NEAR(Tangent.z, ExpectedTangent.z, 1e-6);
		EXPECT_NEAR((Position - Before).Length(), 0, 1e-6f); EXPECT_NEAR(Row.GetNormalMultiplier(), 0.2, 1e-6);
		C.Conservative = true; bool Rejected = false;
		try { Row.Refresh(&A, 1, &B, 1, C, 0); } catch (const std::invalid_argument&) { Rejected = true; }
		EXPECT(Rejected); EXPECT_NEAR(Row.GetNormalMultiplier(), 0.2, 1e-6);
	}
	void ConflictingFrictionAndSeparation()
	{
		Vector3 Position(-0.2f, 0, -0.2f), Fixed = Vector3::Zero();
		SurfaceContactParticle A{ &Position, 1 }, B{ &Fixed, 0 };
		auto Z = Witness(), X = Witness(Vector3::UnitX());
		Z.PointA = Vector3(-0.2f, 0, 0); X.PointA = Vector3(0, 0, -0.2f);
		SurfaceContactMaterial Material; Material.StaticFriction = Material.DynamicFriction = 10;
		SurfaceContactConstraint RowZ(&A, 1, &B, 1, Z, 0, Material), RowX(&A, 1, &B, 1, X, 0, Material);
		for (int I = 0; I < 8; ++I) { RowZ.Solve(0.1f); RowX.Solve(0.1f); }
		EXPECT(RowZ.GetNormalGap() < -0.1);
		const auto Tangent = RowZ.GetTangentMultiplier(); const double Normal = RowZ.GetNormalMultiplier();
		EXPECT(RowZ.ProjectSeparation()); RowX.ProjectSeparation();
		EXPECT(RowZ.GetNormalGap() >= -1e-6 && RowX.GetNormalGap() >= -1e-6);
		EXPECT(RowZ.GetTangentMultiplier() == Tangent); EXPECT(RowZ.GetNormalMultiplier() > Normal);
		Position += Vector3(1, 0, 1); const Vector3 Away = Position;
		EXPECT(!RowZ.ProjectSeparation() && !RowX.ProjectSeparation()); EXPECT(Position == Away);

		Vector3 P(0, 0, -0.2f), Q = Vector3::Zero();
		SurfaceContactParticle PA{ &P, 2 }, PB{ &Q, 3 }; const Vector3 Center = P / 2 + Q / 3;
		SurfaceContactConstraint Pair(&PA, 1, &PB, 1, Witness(), 0.1, Material); EXPECT(Pair.ProjectSeparation());
		EXPECT_NEAR(P.z - Q.z, 0.1, 1e-6); EXPECT_NEAR((P / 2 + Q / 3 - Center).Length(), 0, 1e-6);
		PA.InverseMass = PB.InverseMass = 0; P.z = -0.2f; Q.SetZero();
		SurfaceContactConstraint Immovable(&PA, 1, &PB, 1, Witness(), 0.1, Material);
		EXPECT(!Immovable.ProjectSeparation()); EXPECT(Immovable.GetNormalGap() < -0.2);
	}
	void ContinuousResponseAndValidation()
	{
		SurfaceMotion Point, Face; Point.Start[0] = Vector3(-1, 0, 1); Point.End[0] = Vector3(1, 0, -1);
		Face.Count = 3; Face.Start[0] = Vector3(-10, -10, 0); Face.Start[1] = Vector3(10, -10, 0); Face.Start[2] = Vector3(0, 10, 0);
		for (int I = 0; I < 3; ++I) Face.End[I] = Face.Start[I];
		SurfaceContact Contact; SurfaceCollisionSettings S; EXPECT(SurfaceCollision::Sweep(Point, Face, S, Contact));
		EXPECT(!Contact.Conservative); SurfaceContactParticle A{ &Point.End[0], 1 }, B[3];
		for (int I = 0; I < 3; ++I) B[I] = { &Face.End[I], 0 };
		SurfaceContactMaterial M; M.StaticFriction = 1.5; M.DynamicFriction = 0.5;
		SurfaceContactConstraint Row(&A, 1, B, 3, Contact, 0, M); Row.Solve(0.01f);
		EXPECT_NEAR(Point.End[0].z, 0, 1e-6f); EXPECT_NEAR(Point.End[0].x, Contact.PointA.x, 5e-5f); EXPECT(!Row.IsSliding());
		auto Reject = [](auto Function) { bool Failed = false; try { Function(); } catch (const std::exception&) { Failed = true; } EXPECT(Failed); };
		Contact.Conservative = true; Reject([&] { SurfaceContactConstraint Bad(&A, 1, B, 3, Contact, 0); });
		Contact.Conservative = false; Contact.Normal.SetZero(); Reject([&] { SurfaceContactConstraint Bad(&A, 1, B, 3, Contact, 0); });
		Contact = Witness(); Contact.WeightsA.x = 0.5f; Reject([&] { SurfaceContactConstraint Bad(&A, 1, B, 1, Contact, 0); });
		Contact = Witness(); Reject([&] { SurfaceContactConstraint Bad(nullptr, 1, B, 1, Contact, 0); });
		Reject([&] { Row.Solve(0); }); Reject([&] { Row.Solve(std::numeric_limits<float>::infinity()); });
		M.DynamicFriction = 2; EXPECT(!M.IsValid()); M = SurfaceContactMaterial(); M.Compliance = -1; EXPECT(!M.IsValid());
		SurfaceContactParticle Conflict = A; Conflict.InverseMass = 2;
		Reject([&] { SurfaceContactConstraint Bad(&A, 1, &Conflict, 1, Contact, 0); });
		Point.End[0].x = std::numeric_limits<float>::quiet_NaN(); Reject([&] { Row.Solve(0.01f); });
	}
}
void TestSurfaceContact()
{
	WeightedResponse(); ComplianceAndFixedParticles(); CoulombFriction(); SharedDegreesOfFreedom(); CovarianceAndTwoWayFriction(); NonlinearRefresh(); ConflictingFrictionAndSeparation(); ContinuousResponseAndValidation();
}
