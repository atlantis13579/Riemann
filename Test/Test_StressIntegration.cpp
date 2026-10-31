#include "Test.h"

#include "../Src/Collision/GeometryObject.h"
#include "../Src/Destruction/DestructionSet.h"
#include "../Src/Destruction/StrainSolver.h"
#include "../Src/RigidBodyDynamics/PhysicsWorld.h"

using namespace Riemann;

namespace
{
	void BuildStressChain(DestructionSet& set, int count, float strength, bool explicitIndices = true)
	{
		set.GetDestructionConstants().MinClusterVolume = 0.0f;
		for (int index = 0; index < count; ++index)
		{
			set.AddChunk(Box3(Vector3((float)index, 0.0f, 0.0f),
				Vector3((float)index + 1.0f, 1.0f, 1.0f)), 1.0f, index == 0);
			if (index > 0) EXPECT(set.AddBond(index - 1, index, DestructionBondType::Connect,
				strength, explicitIndices ? 100 + index : -1));
		}
		set.RebuildClustersFromGraph();
	}

	StrainSolverSettings IntegrationStressSettings(float hardness = 1.0f)
	{
		StrainSolverSettings settings;
		settings.Hardness = hardness;
		settings.StressLinearFactor = 1.0f;
		settings.StressAngularFactor = 1.0f;
		settings.MaxIterations = 128;
		return settings;
	}

	void TestStressAutomaticFracture()
	{
		DestructionSet set;
		BuildStressChain(set, 2, 0.01f, false);
		EXPECT(set.GetStressSolver() == nullptr);
		EXPECT_FALSE(set.AddStressImpulse(1, set.GetChunks()[1].Centroid, Vector3::UnitX()));
		set.EnableStressSolver(IntegrationStressSettings());
		EXPECT(set.GetStressSolver() != nullptr);
		EXPECT(set.GetStressSolver()->GetBondCount() == 1);
		const Bond* forward = set.GetGraph().FindBond(0, 1);
		const Bond* reverse = set.GetGraph().FindBond(1, 0);
		EXPECT(forward != nullptr && reverse != nullptr);
		if (forward && reverse)
		{
			EXPECT(forward->bond_index >= 0);
			EXPECT(forward->bond_index == reverse->bond_index);
		}
		EXPECT(set.AddStressImpulse(1, set.GetChunks()[1].Centroid, Vector3(100.0f, 0.0f, 0.0f)));
		set.Update(1.0f / 60.0f);
		EXPECT_FALSE(set.GetGraph().HasBond(0, 1));
		EXPECT(set.GetStressSolver()->GetBondCount() == 0);
		EXPECT(set.FindClusterContaining(0) != set.FindClusterContaining(1));
	}

	void TestStressTopologySynchronization()
	{
		DestructionSet set;
		BuildStressChain(set, 3, 1.0f);
		set.EnableStressSolver(IntegrationStressSettings(1.0e8f));
		EXPECT(set.GetStressSolver()->GetBondCount() == 2);
		set.GetGraph().BreakBond(0, 1);
		set.Update(1.0f / 60.0f);
		EXPECT(set.GetStressSolver()->GetBondCount() == 1);
		EXPECT(set.GetStressSolver()->GetBondInfo(101) == nullptr);
		EXPECT(set.GetStressSolver()->GetBondInfo(102) != nullptr);
		EXPECT(set.GetGraph().AddBond(0, 1, DestructionBondType::Connect, 1.0f, 107));
		set.Update(1.0f / 60.0f);
		EXPECT(set.GetStressSolver()->GetBondCount() == 2);
		EXPECT(set.GetStressSolver()->GetBondInfo(107) != nullptr);
		EXPECT(set.GetStressSolver()->GetBondInfo(101) == nullptr);

		set.GetChunks()[2].IsFree = true;
		EXPECT_FALSE(set.AddStressImpulse(2, set.GetChunks()[2].Centroid, Vector3::UnitX()));
		set.Update(1.0f / 60.0f);
		EXPECT(set.GetStressSolver()->GetBondCount() == 1);
		EXPECT(set.GetStressSolver()->GetBondInfo(102) == nullptr);
		set.GetChunks()[1].IsSmashed = true;
		EXPECT_FALSE(set.AddStressImpulse(1, set.GetChunks()[1].Centroid, Vector3::UnitX()));
		set.Update(1.0f / 60.0f);
		EXPECT(set.GetStressSolver()->GetBondCount() == 0);
	}

	void TestStressLoadLifetime()
	{
		DestructionSet set;
		BuildStressChain(set, 2, 0.01f);
		const StrainSolverSettings settings = IntegrationStressSettings();
		set.EnableStressSolver(settings);
		EXPECT(set.AddStressImpulse(1, set.GetChunks()[1].Centroid, Vector3(100.0f, 0.0f, 0.0f)));
		set.DisableStressSolver();
		EXPECT(set.GetStressSolver() == nullptr);
		set.EnableStressSolver(settings);
		set.Update(1.0f / 60.0f);
		EXPECT(set.GetGraph().HasBond(0, 1));
		EXPECT_NEAR(set.GetStressSolver()->GetBondStress(101), 0.0f, 1e-6f);

		EXPECT(set.AddStressImpulse(1, set.GetChunks()[1].Centroid, Vector3(100.0f, 0.0f, 0.0f)));
		set.RemoveAllBonds();
		EXPECT(set.AddBond(0, 1, DestructionBondType::Connect, 0.01f, 111));
		set.RebuildClustersFromGraph();
		set.Update(1.0f / 60.0f);
		EXPECT(set.GetGraph().HasBond(0, 1));
		EXPECT(set.GetStressSolver()->GetBondCount() == 1);
		EXPECT_NEAR(set.GetStressSolver()->GetBondStress(111), 0.0f, 1e-6f);

		EXPECT(set.AddStressImpulse(1, set.GetChunks()[1].Centroid, Vector3(100.0f, 0.0f, 0.0f)));
		set.Clear();
		EXPECT(set.GetStressSolver() == nullptr);
		EXPECT(set.GetChunks().empty());
		BuildStressChain(set, 2, 0.01f);
		set.EnableStressSolver(settings);
		set.Update(1.0f / 60.0f);
		EXPECT(set.GetGraph().HasBond(0, 1));
		EXPECT_NEAR(set.GetStressSolver()->GetBondStress(101), 0.0f, 1e-6f);
	}

	void TestStressRotatedClusterLoads()
	{
		PhysicsWorld world{ PhysicsWorldParam() };
		DestructionSet reference;
		DestructionSet rotated(world);
		BuildStressChain(reference, 2, 1.0f);
		BuildStressChain(rotated, 2, 1.0f);
		DestructionCluster* cluster = rotated.FindClusterContaining(1);
		RigidBodyDynamic* body = cluster ? cluster->GetRigidBody() : nullptr;
		EXPECT(body != nullptr);
		if (!body) return;
		body->X = Vector3(8.0f, -3.0f, 5.0f);
		body->Q.FromRotationAxis(Vector3(1.0f, 2.0f, -1.0f).SafeUnit(), 1.1f);
		const Vector3 offset(0.2f, 0.3f, -0.15f);
		const Vector3 impulse(2.0f, -3.0f, 1.0f);
		const Vector3 gravity(0.0f, -9.8f, 0.0f);
		const StrainSolverSettings settings = IntegrationStressSettings(1.0e8f);
		reference.EnableStressSolver(settings, gravity);
		rotated.EnableStressSolver(settings, body->Q * gravity);
		Vector3 worldCenter;
		EXPECT(cluster->GetWorldPosition(1, worldCenter));
		const Vector3 worldPoint = worldCenter + body->Q * offset;
		const Vector3 worldImpulse = body->Q * impulse;
		// Rebuilding may change the cluster frame while preserving each chunk's world pose.
		rotated.RebuildClustersFromGraph();
		rotated.RebuildClustersFromGraph();
		EXPECT(reference.AddStressImpulse(1, reference.GetChunks()[1].Centroid + offset, impulse));
		EXPECT(rotated.AddStressImpulse(1, worldPoint, worldImpulse));
		reference.Update(1.0f / 60.0f);
		rotated.Update(1.0f / 60.0f);
		const StrainBondInfo* expected = reference.GetStressSolver()->GetBondInfo(101);
		const StrainBondInfo* actual = rotated.GetStressSolver()->GetBondInfo(101);
		EXPECT(expected != nullptr && actual != nullptr);
		if (expected && actual)
		{
			EXPECT((expected->ForceLinear - actual->ForceLinear).Length() < 0.01f);
			EXPECT((expected->Torque - actual->Torque).Length() < 0.01f);
			EXPECT(expected->ForceLinear.Length() > 1.0f);
			EXPECT(expected->Torque.Length() > 1.0f);
		}
	}

	void TestStressContactDispatch()
	{
		PhysicsWorld world{ PhysicsWorldParam() };
		DestructionSet set(world);
		BuildStressChain(set, 2, 0.01f);
		set.EnableStressSolver(IntegrationStressSettings());
		DestructionCluster* cluster = set.FindClusterContaining(1);
		EXPECT(cluster != nullptr);
		if (!cluster) return;
		PhysicsContactPoint point = {};
		point.Position = set.GetChunks()[1].Centroid;
		point.Normal = Vector3::UnitX();
		point.Impulse = Vector3(100.0f, 0.0f, 0.0f);
		PhysicsContactInfo contact = {};
		contact.BodyB = cluster->GetRigidBody();
		contact.GeomB = cluster->GetGeometryByChunkIndex(1);
		contact.ContactPoints = &point;
		contact.NumContactPoints = 1;
		EXPECT(contact.BodyB != nullptr && contact.GeomB != nullptr);
		set.OnContact(contact);
		set.Update(1.0f / 60.0f);
		EXPECT_FALSE(set.GetGraph().HasBond(0, 1));
		EXPECT(set.GetStressSolver()->GetBondCount() == 0);
	}
}

void TestStressIntegration()
{
	printf("Running TestStressIntegration\n");
	TestStressAutomaticFracture();
	TestStressTopologySynchronization();
	TestStressLoadLifetime();
	TestStressRotatedClusterLoads();
	TestStressContactDispatch();
}
