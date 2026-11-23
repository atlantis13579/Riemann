#include "Test.h"
#include "../Src/Deformation/DeformableBody.h"
#include "../Src/RigidBodyDynamics/PhysicsWorld.h"
#include "../Src/CollisionPrimitive/TriangleMesh.h"
#include "../Viewer/SceneWorld.h"
#include <algorithm>
#include <stdexcept>

using namespace Riemann;
namespace
{
	using DVector = DeformationGradient::DVector;
	DVector Double(const Vector3& P) { return DVector(P.x,P.y,P.z); }
	template<class F> void Reject(F Function)
	{ bool Rejected = false; try { Function(); } catch (const std::exception&) { Rejected = true; } EXPECT(Rejected); }
	TetrahedralAsset Asset(float Scale = 1)
	{
		TetrahedralAssetInput Input; Input.Positions = { Vector3(0), Vector3(Scale,0,0), Vector3(0,Scale,0), Vector3(0,0,Scale) };
		Input.Elements.push_back({ {0,1,2,3}, 0 }); Input.Materials[0].Density = 6; Input.Materials[0].Model = TetrahedralMaterialModel::Corotated;
		return TetrahedralAsset::Prepare(Input);
	}
	void RotationsAndMaterial()
	{
		for (double X : {0.0,1e-10,1.0,100.0,-0.2})
		{
			DeformationGradient F; F[0][0] = X; F[1][1] = 1; F[2][2] = 0.8; F[0][1] = 0.3;
			const auto R = TetrahedralElasticity::ClosestRotation(F); EXPECT_NEAR(R.Determinant(),1,1e-12);
			for (int I = 0; I < 3; ++I) for (int J = 0; J < 3; ++J)
			{ double Dot = 0; for (int K = 0; K < 3; ++K) Dot += R[K][I]*R[K][J]; EXPECT_NEAR(Dot,I==J?1:0,1e-12); }
		}
		TetrahedralMaterial Material; Material.Model = TetrahedralMaterialModel::Corotated;
		DeformationGradient F; F[0][0] = 1.1; F[1][1] = 0.93; F[2][2] = 1.03; F[0][1] = 0.2;
		const auto Response = TetrahedralElasticity::Evaluate(F,Material); constexpr double H = 1e-6;
		for (int I = 0; I < 3; ++I) for (int J = 0; J < 3; ++J)
		{
			auto Plus=F, Minus=F; Plus[I][J]+=H; Minus[I][J]-=H;
			const double Derivative = (TetrahedralElasticity::Evaluate(Plus,Material).EnergyDensity-TetrahedralElasticity::Evaluate(Minus,Material).EnergyDensity)/(2*H);
			EXPECT_NEAR(Derivative,Response.FirstPiola[I][J],1e-5*std::max(1.0,std::abs(Derivative)));
		}
		for (double Nu : {0.0,0.25,0.49})
		{
			Material.PoissonRatio=Nu; DeformationGradient Plus,Minus;
			for (int I=0;I<3;++I) Plus[I][I]=Minus[I][I]=1;
			Plus[0][0]+=H; Minus[0][0]-=H;
			const auto P=TetrahedralElasticity::Evaluate(Plus,Material), M=TetrahedralElasticity::Evaluate(Minus,Material);
			EXPECT_NEAR((P.FirstPiola[0][0]-M.FirstPiola[0][0])/(2*H),Material.LameLambda()+2*Material.ShearModulus(),1e-5*Material.YoungModulus);
			EXPECT_NEAR((P.FirstPiola[1][1]-M.FirstPiola[1][1])/(2*H),Material.LameLambda(),1e-5*Material.YoungModulus);
		}
	}
	void GlobalEquationAndScale()
	{
		for (float Scale : {0.001f,1.0f,1000.0f})
		{
			const auto Prepared=Asset(Scale); auto Positions=Prepared.GetPositions(); Positions[1].x *= 1.1f; const auto Predicted=Positions;
			ProjectiveDynamicsElement Element; Element.Particles={0,1,2,3}; Element.Rest=Prepared.GetRestShapes()[0]; Element.ShearModulus=500;
			std::vector<ProjectiveDynamicsParticle> Particles;
			for (int I=0;I<4;++I) Particles.push_back({&Positions[I],Prepared.GetInverseMasses()[I]});
			const double Dt=0.01*Scale, Weight=2*Element.ShearModulus*Element.Rest.Volume;
			const auto Rotation=TetrahedralElasticity::ClosestRotation(Element.Rest.Evaluate(Positions.data()));
			ProjectiveDynamicsProjection Solver; Solver.Begin(Particles,{Element},Dt); EXPECT(Solver.Project()); EXPECT(Solver.GetStats().Converged);
			EXPECT_NEAR(Solver.GetStats().AcceptedStep,1,1e-12);
			auto Difference=Element.Rest.Evaluate(Positions.data());
			for (int I=0;I<3;++I) for (int J=0;J<3;++J) Difference[I][J]-=Rotation[I][J];
			DVector Momentum(0); double Residual=0,Reference=0;
			for (int I=0;I<4;++I)
			{
				const auto Change=Double(Positions[I])-Double(Predicted[I]); const double Mass=1.0/Particles[I].InverseMass;
				const auto Force=(Difference*Element.Rest.Gradients[I])*Weight;
				Residual+=(Change*(Mass/(Dt*Dt))+Force).SquareLength(); Reference+=Force.SquareLength(); Momentum+=Change*Mass;
			}
			EXPECT(std::sqrt(Residual/std::max(Reference,1e-100))<1e-4);
			EXPECT(std::hypot(std::hypot(Momentum.x,Momentum.y),Momentum.z)/(Scale*Prepared.GetStats().TotalMass)<1e-7);
			Solver.End(); Reject([&]{Solver.Project();});
			Particles[0].InverseMass=0; const auto Pinned=Positions[0]; Solver.Begin(Particles,{Element},Dt);
			for(int I=0;I<8;++I) Solver.Project(); EXPECT(Positions[0]==Pinned);
			Particles[1].Position=Particles[0].Position; Reject([&]{Solver.Begin(Particles,{Element},Dt);});
		}
	}
	void CalibrationAndSwitches()
	{
		for (bool Projective : {false,true}) for (double Young : {1000.0,10000.0}) for (int Substeps : {1,2})
		{
			DeformableBodySettings Settings; Settings.ProjectiveMaterials=Projective; Settings.Substeps=Substeps;
			Settings.SolverIterations=64; Settings.Gravity=Vector3::Zero(); Settings.Damping=12;
			DeformableBody Body(Settings); Body.InitializeAsset(Asset()); Body.PinParticle(0,Vector3(0));
			auto Material=Body.GetMaterials()[0]; Material.YoungModulus=Young; Material.PoissonRatio=0.25; Body.SetMaterial(0,Material);
			for(int Frame=0;Frame<300;++Frame) { Body.AddForce(2,Vector3(0,1.0f/6,0)); Body.Simulate(0.01f); }
			const double Axial=Body.GetParticle(2).Position.y-1, Lateral=(Body.GetParticle(1).Position.x+Body.GetParticle(3).Position.z)/2-1;
			printf("Corotated calibration PD=%d E=%g substeps=%d axial=%g lateral=%g\n",Projective,Young,Substeps,Axial,Lateral);
			EXPECT_NEAR(Axial,1/Young,0.02/Young); EXPECT_NEAR(Lateral,-Material.PoissonRatio/Young,0.02*Material.PoissonRatio/Young);
			if(Projective) EXPECT(Body.GetProjectiveMaterialStats().Converged);
			auto* Render=Body.GetRenderGeometry(); const auto Position=Body.GetParticle(2).Position;
			Body.SetProjectiveMaterialSolver(!Projective); EXPECT(Body.GetParticle(2).Position==Position && Body.GetRenderGeometry()==Render);
			Material.Model=TetrahedralMaterialModel::StableNeoHookean; Body.SetMaterial(0,Material); Body.Simulate(0.01f);
			Body.SetTetrahedronMaterial(0,-1); Body.Simulate(0.01f); Body.SetTetrahedronMaterial(0,0); Body.Reset();
			EXPECT_NEAR(Body.GetMaterialEnergy(),0,1e-10);
		}
	}
	void CoupledElementsAndLoads()
	{
		for (bool Mixed : {false,true})
		{
			TetrahedralAssetInput Input; Input.Positions={Vector3(0),Vector3::UnitX(),Vector3::UnitY(),Vector3::UnitZ(),-Vector3::UnitZ()};
			Input.Materials[0].Model=TetrahedralMaterialModel::Corotated; Input.Materials[0].Density=6; Input.Materials[0].YoungModulus=2000;
			Input.Materials.push_back(Input.Materials[0]); Input.Materials[1].YoungModulus=5000;
			if(Mixed) Input.Materials[1].Model=TetrahedralMaterialModel::SaintVenantKirchhoff;
			Input.Elements={{{0,1,2,3},0},{{0,1,2,4},1}};
			const auto Prepared=TetrahedralAsset::Prepare(Input);
			DeformableBodySettings Settings; Settings.ProjectiveMaterials=true; Settings.Substeps=2; Settings.SolverIterations=64; Settings.Damping=12;
			DeformableBody Body(Settings); Body.InitializeAsset(Prepared);
			const auto Point=BarycentricPoint::Triangle(Index3(0,1,2),Vector3(0.5f,0.25f,0.25f)); const auto Anchor=Body.GetPointPosition(Point);
			const auto Link=Body.AttachToWorld(Point,Anchor); double Reaction=0;
			constexpr int Frames=300, Samples=60;
			for(int Frame=0;Frame<Frames;++Frame)
			{
				Body.Simulate(1.0f/120); EXPECT_NEAR((Body.GetPointPosition(Point)-Anchor).Length(),0,1e-6);
				for(int I=0;I<2;++I) EXPECT(Body.GetTetrahedronMaterialResponse(I).Jacobian>0);
				if(Frame>=Frames-Samples) Reaction+=Body.GetAttachmentForce(Link).y/Samples;
			}
			const double Expected=-Settings.Gravity.y*Prepared.GetStats().TotalMass;
			printf("Projective shared-node load mixed=%d reaction=%g expected=%g\n",Mixed,Reaction,Expected);
			EXPECT_NEAR(Reaction,Expected,Expected*0.01); EXPECT(Body.GetProjectiveMaterialStats().Converged);
			// The rotation block must remain finite from a singular initial state.
			Body.ClearAttachments(); Body.Reset();
			for(int I=0;I<Body.GetParticleCount();++I) Body.SetParticlePosition(I,Vector3(0));
			for(int Frame=0;Frame<30;++Frame) Body.Simulate(1.0f/120);
			for(const auto& P:Body.GetParticles()) EXPECT(std::isfinite(P.Position.SquareLength()));
		}
	}
	void FailedPreparation()
	{
		DeformableBodySettings Settings; Settings.ProjectiveMaterials=true; Settings.Gravity=Vector3::Zero();
		DeformableBody A(Settings),B(Settings); A.InitializeAsset(Asset()); B.InitializeAsset(Asset(1000));
		const auto Normal=B.GetMaterials()[0]; auto Huge=Normal; Huge.YoungModulus=1e307; EXPECT(Huge.IsValid()); B.SetMaterial(0,Huge);
		A.AttachToBody(BarycentricPoint::Vertex(0),&B,BarycentricPoint::Vertex(0));
		Reject([&]{DeformableBody::SimulateGroup({&A,&B},0.01f);});
		bool Recovered=true;
		try { A.SetProjectiveMaterialSolver(false); B.SetMaterial(0,Normal); DeformableBody::SimulateGroup({&A,&B},0.01f); }
		catch(const std::exception&) { Recovered=false; }
		EXPECT(Recovered);
	}
	void ExistingDemo()
	{
		SceneWorld Scene; Scene.SetClothModelDemo(true); std::string Error;
		EXPECT(Scene.LoadFromFile(TestContentsPath()+"Scenes/08_cloth_sphere.json",&Error));
		for(bool Rigid : {false,true}) for(bool Projective : {false,true})
		{
			EXPECT(Scene.ApplyDeformableContactDemo(false)); auto* World=Scene.GetSimulation();
			EXPECT(Scene.SetClothRigidInteraction(false));
			EXPECT(World->SetSolverConfiguration(Rigid?ResolutionPhaseSolver::XPBD:ResolutionPhaseSolver::SequentialImpulse,SolverBackend::CPU,false));
			EXPECT(Scene.SetClothRigidInteraction(Rigid));
			EXPECT(Scene.ApplyDeformableContactDemo(true,Rigid,true)); EXPECT(Scene.ApplyDeformableMaterialDemo(true,false,true,Projective));
			EXPECT(Scene.GetCorotatedSoftMaterialDemo() && Scene.GetProjectiveSoftMaterialDemo()==Projective);
			size_t Contacts=0;
			for(int Frame=0;Frame<120;++Frame)
			{
				Scene.Step(1.0f/60);
				for(size_t I=0;I<2;++I)
				{
					auto* Body=Scene.GetDemoContactBody(I); EXPECT(Body->GetTetrahedronMaterialResponse(0).Jacobian>0);
					Contacts+=Body->GetCollisionStats().InterContacts+Body->GetCollisionStats().RigidContacts;
					for(const auto& P:Body->GetParticles()) EXPECT(std::isfinite(P.Position.SquareLength()) && P.Position.Length()<100);
				}
			}
			printf("Corotated existing demo PD=%d rigid=%d contacts=%zu\n",Projective,Rigid,Contacts); EXPECT(Contacts>0);
			if(Rigid) EXPECT(Scene.GetDemoContactRigid()->X.y>1.7f);
			EXPECT(Scene.ApplyDeformableMaterialDemo(false)); EXPECT(!Scene.GetProjectiveSoftMaterialDemo()); Scene.Step(1.0f/60);
		}
	}
}
void TestProjectiveDynamics()
{
	RotationsAndMaterial(); GlobalEquationAndScale(); CalibrationAndSwitches(); CoupledElementsAndLoads(); FailedPreparation(); ExistingDemo();
}
