#include "../../../Viewer/SceneWorld.h"
#include "ArticulatedRobotController.h"
#include "RoboticsRobot.h"
#include "../../LinearSystem/NeuralNetwork.h"
#include "../../Collision/GeometryQuery.h"
#include "../../CollisionPrimitive/HeightField3.h"
#include "../../RigidBodyDynamics/PhysicsWorld.h"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <stdexcept>

namespace Riemann
{
	namespace
	{
		using namespace Maths::LinearAlgebra;
		constexpr size_t RoboticsTicksPerAction = 50;
		constexpr double Pi = 3.14159265358979323846;

		void ApplyRoboticsPolicy(NeuralNetwork& Actor, const FeatherstoneSolver& Tree,
			const Vector3& Command, RoboticsRobotControl& Control, bool Directional, int& Shift, Vector3& PreviousCommand)
		{
			std::vector<float> Observation = ObserveRoboticsRobot(Tree, Command, Control);
			if (!Directional) { Shift = 0; PreviousCommand = Vector3::Zero(); }
			if (Directional)
			{
				// Rotate the policy's coordinate frame and relabel the eight identical
				// legs together. All motor targets still come from the trained actor.
				const float Sector = 2 * float(Pi) / RoboticsLegCount;
				// Hold the leg identity while a command is held; body yaw must not
				// cause discontinuous relabeling in the middle of a learned stride.
				if (PreviousCommand.SquareLength() < 1e-6f ||
					Command.Dot(PreviousCommand) < std::cos(Sector * 0.5f) * Command.Length() * PreviousCommand.Length())
				{
					Shift = (int(std::round(std::atan2(Observation[10], Observation[9]) / Sector)) + int(RoboticsLegCount)) % int(RoboticsLegCount);
					PreviousCommand = Command;
				}
				const float C = std::cos(Shift * Sector), S = std::sin(Shift * Sector);
				const auto Original = Observation;
				for (int I = 0; I < 12; I += 3)
				{
					Observation[I] = C * Original[I] + S * Original[I + 1];
					Observation[I + 1] = -S * Original[I] + C * Original[I + 1];
				}
				for (int Block = 0; Block < 3; ++Block)
					for (int Leg = 0; Leg < int(RoboticsLegCount); ++Leg)
						for (int Joint = 0; Joint < 3; ++Joint)
							Observation[12 + Block * RoboticsMotorCount + Leg * 3 + Joint] =
								Original[12 + Block * RoboticsMotorCount + ((Leg + Shift) % RoboticsLegCount) * 3 + Joint];
			}
			const std::vector<float>& Output = Actor.Evaluate(Observation);
			std::vector<float> Action(RoboticsMotorCount);
			for (int Leg = 0; Leg < int(RoboticsLegCount); ++Leg)
				for (int Joint = 0; Joint < 3; ++Joint)
					Action[((Leg + Shift) % RoboticsLegCount) * 3 + Joint] = Output[Leg * 3 + Joint];
			ApplyRoboticsAction(Control, Action);
		}

		template<class T>
		void Read(std::istream& File, T* Data, size_t Count = 1)
		{
			if (!File.read(reinterpret_cast<char*>(Data), sizeof(T) * Count))
				throw std::runtime_error("Incomplete walking demo asset");
		}
		void CheckMagic(std::istream& File, const std::string& Expected)
		{
			std::string Header(Expected.size(), ' ');
			Read(File, &Header[0], Header.size());
			if (Header != Expected)
				throw std::runtime_error("Invalid walking demo asset header");
		}
		std::vector<float> LoadWeights(const std::string& Path)
		{
			std::ifstream File(Path, std::ios::binary | std::ios::ate);
			const std::streamoff Bytes = File.tellg();
			if (Bytes <= 0 || Bytes > 1024 * 1024 || Bytes % sizeof(float))
				throw std::runtime_error("Invalid walking policy file");
			std::vector<float> Weights((size_t)Bytes / sizeof(float));
			File.seekg(0);
			Read(File, Weights.data(), Weights.size());
			return Weights;
		}

	}


	void SceneWorld::LoadRobotCourse(const std::string& Path)
	{
		std::ifstream File(Path, std::ios::binary);
		CheckMagic(File, "RHF1");
		uint32_t Width, Height;
		float HalfX, HalfZ, Top;
		Read(File, &Width); Read(File, &Height); Read(File, &HalfX); Read(File, &HalfZ); Read(File, &Top);
		if (Width < 2 || Height < 2 || uint64_t(Width) * Height > 4000000 || !(HalfX > 0 && HalfZ > 3.2f && Top > 0))
			throw std::runtime_error("Invalid walking course dimensions");
		std::vector<uint8_t> Pixels(size_t(Width) * Height);
		Read(File, Pixels.data(), Pixels.size());
		const float Dx = 2 * HalfX / (Width - 1), Dz = 2 * HalfZ / (Height - 1);
		// Keep the six-metre walking corridor level and blend into the original side terrain.
		for (uint32_t Z = 0; Z < Height; ++Z)
		{
			const float Side = std::abs(-HalfZ + Z * Dz);
			const float T = std::max(0.0f, std::min(1.0f, (Side - 3.2f) / 1.6f));
			const float Blend = T * T * (3 - 2 * T);
			for (uint32_t X = 0; X < Width; ++X)
				Pixels[Z * Width + X] = uint8_t(std::round(Pixels[Z * Width + X] * Blend));
		}
		// Adjacent renderer tiles share the last row and column of source samples.
		const uint32_t Tile = 128;
		const uint32_t SouthEnd = uint32_t(std::floor((HalfZ - 3.2f) / Dz));
		const uint32_t NorthStart = uint32_t(std::ceil((HalfZ + 3.2f) / Dz));
		for (uint32_t Side = 0; Side < 2; ++Side)
		{
			const uint32_t First = Side ? NorthStart : 0;
			const uint32_t Last = Side ? Height - 1 : SouthEnd;
			for (uint32_t Z = First; Z < Last; Z += Tile)
				for (uint32_t X = 0; X < Width - 1; X += Tile)
				{
					const uint32_t W = std::min(Tile + 1, Width - X), H = std::min(Tile + 1, Last - Z + 1);
					const Vector3 Min(14 - HalfX + X * Dx, 0, -HalfZ + Z * Dz);
					std::unique_ptr<Geometry> G(GeometryFactory::CreateHeightField(Box3(Min, Min + Vector3((W - 1) * Dx, 0, (H - 1) * Dz)), W, H));
					HeightField3* Field = G->GetShapeObj<HeightField3>();
					Field->HeightScale = Top / 255;
					Field->AllocMemory();
					for (uint32_t J = 0; J < H; ++J)
						for (uint32_t I = 0; I < W; ++I)
							Field->Cells[J * W + I].HeightSample = Pixels[(Z + J) * Width + X + I];
					Field->RecalculateBounds();
					G->SetShapeBounds(Field->BV);
					SceneObjectInstance Object;
					Object.GeometryPtr = G.get();
					Object.Color = Vector4(0.32f, 0.40f, 0.36f, 1);
					Object.Id = "course_" + std::to_string(X) + "_" + std::to_string(Z);
					if (!m_World->CreateRigidBody(G.get(), RigidBodyParam()))
						throw std::runtime_error("Could not create course tile");
					G.release();
					m_Objects.push_back(Object);
				}
		}
	}

	void SceneWorld::CreateRobotLandmarks(bool Course)
	{
		const auto Add = [this](const std::string& Id, Geometry* Shape, const Vector4& Color, bool Collision)
		{
			std::unique_ptr<Geometry> Owned(Shape);
			Shape->SetSimulationEnabled(Collision);
			Shape->SetQueryEnabled(Collision);
			if (!m_World->CreateRigidBody(Shape, RigidBodyParam()))
				throw std::runtime_error("Could not create robot landmark: " + Id);
			SceneObjectInstance Object;
			Object.Id = Id;
			Object.GeometryPtr = Owned.release();
			Object.Color = Color;
			m_Objects.push_back(std::move(Object));
		};
		std::vector<const Geometry*> Terrain;
		for (const SceneObjectInstance& Object : m_Objects)
			if (Object.Id.find("course_") == 0) Terrain.push_back(Object.GeometryPtr);
		const auto GroundHeight = [&Terrain](float X, float Z)
		{
			RayCastOption Option; Option.MaxDist = 12;
			RayCastResult Hit;
			float Distance = Option.MaxDist;
			for (const Geometry* Tile : Terrain)
				if (Tile->RayCast(Tile->GetParent<RigidBody>()->GetGeometryTransform(Tile),
					Vector3(X, 10, Z), -Vector3::UnitY(), &Option, &Hit))
					Distance = std::min(Distance, Hit.hitTime);
			return Distance < Option.MaxDist ? 10 - Distance : 0.0f;
		};

		// World-fixed metre markings provide optic flow under the following camera.
		// They are visual only, so feet still contact the level corridor.
		{
			const int MarkingEnd = Course ? 40 : 80;
			for (int X = -8; X <= MarkingEnd; ++X)
			{
				const bool Major = X % 5 == 0;
				Add("robot_marking_cross_" + std::to_string(X), GeometryFactory::CreateOBB(
					Vector3(float(X), 0.003f, 0), Vector3(Major ? 0.045f : 0.012f, 0.002f, 3)),
					Major ? Vector4(0.86f, 0.69f, 0.32f, 1) : Vector4(0.53f, 0.58f, 0.60f, 1), false);
			}
			for (int Z = -3; Z <= 3; ++Z)
				Add("robot_marking_long_" + std::to_string(Z), GeometryFactory::CreateOBB(
					Vector3((MarkingEnd - 8) * 0.5f, 0.002f, float(Z)),
					Vector3((MarkingEnd + 8) * 0.5f, 0.001f, 0.012f)),
					Vector4(0.48f, 0.53f, 0.55f, 1), false);
		}

		// Deterministic staggered obstacles leave a six-metre-wide walking corridor.
		// The course ends at x = 42; the flat route has references for a longer run.
		const int End = Course ? 39 : 78;
		for (int X = -6, Index = 0; X <= End; X += 3, ++Index)
			for (int Side = -1; Side <= 1; Side += 2)
			{
				const int Variant = Index * 2 + (Side > 0 ? 1 : 0);
				const float Px = float(X) + (Side > 0 ? 1.25f : 0);
				const float Pz = Side * (3.6f + 0.18f * (Variant % 4));
				const float Ground = GroundHeight(Px, Pz);
				const std::string Id = "robot_obstacle_" + std::to_string(Variant);
				if (Variant % 3 == 0)
				{
					const float HalfHeight = 0.45f + 0.1f * (Variant % 4);
					Add(Id, GeometryFactory::CreateOBB(Vector3(Px, Ground + HalfHeight, Pz),
						Vector3(0.18f, HalfHeight, 0.18f)), Vector4(0.86f, 0.55f, 0.22f, 1), true);
				}
				else if (Variant % 3 == 1)
				{
					Add(Id, GeometryFactory::CreateOBB(Vector3(Px, Ground + 0.25f, Pz),
						Vector3(0.65f, 0.25f, 0.35f)), Vector4(0.34f, 0.48f, 0.57f, 1), true);
				}
				else
				{
					const float Radius = 0.28f + 0.05f * (Variant % 4);
					Add(Id, GeometryFactory::CreateSphere(Vector3(Px, Ground + Radius, Pz), Radius),
						Vector4(0.55f, 0.48f, 0.39f, 1), true);
				}
			}

		// Side practice areas remain outside the clear central route.
		const int Columns = 41, Rows = 17;
		std::unique_ptr<Geometry> Hills(GeometryFactory::CreateHeightField(
			Box3(Vector3(3, 0, -7.6f), Vector3(13, 0, -4.8f)), Columns, Rows));
		HeightField3* Field = Hills->GetShapeObj<HeightField3>();
		Field->HeightScale = 0.001f;
		Field->AllocMemory();
		for (int Z = 0; Z < Rows; ++Z)
			for (int X = 0; X < Columns; ++X)
			{
				const float U = float(X) / (Columns - 1), V = float(Z) / (Rows - 1);
				const float Sx = std::sin(float(Pi) * U), Sz = std::sin(float(Pi) * V);
				const float Height = GroundHeight(3 + 10 * U, -7.6f + 2.8f * V) + 0.015f +
					0.65f * Sx * Sx * Sz * Sz *
					(0.65f + 0.35f * std::cos(6 * float(Pi) * U));
				Field->Cells[Z * Columns + X].HeightSample = int16_t(std::max(0.0f, Height) / Field->HeightScale);
			}
		Field->RecalculateBounds();
		Hills->SetShapeBounds(Field->BV);
		Add("robot_terrain_hills", Hills.release(), Vector4(0.38f, 0.53f, 0.32f, 1), true);
		for (int I = 0; I < 16; ++I)
		{
			const float X = 3 + (I + 0.5f) * 0.65f;
			const float Height = 0.15f * (I < 8 ? I + 1 : 16 - I);
			const float Base = GroundHeight(X, 6);
			Add("robot_stair_" + std::to_string(I), GeometryFactory::CreateOBB(
				Vector3(X, Base + Height * 0.5f, 6), Vector3(0.325f, Height * 0.5f, 1.2f)),
				I % 2 ? Vector4(0.64f, 0.57f, 0.43f, 1) : Vector4(0.78f, 0.71f, 0.55f, 1), true);
		}
		for (int I = 0; I < 4; ++I)
		{
			const float X = 18 + I * 3.0f, Z = I % 2 ? 6.0f : -6.0f;
			Add("robot_obstacle_wall_" + std::to_string(I), GeometryFactory::CreateOBB(
				Vector3(X, GroundHeight(X, Z) + 0.7f, Z), Vector3(0.25f, 0.7f, 1.2f)),
				Vector4(0.62f, 0.36f, 0.29f, 1), true);
		}
	}

	void SceneWorld::AddRobotPrimitive(size_t Link, std::unique_ptr<Geometry> Shape, const Vector4& Color)
	{
		Shape->SetSimulationEnabled(false);
		Shape->SetQueryEnabled(false);
		SceneObjectInstance Object;
		Object.Id = "robot_part_" + std::to_string(m_RobotSurfaces.size());
		Object.GeometryPtr = Shape.get();
		Object.Color = Color;
		Object.RenderEnabled = m_TrainingBuildIndex == 0;
		const Transform Local = *Shape->GetTransform();
		m_RobotSurfaces.push_back({Link, Local, std::move(Shape), m_TrainingBuildIndex, m_Objects.size()});
		m_Objects.push_back(std::move(Object));
	}

	void SceneWorld::CreateRobotPrimitives(const ArticulatedRobotSettings& Settings)
	{
		const Vector4 BodyColor(0.90f, 0.43f, 0.13f, 1);
		const Vector4 LimbColor(0.31f, 0.39f, 0.46f, 1);
		const Vector3 BodyCenter(0, 0, 0.04f);
		const Vector3 BodyExtent = Settings.Feet.size() == RoboticsLegCount
			? Vector3(0.27f, 0.22f, 0.08f) : Vector3(0.32f, 0.14f, 0.10f);
		AddRobotPrimitive(0, std::unique_ptr<Geometry>(GeometryFactory::CreateOBB(BodyCenter, BodyExtent)), BodyColor);

		for (size_t I = 1; I < Settings.Links.size(); ++I)
		{
			const RagdollLinkSettings& Child = Settings.Links[I];
			const Vector3 Offset = Child.ParentToJoint.pos;
			if (Child.Parent == 0 || Offset.SquareLength() < 0.0025f)
				continue;
			AddRobotPrimitive(Child.Parent, std::unique_ptr<Geometry>(GeometryFactory::CreateCapsule(
				Vector3::Zero(), Offset, 0.039f)), LimbColor);
		}

		for (const RobotFootContact& Foot : Settings.Feet)
			AddRobotPrimitive(Foot.Link, std::unique_ptr<Geometry>(GeometryFactory::CreateCapsule(
				Vector3::Zero(), Foot.Center, Foot.Radius)), LimbColor);
	}

	void SceneWorld::BeginRoboticsTrainingDisplay(size_t EnvironmentCount)
	{
		if (m_TrainingDisplay || !IsRoboticsWalkingDemo() || !GetWalkingRobot()) return;
		if (EnvironmentCount < 1 || EnvironmentCount > RoboticsTrainingEnvironmentCount) return;
		m_TrainingDisplay = true;
		auto Control = std::make_shared<RoboticsRobotControl>();
		const ArticulatedRobotSettings Settings = CreateRoboticsRobotSettings(Control);
		for (size_t E = 1; E < EnvironmentCount; ++E)
		{
			m_TrainingBuildIndex = E;
			CreateRobotPrimitives(Settings);
		}
		m_TrainingBuildIndex = 0;
		TouchRenderRevision();
	}

	void SceneWorld::ShowRoboticsTrainingSnapshot(const RoboticsTrainingSnapshot& Snapshot, bool ShowAll, size_t Selected)
	{
		if (!m_TrainingDisplay) return;
		bool VisibilityChanged = false;
		for (const RobotSurface& Part : m_RobotSurfaces)
		{
			const size_t E = Part.Environment;
			const bool Visible = ShowAll || E == Selected;
			SceneObjectInstance& Object = m_Objects[Part.ObjectIndex];
			if (Object.RenderEnabled != Visible)
			{
				Object.RenderEnabled = Visible;
				VisibilityChanged = true;
			}
			if (!Snapshot.ready[E] || !Visible)
			{
				Part.Shape->SetTransform(Vector3(100000, 100000, 100000), Quaternion::One());
				continue;
			}
			const Transform T = Snapshot.links[E][Part.Link] * Part.Local;
			const Vector3 Offset = ShowAll ? Vector3(4.5f * float(E % 4), 0, 3.0f * float(E / 4)) : Vector3::Zero();
			Part.Shape->SetTransform(T.pos + Offset, T.quat);
		}
		if (VisibilityChanged) TouchRenderRevision();
	}

	bool SceneWorld::CreateWalkingRobot(bool Course, const std::string& Model, bool Landmarks, std::string* Error)
	{
		try
		{
			if (Model != "Spider" && Model != "SpiderPolicy")
				throw std::runtime_error("Unknown walking robot model: " + Model);
			const std::string Directory = m_BaseDirectory + "../Robots/";
			m_RoboticsWalkingDemo = Model == "SpiderPolicy";
			const bool Sweep = m_RoboticsWalkingDemo && m_RobotWalkMode == RobotWalkMode::Sweep;
			auto Control = std::make_shared<RoboticsRobotControl>();
			ArticulatedRobotSettings Settings = CreateRoboticsRobotSettings(Control);
			if (Model == "SpiderPolicy" && !Sweep)
			{
				std::ifstream WeightsFile(Directory + "Spider/actor.bin", std::ios::binary);
				if (!WeightsFile)
					throw std::runtime_error("Spider policy file is missing");
				auto Actor = std::make_shared<NeuralNetwork>(86,
					std::vector<NeuralLayer>{{NeuralLayerType::Linear, 128}, {NeuralLayerType::ELU},
					{NeuralLayerType::Linear, 128}, {NeuralLayerType::ELU},
					{NeuralLayerType::Linear, 128}, {NeuralLayerType::ELU},
					{NeuralLayerType::Linear, 25}}, LoadWeights(Directory + "Spider/actor.bin"));
				const auto StandActuate = Settings.Actuate;
				Settings.Actuate = [this, Actor, Control, StandActuate, Shift = 0, PreviousCommand = Vector3::Zero()](const FeatherstoneSolver& Tree,
					const Vector3& Command, size_t Tick, std::vector<double>& Torque) mutable
				{
					if (Tick % RoboticsTicksPerAction == 0)
					{
						if (Command.SquareLength() < 1e-6f)
						{
							Control->TargetOffset.fill(0); // PD standing when the operator releases movement.
							PreviousCommand = Vector3::Zero();
						}
						else
							ApplyRoboticsPolicy(*Actor, Tree, Command, *Control, !m_RobotAutoWalk, Shift, PreviousCommand);
					}
					StandActuate(Tree, Command, Tick, Torque);
				};
			}
			Settings.Backend = m_RobotBackend;
			if (m_Objects.empty())
			{
				std::unique_ptr<Geometry> Ground(GeometryFactory::CreatePlane(Vector3::Zero(), Vector3::UnitY()));
				if (!m_World->CreateRigidBody(Ground.get(), RigidBodyParam()))
					throw std::runtime_error("Could not create flat robot ground");
				SceneObjectInstance Surface;
				Surface.Id = "robot_flat_ground";
				Surface.GeometryPtr = Ground.release();
				Surface.Color = Vector4(0.36f, 0.40f, 0.43f, 1);
				m_Objects.push_back(std::move(Surface));
			}
			if (Course)
				LoadRobotCourse(Directory + "Spider/course.height");
			if (Landmarks)
				CreateRobotLandmarks(Course);
			if (Sweep)
			{
				RobotControllerSettings SweepSettings;
				SweepSettings.Radius = 0.32f;
				SweepSettings.HalfHeight = 0.17f;
				SweepSettings.StepHeight = 0.22f;
				SweepSettings.GroundSnapDistance = 0.25f;
				m_SweepWalkingRobot = m_World->CreateRobotController(SweepSettings, Vector3(0, 0.51f, 0));
				m_SweepRobotPrevious = m_SweepWalkingRobot->GetPosition();
				Settings.Links[0].ParentToJoint = Transform();
				m_SweepRobotPose = std::make_unique<FeatherstoneSolver>(Settings.Links);
				m_SweepRobotPose->SetConfiguration(Settings.Pose);
			}
			else m_WalkingRobot = m_World->CreateRobotController(Settings);
			if (!Sweep)
				m_WalkingRobot->AddLinkCollider(0, Transform(),
					std::unique_ptr<Geometry>(GeometryFactory::CreateOBB(Vector3::Zero(), Vector3(0.27f, 0.22f, 0.08f))));
			CreateRobotPrimitives(Settings);
			GetRobotDemoController()->SetMoveVelocity(Model == "Spider" ? Vector3::Zero() : Vector3(0.8f, 0, 0));
			SyncWalkingRobot();
			TouchRenderRevision();
			return true;
		}
		catch (const std::exception& E)
		{
			if (Error)
				*Error = Model + ": " + E.what();
			return false;
		}
	}

	ArticulatedRobotController* SceneWorld::GetWalkingRobot() const
	{
		if (m_WalkingRobot && m_World)
			for (size_t I = 0; I < m_World->GetRobotControllerCount(); ++I)
				if (m_World->GetRobotController(I) == m_WalkingRobot)
					return m_WalkingRobot;
		return nullptr;
	}

	RobotController* SceneWorld::GetRobotDemoController() const
	{
		return m_SweepWalkingRobot ? static_cast<RobotController*>(m_SweepWalkingRobot) : GetWalkingRobot();
	}

	void SceneWorld::SetRobotDriveInput(float Forward, float Turn, float Strafe)
	{
		if (!std::isfinite(Forward) || !std::isfinite(Turn) || !std::isfinite(Strafe))
			throw std::invalid_argument("Invalid robot drive input");
		m_RobotDriveInput = Vector3(std::max(-1.0f, std::min(1.0f, Forward)),
			std::max(-1.0f, std::min(1.0f, Turn)), std::max(-1.0f, std::min(1.0f, Strafe)));
	}

	void SceneWorld::UpdateRobotDrive(float Dt)
	{
		if (!m_RoboticsWalkingDemo) return;
		if (m_RobotAutoWalk)
		{
			m_RobotDriveHeading = 0;
			m_RobotDriveCommand = Vector3(0.8f, 0, 0);
		}
		else
		{
			m_RobotDriveHeading = std::remainder(m_RobotDriveHeading + m_RobotDriveInput.y * Dt, 2 * float(Pi));
			const Vector3 Forward(std::cos(m_RobotDriveHeading), 0, -std::sin(m_RobotDriveHeading));
			const Vector3 Right(-Forward.z, 0, Forward.x);
			m_RobotDriveCommand = Forward * m_RobotDriveInput.x + Right * m_RobotDriveInput.z;
			if (m_RobotDriveCommand.SquareLength() > 1) m_RobotDriveCommand.Normalize();
			m_RobotDriveCommand *= 0.8f;
		}
		GetRobotDemoController()->SetMoveVelocity(m_RobotDriveCommand);
	}

	void SceneWorld::SyncWalkingRobot(float Dt)
	{
		const ArticulatedRobotController* Robot = GetWalkingRobot();
		if (!Robot && !m_SweepWalkingRobot)
			return;
		if (m_SweepWalkingRobot)
		{
			const Vector3 Position = m_SweepWalkingRobot->GetPosition();
			Vector3 Travel = Position - m_SweepRobotPrevious; Travel.y = 0;
			m_SweepRobotPrevious = Position;
			const float Speed = Dt > 0 ? Travel.Length() / Dt : 0;
			m_SweepRobotPhase = std::fmod(m_SweepRobotPhase + Travel.Length() * 7.5f, 2 * float(Pi));
			auto Pose = m_SweepRobotPose->GetConfiguration();
			Quaternion Yaw, Up;
			Yaw.FromRotateY(m_RobotDriveHeading); Up.FromRotateX(-float(Pi) * 0.5f);
			Pose[0].Translation = Position;
			Pose[0].Rotation = Yaw * Up;
			const Vector3 LocalTravel = Pose[0].Rotation.Conjugate() * (Dt > 0 ? Travel / Dt : Vector3::Zero());
			// Sweep alone moves the body. This IK gait is only its visual animation;
			// the policy mode never executes this path or receives these joint angles.
			for (size_t Leg = 0; Leg < RoboticsLegCount; ++Leg)
			{
				const float Angle = float(Leg * 2 * Pi / RoboticsLegCount);
				const Vector3 Radial(std::cos(Angle), std::sin(Angle), 0);
				const float Phase = m_SweepRobotPhase + (Leg % 2 ? float(Pi) : 0);
				Vector3 Foot = Radial * (0.28f + 0.18f + 0.38f * std::cos(-0.6f) + 0.75f * std::cos(1.15f));
				Foot -= LocalTravel * (0.16f * std::cos(Phase));
				Vector3 WorldFoot = Position + Pose[0].Rotation * Foot;
				RayCastOption Option; Option.MaxDist = 3;
				RayCastResult Hit;
				float Ground = Position.y - 0.51f;
				if (m_World->GetGeometryQuery()->RayCastQuery(Vector3(WorldFoot.x, Position.y + 1, WorldFoot.z),
					-Vector3::UnitY(), Option, &Hit)) Ground = Position.y + 1 - Hit.hitTimeMin;
				Foot.z = Ground + 0.035f - Position.y + std::min(Speed, 0.8f) * 0.16f * std::max(0.0f, std::sin(Phase));
				Foot -= Radial * 0.28f;
				const float LocalX = Foot.x * Radial.x + Foot.y * Radial.y;
				const float LocalY = -Foot.x * Radial.y + Foot.y * Radial.x;
				const float Reach = std::sqrt(LocalX * LocalX + LocalY * LocalY) - 0.18f;
				const float Down = -Foot.z;
				const float CosKnee = std::max(-1.0f, std::min(1.0f, (Reach * Reach + Down * Down - 0.38f * 0.38f - 0.75f * 0.75f) / (2 * 0.38f * 0.75f)));
				const float Knee = std::acos(CosKnee);
				Pose[1 + Leg * 3].Position = std::max(-0.45f, std::min(0.45f, std::atan2(LocalY, LocalX)));
				Pose[2 + Leg * 3].Position = std::max(-1.0f, std::min(0.5f,
					std::atan2(Down, Reach) - std::atan2(0.75f * std::sin(Knee), 0.38f + 0.75f * CosKnee)));
				Pose[3 + Leg * 3].Position = std::max(0.5f, std::min(2.3f, Knee));
			}
			m_SweepRobotPose->SetConfiguration(Pose);
		}
		const FeatherstoneSolver& Tree = Robot ? Robot->GetDynamics() : *m_SweepRobotPose;
		for (const RobotSurface& Part : m_RobotSurfaces)
		{
			const Transform T = Tree.GetLinkTransform(Part.Link) * Part.Local;
			Part.Shape->SetTransform(T.pos, T.quat);
		}
	}
}
