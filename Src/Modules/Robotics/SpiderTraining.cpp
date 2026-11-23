#include "SpiderTraining.h"
#include "SpiderRobot.h"
#include "../../RigidBodyDynamics/PhysicsWorld.h"
#include "../../LinearSystem/NeuralNetwork.h"
#include "../../CollisionPrimitive/HeightField3.h"
#include "Models/Model.h"
#include "Optimizer/Optimizer.h"
#include "Reinforcement/PPO.h"
#include "Reinforcement/PpoRunner.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <future>
#include <iostream>
#include <iterator>
#include <limits>
#include <memory>
#include <numeric>
#include <random>
#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <Windows.h>
#endif

namespace
{
	using namespace Riemann;
	constexpr float ActionTime = 0.05f;
	constexpr float PolicyDeviation = 0.15f;
	constexpr float RewardScale = 0.02f;
	constexpr elman::Backend TrainingBackend = elman::Backend::CUDA;
	elman::Json SpiderModelDefinition()
	{
		return elman::Json::parse(R"({"format":"elman.model","version":1,
			"architecture":"continuous_actor_critic","name":"spider","input_features":86,"actions":24,
			"hidden_features":128,"hidden_layers":3})");
	}
	std::unique_ptr<elman::Model> CreateSpiderModel(const elman::Json& Definition)
	{
		std::unique_ptr<elman::Model> Model = elman::create_model(Definition, TrainingBackend, 42);
		auto& Parameters = Model->parameters();
		if (Parameters.size() != 8 || Parameters[6].name != "layers.3.weight" ||
			Parameters[6].value.shape() != elman::Shape{128, 25})
			throw std::runtime_error("Unexpected spider policy output layer");
		// The zero output weights start the robot at its standing controller.
		Parameters[6].value.assign(std::vector<float>(Parameters[6].value.numel(), 0));
		return Model;
	}
	using Evaluation = Riemann::SpiderEvaluation;
	struct CourseMap;
	Evaluation Evaluate(elman::Model& Model, int Steps = 100, float TerrainScale = 0, const CourseMap* Course = nullptr);
	void CheckInferenceParity(elman::Model& Model, const std::string& Path);
	std::unique_ptr<Maths::LinearAlgebra::NeuralNetwork> BuildActor(const elman::Model& Model);

	struct CourseMap
	{
		uint32_t Width = 0, Height = 0;
		float HalfX = 0, HalfZ = 0, Top = 0;
		std::vector<uint8_t> Pixels;
	};

	CourseMap LoadCourseMap(const std::string& Path)
	{
		CourseMap Result;
		std::ifstream File(Path, std::ios::binary);
		char Magic[4]{};
		File.read(Magic, sizeof(Magic));
		File.read(reinterpret_cast<char*>(&Result.Width), sizeof(Result.Width));
		File.read(reinterpret_cast<char*>(&Result.Height), sizeof(Result.Height));
		File.read(reinterpret_cast<char*>(&Result.HalfX), sizeof(Result.HalfX));
		File.read(reinterpret_cast<char*>(&Result.HalfZ), sizeof(Result.HalfZ));
		File.read(reinterpret_cast<char*>(&Result.Top), sizeof(Result.Top));
		if (!File || std::string(Magic, sizeof(Magic)) != "RHF1" || Result.Width < 2 || Result.Height < 2 ||
			uint64_t(Result.Width) * Result.Height > 4000000 || Result.HalfX <= 0 || Result.HalfZ <= 0 || Result.Top <= 0)
			throw std::runtime_error("Invalid spider training course");
		Result.Pixels.resize(size_t(Result.Width) * Result.Height);
		File.read(reinterpret_cast<char*>(Result.Pixels.data()), Result.Pixels.size());
		if (!File) throw std::runtime_error("Incomplete spider training course");
		return Result;
	}

	void AddCourse(PhysicsWorld& World, const CourseMap& Map, float Scale)
	{
		const float Dx = 2 * Map.HalfX / (Map.Width - 1);
		const float Dz = 2 * Map.HalfZ / (Map.Height - 1);
		constexpr uint32_t Tile = 128;
		for (uint32_t Z = 0; Z < Map.Height - 1; Z += Tile)
			for (uint32_t X = 0; X < Map.Width - 1; X += Tile)
			{
				const uint32_t W = std::min(Tile + 1, Map.Width - X);
				const uint32_t H = std::min(Tile + 1, Map.Height - Z);
				const Vector3 Min(14 - Map.HalfX + X * Dx, 0, -Map.HalfZ + Z * Dz);
				std::unique_ptr<Geometry> G(GeometryFactory::CreateHeightField(
					Box3(Min, Min + Vector3((W - 1) * Dx, 0, (H - 1) * Dz)), W, H));
				HeightField3* Field = G->GetShapeObj<HeightField3>();
				Field->HeightScale = Scale * Map.Top / 255;
				Field->AllocMemory();
				for (uint32_t J = 0; J < H; ++J)
					for (uint32_t I = 0; I < W; ++I)
						Field->Cells[J * W + I].HeightSample = Map.Pixels[(Z + J) * Map.Width + X + I];
				Field->RecalculateBounds();
				G->SetShapeBounds(Field->BV);
				if (!World.CreateRigidBody(G.get(), RigidBodyParam()))
					throw std::runtime_error("Could not create spider training course tile");
				G.release();
			}
	}

	struct Environment : elman::rl::PpoEnvironment
	{
		std::unique_ptr<PhysicsWorld> World;
		std::shared_ptr<SpiderRobotControl> Control;
		ArticulatedRobotController* Robot = nullptr;
		int Actions = 0;
		int MaximumActions = 600;
		int LastGroundedFrames = 0;
		float LastMinHeight = 100;
		float LastMinUp = 1;
		int TrainingId = -1;
		int EpisodeIndex = 0;
		float TerrainScale = 0;
		const CourseMap* Course = nullptr;
		Vector3 Command = Vector3(0.8f, 0, 0);

		void Reset()
		{
			World.reset(new PhysicsWorld(PhysicsWorldParam()));
			if (TerrainScale > 0) AddCourse(*World, *Course, TerrainScale);
			else World->CreateRigidBody(GeometryFactory::CreatePlane(Vector3::Zero(), Vector3::UnitY()), RigidBodyParam());
			Control = std::make_shared<SpiderRobotControl>();
			ArticulatedRobotSettings Settings = CreateSpiderRobotSettings(Control);
			if (TrainingId >= 0)
				Settings.Pose[0].Rotation.FromRotateZ(0.20f * std::sin(
					float((TrainingId + 1) * 1.7 + (++EpisodeIndex) * 2.4)));
			Robot = World->CreateRobotController(Settings);
			Robot->AddLinkCollider(0, Transform(),
				std::unique_ptr<Geometry>(GeometryFactory::CreateOBB(Vector3::Zero(), Vector3(0.27f, 0.22f, 0.08f))));
			Actions = 0;
			LastGroundedFrames = 0;
			LastMinHeight = 100;
			LastMinUp = 1;
		}
		std::vector<float> Observe() const { return ObserveSpiderRobot(Robot->GetDynamics(), Command, *Control); }
		std::vector<float> observe() const override { return Observe(); }
		elman::rl::PpoOutcome step(const std::vector<float>& Action) override
		{
			const auto Outcome = Step(Action);
			return {Outcome.first, Outcome.second};
		}
		void reset() override { Reset(); }
		std::pair<float, bool> Step(const std::vector<float>& Action)
		{
			const Transform BaseBefore = Robot->GetDynamics().GetLinkTransform(0);
			std::array<Vector3, SpiderLegCount> FootBefore, RelativeBefore;
			for (size_t Leg = 0; Leg < SpiderLegCount; ++Leg)
			{
				FootBefore[Leg] = Robot->GetDynamics().GetLinkTransform(3 * Leg + 3) * Vector3(0.75f, 0, 0);
				RelativeBefore[Leg] = BaseBefore.quat.Conjugate() * (FootBefore[Leg] - BaseBefore.pos);
			}
			ApplySpiderAction(*Control, Action);
			const float PreviousX = Robot->GetPosition().x;
			const float PreviousZ = Robot->GetPosition().z;
			LastGroundedFrames = 0;
			LastMinHeight = 100;
			LastMinUp = 1;
			for (int Frame = 0; Frame < 3; ++Frame)
			{
				World->Simulate(1.0f / 60.0f);
				LastGroundedFrames += Robot->IsGrounded();
				LastMinHeight = std::min(LastMinHeight, Robot->GetPosition().y);
				LastMinUp = std::min(LastMinUp,
					(Robot->GetDynamics().GetLinkTransform(0).quat * Vector3::UnitZ()).y);
			}
			++Actions;
			const std::vector<float> Observation = Observe();
			const Vector3 Up = Robot->GetDynamics().GetLinkTransform(0).quat * Vector3::UnitZ();
			const float Vx = Observation[0], Vy = Observation[1], Vz = Observation[2];
			const float Error = (Vx - Observation[9]) * (Vx - Observation[9])
				+ (Vy - Observation[10]) * (Vy - Observation[10]) + 0.25f * Vz * Vz;
			const float Progress = (Robot->GetPosition().x - PreviousX) / ActionTime;
			const float LateralSpeed = (Robot->GetPosition().z - PreviousZ) / ActionTime;
			float Reward = 0.2f * Up.y;
			Reward -= 6.0f * std::min(1.0f, std::abs(LateralSpeed));
			const bool Settled = Actions > 100;
			if (Settled)
				Reward += 0.5f * std::exp(-Error / 0.25f)
					+ 8.0f * std::max(-1.0f, std::min(1.0f, Progress));
			const Transform BaseAfter = Robot->GetDynamics().GetLinkTransform(0);
			Reward += 12.0f * ((BaseAfter.quat * Vector3::UnitX()).x - 1.0f);
			Reward -= 1.5f * std::min(2.0f, std::abs(Robot->GetPosition().z));
			Reward -= 0.2f * Observation[5] * Observation[5];
			for (size_t Leg = 0; Leg < SpiderLegCount; ++Leg)
			{
				const Vector3 FootAfter = Robot->GetDynamics().GetLinkTransform(3 * Leg + 3) * Vector3(0.75f, 0, 0);
				float GroundHeight = 0;
				if (TerrainScale > 0)
				{
					RayCastOption Option;
					Option.MaxDist = 3;
					RayCastResult Hit;
					if (World->GetGeometryQuery()->RayCastQuery(
						Vector3(FootAfter.x, 2, FootAfter.z), -Vector3::UnitY(), Option, &Hit))
						GroundHeight = 2 - Hit.hitTimeMin;
				}
				const Vector3 RelativeAfter = BaseAfter.quat.Conjugate() * (FootAfter - BaseAfter.pos);
				const float RelativeSpeed = (RelativeAfter.x - RelativeBefore[Leg].x) / ActionTime;
				const float Swing = std::max(0.0f, std::min(1.0f,
					(FootAfter.y - GroundHeight - 0.05f) / 0.08f));
				if (Settled)
				{
					const float SwingPhase = std::sin(6.283185307179586f *
						(Control->Phase + (Leg % 2 ? 0.5f : 0.0f)));
					const float DesiredHeight = GroundHeight + 0.035f + 0.10f * std::max(0.0f, SwingPhase);
					const float HeightError = FootAfter.y - DesiredHeight;
					Reward += 0.15f * std::exp(-HeightError * HeightError / 0.0025f);
					Reward += 0.08f * Swing * std::max(0.0f, std::min(2.0f, RelativeSpeed));
					Reward += 0.04f * (1.0f - Swing) * std::max(0.0f, std::min(2.0f, -RelativeSpeed));
				}
				const float GroundSlip = (FootAfter - FootBefore[Leg]).SquareLength() / (ActionTime * ActionTime);
				Reward -= 0.02f * (1.0f - Swing) * std::min(4.0f, GroundSlip);
			}
			for (size_t I = 0; I < SpiderMotorCount; ++I)
				Reward -= 0.005f * Action[I] * Action[I] + 0.00005f * Observation[36 + I] * Observation[36 + I];
			const bool Fallen = Robot->GetPosition().y < 0.25f || Up.y < 0.55f;
			Reward -= 5.0f * (3 - LastGroundedFrames) / 3.0f;
			if (Fallen) Reward -= 5;
			return {Reward, Fallen || Actions >= MaximumActions};
		}
	};

	std::vector<float> PackWeights(const elman::Model& Model)
	{
		const std::vector<elman::Parameter>& Parameters = Model.parameters();
		const int Width[] = {86, 128, 128, 128, 25};
		if (Parameters.size() != 8) throw std::runtime_error("Unexpected robot policy parameters");
		std::vector<float> Packed;
		size_t Count = 0;
		for (int Layer = 0; Layer < 4; ++Layer) Count += size_t(Width[Layer] + 1) * Width[Layer + 1];
		Packed.reserve(Count);
		for (int Layer = 0; Layer < 4; ++Layer)
		{
			const std::vector<float> Weight = Parameters[2 * Layer].value.data();
			const std::vector<float> Bias = Parameters[2 * Layer + 1].value.data();
			for (int Out = 0; Out < Width[Layer + 1]; ++Out)
				for (int In = 0; In < Width[Layer]; ++In)
					Packed.push_back(Weight[In * Width[Layer + 1] + Out]);
			Packed.insert(Packed.end(), Bias.begin(), Bias.end());
		}
		return Packed;
	}

	std::unique_ptr<Maths::LinearAlgebra::NeuralNetwork> BuildActor(const elman::Model& Model)
	{
		using namespace Maths::LinearAlgebra;
		return std::unique_ptr<NeuralNetwork>(new NeuralNetwork(86,
			{{NeuralLayerType::Linear, 128}, {NeuralLayerType::ELU},
			{NeuralLayerType::Linear, 128}, {NeuralLayerType::ELU},
			{NeuralLayerType::Linear, 128}, {NeuralLayerType::ELU},
			{NeuralLayerType::Linear, 25}}, PackWeights(Model)));
	}

	void ExportPolicy(const elman::Model& Model, const std::string& Path)
	{
		const std::vector<float> Packed = PackWeights(Model);
		const std::string Temporary = Path + ".tmp";
		std::ofstream File(Temporary, std::ios::binary | std::ios::trunc);
		if (!File) throw std::runtime_error("Cannot open spider policy output");
		File.write(reinterpret_cast<const char*>(Packed.data()), Packed.size() * sizeof(float));
		if (!File) throw std::runtime_error("Could not write spider policy");
		File.close();
		if (!File) throw std::runtime_error("Could not close spider policy");
#ifdef _WIN32
		if (!MoveFileExA(Temporary.c_str(), Path.c_str(), MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH))
			throw std::runtime_error("Could not replace spider policy");
#else
		if (std::rename(Temporary.c_str(), Path.c_str()) != 0)
			throw std::runtime_error("Could not replace spider policy");
#endif
	}

	bool ImportPolicy(elman::Model& Model, const std::string& Path)
	{
		std::ifstream File(Path, std::ios::binary);
		if (!File) return false;
		std::vector<elman::Parameter>& Parameters = Model.parameters();
		const int Width[] = {86, 128, 128, 128, 25};
		for (int Layer = 0; Layer < 4; ++Layer)
		{
			std::vector<float> Weight(Width[Layer] * Width[Layer + 1]), Bias(Width[Layer + 1]);
			for (int Out = 0; Out < Width[Layer + 1]; ++Out)
				for (int In = 0; In < Width[Layer]; ++In)
					File.read(reinterpret_cast<char*>(&Weight[In * Width[Layer + 1] + Out]), sizeof(float));
			File.read(reinterpret_cast<char*>(Bias.data()), Bias.size() * sizeof(float));
			if (!File) throw std::runtime_error("Incomplete spider policy input");
			Parameters[2 * Layer].value.assign(Weight);
			Parameters[2 * Layer + 1].value.assign(Bias);
		}
		if (File.peek() != std::char_traits<char>::eof())
			throw std::runtime_error("Unexpected spider policy data");
		return true;
	}

	std::vector<float> Predict(elman::Model& Model, const std::vector<float>& Observation)
	{
		return Model.forward(elman::Tensor::from_data({1, 86}, Observation, TrainingBackend), false).data();
	}

	Evaluation Evaluate(elman::Model& Model, int Steps, float TerrainScale, const CourseMap* Course)
	{
		std::unique_ptr<Maths::LinearAlgebra::NeuralNetwork> Actor = BuildActor(Model);
		Environment Env;
		Env.MaximumActions = Steps;
		Env.TerrainScale = TerrainScale;
		Env.Course = Course;
		Env.Reset();
		Evaluation Result;
		std::array<double, SpiderMotorCount> PreviousTarget{};
		float SegmentOrigin = 0;
		for (int I = 0; I < Steps; ++I)
		{
			const std::vector<float> Output = Actor->Evaluate(Env.Observe());
			const std::vector<float> Action(Output.begin(), Output.begin() + SpiderMotorCount);
			const float Before = Env.Robot->GetPosition().x;
			const bool Done = Env.Step(Action).second;
			for (size_t M = 0; M < SpiderMotorCount; ++M)
			{
				const double Value = Env.Control->TargetOffset[M];
				Result.MeanActionOffset += static_cast<float>(std::abs(Value));
				if (I >= 100) Result.MeanActionChange += static_cast<float>(std::abs(Value - PreviousTarget[M]));
				PreviousTarget[M] = Value;
			}
			Result.Distance += Env.Robot->GetPosition().x - Before;
			Result.Steps = I + 1;
			if (Result.Steps % 100 == 0 && Result.Steps <= 600)
			{
				Result.Segments[Result.Steps / 100 - 1] = Result.Distance - SegmentOrigin;
				SegmentOrigin = Result.Distance;
			}
			if (Result.Steps == 100) Result.First5 = Result.Distance;
			if (Result.Steps == Steps - 100) Result.Last5 = -Result.Distance;
			Result.MinHeight = std::min(Result.MinHeight, Env.LastMinHeight);
			Result.MinUp = std::min(Result.MinUp, Env.LastMinUp);
			Result.Grounded += Env.LastGroundedFrames;
			if (Done && I != Steps - 1) break;
		}
		if (Result.Steps > 0)
		{
			Result.FinalFacing = (Env.Robot->GetDynamics().GetLinkTransform(0).quat * Vector3::UnitX()).x;
			Result.Lateral = Env.Robot->GetPosition().z;
			Result.MeanActionOffset /= Result.Steps * SpiderMotorCount;
			if (Result.Steps > 100) Result.MeanActionChange /= (Result.Steps - 100) * SpiderMotorCount;
			if (Result.Steps == Steps) Result.Last5 += Result.Distance;
			else Result.Last5 = 0;
		}
		return Result;
	}

	float AcceptanceScore(const Evaluation& Flat, const Evaluation& Course)
	{
		if (Flat.Steps != 600 || Course.Steps != 600) return -1000000.0f;
		const auto Deficit = [](const Evaluation& Run, float Distance)
		{
			return std::max(0.0f, Distance - Run.Distance) / Distance
				+ std::max(0.0f, 2.0f - Run.Last5) / 2.0f
				+ 3.0f * std::max(0.0f, 0.75f - Run.FinalFacing)
				+ 3.0f * std::max(0, 1401 - Run.Grounded) / 1401.0f
				+ std::max(0.0f, std::abs(Run.Lateral) - 2.0f)
				+ 5.0f * std::max(0.0f, 0.35f - Run.MinHeight)
				+ 5.0f * std::max(0.0f, 0.9f - Run.MinUp);
		};
		return -100.0f * (Deficit(Flat, 24) + Deficit(Course, 22.5f))
			+ std::min(Flat.Distance, 34.0f) / 10.0f + std::min(Course.Distance, 33.0f) / 10.0f;
	}

	void CheckInferenceParity(elman::Model& Model, const std::string& Path)
	{
		const int Width[] = {86, 128, 128, 128, 25};
		size_t Count = 0;
		for (int I = 0; I < 4; ++I) Count += size_t(Width[I] + 1) * Width[I + 1];
		std::vector<float> Weights(Count);
		std::ifstream File(Path, std::ios::binary);
		File.read(reinterpret_cast<char*>(Weights.data()), Weights.size() * sizeof(float));
		if (!File || File.peek() != std::char_traits<char>::eof())
			throw std::runtime_error("Invalid spider policy for inference parity");
		using namespace Maths::LinearAlgebra;
		NeuralNetwork Actor(86,
			{{NeuralLayerType::Linear, 128}, {NeuralLayerType::ELU},
			{NeuralLayerType::Linear, 128}, {NeuralLayerType::ELU},
			{NeuralLayerType::Linear, 128}, {NeuralLayerType::ELU},
			{NeuralLayerType::Linear, 25}}, std::move(Weights));
		Environment Cuda, Cpu;
		Cuda.Reset(); Cpu.Reset();
		double ErrorSum = 0, ErrorMax = 0;
		for (int Step = 0; Step < 600; ++Step)
		{
			const std::vector<float> Observation = Cuda.Observe();
			const std::vector<float> CudaOutput = Predict(Model, Observation);
			const std::vector<float> CpuOnSameState = Actor.Evaluate(Observation);
			for (size_t I = 0; I < CudaOutput.size(); ++I)
			{
				const double Error = std::abs(double(CudaOutput[I]) - CpuOnSameState[I]);
				ErrorSum += Error;
				ErrorMax = std::max(ErrorMax, Error);
			}
			const std::vector<float> CpuOutput = Actor.Evaluate(Cpu.Observe());
			Cuda.Step(CudaOutput);
			Cpu.Step(CpuOutput);
		}
		std::cout << "parity_mean_error=" << ErrorSum / (600 * 25)
			<< " parity_max_error=" << ErrorMax
			<< " cuda_x=" << Cuda.Robot->GetPosition().x
			<< " cuda_z=" << Cuda.Robot->GetPosition().z
			<< " cpu_actor_x=" << Cpu.Robot->GetPosition().x
			<< " cpu_actor_z=" << Cpu.Robot->GetPosition().z << std::endl;
	}
}

void Riemann::RunSpiderTraining(const SpiderTrainingOptions& Options, const std::atomic<bool>& Stop,
	const SpiderSnapshotCallback& OnSnapshot, const SpiderProgressCallback& OnProgress)
{
		if (Options.policy_path.empty()) throw std::invalid_argument("Spider policy path is required");
		if (Options.environment_count < 1 || Options.environment_count > int(SpiderTrainingEnvironmentCount))
			throw std::invalid_argument("Invalid spider training world count");
		if (Options.updates < 1 || Options.rollout_actions < 32)
			throw std::invalid_argument("Invalid training duration or rollout action count");
		if (Options.rollout_actions > std::numeric_limits<int>::max() - Options.environment_count)
			throw std::invalid_argument("Rollout action count is too large");
		if (Options.course_scale < 0 || Options.course_scale > 1)
			throw std::invalid_argument("Invalid course scale");
		const size_t Separator = Options.policy_path.find_last_of("/\\");
		const std::string CoursePath = Options.course_path.empty()
			? (Separator == std::string::npos ? std::string() : Options.policy_path.substr(0, Separator + 1)) + "course.height"
			: Options.course_path;
		const CourseMap Course = LoadCourseMap(CoursePath);
		const elman::Json Definition = SpiderModelDefinition();
		std::unique_ptr<elman::Model> Model = CreateSpiderModel(Definition);
		if (Model->parameters().empty() || Model->parameters().front().value.backend() != elman::Backend::CUDA)
			throw std::runtime_error("Spider PPO requires the Elman CUDA backend");
		std::cout << "elman_backend=CUDA physics_backend=CPU" << std::endl;
		const int Updates = Options.updates;
		const int EnvironmentCount = Options.environment_count;
		const int RolloutActions = ((Options.rollout_actions + EnvironmentCount - 1) / EnvironmentCount) * EnvironmentCount;
		const float CourseScale = Options.course_scale;
		const std::string LatestPath = Options.policy_path + ".latest";
		const bool Resumed = ImportPolicy(*Model, LatestPath);
		elman::Optimizer Optimizer(elman::Json::Object{{"optimizer", "adam"}, {"learning_rate", 0.00001}, {"clip_norm", 1.0}}, *Model, true);
		std::unique_ptr<elman::Model> ZeroActor = CreateSpiderModel(Definition);
		const Evaluation Baseline = Evaluate(*ZeroActor, 600);
		const Evaluation Initial = Resumed ? Evaluate(*Model, 600) : Baseline;
		const Evaluation InitialCourse = Resumed ? Evaluate(*Model, 600, 1.0f, &Course) : Evaluation();
		const auto IsAligned = [](const Evaluation& Result)
		{
			return Result.Steps == 600 && Result.Last5 >= 2.0f &&
				Result.FinalFacing >= 0.75f && std::abs(Result.Lateral) <= 2.0f &&
				Result.Grounded >= 1401;
		};
		float BestCandidateDistance = Initial.Steps == 600 && Initial.Last5 >= 2.0f
			? Initial.Distance : Baseline.Distance;
		float BestAlignedDistance = IsAligned(Initial) ? Initial.Distance : Baseline.Distance;
		float BestCourseDistance = InitialCourse.Steps == 600 ? std::max(0.0f, InitialCourse.Distance) : 0;
		float BestNearScore = AcceptanceScore(Initial, InitialCourse);
		float BestDistance = IsAligned(Initial) && Initial.Distance >= 24.0f &&
			Initial.MinHeight > 0.35f && Initial.MinUp > 0.9f &&
			InitialCourse.Steps == 600 && InitialCourse.Distance >= 22.5f && InitialCourse.Last5 >= 2.0f &&
			InitialCourse.FinalFacing >= 0.75f && std::abs(InitialCourse.Lateral) <= 2.0f &&
			InitialCourse.Grounded >= 1401 && InitialCourse.MinHeight > 0.35f && InitialCourse.MinUp > 0.9f
			? Initial.Distance : Baseline.Distance;
		float BestIntermediateDistance = 0;
		std::cout << "standing_baseline_distance_30s=" << Baseline.Distance
			<< " initial_policy_distance_30s=" << Initial.Distance
			<< " initial_policy_last_5s=" << Initial.Last5
			<< " initial_course_distance_30s=" << InitialCourse.Distance
			<< " training_course_scale=" << CourseScale << std::endl;
		std::array<Environment, SpiderTrainingEnvironmentCount> Environments;
		std::vector<elman::rl::PpoEnvironment*> EnvironmentPointers;
		EnvironmentPointers.reserve(EnvironmentCount);
		for (int E = 0; E < EnvironmentCount; ++E)
		{
			Environments[E].TrainingId = E;
			Environments[E].TerrainScale = (EnvironmentCount == 1 ? 1.0f :
				E % 4 == 0 ? 0.0f : E % 4 == 1 ? 0.25f : E % 4 == 2 ? 0.5f : 1.0f) * CourseScale;
			Environments[E].Course = &Course;
			Environments[E].Reset();
			EnvironmentPointers.push_back(&Environments[E]);
		}
		elman::rl::PpoRunnerSettings RunnerSettings;
		RunnerSettings.rollout_actions = RolloutActions;
		RunnerSettings.standard_deviation = PolicyDeviation;
		RunnerSettings.reward_scale = RewardScale;
		RunnerSettings.on_environment_step = [&](size_t Index, const elman::rl::PpoEnvironment& Base)
		{
			if (!OnSnapshot) return;
			const auto& Env = static_cast<const Environment&>(Base);
			std::array<Transform, SpiderTrainingLinkCount> Links;
			for (size_t Link = 0; Link < Links.size(); ++Link)
				Links[Link] = Env.Robot->GetDynamics().GetLinkTransform(Link);
			OnSnapshot(Index, Links);
		};
		elman::rl::PpoRunner Runner(*Model, Optimizer, std::move(EnvironmentPointers),
			[](const elman::Model& Source)
			{
				auto Actor = std::shared_ptr<Maths::LinearAlgebra::NeuralNetwork>(BuildActor(Source).release());
				return elman::rl::PpoInference([Actor](const std::vector<float>& Observation)
				{
					return Actor->Evaluate(Observation);
				});
			}, RunnerSettings);
		Runner.run(Updates, [&](const elman::rl::PpoIteration& Iteration)
		{
			const int Update = int(Iteration.update) - 1;
			if (OnProgress) OnProgress(Iteration.update, Iteration.mean_reward);
			const Evaluation Result = Evaluate(*Model, 200);
			if (Result.Steps == 200 && Result.Last5 > BestIntermediateDistance && Result.Last5 > 0.2f)
			{
				BestIntermediateDistance = Result.Last5;
				ExportPolicy(*Model, Options.policy_path + ".intermediate");
			}
			Evaluation LongRun;
			Evaluation CourseRun;
			if ((Result.Steps == 200 && Result.Last5 > 0.2f) || Update % 10 == 9)
			{
				LongRun = Evaluate(*Model, 600);
				if (LongRun.Steps == 600 && LongRun.Last5 >= 2.0f && LongRun.Distance > BestCandidateDistance)
				{
					BestCandidateDistance = LongRun.Distance;
					ExportPolicy(*Model, Options.policy_path + ".candidate");
				}
				if (IsAligned(LongRun) && LongRun.Distance > BestAlignedDistance)
				{
					BestAlignedDistance = LongRun.Distance;
					ExportPolicy(*Model, Options.policy_path + ".aligned");
				}
			}
			const bool FlatReady = IsAligned(LongRun) && LongRun.Distance >= 24.0f &&
				LongRun.MinHeight > 0.35f && LongRun.MinUp > 0.9f;
			const bool FlatNear = LongRun.Steps == 600 && LongRun.Distance >= 20.0f &&
				LongRun.Last5 >= 2.0f && LongRun.FinalFacing >= 0.5f &&
				std::abs(LongRun.Lateral) <= 4.0f && LongRun.Grounded >= 1400 &&
				LongRun.MinHeight > 0.34f && LongRun.MinUp > 0.88f;
			if (FlatNear || Update % 10 == 9)
			{
					CourseRun = Evaluate(*Model, 600, 1.0f, &Course);
				const float NearScore = AcceptanceScore(LongRun, CourseRun);
				if (NearScore > BestNearScore)
				{
					BestNearScore = NearScore;
					ExportPolicy(*Model, Options.policy_path + ".near");
				}
				if (CourseRun.Steps == 600 && CourseRun.Distance > BestCourseDistance)
				{
					BestCourseDistance = CourseRun.Distance;
					ExportPolicy(*Model, Options.policy_path + ".course_candidate");
				}
				if (FlatReady && CourseRun.Steps == 600 && CourseRun.Distance >= 22.5f &&
					CourseRun.Last5 >= 2.0f && CourseRun.FinalFacing >= 0.75f &&
					std::abs(CourseRun.Lateral) <= 2.0f && CourseRun.Grounded >= 1401 &&
					CourseRun.MinHeight > 0.35f && CourseRun.MinUp > 0.9f &&
					LongRun.Distance > BestDistance)
				{
					BestDistance = LongRun.Distance;
					ExportPolicy(*Model, Options.policy_path);
				}
			}
			ExportPolicy(*Model, LatestPath);
			std::cout << "update=" << Iteration.update << " reward=" << Iteration.mean_reward
				<< " policy_loss=" << Iteration.policy_loss
				<< " value_loss=" << Iteration.value_loss
				<< " gradient_norm=" << Iteration.gradient_norm
				<< " episodes=" << Iteration.episodes << " eval_distance_10s=" << Result.Distance
				<< " eval_distance_5to10s=" << Result.Last5
				<< " eval_action_change=" << Result.MeanActionChange
				<< " eval_steps=" << Result.Steps
				<< " best_intermediate_5to10s=" << BestIntermediateDistance
				<< " best_candidate_30s=" << BestCandidateDistance
				<< " best_aligned_30s=" << BestAlignedDistance
				<< " best_course_30s=" << BestCourseDistance
				<< " best_near_score=" << BestNearScore
				<< " best_deploy_30s=" << BestDistance
				<< " eval30_steps=" << LongRun.Steps
				<< " eval30_distance=" << LongRun.Distance
				<< " eval30_last5=" << LongRun.Last5
				<< " eval30_lateral=" << LongRun.Lateral
				<< " eval30_facing=" << LongRun.FinalFacing
				<< " eval30_grounded=" << LongRun.Grounded
				<< " eval30_minheight=" << LongRun.MinHeight
				<< " eval30_minup=" << LongRun.MinUp
				<< " course30_steps=" << CourseRun.Steps
				<< " course30_distance=" << CourseRun.Distance
				<< " course30_last5=" << CourseRun.Last5
				<< " course30_lateral=" << CourseRun.Lateral
				<< " course30_facing=" << CourseRun.FinalFacing
				<< " course30_grounded=" << CourseRun.Grounded
				<< " course30_minheight=" << CourseRun.MinHeight
				<< " course30_minup=" << CourseRun.MinUp << std::endl;
		}, [&] { return Stop.load(); });
}

Riemann::SpiderEvaluation Riemann::EvaluateSpiderPolicy(const std::string& PolicyPath, int Steps, float CourseScale)
{
	if (Steps < 1 || CourseScale < 0 || CourseScale > 1)
		throw std::invalid_argument("Invalid spider policy evaluation settings");
	const CourseMap Course = CourseScale > 0 ? LoadCourseMap("Contents/Robots/Spider/course.height") : CourseMap{};
	const elman::Json Definition = SpiderModelDefinition();
	std::unique_ptr<elman::Model> Model = CreateSpiderModel(Definition);
	if (!ImportPolicy(*Model, PolicyPath)) throw std::runtime_error("Spider policy file is missing");
	return Evaluate(*Model, Steps, CourseScale, CourseScale > 0 ? &Course : nullptr);
}
