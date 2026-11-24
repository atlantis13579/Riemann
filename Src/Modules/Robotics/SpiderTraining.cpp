#include "SpiderTraining.h"
#include "SpiderRobot.h"
#include "../../RigidBodyDynamics/PhysicsWorld.h"
#include "../../LinearSystem/NeuralNetwork.h"
#include "../../CollisionPrimitive/HeightField3.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
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
#include <winsock2.h>
#include <ws2tcpip.h>
#include <Windows.h>
#endif

namespace
{
	using namespace Riemann;
	constexpr float ActionTime = 0.05f;
	using Evaluation = Riemann::SpiderEvaluation;
	struct CourseMap;
	Evaluation Evaluate(const std::vector<float>& Weights, int Steps = 100, float TerrainScale = 0,
		const CourseMap* Course = nullptr);
	std::unique_ptr<Maths::LinearAlgebra::NeuralNetwork> BuildActor(const std::vector<float>& Weights);

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

	struct Environment
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

	constexpr size_t WeightCount = 47385;
	std::unique_ptr<Maths::LinearAlgebra::NeuralNetwork> BuildActor(const std::vector<float>& Weights)
	{
		using namespace Maths::LinearAlgebra;
		if (Weights.size() != WeightCount) throw std::runtime_error("Invalid spider policy weights");
		return std::unique_ptr<NeuralNetwork>(new NeuralNetwork(86,
			{{NeuralLayerType::Linear, 128}, {NeuralLayerType::ELU},
			{NeuralLayerType::Linear, 128}, {NeuralLayerType::ELU},
			{NeuralLayerType::Linear, 128}, {NeuralLayerType::ELU},
			{NeuralLayerType::Linear, 25}}, Weights));
	}

	void ExportPolicy(const std::vector<float>& Weights, const std::string& Path)
	{
		if (Weights.size() != WeightCount) throw std::runtime_error("Invalid spider policy weights");
		const std::string Temporary = Path + ".tmp";
		std::ofstream File(Temporary, std::ios::binary | std::ios::trunc);
		if (!File) throw std::runtime_error("Cannot open spider policy output");
		File.write(reinterpret_cast<const char*>(Weights.data()), Weights.size() * sizeof(float));
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

	std::vector<float> ImportPolicy(const std::string& Path)
	{
		std::ifstream File(Path, std::ios::binary);
		if (!File) return {};
		std::vector<float> Weights(WeightCount);
		File.read(reinterpret_cast<char*>(Weights.data()), Weights.size() * sizeof(float));
		if (!File || File.peek() != std::char_traits<char>::eof())
			throw std::runtime_error("Invalid spider policy input");
		return Weights;
	}
	Evaluation Evaluate(const std::vector<float>& Weights, int Steps, float TerrainScale, const CourseMap* Course)
	{
		std::unique_ptr<Maths::LinearAlgebra::NeuralNetwork> Actor = BuildActor(Weights);
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

	struct SocketRuntime
	{
		SocketRuntime()
		{
			WSADATA Data{};
			if (WSAStartup(MAKEWORD(2, 2), &Data)) throw std::runtime_error("Cannot start TCP");
		}
		~SocketRuntime() { WSACleanup(); }
	};

	struct Socket
	{
		SOCKET Value = INVALID_SOCKET;
		Socket() = default;
		explicit Socket(SOCKET Value) : Value(Value) {}
		~Socket() { if (Value != INVALID_SOCKET) closesocket(Value); }
		Socket(const Socket&) = delete;
	};

	struct ElmanProcess
	{
		PROCESS_INFORMATION Info{};
		ElmanProcess(const std::string& Path, int Port)
		{
			if (Path.empty()) throw std::runtime_error("Select an Elman executable before training");
			const DWORD Attributes = GetFileAttributesA(Path.c_str());
			if (Attributes == INVALID_FILE_ATTRIBUTES || (Attributes & FILE_ATTRIBUTE_DIRECTORY))
				throw std::runtime_error("Elman executable is missing");
			std::string Command = "\"" + Path + "\" spider-ppo-client --port " + std::to_string(Port);
			STARTUPINFOA Startup{};
			Startup.cb = sizeof(Startup);
			const size_t Separator = Path.find_last_of("/\\");
			const std::string Directory = Separator == std::string::npos ? std::string() : Path.substr(0, Separator);
			if (!CreateProcessA(Path.c_str(), &Command[0], nullptr, nullptr, FALSE, CREATE_NO_WINDOW,
				nullptr, Directory.empty() ? nullptr : Directory.c_str(), &Startup, &Info))
				throw std::runtime_error("Cannot start Elman executable");
		}
		~ElmanProcess()
		{
			if (Info.hProcess)
			{
				if (WaitForSingleObject(Info.hProcess, 100) != WAIT_OBJECT_0)
					TerminateProcess(Info.hProcess, 1);
				CloseHandle(Info.hProcess);
			}
			if (Info.hThread) CloseHandle(Info.hThread);
		}
		bool Running() const { return WaitForSingleObject(Info.hProcess, 0) == WAIT_TIMEOUT; }
	};

	struct Listener
	{
		Socket Handle;
		int Port = 0;
		Listener()
		{
			Handle.Value = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
			if (Handle.Value == INVALID_SOCKET) throw std::runtime_error("Cannot open training TCP listener");
			sockaddr_in Address{};
			Address.sin_family = AF_INET;
			Address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
			if (bind(Handle.Value, reinterpret_cast<sockaddr*>(&Address), sizeof(Address)) != 0 ||
				listen(Handle.Value, 1) != 0)
				throw std::runtime_error("Cannot listen for Elman training TCP connection");
			int Length = sizeof(Address);
			if (getsockname(Handle.Value, reinterpret_cast<sockaddr*>(&Address), &Length) != 0)
				throw std::runtime_error("Cannot determine training TCP port");
			Port = ntohs(Address.sin_port);
		}
		Socket Accept(const std::atomic<bool>& Stop, const ElmanProcess& Process)
		{
			for (;;)
			{
				if (Stop.load()) throw std::runtime_error("Training cancelled");
				if (!Process.Running()) throw std::runtime_error("Elman exited before connecting to training TCP");
				fd_set Ready;
				FD_ZERO(&Ready);
				FD_SET(Handle.Value, &Ready);
				timeval Timeout{0, 100000};
				if (select(0, &Ready, nullptr, nullptr, &Timeout) > 0)
				{
					SOCKET Client = accept(Handle.Value, nullptr, nullptr);
					if (Client == INVALID_SOCKET) throw std::runtime_error("Cannot accept Elman training TCP");
					return Socket(Client);
				}
			}
		}
	};

	struct Packet
	{
		char Type = 0;
		std::vector<char> Bytes;
	};

	class Channel
	{
	public:
		Channel(SOCKET Handle, const std::atomic<bool>& Stop) : Handle(Handle), Stop(Stop) {}
		void Send(char Type, const void* Data = nullptr, uint32_t Size = 0)
		{
			if (Size > 1024 * 1024) throw std::runtime_error("Training TCP packet too large");
			const unsigned char Header[5] = {static_cast<unsigned char>(Type),
				static_cast<unsigned char>(Size), static_cast<unsigned char>(Size >> 8),
				static_cast<unsigned char>(Size >> 16), static_cast<unsigned char>(Size >> 24)};
			Write(Header, sizeof(Header));
			if (Size) Write(Data, Size);
		}
		Packet Receive()
		{
			unsigned char Header[5]{};
			Read(Header, sizeof(Header));
			const uint32_t Size = uint32_t(Header[1]) | (uint32_t(Header[2]) << 8) |
				(uint32_t(Header[3]) << 16) | (uint32_t(Header[4]) << 24);
			if (Size > 1024 * 1024) throw std::runtime_error("Training TCP packet too large");
			Packet Result;
			Result.Type = static_cast<char>(Header[0]);
			Result.Bytes.resize(Size);
			if (Size) Read(Result.Bytes.data(), Size);
			return Result;
		}
	private:
		void Write(const void* Data, size_t Size)
		{
			const char* Bytes = static_cast<const char*>(Data);
			while (Size)
			{
				if (Stop.load()) throw std::runtime_error("Training cancelled");
				const int Count = send(Handle, Bytes, int(Size), 0);
				if (Count <= 0) throw std::runtime_error("Elman training TCP connection closed");
				Bytes += Count;
				Size -= Count;
			}
		}
		void Read(void* Data, size_t Size)
		{
			char* Bytes = static_cast<char*>(Data);
			while (Size)
			{
				if (Stop.load()) throw std::runtime_error("Training cancelled");
				fd_set Ready;
				FD_ZERO(&Ready);
				FD_SET(Handle, &Ready);
				timeval Timeout{0, 100000};
				const int State = select(0, &Ready, nullptr, nullptr, &Timeout);
				if (State < 0) throw std::runtime_error("Training TCP read failed");
				if (!State) continue;
				const int Count = recv(Handle, Bytes, int(Size), 0);
				if (Count <= 0) throw std::runtime_error("Elman training TCP connection closed");
				Bytes += Count;
				Size -= Count;
			}
		}
		SOCKET Handle;
		const std::atomic<bool>& Stop;
	};

	template<class T> T Decode(const Packet& Value, size_t Offset = 0)
	{
		if (Offset + sizeof(T) > Value.Bytes.size()) throw std::runtime_error("Incomplete training TCP packet");
		T Result;
		std::memcpy(&Result, Value.Bytes.data() + Offset, sizeof(T));
		return Result;
	}
	std::vector<float> DecodeWeights(const Packet& Value, size_t Offset = 0)
	{
		if (Value.Bytes.size() != Offset + WeightCount * sizeof(float))
			throw std::runtime_error("Invalid training policy packet");
		std::vector<float> Weights(WeightCount);
		std::memcpy(Weights.data(), Value.Bytes.data() + Offset, WeightCount * sizeof(float));
		return Weights;
	}

}

void Riemann::RunSpiderTraining(const SpiderTrainingOptions& Options, const std::atomic<bool>& Stop,
	const SpiderSnapshotCallback& OnSnapshot, const SpiderProgressCallback& OnProgress)
{
	if (Options.executable_path.empty()) throw std::invalid_argument("Select an Elman executable before training");
	if (Options.policy_path.empty()) throw std::invalid_argument("Spider policy path is required");
	if (Options.environment_count < 1 || Options.environment_count > int(SpiderTrainingEnvironmentCount))
		throw std::invalid_argument("Invalid spider training world count");
	if (Options.updates < 1 || Options.rollout_actions < 32 ||
		Options.rollout_actions > std::numeric_limits<int>::max() - Options.environment_count)
		throw std::invalid_argument("Invalid training duration or rollout action count");
	if (Options.course_scale < 0 || Options.course_scale > 1)
		throw std::invalid_argument("Invalid course scale");
	const size_t Separator = Options.policy_path.find_last_of("/\\");
	const std::string CoursePath = Options.course_path.empty()
		? (Separator == std::string::npos ? std::string() : Options.policy_path.substr(0, Separator + 1)) + "course.height"
		: Options.course_path;
	const CourseMap Course = LoadCourseMap(CoursePath);
	const int Updates = Options.updates;
	const int EnvironmentCount = Options.environment_count;
	const int RolloutActions = ((Options.rollout_actions + EnvironmentCount - 1) / EnvironmentCount) * EnvironmentCount;
	const float CourseScale = Options.course_scale;
	const std::string LatestPath = Options.policy_path + ".latest";
	const std::vector<float> ResumeWeights = ImportPolicy(LatestPath);
	const bool Resumed = !ResumeWeights.empty();
	SocketRuntime Sockets;
	Listener Server;
	ElmanProcess Process(Options.executable_path, Server.Port);
	Socket Client = Server.Accept(Stop, Process);
	Channel Connection(Client.Value, Stop);
	const uint32_t Config[3] = {uint32_t(Updates), uint32_t(EnvironmentCount), uint32_t(RolloutActions)};
	Connection.Send('C', Config, sizeof(Config));
	Connection.Send('I', ResumeWeights.empty() ? nullptr : ResumeWeights.data(),
		uint32_t(ResumeWeights.size() * sizeof(float)));
	const Packet BaselinePacket = Connection.Receive();
	const Packet CurrentPacket = Connection.Receive();
	if (BaselinePacket.Type != 'B' || CurrentPacket.Type != 'W')
		throw std::runtime_error("Invalid Elman training handshake");
	const std::vector<float> BaselineWeights = DecodeWeights(BaselinePacket);
	std::vector<float> Weights = DecodeWeights(CurrentPacket);
	const Evaluation Baseline = Evaluate(BaselineWeights, 600);
	const Evaluation Initial = Resumed ? Evaluate(Weights, 600) : Baseline;
	const Evaluation InitialCourse = Resumed ? Evaluate(Weights, 600, 1.0f, &Course) : Evaluation();
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
	for (int E = 0; E < EnvironmentCount; ++E)
	{
		Environments[E].TrainingId = E;
		Environments[E].TerrainScale = (EnvironmentCount == 1 ? 1.0f :
			E % 4 == 0 ? 0.0f : E % 4 == 1 ? 0.25f : E % 4 == 2 ? 0.5f : 1.0f) * CourseScale;
		Environments[E].Course = &Course;
	}
	Connection.Send('A');
	struct IterationInfo
	{
		int update = 0, episodes = 0;
		float mean_reward = 0;
		double policy_loss = 0, value_loss = 0, gradient_norm = 0;
	};
	for (;;)
	{
		const Packet Message = Connection.Receive();
		if (Message.Type == 'D') return;
		if (Message.Type == 'E')
			throw std::runtime_error(std::string(Message.Bytes.begin(), Message.Bytes.end()));
		if (Message.Type == 'R' || Message.Type == 'O' || Message.Type == 'S')
		{
			if (Message.Bytes.empty()) throw std::runtime_error("Missing PPO environment index");
			const size_t Index = static_cast<uint8_t>(Message.Bytes[0]);
			if (Index >= size_t(EnvironmentCount)) throw std::runtime_error("Invalid PPO environment index");
			Environment& Env = Environments[Index];
			if (Message.Type == 'R')
			{
				if (Message.Bytes.size() != 1) throw std::runtime_error("Invalid PPO reset packet");
				Env.Reset();
				Connection.Send('r');
			}
			else if (Message.Type == 'O')
			{
				if (Message.Bytes.size() != 1 || !Env.Robot)
					throw std::runtime_error("Invalid PPO observation request");
				const std::vector<float> Observation = Env.Observe();
				if (Observation.size() != 86) throw std::runtime_error("Invalid PPO observation width");
				Connection.Send('o', Observation.data(), uint32_t(Observation.size() * sizeof(float)));
			}
			else
			{
				if (Message.Bytes.size() != 1 + 24 * sizeof(float) || !Env.Robot)
					throw std::runtime_error("Invalid PPO step request");
				std::vector<float> Action(24);
				std::memcpy(Action.data(), Message.Bytes.data() + 1, 24 * sizeof(float));
				const auto Outcome = Env.Step(Action);
				if (OnSnapshot)
				{
					std::array<Transform, SpiderTrainingLinkCount> Links;
					for (size_t Link = 0; Link < Links.size(); ++Link)
						Links[Link] = Env.Robot->GetDynamics().GetLinkTransform(Link);
					OnSnapshot(Index, Links);
				}
				char Result[sizeof(float) + 1];
				std::memcpy(Result, &Outcome.first, sizeof(float));
				Result[sizeof(float)] = Outcome.second ? 1 : 0;
				Connection.Send('s', Result, sizeof(Result));
			}
			continue;
		}
		if (Message.Type == 'U')
		{
			Weights = DecodeWeights(Message, 36);
			IterationInfo Iteration;
			Iteration.update = int(Decode<uint32_t>(Message, 0));
			Iteration.mean_reward = Decode<float>(Message, 4);
			Iteration.policy_loss = Decode<double>(Message, 8);
			Iteration.value_loss = Decode<double>(Message, 16);
			Iteration.gradient_norm = Decode<double>(Message, 24);
			Iteration.episodes = int(Decode<uint32_t>(Message, 32));
			const int Update = int(Iteration.update) - 1;
			if (OnProgress) OnProgress(Iteration.update, Iteration.mean_reward);
			const Evaluation Result = Evaluate(Weights, 200);
			if (Result.Steps == 200 && Result.Last5 > BestIntermediateDistance && Result.Last5 > 0.2f)
			{
				BestIntermediateDistance = Result.Last5;
				ExportPolicy(Weights, Options.policy_path + ".intermediate");
			}
			Evaluation LongRun;
			Evaluation CourseRun;
			if ((Result.Steps == 200 && Result.Last5 > 0.2f) || Update % 10 == 9)
			{
				LongRun = Evaluate(Weights, 600);
				if (LongRun.Steps == 600 && LongRun.Last5 >= 2.0f && LongRun.Distance > BestCandidateDistance)
				{
					BestCandidateDistance = LongRun.Distance;
					ExportPolicy(Weights, Options.policy_path + ".candidate");
				}
				if (IsAligned(LongRun) && LongRun.Distance > BestAlignedDistance)
				{
					BestAlignedDistance = LongRun.Distance;
					ExportPolicy(Weights, Options.policy_path + ".aligned");
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
					CourseRun = Evaluate(Weights, 600, 1.0f, &Course);
				const float NearScore = AcceptanceScore(LongRun, CourseRun);
				if (NearScore > BestNearScore)
				{
					BestNearScore = NearScore;
					ExportPolicy(Weights, Options.policy_path + ".near");
				}
				if (CourseRun.Steps == 600 && CourseRun.Distance > BestCourseDistance)
				{
					BestCourseDistance = CourseRun.Distance;
					ExportPolicy(Weights, Options.policy_path + ".course_candidate");
				}
				if (FlatReady && CourseRun.Steps == 600 && CourseRun.Distance >= 22.5f &&
					CourseRun.Last5 >= 2.0f && CourseRun.FinalFacing >= 0.75f &&
					std::abs(CourseRun.Lateral) <= 2.0f && CourseRun.Grounded >= 1401 &&
					CourseRun.MinHeight > 0.35f && CourseRun.MinUp > 0.9f &&
					LongRun.Distance > BestDistance)
				{
					BestDistance = LongRun.Distance;
					ExportPolicy(Weights, Options.policy_path);
				}
			}
			ExportPolicy(Weights, LatestPath);
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
			Connection.Send('u');
			continue;
		}
		throw std::runtime_error("Unknown Elman training TCP packet");
	}
}

Riemann::SpiderEvaluation Riemann::EvaluateSpiderPolicy(const std::string& PolicyPath, int Steps, float CourseScale)
{
	if (Steps < 1 || CourseScale < 0 || CourseScale > 1)
		throw std::invalid_argument("Invalid spider policy evaluation settings");
	const CourseMap Course = CourseScale > 0 ? LoadCourseMap("Contents/Robots/Spider/course.height") : CourseMap{};
	const std::vector<float> Weights = ImportPolicy(PolicyPath);
	if (Weights.empty()) throw std::runtime_error("Spider policy file is missing");
	return Evaluate(Weights, Steps, CourseScale, CourseScale > 0 ? &Course : nullptr);
}
