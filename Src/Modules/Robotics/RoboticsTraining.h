#pragma once

#include "../../Maths/Transform.h"
#include <array>
#include <atomic>
#include <functional>
#include <string>

namespace Riemann {

constexpr size_t RoboticsTrainingEnvironmentCount = 8;
constexpr size_t RoboticsTrainingLinkCount = 25;

struct RoboticsTrainingSnapshot {
    std::array<std::array<Transform, RoboticsTrainingLinkCount>, RoboticsTrainingEnvironmentCount> links{};
    std::array<bool, RoboticsTrainingEnvironmentCount> ready{};
    unsigned int update = 0;
};

struct RoboticsTrainingOptions {
    std::string executable_path;
    std::string policy_path;
    std::string course_path;
    int environment_count = 1;
    int updates = 1000;
    int rollout_actions = 1024;
    float course_scale = 0;
};

struct RoboticsEvaluation {
    float Distance = 0, First5 = 0, Last5 = 0, MinHeight = 100, MinUp = 1;
    float MeanActionOffset = 0, MeanActionChange = 0, FinalFacing = 1, Lateral = 0;
    std::array<float, 6> Segments{};
    int Steps = 0, Grounded = 0;
};

using RoboticsSnapshotCallback = std::function<void(size_t, const std::array<Transform, RoboticsTrainingLinkCount>&)>;
using RoboticsProgressCallback = std::function<void(unsigned int, float)>;

void RunRoboticsTraining(const RoboticsTrainingOptions& options, const std::atomic<bool>& stop,
                       const RoboticsSnapshotCallback& on_snapshot,
                       const RoboticsProgressCallback& on_progress);
RoboticsEvaluation EvaluateRoboticsPolicy(const std::string& policy_path, int steps, float course_scale);

} // namespace Riemann
