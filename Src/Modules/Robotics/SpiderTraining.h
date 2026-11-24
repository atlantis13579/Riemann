#pragma once

#include "../../Maths/Transform.h"
#include <array>
#include <atomic>
#include <functional>
#include <string>

namespace Riemann {

constexpr size_t SpiderTrainingEnvironmentCount = 8;
constexpr size_t SpiderTrainingLinkCount = 25;

struct SpiderTrainingSnapshot {
    std::array<std::array<Transform, SpiderTrainingLinkCount>, SpiderTrainingEnvironmentCount> links{};
    std::array<bool, SpiderTrainingEnvironmentCount> ready{};
    unsigned int update = 0;
};

struct SpiderTrainingOptions {
    std::string executable_path;
    std::string policy_path;
    std::string course_path;
    int environment_count = 1;
    int updates = 1000;
    int rollout_actions = 1024;
    float course_scale = 0;
};

struct SpiderEvaluation {
    float Distance = 0, First5 = 0, Last5 = 0, MinHeight = 100, MinUp = 1;
    float MeanActionOffset = 0, MeanActionChange = 0, FinalFacing = 1, Lateral = 0;
    std::array<float, 6> Segments{};
    int Steps = 0, Grounded = 0;
};

using SpiderSnapshotCallback = std::function<void(size_t, const std::array<Transform, SpiderTrainingLinkCount>&)>;
using SpiderProgressCallback = std::function<void(unsigned int, float)>;

void RunSpiderTraining(const SpiderTrainingOptions& options, const std::atomic<bool>& stop,
                       const SpiderSnapshotCallback& on_snapshot,
                       const SpiderProgressCallback& on_progress);
SpiderEvaluation EvaluateSpiderPolicy(const std::string& policy_path, int steps, float course_scale);

} // namespace Riemann
