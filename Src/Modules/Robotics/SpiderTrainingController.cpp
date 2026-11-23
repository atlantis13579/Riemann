#include "SpiderTrainingController.h"
#include <utility>

namespace Riemann {

SpiderTrainingController::~SpiderTrainingController() { stop(); }

void SpiderTrainingController::start(SpiderTrainingOptions options) {
    if (running_.load()) return;
    if (worker_.joinable()) worker_.join();
    stop_.store(false);
    running_.store(true);
    {
        std::lock_guard<std::mutex> lock(mutex_);
        snapshot_ = SpiderTrainingSnapshot{};
        status_ = "Training";
    }
    worker_ = std::thread([this, options = std::move(options)] {
        try {
            RunSpiderTraining(options, stop_,
                [this](size_t index, const std::array<Transform, SpiderTrainingLinkCount>& links) {
                    if (!display_active_.load()) return;
                    std::lock_guard<std::mutex> lock(mutex_);
                    snapshot_.links[index] = links;
                    snapshot_.ready[index] = true;
                },
                [this](unsigned int update, float reward) {
                    std::lock_guard<std::mutex> lock(mutex_);
                    snapshot_.update = update;
                    status_ = "Update " + std::to_string(update) + ", reward " + std::to_string(reward);
                });
            std::lock_guard<std::mutex> lock(mutex_);
            status_ = stop_.load() ? "Stopped" : "Completed";
        } catch (const std::exception& error) {
            std::lock_guard<std::mutex> lock(mutex_);
            status_ = std::string("Training failed: ") + error.what();
        }
        running_.store(false);
    });
}

void SpiderTrainingController::stop() {
    stop_.store(true);
    if (worker_.joinable()) worker_.join();
    running_.store(false);
}

SpiderTrainingSnapshot SpiderTrainingController::snapshot() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return snapshot_;
}

std::string SpiderTrainingController::status() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return status_;
}

} // namespace Riemann
