#pragma once

#include "SpiderTraining.h"
#include <atomic>
#include <mutex>
#include <string>
#include <thread>

namespace Riemann {

class SpiderTrainingController {
public:
    ~SpiderTrainingController();
    void start(SpiderTrainingOptions options);
    void stop();
    void set_display_active(bool active) { display_active_.store(active); }
    bool running() const { return running_.load(); }
    SpiderTrainingSnapshot snapshot() const;
    std::string status() const;

private:
    std::thread worker_;
    std::atomic<bool> stop_{false}, running_{false}, display_active_{true};
    mutable std::mutex mutex_;
    SpiderTrainingSnapshot snapshot_;
    std::string status_ = "Idle";
};

} // namespace Riemann
