#pragma once
#include <chrono>
#include <thread>

namespace utils {

class Rate {
 public:
  using clock = std::chrono::high_resolution_clock;
  using time_point = clock::time_point;
  using nanoseconds = std::chrono::nanoseconds;
  explicit Rate(const int &freq)
      : event_time_(clock::now()), cycle_(int(1e9) / freq) {}
  void sync() { event_time_ = clock::now(); };
  void sleep() {
    event_time_ += cycle_;
    std::this_thread::sleep_until(event_time_);
  };

 private:
  nanoseconds cycle_;
  time_point event_time_;
};

class Logger {
 public:
  Logger() {}
};
}  // namespace utils
