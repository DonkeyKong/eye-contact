#pragma once

#include <vector>
#include <chrono>

class FPSCounter
{
public:
  FPSCounter(int framesToAverage = 10)
  {
    frameTimes_.resize(framesToAverage);
  }
  void frame()
  {
    frameTimes_[next_] = std::chrono::steady_clock::now();
    last_ = next_;
    next_ = (next_ + 1) % frameTimes_.size();
    if (next_ == 0) minFrameCount_ = true;
  }
  float fps()
  {
    if (!minFrameCount_) return 0;
    float seconds = std::chrono::duration_cast<std::chrono::duration<float>>(frameTimes_[last_] - frameTimes_[next_]).count();
    return (float)frameTimes_.size() / seconds;
  }
private:
  std::vector<std::chrono::steady_clock::time_point> frameTimes_;
  int next_ = 0;
  int last_ = 0;
  bool minFrameCount_ = false;
};