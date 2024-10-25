#pragma once

#include <string>
#include <chrono>
#include <iostream>

#ifdef DEBUG_PROFILE_FUNCTIONS
#define PROFILE_FUNCTION FunctionTimer funcprof(__func__)
#else
#define PROFILE_FUNCTION do { } while(0)
#endif

class FunctionTimer
{
public:
  FunctionTimer(std::string_view functionName) :
    start_ {std::chrono::steady_clock::now()},
    functionName_ {functionName} {}
  ~FunctionTimer()
  {
    std::cout << functionName_ << " execution time: " << msElapsed() << " ms\n";
  }
  float msElapsed()
  {
    return std::chrono::duration_cast<std::chrono::duration<float>>(std::chrono::steady_clock::now() - start_).count() * 1000.0f;
  }
private:
  std::chrono::steady_clock::time_point start_;
  std::string functionName_;
};