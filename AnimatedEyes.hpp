#pragma once

// C++ headers
#include <string>
#include <stdexcept>
#include <functional>
#include <chrono>
#include <thread>
#include <mutex>
#include <atomic>

// C library headers
#include <stdio.h>
#include <string.h>

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

#include <fmt/format.h>

using namespace std::chrono_literals;

const char* PI_PICO_SERIAL_DEVICE = "/dev/ttyACM0";
const speed_t PI_PICO_SERIAL_BAUD_RATE = B115200;

class SerialDevice
{
private:
  std::string devicePath;
  int fd = -1;
  // char readBuf[256];
  // std::stringstream line;

public:
  // Is always [baud]-1-N style connection
  SerialDevice(const std::string& devicePath, speed_t baudRate)
  {
  #ifdef DEBUG_DISABLE_SERIAL
    return;
  #endif
  
    // Thanks to this tutorial for greatly speeding up my implementation
    // https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/

    fd = open(devicePath.c_str(), O_RDWR);

    // Check for errors
    if (fd < 0) 
    {
      throw std::runtime_error("Cannot open serial port: file descriptor did not open");
    }

    // Create new termios struct, we call it 'tty' for convention
    // No need for "= {0}" at the end as we'll immediately write the existing
    // config to this struct
    struct termios tty;

    // Read in existing settings, and handle any error
    // NOTE: This is important! POSIX states that the struct passed to tcsetattr()
    // must have been initialized with a call to tcgetattr() overwise behaviour
    // is undefined
    if(tcgetattr(fd, &tty) != 0) 
    {
      throw std::runtime_error("Cannot get serial port config!");
    }

    tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
    tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
    tty.c_cflag &= ~CSIZE; // Clear all the size bits, then use one of the statements below
    tty.c_cflag |= CS8; // 8 bits per byte (most common)
    tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
    tty.c_cflag |= CREAD | CLOCAL; // enable read, disable modem behavior
    
    tty.c_lflag &= ~ICANON; // disable line by line processing
    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
    // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT IN LINUX)
    // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT IN LINUX)

    tty.c_cc[VTIME] = 0; // reads are non-blocking and always return immediately
    tty.c_cc[VMIN] = 0;

    cfsetspeed(&tty, baudRate);

    // Save tty settings, also checking for error
    if (tcsetattr(fd, TCSANOW, &tty) != 0) 
    {
      throw std::runtime_error("Cannot set serial port config!");
    }
  }

  ~SerialDevice()
  {
    if (fd >= 0)
    {
      close(fd);
    }
  }

  void write(const std::string_view& data)
  {
    if (fd >= 0)
    {
      ::write(fd, data.data(), data.size());
    }
  }

  // std::string readLine(std::chrono::milliseconds timeout)
  // {
  //   return readLine(std::chrono::steady_clock::now() + timeout);
  // }

  // template <typename UnaryPred>
  // bool readUntil(UnaryPred predicate, std::chrono::milliseconds timeout)
  // {
  //   auto timeoutTime = std::chrono::steady_clock::now() + timeout;
  //   do
  //   {
  //     std::string line = readLine();
  //     if (predicate(line))
  //     {
  //       return true;
  //     }
  //   } while (std::chrono::steady_clock::now() < timeoutTime);
  //   return false;
  // }

  // private:
  //   std::string readLine(std::chrono::steady_clock::time_point timeoutTime)
  //   {
  //     do
  //     {
  //       int n = read(fd, readBuf, 256);
  //       for (int i=0; i < n; ++i)
  //       {
  //         if (readBuf[i] == '\n')
  //         {
            
  //         }
  //       }
  //     } while (std::chrono::steady_clock::now() < timeoutTime);
   //}
};



class AnimatedEyes
{
  enum class EyeState
  {
    Sleep,
    Search,
    Track
  };

public:
  AnimatedEyes(const char* serialDevice = PI_PICO_SERIAL_DEVICE, speed_t baudRate = PI_PICO_SERIAL_BAUD_RATE) 
    : serial_(serialDevice, baudRate)
    , faceYaw_{0}, facePitch_{0}, detectedFace_{false}
    , run_{true}
  {
    serial_.write("echo 0\n"); // turn off echo to speed up comms
    serial_.write("en 0\n"); // turn off nunchuck input
    thread_ = std::make_unique<std::thread>(&AnimatedEyes::animate, this);
  }

  ~AnimatedEyes()
  {
    if (thread_ && run_)
    {
      run_ = false;
      thread_->join();
    }
  }

  void registerFace(float yaw, float pitch)
  {
    std::lock_guard lock(mutex_);
    faceYaw_ = yaw;
    facePitch_ = pitch;
    detectedFace_ = true;
  };

private:

  static void moveTowardsExp(float& yaw, float& pitch, const float& destYaw, const float& destPitch, float rate = 0.1f, float epsilon = 1.0f)
  {
    yaw = yaw * (1.0f - rate) + destYaw * rate;
    pitch = pitch * (1.0f - rate) + destPitch * rate;
    if (std::abs(yaw-destYaw) < epsilon)
    {
      yaw = destYaw;
    }
    if (std::abs(pitch-destPitch) < epsilon)
    {
      pitch = destPitch;
    }
  }

  static void moveTowardsLinear(float& yaw, float& pitch, const float& destYaw, const float& destPitch, float rate = 4.0f)
  {
    if (std::abs(yaw-destYaw) < rate)
    {
      yaw = destYaw;
    }
    else if (yaw > destYaw)
    {
      yaw -= rate;
    }
    else if (yaw < destYaw)
    {
      yaw += rate;
    }

    if (std::abs(pitch-destPitch) < rate)
    {
      pitch = destPitch;
    }
    else if (pitch > destPitch)
    {
      pitch -= rate;
    }
    else if (pitch < destPitch)
    {
      pitch += rate;
    }
  }

  void animate()
  {
    const std::chrono::milliseconds blinkTime = 200ms;
    const std::chrono::milliseconds meanInterBlinkTime = 12000ms;
    const std::chrono::milliseconds sleepTime = 4000ms;

    bool detectedFace = false;
    float destYaw = 0, destPitch = 0, yaw = 0, pitch = 0, facePitch = 0, faceYaw = 0;
    EyeState state = EyeState::Sleep;
    bool blinking = false;
    std::chrono::steady_clock::time_point faceLastSeenTime;
    std::chrono::steady_clock::time_point endBlinkTime;
    
    serial_.write("style 5\n"); // turn the eyes off initially
    serial_.write("dir 0 0\n"); // look straight ahead
    serial_.write("play 1\n"); // Go into the blinking state and hold there

    while (run_)
    {
      auto now = std::chrono::steady_clock::now();

      // Update the local state with data from the main thread
      {
        std::lock_guard lock(mutex_);
        if (detectedFace_)
        {
          detectedFace = true;
          detectedFace_ = false;
          faceYaw = faceYaw_;
          facePitch = facePitch_;
          faceLastSeenTime = now;
        }
        else
        {
          detectedFace = false;
        }
      }

      if (state == EyeState::Sleep)
      {
        if (detectedFace)
        {
          state = EyeState::Track;
          serial_.write("style 1\n"); // show red eyes
          serial_.write("stop\n");  // unblink
          endBlinkTime = now;
          blinking = false;
        }
        else
        {
          // Move slowly towards dest, then shut off lights when there
          moveTowardsLinear(yaw, pitch, destYaw, destPitch);

          // Shut off lights if at dest
          if (!blinking && destYaw == yaw && destPitch == pitch)
          {
            serial_.write("style 5\n");
            serial_.write("play 1\n");
            blinking = true;
          }
        }
      }
      
      if (state == EyeState::Track)
      {
        if (now - faceLastSeenTime > sleepTime)
        {
          state = EyeState::Sleep;
          destYaw = 0; // move gaze towards center
          destPitch = 0;
          blinking = false;
          serial_.write("stop\n"); // abort blinking
        }
        else
        {
          destPitch = facePitch;
          destYaw = faceYaw;

          // Move quickly towards dest
          moveTowardsExp(yaw, pitch, destYaw, destPitch, 0.5f);
          
          // Blink periodically
          if (now - endBlinkTime > meanInterBlinkTime)
          {
            serial_.write("play 1\n");
            endBlinkTime = now + blinkTime;
            blinking = true;
          }
          else if (blinking && now > endBlinkTime)
          {
            serial_.write("stop\n");
            blinking = false;
          }
        }
      }

      // If in awake state, follow the face
      serial_.write(fmt::format("dir {} {}\n", yaw, pitch));

      std::this_thread::sleep_until(now + 10ms);
    }
  }

  SerialDevice serial_;
  float faceYaw_, facePitch_;
  bool detectedFace_;
  std::atomic_bool run_;
  std::mutex mutex_;
  std::unique_ptr<std::thread> thread_;
};