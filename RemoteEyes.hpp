#pragma once

// C++ headers
#include <string>
#include <stdexcept>
#include <functional>
#include <chrono>

// C library headers
#include <stdio.h>
#include <string.h>

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

#include <fmt/format.h>

const char* PI_PICO_SERIAL_DEVICE = "/dev/ttyACM0";
const speed_t PI_PICO_SERIAL_BAUD_RATE = B115200;

class SerialDevice
{
private:
  std::string devicePath;
  int fd = -1;
  char readBuf[256];
  std::stringstream line;

public:
  // Is always [baud]-1-N style connection
  SerialDevice(const std::string& devicePath, speed_t baudRate)
  {
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
    ::write(fd, data.data(), data.size());
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

class RemoteEyes
{
public:
  RemoteEyes() : serial(PI_PICO_SERIAL_DEVICE, PI_PICO_SERIAL_BAUD_RATE)
  {
    serial.write("echo 0\n");
    serial.write("en 0\n");
  }

  void look(float yaw, float pitch)
  {
    serial.write(fmt::format("dir {} {}\n", yaw, pitch));
  };
private:
  SerialDevice serial;

};