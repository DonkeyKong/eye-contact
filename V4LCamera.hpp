#pragma once

#include <string>
#include <vector>
#include <iostream>

#include <linux/videodev2.h>

namespace V4L2
{

struct MappedBuffer
{
  size_t length = 0;
  uint8_t* start = nullptr;
};

class Frame
{
public:
  Frame(int fd, v4l2_buffer buffer, const MappedBuffer& mBuf, int width, int height, int stride, std::string format);
  // no copy but move ok
  Frame(const Frame&) = delete;
  Frame(Frame&&);
  ~Frame();
  const MappedBuffer data;
  const int width;
  const int height;
  const int strideInBytes;
  const std::string format;
private:
  v4l2_buffer buffer;
  int fd;
};

class Camera
{
public:
  //static std::vector<V4LCamInfo> Devices();

  Camera(std::string devicePath, int requestedWidth = 640, int requestedHeight = 480, size_t bufferCount = 2);
  ~Camera();

  // No copy no move
  Camera(const Camera&) = delete;
  Camera(Camera&&) = delete;

  Frame getFrame();
private:
  std::string devicePath;
  int fd;
  std::vector<MappedBuffer> buffers;

  // Image format information
  int width = 0;
  int height = 0;
  int strideInBytes = 0;
  std::string format = "UKWN";
};

}