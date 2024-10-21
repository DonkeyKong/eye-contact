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
  Camera(std::string devicePath, int requestedWidth = 640, int requestedHeight = 480, size_t bufferCount = 2);
  ~Camera();

  // No copy no move
  Camera(const Camera&) = delete;
  Camera(Camera&&) = delete;

  Frame getFrame();

  int width() const;
  int height() const;

private:
  std::string devicePath_;
  int fd_;
  std::vector<MappedBuffer> buffers_;

  // Image format information
  int width_ = 0;
  int height_ = 0;
  int strideInBytes_ = 0;
  std::string format_ = "UKWN";
};

}