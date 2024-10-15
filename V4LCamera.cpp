#include "V4LCamera.hpp"

#include <regex>
#include <chrono>
#include <filesystem>

#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <string.h> //memset


namespace fs = std::filesystem;
using namespace std::chrono_literals;
using namespace V4L2;

static constexpr std::string_view V4LDevPath = "/dev/v4l/by-id";

// Wrapper around ioctl calls
static int xioctl(int fd, int request, void* arg) 
{
  int r;

  do 
  {
    r = ioctl(fd, request, arg);
  } while (-1 == r && EINTR == errno);

  return r;
}

Frame::Frame(int fd, v4l2_buffer buffer, const MappedBuffer& mBuf, int width, int height, int stride, std::string format) 
  : fd(fd)
  , buffer(buffer)
  , data(mBuf)
  , width(width)
  , height(height)
  , strideInBytes(stride)
  , format(format)
{
}

Frame::Frame(Frame&& other)
  : fd(other.fd)
  , buffer(other.buffer)
  , data(other.data)
  , width(other.width)
  , height(other.height)
  , strideInBytes(other.strideInBytes)
  , format(std::move(other.format))
{
  // Invalidate other where possible
  other.buffer = {};
  other.fd = -1;
}

Frame::~Frame()
{
  if (buffer.length > 0 && fd != -1)
  {
    // Enqueue the buffer again
    if (-1 == xioctl(fd, VIDIOC_QBUF, &buffer)) 
    {
      perror("VIDIOC_QBUF");
    }
  }
}

Camera::Camera(std::string devicePath_, int requestedWidth, int requestedHeight, size_t bufferCount) :  devicePath(std::move(devicePath_))
{
  // Open the device file
  fd = open(devicePath.c_str(), O_RDWR);
  if (fd < 0) 
  {
    throw std::runtime_error("Cannot create camera: file descriptor did not open");
  }

  v4l2_fmtdesc fmtdesc = {0};
  fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

  // Get the format with the largest index and use it
  std::cout << "Formats available: ";
  while(0 == xioctl(fd, VIDIOC_ENUM_FMT, &fmtdesc)) 
  {
    fmtdesc.index++;
    
    std::string_view formatCode((char*)&fmtdesc.pixelformat, 4);
    std::cout << formatCode << " ";
  }
  printf("\nUsing format: %s\n", fmtdesc.description);
  
  v4l2_format fmt = {};
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.fmt.pix.width = requestedWidth;
  fmt.fmt.pix.height = requestedHeight;
  fmt.fmt.pix.pixelformat = fmtdesc.pixelformat;
  fmt.fmt.pix.field = V4L2_FIELD_NONE;

  if (-1 == xioctl(fd, VIDIOC_S_FMT, &fmt)) 
  {
    perror("VIDIOC_S_FMT");
    throw std::runtime_error("Could not get format.");
  }

  width = fmt.fmt.pix.width;
  height = fmt.fmt.pix.height;
  strideInBytes = fmt.fmt.pix.bytesperline;
  format = std::string_view((char*)&fmt.fmt.pix.pixelformat, 4);

  printf(
    "Set format:\n"
    " Width: %d\n"
    " Height: %d\n"
    " Pixel format: %s\n"
    " Field: %d\n",
    width,
    height,
    format.c_str(),
    fmt.fmt.pix.field
  );

  v4l2_requestbuffers reqbuf = {};
  reqbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  reqbuf.memory = V4L2_MEMORY_MMAP;
  reqbuf.count = (uint32_t) bufferCount;

  if (-1 == ioctl(fd, VIDIOC_REQBUFS, &reqbuf)) 
  {
    if (errno == EINVAL)
      printf("Video capturing or mmap-streaming is not supported\\n");
    else
      perror("VIDIOC_REQBUFS");
    throw std::runtime_error("Cannot create camera: video streaming not supported");
  }

  // Verify that we have at least five buffers
  if (reqbuf.count < bufferCount)
  {
    printf("Not enough buffer memory\n");
    throw std::runtime_error("Cannot create camera: could not create buffers");
  }

  buffers.resize(reqbuf.count);
  for (int i = 0; i < reqbuf.count; i++) 
  {
    // Prepare the buffer variable for the next buffer
    v4l2_buffer buffer = {};
    buffer.type = reqbuf.type;
    buffer.memory = V4L2_MEMORY_MMAP;
    buffer.index = i;

    // Query the buffer for its information
    if (-1 == ioctl(fd, VIDIOC_QUERYBUF, &buffer)) 
    {
      perror("VIDIOC_QUERYBUF");
      throw std::runtime_error("Cannot create camera: could not query buffers");
    }

    buffers[i].length = buffer.length; // Remember the buffer length for unmapping later
    buffers[i].start = (uint8_t*)mmap(
      NULL,
      buffer.length,
      PROT_READ | PROT_WRITE,
      MAP_SHARED,
      fd,
      buffer.m.offset
    );

    if (MAP_FAILED == buffers[i].start) 
    {
      perror("mmap");
      throw std::runtime_error("Cannot create camera: memory mapping failed");
    }

    // Enqueue the buffer with VIDIOC_QBUF
    if (-1 == xioctl(fd, VIDIOC_QBUF, &buffer)) 
    {
      perror("VIDIOC_QBUF");
      throw std::runtime_error("Cannot create camera: buffer enqueue failed");
    }
  }

  v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (-1 == xioctl(fd, VIDIOC_STREAMON, &type)) 
  {
    perror("VIDIOC_STREAMON");
    throw std::runtime_error("Cannot create camera: stream start failed");
  }
}

Camera::~Camera()
{
  // Stop streaming
  enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (-1 == xioctl(fd, VIDIOC_STREAMOFF, &type)) 
  {
    perror("VIDIOC_STREAMOFF");
  }

  // Unmap all the buffers
  for (size_t i = 0; i < buffers.size(); i++)
  {
    munmap(buffers[i].start, buffers[i].length);
  }

  close(fd);
}

Frame Camera::getFrame()
{
  v4l2_buffer buffer = {};
  buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  buffer.memory = V4L2_MEMORY_MMAP;

  // Dequeue a buffer
  auto timeoutTime = std::chrono::steady_clock::now() + 50ms;

  while (true)
  {
    if (-1 == xioctl(fd, VIDIOC_DQBUF, &buffer)) 
    {
      switch(errno) 
      {
      case EAGAIN:
        if (std::chrono::steady_clock::now() >= timeoutTime)
        {
          throw std::runtime_error("getFrame timed out!");
        }
        break;
      case EIO:
      default:
        perror("VIDIOC_DQBUF");
        throw std::runtime_error("getFrame encountered an IO error on dequeue!");
      }
    }
    else
    {
      break; // break out of the loop, we got a buffer!
    }
  }
  // assert(buffer.index < num_buffers);
  return {fd, buffer, buffers[buffer.index], width, height, strideInBytes, format};
}