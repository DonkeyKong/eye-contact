#include "V4LCamera.hpp"
#include "FunctionTimer.hpp"

#include <regex>
#include <chrono>
#include <filesystem>
#include <map>

#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <string.h> //memset


namespace fs = std::filesystem;
using namespace std::chrono_literals;
using namespace V4L2;

// Wrapper around ioctl calls
static int xioctl(int fd_, int request, void* arg) 
{
  int r;

  do 
  {
    r = ioctl(fd_, request, arg);
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

static void printV4L2Info(const v4l2_format& format)
{
  if (format.type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
  {
    std::cout << "Unsupported format type\n";
    return;
  }
  
  std::string_view formatCode((char*)&format.fmt.pix.pixelformat, 4);
  std::cout << formatCode << " " << format.fmt.pix.width << "x" << format.fmt.pix.height << "\n";
}

static void printV4L2Info(const v4l2_frmsizeenum& format)
{
  std::string_view formatCode((char*)&format.pixel_format, 4);
  if (format.type == V4L2_FRMSIZE_TYPE_DISCRETE)
  {
    std::cout << formatCode << " " << format.discrete.width << "x" << format.discrete.height << " (fixed)\n";
  }
  else if (format.type == V4L2_FRMSIZE_TYPE_STEPWISE)
  {
    std::cout << formatCode << " " << format.stepwise.min_width << "x" << format.stepwise.min_height
              << " to " << format.stepwise.max_width << "x" << format.stepwise.max_height 
              << " (" << format.stepwise.step_width << " , " << format.stepwise.step_height << " step)" << "\n";
  }
  else if (format.type == V4L2_FRMSIZE_TYPE_CONTINUOUS)
  {
    std::cout << formatCode << " " << format.stepwise.min_width << "x" << format.stepwise.min_height
          << " to " << format.stepwise.max_width << "x" << format.stepwise.max_height 
          << " (continuous)" << "\n";
  }
}

static void printV4L2Info(const v4l2_frmivalenum& interval)
{
  if (interval.type == V4L2_FRMIVAL_TYPE_DISCRETE)
  {
    std::cout << "\t" << (float)interval.discrete.denominator / (float)interval.discrete.numerator << " FPS (fixed)\n";
  }
  else if (interval.type == V4L2_FRMIVAL_TYPE_STEPWISE)
  {
    std::cout << "\t" << (float)interval.stepwise.min.denominator / (float)interval.stepwise.min.numerator << " - "
              << (float)interval.stepwise.max.denominator / (float)interval.stepwise.max.numerator << " FPS ( " 
              << (float)interval.stepwise.step.denominator / (float)interval.stepwise.step.numerator << " step)\n";
  }
  else if (interval.type == V4L2_FRMIVAL_TYPE_CONTINUOUS)
  {
    std::cout << "\t" << (float)interval.stepwise.min.denominator / (float)interval.stepwise.min.numerator << " - "
              << (float)interval.stepwise.max.denominator / (float)interval.stepwise.max.numerator << " FPS (continuous)\n";
  }
}

Camera::Camera(std::string devicePath, int requestedWidth, int requestedHeight, size_t bufferCount) :  devicePath_(std::move(devicePath))
{
  // Open the device file
  fd_ = open(devicePath_.c_str(), O_RDWR);
  if (fd_ < 0) 
  {
    throw std::runtime_error("Cannot create camera: file descriptor did not open");
  }

  // Get the formats and resolutions supported...
  std::cout << "Available video formats:\n";
  v4l2_fmtdesc fmtdesc = 
  {
    index: 0,
    type: V4L2_BUF_TYPE_VIDEO_CAPTURE
  };
  while(0 == xioctl(fd_, VIDIOC_ENUM_FMT, &fmtdesc)) 
  {
    v4l2_frmsizeenum frmsize 
    {
      index: 0,
      pixel_format: fmtdesc.pixelformat
    };
    while (ioctl(fd_, VIDIOC_ENUM_FRAMESIZES, &frmsize) >= 0) 
    {
      printV4L2Info(frmsize);
      frmsize.index++;

      v4l2_frmivalenum frmival
      {
        index : 0,		/* Frame format index */
        pixel_format: frmsize.pixel_format, /* Pixel format */
        width: frmsize.type == V4L2_FRMSIZE_TYPE_DISCRETE ? frmsize.discrete.width : frmsize.stepwise.max_width,
        height: frmsize.type == V4L2_FRMSIZE_TYPE_DISCRETE ? frmsize.discrete.height : frmsize.stepwise.max_height
      };
      while (ioctl(fd_, VIDIOC_ENUM_FRAMEINTERVALS, &frmival) >= 0) 
      {
        printV4L2Info(frmival);
        frmival.index++;
      }
    }
    fmtdesc.index++;
  }

  v4l2_format fmt = 
  {
    type: V4L2_BUF_TYPE_VIDEO_CAPTURE,
    fmt: {
      pix: {
        width: (uint32_t)requestedWidth,
        height: (uint32_t)requestedHeight,
        pixelformat: fmtdesc.pixelformat,
        field: V4L2_FIELD_NONE,
      }
    }
  };

  if (-1 == xioctl(fd_, VIDIOC_S_FMT, &fmt)) 
  {
    perror("VIDIOC_S_FMT");
    throw std::runtime_error("Could not get format.");
  }

  std::cout << "Using format: ";
  printV4L2Info(fmt);

  width_ = fmt.fmt.pix.width;
  height_ = fmt.fmt.pix.height;
  strideInBytes_ = fmt.fmt.pix.bytesperline;
  format_ = std::string_view((char*)&fmt.fmt.pix.pixelformat, 4);

  v4l2_requestbuffers reqbuf = {};
  reqbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  reqbuf.memory = V4L2_MEMORY_MMAP;
  reqbuf.count = (uint32_t) bufferCount;

  if (-1 == ioctl(fd_, VIDIOC_REQBUFS, &reqbuf)) 
  {
    if (errno == EINVAL)
      printf("Video capturing or mmap-streaming is not supported\\n");
    else
      perror("VIDIOC_REQBUFS");
    throw std::runtime_error("Cannot create camera: video streaming not supported");
  }

  // Verify that we have at least five buffers_
  if (reqbuf.count < bufferCount)
  {
    printf("Not enough buffer memory\n");
    throw std::runtime_error("Cannot create camera: could not create buffers_");
  }

  buffers_.resize(reqbuf.count);
  for (int i = 0; i < reqbuf.count; i++) 
  {
    // Prepare the buffer variable for the next buffer
    v4l2_buffer buffer = {};
    buffer.type = reqbuf.type;
    buffer.memory = V4L2_MEMORY_MMAP;
    buffer.index = i;

    // Query the buffer for its information
    if (-1 == ioctl(fd_, VIDIOC_QUERYBUF, &buffer)) 
    {
      perror("VIDIOC_QUERYBUF");
      throw std::runtime_error("Cannot create camera: could not query buffers_");
    }

    buffers_[i].length = buffer.length; // Remember the buffer length for unmapping later
    buffers_[i].start = (uint8_t*)mmap(
      NULL,
      buffer.length,
      PROT_READ | PROT_WRITE,
      MAP_SHARED,
      fd_,
      buffer.m.offset
    );

    if (MAP_FAILED == buffers_[i].start) 
    {
      perror("mmap");
      throw std::runtime_error("Cannot create camera: memory mapping failed");
    }

    // Enqueue the buffer with VIDIOC_QBUF
    if (-1 == xioctl(fd_, VIDIOC_QBUF, &buffer)) 
    {
      perror("VIDIOC_QBUF");
      throw std::runtime_error("Cannot create camera: buffer enqueue failed");
    }
  }

  v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (-1 == xioctl(fd_, VIDIOC_STREAMON, &type)) 
  {
    perror("VIDIOC_STREAMON");
    throw std::runtime_error("Cannot create camera: stream start failed");
  }
}

Camera::~Camera()
{
  // Stop streaming
  enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (-1 == xioctl(fd_, VIDIOC_STREAMOFF, &type)) 
  {
    perror("VIDIOC_STREAMOFF");
  }

  // Unmap all the buffers
  for (size_t i = 0; i < buffers_.size(); i++)
  {
    munmap(buffers_[i].start, buffers_[i].length);
  }

  close(fd_);
}

Frame Camera::getFrame()
{
  PROFILE_FUNCTION;
  
  v4l2_buffer buffer = {};
  buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  buffer.memory = V4L2_MEMORY_MMAP;

  // Dequeue a buffer
  auto timeoutTime = std::chrono::steady_clock::now() + 50ms;

  while (true)
  {
    if (-1 == xioctl(fd_, VIDIOC_DQBUF, &buffer)) 
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
  return {fd_, buffer, buffers_[buffer.index], width_, height_, strideInBytes_, format_};
}

int Camera::width() const
{
  return width_;
}

int Camera::height() const
{
  return height_;
}