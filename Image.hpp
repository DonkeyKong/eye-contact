#pragma once

#include "Color.hpp"
#include "BoundingBox.hpp"

#include <base_resample.h>

#include <vector>
#include <string>
#include <memory>

enum class ScaleMode
{
  Stretch,
  Fit,
  Fill
};

enum class InterpolationMode
{
  Auto = -1,
  Nearest  = (int) base::KernelTypeNearest,
  Average  = (int) base::KernelTypeAverage,
  Bilinear = (int) base::KernelTypeBilinear,
  Bicubic  = (int) base::KernelTypeBicubic,
  Mitchell = (int) base::KernelTypeMitchell,
  Cardinal = (int) base::KernelTypeCardinal,
  BSpline  = (int) base::KernelTypeBSpline,
  Lanczos  = (int) base::KernelTypeLanczos,
  Lanczos2 = (int) base::KernelTypeLanczos2,
  Lanczos3 = (int) base::KernelTypeLanczos3,
  Lanczos4 = (int) base::KernelTypeLanczos4,
  Lanczos5 = (int) base::KernelTypeLanczos5,
  Catmull  = (int) base::KernelTypeCatmull,
  Gaussian = (int) base::KernelTypeGaussian,
};

// Operations that flip and rotate the image
// NOTE: these correspond 1:1 with the exif orientation values
// but the names describe the forward transform rather than the inverse
enum class FlipRotateOperation : uint8_t
{
  None = 1,
  Mirror = 2,
  Rotate180 = 3,
  Rotate180Mirror = 4,
  Rotate90Mirror = 5, 
  Rotate90 = 6, 
  Rotate270Mirror = 7,
  Rotate270 = 8
};

struct ScaleSettings
{
  // Determines aspect and scale of image resizing
  // "Stretch" changes the source image aspect ratio to match the destination image
  // "Fill" maintains the source image aspect ratio and scales it large enough to fill all destination pixels
  // "Fit" maintains the source image aspect ratio and scales it to fit inside the destination image
  ScaleMode scaleMode = ScaleMode::Stretch;

  InterpolationMode interpolationMode = InterpolationMode::Auto;

  // When scaling or cropping, this determines what color fills in the background if not all destination pixels are covered
  RGB24 backgroundColor {0, 0, 0};
};

template <typename C>
class Image
{
public:

    using Color = C;

    // Construct an image with no data
    Image()
    {
      width_ = 0;
      height_ = 0;
    }

    // Construct an image with the specified size, allocating memory.
    Image(int width, int height)
    {
      width_ = width;
      height_ = height;
      data_.resize(width_ * height_ * sizeof(Color));
    }

    // Get a pointer to the first pixel element
    uint8_t* data()
    {
      return data_.data();
    }

    // Get a const pointer to the first pixel element
    const uint8_t* data() const
    {
      return data_.data();
    }

    size_t dataSizeBytes() const 
    {
      return data_.size();
    }

    // Get a pointer to the first pixel element
    Color* pixel()
    {
      return (Color*) data_.data();
    }

    // Get a const pointer to the first pixel element
    const Color* pixel() const
    {
      return (const Color*) data_.data();
    }

    // Get the image width
    int width() const
    {
      return width_;
    }

    // Get the image height
    int height() const
    {
      return height_;
    }

    // Get the bounding box of this image's pixel grid
    BoundingBox bounds() const
    {
      return {0, 0, width_, height_};
    }

    // Get the number of bytes per pixel
    int bytesPerPixel() const
    {
      return sizeof(Color);
    }

    void rotateFlip(FlipRotateOperation op)
    {
      rotateFlip(*this, op);
    }

    void rotateFlip(Image<Color> &dest, FlipRotateOperation op) const
    {
      bool inPlace = (&dest == this);

      // No operation needed
      if (op == FlipRotateOperation::None)
      {
        if (!inPlace)
        {
          dest.data_ = data_;
          dest.width_ = width_;
          dest.height_ = height_;
        }
      }
      // We have to move the data
      else
      {
        std::vector<uint8_t> destData(data_.size());
        int bpp = bytesPerPixel();
        const uint8_t *src = data_.data();
        uint8_t *dst = destData.data();
        dest.width_ = width_;
        dest.height_ = height_;
        switch (op)
        {
        case FlipRotateOperation::Mirror:
          for (int y = 0; y < dest.height_; ++y)
          {
            for (int x = dest.width_ - 1; x >= 0; --x)
            {
              memcpy(dst + (x + y * dest.width_) * bpp, src, bpp);
              src += bpp;
            }
          }
          break;
        case FlipRotateOperation::Rotate180:
          for (int y = dest.height_ - 1; y >= 0; --y)
          {
            for (int x = dest.width_ - 1; x >= 0; --x)
            {
              memcpy(dst + (x + y * dest.width_) * bpp, src, bpp);
              src += bpp;
            }
          }
          break;

        case FlipRotateOperation::Rotate180Mirror:
          for (int y = dest.height_ - 1; y >= 0; --y)
          {
            for (int x = 0; x < dest.width_; ++x)
            {
              memcpy(dst + (x + y * dest.width_) * bpp, src, bpp);
              src += bpp;
            }
          }
          break;
        case FlipRotateOperation::Rotate90:
          std::swap(dest.width_, dest.height_);
          for (int x = dest.width_ - 1; x >= 0; --x)
          {
            for (int y = 0; y < dest.height_; ++y)
            {
              memcpy(dst + (x + y * dest.width_) * bpp, src, bpp);
              src += bpp;
            }
          }
          break;
        case FlipRotateOperation::Rotate90Mirror:
          std::swap(dest.width_, dest.height_);
          for (int x = 0; x < dest.width_; ++x)
          {
            for (int y = 0; y < dest.height_; ++y)
            {
              memcpy(dst + (x + y * dest.width_) * bpp, src, bpp);
              src += bpp;
            }
          }
          break;
        case FlipRotateOperation::Rotate270:
          std::swap(dest.width_, dest.height_);
          for (int x = 0; x < dest.width_; ++x)
          {
            for (int y = dest.height_ - 1; y >= 0; --y)
            {
              memcpy(dst + (x + y * dest.width_) * bpp, src, bpp);
              src += bpp;
            }
          }
          break;
        case FlipRotateOperation::Rotate270Mirror:
          std::swap(dest.width_, dest.height_);
          for (int x = dest.width_ - 1; x >= 0; --x)
          {
            for (int y = dest.height_ - 1; y >= 0; --y)
            {
              memcpy(dst + (x + y * dest.width_) * bpp, src, bpp);
              src += bpp;
            }
          }
          break;
        default:
          break;
        }
        dest.data_ = std::move(destData);
      }

      return;
    }

    void scale(int width, int height, ScaleSettings settings)
    {
      scale(*this, width, height, settings);
    }

    void scale(Image<Color> &dest, int width, int height, ScaleSettings settings) const
    {
      bool inPlace = (&dest == this);

      float xScale = (float)width / (float)width_;
      float yScale = (float)height / (float)height_;

      int uncroppedWidth = width;
      int uncroppedHeight = height;

      if (settings.scaleMode == ScaleMode::Fill)
      {
        float scale = std::max(xScale, yScale);
        uncroppedWidth = (int)((float)width_ * scale);
        uncroppedHeight = (int)((float)height_ * scale);
      }
      else if (settings.scaleMode == ScaleMode::Fit)
      {
        float scale = std::min(xScale, yScale);
        uncroppedWidth = (int)((float)width_ * scale);
        uncroppedHeight = (int)((float)height_ * scale);
      }

      if (settings.interpolationMode == InterpolationMode::Auto)
      {
        // Use Bilinear for enlargement and Gaussian for reduction
        settings.interpolationMode = (width > width_) ? InterpolationMode::Bilinear : InterpolationMode::Gaussian;
      }

      if (width == width_ && height == height_)
      {
        // Image is the correct size already!
        // Just copy the data as-is
        if (!inPlace)
        {
          dest.data_ = data_;
        }
      }
      else
      {
        std::vector<uint8_t> scaleData(uncroppedWidth * uncroppedHeight * sizeof(Color));
        base::ResampleImage<sizeof(Color)>(data_.data(), (uint32_t)width_, (uint32_t)height_,
                              scaleData.data(), (uint32_t)uncroppedWidth, (uint32_t)uncroppedHeight,
                              (base::KernelType)settings.interpolationMode);
        dest.data_ = std::move(scaleData);
      }

      dest.width_ = uncroppedWidth;
      dest.height_ = uncroppedHeight;

      if (width != uncroppedWidth || height != uncroppedHeight)
      {
        dest.crop((uncroppedWidth - width) / 2, (uncroppedHeight - height) / 2, width, height, settings);
      }
    }

    void crop(int x, int y, int width, int height, ScaleSettings settings)
    {
      crop(*this, x, y, width, height, settings);
    }

    void crop(Image &dest, int x, int y, int width, int height, ScaleSettings settings) const
    {
      bool inPlace = (&dest == this);

      if (x == 0 && y == 0 && width == width_ && height_ == height)
      {
        if (!inPlace)
        {
          dest.width_ = width_;
          dest.height_ = height_;
          dest.data_ = data_;
        }
        return;
      }

      // Create a buffer for the cropped image
      std::vector<uint8_t> croppedData(width * height * sizeof(Color));
      int size = width * height;

      // Fill in the background color
      Color* data = (Color*)croppedData.data();
      for (int i = 0; i < size; ++i)
      {
        data[i] = settings.backgroundColor;
      }

      // Figure out the copy boundaries
      int srcX = (x < 0) ? 0 : x;
      int srcY = (y < 0) ? 0 : y;
      int dstX = (x > 0) ? 0 : -x;
      int dstY = (y > 0) ? 0 : -y;
      int cpyWidth = std::min(width - dstX, width_ - srcX);
      int cpyHeight = std::min(height - dstY, height_ - srcY);

      // Do the copy row by row
      for (int row = 0; row < cpyHeight; ++row)
      {
        const uint8_t *src = data_.data() + (srcX + (srcY + row) * width_) * bytesPerPixel();
        uint8_t *dst = croppedData.data() + (dstX + (dstY + row) * width) * bytesPerPixel();
        memcpy(dst, src, cpyWidth * bytesPerPixel());
      }

      // Move the cropped data into place
      dest.width_ = width;
      dest.height_ = height;
      dest.data_ = std::move(croppedData);
    }

    template <typename DestColor>
    Image<DestColor> convert() const
    {
      std::vector<uint8_t> destData;

      // Image is the correct format already!
      // Just copy the data as-is
      if constexpr(std::is_same_v<DestColor,Color>)
      {
        destData = data_;
      }
      else
      {
        size_t count = width_ * height_;
        destData.resize(sizeof(DestColor) * count);
        DestColor* destPtr = (DestColor*) destData.data();
        auto srcPtr = pixel();
        for (size_t i = 0; i < count; ++i)
        {
          destPtr[i] = srcPtr[i];
        }
      }

      // Construct the destination image
      Image<DestColor> dest;
      dest.width_ = width_;
      dest.height_ = height_;
      dest.data_ = std::move(destData);

      return dest;
    }

    template <typename DestColor>
    Image<DestColor> moveConvert()
    {
      std::vector<uint8_t> destData;

      // Image is the correct format already!
      // Just copy the data as-is
      if constexpr(std::is_same_v<DestColor,Color>)
      {
        destData = std::move(data_);
      }
      else
      {
        size_t count = width_ * height_;
        destData.resize(sizeof(DestColor) * count);
        DestColor* destPtr = (DestColor*) destData.data();
        auto srcPtr = pixel();
        for (size_t i = 0; i < count; ++i)
        {
          destPtr[i] = srcPtr[i];
        }
      }

      // Construct the destination image
      Image<DestColor> dest;
      dest.width_ = width_;
      dest.height_ = height_;
      dest.data_ = std::move(destData);

      // Delete the original data
      width_ = 0;
      height_ = 0;
      data_.clear();

      return dest;
    }

private:
    // ImageIO likes to access internals while deserializing
    friend class ImageIO;
    
    // We need to friend all the possible specializations
    // because C++ is a mystery beyond understanding.
    friend class Image<RGBA32>;
    friend class Image<RGB24>;
    friend class Image<Gray8>;
    friend class Image<HSV3f>;
    friend class Image<Lab3f>;

    int width_, height_;
    std::vector<uint8_t> data_;
};
