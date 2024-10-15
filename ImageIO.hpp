#pragma once

#include "Image.hpp"
#include <filesystem>
#include <iostream>
#include <fstream>
#include <sstream>

enum class ImageFormat
{
  Auto,
  PNG,
  JPEG
};

struct ImageLoadSettings
{
  // Use any orientation metadata to rectify the image as it is loaded
  bool autoRotate = true;
};

struct ImageSaveSettings
{
  ImageFormat saveFormat = ImageFormat::Auto;
  int jpegQuality = 75;
};

struct ImageIO
{
public:
  ImageIO() = delete;

  template<typename Color>
  static Image<Color> LoadFromStream(std::istream& stream, ImageLoadSettings settings = {})
  {
    // Detect the file type
    uint8_t header[8];
    stream.read((char*)header, 8);
    auto format = detectFormat(header);

    // Rewind the input stream...
    stream.clear();
    stream.seekg(0, std::ios::beg);

    if (format == ImageFormat::JPEG)
    {
      Image<RGBColor> img;
      readJpeg(stream, img, settings);
      return img.moveConvert<Color>();
    }
    else if (format == ImageFormat::PNG)
    {
      Image<RGBAColor> img;
      readPng(stream, img, settings);
      return img.moveConvert<Color>();
    }
    else
    {
      throw std::runtime_error("Unsupported image data!");
    }
  }

  template<typename Color>
  static Image<Color> LoadFromBuffer(const std::string& str, ImageLoadSettings settings = {})
  {
    std::istringstream inputStream(str);
    return LoadFromStream<Color>(inputStream, settings);
  }

  template<typename Color>
  static Image<RGBAColor> LoadFromFile(std::filesystem::path imagePath, ImageLoadSettings settings = {})
  {
    std::ifstream inputStream(imagePath, std::ios::binary);
    return LoadFromStream<Color>(inputStream, settings);
  }

  template<typename Color>
  static void SaveToStream(const Image<Color>& image, std::ostream& stream, ImageSaveSettings settings = {})
  {
    // At this point we have no context for Auto, so
    // just pick PNG
    if (settings.saveFormat == ImageFormat::Auto)
    {
      settings.saveFormat = ImageFormat::PNG;
    }

    // Verify the image is valid
    if (image.width() < 1 || image.height() < 1)
    {
      throw std::runtime_error("Cannot save zero-dimension image!");
    }

    if (settings.saveFormat == ImageFormat::JPEG)
    {
      if constexpr(std::is_same_v<Color, RGBColor>)
      {
        writeJpeg(stream, image, settings);
      }
      else
      {
        Image<RGBColor> conv = image.template convert<RGBColor>();
        writeJpeg(stream, conv, settings);
      }
    }
    else if (settings.saveFormat == ImageFormat::PNG)
    {
      if constexpr(std::is_same_v<Color, RGBAColor>)
      {
        writePng(stream, image, settings);
      }
      else
      {
        Image<RGBAColor> conv = image.template convert<RGBAColor>();
        writePng(stream, conv, settings);
      }
    }
    else
    {
      throw std::runtime_error("Unsupported image data!");
    }
  }

  template<typename Color>
  static void SaveToBuffer(const Image<Color>& image, std::string& str, ImageSaveSettings settings = {})
  {
    std::ostringstream outputStream;
    SaveToStream<Color>(image, outputStream, settings);
    str = outputStream.str();
  }

  template<typename Color>
  static void SaveToFile(std::filesystem::path imagePath, const Image<Color>& image, ImageSaveSettings settings = {})
  {
    if (settings.saveFormat == ImageFormat::Auto)
    {
      settings.saveFormat = detectFormat(imagePath);
    }
    std::ofstream outputStream(imagePath, std::ios::binary | std::ios::trunc);
    SaveToStream<Color>(image, outputStream, settings);
    outputStream.close();
  }

private: 
  static ImageFormat detectFormat(uint8_t header[8]);
  static ImageFormat detectFormat(std::filesystem::path imagePath);
  static void readJpeg(std::istream&, Image<RGBColor>&, ImageLoadSettings);
  static void writeJpeg(std::ostream&, const Image<RGBColor>&, ImageSaveSettings);
  static void readPng(std::istream&, Image<RGBAColor>&, ImageLoadSettings);
  static void writePng(std::ostream&, const Image<RGBAColor>&, ImageSaveSettings);
};