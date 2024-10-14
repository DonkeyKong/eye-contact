#pragma once

#include "Image.hpp"
#include <filesystem>
#include <iostream>

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
  static Image<RGBAColor> LoadFromStream(std::istream&, ImageLoadSettings settings = {});
  static Image<RGBAColor> LoadFromBuffer(const std::string&, ImageLoadSettings settings = {});
  static Image<RGBAColor> LoadFromFile(std::filesystem::path, ImageLoadSettings settings = {});
  static void SaveToStream(const Image<RGBAColor>&, std::ostream&, ImageSaveSettings settings = {});
  static void SaveToBuffer(const Image<RGBAColor>&, std::string&, ImageSaveSettings settings = {});
  static void SaveToFile(std::filesystem::path, const Image<RGBAColor>&, ImageSaveSettings settings = {});
private: 
  static void readJpeg(std::istream&, Image<RGBAColor>&, ImageLoadSettings);
  static void writeJpeg(std::ostream&, const Image<RGBAColor>&, ImageSaveSettings);
  static void readPng(std::istream&, Image<RGBAColor>&, ImageLoadSettings);
  static void writePng(std::ostream&, const Image<RGBAColor>&, ImageSaveSettings);
};