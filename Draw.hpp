#pragma once

#include "Image.hpp"
#include <algorithm>

enum class BlendMode
{
  Overwrite,
  Multiply,
  Add
};

template <typename Color>
void FillRect(Image<Color>& img, float originX, float originY, float width, float height, Color color, BlendMode blend = BlendMode::Overwrite)
{
  int minX = std::clamp((int)std::round(originX), 0, img.width());
  int minY = std::clamp((int)std::round(originY), 0, img.height());
  int maxX = std::clamp((int)std::round(originX+width), 0, img.width());
  int maxY = std::clamp((int)std::round(originY+height), 0, img.height());

  Color* px = img.pixel();

  if (blend == BlendMode::Overwrite)
  {
    for (int y = minY; y < maxY; ++y)
    {
      for (int x = minX; x < maxX; ++x)
      {
        px[x+y*img.width()] = color;
      }
    }
  }
  else if (blend == BlendMode::Add)
  {
    for (int y = minY; y < maxY; ++y)
    {
      for (int x = minX; x < maxX; ++x)
      {
        px[x+y*img.width()] += color;
      }
    }
  }
  else if (blend == BlendMode::Multiply)
  {
    for (int y = minY; y < maxY; ++y)
    {
      for (int x = minX; x < maxX; ++x)
      {
        px[x+y*img.width()] *= color;
      }
    }
  }
}

template <typename Color>
void DrawRect(Image<Color>& img, float originX, float originY, float width, float height, float strokeWeight, Color color, BlendMode blend = BlendMode::Overwrite)
{
  FillRect(img, originX, originY, width, strokeWeight, color, blend); // top
  FillRect(img, originX, originY+height-strokeWeight, width, strokeWeight, color, blend); // bottom
  FillRect(img, originX, originY, strokeWeight, height , color, blend); // left
  FillRect(img, originX+width-strokeWeight, originY, strokeWeight, height , color, blend); // right
}

template <typename Color>
void DrawPoint(Image<Color>& img, float x, float y, float sizePx, Color color, BlendMode blend = BlendMode::Overwrite)
{
  FillRect(img, x - sizePx / 2.0f, y - sizePx / 2.0f, sizePx, sizePx, color, blend);
}