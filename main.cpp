#include <iostream>

#include <fmt/format.h>

#include "Color.hpp"
#include "Image.hpp"
#include "ImageIO.hpp"
#include "V4LCamera.hpp"

using namespace V4L2;

Image imageFromFrame(const Frame& f)
{
  Image img(f.width, f.height);

  RGBAColor* imgPtr = (RGBAColor*)img.data();
  
  for (int y=0; y < f.height; y++)
  {
    uint8_t* yuyvPtr = f.data.start + f.strideInBytes * y;
    for (int x=0; x < f.width; x+=2)
    {
      imgPtr->setFromYuv(yuyvPtr[0], yuyvPtr[1], yuyvPtr[3]);
      ++imgPtr;
      imgPtr->setFromYuv(yuyvPtr[2], yuyvPtr[1], yuyvPtr[3]);
      ++imgPtr;
      yuyvPtr += 4;
    }
  }
  return img;
}

int main(int argc, char *argv[])
{
    std::cout << "Preparing to make eye contact...\n";

    std::cout << "Setting up cam 0...\n";
    Camera cam0("/dev/video0", 2);
    std::cout << "Cam 0 ok!\n";

    std::cout << "Setting up cam 1...\n";
    Camera cam1("/dev/video2", 2);
    std::cout << "Cam 1 ok!\n";

    for (int i=0; i < 1; ++i)
    {
      auto frame0 = cam0.getFrame();
      auto frame1 = cam1.getFrame();

      ImageIO::SaveToFile(fmt::format("cam0_frame{}.jpg", i), imageFromFrame(frame0));
      ImageIO::SaveToFile(fmt::format("cam1_frame{}.jpg", i), imageFromFrame(frame1));
    }

    std::cout << "Done\n";
    return 0;
}
