#include <iostream>
#include <thread>
#include <chrono>
#include <vector>

#include <fmt/format.h>

#include "Color.hpp"
#include "Image.hpp"
#include "Draw.hpp"
#include "ImageIO.hpp"
#include "V4LCamera.hpp"
#include "RemoteEyes.hpp"

using namespace V4L2;
using namespace std::chrono_literals;

template <typename Color>
Image<Color> imageFromFrame(const Frame& f)
{
  Image<Color> img(f.width, f.height);

  auto imgPtr = img.pixel();
  
  for (int y=0; y < f.height; y++)
  {
    uint8_t* yuyvPtr = f.data.start + f.strideInBytes * y;
    for (int x=0; x < f.width; x+=2)
    {
      if constexpr(std::is_same_v<Color, Gray8>)
      {
        imgPtr[0].I = yuyvPtr[0];
        imgPtr[1].I = yuyvPtr[2];
      }
      else
      {
        imgPtr[0] = YUV24{yuyvPtr[0], yuyvPtr[1], yuyvPtr[3]};
        imgPtr[1] = YUV24{yuyvPtr[2], yuyvPtr[1], yuyvPtr[3]};
      }
      imgPtr += 2;
      yuyvPtr += 4;
    }
  }
  return img;
}

int main(int argc, char *argv[])
{
    std::cout << "Preparing to make eye contact...\n";

    std::cout << "Setting up cam 0...\n";
    Camera cam0("/dev/video0", 1280, 720, 3);
    std::cout << "Cam 0 ok!\n";

    std::cout << "Setting up cam 1...\n";
    Camera cam1("/dev/video2", 1280, 720, 3);
    std::cout << "Cam 1 ok!\n";

    Image<Gray8> image0, image1;
    std::chrono::steady_clock::time_point nextSave;

    //RemoteEyes eyes;
    
    while (true)
    {
      auto t = std::thread([&]()
      {
        image0 = imageFromFrame<Gray8>(cam0.getFrame());
      });
      image1 = imageFromFrame<Gray8>(cam1.getFrame());
      t.join();
      
      //std::vector<Rect> faces0, faces1;
      // Detect faces
      //if (faces0.size() > 0)
      //{
        // float yaw = (float)(faces0[0].x - cam0.width() / 2) / (float)(cam0.width() / 2) * -60.0;
        // float pitch = (float)(faces0[0].y - cam0.height() / 2) / (float)(cam0.height() / 2) * -60.0;
        //std::cout << "Looking at: ( " << yaw << "º , " << pitch << "º )\n";
        //eyes.look(yaw, pitch);
      //}
      // for (const auto& face : faces0)
      // {
      //   DrawRect(image0, face.x, face.y, face.width, face.height, 3.0f, {255});
      //   DrawRect(image0, face.x, face.y, face.width, face.height, 1.0f, {0});
      // }

      // for (const auto& face : faces1)
      // {
      //   DrawRect(image1, face.x, face.y, face.width, face.height, 3.0f, {255});
      //   DrawRect(image1, face.x, face.y, face.width, face.height, 1.0f, {0});
      // }

      if (std::chrono::steady_clock::now() > nextSave)
      {
        ImageIO::SaveToFile("cam0.jpg", image0);
        ImageIO::SaveToFile("cam1.jpg", image1);
        nextSave = std::chrono::steady_clock::now() + 200ms;
      }
    }

    return 0;
}
