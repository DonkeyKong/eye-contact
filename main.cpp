#include <iostream>
#include <thread>
#include <chrono>
#include <vector>

#include <fmt/format.h>

#include <opencv2/objdetect.hpp>
#include <opencv2/imgproc.hpp>

#include "Color.hpp"
#include "Image.hpp"
#include "ImageIO.hpp"
#include "V4LCamera.hpp"
#include "RemoteEyes.hpp"

using namespace V4L2;
using namespace cv;

const char* OPENCV_DATA_DIR = "/usr/share/opencv4";

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
    
    // Load face classifier
    CascadeClassifier cascade;
    cascade.load(fmt::format("{}/haarcascades/haarcascade_frontalface_alt.xml", OPENCV_DATA_DIR));

    Image<Gray8> image0, image1;
    //RemoteEyes eyes;
    
    while (true)
    {
      auto t = std::thread([&]()
      {
        image0 = imageFromFrame<Gray8>(cam0.getFrame());
      });
      image1 = imageFromFrame<Gray8>(cam1.getFrame());
      t.join();

      cv::Mat mat0(image0.height(), image0.width(), CV_8UC1, (uchar*) image0.data());
      cv::Mat mat1(image1.height(), image1.width(), CV_8UC1, (uchar*) image1.data());
      
      std::vector<Rect> faces0, faces1;
  
      // Detect faces of different sizes using cascade classifier 
      auto t2 = std::thread([&]()
      {
        cascade.detectMultiScale(mat0, faces0, 1.1, 3, CASCADE_SCALE_IMAGE, Size(128, 128), Size(320, 320));
        ImageIO::SaveToFile("cam0.jpg", image0);
      });
      //cascade.detectMultiScale(mat1, faces1, 1.1, 3, CASCADE_SCALE_IMAGE, Size(128, 128), Size(320, 320));
      //ImageIO::SaveToFile("cam1.jpg", image1);
      t2.join();

      //std::cout << "Faces found: ( " << faces0.size() << " , " << faces1.size() << " )\n";

      if (faces0.size() > 0)
      {
        float yaw = (float)(faces0[0].x - cam0.width() / 2) / (float)(cam0.width() / 2) * -60.0;
        float pitch = (float)(faces0[0].y - cam0.height() / 2) / (float)(cam0.height() / 2) * -60.0;

        std::cout << "Looking at: ( " << yaw << "º , " << pitch << "º )\n";
        //eyes.look(yaw, pitch);
      }
    }

    return 0;
}
