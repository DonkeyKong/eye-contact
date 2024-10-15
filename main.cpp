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
    Camera cam0("/dev/video0", 640, 480);
    std::cout << "Cam 0 ok!\n";

    std::cout << "Setting up cam 1...\n";
    Camera cam1("/dev/video2", 640, 480);
    std::cout << "Cam 1 ok!\n";
    
    // Load face classifier
    CascadeClassifier cascade;
    cascade.load(fmt::format("{}/haarcascades/haarcascade_frontalface_alt.xml", OPENCV_DATA_DIR));

    while (true)
    {
      auto image0 = imageFromFrame<Grayscale>(cam0.getFrame());
      auto image1 = imageFromFrame<Grayscale>(cam1.getFrame());
      
      Mat1b mat0(image0.height(), image0.width(), (uchar*) image0.data());
      Mat1b mat1(image1.height(), image1.width(), (uchar*) image1.data());

      std::vector<Rect> faces0, faces1;
      // Mat small0, small1;
  
      // // cvtColor( img, gray, COLOR_BGR2GRAY ); // Convert to Gray Scale
      // // double fx = 1 / scale;
  
      // // Resize the Grayscale Image 
      // resize(mat0, small0, Size(), fx, fx, INTER_LINEAR );
      // resize(mat1, small1, Size(), fx, fx, INTER_LINEAR );
      // equalizeHist( smallImg, smallImg );
  
      // // Detect faces of different sizes using cascade classifier 
      cascade.detectMultiScale( mat0, faces0, 1.1, 2, CASCADE_SCALE_IMAGE, Size(30, 30));
      cascade.detectMultiScale( mat1, faces1, 1.1, 2, CASCADE_SCALE_IMAGE, Size(30, 30));

      std::cout << "Faces found: ( " << faces0.size() << " , " << faces1.size() << " )\n";

      ImageIO::SaveToFile("cam0.jpg", image0);
      ImageIO::SaveToFile("cam1.jpg", image1);
      // std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::cout << "Done\n";
    return 0;
}
