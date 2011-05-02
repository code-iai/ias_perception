#include "opencv/cv.h"
#include "opencv/highgui.h"
#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include "ccd/sift_init.h"
#include "ccd/bspline.h"
#include "ccd/ccd.h"
using namespace std;
using namespace cv;
namespace {
void help(char** av) {
  cout << "\nThis program justs gets you started reading images from video\n"
      "Usage:\n./" << av[0] << " <video device number>\n"
       << "\tThis is a starter sample, to get you up and going in a copy pasta fashion\n"
       << "\tThe program captures frames from a camera connected to your computer.\n"
       << "\tTo find the video device number, try ls /dev/video* \n"
       << "\tYou may also pass a video file, like my_vide.avi instead of a device number"
       << endl;
}

int process(VideoCapture& capture) {
  string window_name = "video | q or esc to quit";
  cout << "press q or esc to quit" << endl;
  //  capture.set(CV_CAP_PROP_FPS, 10);
  CCD my_ccd;  


  // namedWindow(window_name, CV_WINDOW_KEEPRATIO); //resizable window;
  Mat frame;
  for (int frame_count =0 ;; frame_count++) {
    capture >> frame;
    if (frame.empty())
      continue;
    // imshow(window_name, frame);
    frame.copyTo(my_ccd.canvas);
    frame.copyTo(my_ccd.image);
    // if(template_path != "")
    //   my_ccd.tpl = cv::imread(template_path, 1 );
    if(frame_count == 0)
    {
      my_ccd.init_pts(1);
      my_ccd.read_params("ccd_params.xml");
      my_ccd.init_mat();
    }

  // std::cout << "hellooooooo" << std::endl;
    my_ccd.run_ccd();
    std::cout << "frame_count: " << frame_count << std::endl;
    
    cv::imshow("CCD", my_ccd.canvas);
    cv::waitKey(100);
  }
  return 0;
}

}

int main (int argc, char * argv[]) 
{
  if (argc != 2) {
    help(argv);
    return 1;
  }
  std::string arg = argv[1];
  VideoCapture capture(arg); //try to open string, this will attempt to open it as a video file
  if (!capture.isOpened()) //if this fails, try to open as a video camera, through the use of an integer param
    capture.open(atoi(arg.c_str()));
  if (!capture.isOpened()) {
    cerr << "Failed to open a video device or video file!\n" << endl;
    help(argv);
    return 1;
  }
  std::cout << "fps: " << capture.get(CV_CAP_PROP_FPS)<< std::endl;
  return process(capture);
}
