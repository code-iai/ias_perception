// Software License Agreement (BSD License)
// 
//   Copyright (c) 2011, Shulei Zhu <schuleichu@gmail.com>
//   All rights reserved.
// 
//   Redistribution and use in source and binary forms, with or without
//   modification, are permitted provided that the following conditions
//   are met:
// 
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//    * Redistributions in binary form must reproduce the above
//      copyright notice, this list of conditions and the following
//      disclaimer in the documentation and/or other materials provided
//      with the distribution.
//    * Neither the name of Shulei Zhu nor the names of its
//      contributors may be used to endorse or promote products derived
//      from this software without specific prior written permission.
// 
//   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
//   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
//   COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//   INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
//   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
//   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
//   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//   POSSIBILITY OF SUCH DAMAGE.
// 
// 
// ccd_tracker.cpp --- 
// File            : ccd_tracker.cpp
// Created: Sa Jun 18 14:04:47 2011 (+0200)
// Author: Shulei Zhu

// Code:

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
