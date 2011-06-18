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
// contour.cpp --- 
// File            : contour.cpp
// Created: Sa Jun 18 14:05:01 2011 (+0200)
// Author: Shulei Zhu

// Code:



#include "opencv/cv.h"
#include "opencv/highgui.h"
#include <iostream>
#include <vector>
#include <string>
#include "ccd/sift_init.h"
#include "ccd/bspline.h"
#include <fstream>
using namespace std;
using namespace cv;
vector<cv::Point3d> pts;
void on_mouse(int event, int x, int y, int flags, void* param )
{
  //  MaskParams* params = (MaskParams*)_params;
  
  cv::Mat *image = (cv::Mat *)param;
  if( image->empty())
    return;

  //caution: check
  // if( image1.at<double>() )
  //   y = image1->height - y;

  switch( event )
  {
    case CV_EVENT_LBUTTONDOWN:
      break;
    case CV_EVENT_LBUTTONUP:
      cv::circle(*image,cv::Point(x,y),2,cv::Scalar(0,0,255),2);
      pts.push_back(cv::Point3d(x,y,1.0));
      cv::imshow("Contour", *image);
      break;
  }
}

int main (int argc, char * argv[]) 
{
  char key;
  cv::Mat tpl = cv::imread(argv[1]);
  cv::namedWindow("Contour", 1);
  cv::setMouseCallback( "Contour", on_mouse,  (void*)&tpl);
  cv::imshow("Contour", tpl);
  while (1)
  {
    key = cv::waitKey(10);
    if (key == 27) break;
  }
  cv::Mat contour_mat(pts.size(),3,CV_64FC1);
  for (size_t i = 0; i < pts.size(); ++i){
    double *ptr = contour_mat.ptr<double>(i);
    ptr[0] = pts[i].x;
    ptr[1] = pts[i].y;
    ptr[2] = pts[i].z;
  }

  FileStorage fs("contour.xml", cv::FileStorage::WRITE);
  fs.open("contour.xml", FileStorage::WRITE);
  fs<< "contour_points" <<  contour_mat ;
  fs.release();

  
  CvFileStorage* new_fs= cvOpenFileStorage("contour.xml", 0, CV_STORAGE_READ);
  CvMat *coordinates= (CvMat*)cvReadByName(new_fs,  NULL, "contour_points", NULL);
  // CvMat *coordinates_t = cvCreateMat(3 ,coordinates->rows,  CV_64FC1);
  int step = coordinates->step/sizeof(double);
  double *ptr = coordinates->data.db;
  vector<cv::Point3d> test(pts.size());     
  for (int i = 0; i < coordinates->rows; ++i)
  {
      test[i].x = (ptr+i*step)[0];
      test[i].y = (ptr+i*step)[1];
      test[i].z = (ptr+i*step)[2];
      std::cout << test[i].x  << " " << test[i].y  << " " <<test[i].z<< std::endl;
    }
  
  // cv::Mat test_mat(pts.size(), 2,CV_64FC1);;
  // vector<cv::Point2d> test(pts.size());
  // if (!fs.isOpened()){
  //   fs.open("contour.xml", FileStorage::READ);
  //   fs["contour_points"] >> test_mat ;
  //   for (size_t i = 0; i < pts.size(); ++i)
  //   {
  //     double *ptr = test_mat.ptr<double>(i);
  //     test[i].x = ptr[0];
  //     test[i].y = ptr[1];
  //     std::cout << test[i].x  << " " << test[i].y << std::endl;
  //   }
  //   fs.release();
  // }
  return 0;
}
