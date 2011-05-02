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
