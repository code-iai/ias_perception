#include <iostream>
#include <vector>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "ccd/bspline.h"
#include "ccd/ccd.h"
using namespace std;
using namespace cv;
std::vector<cv::Point3d> pts;
void on_mouse(int event, int x, int y, int flags, void *param )
{
  CCD *my_ccd = (CCD *)param;
  cv::Mat image = my_ccd->canvas;
  if( image.empty())
    return ;

  //caution: check
  // if( image1.at<double>() )
  //   y = image1->height - y;
  switch( event )
  {
    case CV_EVENT_LBUTTONDOWN:
      break;
    case CV_EVENT_LBUTTONUP:
      cv::circle(image,cv::Point(x,y),1,cv::Scalar(0,0,255),1);
      pts.push_back(cv::Point3d(x,y,1));
      cv::imshow("B-spline", image);
      break;
  }
}
void contourManually(CCD &my_ccd)
{
  int key;
  cv::namedWindow("B-spline", 1);
  cv::setMouseCallback( "B-spline", on_mouse,  (void*)&my_ccd);
  cv::imshow("B-spline", my_ccd.canvas);
  while (1)
  {
    key = cv::waitKey(10);
    if (key == 27) break;
  }
}

int main (int argc, char * argv[]) 
{
  cv::namedWindow("B-spline", 1);
  cv::Mat image = cv::Mat::ones(600, 800, CV_8UC3);
  for (int i = 0; i < image.rows; ++i){
    for (int j =0; j < image.cols; ++j){
      image.at<Vec3b>(i,j)[0] = 255;
      image.at<Vec3b>(i,j)[1] = 255;
      image.at<Vec3b>(i,j)[2] = 255;
    }
  }

  CCD my_ccd;
  image.copyTo(my_ccd.canvas);
  image.copyTo(my_ccd.image);
  contourManually(my_ccd);
  // if((int)my_ccd.pts.size() > my_ccd.degree())
  // {
  //   for (int i = 0; i < my_ccd.degree(); ++i)
  //     my_ccd.pts.push_back(my_ccd.pts[i]);
  // }
  for (int i = 0; i < pts.size()-1; ++i)
  {
        int j = (i+1)%pts.size();
        std::cout << pts[i].x << " " << pts[i].y << std::endl;
        cv::circle(my_ccd.canvas, cv::Point2d(pts[i].x, pts[i].y), 2, CV_RGB(0,255,0), 2);
        cv::line(my_ccd.canvas, cv::Point2d(pts[i].x, pts[i].y),cv::Point2d(pts[i+1].x, pts[i+1].y),CV_RGB( 0, 0, 0 ),2,8,0);
    }
          cv::circle(my_ccd.canvas, cv::Point2d(pts[pts.size()-1].x, pts[pts.size()-1].y), 2, CV_RGB(0,255,0), 2);
  // std::cout << pts[pts.size()-1].x << " " << pts[pts.size()-1].y << std::endl;
  my_ccd.read_params("ccd_params1.xml");
  BSpline bs1(my_ccd.degree() , my_ccd.resolution(), pts);
  for (int i = 0; i < my_ccd.resolution()-1; ++i)
  {
        int j = (i+1)%(int)my_ccd.resolution();
        cv::line(my_ccd.canvas, cv::Point2d(bs1[i].x, bs1[i].y),cv::Point2d(bs1[i+1].x, bs1[i+1].y),CV_RGB( 255, 0, 0 ),2,8,0);
    }

  my_ccd.read_params("ccd_params.xml");
  BSpline bs2(my_ccd.degree() , my_ccd.resolution(), pts);
  for (int i = 0; i < my_ccd.resolution()-1; ++i)
  {
        int j = (i+1)%(int)my_ccd.resolution();
        cv::line(my_ccd.canvas, cv::Point2d(bs2[i].x, bs2[i].y),cv::Point2d(bs2[i+1].x, bs2[i+1].y),CV_RGB(0, 0, 255 ),2,8,0);
  }
  char key;
  cv::imshow("B-spline", my_ccd.canvas);
  cv::imwrite("bspline.jpg", my_ccd.canvas);
  while (1)
  {
    key = cv::waitKey(10);
    if (key == 27) break;
  }  
  return 0;
}
