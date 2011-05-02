#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "bspline.h"
#include <iostream>
#include <string>
#include <cstdio>
#include <algorithm>
using namespace cv;
struct CCDParams
{
  CCDParams(): gamma_1(0.5), gamma_2(4), gamma_3(4), gamma_4(3), h(40), delta_h(1), kappa(0.5), c(0.25), resolution(50)
  {
  }
  CCDParams(double p1,
            double p2,
            double p3,
            double p4,
            double p5,
            double p6,
            int p7,
            int p8,
            int p9)
  {
    gamma_1 = p1;
    gamma_2 = p2;
    gamma_3 = p3;
    gamma_4 = p4;
    kappa = p5;
    c = p6;
    h = p7;
    delta_h = p8;
    resolution = p9;
  }

  ~CCDParams()
  {
  }
  double gamma_1;
  double gamma_2;
  double gamma_3;
  double gamma_4;
  int h;
  int delta_h;
  double kappa;
  double c;
  int resolution;
};

class CCD
{
public:
  cv::Mat img, canvas;
  CCD():Phi(cv::Mat(6,1, CV_64F)),Sigma_Phi(cv::Mat(6,6, CV_64F)), delta_Phi(cv::Mat(6,1, CV_64F))
  {
  
  };
  void init_pts(std::vector<CvPoint2D64f> &pts);
  void set_params(double *params);
  void run_ccd();
  ~CCD(){clear();}
private:
  void clear();
  void init_cov(BSpline &bs, int degree);
  void local_statistics(BSpline &bs);
  void refine_parameters(BSpline &bs);
  CCDParams params_;
  std::vector<CvPoint2D64f> pts;
  cv::Mat vic;
  cv::Mat mean_vic;
  cv::Mat cov_vic;
  cv::Mat nv;
  cv::Mat Phi;
  cv::Mat Sigma_Phi;
  cv::Mat delta_Phi;
  cv::Mat bs_old;
  cv::Mat nabla_E;
  cv::Mat hessian_E;
};
