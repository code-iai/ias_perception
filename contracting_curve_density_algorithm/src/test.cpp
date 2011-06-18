#include "opencv/cv.h"
#include "opencv/highgui.h"
#include <iostream>
#include <vector>
#include <string>
#include "ccd/bspline.h"
#include "ccd/ccd.h"
#include "auto_init.h"
#include <fstream>

using namespace cv;
using namespace std;

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
      my_ccd->pts.push_back(cv::Point3d(x,y,1));
      cv::imshow("CCD", image);
      break;
  }
}

void contourFPM(CCD &my_ccd)
{
  // CvMat points_mat = FPM_init(tpl_ptr, tpl_img_ptr, 30);
  AutoInit *ai = new AutoInit(1,0, 30);
  ai->init(my_ccd.tpl, my_ccd.canvas);
  double *ptr;
  for (int row = 0; row < (ai->control_points).rows; ++row)
  {
    ptr = (ai->control_points).ptr<double>(row);
    my_ccd.pts.push_back(cv::Point3d(ptr[0]/ptr[2], ptr[1]/ptr[2], 1));
  }
}

  
void contourManually(CCD &my_ccd)
{
  int key;
  cv::namedWindow("CCD", 1);
  cv::setMouseCallback( "CCD", on_mouse,  (void*)&my_ccd);
  cv::imshow("CCD", my_ccd.canvas);
  while (1)
  {
    key = cv::waitKey(10);
    if (key == 27) break;
  }
}

static int print_help()
{
  cout << "Usage:\n ./test -m init_method [-t template_image] -i input_image params.xml"<< endl;
  return 0;
}

int main (int argc, char * argv[]) 
{
  int key;
  std::vector<cv::Point2d> pts;


  string  image_path, template_path, params_file_path;
  int init_method;
  if(argc <= 1)
    return print_help();
  
  for( int i = 1; i < argc; i++ )
  {
    if( string(argv[i]) == "-m" )
    {
      if( sscanf(argv[++i], "%d", &init_method) != 1 || (init_method < 0  ||  init_method >3))
      {
        cout << "invalid initialization method" << endl;
        return print_help();
      }
    }
    else if( string(argv[i]) == "-i" )
    {
      if( string(argv[++i]).length() <= 0 )
      {
        cout << "invalid image file path" << endl;
        return print_help();
      }
      else
        image_path = argv[i];
    }
    else if( string(argv[i]) == "-t" )
    {
      if( string(argv[++i]).length() <= 0 )
      {
        cout << "invalid template image file path" << endl;
        return print_help();
      }
      else
        template_path = argv[i];
    }
    else if( string(argv[i]) == "--help" )
      return print_help();
    else if( argv[i][0] == '-' )
    {
      cout << "invalid option " << argv[i] << endl;
      return 0;
    }
    else
      params_file_path = argv[i];

  }
  if(params_file_path == "")
  {
    params_file_path = "ccd_params.xml";
  }
    
  // double *params = new double[10];

  
  CCD my_ccd;
  my_ccd.canvas = cv::imread(image_path, 1);
  my_ccd.image = cv::imread(image_path, 1);
  if(template_path != "")
    my_ccd.tpl = cv::imread(template_path, 1 );
  if(init_method == 0)
    contourManually(my_ccd);
  else if(init_method == 1)
    contourFPM(my_ccd);
  // else if(init_method == 3)
  //   contourP()
  if((int)my_ccd.pts.size() > my_ccd.degree())
  {
    for (int i = 0; i < my_ccd.degree(); ++i)
      my_ccd.pts.push_back(my_ccd.pts[i]);
  }
  // my_ccd.init_pts(init_method);
  // std::cout << "hellooooooo" << std::endl;
  my_ccd.read_params(params_file_path);
  my_ccd.init_mat();
  
  my_ccd.run_ccd();
  cv::imshow("CCD", my_ccd.canvas);
  while (1)
  {
    key = cv::waitKey(10);
    if (key == 27) break;
  }
  // double interval = (pts.size() - params[9])/params[8];
  // std::cout << "resolution: " << params[8] << " pts_number - degree: " << (pts.size() - params[9]) << " increment: " <<  interval  << " interval " << params[8]/(pts.size() - params[9]) << std::endl;
  return 0;
}
