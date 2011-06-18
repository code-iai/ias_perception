#include <opencv/cv.h>
#include <opencv/cvaux.h>
#include <opencv/highgui.h>
#include "auto_init.h"
#include <iostream>

AutoInit::AutoInit():dp_extract_method(0),dp_match_method(0), inteval(0){}
AutoInit::AutoInit(int dpem, int dpmm, int itv):dp_extract_method(dpem>=0 ? dpem : 0), dp_match_method(dpmm >= 0 ? dpmm : 0),inteval(itv){}
void AutoInit::extract_descriptors(cv::Mat& tpl, cv::Mat& training_img)
{
  cv::FeatureDetector *fd = 0;
  cv::DescriptorExtractor *de = 0;
  if(dp_extract_method == DP_EXTRACT_SIFT)
  {
    fd = new cv::SiftFeatureDetector(cv::SIFT::DetectorParams::GET_DEFAULT_THRESHOLD(),
                                     cv::SIFT::DetectorParams::GET_DEFAULT_EDGE_THRESHOLD());
    // ( double magnification=SIFT::DescriptorParams::GET_DEFAULT_MAGNIFICATION(),
    //   bool isNormalize=true, bool recalculateAngles=true,
    //   int nOctaves=SIFT::CommonParams::DEFAULT_NOCTAVES,
    //   int nOctaveLayers=SIFT::CommonParams::DEFAULT_NOCTAVE_LAYERS,
    //   int firstOctave=SIFT::CommonParams::DEFAULT_FIRST_OCTAVE,
    //   int angleMode=SIFT::CommonParams::FIRST_ANGLE )
    de = new cv::SiftDescriptorExtractor;
  }
  else if(dp_extract_method == DP_EXTRACT_SURF)
  {
    fd  = new cv::SurfFeatureDetector(400);
    de = new cv::SurfDescriptorExtractor(4,2,false);
  }
  fd->detect(tpl, keypoints1);
  fd->detect(training_img, keypoints2);
  cv::Mat query;
  descriptors2.push_back(query);

  de->compute( tpl, keypoints1, descriptors1);
  de->compute( training_img, keypoints2, descriptors2[0]);
}

void AutoInit::match_descriptors()
{
  cv::DescriptorMatcher *dm = 0;
  if(dp_match_method == DP_MATCH_FLANN)
  {
    dm = new cv::FlannBasedMatcher;
    // dm->add(descriptors2);
    // dm->train();
    // dm->match(descriptors1, matches);
  }
  else if(dp_match_method == DP_MATCH_BRUTE)
  {
    cv::BruteForceMatcher<cv::L2<float> > bfm;
    dm = &bfm;
  }
  dm->add(descriptors2);
  dm->train();
  dm->match(descriptors1, matches);
  std::cout << "Found total matches: " << matches.size() << std::endl;  
}

cv::Mat AutoInit::compute_homography(
    cv::Mat& tpl,
    cv::Mat& training_img)
{
  cv::Mat points1 = cv::Mat(1,matches.size(),CV_64FC2);
  cv::Mat points2 = cv::Mat(1,matches.size(),CV_64FC2);
  cv::Mat_<cv::Vec2d>& ptr1 = (cv::Mat_<cv::Vec2d>&)points1;
  cv::Mat_<cv::Vec2d>& ptr2 = (cv::Mat_<cv::Vec2d>&)points2;
  
  for (size_t i = 0; i < matches.size(); ++i)
  {
    ptr1(0, i)[0] = keypoints1[matches[i].queryIdx].pt.x;
    ptr1(0, i)[1] = keypoints1[matches[i].queryIdx].pt.y;
    ptr2(0, i)[0] = keypoints2[matches[i].trainIdx].pt.x;
    ptr2(0, i)[1] = keypoints2[matches[i].trainIdx].pt.y;
    // cvLine( stacked, cvPoint(cvRound( points1->data.db[i*2] ), cvRound( points1->data.db[i*2+1])),
    //         cvPoint( cvRound( points2->data.db[i*2] ), cvRound( points2->data.db[i*2+1]) + tpl->height ),
    //         CV_RGB(255,0,255), 1, 8, 0 );
  }

  // namedWindow("SIFTFEATUREdetector");
  // imshow("SIFTFEATUREdetector", stacked);
  // cvWaitKey( 0 );

  cv::Mat status;
  return cv::findHomography(points1, points2, CV_RANSAC, 3.0);
}

void AutoInit::set_control_points(cv::Mat &tpl)
{
  int row, col;
  if(inteval <= 0)
  {
    cv::FileStorage fs("contour.xml", cv::FileStorage::READ);
    fs["contour_points"] >> control_points ;
    
    printf("contour_points number : %d\n", control_points.rows);
  }
  else
  {
    int control_points_count = ceil((double)tpl.rows/inteval)*2+ ceil((double)tpl.cols/inteval)*2;
    control_points = cv::Mat::zeros(control_points_count, 3 , CV_64FC1);
    int i = 0;
    for (row = inteval; row < tpl.rows; row+=inteval){
      control_points.ptr<double>(i)[0] = 0;
      control_points.ptr<double>(i)[1] = row;
      control_points.ptr<double>(i)[2] = 1;
      // (ptr+i*step)[0] = 0;
      // (ptr+i*step)[1] = row;
      // (ptr+i*step)[2] = 1;
      i++;
    }
    control_points.ptr<double>(i)[0] = 0;
    control_points.ptr<double>(i)[1] = tpl.rows;
    control_points.ptr<double>(i)[2] = 1;
    i++;
    for (col = inteval; col < tpl.cols; col+=inteval){
      control_points.ptr<double>(i)[0] = col;
      control_points.ptr<double>(i)[1] = tpl.rows;
      control_points.ptr<double>(i)[2] = 1;
      i++;
    }
    control_points.ptr<double>(i)[0] = tpl.cols;
    control_points.ptr<double>(i)[1] = tpl.rows;
    control_points.ptr<double>(i)[2] = 1;
    i++;
    
    for (row = tpl.rows - inteval; row > 0 ; row-=inteval){
      control_points.ptr<double>(i)[0] = tpl.cols;
      control_points.ptr<double>(i)[1] = row;
      control_points.ptr<double>(i)[2] = 1;
      i++;
    }    
    control_points.ptr<double>(i)[0] = tpl.cols;
    control_points.ptr<double>(i)[1] = 0;
    control_points.ptr<double>(i)[2] = 1;
    i++;
    for (col = tpl.cols-inteval; col > 0; col-=inteval){
      control_points.ptr<double>(i)[0] = col;
      control_points.ptr<double>(i)[1] = 0;
      control_points.ptr<double>(i)[2] = 1;
      i++;
    }
    control_points.ptr<double>(i)[0] = 0;
    control_points.ptr<double>(i)[1] = 0;
    control_points.ptr<double>(i)[2] = 1;
    i++;
  }
}

void AutoInit::init(cv::Mat &tpl,
                    cv::Mat &training_img)
{
  char key;
  double *ptr;
  cv::Mat H, control_points_t, xformed;
  std::vector<cv::DMatch> matches;
  
  extract_descriptors(tpl, training_img);
  match_descriptors();
  H = compute_homography(tpl, training_img);
  set_control_points(tpl);
  cv::gemm(H, control_points, 1, cv::Mat(), 0, control_points_t, cv::GEMM_2_T);

  // for (row = 0; row < H.rows; ++row){
  //   ptr = H.ptr<double>(row);
  //   std::cout << ptr[0] << " " << ptr[1] << " " << ptr[2] << std::endl;
  // }
  /* printf("cont_n = %d\n", control_points_count); */
  
  xformed = cv::Mat( training_img.size(), CV_8UC3);
  cv::warpPerspective( tpl, xformed, H, xformed.size(), cv::INTER_LINEAR , cv::BORDER_CONSTANT, cv::Scalar());
  control_points = control_points_t.t();
  for (int row = 0; row < control_points.rows; ++row){
    ptr = control_points.ptr<double>(row);
    // std::cout << ptr[0] << " " << ptr[1] << " "<< ptr[2] << std::endl;
    cv::circle(xformed, cvPoint(ptr[0]/ptr[2], ptr[1]/ptr[2]), 2, CV_RGB(0,255,0), 2, 8, 0);
  }
  cv::namedWindow( "CCD", 1 );
  cv::imshow( "CCD", xformed);
  while (1)
  {
    key = cv::waitKey(10);
    if (key == 27) break;
  }
  xformed.release();
  H.release();
  control_points_t.release();
}


// void stack_imgs( cv::Mat &tpl,
//                  cv::Mat &training_img,
//                  cv::Mat &stacked)
// {
//   int max_width = MAX(tpl.cols, training_img.cols);
//   int x_start = (max_width - tpl.cols)*0.5;
//   stacked = cv::Mat::zeros( cv::Size( max_width, tpl.rows + training_img.rows), CV_8UC3);
//   // int i,j;
//   // for (i = 0; i < tpl->height; i++)
//   // {
//   //   uchar* ptr = (uchar*) (stacked->imageData + i * stacked->widthStep);
//   //   for (j = 0; j < x_start; j++)
//   //   {
//   //     ptr[3*j] = 100;
//   //     ptr[3*j + 1] = 100;
//   //     ptr[3*j + 2] = 100;
//   //   }
//   //   for (j = max_width - x_start; j < max_width; j++)
//   //   {
//   //     ptr[3*j] = 100;
//   //     ptr[3*j + 1] = 100;
//   //     ptr[3*j + 2] = 100;
//   //   }
//   // }
//   cv::Mat roi1(stacked, cv::Rect( x_start, 0, tpl.cols, tpl.rows ) );
//   roi1 = tpl;
//   cv::Mat roi2( stacked, cv::Rect(0, tpl.rows, training_img.cols, training_img.rows) );
//   roi2 = training_img;
// }
