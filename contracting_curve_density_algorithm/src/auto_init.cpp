#include <opencv/cv.h>
#include <opencv/cvaux.h>
#include <opencv/highgui.h>
#include "auto_init.h"
#include <iostream>


void AutoInit::extract_descriptors(cv::Mat& img1, cv::Mat& img2)
{
  // cv::FeatureDetector* fd  = new cv::SiftFeatureDetector(cv::SIFT::DetectorParams::GET_DEFAULT_THRESHOLD(),
  //                                                cv::SIFT::DetectorParams::GET_DEFAULT_EDGE_THRESHOLD());
  // // ( double magnification=SIFT::DescriptorParams::GET_DEFAULT_MAGNIFICATION(),
  // //   bool isNormalize=true, bool recalculateAngles=true,
  // //   int nOctaves=SIFT::CommonParams::DEFAULT_NOCTAVES,
  // //   int nOctaveLayers=SIFT::CommonParams::DEFAULT_NOCTAVE_LAYERS,
  // //   int firstOctave=SIFT::CommonParams::DEFAULT_FIRST_OCTAVE,
  // //   int angleMode=SIFT::CommonParams::FIRST_ANGLE )
  // cv::DescriptorExtractor* de = new cv::SiftDescriptorExtractor;
  cv::FeatureDetector* fd  = new cv::SurfFeatureDetector(400);
  cv::DescriptorExtractor* de = new cv::SurfDescriptorExtractor(4,2,false);
  fd->detect(img1, keypoints1);
  fd->detect(img2, keypoints2);
  cv::Mat tpl;
  descriptors2.push_back(tpl);

  de->compute( img1, keypoints1, descriptors1);
  de->compute( img2, keypoints2, descriptors2[0]);
}

void AutoInit::match_descriptors()
{
  // cv::DescriptorMatcher *dm = new cv::FlannBasedMatcher;
  // dm->add(descriptors2);
  // dm->train();
  // dm->match(descriptors1, matches);

  cv::BruteForceMatcher<cv::L2<float> > dm;
  dm.add(descriptors2);
  dm.train();
  dm.match(descriptors1, matches);
  std::cout << "Found total matches: " << matches.size() << std::endl;  
}

cv::Mat AutoInit::compute_homography(
    cv::Mat& img1,
    cv::Mat& img2)
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
    //         cvPoint( cvRound( points2->data.db[i*2] ), cvRound( points2->data.db[i*2+1]) + img1->height ),
    //         CV_RGB(255,0,255), 1, 8, 0 );
  }

  // namedWindow("SIFTFEATUREdetector");
  // imshow("SIFTFEATUREdetector", stacked);
  // cvWaitKey( 0 );

  cv::Mat status;
  return cv::findHomography(points1, points2, CV_RANSAC, 3.0);
}



cv::Mat AutoInit::init(cv::Mat &img1,
                       cv::Mat &img2,
                       int inteval)
{
  char key;
  // cv::Mat stacked;
  cv::Mat coordinates, coordinates_t;
  cv::Mat xformed;
  cv::Mat descriptors1;
  std::vector<cv::Mat> descriptors2;
  double *ptr;
  int row, col;

  // stack_imgs(img1, img2, stacked);

  std::vector<cv::KeyPoint> keypoints1, keypoints2;
  std::vector<cv::DMatch> matches;


  extract_descriptors(img1, img2);
  match_descriptors();
  cv::Mat H = compute_homography(img1, img2);
  
  if(!H.empty())
  {
    // int   step  = H->step/sizeof(double);
    
    for (row = 0; row < H.rows; ++row){
      ptr = H.ptr<double>(row);
      std::cout << ptr[0] << " " << ptr[1] << " " << ptr[2] << std::endl;
    }
    /* printf("cont_n = %d\n", control_points_count); */


    if(inteval <= 0)
    {
      // CvFileStorage* fs= cvOpenFileStorage("contour.xml", 0, CV_STORAGE_READ);
      // coordinates= (CvMat*)cvReadByName(fs,  NULL, "contour_points", NULL);
      // coordinates_t = cvCreateMat(3 ,coordinates->rows,  CV_64FC1);
      // printf("contour_points number : %d\n", coordinates->rows);
      // step = coordinates->step/sizeof(double);
      // ptr = coordinates->data.db;
    }
    else
    {
      int control_points_count = ceil((double)img1.rows/inteval)*2+ ceil((double)img1.cols/inteval)*2;
      coordinates = cv::Mat::zeros(control_points_count, 3 , CV_64FC1);
      coordinates_t = cv::Mat::zeros(3 ,control_points_count,  CV_64FC1);
      int i = 0;
      for (row = inteval; row < img1.rows; row+=inteval){
        coordinates.ptr<double>(i)[0] = 0;
        coordinates.ptr<double>(i)[1] = row;
        coordinates.ptr<double>(i)[2] = 1;
        // (ptr+i*step)[0] = 0;
        // (ptr+i*step)[1] = row;
        // (ptr+i*step)[2] = 1;
        i++;
      }
      coordinates.ptr<double>(i)[0] = 0;
      coordinates.ptr<double>(i)[1] = img1.rows;
      coordinates.ptr<double>(i)[2] = 1;
      i++;
    
      for (col = inteval; col < img1.cols; col+=inteval){
        coordinates.ptr<double>(i)[0] = col;
        coordinates.ptr<double>(i)[1] = img1.rows;
        coordinates.ptr<double>(i)[2] = 1;
        i++;
      }
      coordinates.ptr<double>(i)[0] = img1.cols;
      coordinates.ptr<double>(i)[1] = img1.rows;
      coordinates.ptr<double>(i)[2] = 1;
      i++;
    
      for (row = img1.rows - inteval; row > 0 ; row-=inteval){
        coordinates.ptr<double>(i)[0] = img1.cols;
        coordinates.ptr<double>(i)[1] = row;
        coordinates.ptr<double>(i)[2] = 1;
        i++;
      }    
      coordinates.ptr<double>(i)[0] = img1.cols;
      coordinates.ptr<double>(i)[1] = 0;
      coordinates.ptr<double>(i)[2] = 1;
      i++;
      for (col = img1.cols-inteval; col > 0; col-=inteval){
        coordinates.ptr<double>(i)[0] = col;
        coordinates.ptr<double>(i)[1] = 0;
        coordinates.ptr<double>(i)[2] = 1;
        i++;
      }
      coordinates.ptr<double>(i)[0] = 0;
      coordinates.ptr<double>(i)[1] = 0;
      coordinates.ptr<double>(i)[2] = 1;
      i++;
    }

    
    cv::gemm(H, coordinates, 1, cv::Mat(), 0, coordinates_t, cv::GEMM_2_T);

    cv::Mat xformed = cv::Mat( img2.size(), CV_8UC3);
    printf("image2 size %d %d \n", img2.cols, img2.rows);
    // printf("stacked size %d %d \n", stacked.cols, stacked.rows);
    cv::warpPerspective( img1, xformed, H, xformed.size(), cv::INTER_LINEAR , cv::BORDER_CONSTANT, cv::Scalar());
    // cvTranspose(coordinates_t, coordinates);
    coordinates = coordinates_t.t();
    for (row = 0; row < coordinates.rows; ++row){
      ptr = coordinates.ptr<double>(row);
      // std::cout << ptr[0] << " " << ptr[1] << " "<< ptr[2] << std::endl;
      cv::circle(xformed, cvPoint(ptr[0]/ptr[2], ptr[1]/ptr[2]), 2, CV_RGB(0,255,0), 2, 8, 0);
    }
    // cvReleaseMat(&coordinates_t);
    
    cv::namedWindow( "CCD", 1 );
    cv::imshow( "CCD", xformed);
    while (1)
    {
      key = cv::waitKey(10);
      if (key == 27) break;
    }
    xformed.release();
    H.release();
	// cvReleaseImage( &xformed );
	// cvReleaseMat( &H );
  }

  // cvSaveImage("stacked.jpg", stacked,0);
  // cvReleaseImage( &stacked );
  // stacked.release();
  
  coordinates_t.release();
  return coordinates;
}


// void stack_imgs( cv::Mat &img1,
//                  cv::Mat &img2,
//                  cv::Mat &stacked)
// {
//   int max_width = MAX(img1.cols, img2.cols);
//   int x_start = (max_width - img1.cols)*0.5;
//   stacked = cv::Mat::zeros( cv::Size( max_width, img1.rows + img2.rows), CV_8UC3);
//   // int i,j;
//   // for (i = 0; i < img1->height; i++)
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
//   cv::Mat roi1(stacked, cv::Rect( x_start, 0, img1.cols, img1.rows ) );
//   roi1 = img1;
//   cv::Mat roi2( stacked, cv::Rect(0, img1.rows, img2.cols, img2.rows) );
//   roi2 = img2;
// }
