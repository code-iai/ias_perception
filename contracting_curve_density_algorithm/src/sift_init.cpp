#include <opencv/cv.h>
#include <opencv/cvaux.h>
#include <opencv/highgui.h>
#include <iostream>
using namespace cv;
using namespace std;
IplImage* stack_imgs( IplImage* img1, IplImage* img2 )
{
  IplImage* stacked = cvCreateImage( cvSize( MAX(img1->width, img2->width),
                                             img1->height + img2->height ),
                                     IPL_DEPTH_8U, 3 );

  cvZero( stacked );
  cvSetImageROI( stacked, cvRect( 0, 0, img1->width, img1->height ) );
  cvAdd( img1, stacked, stacked, NULL );
  cvSetImageROI( stacked, cvRect(0, img1->height, img2->width, img2->height) );
  cvAdd( img2, stacked, stacked, NULL );
  cvResetImageROI( stacked );

  return stacked;
}


CvMat sift_init(IplImage *img1, IplImage *img2, int inteval)
{
  IplImage * stacked;
  vector<KeyPoint> keypoints1, keypoints2;
  
  CvMat* H;
  CvMat *coordinates = 0, *coordinates_t = 0;
  IplImage* xformed;
  double *ptr;
  int row, col;
  char key;
    

  stacked = stack_imgs( img1, img2 );

  
  FeatureDetector* fd  = new SiftFeatureDetector(SIFT::DetectorParams::GET_DEFAULT_THRESHOLD(),
                                                 SIFT::DetectorParams::GET_DEFAULT_EDGE_THRESHOLD());
  DescriptorExtractor* de = 0;
  de = new SiftDescriptorExtractor/*( double magnification=SIFT::DescriptorParams::GET_DEFAULT_MAGNIFICATION(),
                                    bool isNormalize=true, bool recalculateAngles=true,
                                    int nOctaves=SIFT::CommonParams::DEFAULT_NOCTAVES,
                                    int nOctaveLayers=SIFT::CommonParams::DEFAULT_NOCTAVE_LAYERS,
                                    int firstOctave=SIFT::CommonParams::DEFAULT_FIRST_OCTAVE,
                                    int angleMode=SIFT::CommonParams::FIRST_ANGLE )*/;

  fd->detect(img1, keypoints1);
  fd->detect(img2, keypoints2);
  std::vector<cv::Mat> descriptors1;
  cv::Mat tpl;
  descriptors1.push_back(tpl);

  cv::Mat descriptors2;
  de->compute( img1, keypoints1, descriptors1[0]);
  de->compute( img2, keypoints2, descriptors2);
  DescriptorMatcher *dm = new FlannBasedMatcher;
  dm->add(descriptors1);
  dm->train();

  std::vector<cv::DMatch> matches;

  dm->match(descriptors2, matches);


  std::cout << "Found total matches: " << matches.size() << std::endl;
  namedWindow("SIFTFEATUREdetector");
  imshow("SIFTFEATUREdetector", stacked);
  cvWaitKey( 0 );

  /* 
     UNCOMMENT BELOW TO SEE HOW RANSAC FUNCTION WORKS
     
     Note that this line above:
     
     feat1[i].fwd_match = nbrs[0];
     
     is important for the RANSAC function to work.
  */

  /* for kk=1 to matches.size()

       the best match for queryKeypoints[matches[kk].queryIdx].pt 
       is dbKeypoints[matches[kk].imgIdx][matches[kk].trainIdx].pt

 */
  
  CvMat *points1 = cvCreateMat(1,matches.size(),CV_64FC2);
  CvMat *points2 = cvCreateMat(1,matches.size(),CV_64FC2);

  for (size_t i = 0; i < matches.size(); ++i)
  {
    points1->data.db[i*2] = keypoints2[matches[i].queryIdx].pt.x;
    points1->data.db[i*2+1] = keypoints2[matches[i].queryIdx].pt.y;
    points2->data.db[i*2] = keypoints1[matches[i].trainIdx].pt.x;
    points2->data.db[i*2+1] = keypoints1[matches[i].trainIdx].pt.y;
  }

  CvMat *status = cvCreateMat(1,matches.size(),CV_8UC1);
  cvFindFundamentalMat(points1, points2, H, CV_FM_RANSAC, 1.0, 0.99, status);
  if( H )
  {
    int   step  = H->step/sizeof(double);
    /* 
     * for (row = 0; row < H->rows; ++row){
     *   double *ptr = H->data.db;
     *   for (col = 0; col < H->cols; ++col){
     *     printf("%f ", (ptr+row*step)[col]);
     *   }
     *   printf("\n");
     * }
     */
    /* printf("cont_n = %d\n", control_points_number); */


    int i = 0;
    if(inteval <= 0)
    {
      CvFileStorage* fs= cvOpenFileStorage("contour.xml", 0, CV_STORAGE_READ);
      coordinates= (CvMat*)cvReadByName(fs,  NULL, "contour_points", NULL);
      coordinates_t = cvCreateMat(3 ,coordinates->rows,  CV_64FC1);
      printf("contour_points number : %d\n", coordinates->rows);
      step = coordinates->step/sizeof(double);
      ptr = coordinates->data.db;
    }
    else
    {
      int control_points_number = ceil((double)img1->height/inteval)*2+ ceil((double)img1->width/inteval)*2;
      coordinates = cvCreateMat(control_points_number, 3 , CV_64FC1);
      coordinates_t = cvCreateMat(3 ,control_points_number,  CV_64FC1);
      step = coordinates->step/sizeof(double);
      ptr = coordinates->data.db;
      for (row = inteval; row < img1->height; row+=inteval){
        (ptr+i*step)[0] = 0;
        (ptr+i*step)[1] = row;
        (ptr+i*step)[2] = 1;
        i++;
      }
      (ptr+i*step)[0] = 0;
      (ptr+i*step)[1] = img1->height;
      (ptr+i*step)[2] = 1;
      i++;
    
      for (col = inteval; col < img1->width; col+=inteval){
        (ptr+i*step)[0] = col;
        (ptr+i*step)[1] = img1->height;
        (ptr+i*step)[2] = 1;
        i++;
      }
      (ptr+i*step)[0] = img1->width;
      (ptr+i*step)[1] = img1->height;
      (ptr+i*step)[2] = 1;
      i++;
    
      for (row = img1->height - inteval; row > 0 ; row-=inteval){
        (ptr+i*step)[0] = img1->width;
        (ptr+i*step)[1] = row;
        (ptr+i*step)[2] = 1;
        i++;
      }    
      (ptr+i*step)[0] = img1->width;
      (ptr+i*step)[1] = 0;
      (ptr+i*step)[2] = 1;
      i++;
      for (col = img1->width-inteval; col > 0; col-=inteval){
        (ptr+i*step)[0] = col;
        (ptr+i*step)[1] = 0;
        (ptr+i*step)[2] = 1;
        i++;
      }
      (ptr+i*step)[0] = 0;
      (ptr+i*step)[1] = 0;
      (ptr+i*step)[2] = 1;
      i++;
    }
    /* printf("i == %d \n", i); */

    
    cvGEMM(H, coordinates, 1, 0,0, coordinates_t, CV_GEMM_B_T);


	xformed = cvCreateImage( cvGetSize( img2 ), IPL_DEPTH_8U, 3 );
    printf("image2 size %d %d \n", img2->width, img2->height);
    printf("stacked size %d %d \n", stacked->width, stacked->height);
	cvWarpPerspective( img1, xformed, H, 
                       CV_INTER_LINEAR + CV_WARP_FILL_OUTLIERS,
                       cvScalarAll( 0 ) );
    cvTranspose(coordinates_t, coordinates);
    for (row = 0; row < coordinates->rows; ++row){
      cvCircle(xformed, cvPoint((ptr+step*row)[0]/(ptr+step*row)[2], (ptr+step*row)[1]/(ptr+step*row)[2]), 2, CV_RGB(0,255,0), 2, 8, 0);
    }
    cvReleaseMat(&coordinates_t);
    
	cvNamedWindow( "CCD", 1 );
	cvShowImage( "CCD", xformed );
    while (1)
    {
      key = cvWaitKey(10);
      if (key == 27) break;
    }    
	cvReleaseImage( &xformed );
	cvReleaseMat( &H );
  }

  cvSaveImage("stacked.jpg", stacked,0);
  cvReleaseImage( &stacked );
  return *coordinates;
}
