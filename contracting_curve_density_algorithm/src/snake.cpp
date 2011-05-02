#include <cstdio>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <map>
#include <iostream>
#include <sstream>
#include <fstream>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cvaux.h>


#define KEY_ESC 1048603

int posFrame = 0;
int currPos = 0;

IplImage* image = 0;
std::vector<CvPoint> contourPoints;

/*
 * Threshold a color image
 */
void sum_rgb( IplImage* src, IplImage* dst ) {

  double minval, maxval;
  int threshold;

  // Allocate individual image  planes.
  IplImage* r = cvCreateImage(  cvGetSize(src), IPL_DEPTH_8U, 1 );
  IplImage* g = cvCreateImage(  cvGetSize(src), IPL_DEPTH_8U, 1 );
  IplImage* b = cvCreateImage(  cvGetSize(src), IPL_DEPTH_8U, 1 );
  // Split image onto the color planes.
  cvSplit( src, r, g, b, NULL );
  // Temporary storage.
  IplImage* s = cvCreateImage( cvGetSize(src), IPL_DEPTH_8U, 1 );
  // Add equally weighted rgb values.
  cvAddWeighted( r, 1./3., g, 1./3., 0.0, s );
  cvAddWeighted( s, 2./3., b, 1./3., 0.0, s );
  // Truncate values above 100.
  cvMinMaxLoc(s, &minval, &maxval);
  threshold = (int)((minval + maxval)/2);
  cvThreshold( s, dst, threshold, 255, CV_THRESH_BINARY );

  cvReleaseImage(  &r );
  cvReleaseImage(  &g );
  cvReleaseImage(  &b );
  cvReleaseImage(  &s );
}


/*
 * Convert hsv to rgb
 */
CvScalar hsv2rgb( float hue )
{
    int rgb[3], p, sector;
    static const int sector_data[][3]=
        {{0,2,1}, {1,2,0}, {1,0,2}, {2,0,1}, {2,1,0}, {0,1,2}};
    hue *= 0.033333333333333333333333333333333f;
    sector = cvFloor(hue);
    p = cvRound(255*(hue - sector));
    p ^= sector & 1 ? 255 : 0;

    rgb[sector_data[sector][0]] = 255;
    rgb[sector_data[sector][1]] = 0;
    rgb[sector_data[sector][2]] = p;

    return cvScalar(rgb[2], rgb[1], rgb[0],0);
}


void on_mouse( int event, int x, int y, int flags, void* param )
{
    if( !image )
        return;

    if( image->origin )
        y = image->height - y;

    switch( event )
    {
		case CV_EVENT_LBUTTONDOWN:
			//std::cout << "Event = " << event << std::endl;
			break;
		case CV_EVENT_LBUTTONUP:
			//std::cout << "Event = " << event << std::endl;
			cvCircle(image,cvPoint(x,y),2,cvScalar(0,0,255),2);
			contourPoints.push_back(cvPoint(x,y));
			cvShowImage("Original",image);
			break;
    }
}

int main(int argc, char** argv)
{

	float alpha = atof(argv[2]);
	float beta = atof(argv[3]);
	float gamma = atof(argv[4]);

	CvSize win;
	CvTermCriteria criteria;

    criteria.max_iter = 20; // Do max N iterations
    criteria.epsilon = 30; // If only N points is moved then terminate
    criteria.type = CV_TERMCRIT_EPS|CV_TERMCRIT_ITER;

    win.width = 5; // search for energy minimizing in this area around snake points
    win.height = 5; // Be sure that width and heigth is uneven

	contourPoints = std::vector<CvPoint>();

	//char* filename = "/home/robotcontrol/Desktop/camshift/sequence.avi";
	char* filename = argv[1];
	//char* filename = "/home/robotcontrol/Desktop/camshift/tracker.avi";
	//char* filename = "/home/robotcontrol/Desktop/camshift/rope1.avi";

	cvNamedWindow("Original", 1);
	// cvNamedWindow("Snake" , 1);

    cvSetMouseCallback( "Original", on_mouse, 0 );

	image = cvLoadImage(filename);

    IplImage *binary = cvCreateImage(cvGetSize(image), IPL_DEPTH_8U, 1);
    IplImage *dummy = cvCreateImage(cvGetSize(image), IPL_DEPTH_8U, 3);
    //canny = cvCreateImage(cvGetSize(frame), IPL_DEPTH_8U, 1);

    cvCvtColor(image, binary, CV_BGR2GRAY);
	cvShowImage("Original",image);
	// cvShowImage("Snake",binary);

	char key ;

	//choose the search object first

	// while (1)
	// {
	// 	key = cvWaitKey(10);
	// 	if (key == 27)
	// 	{
	// 		break;
	// 	}
	// }

	std::cout << "Click n to start snaking" << std::endl;
	//start tracking
	bool updated = true;
	while (1)
	{
		if (updated)
		{
			// real computational part of the algo
	        cvShowImage( "Original", image );
	        // cvShowImage("Snake", binary);
			updated = false;
		}

		// control the picture flow
		key = cvWaitKey(10);
		if (key == 27)
		{
			break;
		}

		if (key == 'n')
		{
			//std::cout << "alpha = " << alpha << ", beta = " << beta << ", gamma = " << gamma << std::endl;
			//std::cout << "Size = " << contourPoints.size() << std::endl;

			cvSnakeImage(binary, &contourPoints.at(0), contourPoints.size(), &alpha, &beta, &gamma, CV_VALUE, win, criteria, 1);

			for( int i=0; i<contourPoints.size(); i++ ) {
				// cvCircle(image,contourPoints[i],1,cvScalar(255,0,0),0);
              int j = (i+1)%contourPoints.size();
              cvLine(image, contourPoints[i], contourPoints[j], cvScalar(255,0,0),1,CV_AA);
			}
			updated = true;
		}
	}

	cvDestroyWindow("Original");
	std::cout << "Done." << std::endl;

	return 0;
}

