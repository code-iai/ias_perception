#include "sift.h"
#include "imgfeatures.h"
#include "kdtree.h"
#include "utils.h"
#include "xform.h"

#include <cv.h>
#include <cxcore.h>
#include <highgui.h>
#include "ccd/sift_init.h"
/* the maximum number of keypoint NN candidates to check during BBF search */
#define KDTREE_BBF_MAX_NN_CHKS 200
/* threshold on squared ratio of distances between NN and 2nd NN */
#define NN_SQ_DIST_RATIO_THR 0.49

CvMat sift_init(IplImage *img1, IplImage *img2, int inteval)
{
  IplImage * stacked;
  struct feature* feat1, * feat2, * feat;
  struct feature** nbrs;
  struct kd_node* kd_root;
  CvPoint pt1, pt2;
  double d0, d1;
  int n1, n2, k, i, m = 0;
  CvMat* H;
  CvMat *coordinates = 0, *coordinates_t = 0;
  IplImage* xformed;
  double *ptr;
  int row, col;
  char key;
    

  stacked = stack_imgs( img1, img2 );

  n1 = sift_features( img1, &feat1 );
  n2 = sift_features( img2, &feat2 );
  kd_root = kdtree_build( feat2, n2 );
  for( i = 0; i < n1; i++ )
  {
    feat = feat1 + i;
    k = kdtree_bbf_knn( kd_root, feat, 2, &nbrs, KDTREE_BBF_MAX_NN_CHKS );
    if( k == 2 )
	{
	  d0 = descr_dist_sq( feat, nbrs[0] );
	  d1 = descr_dist_sq( feat, nbrs[1] );
	  if( d0 < d1 * NN_SQ_DIST_RATIO_THR )
      {
        pt1 = cvPoint( cvRound( feat->x ), cvRound( feat->y ) );
        pt2 = cvPoint( cvRound( nbrs[0]->x ), cvRound( nbrs[0]->y ) );
        pt2.y += img1->height;
        cvLine( stacked, pt1, pt2, CV_RGB(255,0,255), 1, 8, 0 );
        m++;
        feat1[i].fwd_match = nbrs[0];
      }
	}
    free( nbrs );
  }

  fprintf( stderr, "Found %d total matches\n", m );
  display_big_img( stacked, "Matches" );
  cvWaitKey( 0 );

  /* 
     UNCOMMENT BELOW TO SEE HOW RANSAC FUNCTION WORKS
     
     Note that this line above:
     
     feat1[i].fwd_match = nbrs[0];
     
     is important for the RANSAC function to work.
  */
  H = ransac_xform( feat1, n1, FEATURE_FWD_MATCH, lsq_homog, 4, 0.01,
                    homog_xfer_err, 3.0, NULL, NULL );
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


    i = 0;
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
  kdtree_release( kd_root );
  free( feat1 );
  free( feat2 );
  return *coordinates;
}
