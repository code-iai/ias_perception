#include "opencv/ml.h"
#include "opencv/highgui.h"
using namespace std;
using namespace cv;

int main( int argc, char** argv )
{
  cv::Mat image = cv::imread(argv[1]);
  const int MAX_CLUSTERS = 10;
  Scalar colorTab[] =
      {
        Scalar(0, 0, 255),
        Scalar(0,255,0),
        Scalar(255,100,100),
        Scalar(255,0,255),
        Scalar(0,255,255),
        Scalar(1,55,25),
        Scalar(0,255,100),
        Scalar(5,9,199),
        Scalar(55,255,55),
        Scalar(4,25,55)
      };      

  Mat imageShow= Mat::zeros(image.rows, image.cols, CV_8UC3);
  int sampleCount = image.rows*image.cols;
  int clusterCount = MIN(atoi(argv[2]), MAX_CLUSTERS);

  Mat points(sampleCount, 1, CV_32FC3), labelsKmeans;
  cv::Mat_<cv::Vec3b>& img = (cv::Mat_<cv::Vec3b>&)image;
  cv::Mat_<cv::Vec3b>& imgS = (cv::Mat_<cv::Vec3b>&)imageShow;
  
  double *pPtr;
  for (int i = 0 ; i < image.rows; ++i)
  {
    for (int j = 0; j < image.cols; ++j)
    {
      pPtr = points.ptr<double>(i*image.cols + j);
      pPtr[0] = img(i,j)[0];
      pPtr[1] = img(i,j)[1];
      pPtr[2] = img(i,j)[2];
    }
  }
  
  clusterCount = MIN(clusterCount, sampleCount);
  Mat centers(clusterCount, 1, points.type());
        
  kmeans(points, clusterCount, labelsKmeans, TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 10, 1.0), 3, KMEANS_PP_CENTERS, &centers);

  CvEM em_model;
  CvEMParams params;
  CvMat *samples = cvCreateMat( sampleCount, 3, CV_32FC1 );

  int step = samples->step/sizeof(CV_32F);

  float *ptr = samples->data.fl;
  for (int i = 0; i < sampleCount; ++i)
  {
    (ptr+i*step)[0] = points.ptr<double>(i)[0];
    (ptr+i*step)[1] = points.ptr<double>(i)[1];
    (ptr+i*step)[2] = points.ptr<double>(i)[2];
  }

  
  CvMat labelsMat = labelsKmeans;
  CvMat *labels = &labelsMat;
  
  CvMat *initMeans = cvCreateMat(clusterCount,3, CV_64FC1);
  cvZero(initMeans);
  double *mptr = initMeans->data.db;
  int dstep = initMeans->step/sizeof(CV_64F);

  std::vector<int> clusterPoints(clusterCount,0);
  for (int i = 0 ; i < sampleCount; ++i)
  {
    clusterPoints[labelsKmeans.at<int>(i)] += 1;
    (mptr+labelsKmeans.at<int>(i)*dstep)[0] += (ptr+i*step)[0];
    (mptr+labelsKmeans.at<int>(i)*dstep)[1] += (ptr+i*step)[1];
    (mptr+labelsKmeans.at<int>(i)*dstep)[2] += (ptr+i*step)[2];
  }
  
  for (int i = 0; i < clusterCount; ++i)
  {
    (mptr+i*dstep)[0] /= clusterPoints[i];
    (mptr+i*dstep)[1] /= clusterPoints[i];
    (mptr+i*dstep)[2] /= clusterPoints[i];
  }

  CvMat **clusters = (CvMat**)cvAlloc( clusterCount * sizeof(*clusters));
  for (int i ; i < clusterCount; ++i)
  {
    clusters[i] = cvCreateMat(clusterPoints[i], 3, CV_64FC1);
  }


  CvMat **initCovs = (CvMat**)cvAlloc( clusterCount * sizeof(*initCovs));
  for (int i = 0; i < clusterCount; ++i)
  {
    cvCalcCovarMatrix( , clusterPoints[i], initCovs[i], mptr+i*dstep, CV_COVAR_NORMAL | CV_COVAR_SCALE );
  }

  
  CvMat *initWeights = cvCreateMat(1, clusterCount, CV_64FC1);
  mptr = initWeights->data.db;
  for (int i = 0; i < clusterCount; ++i)
  {
    (mptr+i*dstep)[0] = clusterPoints[i]/sampleCount;
  }

  CvMat *initProbs = cvCreateMat(sampleCount, clusterCount, CV_64FC1);
  cvZero(initProbs);
  mptr = initProbs->data.db;
  
  for (int i = 0; i < sampleCount; ++i)
  {
    (mptr + i*dstep)[labelsKmeans.at<int>(i)] = 1;
  }

  params.covs      = NULL;
  params.means     = initMeans;
  params.weights   = initWeights;
  params.probs     = initProbs;
  params.nclusters = clusterCount;
  params.cov_mat_type       = CvEM::COV_MAT_SPHERICAL;
  params.start_step         = CvEM::START_AUTO_STEP;
  params.term_crit.max_iter = 10;
  params.term_crit.epsilon  = 0.1;
  params.term_crit.type     = CV_TERMCRIT_ITER|CV_TERMCRIT_EPS;

  // cluster the data
  em_model.train( samples, 0, params, labels);

#if 0
  // the piece of code shows how to repeatedly optimize the model
  // with less-constrained parameters
  //(COV_MAT_DIAGONAL instead of COV_MAT_SPHERICAL)
  // when the output of the first stage is used as input for the second.
  CvEM em_model2;
  params.cov_mat_type = CvEM::COV_MAT_DIAGONAL;
  params.start_step = CvEM::START_E_STEP;
  params.means = em_model.get_means();
  params.covs = (const CvMat**)em_model.get_covs();
  params.weights = em_model.get_weights();

  em_model2.train( samples, 0, params, labels );
  // to use em_model2, replace em_model.predict()
  // with em_model2.predict() below
#endif

  for (int i = 0 ; i < image.rows; ++i)
  {
    for (int j = 0; j < image.cols; ++j)
    {
      int imgRow = i*image.cols+j;
      imgS(i,j)[0] = colorTab[labelsKmeans.at<int>(imgRow)][0];
      imgS(i,j)[1] = colorTab[labelsKmeans.at<int>(imgRow)][1];
      imgS(i,j)[2] = colorTab[labelsKmeans.at<int>(imgRow)][2];
    }
  }  
  imshow("clusters", imageShow);
  imwrite("flowers_kmeans.png", imageShow);
  waitKey(0);

  cvReleaseMat( &samples);
  cvReleaseMat( &labels );  
  return 0;
}
