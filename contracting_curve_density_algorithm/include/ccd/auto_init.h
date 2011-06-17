class AutoInit
{
 public:
  AutoInit();
  AutoInit(int fd):fd_method(fd){}
  std::vector<cv::DMatch> matches;
  cv::Mat compute_homography(cv::Mat& img1, cv::Mat& img2);
  cv::Mat init(cv::Mat &img1, cv::Mat &img2, int inteval);
 private:
  int fd_method;
  std::vector<cv::KeyPoint> keypoints1;
  std::vector<cv::KeyPoint> keypoints2;
  cv::Mat descriptors1;
  std::vector<cv::Mat> descriptors2;
  void extract_descriptors(cv::Mat& img1, cv::Mat& img2);
  void match_descriptors();
};
