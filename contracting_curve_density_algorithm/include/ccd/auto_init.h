#define DP_EXTRACT_SIFT 0
#define DP_EXTRACT_SURF 1
#define DP_MATCH_FLANN 0
#define DP_MATCH_BRUTE 1

class AutoInit
{
public:
  cv::Mat control_points;
  std::vector<cv::DMatch> matches;
  AutoInit();
  AutoInit(int dpem, int dpmm, int itv);
  cv::Mat compute_homography(cv::Mat &tpl, cv::Mat &training_img);
  void init(cv::Mat &tpl, cv::Mat &training_img);
private:
  int dp_extract_method;
  int dp_match_method;
  int inteval;
  std::vector<cv::KeyPoint> keypoints1;
  std::vector<cv::KeyPoint> keypoints2;
  cv::Mat descriptors1;
  std::vector<cv::Mat> descriptors2;
  void extract_descriptors(cv::Mat &tpl, cv::Mat &training_img);
  void match_descriptors();
  void set_control_points(cv::Mat &tpl);
};
