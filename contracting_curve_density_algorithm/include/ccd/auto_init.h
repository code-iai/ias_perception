/* Software License Agreement (BSD License)
 * 
 *   Copyright (c) 2011, Shulei Zhu <schuleichu@gmail.com>
 *   All rights reserved.
 * 
 *   Redistribution and use in source and binary forms, with or without
 *   modification, are permitted provided that the following conditions
 *   are met:
 * 
 *    * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *    * Redistributions in binary form must reproduce the above
 *      copyright notice, this list of conditions and the following
 *      disclaimer in the documentation and/or other materials provided
 *      with the distribution.
 *    * Neither the name of Shulei Zhu nor the names of its
 *      contributors may be used to endorse or promote products derived
 *      from this software without specific prior written permission.
 * 
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *   COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *   INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */
/* auto_init.h --- 
 * File            : auto_init.h
 * Created: Sa Jun 18 14:06:18 2011 (+0200)
 * Author: Shulei Zhu

/* Code: */
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
