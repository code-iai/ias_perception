// Software License Agreement (BSD License)

//   Copyright (c) 2011, Shulei Zhu <schuleichu@gmail.com>
//   All rights reserved.

//   Redistribution and use in source and binary forms, with or without
//   modification, are permitted provided that the following conditions
//   are met:

//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//    * Redistributions in binary form must reproduce the above
//      copyright notice, this list of conditions and the following
//      disclaimer in the documentation and/or other materials provided
//      with the distribution.
//    * Neither the name of Shulei Zhu nor the names of its
//      contributors may be used to endorse or promote products derived
//      from this software without specific prior written permission.

//   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
//   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
//   COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//   INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
//   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
//   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
//   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//   POSSIBILITY OF SUCH DAMAGE.
// bspline.h --- 
// File            : bspline.h
// Created: Sa Jun 18 14:06:52 2011 (+0200)
// Author: Shulei Zhu

// Code:

class BSpline{
public:
  BSpline();
  /* BSpline(const BSpline &bs){} */
  BSpline(int n, int resolution, std::vector<cv::Point3d> control_points);
  ~BSpline();
  /* 
   * void release(){
   *     /\* basic_mat_.release(); *\/
   *     /\* 
   *      * if (curve_ != NULL) delete [] curve_;
   *      * if (tangent_ != NULL) delete [] tangent_;    
   *      *\/
   * }
   */

  /* void clear(){delete [] basic_mat_; delete [] curve_; delete [] tangent_;} */
  inline cv::Point3d& operator[](const size_t index)
  {
    return curve_[index];
  }

  inline const cv::Point3d& operator[](const size_t index) const
  {
    return curve_[index];
  }

  inline cv::Point3d& dt(const size_t index)
  {
    return tangent_[index];
  }
  cv::Mat basic_mat_;
private:
  void computeKnots();
  void computePoint(std::vector<cv::Point3d> control,
                    cv::Point3d *p,
                    cv::Point3d *tangent,
                    double *mat_ptr,
                    double v,
                    int degree);
  double basic(int k, int t, double v);
  double basic(int i, int degree, double t, double *bp);
  std::vector<int> knots;
  cv::Point3d *curve_;
  cv::Point3d *tangent_;
};
