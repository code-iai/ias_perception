// 
// Software License Agreement (BSD License)
// 
//   Copyright (c) 2011, Shulei Zhu <schuleichu@gmail.com>
//   All rights reserved.
// 
//   Redistribution and use in source and binary forms, with or without
//   modification, are permitted provided that the following conditions
//   are met:
// 
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//    * Redistributions in binary form must reproduce the above
//      copyright notice, this list of conditions and the following
//      disclaimer in the documentation and/or other materials provided
//      with the distribution.
//    * Neither the name of Shulei Zhu nor the names of its
//      contributors may be used to endorse or promote products derived
//      from this software without specific prior written permission.
// 
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
// 
// 
// bspline.cpp --- 
// File            : bspline.cpp
// Created: Sa Jun 18 14:04:27 2011 (+0200)
// Author: Shulei Zhu

// Code:

#include <cv.h>
#include "ccd/bspline.h"
double BSpline::basic(int i,
                      int k,
                      double t)
{
  double value;
  if (k == 1)
    ((t >= knots[i]) && (t<knots[i+1]))? value = 1 : value = 0;
  else{
    if((knots[i+k-1] == knots[i]) && (knots[i+k] == knots[i+1])) value = 0;
    else if(knots[i+k-1] == knots[i]) value = (knots[i+k] -t )/(knots[i+k] -knots[i+1])*basic(i+1,k -1 , t);
    else if(knots[i+k] == knots[i+1]) value = (t - knots[i])/(knots[i+k-1] - knots[i]) * basic(i,k-1, t);
    else value = (t - knots[i])/(knots[i+k-1] - knots[i]) * basic(i,k-1, t) + (knots[i+k] -t )/(knots[i+k] -knots[i+1])*basic(i+1,k -1 , t);
  }
  return value;
}

double BSpline::basic(int i,
                      int degree,
                      double t,
                      double *bp)
{
  double temp = 0;
  double t_floor = t-floor(t);
  // std::cout << " t_floor: " << t_floor<<std::endl;
  if(degree == 3)
  {
    if(t -i >= 2 && t- i <= 3)
    {
      temp = 0.5*(1- t_floor)*(1-t_floor);
      *bp = t_floor - 1;
    }
    else if (t - i >= 1 && t-i< 2)
    {
      temp = -t_floor*t_floor + t_floor + 0.5;
      *bp = 1 - 2*t_floor;
    }
    else if (t-i >= 0 && t-i < 1)
    {
      temp = 0.5*t_floor*t_floor;
      *bp = t_floor;
    }
    else
    {
      temp = 0;
      *bp = 0;
    }
  }
  else if(degree == 4)
  {
    
    if((t -i >= 3 && t- i <= 4))
    {
      temp = (-t_floor*t_floor*t_floor+3*t_floor*t_floor - 3*t_floor + 1)/6;
      *bp = -0.5*t_floor*t_floor + t_floor - 0.5;
    }
    else if (t - i >= 2 && t-i< 3)
    {
      temp = (3*t_floor*t_floor*t_floor - 6*t_floor*t_floor + 4)/6;
      *bp = 1.5*t_floor*t_floor - 2*t_floor;
    }
    else if (t-i >= 1 && t-i < 2)
    {
      temp = (-3*t_floor*t_floor*t_floor + 3*t_floor*t_floor + 3*t_floor +1)/6;
      *bp = -1.5*t_floor*t_floor + t_floor + 0.5;
    }
    else if(t-i >= 0 && t-i < 1)
    {
      temp = t_floor*t_floor*t_floor/6;
      *bp = 0.5*t_floor*t_floor;
    }
    else
    {
      temp = 0;
      *bp = 0;
    }    
  }
  return temp;
}


void BSpline::computeKnots()
{
    
  for (size_t j = 0; j < knots.size() ; ++j){
    knots[j] = j;
    // if (j < n_order)
    //     knots[j] = 0;
    // else if ((j >= n_order) && (j <= n_control_points))
    //     knots[j] = j-n_order + 1;
    // else if (j > n_control_points)
    //     knots[j] = n_control_points - n_order + 2;
    // std::cout << knots[j] << std::endl;
  }
}

void BSpline::computePoint(
    std::vector<cv::Point3d> control,
    cv::Point3d *output,
    cv::Point3d *slope,
    double *mat_ptr,
    double t,
    int degree)
{
  double b = 0, bp = 0;
  // initialize the variables that will hold our outputted point
  output->x=0;
  output->y=0;
  output->z=0;
  slope->x = 0;
  slope->y = 0;
  slope->z = 0;
  for (size_t i = 0; i < control.size(); i++)
  {
    // b = basic(i, n_order_, t);
    b = basic(i, degree, t, &bp);
    mat_ptr[i] = b;
    output->x += control[i].x * b;
    output->y += control[i].y * b;
    output->z += control[i].z * b;
    slope->x += (control[i]).x * bp;
    slope->y += (control[i]).y * bp;
    slope->z += (control[i]).z * bp;
  }
}  
BSpline::BSpline():curve_(NULL), tangent_(NULL){}

BSpline::BSpline(int n,
                 int resolution,
                 std::vector<cv::Point3d> control)
    :basic_mat_(cv::Mat(resolution, control.size(), CV_64FC1)),
     knots(std::vector<int>(control.size()+n, 0)),
  curve_((n>0 && resolution > 0)? new cv::Point3d[resolution]:NULL),
  tangent_((n>0 && resolution > 0)? new cv::Point3d[resolution]:NULL)
{
  double increment, interval;
  cv::Point3d tmp_point, tmp_tangent;
  int m = control.size() - 1;
  computeKnots();
  increment = (double) (m - n + 1)/resolution;
  // interval = 0;
  // std::cout <<  "increment << " << increment << std::endl;  
  // for (interval = n-1; fabs(interval - m) > 0.0000001 ; ++i){
  interval = n -1;
  for (int i = 0; i < resolution; ++i){
    double *mat_ptr = basic_mat_.ptr<double>(i);
    computePoint(control, &tmp_point, &tmp_tangent, mat_ptr, interval, n);
    curve_[i].x = tmp_point.x;
    curve_[i].y = tmp_point.y;
    curve_[i].z = tmp_point.z;
    // if(i<20)
    //   std::cout << interval <<"i: " <<i << " x " << round(tmp_point.x) << " y "<< round(tmp_point.y) << std::endl;
    // std::cout <<  interval << "       i: " << i << " x " << curve_[i].x << " y "<< curve_[i].y << std::endl;
    tangent_[i].x = tmp_tangent.x;
    tangent_[i].y = tmp_tangent.y;
    tangent_[i].z = tmp_tangent.z;
    // double min = (double)(100%((int)round(increment*100)))/100.0;
    // std::cout <<"increment: " << (int)round(increment*100) <<  " min = " << min << " interval - round(interval) : " << abs(interval - round(interval))<< std::endl;

    // if(abs(interval - round(interval)) <= min )
    // {
    //   std::cout << "interval: " << interval << std::endl;
    // }

    interval += increment;
  }
  // std::cout <<"resolution: " << resolution<<  " i = " << i<<  std::endl;
  // curve_[resolution-1].x=control[m].x;
  // curve_[resolution-1].y=control[m].y;
}


BSpline::~BSpline(){
       /* basic_mat_.release(); */
       /* knots.clear(); */
       if (curve_ != NULL) delete [] curve_;
       if (tangent_ != NULL) delete [] tangent_;
  }
