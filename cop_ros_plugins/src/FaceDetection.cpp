/*
 * Copyright (c) 2010, Ulrich Klank, Dejan Pangercic
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "FaceDetection.h"
#include "XMLTag.h"

#include <iostream>
#include <cstdio>

/*  . */
#include "IplImageReading.h"
/* */
#include "ImageSubscription.h"
/* */
#include "DetectedFace.h"

#define XML_ATTRIBUTE_CASCADE_NAME "cascade_name"
#define XML_ATTRIBUTE_NESTED_CASCADE_NAME "nested_cascade_name"
#define XML_ATTRIBUTE_SCALE "scale"


using namespace cop;

FaceDetection::FaceDetection()
{
  printf("FaceDetection created\n");
}

FaceDetection::~FaceDetection(void)
{
}

XMLTag* FaceDetection::Save()
{
  XMLTag* tag = new XMLTag(GetName());
  return tag;
}


void FaceDetection::SetData(XMLTag* tag)
{
  if(tag != NULL)
  {
    cascade_name_ = tag->GetProperty(XML_ATTRIBUTE_CASCADE_NAME, "");
    nested_cascade_name_ = tag->GetProperty(XML_ATTRIBUTE_NESTED_CASCADE_NAME, "");
    scale_ = tag->GetPropertyDouble(XML_ATTRIBUTE_SCALE, 1.5);

   if( !nested_cascade_.load( nested_cascade_name_ ) )
      cerr << "WARNING: Could not load classifier cascade for nested objects" << endl;

    if( !cascade_.load( cascade_name_ ) )
    {
      cerr << "ERROR: Could not load classifier cascade" << endl;
      cerr << "Usage: facedetect [--cascade=\"<cascade_path>\"]\n"
        "   [--nested-cascade[=\"nested_cascade_path\"]]\n"
        "   [--scale[=<image scale>\n"
        "   [filename|camera_index]\n" ;
      //return -1;
    }
  }
}

double angle_two_pixel(double x1, double y1, double x2, double y2, MinimalCalibration& calib)
{
  double X1 = (x1 - calib.proj_center_x)*calib.pix_size_x;
  double Y1 = (y1 - calib.proj_center_y)*calib.pix_size_y;
  double Z1 = calib.focal_length;
  double X2 = (x2 - calib.proj_center_x)*calib.pix_size_x;
  double Y2 = (y2 - calib.proj_center_y)*calib.pix_size_y;
  double Z2 = calib.focal_length;

  printf("in: x1 %f, y1 %f, x2 %f,y2 %f\n", x1, y1, x2, y2);
  printf("3d: x1 %f, y1 %f,z1 %f, x2 %f,y2 %f, z2 %f\n", X1, Y1, Z1, X2, Y2, Z2);
  double ret = (((X1 * X2) + (Y1 * Y2) + (Z1 * Z2)) / (sqrt( X1*X1 + Y1*Y1 + Z1*Z1) * sqrt(X2*X2 + Y2*Y2 + Z2*Z2)));
  printf("res: acos(%f)\n", ret);
  ret = acos(ret);
  printf("res: %f\n", ret);
  return ret;
}

RelPose* distance_to_3d(RelPose *sensor_pose, MinimalCalibration &calib, double length_min,
           double length_max, double x, double y, double cov_width, double cov_height)
{
  printf("istance_to_3d with lmin %f  lmax %f x %f y %f co_w %f co_h%f\n ",
  length_min, length_max, x, y, cov_width, cov_height);
  Matrix m(4,4), cov(6,6);
  double X, Y, Z;
  X = (x - calib.proj_center_x)*calib.pix_size_x;
  Y = (y - calib.proj_center_y)*calib.pix_size_y;
  Z = calib.focal_length;
  double norm = sqrt( X*X + Y*Y + Z*Z);

  X *= (length_max + length_min)/(2*norm);
  Y *= (length_max + length_min)/(2*norm);
  Z *= (length_max + length_min)/(2*norm);
  /* TODO rotate the matrix*/
  m << 1.0 << 0.0 << 0.0 << X
    << 0.0 << 1.0 << 0.0 << Y
    << 0.0 << 0.0 << 1.0 << Z
    << 0.0 << 0.0 << 0.0 << 1.0;

  cov << cov_width / 2 << 0.0 << 0.0 << 0.0 << 0.0 << 0.0
      << 0.0 << cov_height / 2<< 0.0 << 0.0 << 0.0 << 0.0
      << 0.0 << 0.0 << (length_max - length_min)/2 << 0.0 << 0.0 << 0.0
      << 0.0 << 0.0 << 0.0 << 0.0 << 0.0 << 0.0
      << 0.0 << 0.0 << 0.0 << 0.0 << 0.0 << 0.0
      << 0.0 << 0.0 << 0.0 << 0.0 << 0.0 << 0.0;
  return RelPoseFactory::FRelPose(sensor_pose, m, cov);
}



// Public attribute accessor methods
//
std::vector<RelPose*> FaceDetection::Perform(std::vector<Sensor*> sensors, RelPose* pose, Signature& object, int &numOfObjects, double& qualityMeasure)
{
  std::vector<RelPose*> results;
  int i = 0;
  double t = 0;
  vector<Rect> faces;
  DetectedFace* face_descr = (DetectedFace*)object.GetElement(0, DESCRIPTOR_DETECTEDFACE);
  try
  {
    ScopedCvMat_t image (sensors, ReadingType_IplImage);

    RelPose* sensor_pose = image.sensor_pose_at_capture_time;

    Mat gray, smallImg( cvRound ((*image).rows/scale_), cvRound((*image).cols/scale_), CV_8UC1 );
    cvtColor( (*image), gray, CV_BGR2GRAY );
    resize( gray, smallImg, smallImg.size(), 0, 0, INTER_LINEAR );
    equalizeHist( smallImg, smallImg );

    t = (double)cvGetTickCount();
    cascade_.detectMultiScale( smallImg, faces,
                                1.1, 2, 0
                                //|CV_HAAR_FIND_BIGGEST_OBJECT
                                //|CV_HAAR_DO_ROUGH_SEARCH
                                |CV_HAAR_SCALE_IMAGE
                                ,
                                Size(30, 30) );
      t = (double)cvGetTickCount() - t;
      printf( "detection time = %g ms\n", t/((double)cvGetTickFrequency()*1000.) );
      for( vector<Rect>::const_iterator r = faces.begin(); r != faces.end(); r++, i++ )
      {
        Mat smallImgROI;
        vector<Rect> nestedObjects;
        Point center;
        int radius;

        printf("Enter inner loop\n");

        center.x = cvRound((r->x + r->width*0.5)*scale_);
        center.y = cvRound((r->y + r->height*0.5)*scale_);
        radius = cvRound((r->width + r->height)*0.25*scale_);
        /*circle( (*image), center, radius, color, 3, 8, 0 );*/
        if( nested_cascade_.empty() )
          continue;
        smallImgROI = smallImg(*r);
        nested_cascade_.detectMultiScale( smallImgROI, nestedObjects,
                                        1.1, 2, 0
                                        //|CV_HAAR_FIND_BIGGEST_OBJECT
                                        //|CV_HAAR_DO_ROUGH_SEARCH
                                        //|CV_HAAR_DO_CANNY_PRUNING
                                        |CV_HAAR_SCALE_IMAGE
                                        ,
                                        Size(30, 30) );
        double max_face_height = 0.30;
        double min_face_height = 0.20;
        double max_face_width  = 0.25;
        double min_face_width = 0.15;
        printf("After nested cascade\n");

        vector<Rect>::const_iterator nr = nestedObjects.begin();
        if(nr == nestedObjects.end())
        {
          /*angle in x*/
          double alpha = angle_two_pixel(r->x ,
                                         r->y + r->height*0.5,
                                         r->x + r->width,
                                         r->y + r->height*0.5, image.calib);
          /*angle in y*/
          double beta =  angle_two_pixel(r->x + r->width*0.5,
                                         r->y,
                                         r->x + r->width*0.5,
                                         r->y + r->height, image.calib);
          /*Assuming floor aligned camera (in the x axis) and face size*/
          double length_min = min_face_height / (2* tan(alpha));
          double length_max = max_face_height / (2* tan(alpha));

          length_min = max(length_min, min_face_width / (2* tan(beta)));
          length_max = min(length_max, max_face_width / (2* tan(beta)));

          RelPose* pose = distance_to_3d(sensor_pose, image.calib, length_min, length_max,
                                        center.x, center.y, max_face_width - min_face_width,
                                        max_face_height - min_face_height);
          pose->m_qualityMeasure = 1.0;
          qualityMeasure = 1.0;
          results.push_back(pose);
          std::vector<double> vec;
          vec.push_back(center.x);
          vec.push_back(center.y);
          vec.push_back((r->width + r->height)*0.25*scale_);
          face_descr->m_lastlyDetectedFaces[pose->m_uniqueID] = (vec);
          if(results.size() == (unsigned)numOfObjects)
            break;
        }

        for( ; nr != nestedObjects.end(); nr++ )
        {

          printf("Innerst loop\n");

          center.x = cvRound((r->x + nr->x + nr->width*0.5)*scale_);
          center.y = cvRound((r->y + nr->y + nr->height*0.5)*scale_);

          /*angle in x*/
          double alpha = angle_two_pixel(r->x + nr->x,
                                         r->y + nr->y + nr->height*0.5,
                                         r->x + nr->x + nr->width,
                                         r->y + nr->y + nr->height*0.5, image.calib);
          /*angle in y*/
          double beta =  angle_two_pixel(r->x + nr->x + nr->width*0.5,
                                         r->y + nr->y,
                                         r->x + nr->x + nr->width*0.5,
                                         r->y + nr->y + nr->height, image.calib);
          /*Assuming floor aligned camera (in the x axis) and face size*/
          double length_min = min_face_height / (2* tan(alpha));
          double length_max = max_face_height / (2* tan(alpha));

          length_min = max(length_min, min_face_width / (2* tan(beta)));
          length_max = min(length_max, max_face_width / (2* tan(beta)));

          RelPose* pose = distance_to_3d(sensor_pose, image.calib, length_min, length_max,
                                        center.x, center.y, max_face_width - min_face_width,
                                        max_face_height - min_face_height);
          pose->m_qualityMeasure = 1.0;
          qualityMeasure = 1.0;
          results.push_back(pose);
          std::vector<double> vec;
          vec.push_back(center.x);
          vec.push_back(center.y);
          vec.push_back((nr->width + nr->height)*0.25*scale_);
          face_descr->m_lastlyDetectedFaces[pose->m_uniqueID] = (vec);
          if(results.size() == (unsigned)numOfObjects)
            break;
        }
      }
  }
  catch(const char* exception)
  {
    printf("FaceDetection failed: %s\n", exception);
  }
  if(results.size() != (unsigned)numOfObjects)
    numOfObjects = results.size();

  return results;
}

double FaceDetection::CheckSignature(const Signature& object, const std::vector<Sensor*> &sensors)
{
  printf("FaceDetection::CheckSignature\n");
  if(object.GetElement(0, DESCRIPTOR_DETECTEDFACE) != NULL)
  {
    for(size_t i = 0; i < sensors.size(); i++)
    {
      printf("Yes\n");

        return 1.0;
    }
  }
  printf("No\n");
  return 0.0;
}

