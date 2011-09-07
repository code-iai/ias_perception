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

#include "ImageSubscription.h"

#include "cv.h"
#include "highgui.h"


#include "cv_bridge/CvBridge.h"


using namespace cop;
Reading*	ImageSubscription::GetReading(const long &Frame )
{
  if((signed)m_images.size() < (Frame - m_deletedOffset + 1) || m_images.size() == 0)
  {
    if(m_grabbing)
    {
      while(m_grabbing && ((signed)m_images.size() < (Frame - m_deletedOffset + 1) || m_images.size() == 0))
      {
        printf("waiting for the camera %s to start grabbing\n", GetSensorID().c_str());
        sleep(0.2);
      }
      printf("Got a new image: %d\n", (int)m_images.size());
    }
    if((signed)m_images.size() < (Frame - m_deletedOffset + 1) || m_images.size() == 0)
    {
      printf("unexpected error\n");
      throw "Asking for images from a camera that has no images";
    }
  }
  if(Frame == -1 || (Frame - m_deletedOffset < 0 && (unsigned)(Frame - m_deletedOffset) >= m_images.size()))
  {
    return GetReading_Lock(m_images.size() -1);
    /*return m_images[m_images.size() -1];*/
  }
  return GetReading_Lock(Frame - m_deletedOffset);
}

std::string ImageSubscription::GetWindowName()
{
  if(!m_bWindowCreated)
  {
    cvNamedWindow( GetSensorID().c_str(), 1 );
    m_bWindowCreated = true;
  }
  return GetSensorID();
}

bool ImageSubscription::CanSee(RelPose &pose) const
{
  Matrix m = pose.GetMatrix(m_relPose->m_uniqueID);
  double x = m.element(0,3);  
  double y = m.element(1,3);  
  double z = m.element(2,3);  
  x = (x / z) * m_calib.focal_length;
  y = (y / z) * m_calib.focal_length; /* bring z to focallength*/
  double column = (x / m_calib.pix_size_x) - m_calib.proj_center_x;
  double row    = (y / m_calib.pix_size_y) - m_calib.proj_center_y;
  printf("ImageSubscription::CanSee:  P[X=%f, Y=%f, Z=%f] *K = p[r=%f, c=%f]\n", x,y,z,row, column);
  if(row > 0 && column > 0 && row <  m_calib.pix_size_y * 2.2 &&
     column < m_calib.proj_center_x*2.2)
    return true;
  return false;
}


void ImageSubscription::CallBack(const sensor_msgs::ImageConstPtr& msg_ptr)
{
  try
  {
    IplImageReading *reading = new IplImageReading(m_bridge->imgMsgToCv(msg_ptr, "bgr8"));
    PushBack(reading);
    while(m_images.size() > m_max_cameraImages)
    {
      if(DeleteReading())
        continue;
      else
      {
        printf("SR: Could not delete an image!");
        break;
      }
    }

   }
  catch (sensor_msgs::CvBridgeException error)
  {
    ROS_ERROR("error");
  }
}

void ImageSubscription::SetData(XMLTag* tag)
{
     Sensor::SetData(tag);
     m_bridge = new sensor_msgs::CvBridge();
    m_stTopicName = tag->GetProperty(XML_PROPERTY_TOPIC, "/yarp_to_ros_image/yarp_to_ros_image");
    m_calib.focal_length = tag->GetPropertyDouble(XML_ATTRIBUTE_FOCAL_LENGTH, 0.06);
    m_calib.pix_size_x = tag->GetPropertyDouble(XML_ATTRIBUTE_PIX_SIZE_X, 0.000001);
    m_calib.pix_size_y = tag->GetPropertyDouble(XML_ATTRIBUTE_PIX_SIZE_Y, 0.000001);
    m_calib.proj_center_x = tag->GetPropertyDouble(XML_ATTRIBUTE_PROJ_CENTER_X, 320.0);
    m_calib.proj_center_y =tag->GetPropertyDouble(XML_ATTRIBUTE_PROJ_CENTER_Y, 240.0);
}

void ImageSubscription::Show(const long frame)
{
 IplImageReading *reading = (IplImageReading*)GetReading(frame);
 cv::Mat &img = reading->m_image;
 cv::imshow(GetWindowName().c_str() , img);
 reading->Free();
}
