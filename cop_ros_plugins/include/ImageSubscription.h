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

#include "Sensor.h"
#include "XMLTag.h"

#include <ros/ros.h>
#include <ros/node_handle.h>
#include <sensor_msgs/Image.h>
#include "IplImageReading.h"

#define XML_NODE_IMAGESUBSCRIPTION "ImageSubscription"
#define XML_PROPERTY_TOPIC "TopicName"
#define XML_ATTRIBUTE_FOCAL_LENGTH "focal_length"
#define XML_ATTRIBUTE_PIX_SIZE_X "pix_size_x"
#define XML_ATTRIBUTE_PIX_SIZE_Y "pix_size_y"
#define XML_ATTRIBUTE_PROJ_CENTER_X "proj_center_x"
#define XML_ATTRIBUTE_PROJ_CENTER_Y "proj_center_y"

namespace sensor_msgs
{
   class CvBridge;
}

namespace cop
{

  class ImageSubscription : public Sensor
  {
  public:
    ImageSubscription() :
          m_bWindowCreated(false),
          m_grabbing(false)
    {

    }

    ~ImageSubscription(){Stop();}
    std::string GetWindowName();
    /**
    *  Get Type of the camera by its Name
    */
    std::string GetName() const{return XML_NODE_IMAGESUBSCRIPTION;};
    /**
    *  Show
    *  @param frame number, to specify an temporal offset or a specific file
    *  @brief should display the sensors currenbt reading, if wanted
    */
    void Show(const long frame);
    /**
    * GetReading
    * @param Frame frame number, to specify an offset or a specific file
    * @throw char* with an error message in case of failure
    */
    Reading*	GetReading(const long &Frame);

    /**
    * CanSee
    * Checks if a pose is inside the view of this sensor
    * @param pose pose that has to be looked at
    */
    bool	CanSee(RelPose &pose) const;

    void CallBack(const sensor_msgs::ImageConstPtr& msg_ptr);

    /**
    *    Start
    *   @brief overwrite to start up the data reading, is called at least once after creation
    */
    bool	Start()
    {
      printf("Start subscribing topic %s \n", m_stTopicName.c_str());
      ros::NodeHandle nh;
      printf("Subscribe to topic %s \n", m_stTopicName.c_str());
      m_imageSub = nh.subscribe (m_stTopicName, 1, &ImageSubscription::CallBack, this);
      m_grabbing = true;
      return true;
    }
    /**
    *    Start
    *   @brief overwrite to stop call this in the destructor if necessary, will be used to shut down cameras
    */
    bool	Stop()
    {
      m_grabbing = false;
      m_imageSub.shutdown();
      return true;
      /*TODO unsunscribe*/
    }
    /**
     *   @return the pose of this sensor
     */
    /*RelPose* GetRelPose(){return m_relPose;}*/

    XMLTag* Save()
    {
      XMLTag* tag = new XMLTag(GetName());
      Sensor::SaveTo(tag);
      tag->AddProperty(XML_PROPERTY_TOPIC, m_stTopicName);
      tag->AddProperty(XML_ATTRIBUTE_FOCAL_LENGTH, m_calib.focal_length);
      tag->AddProperty(XML_ATTRIBUTE_PIX_SIZE_X, m_calib.pix_size_x);
      tag->AddProperty(XML_ATTRIBUTE_PIX_SIZE_Y, m_calib.pix_size_y);
      tag->AddProperty(XML_ATTRIBUTE_PROJ_CENTER_X, m_calib.proj_center_x);
      tag->AddProperty(XML_ATTRIBUTE_PROJ_CENTER_Y, m_calib.proj_center_y);
      return tag;
    }
    /**
    *   Can this Sensor be used like a camera, (incl. Calibration, Showing, usw.)
    */
    bool IsCamera() const {return true;}
    /**
    *  Calibration
    */
    MinimalCalibration m_calib;
    std::pair< std::string, std::vector<double> > GetUnformatedCalibrationValues()
    {
      std::vector<double> vec; 
      vec.push_back(m_calib.focal_length); 
      vec.push_back(m_calib.pix_size_x); 
      vec.push_back(m_calib.pix_size_y); 
      vec.push_back(m_calib.proj_center_x); 
      vec.push_back(m_calib.proj_center_y); 
      
      return std::pair< std::string, std::vector<double> >("RECTHALCONCALIB", vec);
    }
  protected:
    /**
    *  This can be overwritten to get the data necessary to reconstruct a saved reading
    */
    void SetData(XMLTag* tag);

  private:
      std::string m_stTopicName;
      sensor_msgs::CvBridge *m_bridge;
      unsigned int m_max_cameraImagesStored;
      ros::Subscriber m_imageSub;
      bool m_bWindowCreated;
      bool m_grabbing;
  };
}
