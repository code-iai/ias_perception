/*
 * Copyright (C) 2009 by Ulrich Friedrich Klank <klank@in.tum.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */


/**
*	klank 18.11.2009
*/

#ifndef SENSOR_H
#define SENSOR_H

#include <vector>
#include <cstdio>
#include "Reading.h"


#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/thread/condition.hpp>

using namespace boost;

#define XML_NODE_SENSOR "Sensor"
#define XML_PROPERTY_SENSORNAME "SensorName"


#define XML_NODE_SENSORRELAY "SensorRelay"
#define XML_ATTRIBUTE_TOPICNAME "TopicName"
#define XML_ATTRIBUTE_SENSORTYPE "SensorType"
#define XML_ATTRIBUTE_MESSAGTYPE "MessageType"
#define XML_ATTRIBUTE_RATE  "Rate"

namespace cop
{
  class RelPose;
  class XMLTag;
  /**
  *   Class Sensor
  *   @brief Provides an interface for camera usage
  */
  class Sensor
  {
  public:

      /***
      *   @brief Constructor with pose, initializes parameters
      */
      Sensor() :
         m_relPose(NULL),
         m_FrameCount(0),
         m_deletedOffset(0),
         m_max_cameraImages(2)
      {}
      /***
      *   @brief Constructor with pose, initializes parameters, sets the sensors pose
      */
      Sensor(RelPose* pose) :
         m_relPose(pose),
         m_FrameCount(0),
         m_deletedOffset(0),
         m_max_cameraImages(2)
      {};
      /**
      *   The destructor is virtual
      */
      virtual ~Sensor();

      /**
      *   @brief Load sensor parameter from xml
      *   Only derivatives of Camera can use this constructor to initialize values
      */
      static Sensor* SensorFactory(XMLTag* tag);

  public:
      virtual void SaveTo(XMLTag* tag);
      /**
      *  Get Type of the camera by its Name
      */
      virtual std::string GetName() const {return XML_NODE_SENSOR;};
      /**
      *  Show
      *  @param frame number, to specify an temporal offset or a specific file
      *  @brief should display the sensors currenbt reading, if wanted
      */
      virtual void Show(const long frame = -1){}
      /**
      * GetReading
      * @param frame frame number, to specify an offset or a specific file
      * @throw char* with an error message in case of failure
      */
      virtual Reading*	GetReading(const long &frame = -1) = 0;
      /**
      * CanSee
      * Checks if a pose is inside the view of this sensor
      * @param pose pose that has to be looked at
      */
      virtual bool CanSee(RelPose &pose) const {return false;}
      /**
      *    Start
      *   @brief overwrite to start up the data reading, is called at least once after creation
      */
      virtual bool	Start()				= 0;
      /**
      *    Start
      *   @brief overwrite to stop call this in the destructor if necessary, will be used to shut down cameras
      */
      virtual bool	Stop()
      {
          m_newDataArrived.notify_all();
          return true;
      }
      /**
       *   @return the pose of this sensor
       */
      RelPose* GetRelPose(){return m_relPose;}

      virtual XMLTag* Save() = 0;
      /**
      *   Can this Sensor be used like a camera, (incl. Calibration, Showing, usw.)
      */
      virtual bool IsCamera()const  {return false;}

      /**
      * GetSensorID
      * This contains optinally an identifier for the sensor (Loaded from)
      */
      std::string GetSensorID() const{return m_stSensorName;}

      /**
      * m_stSensorName
      *  @brief Stereo works as a composition of names sensors
      */
      virtual bool RequiresSensorList(){return false;}
      /**
      *  SetSensorList
      *  @brief Stereo works as a composition of names sensors
      */
      virtual void SetSensorList(std::vector<Sensor*>){};

      /**
      *  GetUnformatedCalibrationValues
      *  @return a pair of a fomrat string describing the content and a list of doubles
      */
      virtual std::pair<std::string, std::vector<double> > GetUnformatedCalibrationValues() const{return std::pair<std::string, std::vector<double> >();}

      /**
      *  Helper function for projecting 3D data to the images
      */
      virtual void ProjectPoint3DToSensor(const double &x, const double &y, const double &z, double &row, double &column)  const;
      /**
      *  Helper function for sending 3D display data to rviz
      *  @remarks only takes points which are in map and from time now
      */
      void Publish3DData(const std::vector<double>& x, const std::vector<double>& y, const std::vector<double>& z);
      /**
      * DeleteReading
      *  @brief removes entries of the reading buffer, this function will not release still references functions (@see cop::Reading::Free)
      *
      */
      virtual bool DeleteReading();

      /**
      *  GetShowLock()
      */
      virtual void GetShowLock(){printf("Lock Sensor Show %s\n", m_stSensorName.c_str()); m_mutexShow.lock();}
      /**
      *  ReleaseShowLock()
      */
      virtual void ReleaseShowLock(){printf("UnLock Sensor Show %s\n", m_stSensorName.c_str()); m_mutexShow.unlock();}

      /**
      *
      *  @brief Wait the condition variable m_mutexImageList
      */
      virtual void WaitForNewData();
      virtual void PushBackAsync(){};

      virtual Reading* ApplySelfFilter(Reading* read){return read;}
  protected:
     virtual void PushBack(Reading* img);
     Reading* GetReading_Lock(size_t index);
     virtual void SetData(XMLTag* tag);

  protected:
      RelPose*		m_relPose;
      std::vector<Reading*> m_images;
      std::vector<Reading*> m_temp_images;
      long				m_FrameCount;
      long				m_deletedOffset;
      unsigned long m_max_cameraImages;
      std::string m_stSensorName;

      typedef boost::mutex::scoped_lock lock;
      boost::mutex m_mutexImageList;
      boost::mutex m_mutexShow;
      boost::condition m_newDataArrived;
  };
}

#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>

namespace cop
{
  template<class SensorType, class MessageType>
  class SensorNetworkRelay : public SensorType
  {
  public:
      /***
      *   @brief Constructor with pose, initializes parameters
      */
    SensorNetworkRelay()
       : SensorType(),
         m_readyToPub(true)
    {m_rateCounter = 0;m_bCameraInfo=false;};
      /***
      *   @brief Constructor with pose, initializes parameters, sets the sensors pose
      */
      /**
      *   The destructor is virtual
      */
      virtual ~SensorNetworkRelay()
      {
        /*SensorType::~SensorType();*/
      };
      /***
      *  @brief
      *  @param tag data, containing the class to load

      virtual void SetData(XMLTag* tag); */
      /**
      *  Get Type of the camera by its Name
      */
      virtual std::string GetName() const {throw "Error in SensorNetworkRelay::GetName(): This function has to be overwritten";};

      virtual MessageType ConvertData(Reading* img) = 0;
  protected:
     virtual void PushBackAsync()
     {
       try
       {
         if(++m_rateCounter >  m_rate)
         {
            m_rateCounter = 0;
            MessageType temp = ConvertData(m_curPubReading);
            m_pub.publish(temp);
            if(m_bCameraInfo)
            {
              m_cameraInfoMessage.header = temp.header;
              m_pubCamInfo.publish(m_cameraInfoMessage);
            }
         }
       }
       catch(const char* text)
       {
         printf("Error converting Data in SensorRelay::PushBack\n");
       }
       catch(...)
       {
         printf("Error publishing data in SensorRelay::PushBack\n");
       }
       m_readyToPub = true;
     }

     virtual void PushBack(Reading* img)
     {
       if(m_readyToPub)
       {
         m_readyToPub = false;
         m_curPubReading = img;
         SensorNetworkRelay::PushBackAsync();
       }
       Sensor::PushBack(img);
     }

  protected:
     std::string m_stTopic;
     ros::Publisher m_pub;
     bool m_bCameraInfo;
     bool m_readyToPub;
     Reading* m_curPubReading;
     ros::Publisher m_pubCamInfo;
     sensor_msgs::CameraInfo m_cameraInfoMessage;
     int m_rate;
     int m_rateCounter;
  };


  class MinimalCalibration
  {
  public:
    MinimalCalibration(){}
    MinimalCalibration(XMLTag* tag);
    void SetData(XMLTag* tag);
    MinimalCalibration(std::pair<std::string, std::vector<double> > calib_temp)
    {
      if(calib_temp.first.compare("RECTHALCONCALIB") == 0)
      {
        focal_length = calib_temp.second[0];
        pix_size_x = calib_temp.second[1];
        pix_size_y = calib_temp.second[2];
        proj_center_x = calib_temp.second[3];
        proj_center_y = calib_temp.second[4];
        width = calib_temp.second[5];
        height = calib_temp.second[6];
        distortion_param = 0;
      }
      else if(calib_temp.first.compare("HALCONCALIB") == 0)
      {
        focal_length = calib_temp.second[0];
        distortion_param = calib_temp.second[1];
        pix_size_x = calib_temp.second[2];
        pix_size_y = calib_temp.second[3];
        proj_center_x = calib_temp.second[4];
        proj_center_y = calib_temp.second[5];
        width = calib_temp.second[6];
        height = calib_temp.second[7];
      }
    };

    double focal_length;
    double pix_size_x;
    double pix_size_y;
    double proj_center_x;
    double proj_center_y;
    double width;
    double height;
    double distortion_param;

    void Project3DPoint(const double &x, const double &y, const double &z, double &row, double &column)
    {
      double temp1,temp2;
      if (focal_length > 0.0)
      {
        temp1 = focal_length * (x / z );
        temp2 = focal_length * (y / z );
      }
      else
      {
        temp1 = x;
        temp2 = y;
      }
      column = ( temp1 / pix_size_x ) + proj_center_x;
      row = ( temp2 / pix_size_y ) + proj_center_y;
    }
  };

template<typename TypeReading, typename DataType> class ScopedImage
{
public:
  ScopedImage(std::vector<Sensor*> sensors, ReadingType_t type) /*:
   selected_sensor(ExtractSensor(sensors, type)),
   calib(selected_sensor->GetUnformatedCalibrationValues()),
   original(ExtractOriginal(selected_sensor, type)),
   sensor_pose_at_capture_time(original->GetPose()),
   converted(original->GetType() != type),
   copy(converted ? (TypeReading*)original->ConvertTo(type) : NULL)
   image(converted ? (copy->m_image) : ((TypeReading*)original)->m_image)*/
  {
   selected_sensor = ExtractSensor(sensors, type);
   copy = NULL;

   if(selected_sensor != NULL)
   {
     calib = selected_sensor->GetUnformatedCalibrationValues();
     original = ExtractOriginal(selected_sensor, type);
     sensor_pose_at_capture_time = original->GetPose();
     converted = original->GetType() != type;
     m_type= type;
    }
    else
    {
      printf("Failed to extract sensor\n");
      throw "ScopedImage: No sensors hits the requested criteria";
    }
   /* if(copy == NULL)
      throw "ScopedImage: No conversion to the requested type is available";*/
  }

  ~ScopedImage()
  {
    original->Free();
    if(copy != NULL)
      delete copy;
  }

  static Sensor* ExtractSensor(std::vector<Sensor*> sensors, ReadingType_t type)
  {
    for(size_t i = 0; i < sensors.size(); i++)
    {
      std::pair<std::string,  std::vector<double> > calib_temp = sensors[i]->GetUnformatedCalibrationValues();
      // TODO: sensors[i]->IsCamera()  => sensors[i]->ReadingType() == type

      if(((sensors[i]->IsCamera() && !(type == ReadingType_PointCloud)) ||
          (!sensors[i]->IsCamera() && (type == ReadingType_PointCloud))) &&
         (calib_temp.first.compare("RECTHALCONCALIB")) == 0 &&
         (calib_temp.second.size() == 7) )
      {
        printf("Got another  sensor that gives the necessary data\n");
        return sensors[i];
      }
    }
    return NULL;
  }


  static Reading* ExtractOriginal(Sensor* sensor, ReadingType_t type)
  {
    if(sensor == NULL)
      throw "ScopedImage: No sensors fulfills this algorithm requirements";
    return sensor->GetReading(-1);
  }


  DataType& operator* ()
  {
    if(converted )
    {
      if(copy == NULL)
      {
        printf("Conversion to be done\n");
        copy = (TypeReading*)(original->ConvertTo(m_type));
        printf("Conversion done\n");
      }
      return copy->m_image;
    }
    else
      return ((TypeReading*)original)->m_image;
  }
  Sensor* selected_sensor;
  MinimalCalibration calib;
private:
  Reading* original;
public:
  RelPose* sensor_pose_at_capture_time;
private:
  bool converted;
  ReadingType_t m_type;
  TypeReading* copy;
  /*TypeReading* copy;
    DataType& image;*/

};


}
#endif /*CAMERA_H*/
