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


/************************************************************************
                        ImageInputSystem.cpp - Copyright klank

**************************************************************************/

#include "ImageInputSystem.h"
#include "XMLTag.h"
#include <signal.h>
extern volatile bool g_stopall;
void COPCTRLC(int sig)
{
  printf("In SIG INT handler of cop\n");
  g_stopall = true;
  boost::xtime t;
  boost::xtime_get(&t, boost::TIME_UTC);
  t.sec += 1;
  printf("Waiting 1 seconds before shutting down\n");
  boost::thread::sleep(t);
  raise(15);
}


    /*printf("Sleeping\n");*/
void Sleeping(long ms)
 {
  boost::xtime t;
  boost::xtime_get(&t, boost::TIME_UTC);
  t.nsec += ms * 1000000;  //TODO Check
  boost::thread::sleep(t);
 }

using namespace cop;


// Constructors/Destructors
//

ImageInputSystem::ImageInputSystem (XMLTag* configFile)
{
  if(configFile != NULL && configFile->CountChildren() > 0)
  {
    XMLTag* tag = configFile->GetChild(0);
    if(tag == NULL)
    {
      ROS_WARN("Error Loading Image Acquisition modules\n");
      throw "Error in ImageInputSystem::ImageInputSystem";
    }
    m_cameras = XMLTag::Load(tag, &m_cameras);
    for(std::vector<Sensor*>::const_iterator it = m_cameras.begin();
      it != m_cameras.end(); it++)
     {
       if((*it) != NULL && (*it)->RequiresSensorList())
       {
         (*it)->SetSensorList(m_cameras);
       }
     }
    ROS_INFO("Loaded %ld Cameras\n", m_cameras.size());
    m_stConverterNames = configFile->GetProperty(XML_ATTIBUTE_READINGCONVERTER, "");
    if(m_stConverterNames.length() > 0)
    {
      ReadingConverter* reading = ReadingConverter::ReadingConverterFactory(m_stConverterNames);
      Reading::s_conv[std::pair<ReadingType_t, ReadingType_t>(reading->TypeIn(), reading->TypeOut())] = reading;
    }
  }
}

ImageInputSystem::~ImageInputSystem ( )
{
  std::map<std::pair<ReadingType_t, ReadingType_t>, ReadingConverter*>::iterator iter = Reading::s_conv.begin();
  for(;iter != Reading::s_conv.end(); )
  {
    delete (*iter).second;
    Reading::s_conv.erase(iter);
    iter = Reading::s_conv.begin();
  }
}


void ImageInputSystem::AddSensor(Sensor* sensor)
{
  m_cameras.push_back(sensor);

   for(std::vector<Sensor*>::const_iterator it = m_cameras.begin();
       it != m_cameras.end(); it++)
   {
     if((*it)->RequiresSensorList())
     {
        (*it)->SetSensorList(m_cameras);
     }
   }
}


// Accessor methods
//
XMLTag* ImageInputSystem::Save()
{
	XMLTag* tag = new XMLTag(XML_NODE_IMAGEINPUTSYSTEM);
	tag->AddChild(XMLTag::Tag(m_cameras));
	return tag;
}


/**
 * GetBestCamera
 *  @brief Selected depening on the position that should be observed a camera and returns it.
 *	@return Camera
 *	@param  pose the pose that should be observed
 *   @throws char* with an error message in case of failure
 */
std::vector<Sensor*> ImageInputSystem::GetBestSensor (RelPose &pose)
{
	size_t nSize = m_cameras.size();
  std::vector<Sensor*> sensors_seeing;
	for(unsigned int i = 0; i < nSize; i++)
	{
	  if(m_cameras[i] == NULL)
      continue;
		if(m_cameras[i]->CanSee(pose))//TODO: choose best
		{
			sensors_seeing.push_back(m_cameras[i]);
		}
	}
	return sensors_seeing;
}

