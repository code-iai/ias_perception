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


#ifdef LOGFILE
#include "LogFile.h"

#include <ros/ros.h>
#include <std_msgs/String.h>


#ifdef BOOST_THREAD
#include <boost/thread.hpp>
#include <boost/bind.hpp>
boost::mutex s_mutexLogFile;
#include <boost/thread/mutex.hpp>
#define BOOST(A) A

#else
#define BOOST (A) ;
#endif

using namespace cop;

ros::Publisher s_pubDebug;

LogFile::LogFile(std::string filename) :
    m_filename(filename)
{
    if(m_filename.length() == 0)
    {
        m_filename = "LogFile.log";
    }
    try
    {
      ros::NodeHandle nh;
      s_pubDebug = nh.advertise<std_msgs::String>("/cop/logging", 5);
    }
    catch(...)
    {
      printf("\n\n\n PROBLEMS ADVERTISING IN LOGFILE\n\n\n");
    }
       
}

LogFile::~LogFile(void)
{
}


void LogFile::ReportEvent(unsigned long ppid, std::string ppalg)
{
  char test[1024];
  std_msgs::String st;
  sprintf(test, "%ld %s ",ppid, ppalg.c_str());
  st.data = test;
  s_pubDebug.publish(st);
}


void LogFile::ReportError(std::string action_name,  std::string caller, long timestamp, double risk, Elem* relatedElem, std::string problemDescription)
{

   ROS_ERROR("The action %s reported problems: %s\n", action_name.c_str(), problemDescription.c_str());
   ROS_ERROR("  Caller: %s, Elem involved: %ld (Type: %s)\n", caller.c_str(), relatedElem->m_ID, relatedElem->GetNodeName().c_str());
   ROS_ERROR("  Time Information: %ld Risk: %f\n", timestamp, risk);
}


/**  WARNING: ifdef goes over two functions*/
void LogFile::Log(std::string action_name,  std::string caller, double duration_s, double success, Elem* relatedElem)
#ifdef BOOST_THREAD
{
		boost::thread(
            boost::bind(&LogFile::LogThread, this,
          action_name,
        caller,
      duration_s, success, relatedElem)) ;
}

void LogFile::LogThread(std::string action_name,  std::string caller, double duration_s, double success, Elem* relatedElem)
#endif
{
   FILE* file = NULL;
   XMLTag* tag = NULL;
   printf("Lock for logfile\n");
   BOOST(s_mutexLogFile.lock());
    try
    {
      tag = new XMLTag(XML_NODE_LOGENTRY);
      tag->AddProperty(XML_ATTRIBUTE_TIMESTAMP_LOG, tag->date());
      tag->AddProperty(XML_ATTRIBUTE_CALLER,  caller);
      tag->AddProperty(XML_ATTRIBUTE_DURATION, duration_s);
      tag->AddProperty(XML_ATTRIBUTE_SUCCESS, success);

      XMLTag* child = new XMLTag(XML_NODE_ACTION_DESCRIPTION);
      child->SetCData(action_name);
      tag->AddChild(child);

      if(relatedElem != NULL)
      {
        XMLTag* elem = relatedElem->Save(true);
        tag->AddChild(elem);
      }
      char* ch = tag->WriteToString();
#ifdef WIN32
      fopen_s(&file,  m_filename.c_str(), "a");
  #else /*WIN32*/

      printf("Opening file: %s for appending logs\n", m_filename.c_str());
      file = fopen( m_filename.c_str(), "a");
      if(file == NULL)
      {
        printf("LogFile Log: Not able to open logfile to append an logentry\n");
      }
  #endif  /*WIN32*/
      if(file != NULL)
      {
        fwrite(ch , strlen(ch), 1, file);
        fclose(file);
        tag->FreeAfterWriteToString();
      }
    }
    catch(...)
    {
      printf("LogFile::Log: Unexpected Error in logging\n");
    }
    delete tag;
    printf("Unlock for logfile\n");
  BOOST(s_mutexLogFile.unlock());
#ifdef _DEBUG
  printf("Leaving temporary logging thread.\n");
#endif

}
#endif /*LOGFILE*/
