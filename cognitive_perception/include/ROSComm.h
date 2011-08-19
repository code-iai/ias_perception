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


#ifndef USE_YARP_COMM /*Only this or Yarp*/

#ifndef ROSCOMM_H
#define ROSCOMM_H

#include "RelPose.h"
#include "VisFinder.h"
#include "PerceptionPrimitive.h"
#include "Comm.h"


#include <map>

#include <vision_srvs/cop_call.h>
#include <vision_srvs/cop_save.h>
#include <vision_msgs/cop_answer.h>
#include <vision_msgs/cop_feedback.h>
#include <vision_msgs/cop_status.h>
#include <vision_srvs/cop_get_methods_list.h>
#include <std_msgs/String.h>
#include <ros/ros.h>

#define STD_COP_OUTPUT_PORT "/tracking/out"
#ifdef BOOST_THREAD
#include <boost/thread.hpp>
#include <boost/bind.hpp>
using namespace boost;
#endif


namespace cop
{
  /*****************************************************************************************
  *  class ROSComm                                                                        */
  /*****************************************************************************************
  *   This class implements a ros service that answers cop_querys
  ******************************************************************************************/
  class ROSComm : public Comm
  {
  public:
    ROSComm(VisFinder* visFinder, PossibleLocations_t* pose, PerceptionPrimitive& vis, ros::Publisher * pub,
               int numOfObjects, int actionType, std::string callerid, std::map<std::string, std::vector<std::string> > &callerids) :
      m_visFinder(visFinder),
      m_pose(pose),
      m_visPrim(vis),
      m_publisher(pub),
      m_numOfObjects(numOfObjects),
      m_actionType(actionType),
      m_callerid(callerid),
      m_calleridMap(callerids)
    {
    }

    ~ROSComm();

    /*****************************************************************************************
    *  NotifyPoseUpdate                                                                        */
    /*****************************************************************************************
    *  Call back that is called whenever a new pose is set for a certain model
    *  This callback must be told the signature that is tracked
    ******************************************************************************************/
    virtual void NotifyPoseUpdate(RelPose* pose, bool sendObjectRelation = true);

    /*****************************************************************************************
    *  NotifyNewObject                                                                        */
    /*****************************************************************************************
    *  Call back that is called whenever a object fullfilling the requested object prototype
    *  The prototype can be set by the
    ******************************************************************************************/
    virtual void NotifyNewObject(Signature* sig, RelPose* pose);


    /*****************************************************************************************
    *  Start                                                                                */
    /*****************************************************************************************
    *  Calls the yarp threadfunc, with a new thread if this is possible
    ******************************************************************************************/
    void Start();
private:
    /*****************************************************************************************
    *  ProcessCall                                                                           */
    /*****************************************************************************************
    *  Action: calls visual finder, and writes the results on a topic
    ******************************************************************************************/
    void ProcessCall();


    /*****************************************************************************************
    *  PublishAnswer                                                                           */
    /*****************************************************************************************
    *  Mutex around publish to topic, checks if topic is up
    ******************************************************************************************/
    void PublishAnswer(vision_msgs::cop_answer &answer);

    /*yarp::os::BufferedPort<yarp::os::Bottle>* m_port;*/
    VisFinder* m_visFinder;
    PossibleLocations_t* m_pose;
    PerceptionPrimitive& m_visPrim;
    ros::Publisher* m_publisher;
    int m_numOfObjects;
    int m_actionType;
    std::string m_callerid;
    std::map<std::string, std::vector<std::string> >  &m_calleridMap;
  };

  class ROSTopicManager
  {
  public:
    ROSTopicManager(VisFinder* s_visFinder, SignatureDB *s_sigDb);
    ~ROSTopicManager();

    void CloseROSTopic(std::string name);

    void Listen(std::string name, volatile bool &g_stopall, ros::NodeHandle* node);
    bool ListenCallBack(vision_srvs::cop_call::Request& request, vision_srvs::cop_call::Response&  answer);

    bool SaveCallBack(vision_srvs::cop_save::Request& msg, vision_srvs::cop_save::Response&  answer);
    bool ListProgramCallBack(vision_srvs::cop_get_methods_list::Request& msg, vision_srvs::cop_get_methods_list::Response& answer);

    void NewSignatureCallBack(std_msgs::String::ConstPtr xmlFilename);
    void FeedbackCallBack(vision_msgs::cop_feedback::ConstPtr feedback);
    /**/
    void StatusThread();

    /*void ListenCallBack(const boost::shared_ptr<const cop::cop_call> &msg);*/
    bool OpenCommOnROSTopic(std::string st);

    void Subscriber(const ros::SingleSubscriberPublisher& subs);


    std::map<std::string, ros::Publisher*> m_openTopics;

    ros::Publisher m_statusPublisher;

    VisFinder* m_visFinder;
    SignatureDB* m_sig;
    std::map<std::string, std::vector<std::string> > m_subscriberPerTopics;
  };
}
#endif /* ROSCOMM_H */

#endif /*USE_YARP_COMM */
