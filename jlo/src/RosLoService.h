/**********************************************************************************************/
/**
*              Jennifer's Located Object
*              Copyright  (C) 2008, U. Klank
*
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*
*
 **********************************************************************************************/
#include <ros/node_handle.h>
#include <ros/time.h>
#include <sstream>

#include <vision_msgs/partial_lo.h>
#include <vision_srvs/srvjlo.h>
#include <vision_srvs/register_jlo_callback.h>
#include <lo/ServiceInterface.h>

#include "tf/tfMessage.h"
#include "tf/tf.h"

#include <set>
#include <queue>
#include <string>

/**
*     class RosLoService
*     Implementation for ros calling the lo-ServiceInterface functionality
*/
class RosLoService : public jlo::ServiceInterface
{
public:
  /**
  *     Start the service listening to listeningTopic, g_stopAll can stop the loop after next read
  */
  RosLoService (const char* nodename, ros::NodeHandle &n, std::string configFile);
  /**
  *   Closing of the ports
  */
  ~RosLoService ();


  class PubParentFilterStruct
  {
  public:
    PubParentFilterStruct()
      : parent_id(0), last_value_set(false) {}
    PubParentFilterStruct( std::vector<ros::Publisher*> pub, unsigned long parent, std::vector<double> filt)
       : publisher(pub), parent_id(parent), filter(filt), last_value_set(false) {}
    std::vector<ros::Publisher*> publisher;
    unsigned long                parent_id;
    std::vector<double>           filter;
    std::vector<double> last_val;
    bool last_value_set;

  };

  /**
  * Parses a service request and reacts on the input, by generating the answer
  * @param request incoming request: command := {idquery, framequery, update, del, cout}, [id|parentid|matrix, cov,type|name]+
  * @param answer outgoing answer: error, [id, parentid, matrix, cov, type, name]
  */
  bool ServiceCallback(vision_srvs::srvjlo::Request& request, vision_srvs::srvjlo::Response&  answer);

  /**
  * Register a callback that wil publish any change on a certain lo id
  * @param request incoming request: topicname and id
  * @param answer outgoing answer: error
  */
  bool CallbackRegisterService(vision_srvs::register_jlo_callback::Request& request, vision_srvs::register_jlo_callback::Response&  answer);
  /**
  *   UpdateEventHandler
  *   @brief this function will wait for update events and publish then a partial lo to specified topics
  */
  void UpdateEventHandler();

  /**
  *   PublishUpdate
  *   @brief Helper fundction handling the final publishing of updated registered jlos
  *   @param id   jlo to be published
  *   @param obj_to_pub   object containing parent id and list of publishers to which the update will be send
  *   @param return false if the referenced jlo was already deleted
  */
  bool PublishUpdate(unsigned long id, PubParentFilterStruct& obj_to_pub);
  /**
  *   UpdateEventNotifier
  *   @brief raises and update event
  */
  static void UpdateEventNotifier(unsigned long object_id);
private:
  /**
  *   Members to handle tf subscription and main service
  */
  ros::Subscriber tf_subscription;
  ros::ServiceServer located_object_service;
  std::set<std::string> tf_blacklist;

  /**
  *   Handle call
  */
/*  ros::ServiceServer located_object_callback_reagister_service;
*/
  std::map<unsigned long, PubParentFilterStruct > m_jlotopicMap;
  std::map<unsigned long, unsigned long> m_parentToRegisteredJlo;
  std::map<std::string, ros::Publisher> m_topicPublisherMap;
  static std::queue<unsigned long> m_queueOfLosToPublish;

  /**
  * subscribing to one tf topic to update lowlevel entries
  **/
  void tf_subscription_callback(const tf::tfMessage::ConstPtr &msg_in_);
};
