/**********************************************************************************************/
/**********************************************************************************************/
/**
*              Jennifer's Located Object Service
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


#include "RosLoService.h"

#include "lo/ServiceLocatedObject.h"

#include <tf/transform_listener.h>
#include <string>
#include <boost/bind.hpp>
#include <boost/thread.hpp>

#define IDQUERY "idquery"
#define NAMEQUERY "namequery"
#define FRAMEQUERY "framequery"
#define DELETE "del"
#define UPDATE "update"
#define COUTLO "coutlo"
#ifdef _DEBUG
#define PRINTF_DEBUG(A) printf(A)
#else
#define PRINTF_DEBUG(A) printf(A)
#endif

std::queue<unsigned long> RosLoService::m_queueOfLosToPublish;

RosLoService::RosLoService(const char* nodename, ros::NodeHandle &n, std::string configFile)
    : jlo::ServiceInterface(configFile.c_str()),
      located_object_service( n.advertiseService("/located_object", &RosLoService::ServiceCallback, this) )/*,
      located_object_callback_reagister_service( n.advertiseService("/register_jlo_callback", &RosLoService::CallbackRegisterService, this) )*/
{
  using namespace std;

  /**
  * Advertise the service that answers to normal lo-quieries
  */
  printf("Advertise Service \"/located_object\"\n");

  /**
  * Subscribe to a tf topic and listening to it.
  */
  if( n.hasParam( "tf_blacklist" ) )
  {
      using namespace XmlRpc;

      XmlRpcValue blacklist_param_value;
      n.getParam( "tf_blacklist", blacklist_param_value );
      if( blacklist_param_value.getType() != XmlRpcValue::TypeArray )
          ROS_WARN("tf_blacklist has invalid type.");
      else
      {
          for(int i=0; i<blacklist_param_value.size(); i++)
          {
              if( blacklist_param_value[i].getType() != XmlRpcValue::TypeString )
              {
                  ROS_WARN("Ignoring blacklist entry. Not a string");
                  continue;
              }
              string blacklist_entry = blacklist_param_value[i];
              tf_blacklist.insert( blacklist_entry );
              ROS_INFO("Adding '%s' to tf blacklist", blacklist_entry.c_str());
          }
      }
  }

  string tf_topic;
  n.param<string>( "tf_topic", tf_topic, "/tf" );
  printf("Subscribe \"%s\"\n", tf_topic.c_str());
  tf_subscription = n.subscribe<tf::tfMessage>( tf_topic, 5, boost::bind(&RosLoService::tf_subscription_callback, this, _1) );
  /*boost::thread(boost::bind(&RosLoService::UpdateEventHandler, this));*/
}

RosLoService::~RosLoService()
{
  SaveList()->WriteToFile("bla.ini");
  printf("Wrote data to bla.ini\n");
}

bool PutLoIntoPartialLo( jlo::ServiceLocatedObject* lo, vision_msgs::partial_lo&  out_lo, bool incRef = true)
{
  try
  {
    out_lo.id = lo->m_uniqueID;
    out_lo.parent_id = lo->m_parentID;
    int width = 4;
    Matrix m = lo->GetMatrix();
    Matrix cov = lo->GetCovarianceMatrix();
    if(incRef)
      lo->IncreaseReferenceCounter();
    for(int r = 0; r < width; r++)
    {
      for(int c = 0; c < width; c++)
      {
          out_lo.pose[r * width + c] = m.element(r,c);
      }
    }
    width = 6;
    for(int r = 0; r < width; r++)
    {
      for(int c = 0; c < width; c++)
      {
          out_lo.cov[r * width + c] = cov.element(r,c);
      }
    }
    out_lo.type = lo->GetLOType();
    out_lo.name = lo->m_mapstring;
  }
  catch(...)
  {
    printf("Critical Error in PutLoIntoResponse \n");
    return false;
  }
  return true;
}

bool PutLoIntoResponse( jlo::ServiceLocatedObject* lo, vision_srvs::srvjlo::Response&  answer)
{
  vision_msgs::partial_lo& out_lo = answer.answer;
  return PutLoIntoPartialLo(lo, out_lo);
}

std::map<std::string, int> s_tester;
std::map<std::string, int> s_tester_max;

void IncreaseTester(std::string callerid)
{
  s_tester[callerid] += 1;
  if(s_tester[callerid] >= 100)
  {
    s_tester_max[callerid] += s_tester[callerid];
    s_tester[callerid] = 0;
    ROS_WARN("Warning: the jlo caller %s registered %d jlos without deleting", callerid.c_str(), s_tester_max[callerid] );
  }
}

void DecreaseTester(std::string callerid)
{
  s_tester[callerid] -= 1;
}


bool RosLoService::ServiceCallback(vision_srvs::srvjlo::Request& request, vision_srvs::srvjlo::Response&  answer)
{
std::string callerid = (*request.__connection_header)["callerid"];
 try
 {
   if(request.command.compare(IDQUERY) == 0)
   {
     /* case 1: One number can only mean an id: ID-Query*/
     jlo::ServiceLocatedObject* lo = GetServiceLocatedObject(request.query.id);
     /*printf("Requested ID: %d => %p\n", (int)request.query.id, lo);*/
     if (lo == NULL)
     {
       ROS_WARN("Error in IDquery: Id %ld does not exist! (Caller: %s)\n", request.query.id, callerid.c_str());
       answer.error = "Error in IDquery: Id does not exist!\n";
       return true;
     }
     IncreaseTester(callerid);
     PutLoIntoResponse(lo, answer);
   }
   else if(request.command.compare(NAMEQUERY) == 0)
   {
     long id = GetServiceLocatedObjectID(request.query.name);
     if(id >= ID_WORLD)
     {
       jlo::ServiceLocatedObject* lo = GetServiceLocatedObject(id);
       if (lo == NULL)
       {
         ROS_WARN("Error in NameQuery: Id %s does not exist!(Caller: %s)\n", request.query.name.c_str(), callerid.c_str());
         answer.error = "Error in NameQuery: Id does not exist!\n";
         return true;
       }
       PutLoIntoResponse(lo, answer);
     }
     else
     {
        ROS_WARN("Error in NameQuery: Name %s does not exist!(Caller: %s)\n", request.query.name.c_str(), callerid.c_str());
        answer.error = std::string("Error in NameQuery: Name does not exist: ").append(request.query.name);
        answer.answer.id = 0;
        return true;
      }
  }
  else if (request.command.compare(FRAMEQUERY) == 0)
  {
    try
    {
      unsigned long parentID = request.query.parent_id;
      jlo::ServiceLocatedObject* lo = GetServiceLocatedObject(request.query.id);
      if(lo != NULL && request.query.id == parentID)
      {
         answer.error = "Warning Parsing Input: Self reference ignored!\n";
         IncreaseTester(callerid);
         PutLoIntoResponse(lo, answer);
      }
      else
      {
        if (lo == NULL)
        {
          ROS_WARN("Error in Framequery: Id %ld does not exist! (Caller %s)\n", request.query.id, callerid.c_str());
          answer.error = "Error in Framequery: Id does not exist!\n";
          return true;
        }
        if(lo->m_uniqueID != ID_WORLD && lo->m_relation->m_uniqueID == (unsigned long)parentID)
        {
          IncreaseTester(callerid);
          PutLoIntoResponse(lo, answer);
        }
        else
        {
          jlo::ServiceLocatedObject* parent = GetServiceLocatedObject(parentID);
          if (parent == NULL)
          {
             ROS_WARN("Error in Framequery: Parent Id %ld does not exist! (Caller: %s)\n", parentID, callerid.c_str());
             answer.error = "Error in Framequery: Parent Id does not exist!\n";
             return true;
          }
          Matrix mat = lo->GetMatrix(*parent);
          Matrix cov = lo->GetCovarianceMatrix(*parent);
          IncreaseTester(callerid);
          PutLoIntoResponse(FServiceLocatedObject(parent, mat, cov, 0), answer);
        }
      }
    }
    catch(...)
    {
      printf("Critical error in FrameQuery\n");
      return true;
    }
  }
  else if(request.command.compare(DELETE) == 0)
  {
    unsigned long id = request.query.id;
    if(id != ID_WORLD)
    {
      jlo::ServiceLocatedObject* obj  = GetServiceLocatedObject(id);
      if(obj == NULL)
      {
         ROS_WARN("Error in delete: Object %ld does not exist! (Caller: %s)\n", id, callerid.c_str());
         answer.error = "Error in delete: Object does not exist!\n";
         return true;
      }
      else
      {
        DecreaseTester(callerid);
        unsigned long ref_count = obj->GetReferenceCounter();
        /*ROS_DEBUG("Delete request from node %s for id %ld, parent %ld", callerid.c_str(), id, obj->m_parentID);*/
        FreeServiceLocatedObject(obj);
        if(ref_count > 1)
        {
          answer.error = "Warning in delete: Object is still referenced.";
        }
      }
    }
    else
    {
      PRINTF_DEBUG("Error in delete: Can't delete ID_WORLD!\n");
       answer.error = "Error in delete: Can't delete ID_WORLD!\n";
      return true;
    }
  }
 /*case error */
/*semi complete*/
  else if(request.command.compare(UPDATE) == 0)
  {
    unsigned long parentID, id;
    bool update = false;
    unsigned long type = request.query.type;
    id =  request.query.id;
    parentID =  request.query.parent_id;
    if(id != 0)
    {
      update = true;
    }
    Matrix mat(4,4), cov(6,6);

    int width = 4;
    for(int r = 0; r < width; r++)
    {
      for(int c = 0; c < width; c++)
      {
          if((unsigned)(r * width + c) < request.query.pose.size())
             mat.element(r,c) = request.query.pose[r * width + c];
          else
            printf("Wrong length of pose\n");

      }
    }
    width = 6;
    for(int r = 0; r < width; r++)
    {
      for(int c = 0; c < width; c++)
      {
          if((unsigned)(r * width + c) < request.query.cov.size())
             cov.element(r,c) = request.query.cov[r * width + c];
          else
            printf("Wrong length of cov\n");
      }
    }
    if(update)
    {
      jlo::ServiceLocatedObject* pose = GetServiceLocatedObject(id);
      if(pose == NULL)
      {
        jlo::ServiceLocatedObject* parent = GetServiceLocatedObject(parentID);
        if(parent == NULL)
        {
          PRINTF_DEBUG("Error in Update: Requested parent does not exist!\n");
          answer.error = "Error in Update: Requested parent does not exist!\n";
          return true;
        }
        jlo::ServiceLocatedObject* pose = FServiceLocatedObject(parent, mat, cov, type);
        IncreaseTester(callerid);
        if(request.query.name.length() > 0 && GetServiceLocatedObjectID(request.query.name) < ID_WORLD)
        {
          ServiceInterface::AddMapString(pose, request.query.name);
        }
        PutLoIntoResponse(pose, answer);
      }
      else
      {
        jlo::ServiceLocatedObject* parent = GetServiceLocatedObject(parentID);
        if(parent == NULL || (parent->m_uniqueID == ID_WORLD && pose->m_uniqueID == ID_WORLD))
        {
          ROS_WARN("Error in Update: Requested parent %ld does not exist!\n", parentID);
           answer.error = "Error in Update: Requested parent does not exist!\n";
          return true;
        }
        if((parent->m_uniqueID == ID_WORLD && pose->m_uniqueID == ID_WORLD))
        {
            answer.error = "Error in Update: Asked for world in world\n";
            return true;
        }
        if(parent->m_uniqueID != pose->m_parentID)
        {
          ROS_WARN("Updated parent, dangerous in terms of circles (Old parent: %ld New Parent %ld)", pose->m_parentID, parent->m_uniqueID);
          pose->UpdateParent(parent);
        }
        pose->Update(mat, cov, ServiceInterface::FServiceLocatedObjectCopy, ServiceInterface::FreeServiceLocatedObject, &RosLoService::UpdateEventNotifier);
        if(request.query.name.length() > 0 && GetServiceLocatedObjectID(request.query.name) < ID_WORLD)
        {
          if(pose->m_mapstring.compare(request.query.name) != 0)
          {
            ServiceInterface::RemoveMapString(pose->m_mapstring);
          }
          ServiceInterface::AddMapString(pose, request.query.name);
        }
        IncreaseTester(callerid);
        PutLoIntoResponse(pose, answer);
      }
    }
    else
    {
      jlo::ServiceLocatedObject* parent = GetServiceLocatedObject(parentID);
      if(parent == NULL)
      {
        ROS_WARN("Error in Update: Requested parent %ld does not exist!\n", parentID);
        answer.error = "Error in Update: Requested parent does not exist!\n";
        return true;
      }
      jlo::ServiceLocatedObject* pose = FServiceLocatedObject(parent, mat, cov, type);
      if(request.query.name.length() > 0 && GetServiceLocatedObjectID(request.query.name) < ID_WORLD)
      {
         ServiceInterface::AddMapString(pose, request.query.name);
      }
      IncreaseTester(callerid);
      PutLoIntoResponse(pose, answer);
    }
  }
  else if (request.command.compare(COUTLO) == 0)
  {

    jlo::ServiceLocatedObject* lo = GetServiceLocatedObject(request.query.id);
    /*printf("Requested ID: %d => %p\n", (int)request.query.id, lo);*/
    if (lo == NULL)
    {
      ROS_WARN("Error Parsing Input: Id %ld does not exist!\n", request.query.id);
      answer.error = "Error Parsing Input: Id does not exist!\n";
      return true;
    }
    IncreaseTester(callerid);
    PutLoIntoResponse(lo, answer);
    std::string st = lo->SaveComplete()->WriteToString();
    cout << st << endl;
  }
  else
  {
    printf("Command %s not accepted\n", request.command.c_str());
    PRINTF_DEBUG("Error: Requested command does not exist!\n");
    answer.error = "Erro: Requested command does not exist!\n";
    return true;
  }
  return true;
 }
 catch(...)
 {
  printf("An exception occured during parsing the message!\n");
  answer.error = "Error in jlo: Exception happened during execution!\n";
  return true;
 }
}

boost::condition_variable cond;boost::mutex mut;

bool RosLoService::PublishUpdate(unsigned long id, PubParentFilterStruct& obj_to_pub)
{
  jlo::ServiceLocatedObject* lo = GetServiceLocatedObject(id);
  if(obj_to_pub.parent_id != 0)
  {
  jlo::ServiceLocatedObject* parent = GetServiceLocatedObject(obj_to_pub.parent_id);
  if (parent != NULL)
  {
    Matrix mat = lo->GetMatrix(*parent);
    Matrix cov = lo->GetCovarianceMatrix(*parent);
    lo = FServiceLocatedObject(parent, mat, cov, 0);
  }
  else
  {
     PRINTF_DEBUG("Error in Framequery: Parent Id does not exist!\n");
     lo = NULL;
  }
  }
  if(lo == NULL)
  {
    return false;
  }
  else
  {
  /**apply filter*/
    if(obj_to_pub.last_value_set)
    {
      Matrix pos = lo->GetMatrix(lo->m_relation->m_uniqueID);
      ColumnVector rot = irpy(pos);
      ColumnVector xyz(3);
      xyz << pos.element(0,3)<< pos.element(1,3) << pos.element(2,3);
      if(fabs(obj_to_pub.last_val[0] - rot.element(0)) < obj_to_pub.filter[0] &&
         fabs(obj_to_pub.last_val[1] - rot.element(1)) < obj_to_pub.filter[1] &&
         fabs(obj_to_pub.last_val[2] - rot.element(2)) < obj_to_pub.filter[2] &&
         fabs(obj_to_pub.last_val[3] - xyz.element(0)) < obj_to_pub.filter[3] &&
         fabs(obj_to_pub.last_val[4] - xyz.element(1)) < obj_to_pub.filter[4] &&
         fabs(obj_to_pub.last_val[5] - xyz.element(2)) < obj_to_pub.filter[5])
      {
        return true;
      }
      else
      {
        obj_to_pub.last_val[0] = rot.element(0);
        obj_to_pub.last_val[1] = rot.element(1);
        obj_to_pub.last_val[2] = rot.element(2);
        obj_to_pub.last_val[3] = xyz.element(0);
        obj_to_pub.last_val[4] = xyz.element(1);
        obj_to_pub.last_val[5] = xyz.element(2);
      }
    }
    else
    {
      Matrix pos = lo->GetMatrix(lo->m_relation->m_uniqueID);
      ColumnVector rot = irpy(pos);
      ColumnVector xyz(3);
      obj_to_pub.last_val.push_back(rot.element(0));
      obj_to_pub.last_val.push_back(rot.element(1));
      obj_to_pub.last_val.push_back(rot.element(2));
      obj_to_pub.last_val.push_back(xyz.element(0));
      obj_to_pub.last_val.push_back(xyz.element(1));
      obj_to_pub.last_val.push_back(xyz.element(2));
      obj_to_pub.last_value_set = true;
    }
  }
  vision_msgs::partial_lo answer;
  PutLoIntoPartialLo(lo, answer, false);

  for(std::vector<ros::Publisher*>::iterator it = obj_to_pub.publisher.begin();
       it != obj_to_pub.publisher.end(); it++)
  {
   (*it)->publish(answer);
  }
  return true;
}

void RosLoService::UpdateEventHandler()
{
  while(true)
  {
    boost::unique_lock<boost::mutex> lock(mut);
    while(m_queueOfLosToPublish.size() > 0 )
    {
      unsigned long id = m_queueOfLosToPublish.front();
      m_queueOfLosToPublish.pop();
      std::map<unsigned long, PubParentFilterStruct>::iterator iter =
         m_jlotopicMap.find(id);
      if(iter != m_jlotopicMap.end())
      {
        PubParentFilterStruct& obj_to_pub = (*iter).second;
        if(!PublishUpdate(id, obj_to_pub))
           m_jlotopicMap.erase(iter);
      }
      /** register all parents and map them back to updates of children*/
      std::map<unsigned long, unsigned long>::iterator iter_parent = m_parentToRegisteredJlo.find(id);
      if(iter_parent != m_parentToRegisteredJlo.end())
      {
        std::map<unsigned long, PubParentFilterStruct>::iterator iter_child =
           m_jlotopicMap.find(m_parentToRegisteredJlo[id]);
        if(iter_child != m_jlotopicMap.end())
        {
          PubParentFilterStruct& obj_to_pub = (*iter_child).second;
          if(!PublishUpdate(id, obj_to_pub))
            m_jlotopicMap.erase(iter_child);

        }
        else
        {
          m_parentToRegisteredJlo.erase(iter_parent);
        }
      }
    }
    cond.wait(lock);
  }
}

void RosLoService::UpdateEventNotifier(unsigned long id)
{
  boost::lock_guard<boost::mutex> lock(mut);
  m_queueOfLosToPublish.push(id);
  cond.notify_one();
}


bool RosLoService::CallbackRegisterService(vision_srvs::register_jlo_callback::Request& request, vision_srvs::register_jlo_callback::Response&  answer)
{
  jlo::ServiceLocatedObject* obj = GetServiceLocatedObject(request.jlo_id);
  if(obj != NULL)
  {
    ros::NodeHandle nh;
    std::vector<double> filter(request.filter_delta_cov.size());
    for(size_t i = 0; i <  request.filter_delta_cov.size(); i++)
      filter[i] = request.filter_delta_cov[i];
    if(m_jlotopicMap.find(request.jlo_id) == m_jlotopicMap.end())
    {
      m_jlotopicMap[request.jlo_id] = RosLoService::PubParentFilterStruct::PubParentFilterStruct(std::vector<ros::Publisher*>(), request.parent_id, filter);
      m_parentToRegisteredJlo[request.parent_id] = request.jlo_id;
    }
    else
    {
      if(request.parent_id != m_jlotopicMap[request.jlo_id].parent_id)
        answer.error = "Error: Object will be puplished with wrong parent";
      else
        answer.error = "Error: Object will be most probbably filtered incorrect";
    }
    std::map<std::string, ros::Publisher>::iterator it = m_topicPublisherMap.find(request.topic_name);
    if(it == m_topicPublisherMap.end())
    {
      m_topicPublisherMap[request.topic_name] = (nh.advertise<vision_msgs::partial_lo>(request.topic_name, 20));
    }
    m_jlotopicMap[request.jlo_id].publisher.push_back(&m_topicPublisherMap[request.topic_name]);
  }
  else
    answer.error = "Error: Object does not exist";
  return true;
}


void RosLoService::tf_subscription_callback(const tf::tfMessage::ConstPtr &msg_in_)
{
  for (unsigned int i = 0; i < msg_in_->transforms.size(); i++)
  {
    tf::StampedTransform trans;
    /*Obsolete: TransformStampedMsgToTF, if you get here an error cause the following function is not defined, pls update tf*/
    transformStampedMsgToTF(msg_in_->transforms[i], trans);
    if( tf_blacklist.find( trans.child_frame_id_ ) != tf_blacklist.end() )
        continue;


 /*   printf("got matrix:= \n[");
    for(int i = 0; i < 16; i++)
    {
      printf("%f ", matrix[i]);
      if(i % 4  == 3)
        printf("\n ");
    }
    printf("]\n");*/
    try
    {
      std::map<std::string, std::string>* msg_header_map = msg_in_->__connection_header.get();
      std::string authority;
      std::map<std::string, std::string>::iterator it = msg_header_map->find("callerid");
      if (it == msg_header_map->end())
      {
        ROS_WARN("Message recieved without callerid");
        authority = "no callerid";
      }
      else
      {
        authority = it->second;
      }
      /*setTransform(trans, authority);*/
    }
    catch(...)
    {
       ROS_ERROR("Failure to set recieved transform\n");
    }
    if(trans.frame_id_.length() == 0)
    {
      printf("Empty tf\n");
      continue;
    }
    else
    {
      //printf("Getting tf %s parent %s\n", trans.frame_id_.c_str(), trans.parent_id_.c_str());
    }

    //if ((trans.child_frame_id_.find("torso") != -1 ) || (trans.frame_id_.find("torso") != -1 ))
    //  ROS_ERROR("Getting tf -%s- parent -%s-\n", trans.child_frame_id_.c_str(), trans.frame_id_.c_str());

    unsigned long id = (long)GetServiceLocatedObjectID(trans.child_frame_id_);
    unsigned long parentid = (long)GetServiceLocatedObjectID(trans.frame_id_);
    if(parentid < ID_WORLD && trans.frame_id_.compare("/map") != 0)
    {
      printf("Error reading tf: Requested parent does not exist! (%s in %s)\n", trans.child_frame_id_.c_str(), trans.frame_id_.c_str());
      continue;
      /*jlo::ServiceLocatedObject* parent = GetServiceLocatedObject(parentid);
      parent->m_mapstring = trans.parent_id_;*/
    }
    if(parentid < ID_WORLD)
    {
      parentid = ID_WORLD;
      printf("Setting name of world to %s\n", trans.frame_id_.c_str());
      jlo::ServiceLocatedObject* parent = GetServiceLocatedObject(parentid);
      ServiceInterface::AddMapString(parent, trans.frame_id_);
    }
    jlo::ServiceLocatedObject* pose = NULL;

    btScalar matrix[16];
    trans.getOpenGLMatrix(matrix);

    Matrix mat(4,4);
    mat << matrix[0] << matrix[4]<< matrix[8]<< matrix[12]
        << matrix[1]<< matrix[5]<< matrix[9]<< matrix[13]
        << matrix[2]<< matrix[6]<< matrix[10]<< matrix[14]
        << matrix[3]<< matrix[7]<< matrix[11]<< matrix[15];

    Matrix cov(6,6);

    cov << 0 << 0 << 0 << 0 << 0 << 0
        << 0 << 0 << 0 << 0 << 0 << 0
        << 0 << 0 << 0 << 0 << 0 << 0
        << 0 << 0 << 0 << 0 << 0 << 0
        << 0 << 0 << 0 << 0 << 0 << 0
        << 0 << 0 << 0 << 0 << 0 << 0;

    int type = LO_TYPE_PHYSICAL;

    if(id < ID_WORLD)
    {
      jlo::ServiceLocatedObject* parent = GetServiceLocatedObject(parentid);
      if(parent == NULL)
      {
          printf("Error tf: Requested parent does not exist! (%s -> %ld)\n", trans.child_frame_id_.c_str(),  parentid);
          continue;
      }
      pose = FServiceLocatedObject(parent, mat, cov, type);
      ServiceInterface::AddMapString(pose, trans.child_frame_id_);
    }
    else
    {
      pose = GetServiceLocatedObject(id);
      if (pose == NULL)
      {
        ROS_ERROR("Error tf: Reject overwritten frame %s -> %ld\n", trans.child_frame_id_.c_str(), id);
        continue;
      }

      if(parentid == pose->m_parentID)
      {
          Matrix m = pose->GetMatrix();
          Matrix diff =  mat - m;
          if((fabs(diff.element(0,0)) < 0.000001 &&
              fabs(diff.element(0,1)) < 0.000001 &&
              fabs(diff.element(0,2)) < 0.000001 &&
              fabs(diff.element(0,3)) < 0.000001 &&
              fabs(diff.element(1,0)) < 0.000001 &&
              fabs(diff.element(1,1)) < 0.000001 &&
              fabs(diff.element(1,2)) < 0.000001 &&
              fabs(diff.element(1,3)) < 0.000001 &&
              fabs(diff.element(2,0)) < 0.000001 &&
              fabs(diff.element(2,1)) < 0.000001 &&
              fabs(diff.element(2,2)) < 0.000001 &&
              fabs(diff.element(2,3)) < 0.000001) || pose->m_uniqueID == ID_WORLD)
          {
            continue;
          }
          pose->Update(mat, cov, ServiceInterface::FServiceLocatedObjectCopy, ServiceInterface::FreeServiceLocatedObject, &RosLoService::UpdateEventNotifier);

          //if  ((trans.child_frame_id_.find("torso") != -1 ) || (trans.frame_id_.find("torso") != -1 ))
          //   ROS_ERROR("Getting tf -%s- parent -%s-\n", trans.child_frame_id_.c_str(), trans.frame_id_.c_str());
      }
      else
      {
          printf("tf from  %s with parent %s does not correspond with the actual situation %ld with parentID %ld\n", trans.child_frame_id_.c_str(), trans.frame_id_.c_str(), pose->m_uniqueID, pose->m_parentID);
          pose->m_mapstring = "";
          jlo::ServiceLocatedObject* parent = GetServiceLocatedObject(parentid);
          if(parent == NULL || parent->m_uniqueID != parentid)
          {
              printf("Error tf: Requested parent does not exist! (%s -> %ld)\n", trans.frame_id_.c_str(), parentid);
              continue;
          }
          pose = FServiceLocatedObject(parent, mat, cov, type);
          ServiceInterface::AddMapString(pose, trans.child_frame_id_);
          pose->IncreaseReferenceCounter();
      }
    }

    /*catch (TransformException& ex)
    {
      ///\todo Use error reporting
      std::string temp = ex.what();
      ROS_ERROR("Failure to set recieved transform %s to %s with error: %s\n", msg_in_.transforms[i].header.frame_id.c_str(), msg_in_.transforms[i].parent_id.c_st
r(), temp.c_str());
    }*/
  }



};


