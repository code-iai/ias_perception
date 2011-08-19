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


#ifndef USE_YARP_COMM




#include "ROSComm.h"
using namespace vision_msgs;
using namespace vision_srvs;

#include "BoostUtils.h"

#ifdef BOOST_THREAD
#include <boost/thread/mutex.hpp>
#include <sstream>
boost::mutex s_mutexAnswer;
#endif

using namespace cop;

unsigned long Comm::s_lastCommID = 0;

/********************************************************************
*     CreatePoseFromMessage                                         */
/********************************************************************
*   @brief Creates a lo from a bottle
*********************************************************************/
std::pair<RelPose*, Probability_1D_t> CreatePoseFromMessage(const apriori_position& call)
{
    std::pair<RelPose*, Probability_1D_t> result;
    result.second = call.probability;
    result.first = RelPoseFactory::FRelPose(call.positionId);
    if(result.first == NULL)
    {
      printf("ROSComm: Major Error in Message: got an undefined pose\n");
      throw "Wrong Pose";
    }
    return result;
}

std::string AlgorihtmTypeToName(int type)
{
  std::string name = "Algorithm";
  switch(type)
  {
    case ALGORITHMTYPE_LOCATE:
    name = "Locate";
    break;
    case ALGORITHMTYPE_TRACK:
    name = "Track";
    break;
    case ALGORITHMTYPE_2OBJS:
    name = "Locate2Obj";
    break;
    case ALGORITHMTYPE_REFINE:
    name = "Refine";
    break;
    case ALGORITHMTYPE_RPOVE:
    name = "Prove";
    break;
    case ALGORITHMTYPE_STOPTRACK:
    name = "StopTrack";
    break;
    case ALGORITHMTYPE_STARTATTEND:
    name = "StartAttend";
    break;
    case ALGORITHMTYPE_STOPATTEND:
    name = "StopAttend";
    break;
    case ALGORITHMTYPE_LOOKUP:
    name = "Database LookUp";
    break;
    case ALGORITHMTYPE_LOOKUPDB:
    name = "Database: Subscribe to All";
    break;
    case ALGORITHMTYPE_LOOKUPALL:
    name = "Database: Subscribe to All";
    break;
  }
  return name;
}

void PutPoseIntoAMessage(cop_answer &answer, SignatureLocations_t new_location)
{
  for(unsigned int i = 0; i < new_location.size(); i++)
  {
    aposteriori_position apo_pose;
    apo_pose.objectId = new_location[i].second->m_ID;
    apo_pose.probability = new_location[i].first->m_qualityMeasure;
    apo_pose.position = new_location[i].first->m_uniqueID;
    if(new_location[i].first->m_uniqueID == 0 && new_location[i].second->GetObjectPose() != NULL)
    {
      apo_pose.probability = new_location[i].second->GetObjectPose()->m_qualityMeasure;
      apo_pose.position = new_location[i].second->GetObjectPose()->m_uniqueID;
    }
    int elems_c = new_location[i].second->CountElems();
    for(int j = 0; j < elems_c; j++)
    {
      Descriptor* descriptor = (Descriptor*)new_location[i].second->GetElement(j, ELEM);
      if(descriptor != NULL)
      {
        vision_msgs::cop_descriptor descr;
        if(descriptor->GetClass() != NULL)
        {
          descr.sem_class = descriptor->GetClass()->GetName();
          descr.object_id = descriptor->m_ID;
          descr.type      = descriptor->GetNodeName();
          descr.quality   = descriptor->GetQuality();
          if(descr.type.compare("NamedClass") == 0)
            apo_pose.models.insert(apo_pose.models.begin(), descr);
          else
            apo_pose.models.push_back(descr);
        }
      }
    }
    answer.found_poses.push_back(apo_pose);
  }
}


void PutPoseIntoAMessage(cop_answer &answer, Signature* sig)
{
  aposteriori_position apo_pose;
  apo_pose.objectId = sig->m_ID;
  int elems_c = sig->CountElems();
  RelPose* pose = sig->GetObjectPose();
  unsigned long max_timestamp = sig->date();
  double pose_qual = 0.0;
  for(int j = 0; j < elems_c; j++)
  {
    Descriptor* descriptor = (Descriptor*)sig->GetElement(j, ELEM);
    if(descriptor != NULL)
    {
      vision_msgs::cop_descriptor descr;
      if( descriptor->GetClass() != NULL)
      {
        descr.sem_class = descriptor->GetClass()->GetName();
        descr.object_id = descriptor->m_ID;
        descr.type      = descriptor->GetNodeName();
        descr.quality   = descriptor->GetQuality();
        if(descriptor->date() > max_timestamp || pose == NULL)
        {
          pose = descriptor->GetLastMatchedPose();
          if(pose != NULL)
          {
            max_timestamp = descriptor->date();
            pose_qual = pose->m_qualityMeasure;
          }
        }
        apo_pose.models.push_back(descr);
      }
    }
  }
  apo_pose.probability = pose_qual;
  if(pose != NULL)
    apo_pose.position = pose->m_uniqueID;
  answer.found_poses.push_back(apo_pose);
}

void ROSComm::NotifyPoseUpdate(RelPose* pose, bool sendObjectRelation)
{
  try
  {
    cop_answer answer;

    PutPoseIntoAMessage(answer, m_visPrim.GetSignature());
    if(pose != NULL)
    {
      answer.found_poses[0].probability = pose->m_qualityMeasure;
      answer.found_poses[0].position = pose->m_uniqueID;
      answer.perception_primitive = m_visPrim.GetID();
    }
    else
    {
        answer.error = "Object lost";
    }
    PublishAnswer(answer);
  }
  catch(...)
  {
    printf("Problems publishing answer in ROSComm::NotifyPoseUpdate\n");
  }
#ifdef _DEBUG
  /*printf("Writing a bottle in Pose Update Notification:  %s\n", new_loc_bottle.toString().c_str());*/
#endif /*_DEBUG*/
}

void ROSComm::NotifyNewObject(Signature* sig, RelPose* pose)
{
  try
  {
    cop_answer answer;
    PutPoseIntoAMessage(answer, sig);
    if(pose != NULL)
    {
      answer.found_poses[0].probability = pose->m_qualityMeasure;
      answer.found_poses[0].position = pose->m_uniqueID;
    }
    else
    {
      answer.found_poses[0].probability = 0.0;
      answer.found_poses[0].position = 0;
      answer.error = "Object not localized";
    }
    answer.perception_primitive = m_visPrim.GetID();
    PublishAnswer(answer);
  }
  catch(...)
  {
    printf("Problems publishing answer in ROSComm::NotifyNewObject\n");
  }
#ifdef _DEBUG
  /*printf("Writing a bottle in Pose Update Notification:  %s\n", new_loc_bottle.toString().c_str());*/
#endif /*_DEBUG*/
}


ROSComm::~ROSComm()
{
  /*for(int i = 0; i < m_pose->size(); i++)
    RelPoseFactory::FreeRelPose((*m_pose)[i].first);*/
  printf("\n\n\nDelete ROS COMM and set Vision Primitive %ld  to terminated\n\n\n", m_visPrim.GetID());
  m_visPrim.SetTerminated();
  delete m_pose;
}

void ROSComm::Start()
{
#ifdef BOOST_THREAD
  boost::thread( boost::bind(&ROSComm::ProcessCall, this));
#else
  ROSComm::ProcessCall(this);
#endif /*BOOST_THREAD*/
}

void ROSComm::PublishAnswer(cop_answer &answer)
{
  int timeout = 0;
  if(m_callerid.length() == 0 && m_publisher != NULL)
  {
    while(m_publisher->getNumSubscribers() < 1 && timeout < 100)
    {
      if(timeout % 20 == 0)
        printf("No subscribers, waiting\n");
      Sleeping(10);
      timeout++;
    }
  }
  else
  {
    while(m_publisher != NULL && m_publisher->getNumSubscribers() < 1 && timeout < 100)
    {
      if(timeout % 20 == 0)
        printf("No subscribers, waiting\n");
      Sleeping(10);
      timeout++;
    }
    bool right_publisher = false;
    while(timeout < 100)
    {
      if(m_calleridMap.find(m_callerid) == m_calleridMap.end())
      {
        ROS_WARN("Problems with caller %s\n", m_callerid.c_str());
         m_calleridMap[m_callerid] = std::vector<std::string> ();
      }
      else
      {
        if(m_publisher == NULL)
        {
          ROS_ERROR("cop ros comm: Empty publisher, this is unexpected\n");
        }
        else
        {
          for(size_t i = 0;  i < m_calleridMap[m_callerid].size(); i++)
          {
            if(m_calleridMap[m_callerid][i].compare(m_publisher->getTopic()) == 0)
            {
              right_publisher = true;
              break;
            }

          }
        }
        if(right_publisher)
          break;
      }
      printf("Wrong subscribers, waiting for %s\n", m_callerid.c_str());
      Sleeping(50);
      timeout++;
    }
  }
  if(m_publisher != NULL)
  {
    BOOST(s_mutexAnswer.lock());
    try
    {
      answer.perception_primitive = m_visPrim.GetID();
      m_publisher->publish(answer);
    }
    catch(...)
    {
       printf("Problems publishing answer in ROSComm::PublishAnswer\n");
    }
    BOOST(s_mutexAnswer.unlock());
  }
}


void ROSComm::ProcessCall()
{
#ifdef _DEBUG
    printf("Answer Thread started for object %ld with command %s\n", m_visPrim.GetSignature()->m_ID, AlgorihtmTypeToName(m_actionType).c_str());
#endif
    bool bFinished = false;
    cop_answer answer;

    switch(m_actionType)
    {
    case ALGORITHMTYPE_LOOKUPDB:
        m_visFinder->m_sigdb->SetNewObjectCallback(this, false);
        bFinished = true;
        break;
  case ALGORITHMTYPE_LOOKUPALL:
        m_visFinder->m_sigdb->SetNewObjectCallback(this);
        bFinished = false;
        break;
    case ALGORITHMTYPE_LOOKUP:
        PutPoseIntoAMessage(answer, m_visPrim.GetSignature());
        PublishAnswer(answer);
        break;
    case ALGORITHMTYPE_REFINE:
        try
        {
          const SignatureLocations_t &new_location = m_visFinder->m_visLearner->RefineObject(m_pose, m_visPrim, m_numOfObjects);
          if(new_location.size() > 0)
          {
            PutPoseIntoAMessage(answer, new_location);
          }
          else
          {
            SignatureLocations_t old_location;
            std::pair<RelPose*, Signature*> olddata;
            if(m_pose->size() > 0)
                olddata.first = (*m_pose)[0].first;
            else
                olddata.first = NULL;
            olddata.second = m_visPrim.GetSignature();
            old_location.push_back(olddata);
            PutPoseIntoAMessage(answer, old_location);
            answer.error = "No Refinement Found!";
          }
          PublishAnswer(answer);
        }
        catch(const char  *error)
        {
          printf("Refine failed: %s\n", error);
          std::string error_copy = error;
          answer.error = error_copy;
          PublishAnswer(answer);
        }
        catch(...)
        {
          answer.error = "Refine failed";
          PublishAnswer(answer);
        }
        bFinished = true;
        break;
    case ALGORITHMTYPE_TRACK:
       m_visPrim.GetSignature()->SetCommCallBack(this);
        if(m_pose->size() > 0)
        {
          PossibleLocations_t::const_iterator it = m_pose->begin();
          m_visFinder->StartTrack(m_visPrim, (*it).first);
        }
        else
          m_visFinder->StartTrack(m_visPrim, NULL);
       break;
    case ALGORITHMTYPE_STOPTRACK:
        m_visPrim.GetSignature()->SetCommCallBack(NULL);
        m_visFinder->StopTrack(*m_visPrim.GetSignature());
        bFinished = true;
        break;
    case ALGORITHMTYPE_STARTATTEND:
        m_visFinder->m_attentionMan->SetObjectToAttend(&m_visPrim,m_pose, this);
        bFinished = false;
        break;
    case ALGORITHMTYPE_STOPATTEND:
        m_visFinder->m_attentionMan->StopAttend(this);
        bFinished = true;
        break;

    case ALGORITHMTYPE_LOCATE:
      try
      {
        const SignatureLocations_t &new_location = m_visFinder->Locate(m_pose, m_visPrim, m_numOfObjects);
        if(m_numOfObjects > 0 && new_location.size() > 0)
        {
           PutPoseIntoAMessage(answer, new_location);
        }
        else
        {
          answer.error = "No Object Found!";
        }
        PublishAnswer(answer);
      }
      catch(char const* text)
      {
        printf("Locate called by ros failed: %s\n", text);
        stringstream st;
        st << "Locate failed: " << text;
        answer.error = st.str();
        PublishAnswer(answer);
      }
      catch(...)
      {
        answer.error = "Locate failed";
        PublishAnswer(answer);
      }
      bFinished = true;
      break;
    default:
       answer.error = "Unknown command";
       PublishAnswer(answer);
       bFinished = true;
       break;
    }
#ifdef _DEBUG
    printf("Finished  with action of type \"%s\".\nReturning to listen loop.\n", AlgorihtmTypeToName(m_actionType).c_str());
#endif
    /*TODO delete this*/
    if(bFinished)
      delete this;

}


ROSTopicManager::ROSTopicManager(VisFinder* visFinder, SignatureDB *sigDb) :
  m_visFinder(visFinder),
  m_sig(sigDb)
{
}

ROSTopicManager:: ~ROSTopicManager()
{
  for(std::map<std::string, ros::Publisher*>::iterator it = m_openTopics.begin(); it != m_openTopics.end(); it++)
  {
    delete (*it).second;
  }
}

void ROSTopicManager::CloseROSTopic(std::string name)
{
}


void ROSTopicManager::Subscriber(const ros::SingleSubscriberPublisher& subs)
{
  std::string topic = subs.getTopic();
  std::string subscriber = subs.getSubscriberName();
  printf("\n\nCop got new subscriber: %s at topic %s\n\n", subscriber.c_str(), topic.c_str());
  if(m_subscriberPerTopics.find(subscriber) != m_subscriberPerTopics.end())
  {
      m_subscriberPerTopics[subscriber].push_back(topic);
  }
  else
  {
    std::vector<string> vst;
    vst.push_back(topic);
    m_subscriberPerTopics[subscriber] = vst;
  }
}


/*void ROSTopicManager::ListenCallBack(const boost::shared_ptr<const cop_call> &msg)*/
bool ROSTopicManager::ListenCallBack(cop_call::Request& msg, cop_call::Response&  answer)
{
#ifdef _DEBUG
  printf("Entering ROSTopicManager::ListenCallBack with find %p sigdb %p\n", m_visFinder, m_sig);
  printf("Got Message: noo %ld \n", msg.number_of_objects);
#endif
  std::vector<ObjectID_t> class_id;
  std::string callerid;
  try
  {
    callerid = (*msg.__connection_header)["callerid"];
  }
  catch(...)
  {
    callerid = "";
  }
  std::string topicname = msg.outputtopic.length() == 0 ? STD_COP_OUTPUT_PORT : msg.outputtopic;
  Signature* sig = NULL;
  PossibleLocations_t* poses = new PossibleLocations_t();

  for(unsigned int i = 0; i < msg.object_classes.size(); i++)
  {
    try
    {
      int class_from_string = m_sig->CheckClass(msg.object_classes[i]);
#ifdef _DEBUG
      printf("  Class as string: %s -> %d\n", msg.object_classes[i].c_str(), class_from_string);
#endif
      if(class_from_string != -1)
        class_id.push_back(class_from_string);
      else
      {
        Class cl;
        cl.SetName(msg.object_classes[i]);
        cl.m_ID = m_sig->AddClass(cl.GetName(), cl.m_ID);
        int class_from_string = m_sig->CheckClass(msg.object_classes[i]);
        if(class_from_string != -1)
        {
          class_id.push_back(class_from_string);
          printf("Added Class %s as %ldth element\n", msg.object_classes[i].c_str(), class_id.size());
        }

      }
    }
    catch(...)
    {
        printf("ROSComm: Problems reading classes\n");
        sig = NULL;
    }
  }
  for(unsigned int j = 0; j < msg.object_ids.size(); j++)
  {
    try
    {
        int index;
        ObjectID_t id = msg.object_ids[j];
        if(m_sig->CheckClass(id).length() > 0)
        {
          class_id.push_back(id);
        }
        else if(m_sig->Check(id, index))
        {
          sig = m_sig->GetSignatureByID(id);
        }
        else
        {
          printf("Received unknown Element id: %ld\n", id);
        }
    }
    catch(...)
    {
        printf("ROSComm: Problems reading classes\n");
        sig = NULL;
    }
  }
  if(sig == NULL)
  {
     try
     {
        sig = m_sig->GetSignature(class_id);
        if(sig == NULL)
        {

          printf("ROSComm: Could not generate signature for the requested description, trying to download\n");
          return false;
        }
     }
     catch(char const* text)
     {
       printf("Error Creating signature: %s\n", text);
       sig = NULL;
     }
     catch(...)
     {
       printf("Error Creating signature\n");
       sig = NULL;
     }
  }
  else
  {
    m_sig->CompleteSignature(sig, class_id);
  }
#ifdef _DEBUG
  if(sig != NULL)
  {
    printf("Created Signature with all classes (%ld)\n", sig->m_ID);
  }
#endif
  if(sig == NULL)
  {
    ROS_INFO("Reaction on cop_call: Creating an empty signature since no further informtion was passed\n");
    sig = new Signature();
    printf("Created new and empty Signature (%ld)\n", sig->m_ID);
    m_sig->AddSignature(sig);
  }
  for(unsigned int k = 0; k < msg.list_of_poses.size(); k++)
  {
    try
    {
      poses->push_back(CreatePoseFromMessage(msg.list_of_poses[k]));
    }
    catch(char* txt)
    {
      continue;
    }
  }

  if(m_openTopics.find(topicname) == m_openTopics.end())
  {
    ros::NodeHandle n;

    ROS_INFO("Publisher pub = n.advertise<cop_answer>(%s, 1000)\n", topicname.c_str());
    ros::Publisher* pub= new ros::Publisher();
    *pub = n.advertise<cop_answer>(topicname, 5, boost::bind(&ROSTopicManager::Subscriber, this, _1));
/*     ros::Rate r(1);
     r.sleep();
    cop_answer answer;
     answer.error = "No Object Found!";
     pub->publish(answer);
     printf("Published?\n");*/
     m_openTopics[topicname] = pub;
  }
  PerceptionPrimitive& vis = m_sig->CreateNewPerceptionPrimitive(sig);
  ROSComm* comm = new ROSComm(m_visFinder, poses, vis, m_openTopics[topicname], (int)msg.number_of_objects, (int)msg.action_type, callerid, this->m_subscriberPerTopics);
  comm->Start();
  answer.perception_primitive = vis.GetID();
  return true;
}

#ifdef WIN32
#include <direct.h>
#define GETCWD(A, B) _getcwd(A, B)
#else
#include <unistd.h>
#define GETCWD(A, B) getcwd(A, B)
#endif


bool ROSTopicManager::SaveCallBack(cop_save::Request& msg, cop_save::Response&  answer)
{
  try
  {
    ObjectID_t id = msg.object_id;
    char cwd[2048];
    char * cwdptr;
    std::ostringstream os;
    Signature* sig = NULL;
    int index;
    if(m_sig->CheckClass(id).length() > 0)
    {
      std::vector<ObjectID_t> class_id;
      class_id.push_back(id);
      sig = m_sig->GetSignature(class_id);
    }
    else if(m_sig->Check(id, index))
    {
      sig = m_sig->GetSignatureByID(id);
    }
    else
    {
       ROS_ERROR("Failed to find a specific object (%ld)\n", id);
       return false;
    }
    XMLTag* tag = sig->Save(true);


    cwdptr = GETCWD(cwd, 2048);

    printf("cwd:  %s / %s\n", cwdptr, cwd);

    os << cwd << "/signature" << tag->date() << ".xml";

    tag->WriteToFile(os.str());

    answer.xmlfilename = os.str();
    answer.filenames =  tag->GetSubFilenames();

  }
  catch(const char *text)
  {
     ROS_ERROR("Failed to save an object: %s", text);
     return false;

  }
  catch(...)
  {
    ROS_ERROR("Failed to save an object");
    return false;
  }

  return true;
}

void ROSTopicManager::NewSignatureCallBack(std_msgs::String::ConstPtr xmlFilename)
{
  try
  {
    XMLTag* signature = XMLTag::ReadFromFile(xmlFilename->data);
    std::string global_path = xmlFilename->data.substr(0, xmlFilename->data.find_last_of("/") + 1);
    signature->ReplaceSubFilenames(global_path);
    if(signature != NULL)
    {
      if(signature->GetName().compare(XML_NODE_SIGNATURE) == 0)
      {
        Signature* sig = (Signature*)Elem::ElemFactory(signature);
        if(sig != NULL)
        {
          m_sig->AddSignature(sig);
          ROS_INFO("Signature created remotely. num descriptors: %ld\n", sig->CountElems());
        }
      }
      else
      {
        ROS_ERROR("Unexpected Type incoming\n");
      }
    }
    else
    {
      ROS_ERROR("Failed ot load %s\n", xmlFilename->data.c_str());
    }
  }
  catch(const char* text)
  {
    printf("Error loading incoming Signature file: %s\n", text);
  }
}

typedef AlgorithmSelector<std::vector<RelPose*> >  LocateEval;
typedef  AlgorithmSelector<Descriptor*>            RefineEval;
typedef AlgorithmSelector<ImprovedPose >            ProoveEval;
typedef AlgorithmSelector<std::vector<Signature*> >  AttendEval;


typedef AlgorithmSelector<std::vector<RelPose*> > LocateSelect;
typedef AlgorithmSelector<std::vector<Descriptor*> > RefineSelect;
typedef AlgorithmSelector<std::vector<ImprovedPose> > ProoveSelect;
typedef AlgorithmSelector<std::vector<Signature*> > AttendSelect;

template<typename T>
void AddEvals(const std::vector<AlgorithmEval< T > > &locators, vision_srvs::cop_get_methods_list::Response& answer)
{

  /*std::vector< AlgorithmEval<T> >::const_iterator iter */

  for(size_t i = 0; i < locators.size(); i++)
  {
    vision_msgs::algorithm_evaluation eval;
    eval.plugin_name = locators[i].m_algorithm->GetName();
    eval.basic_evaluation = locators[i].m_eval;
    //printf("Alg %s with length of objects %ld\n", locators[i].m_algorithm->GetName().c_str(), locators[i].m_objectEval.size());
    const std::map<ObjectID_t, std::pair<double, int> > map = locators[i].GetMap();
    std::map<ObjectID_t, std::pair<double, int> >::const_iterator inner =  map.begin();
    for(;inner != map.end(); inner++)
    {
      vision_msgs::object_algorithm_relation obj;
      obj.object_id = (*inner).first;
      obj.basic_evaluation = (*inner).second.first;
      eval.objects.push_back(obj);
    }
    answer.method.push_back(eval);
  }
}


bool ROSTopicManager::ListProgramCallBack(vision_srvs::cop_get_methods_list::Request& msg, vision_srvs::cop_get_methods_list::Response& answer)
{
  const LocateEval&  locators   = m_visFinder->GetAlgorithmSelection();
  const RefineEval&  refiners   = m_visFinder->m_visLearner->GetRefineAlgorithmSelection();
  const ProoveEval&  proovers   = m_visFinder->m_visLearner->GetProoveAlgorithmSelection();
  const AttendEval&   attendants = m_visFinder->m_attentionMan->GetAttentionAlgorithmSelection();

  /**
     #define ALGORITHMTYPE_LOCATE            0x000
     #define ALGORITHMTYPE_TRACK             0x010
     #define ALGORITHMTYPE_REFINE            0x100
     #define ALGORITHMTYPE_2OBJS             0x200
     #define ALGORITHMTYPE_RPOVE             0x400
     #define ALGORITHMTYPE_STARTATTEND       0x1000
  **/
  printf("Queried msg.algorithmtype = %ld (msg.algorithmtype & ALGORITHMTYPE_STARTATTEND == %s)\n", msg.algorithmtype, msg.algorithmtype & ALGORITHMTYPE_STARTATTEND ? "true" : "false");
  if(msg.algorithmtype == 0 || msg.algorithmtype & ALGORITHMTYPE_TRACK)
  {
    AddEvals(locators.GetAlgorithmList(), answer);
  }
  if(msg.algorithmtype == 0 || msg.algorithmtype & ALGORITHMTYPE_REFINE)
  {
    AddEvals(refiners.GetAlgorithmList(), answer);
  }
  if(msg.algorithmtype == 0 || msg.algorithmtype & ALGORITHMTYPE_RPOVE)
  {
    AddEvals(proovers.GetAlgorithmList(), answer);
  }
  if(msg.algorithmtype == 0 || msg.algorithmtype & ALGORITHMTYPE_STARTATTEND)
  {
    AddEvals(attendants.GetAlgorithmList(), answer);
  }

  return true;
}


void ROSTopicManager::FeedbackCallBack(vision_msgs::cop_feedback::ConstPtr feedback)
{
  std::vector<ObjectID_t> whitelist;
  for(size_t k = 0; k < feedback->eval_whitelist.size(); k++)
  {
    whitelist.push_back(feedback->eval_whitelist[k]);
  }
  if( feedback->error.size() > 0)
  {
    for(size_t i = 0; i < feedback->error.size(); i++)
    {
      printf("got a feedback:\n Error: %ld (node: %s, description: %s)\n", feedback->error[i].error_id, feedback->error[i].node_name.c_str(), feedback->error[i].error_description.c_str());
      if(feedback->error[0].error_id == vision_msgs::system_error::GRASP_FAILED)
      {
        printf("Negative eval on this error\n");
        m_sig->EvaluatePerceptionPrimitive(feedback->perception_primitive, feedback->evaluation,STARTING_WEIGHT, whitelist);
      }
      else if(feedback->error[0].error_id == vision_msgs::system_error::CONTRADICTING_VISION_RESULTS)
      {
        printf("Negative eval on this error\n");
        m_sig->EvaluatePerceptionPrimitive(feedback->perception_primitive, feedback->evaluation,STARTING_WEIGHT, whitelist);
      }
    }
  }
  else
  {
      printf("Got positive feedback for PP %ld   (%f)\n", feedback->perception_primitive, feedback->evaluation);
      m_sig->EvaluatePerceptionPrimitive(feedback->perception_primitive, feedback->evaluation, STARTING_WEIGHT, whitelist);
  }
}

void ROSTopicManager::Listen(std::string name, volatile bool &g_stopall, ros::NodeHandle* node)
{
  /**
   * Now we subscribe using the normal method, to demonstrate the difference.
   */
  /*ros::Publisher pub = n.advertise<cop_call>(name, 1000);*/

  ROS_INFO("advertiseService<cop_call> (%s, ...)\n", node->resolveName(name).c_str());
  ros::ServiceServer srv = node->advertiseService(node->resolveName(name), &ROSTopicManager::ListenCallBack, this);

  ROS_INFO("advertiseService<cop_save> (%s, ...)\n", node->resolveName("save").c_str());
  ros::ServiceServer savesrv = node->advertiseService(node->resolveName("save"), &ROSTopicManager::SaveCallBack, this);

  ROS_INFO("advertiseService<cop_save> (%s, ...)\n", node->resolveName("listmethods").c_str());
  ros::ServiceServer listmethods = node->advertiseService(node->resolveName("listmethods"), &ROSTopicManager::ListProgramCallBack, this);

  ROS_INFO("Subscribe to topic %s\n", node->resolveName("new_signatures").c_str());
  ros::Subscriber sub_add_sig =
        node->subscribe<std_msgs::String>(
                 node->resolveName("new_signatures"), 1000,
                 boost::bind(&ROSTopicManager::NewSignatureCallBack, this, _1));

  ROS_INFO("Subscribe to topic %s\n", node->resolveName("feedback").c_str());
  ros::Subscriber sub_feedback =
        node->subscribe<vision_msgs::cop_feedback>(
                 node->resolveName("feedback"), 1000,
                 boost::bind(&ROSTopicManager::FeedbackCallBack, this, _1));

  ROS_INFO("Publish topic %s\n", node->resolveName("status").c_str());
  m_statusPublisher = node->advertise<cop_status>(node->resolveName("status").c_str(), 1);
  boost::thread(boost::bind(&ROSTopicManager::StatusThread, this));
  ros::Rate r(10);
  while (node->ok() && !g_stopall)
  {
    printf("Call ros spin \n");
    try
    {
      ros::spin();
    }
    catch(char const * text)
    {
      printf("Error while spinning: %s\n", text);
    }
    catch(...)
    {
      printf("Unknown Error while spinning\n");
    }
    r.sleep();
  }
  printf("Returning from ros spinning\n");
  return;
}

bool ROSTopicManager::OpenCommOnROSTopic(std::string st)
{
  return true;
}

extern volatile bool g_stopall;

void ROSTopicManager::StatusThread()
{
  ros::Rate r(10);
  r.sleep();
  while(!g_stopall)
  {
    std::vector<std::pair <PerceptionPrimitiveID_t, PerceptionPrimitiveState> >  result = m_sig->GetCurrentRunState();
    vision_msgs::cop_status st;
    std::vector<std::pair <PerceptionPrimitiveID_t, PerceptionPrimitiveState> >::const_iterator it = result.begin();
    for(;it != result.end();it++)
    {
      vision_msgs::pp_status pp;
      pp.perception_primitive = (*it).first;
      pp.status  = (*it).second;
      st.cop_status.push_back(pp);
    }
    m_statusPublisher.publish(st);
    r.sleep();
  }
}

#endif /*USE_YARP_COMM*/

