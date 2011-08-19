#ifndef REMOTEATTENTION_H
#define REMOTEATTENTION_H


#include <ros/ros.h>
#include "XMLTag.h"
#include "AttentionAlgorithm.h"

#define XML_NODE_REMOTEATTENTION "RemoteAttention"

extern volatile bool g_stopall;

namespace cop
{
  /**
    * class RemoteAttention
    * @brief Specialisation of AttentionAlgorithm that waits for external events
    */
  template <typename Message>
  class RemoteAttention : public AttentionAlgorithm
  {
  public:

    /**
     * Constructor
     */
    RemoteAttention () : m_bStarted(false), m_neededObjectType(ELEM){};

    /**
     * Destructor
     */
    virtual ~RemoteAttention ( ){};

    typedef boost::mutex::scoped_lock lock;

    /**
     *  Perform
     *  @brief  the function that is called to execute this current algorithm
     *  @param   sensors       empty
     *  @param   pose          Position that can restict the  search
     *  @param   prototype        Description of the object to search for
     *  @param   numOfObjects     number of objects that should be found and on return that were found
     *  @param   qualityMeasure  Takes threshold limiting results quality and receives resulting quality
     *  @return a list of objects which should be attended
     */
    virtual std::vector<Signature*> Perform(std::vector<Sensor*> sensors, RelPose* pose,
                          Signature& prototype, int &numOfObjects, double& qualityMeasure)
    {
      boost::xtime t;
      if(!m_bStarted && !g_stopall)
        Start();
      lock lk(m_mutexNewData);
      ROS_INFO("Waiting for new data\n");

      while(!m_bNew && !g_stopall)
      {
         try
         {
          boost::xtime_get(&t, boost::TIME_UTC);
          t.sec += 1;
           m_newDataArrived.timed_wait(lk, t);
         }
         catch(...)
         {
           ROS_ERROR("Error: condition variable error in RemoteAttention\n");
         }
      }
      ROS_INFO("Got %snew data\n", m_bNew ? "" : "no ");
      m_bNew = false;
      if(!g_stopall)
      {
        std::vector<Signature*> copy = m_latestResult;
        m_latestResult.clear();
        numOfObjects = copy.size();
        printf("returning %d objects\n", numOfObjects);
        qualityMeasure = 0.5;
        return copy;
      }

      return  std::vector<Signature*>();
    }


    /**
    * CheckSignatures
    * local check for preconditions
    */
    virtual double CheckSignature(const Signature& object, const std::vector<Sensor*> &sensors)
    {
      lock lk(m_mutexNewData);
      /*printf("CheckSignature:  %s\n", !m_bStarted || m_bNew ? "true" : "false");*/

      if(!m_bStarted || m_bNew)
      {
        if(m_neededObjectType == ELEM  || object.GetElement(0, m_neededObjectType) != NULL)
        {
          return 1.00;
        }
        else
          return 0.0;
      }
      return 0.0;
    }

    virtual XMLTag* Save()
    {
      XMLTag* tag = new XMLTag(XML_NODE_REMOTEATTENTION);
      SetData(tag);
      return  tag;
    }

    void SaveRemoteProps(XMLTag* tag){tag->AddProperty(XML_ATTRIBUTE_TOPICNAME, m_stTopic);}

    void Start()
    {
      m_bStarted = true;
      try
      {
        ros::NodeHandle node;
        printf("Subscribe to %s\n", m_stTopic.c_str());
        m_subs = node.subscribe<Message>(
                 m_stTopic, 1,
                 boost::bind(&RemoteAttention::CallbackToTopic, this, _1));
       }
       catch(...)
       {
         ROS_ERROR("cop::RemoteAttention: Could not subscribe to %s\n",  m_stTopic.c_str());
       }
    }

    virtual void Stop()
    {
      m_subs.shutdown();
    }

    virtual void SetData(XMLTag* tag)
    {
      m_stTopic = tag->GetProperty(XML_ATTRIBUTE_TOPICNAME, "/cop/attention");
    }

    void CallbackToTopic(boost::shared_ptr<Message const> msg)
    {
      printf("Got something\n");
      std::vector<Signature*> new_sigs = MessageToSignature(msg);
      m_latestResult.insert(m_latestResult.end(), new_sigs.begin(), new_sigs.end());
      lock lk(m_mutexNewData);
      m_bNew = true;
      m_newDataArrived.notify_all();
    }
    virtual std::vector<Signature*> MessageToSignature(boost::shared_ptr<Message const> msg)
    {
      throw "Not implemented";
    }

    void SetObjectType(ElemType_t neededObjectType) {m_neededObjectType = neededObjectType;}

    std::string m_stTopic;
    std::vector<Signature*> m_latestResult;
    bool m_bStarted;
    bool m_bNew;
    ros::Subscriber m_subs;
    boost::condition m_newDataArrived;
    boost::mutex m_mutexNewData;
    ElemType_t m_neededObjectType;

  };
}
#endif
