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

 
#ifdef USE_YARP_COMM
#ifndef YARP_COMM_H
#define YARP_COMM_H

#include "RelPose.h"
#include "VisFinder.h"
#include "Signature.h"
#include "Comm.h"


#include <yarp/os/all.h>
#include <map>

#ifdef BOOST_THREAD
#ifdef BOOST_1_35
#include <boost/thread/mutex.hpp>
#else
#include <boost/thread/detail/lock.hpp>
typedef boost::detail::thread::lock_ops<boost::mutex> locker;
#endif
boost::mutex s_mutexAnswer;
#define BOOST(A) A
#else
#define BOOST(A) ;
#endif


#define STD_COP_OUTPUT_PORT "/tracking/out"

#ifdef BOOST_THREAD
#include <boost/thread.hpp>
#include <boost/bind.hpp>
using namespace boost;
#endif

/********************************************************************
*     CreatePoseFromBottle                                         */
/********************************************************************
*   @brief Creates a lo from a bottle
*********************************************************************/
std::pair<RelPose*, Probability_1D_t> CreatePoseFromBottle(yarp::os::Bottle* lo)
{
    std::pair<RelPose*, Probability_1D_t> result;

    result.first = NULL;
    if(!lo->get(0).isDouble())
      return result;
    result.second = lo->get(0).asDouble();
    if(!lo->get(1).isInt())
      return result;
    /** Containing a parent id,*/
    int id = lo->get(1).asInt();
    RelPose* pose = RelPoseFactory::FRelPose(id);
    /** And two matrices */
    result.first = pose;
    return result;
}

/********************************************************************
*     AddMatrixToABottle                                         */
/********************************************************************
*   @brief Adds a bottle containing a matrix
*********************************************************************/
void AddMatrixToABottle(int width, yarp::os::Bottle& lo, Matrix m)
{
  yarp::os::Bottle &mat_b = lo.addList();;
  for(int r = 0; r < width; r++)
  {
    for(int c = 0; c < width; c++)
    {
        mat_b.addDouble(m.element(r,c));
    }
  }
}
/********************************************************************
*     PutPoseIntoABottle                                         */
/********************************************************************
*   @brief Creates a lo from a bottle
*********************************************************************/
void PutPoseIntoABottle(yarp::os::Bottle &lo, jlo::LocatedObject* pose)
{
  lo.addInt(pose->m_uniqueID);
  if(pose->m_uniqueID != ID_WORLD)
    lo.addInt(pose->m_parentID);
  else
    lo.addInt(1);
  Matrix m = pose->GetMatrix();
  /* Bottle per matrix*/
  int width = 4;
  AddMatrixToABottle(width, lo, m);
  width = 6;
  Matrix cov = pose->GetCovarianceMatrix();
  AddMatrixToABottle(width,lo,cov);
}

/*****************************************************************************************
*  class YarpComm                                                                             */
/*****************************************************************************************
*  Class organizing the answer system to a specifie yarp port
******************************************************************************************/
class YarpComm : public Comm
{
public:

  /*****************************************************************************************
  *  YarpComm                                                                             */
  /*****************************************************************************************
  *  Constructor
  ******************************************************************************************/
  YarpComm(VisFinder& visFinder, PossibleLocations_t* pose, Signature& sig, int numOfObjects, yarp::os::BufferedPort<yarp::os::Bottle>* port, int actionType) :
    m_visFinder(visFinder),
    m_pose(pose),
    m_sig(sig),
    m_numOfObjects(numOfObjects),
    m_port(port),
    m_actionType(actionType)
  {
  }


  /*****************************************************************************************
  *  ~YarpComm                                                                             */
  /*****************************************************************************************
  *  Destructor
  ******************************************************************************************/
  ~YarpComm(void)
  {
#ifdef BOOST_THREAD
    /*delete m_YarpAnswerThread;*/
#endif /*BOOST_THREAD*/
    delete m_pose;
    m_visFinder.m_sigdb.FreeActiveSignature(&m_sig);
    /*delete &m_sig;*/
  }

  /*****************************************************************************************
  *  Start                                                                                */
  /*****************************************************************************************
  *  Calls the yarp threadfunc, with a new thread if this is possible
  ******************************************************************************************/
  void Start()
  {
#ifdef BOOST_THREAD
    /*m_YarpAnswerThread = new */boost::thread( boost::bind(&YarpComm::threadfunc, this));
#else
    YarpComm::threadfunc();
#endif /*BOOST_THREAD*/

  }

  /*****************************************************************************************
  *  NotifyPoseUpdate                                                                        */
  /*****************************************************************************************
  *  Call back that is called whenever a new pose is set for a certain model
  *  This callback must be told the signature that is tracked
  ******************************************************************************************/
  virtual void NotifyPoseUpdate(RelPose* pose, bool sendObjectRelation = true)
  {
#ifdef BOOST_1_35
  BOOST(s_mutexAnswer.lock());
#else
  BOOST(locker::lock(s_mutexAnswer));
#endif

        yarp::os::Bottle& new_loc_bottle = m_port->prepare();
	new_loc_bottle.clear();
	yarp::os::Bottle& subelement = new_loc_bottle.addList();
        if(sendObjectRelation)
        {
          subelement.addInt(m_sig.m_ID);
          subelement.addDouble(pose->m_qualityMeasure);
        }
        subelement.add(pose->m_uniqueID);
/*        PutPoseIntoABottle(new_loc_bottle, pose);*/
        m_port->write();
#ifdef _DEBUG
	printf("Writing a bottle in Pose Update Notification:  %s\n", new_loc_bottle.toString().c_str());
#endif /*_DEBUG*/
#ifdef BOOST_1_35
  BOOST(s_mutexAnswer.unlock());
#else
  BOOST(locker::unlock(s_mutexAnswer));
#endif
  };

  /*****************************************************************************************
  *  threadfunc                                                                           */
  /*****************************************************************************************
  *  Yarp action: calls visual finder, and writes the results on a yarp port
  ******************************************************************************************/
  void threadfunc()
  {
    printf("Answer Thread started for object %d command %s\n", m_sig.m_ID, ((m_actionType == ALGORITHMTYPE_LOCATE) ? "Locate" : ((m_actionType == ALGORITHMTYPE_TRACK) ? "Track" : "StopTrack or unknown action")));
    bool bFinished = false;
    switch(m_actionType)
    {
    case ALGORITHMTYPE_TRACK:
       m_sig.SetCommCallBack(this);
        if(m_pose->size() > 0)
        {
          PossibleLocations_t::const_iterator it = m_pose->begin();
          m_visFinder.StartTrack(m_sig, (*it).first);
        }
        else
          m_visFinder.StartTrack(m_sig, NULL);
       break;
    case ALGORITHMTYPE_STOPTRACK:
        m_sig.SetCommCallBack(NULL);
        m_visFinder.StopTrack(m_sig);
        bFinished = true;
        break;
    case ALGORITHMTYPE_LOCATE:
        {
          Locate(m_pose);
          bFinished = true;
        }
        break;
    }
    printf("Finished  with action of type \"%s\".\nReturning to listen loop.\n", ((m_actionType == ALGORITHMTYPE_LOCATE) ? "Locate" : ((m_actionType == ALGORITHMTYPE_TRACK) ? "Track" : "StopTrack or unknown action")) );
    if(bFinished)
      delete this;
  }
  void Locate(PossibleLocations_t* pose)
  {
        try
        {
          const SignatureLocations_t &new_location = m_visFinder.Locate(pose, m_sig, m_numOfObjects);
#ifdef BOOST_1_35
  BOOST(s_mutexAnswer.lock());
#else
  BOOST(locker::lock(s_mutexAnswer));
#endif
          yarp::os::Bottle& main_bot = m_port->prepare();
          main_bot.clear();
          if(m_numOfObjects > 0 && new_location.size() > 0)
          {
            for(unsigned int i = 0; (signed)i < m_numOfObjects && i < new_location.size(); i++) /**TODO, replace loicate and ...*/
            {
              yarp::os::Bottle& new_loc_bottle = main_bot.addList();
              new_loc_bottle.addInt(new_location[i].second->m_ID);
              new_loc_bottle.addDouble(new_location[i].first->m_qualityMeasure);
    /*          PutPoseIntoABottle(new_loc_bottle, new_location[i].first);*/
              new_loc_bottle.addInt(new_location[i].first->m_uniqueID);
            }
          }
          else
          {
            main_bot.addString("No Object Found!");
          }
          m_port->write();
#ifdef BOOST_1_35
  BOOST(s_mutexAnswer.unlock());
#else
  BOOST(locker::unlock(s_mutexAnswer));
#endif

#ifdef WIN32
#ifdef BOOST_1_35
  BOOST(boost::system_time t);
#else
  BOOST(boost::xtime t);
#endif

#ifdef BOOST_1_35
          BOOST(t = get_system_time());
          BOOST(t +=  boost::posix_time::seconds(1));  //TODO Check

#else
          BOOST(boost::xtime_get(&t, boost::TIME_UTC));
          BOOST(t.sec +=  1);  //TODO Check
#endif

          BOOST(boost::thread::sleep(t));
          delete m_visFinder.m_imageSys.GetCamara(0)->m_win;
          m_visFinder.m_imageSys.GetCamara(0)->m_win = NULL;
#endif
        }
        catch(char const* text)
        {
          printf("Locate called by yarp failed: %s\n", text);
#ifdef BOOST_1_35
  BOOST(s_mutexAnswer.lock());
#else
  BOOST(locker::lock(s_mutexAnswer));
#endif

          yarp::os::Bottle& main_bot = m_port->prepare();
          main_bot.addString("Locate failed: %s");
          m_port->write();
#ifdef BOOST_1_35
  BOOST(s_mutexAnswer.unlock());
#else
  BOOST(locker::unlock(s_mutexAnswer));
#endif

          return;
        }
        catch(...)
        {
#ifdef BOOST_1_35
  BOOST(s_mutexAnswer.lock());
#else
  BOOST(locker::lock(s_mutexAnswer));
#endif

          yarp::os::Bottle& main_bot = m_port->prepare();
          main_bot.addString("Locate failed.");
          m_port->write();
#ifdef BOOST_1_35
  BOOST(s_mutexAnswer.unlock());
#else
  BOOST(locker::unlock(s_mutexAnswer));
#endif

          return;
        }
  }

  yarp::os::BufferedPort<yarp::os::Bottle>* m_port;
  VisFinder& m_visFinder;
  PossibleLocations_t* m_pose;
  Signature& m_sig;
  int m_numOfObjects;
  int m_actionType;
private:

#ifdef BOOST_THREAD
  /*boost::thread* m_YarpAnswerThread;*/
#endif
  YarpComm& operator=(YarpComm&){}
};

/*****************************************************************************************
*  class YarpPortManager                                                                */
/*****************************************************************************************
*  Class organizing opened yarp ports
******************************************************************************************/
class YarpPortManager
{
public:
  YarpPortManager()
  {
  }

  ~YarpPortManager()
  {
    std::map<std::string, yarp::os::BufferedPort<yarp::os::Bottle>* > ::iterator it = m_PortMapping.begin();
    for(; it != m_PortMapping.end(); it++)
    {
      (*it).second->close();
      delete (*it).second;
    }
  }

  void CloseYarpPort(std::string port_name)
  {

  }

  void Listen(yarp::os::BufferedPort<yarp::os::Bottle> & port, volatile bool &g_stopall, VisFinder* s_visFinder, SignatureDB *s_sigDb)
  {
        while(!g_stopall)
        {
          try
          {
            /** Blocking read on the openend port!*/
            yarp::os::Bottle *input = port.read(true);
            if(g_stopall)
              return;
#ifdef _DEBUG
            printf("Got something\n");
  #endif /*_DEBUG*/
            int num = input == NULL ? 0 : input->size();
            if(num == 0)
            {
#ifdef _DEBUG
              printf("Got Empty Bottle\n");
  #endif /*_DEBUG*/
              continue;
            }
            std::string outputname;
            std::vector<int> class_id;
            int i = 0;
            if(input->get(i).isString())
            {
              outputname = input->get(i).asString();
               i++;
            }
            else
            {
              outputname = STD_COP_OUTPUT_PORT;
              printf("YarpComm: Incoming Request did not specify a portname.\n  Using %s\n",STD_COP_OUTPUT_PORT);
            }
            try
            {
              if(i > num)
              {
#ifdef _DEBUG
                printf("Got Bottle of a too short length\n");
    #endif /*_DEBUG*/
                continue;
              }
              Signature* sig = NULL;
              if(input->get(i).isList())
              {
                yarp::os::Bottle* bot = input->get(i).asList();
                int num_list = bot->size();
#ifdef _DEBUG
                printf("Yarp Comm: Got %d describing entries\n", num_list);
    #endif /*_DEBUG*/
                try
                {
                  for(int j = 0; j < num_list ; j++)
                  {
                    if(bot->get(j).isInt())
                    {
                      int index;
                      int id = bot->get(j).asInt();
                      if(s_sigDb->CheckClass(id).length() > 0)
                      {
                        class_id.push_back(id);
                      }
                      else if(s_sigDb->Check(id, index))
                      {
                        sig = s_sigDb->GetSignatureByID(id);
                      }
                      else
                      {
                        printf("Received unknown Element id: %d\n", id);
                      }
                    }
                    else if(bot->get(j).isString())
                    {
                      int class_from_string = s_sigDb->CheckClass(std::string(bot->get(j).asString()));
                      printf("  Class as string: %s\n", bot->get(j).asString().c_str());
                      if(class_from_string != -1)
                        class_id.push_back(class_from_string);
                      else
                      {
                        Class cl;
                        cl.SetName(std::string(bot->get(j).asString()));
                        s_sigDb->AddClass(cl.GetName(), cl.m_ID);
                        int class_from_string = s_sigDb->CheckClass(std::string(bot->get(j).asString()));
                        if(class_from_string != -1)
                          class_id.push_back(class_from_string);
                      }
                    }
                    else if(bot->get(j).isList())
                    {
                      /** This is the case if more than one object has to searched synchronously and not overlapping*/
                      throw "Not yet implemented";
                    }
                  }
                }
                catch(...)
                {
                  printf("YarpComm: Problems reading classes\n");
                  continue;
                }
              }
              else
              {
                printf("Error parsing bottle: Element %d was not of expected type(expected Bottle of ints + strings)", i);
                continue;
              }
              if(sig == NULL)
              {
                try
                {
                  sig = s_sigDb->GetSignature(class_id);
                  if(sig == NULL)
                  {
                    printf("YarpComm: Could not generatesignature for the requested description (%s)\n", input->toString().c_str());
                    continue;
                  }
                }
                catch(char const* text)
                {
                  printf("Error Creating signature: %s\n", text);
                  break;
                }
                catch(...)
                {
                  printf("Error Creating signature\n");
                  break;
                }
              }
#ifdef _DEBUG
              printf("Created Signature with all classes (%d)\n", sig->m_ID);
    #endif
              i++;
              if(!input->get(i).isInt())
              {
                printf("Error parsing bottle: Element %d was not of expected type (expected number of elements)", i);
                  continue;
              }
              int numOfObjects = input->get(i).asInt();
              i++;
              if(!input->get(i).isInt())
              {
                printf("Error parsing bottle: Element %d was not of expected type (expected action type (Track or Locate))", i);
                  continue;
              }
              int actionType = input->get(i).asInt();
              i++;
              PossibleLocations_t* poses = new PossibleLocations_t();
              if(input->get(i).isList())
              {
                yarp::os::Bottle* bot = input->get(i).asList();
                int num_list = bot->size();
                /** Now we want a list of lo*/
                for(int j = 0; j < num_list ; j++)
                {
                  if(!bot->get(j).isList())
                    continue;
                  /** an lo is represented in one bottle*/
                  yarp::os::Bottle* lo = bot->get(j).asList();
                  std::pair<RelPose*, double> pose =  CreatePoseFromBottle(lo);
                  if(pose.first == NULL)
                  {
                    printf("YarpComm: Received Pose is NULL!\n");
                    break;
                  }
                  poses->push_back(pose);
                  printf("Start Locate with:\n", sig->m_ID);
#ifdef _DEBUG
                  pose.first->Print();
    #endif
                }
#ifdef _DEBUG
    #endif /**USE_YARP_COMM*/
              }
              else
              {
                printf("No poses given, trying last known pose.\n");
                if(sig->m_relPose != NULL)
                  poses->push_back(std::pair<RelPose*, double>(sig->m_relPose, 1.0));
                else
                  printf("A Bottle was received that could not be used for finding the requested object\n.");
              }
              yarp::os::BufferedPort<yarp::os::Bottle>* port = OpenCommOnYarpPort(outputname);
              YarpComm* comm = new YarpComm(*s_visFinder, poses, *sig, numOfObjects, port, actionType);
              comm->Start();
            }
            catch(char const* text)
            {
              printf("Could not open outputport TOCHECK, wait? %s\n", text);
              continue;
            }
          }
          catch(char const* text)
          {
            printf("StartListeningYarpPort Exception: %s\n", text);
          }
          catch(...)
          {
            printf("StartListeningYarpPort Unknown Exception\n");
          }
        }
  }

  yarp::os::BufferedPort<yarp::os::Bottle>* OpenCommOnYarpPort(std::string st)
  {
      if(m_PortMapping.find(st) == m_PortMapping.end())
      {
        m_PortMapping[st] = new yarp::os::BufferedPort<yarp::os::Bottle>();
        try
        {
          yarp::os::Contact contact = yarp::os::Network::queryName(st.c_str());
          if(!contact.isValid() && m_PortMapping[st]->open(st.c_str()))
          {
            return m_PortMapping[st];
          }
          else
          {
            system ("yarp clean");
            for(int i = 0; i < 1000; i++)
            {
              yarp::os::Contact contact = yarp::os::Network::queryName(st.c_str());
              if(!contact.isValid())
                break;
            }
            if(m_PortMapping[st]->open(st.c_str()))
            {
              return m_PortMapping[st];
            }
            else
            {
              printf("YarpManager::OpenCommOnYarpPort : Error (Try: yarp clean  before starting again)\n");
              throw "Error opening port";
            }
          }
        }
        catch(char * text)
        {
              printf("YarpManager::OpenCommOnYarpPort : (%s) (Try: yarp clean  before starting again)\n", text);
              throw "Error opening port";
        }
        catch(...)
        {
          printf("YarpManager::OpenCommOnYarpPort : Unknown Exception: Try Yarp Clean\n");
          throw "Error opening port";
        }
      }
      else
        return m_PortMapping[st];
  }
private:
  std::map<std::string, yarp::os::BufferedPort<yarp::os::Bottle>* > m_PortMapping;
};

#endif /*YARP_COMM_H*/
#endif /*USE_YARP_COMM*/

