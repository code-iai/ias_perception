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
#ifndef YARPRPCCOMM_H
#define YARPRPCCOMM_H

#include "RelPose.h"
#include "Comm.h"


#include <yarp/os/all.h>

#define STD_LO_RPC_PORT "/lo/in"

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
RelPose* ReadPoseFromBottle(yarp::os::Bottle& lo, jlo::LazyLocatedObjectLoader* loca)
{
    RelPose* result = NULL;
    if(!lo.get(0).isInt() || !lo.get(1).isInt())
    {
      printf("Bottle that should contain a lo is invalid: no id or no parent id\n");
      return result;
    }
    /** Containing a parent id,*/
    int id = lo.get(0).asInt();
    int parent = lo.get(1).asInt();
    /** And two matrices */
    Matrix m(4,4), cov(6,6);
    /* Bottle per matrix*/
    if(lo.get(2).isList())
    {
      yarp::os::Bottle* matrix = lo.get(2).asList();
      int width = 4;
      for(int r = 0; r < width; r++)
      {
        for(int c = 0; c < width; c++)
        {
          yarp::os::Value& val = matrix->get(r * width + c);
          if(val.isDouble())
            m.element(r,c) = val.asDouble();
          else
            m.element(r,c) = 0;
        }
      }
    }
    else return result;
    if(lo.get(3).isList())
    {
      int width = 6;
      yarp::os::Bottle* matrix = lo.get(3).asList();
      for(int r = 0; r < width; r++)
      {
        for(int c = 0; c < width; c++)
        {
          yarp::os::Value& val = matrix->get(r * width + c);
          if(val.isDouble())
            cov.element(r,c) = val.asDouble();
          else
            cov.element(r,c) = 0;
        }
      }
    }
    else
    {
      printf("Bottle that should contain a lo is invalid: one of the matrices missing\n");
      return result;
    }
    return new RelPose(loca, id, parent, m, cov);
}

/*****************************************************************************************
*  class YarpRpcComm                                                                             */
/*****************************************************************************************
*  Class organizing the answer system to a specifie yarp port
******************************************************************************************/
class YarpRpcComm : public Comm, public jlo::LazyLocatedObjectLoader
{
public:

  /*****************************************************************************************
  *  YarpRpcComm                                                                             */
  /*****************************************************************************************
  *  Constructor
  ******************************************************************************************/
  YarpRpcComm(std::string st)
  {
    yarp::os::Network::init();
   /* while(!yarp::os::Network::checkNetwork())*/
    {
/*      printf("Waiting for the yarp server to initialize\n");
#ifdef BOOST_THREAD
			#ifdef BOOST_1_35
  BOOST(boost::system_time t);
#else
  boost::xtime t;
#endif

			#ifdef BOOST_1_35
  BOOST(t = get_system_time());
#else
  boost::xtime_get(&t, boost::TIME_UTC);
#endif

			t.sec += 4;
			boost::thread::sleep(t);
#else
#endif*/
    }
    m_port.open(st.length() == 0 ? STD_LO_RPC_PORT_INTERNAL : st.c_str());
    yarp::os::Network::connect(st.length() == 0 ? STD_LO_RPC_PORT_INTERNAL : st.c_str(), STD_LO_RPC_PORT);
  }


  /*****************************************************************************************
  *  ~YarpRpcComm                                                                             */
  /*****************************************************************************************
  *  Destructor
  ******************************************************************************************/
  ~YarpRpcComm(void)
  {
  }

  /*****************************************************************************************
  *  NotifyPoseUpdate                                                                        */
  /*****************************************************************************************
  *  Call back that is called whenever a new pose is set for a certain model
  *  This callback must be told the signature that is tracked
  ******************************************************************************************/
  virtual void NotifyPoseUpdate(RelPose* pose, bool sendObjectRelation = true)
  {
        yarp::os::Bottle query_bottle;
        yarp::os::Bottle port_answer_bottle;
        query_bottle.clear();
        port_answer_bottle.clear();
        PutPoseIntoABottle(query_bottle, pose);
        sendObjectRelation;
        if(!m_port.write(query_bottle, port_answer_bottle))
        {
          throw "Error in LO service!\n";
        }
  }
  virtual RelPose* CreateNewPose(RelPose* pose, Matrix* mat, Matrix* cov)
  {
        yarp::os::Bottle query_bottle;
        yarp::os::Bottle port_answer_bottle;
        query_bottle.clear();
        port_answer_bottle.clear();
        if(pose != NULL)
          query_bottle.addInt(pose->m_uniqueID);
        else
          query_bottle.addInt(ID_WORLD);
        AddMatrixToABottle(4, query_bottle, *mat);
        AddMatrixToABottle(6, query_bottle, *cov);
        if(!m_port.write(query_bottle, port_answer_bottle))
        {
          throw "Error in LO service!\n";
        }
        if(port_answer_bottle.size() > 0)
        {
          return ReadPoseFromBottle(port_answer_bottle, this);
        }
        else
          return NULL;
  }
  virtual RelPose* GetPose(int poseId)
  {
        yarp::os::Bottle query_bottle;
        yarp::os::Bottle port_answer_bottle;
        query_bottle.clear();
        port_answer_bottle.clear();
        query_bottle.addInt(poseId);
        if(!m_port.write(query_bottle, port_answer_bottle))
        {
          printf("Could not reach LO service\n");
          throw "Error in LO service!\n";
        }
        if(port_answer_bottle.size() > 0)
        {
          return ReadPoseFromBottle(port_answer_bottle, this);
        }
        else
        {
          printf("Empty answer from LO service (Wrong query?)\n");
          throw "Error in LO service!\n";
        }
  };

  virtual jlo::LocatedObject* GetParent(const jlo::LocatedObject& child)
  {
    if(child.m_uniqueID == ID_WORLD)
      return NULL;
    return GetPose(child.m_parentID);
  };

  virtual RelPose* GetPoseRelative(int poseId, int parentPoseId)
  {
        yarp::os::Bottle query_bottle;
        yarp::os::Bottle port_answer_bottle;
        query_bottle.clear();
        port_answer_bottle.clear();
        query_bottle.addInt(poseId);
        query_bottle.addInt(parentPoseId);
        if(!m_port.write(query_bottle, port_answer_bottle))
        {
          throw "Error in LO service!\n";
        }
        if(port_answer_bottle.size() > 0)
        {
          return ReadPoseFromBottle(port_answer_bottle, this);
        }
        else
          return NULL;
  };

  virtual void FreePose(int poseId)
  {
        yarp::os::Bottle query_bottle;
        yarp::os::Bottle port_answer_bottle;
        query_bottle.clear();
        port_answer_bottle.clear();
        query_bottle.addString("del");
        query_bottle.addInt(poseId);
        if(!m_port.write(query_bottle, port_answer_bottle))
        {
          throw "Error in LO service!\n";
        }
  }

private:
  yarp::os::Port m_port;
  YarpRpcComm& operator=(YarpRpcComm&){}
};

#endif /*YARPRPCCOMM_H*/
#endif /*USE_YARP_COMM*/

