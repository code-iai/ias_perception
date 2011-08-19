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
                        AttentionManager.cpp - Copyright klank


**************************************************************************/

#include "AttentionManager.h"
#include "AttentionAlgorithm.h"
#include "ImageInputSystem.h"
#include "SignatureDB.h"
#include "XMLTag.h"
#include "BoostUtils.h"
#ifdef BOOST_THREAD
#include "boost/bind.hpp"
#define BOOST(A) A
#else
#define BOOST(A)
#endif

#define XML_PROPERTY_ATTENDING "IsAttending"
#define XML_NODE_ATTENTIONALGORITHMS "Attendants"
#define XML_NODE_ATTENTEDOBJ "AttentedObj"


using namespace cop;
extern volatile bool g_stopall;
// Constructors/Destructors
//

AttentionManager::AttentionManager ( XMLTag* config , SignatureDB& sig_db, ImageInputSystem& imginsys
#ifdef LOGFILE
                                    , LogFile& log
#endif /*LOGFILE*/
                                    ) :
	m_imginsys(imginsys),
#ifdef LOGFILE
    m_attendants(config != NULL ? config->GetChild(XML_NODE_ATTENTIONALGORITHMS) : NULL, log),
    m_logFile(log),
#else  /*LOGFILE*/
    m_attendants(config != NULL ? config->GetChild(XML_NODE_ATTENTIONALGORITHMS) : NULL),
#endif /*LOGFILE*/
    m_sigDB(sig_db)
{
  m_Attending = true;
  if(config != NULL)
  {
    XMLTag* objs = config->GetChild(XML_NODE_ATTENTEDOBJ);
    if(objs != NULL)
    {
      for(unsigned int i = 0; i < objs->CountChildren(); i++)
      {
        try
        {
          XMLTag* tag2 = objs->GetChild(i);
          if(tag2->GetName().compare(XML_NODE_SIGNATURE) == 0)
          {
            Signature* sig = (Signature*)Elem::ElemFactory(tag2);
            PerceptionPrimitive* p = new PerceptionPrimitive(sig);
            SetObjectToAttend(p, NULL, NULL);
          }
          else
          {
            ROS_WARN("Trying to instantiate a broken XMLNode as Signature: %s\n", tag2->GetName().c_str());
          }
        }
        catch(const char* text)
        {
          ROS_ERROR("Error loading object to attend in AttentionManager: %s\n", text);
        }

      }
    }
  }


  m_learningThread = new boost::thread( boost::bind(&AttentionManager::threadfunc, this) ) ;

  m_attendants.SetName(XML_NODE_ATTENTIONMANAGER);
}

AttentionManager::~AttentionManager ( )
{
	m_Attending = false;

	if(m_learningThread != NULL)
		m_learningThread->join();
	delete m_learningThread;

  for(std::vector<AttendedObjects>::iterator it = m_attendedObjectPrototypes.begin();
    it != m_attendedObjectPrototypes.end(); )
  {
    it = m_attendedObjectPrototypes.erase(it);
  }
}

//
// Methods
//


void AttentionManager::SetObjectToAttend (PerceptionPrimitive* prototype, PossibleLocations_t* pointOfInterest, Comm* comm)
{
  AttendedObjects obj(prototype, pointOfInterest, comm);
  ROS_DEBUG("Set an object to Attent\n");
  m_attendedObjectPrototypes.push_back(obj);
}

void AttentionManager::StopAttend(Comm* comm)
{
  unsigned long actionToStop = comm->GetCommID();
  for(std::vector<AttendedObjects>::iterator it = m_attendedObjectPrototypes.begin();
    it != m_attendedObjectPrototypes.end(); it++)
  {
    if((*it).comm != NULL && (*it).comm->GetCommID() == actionToStop)
    {
      m_attendedObjectPrototypes.erase(it);
      break;
    }
  }
}

void AttentionManager::PerformAttentionAlg( std::vector<Sensor*> &sensors, RelPose* pose, Signature* sig, AttendedObjects& objProto)
{
  AttentionAlgorithm* refalg = (AttentionAlgorithm*)m_attendants.BestAlgorithm(4096, *sig, sensors);
  int numOfObjects = 0;
  double qualityMeasure = 0.0;
  if(refalg != NULL)
  {
    printf("Alg selected: %s\n", refalg->GetName().c_str());
    try
    {


      std::vector<Signature*> results = refalg->Perform(sensors, pose, *sig, numOfObjects, qualityMeasure);
      for(size_t j = 0; j < results.size(); j++)
      {
        if(objProto.comm != NULL && results[j]->GetObjectPose() != NULL)
        {
           objProto.comm->NotifyNewObject(results[j], results[j]->GetObjectPose());
        }
        printf("Adding new object: Signature with ID %ld\n", results[j]->m_ID);
        m_sigDB.AddSignature(results[j]);
      }
    }
    catch(char const* error_text)
    {
      ROS_WARN("Error in Attention System: %s\n", error_text);

    }
  }
}

void AttentionManager::threadfunc()
{
  sleep(3);
  while(!g_stopall)
  {
    for(size_t i = 0 ; i < m_attendedObjectPrototypes .size(); i++)
    {
      Signature* sig = m_attendedObjectPrototypes[i].proto->GetSignature();
      PossibleLocations_t* area = m_attendedObjectPrototypes[i].poses;
      std::vector<Sensor*> sensors;

      if(area != NULL  && area->size() != 0)
      {
        PossibleLocations_t::const_iterator iter = area->begin();
        for( ;iter != area->end(); iter++)
        {
          RelPose* pose = (*iter).first;
          sensors = m_imginsys.GetBestSensor(*pose);
          PerformAttentionAlg(sensors, pose, sig, m_attendedObjectPrototypes[i]);

        }
      }
      else
      {
        sensors = m_imginsys.GetAllSensors();
        PerformAttentionAlg(sensors, NULL, sig, m_attendedObjectPrototypes[i]);
      }
    }
    Sleeping(10);
  }
}



XMLTag* AttentionManager::Save()
{
	XMLTag* tag = new XMLTag(XML_NODE_ATTENTIONMANAGER);
  tag->AddChild(m_attendants.Save(XML_NODE_ATTENTIONALGORITHMS));
  XMLTag* objs = new XMLTag(XML_NODE_ATTENTEDOBJ);

  for(size_t obj = 0; obj< m_attendedObjectPrototypes.size(); obj++)
  {
    XMLTag* sig = m_attendedObjectPrototypes[obj].proto->GetSignature(0)->Save();
    objs->AddChild(sig);
  }
  tag->AddChild(objs);
	return tag;
}


#ifndef WIN32
#include "AlgorithmSelector.hpp"
template class AlgorithmSelector<std::vector<Signature*> >;
#else
#include "AlgorithmSelector.hpp"
template AlgorithmSelector<std::vector<Signature*> >;
#endif



