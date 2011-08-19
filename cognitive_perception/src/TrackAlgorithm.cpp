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
                        TrackAlgorithm.cpp - Copyright klank


**************************************************************************/

#include "TrackAlgorithm.h"
#include <boost/bind.hpp>
#include <boost/thread.hpp>
extern volatile bool g_stopall;



using namespace cop;


// Constructors/Destructors
//

TrackAlgorithm::TrackAlgorithm (PerceptionPrimitive& sig, Evaluator* eval, Algorithm<std::vector<RelPose*> > *alg,
                               ImageInputSystem* imageSys) :
    m_Running(true),
    m_alg(alg),
    m_curPrim(sig),
    m_imageSys( imageSys),
    m_eval(eval)
{
    m_trackingThread = new boost::thread( boost::bind(&TrackAlgorithm::threadfunc, this));
}


TrackAlgorithm::~TrackAlgorithm ( )
{
  /* Stop Thread:*/
  m_Running = false;
  if(m_trackingThread != NULL)
    m_trackingThread->join();
    delete m_trackingThread;
  m_curPrim.SetTerminated();

    /* Wait for termination and continue destroying*/
}

//
// Methods
//
//
// Methods
//
void TrackAlgorithm::threadfunc()
{
  bool resultReceived = false;
  Signature& object = *m_curPrim.GetSignature();
  RelPose* lastKnownPose = object.GetObjectPose();
  if(lastKnownPose == NULL)
    lastKnownPose = RelPoseFactory::FRelPoseWorld();
  std::vector<Sensor*> sensors = m_imageSys->GetBestSensor(*lastKnownPose);
  while(m_Running && !g_stopall)
  {
    int numOfObjects = 1;
    try
    {

#ifdef _DEBUG
      printf("Algorithm for track: %s (%p)\n",m_alg != NULL ? m_alg->GetName().c_str() : "NoName", m_alg );
#endif
      if(m_alg != NULL)
      {

         double qualityMeasure = 0.0;
         boost::xtime t0, t1;
         boost::xtime_get(&t0, boost::TIME_UTC);
         std::vector<RelPose*> pose = m_alg->Perform(sensors, lastKnownPose, object, numOfObjects, qualityMeasure);
         boost::xtime_get(&t1, boost::TIME_UTC);
         unsigned long time = ((1000000000 * (t1.sec - t0.sec))+(t1.nsec - t0.nsec));
#ifdef _DEBUG
        printf("Calc time: %ld ns\n", time);
#endif
        if(pose.size() > 0)
        {
          pose[0]->m_qualityMeasure = qualityMeasure;
          RelPose*& pose_curr = object.GetObjectPose();
          if(pose_curr != NULL && pose_curr->m_uniqueID != ID_WORLD && resultReceived)
          {
            RelPose* temp = pose[0];
            printf("temp id: %ld", pose[0]->m_uniqueID);

            pose[0] = RelPoseFactory::FRelPose(pose_curr, temp->m_parentID, temp->GetMatrix(0), temp->GetCovarianceMatrix());
            pose[0]->m_qualityMeasure = qualityMeasure;
            //RelPoseFactory::FreeRelPose(temp);
          }
          object.SetPose(pose[0]);
          printf("Sent %ld\n", pose[0]->m_uniqueID);
          if(!resultReceived)
          {
            m_curPrim.AddResult(m_alg, m_eval, object.m_ID, qualityMeasure, time);
            resultReceived = true;
          }
        }
        else
        {
           object.SetPose(NULL);
        }
      }
    }
    catch(char const* message)
    {
      printf("Error in Locate: %s\n", message);
    }
    /** Setting new search space*/
    lastKnownPose = object.GetObjectPose();
    /** TODO select the right one...*/
    sensors[0]->WaitForNewData();
  }
}

XMLTag* TrackAlgorithm::Save()
{
    return NULL;
}

