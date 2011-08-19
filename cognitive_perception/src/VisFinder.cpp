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
                        VisFinder.cpp - Copyright klank


L**************************************************************************/

#include "VisFinder.h"
#include "XMLTag.h"
#include <algorithm>

#ifdef BOOST_THREAD
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include "boost/date_time/posix_time/posix_time.hpp"
#include <iostream>
boost::mutex s_mutexDisplay;
#define BOOST(A) A
#else
#define BOOST (A) ;
#endif

//#ifdef _DEBUG
#define  DEBUG(A) A
//#else
//#define  DEBUG(A) ;
//#endif

using namespace cop;


// Constructors/Destructors
//

VisFinder::VisFinder( XMLTag* configFile, ImageInputSystem* imageSystem,
SignatureDB* db, AttentionManager* manager, VisLearner*	visLearner
#ifdef LOGFILE
                      ,LogFile& log) :
    m_selLocate(configFile != NULL ? configFile->GetChild(0) : NULL, log),
#else /*LOGFILE*/
     ) :
    m_selLocate(configFile != NULL ? configFile->GetChild(0) : NULL),
#endif /*LOGFILE*/
    m_imageSys(imageSystem),
    m_sigdb(db),
    m_visLearner(visLearner),
    m_attentionMan(manager)
{
    m_selLocate.SetName(XML_NODE_VISFINDER);
}

VisFinder::~VisFinder ( )
{
    for(std::map<int, TrackAlgorithm*>::iterator it = m_runningTracks.begin();
    it != m_runningTracks.end(); it++)
    {
        delete (*it).second;
    }
}




//
// Methods
//
/**
*	Saves the status of the Visual Finder
*/
XMLTag* VisFinder::Save()
{
    XMLTag* tag = new XMLTag(XML_NODE_VISFINDER);
    tag->AddChild(m_selLocate.Save());
    return tag;
}


bool VisFinder::GetPlaneClusterCall(PossibleLocations_t *cluster_ids, RelPose* pose, PerceptionPrimitive& visPrim, const std::vector<Sensor*> &sensors)
{
  Signature &sig = *visPrim.GetSignature();
  LocateAlgorithm* det = (LocateAlgorithm*)m_selLocate.BestAlgorithm(123, sig, sensors);
  if(det == NULL)
    return false;
  int num;
  double qual;
  std::vector<RelPose*>  results =  det->Perform(sensors, pose, sig, num, qual);
  for(size_t i = 0; i < results.size(); i++)
  {
    cluster_ids->push_back(std::pair<RelPose*, double>(results[i], qual));
  }
  return true;
}

void ShowSearchSpace(PossibleLocations_t* lastKnownPoses, std::vector<Sensor*> cam)
{
  if(cam[0] != NULL)
  {
    /*TODO*/
    /*cam->Show();*/
    printf("Showing searchspaces\n");
    for(size_t i = 0; i < lastKnownPoses->size(); i++)
    {
      /*TODO*/
    }
  }
}



bool comp_qual (const Results_t &t1, const Results_t &t2)
{
  return t1.quality > t2.quality;
}

/**
     * @return RelPose
     * @param  LastKnownPose
     * @param  Object
     * @param  numOfObjects
     */
SignatureLocations_t VisFinder::Locate (PossibleLocations_t* lastKnownPoses, PerceptionPrimitive& visPrim, int &numOfObjects)
{
        std::vector<Sensor*> cameras;
        Signature& object = *visPrim.GetSignature();
        double qualityMeasure = 0.0;
        Algorithm<std::vector<RelPose*> >* alg_fail = NULL;
        SignatureLocations_t ret;
        PossibleLocations_t::const_iterator it = lastKnownPoses->begin();
        std::vector<Results_t> all_matches;
        int maxObjToAdd = numOfObjects;
        BOOST(boost::system_time t0);
        BOOST(boost::system_time t1);
        //int count_poses = 1;
        if(lastKnownPoses->size() == 0)
        {
          std::vector<Sensor*> allsensors = m_imageSys->GetAllSensors();
          if(allsensors.size() > 0 && allsensors[0] != NULL)
          {
            if(!GetPlaneClusterCall(lastKnownPoses, allsensors[0]->GetRelPose(), visPrim, allsensors))
            {
              numOfObjects = 0;
              printf("No search position specified: 0 Results, no search\n");
            }
            printf("Got a new Searchspace\n");
            printf("lkp size %ld\n", lastKnownPoses->size());
            if(lastKnownPoses->size() > 0)
              printf("lastKnownPoses[0]: id %ld\n", (*lastKnownPoses)[0].first->m_uniqueID);
          }
        }
        it = lastKnownPoses->begin();
        Algorithm<std::vector<RelPose*> > * alg = NULL;
        bool anysensor = false;
        for(;it != lastKnownPoses->end(); it++)
        {
            RelPose* lastKnownPose = (*it).first;
            if(lastKnownPoses == NULL)
            {
              printf("VisFinder: Errorn in pose information\n");
              continue;
            }
            try
            {
                cameras = m_imageSys->GetBestSensor(*lastKnownPose);
                if(cameras.size() == 0)
                {
                  printf("No Sensor for this location available\n");
                  continue;
                }
                else
                {
                 anysensor = true;
                }
                int locatertype = ALGORITHMTYPE_LOCATE;
                if(object.GetObjectPose() == NULL)
                  object.SetPose(&lastKnownPose[0]);

                /*if(count_poses == 1)
                {
                  ShowSearchSpace(lastKnownPoses, cameras);
                  count_poses++;
                }*/


                locatertype += numOfObjects > 1 ? ALGORITHMSPEC_SEVERALTARGET : ALGORITHMSPEC_ONETARGET ;
                 alg = m_selLocate.BestAlgorithm(locatertype, object, cameras);
                DEBUG(printf("Selected Algorithm: %s\n", alg != NULL ? alg->GetName().c_str() : "None" ));

                if(alg != NULL)
                {
                  /* TODO remove next line?*/
                  m_selLocate.m_logfile.ReportEvent(visPrim.GetID(), alg->GetName());


                  int numOfObjects_tmp = numOfObjects;
                  BOOST(t0 = boost::get_system_time());
                  std::vector<RelPose*> r = alg->Perform(cameras, lastKnownPose, object, numOfObjects_tmp, qualityMeasure);
                  BOOST(t1 = boost::get_system_time());

                  /** Collect results and  measure time*/
                  numOfObjects_tmp = r.size() < (unsigned)numOfObjects_tmp ?  r.size() : (unsigned)numOfObjects_tmp;
                  printf("Results num = %d\n", numOfObjects_tmp);
                  alg_fail = alg;
                  BOOST(boost::posix_time::time_duration td = t1 - t0);
                  BOOST(printf("Calc time: %s\n", boost::posix_time::to_simple_string(td).c_str()));

                  /** Make results sortable*/
                  for(std::vector<RelPose*>::const_iterator it_poses = r.begin(); it_poses != r.end(); it_poses++)
                  {
                    Results_t res_tmp;
                    res_tmp.pose = *it_poses;
                    res_tmp.quality = it_poses == r.begin() ? qualityMeasure : (*it_poses)->m_qualityMeasure;
                    res_tmp.camera = cameras[0];/*TODO fix! BestAlgorithm?!?*/
                    res_tmp.alg = alg;
                    m_selLocate.ReevaluatePose(alg, res_tmp.pose);
                    all_matches.push_back(res_tmp);
                  }
                }
                else
                {
                  printf("No algorithm found for the quieried Problem\n");
                }
            }
            catch(char const* message)
            {
                printf("Error in Locate: %s\n", message);

            }
        }
        if(!anysensor)
        {
          throw "No Sensor could see any of the requested locations";
        }
        /**  Sort results*/
        std::sort(all_matches.begin(), all_matches.end(), comp_qual);
        if(all_matches.size() > 0 && numOfObjects > 0)
        {
          for(unsigned int i = 0; i < all_matches.size(); i++)
          {
            if(i == 0)
            {
              numOfObjects = 0;
            }
            RelPose* pose = all_matches[i].pose;
            if(numOfObjects >= maxObjToAdd)
            {
              RelPoseFactory::FreeRelPose(&pose, true);
              continue;
            }
            Signature* sig = (Signature*)(object.Duplicate(false));
            sig->SetLastPerceptionPrimitive(visPrim.GetID());
            visPrim.AddResult(alg, &m_selLocate, sig->m_ID, pose->m_qualityMeasure, ((double)((t1 - t0).total_milliseconds()) /  1000.0));
            sig->SetPose(pose);
            /*m_selLocate.EvalAlgorithm(all_matches[i].alg, pose->m_qualityMeasure, ((double)((t1 - t0).total_milliseconds()) /  1000.0), sig);
              */

            /*pose->m_qualityMeasure = qualityMeasure;*/
            ret.push_back(std::pair<RelPose*, Signature*>(pose, sig));
#ifdef BOOST_THREAD
             all_matches[i].camera = cameras[0];
             all_matches[i].signature = sig;
#else
            try
            {
              if(all_matches[i].camera->IsCamera())
                sig->Show(all_matches[i].camera);
            }
            catch(char* error)
            {
              printf("Error in Display: %s\n", error);
            }
            m_sigdb->AddSignature(sig);
#endif
            numOfObjects++;
            /*if(qualityMeasure > 0.70)
               m_visLearner.RefineObject(sig);*/
          }
          m_sigdb->AddAndShowSignaturesAsync(all_matches, numOfObjects);

        }
        else
        {
           //m_selLocate.EvalAlgorithm(alg_fail, qualityMeasure,((double)((t1 - t0).total_milliseconds()) /  1000.0), &object);
#ifdef BOOST_THREAD
           m_sigdb->AddAndShowSignatureAsync(&object, NULL);
#else
           m_sigdb->AddSignature(&object);
#endif
        }
   return ret;
}


/**
 * @param Object
 * @param poseEstimation
 */
void VisFinder::StartTrack  ( PerceptionPrimitive& visPrim, RelPose* poseEstimation)
{
    try
    {
      Signature& object = *visPrim.GetSignature();
      std::vector<Sensor*> cameras = m_imageSys->GetBestSensor(*poseEstimation);
      int locatertype = ALGORITHMTYPE_TRACK;
      locatertype = ALGORITHMSPEC_SEVERALTARGET; /** TODO: Check how to include Locating and special Tracking algorithms*/
      Algorithm<std::vector<RelPose*> > * alg = m_selLocate.BestAlgorithm(locatertype, object, cameras);
      if(alg != NULL)
      {
        if(m_runningTracks.find(object.m_ID) == m_runningTracks.end())
        {
          if(poseEstimation != NULL)
            object.SetPose(poseEstimation);
          TrackAlgorithm*  tracking = new TrackAlgorithm(visPrim, &m_selLocate, alg, m_imageSys);
          m_runningTracks[object.m_ID] = tracking;
        }
        else
        {
          ROS_ERROR("There was already a query for the same object, ignoring the new (FIX necessary if this feature is required)\n"); /*TODO fix this*/
          visPrim.SetTerminated();
        }
      }
      else
      {
        printf("No Algorithm for Tracking of object %ld found\n", object.m_ID);
      }
    }
    catch(...)
    {
    }
    return;
}

/**
 * @param  Object
 */
void VisFinder::StopTrack (Signature& object)
{
    std::map<int, TrackAlgorithm*>::iterator it = m_runningTracks.find(object.m_ID);
    if(it != m_runningTracks.end())
    {
        delete (*it).second;
        m_runningTracks.erase(it);
    }

}

/**
* @return RelPose
* @param  CurImage
* @param  Pose
* @param  Obj1
* @param  Obj2
*/
RelPose* VisFinder::RelTwoObjects (const RelPose &Pose, Signature& Obj1, Signature& Obj2 )
{
     /* TODO reason about best match, not yet integrated*/
    return NULL;
}

/**
*	AddAlgorithm
*	@param alg
*/
void VisFinder::AddAlgorithm(Algorithm<std::vector<RelPose*> >* alg)
{
    m_selLocate.AddAlgorithm(alg, ALGORITHMSPEC_ONETARGET, 1.0, 0.0);
}

#ifndef WIN32
#include "AlgorithmSelector.hpp"
template class AlgorithmSelector<std::vector<RelPose*> >;;
#else
#include "AlgorithmSelector.hpp"
template AlgorithmSelector<std::vector<RelPose*> >;
#endif

