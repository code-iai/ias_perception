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
                        VisLearner.cpp - Copyright klank

**************************************************************************/

#include "VisLearner.h"
#include "boost/date_time/posix_time/posix_time.hpp"


extern volatile bool g_stopall;

#ifdef BOOST_THREAD
#include "boost/bind.hpp"
#define BOOST(A) A
#else
#define BOOST(A)
#endif


#define XML_NODE_REFINEMENTALGORITHMS "RefineAlgs"
#define XML_NODE_PROVINGALGORITHMS "ProveAlgs"

using namespace cop;


bool VisLearner::s_Learning = false;
bool VisLearner::s_Running = true;
// Constructors/Destructors
//

VisLearner::VisLearner ( XMLTag* tag, SignatureDB& sigDB, ImageInputSystem& imgSys
#ifdef LOGFILE
            ,LogFile& log
#endif /*LOGFILE*/
            , bool bLearning) :
		m_signatureDB(sigDB),
		m_imageSys(imgSys),
		m_stats(tag != NULL ? tag->GetChild(XML_NODE_STATISTICS) : NULL)
#ifdef LOGFILE
		,m_refinements(tag != NULL ? tag->GetChild(XML_NODE_REFINEMENTALGORITHMS) : NULL, log)
		,m_checks(tag != NULL ? tag->GetChild(XML_NODE_PROVINGALGORITHMS): NULL, log)
#else  /*LOGFILE*/
    ,m_refinements(tag != NULL ? tag->GetChild(XML_NODE_REFINEMENTALGORITHMS) : NULL)
		,m_checks(tag != NULL ? tag->GetChild(XML_NODE_PROVINGALGORITHMS): NULL)
#endif /*LOGFILE*/
{
#ifdef BOOST_THREAD
  bLearning = false;
	if(bLearning)
		m_learningThread = new boost::thread( boost::bind(&VisLearner::threadfunc, this) ) ;
	else
		m_learningThread = NULL;
#else
#endif
       m_refinements.SetName(XML_NODE_VISLEARNER);
       m_checks.SetName(XML_NODE_VISLEARNER);
}

VisLearner::~VisLearner ( )
{
	s_Running = false;
#ifdef BOOST_THREAD
	if(m_learningThread != NULL)
		m_learningThread->join();
	delete m_learningThread;
#else
#endif
}

//
// Methods
//
 void VisLearner::threadfunc()
{
  std::vector<Sensor*> cam = m_imageSys.GetAllSensors();
  while(VisLearner::s_Running && !g_stopall)
  {
    size_t n = m_taskList.size();
    if(n > 0)
    {
      s_Learning = true; TaskID type = m_taskList[0].first;
      Signature* sig = m_taskList[0].second;
      int numOfObjects = 1;
      double qualityMeasure = 0.0;
#ifdef _DEBUG
      printf("New Learning Task\n");
#endif /*_DEBUG*/
      RefineAlgorithm* refalg = (RefineAlgorithm*)m_refinements.BestAlgorithm(type, *sig, cam);
      if(refalg != NULL)
      {
          printf("Alg selected: %s\n", refalg->GetName().c_str());
          try
          {
            Descriptor* d = refalg->Perform(cam, sig->GetObjectPose(), *sig, numOfObjects, qualityMeasure);
	    if(d == NULL)
	      break;
            sig->SetElem(d);
          }
          catch(char const* error_text)
          {
            printf("Refinement failed due to: %s\n", error_text);
          }
      }
      else
      {
         ProveAlgorithm* provalg = (ProveAlgorithm*)m_checks.BestAlgorithm(type, *sig, cam);
	 if(provalg != NULL)
	 {
           printf("Alg selected: %s\n", refalg->GetName().c_str());
           try
           {
	     ImprovedPose res = provalg->Perform(cam, sig->GetObjectPose(), *sig, numOfObjects, qualityMeasure);
             if(res.first != NULL)
	     {
                res.first->m_qualityMeasure = res.second;
	     }
	     sig->SetPose(res.first);
#ifdef _DEBUG
	     printf("Evaluation results in: %f\n", res.second);
#endif /*_DEBUG*/
	     if(res.second > 0.5)
	     {
#ifdef _DEBUG
	       printf("This is good!\n");
#endif /*_DEBUG*/
	     }
           }
           catch(char const* error_text)
           {
#ifdef _DEBUG
             printf("Evaluation failed due to: %s\n", error_text);
#endif /*_DEBUG*/
           }
	}
     }
     m_taskList.erase(m_taskList.begin());
     s_Learning = false;
   }
   else
   {
     //printf("Learning Thread Sleeps \n\n\n");
      BOOST(boost::system_time t);
      BOOST(t = get_system_time());
      t += boost::posix_time::seconds(1);
      boost::thread::sleep(t);
      return;
    }
  }
}

void VisLearner::AddAlgorithm(Algorithm<Descriptor*>* alg)
{
	m_refinements.AddAlgorithm(alg, ELEM, 1.0, 0.0);
}
void VisLearner::AddAlgorithm(Algorithm<ImprovedPose>* alg)
{
	m_checks.AddAlgorithm(alg, ELEM, 1.0, 0.0);
}

/**
 * @return SignatureLocations_t
 * @param  lastKnownPoses
 * @param  sig
 * @param  numOfObjects
 */
SignatureLocations_t VisLearner::RefineObject (PossibleLocations_t* lastKnownPoses, PerceptionPrimitive &visPrim, int &numOfObjects)
{
  SignatureLocations_t ret_vec;
  Signature& sig = *visPrim.GetSignature();
  TaskID type = 0x100;
  PossibleLocations_t::const_iterator it = (*lastKnownPoses).begin();
  std::vector<Sensor*> sensors;
  for(;it!=(*lastKnownPoses).end(); it++)
  {
    RelPose* lastKnownPose = (*it).first;
    try
    {
      sensors = m_imageSys.GetBestSensor(*lastKnownPose);
    }
    catch(const char* text)
    {
      printf ("Error selecting a camera: %s\n", text);
      continue;
    }

    std::vector<Algorithm<Descriptor*>*> refalg_list = m_refinements.BestAlgorithmList(type, sig, sensors);
    printf("The list of algorithms for refinement contains %ld algs\n", refalg_list.size());
    if(refalg_list.size() > 0)
    {
      /** We found at least one alg, */
      bool some_success = false;
      std::vector<Algorithm<Descriptor*>*>::const_iterator iter = refalg_list.begin();
      for(;iter != refalg_list.end(); iter++)
      {
        RefineAlgorithm* refalg = (RefineAlgorithm*)(*iter);
        try
        {
          double qualityMeasure;
          boost::system_time t0;
          boost::system_time t1;
          t0 = boost::get_system_time();
          Descriptor* d = refalg->Perform(sensors, lastKnownPose, sig, numOfObjects, qualityMeasure);
          t1 = boost::get_system_time();
          boost::posix_time::time_duration td = t1 - t0;
          printf("Calc time: %s\n", boost::posix_time::to_simple_string(td).c_str() );
          /*sensors[0]->Show(-1);*/
          if(d != NULL)
          {
            d->SetLastPerceptionPrimitive(visPrim.GetID());
            d->Evaluate(qualityMeasure, 1.0);

            sig.SetElem(d);
            RelPose* pose = d->GetLastMatchedPose();
            if(pose != NULL)
              lastKnownPose = pose;

            /*d->Show(lastKnownPose, sensors[0] );*/
            lastKnownPose->m_qualityMeasure = qualityMeasure;
            visPrim.AddResult(refalg, &m_refinements, d->m_ID, lastKnownPose->m_qualityMeasure,((double)((t1 - t0).total_milliseconds()) /  1000.0));
            /*m_refinements.EvalAlgorithm(refalg, lastKnownPose->m_qualityMeasure, ((double)((t1 - t0).total_milliseconds()) /  1000.0), &sig);*/
            some_success = true;
          }
          else
          {
            /*No result for this object, remeber*/
            /*m_refinements.EvalAlgorithm(refalg, 0.0, ((double)((t1 - t0).total_milliseconds()) /  1000.0), &sig);*/
          }
        }
        catch(char const* error_text)
        {
          /*Crash for this object, remeber*/
          m_refinements.EvalAlgorithm(refalg, 0.0, 1.0, &sig);
          printf("Refinement failed due to: %s\n", error_text);
        }
      }
      if(some_success)
      {
        m_signatureDB.AddSignature(&sig);
        ret_vec.push_back(std::pair<RelPose*, Signature*>(lastKnownPose, &sig));
      }
    }
  }
  return ret_vec;
}


SignatureLocations_t VisLearner::ProoveObject (PossibleLocations_t* lastKnownPoses, PerceptionPrimitive &visPrim, int &numOfObjects)
{
  SignatureLocations_t ret_vec;
  Signature& sig = *visPrim.GetSignature();
  TaskID type = 0x100;
  PossibleLocations_t::const_iterator it = (*lastKnownPoses).begin();
  std::vector<Sensor*> sensors;
  for(;it!=(*lastKnownPoses).end(); it++)
  {
    RelPose* lastKnownPose = (*it).first;
    try
    {
      sensors = m_imageSys.GetBestSensor(*lastKnownPose);
    }
    catch(const char* text)
    {
      printf ("Error selecting a camera: %s\n", text);
      continue;
    }
    ProveAlgorithm* provalg = (ProveAlgorithm*)m_checks.BestAlgorithm(type, sig, sensors);
    if(provalg != NULL)
    {
      printf("Alg selected: %s\n", provalg->GetName().c_str());
      try
      {
        double qualityMeasure;
       ImprovedPose ipose = provalg->Perform(sensors, lastKnownPose, sig, numOfObjects, qualityMeasure);
        /** TODO, set evaluation*/
        #ifdef _DEBUG
        printf("Evaluation results in: %f\n", ipose.second);
        #endif /*_DEBUG*/
        if(ipose.second > 0.5)
        {
        #ifdef _DEBUG
          printf("This is good!\n");
        #endif /*_DEBUG*/
        }
        ret_vec.push_back(std::pair<RelPose*, Signature*>(lastKnownPose, &sig));
      }
      catch(char const* error_text)
      {
#ifdef _DEBUG
         printf("Evaluation failed due to: %s\n", error_text);
#endif /*_DEBUG*/
      }
    }
  }
  return ret_vec;
}

/**
 * @return double
 * @param  sig
 */
double VisLearner::RefineObject (Signature& sig)
{
	if(s_Learning == true)
		return 0.0;
	TaskID id = ELEM;

	m_taskList.push_back(std::pair<TaskID, Signature*>
			(id, &sig));

#ifndef BOOST_THREAD
	threadfunc();
#endif
	return m_taskList.size();
}



XMLTag* VisLearner::Save()
{
	XMLTag* tag = new XMLTag(XML_NODE_VISLEARNER);
	tag->AddChild(m_stats.Save());
	tag->AddChild(m_refinements.Save(XML_NODE_REFINEMENTALGORITHMS));
	tag->AddChild(m_checks.Save(XML_NODE_PROVINGALGORITHMS));
	return tag;
}

/**
 * @return Signature
 * @param  index
 */
Signature* VisLearner::GetObjectSignature (int index )
{
	return NULL;
}

#ifndef WIN32
#include "AlgorithmSelector.hpp"
template class AlgorithmSelector<Descriptor* >;
template class AlgorithmSelector<double >;
#else
#include "AlgorithmSelector.hpp"
template AlgorithmSelector<Descriptor* >;
template AlgorithmSelector<double >;
#endif

