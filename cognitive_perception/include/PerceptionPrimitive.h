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
                        PerceptionPrimitive.h - Copyright klank


**************************************************************************/

#ifndef PERCEPTIONPRIMITIVE_H
#define PERCEPTIONPRIMITIVE_H


#include "Signature.h"
#include "AlgorithmEval.h"
#include "Sensor.h"

namespace cop
{
  enum PerceptionPrimitiveState
  {
    PP_STARTED,
    PP_TERMINATED,
    PP_EVALUATING,
    PP_EVALUATING_RUNNING,
    PP_EVALUATED,
    PP_DELETABLE
  };

  template< typename T>
  bool find_in_vec(std::vector<T> list, T id)
  {
    for(size_t i = 0; i < list.size(); i++)
    {
      if(list[i] == id)
        return true;
    }
    return false;
  }


  /************************************************************************
  *  class PerceptionPrimitive
  *  @brief Describes the entity created by each call of cop in order
  *         to answer a query and enable later evaluation (short name PP)
  **************************************************************************/
  class PerceptionPrimitive
  {
  public:
    /**
    * @param sig Create a PerceptionPrimitive always with a corresponding Signature
    *
    *  A PerceptionPrimitive (PP) has at least one Signature it validates
    */
    PerceptionPrimitive(Signature* sig) :
      m_evaluation(0),
      m_timing(0),
      m_count(0),
      m_startTime(time(NULL)),
      m_uniqueID(m_lastID++),
      m_currState(PP_STARTED)
    {
      m_signatures.push_back(sig);
    }

    ~PerceptionPrimitive()
    {
      printf("Creating time: %ld\n", m_startTime);
    }
    /**
    *  GetSignature
    *  @return returns the index-th Signature stored in the PP
    */
    Signature* GetSignature(size_t index = 0){return m_signatures[index];}
    PerceptionPrimitiveID_t GetID(){return m_uniqueID;}
    /**
    * AddResult
    * Add resulting descriptors/ignatures to the current list of derived elements.
    * @param algorithm Algorithm that created this result
    * @param id  The Object-id this Evaluation is realted to
    * @param quality Quality assesment between 0.0 and 1.0
    * @param calctime Any time involved for processing these data
    */
    void AddResult(Evaluable* alg, Evaluator* eval, ObjectID_t id, double quality =  1.0, unsigned long calctime = 0)
    {
      m_results.push_back(id);
      bool found = false;
      for(size_t i = 0; i < m_AlgorithmIDs.size(); i++)
      {
        if(m_AlgorithmIDs[i].first == alg)
        {
          found = true;
        }
      }
      if(!found)
        m_AlgorithmIDs.push_back(std::pair<Evaluable*, Evaluator*>(alg, eval));

      m_evaluation += quality;
      m_timing += calctime;
      m_count++;
    }

    void SetTerminated()
    {
      if(m_currState == PP_EVALUATED)
        m_currState = PP_DELETABLE;
      else if (m_currState == PP_TERMINATED)
        printf("State transition from PP_TERMINATED to PP_TERMINATED\n");
      else if(m_currState == PP_EVALUATING_RUNNING)
        m_currState = PP_EVALUATING;
      else
        m_currState = PP_TERMINATED;
    }

    void SetEvaluated()
    {
       if(m_currState == PP_TERMINATED || m_currState == PP_EVALUATING)
        m_currState = PP_DELETABLE;
      else
        m_currState = PP_EVALUATED;
    }
    void SetEvaluating()
    {
      if(m_currState == PP_TERMINATED)
        m_currState = PP_EVALUATING;
      else
        m_currState = PP_EVALUATING_RUNNING;
    }

    PerceptionPrimitiveState GetCurrState(){return m_currState;}
    std::vector<Signature*> m_signatures;
    std::vector< std::pair<Evaluable*, Evaluator*> > m_AlgorithmIDs;
    std::vector<Sensor*> m_sensors;

    std::vector<ObjectID_t> m_results;
    double        m_evaluation;
    unsigned long m_timing;
    unsigned long m_count;
    time_t        m_startTime;
  private:
    PerceptionPrimitiveID_t m_uniqueID;
    static PerceptionPrimitiveID_t m_lastID;
    PerceptionPrimitiveState m_currState;
  };
}
#endif /* PERCEPTIONPRIMITIVE_H*/
