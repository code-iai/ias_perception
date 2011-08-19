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


#ifndef ALGORITHMEVAL_H
#define ALGORITHMEVAL_H

#include "Algorithm.h"
#include "LocateAlgorithm.h"
#include "RefineAlgorithm.h"
#include "ProveAlgorithm.h"
#include "AttentionAlgorithm.h"
#include "Signature.h"

#define XML_NODE_ALGORITHMEVAL "AlgorithmEval"
#define XML_NODE_EVAL "Eval"
#define XML_NODE_ALGTYPE "AlgType"
#define XML_NODE_AVGTIME "AvgTime"

#define XML_NODE_OBJECTEVALMAP "ObjEvalMap"


#define ALGORITHMTYPE_LOCATE            0x000
#define ALGORITHMTYPE_TRACK             0x010
#define ALGORITHMTYPE_2OBJS             0x200

#define ALGORITHMTYPE_REFINE            0x100
#define ALGORITHMTYPE_RPOVE             0x400

#define ALGORITHMTYPE_STOPTRACK         0x800


#define ALGORITHMTYPE_STARTATTEND       0x1000
#define ALGORITHMTYPE_STOPATTEND        0x2000


#define ALGORITHMTYPE_LOOKUP            0x6400
#define ALGORITHMTYPE_LOOKUPDB          0x6401
#define ALGORITHMTYPE_LOOKUPALL         0x6402

#define ALGORITHMSPEC_ONETARGET         0
#define ALGORITHMSPEC_SEVERALTARGET     1

namespace cop
{
  /**
  * template<typename T> class AlgorithmEval
  * @brief Template for saving the evaluation of an algorithm for the algorithm selector, the type refers to the used algorithm
  */
  template<typename T> class AlgorithmEval
  {
  public:
    AlgorithmEval(Algorithm<T>* alg, int type, double eval, double m_avgRunTime) :
      m_algorithm(alg),
      m_algorithmType(type),
      m_eval(eval),
      m_avgRunTime(m_avgRunTime)
    {
      if(alg == NULL)
        printf("Empty Algorithm\n");
    }


    /******************************************************************************
    * Constructor Algorithm Eval                                                 */
    /******************************************************************************
    * @param tag Contains data to load the Evaluation Algorithm.
    * Evaluates a tag of the following structure:
    * Parent XML_NODE_ALGORITHMEVAL:
    *          Child index0: Algorithm<T>      containt any Algorithm  that is loadable by the plugin to load a specific algorithm type
    *          XML_NODE_ALGTYPE                one of ALGORITHMTYPE_LOCATE, ALGORITHMTYPE_REFINE and ALGORITHMTYPE_RPOVE (+ALGORITHMSPEC_SEVERALTARGET|ALGORITHMTYPE_TRACK)
    *          XML_NODE_EVAL                   double value representing the overall Method score
    *          XML_NODE_AVGTIME                double value representing the average running time of the Method
    *          XML_NODE_OBJECTEVALMAP[optional] map of int -> double, ulong pair representing specific object<->method relation
    *
    * @throw char* with an error message in case of failure
    *******************************************************************************/
    AlgorithmEval(XMLTag* tag);

    std::pair<double, int> GetEval(ObjectID_t id) const 
    {
      std::map<ObjectID_t, std::pair<double, int> >::const_iterator it;
      if((it = m_objectEval.find(id)) != m_objectEval.end())
      {
        return (*it).second;
      }
      else
      {
         std::pair<double, int> p;
         p.first = 0.0;
         p.second = 0;
         return p;
      }
    }

    void SetEval(ObjectID_t id, double eval)
    {
      if(m_objectEval.find(id) != m_objectEval.end())
      {
        m_objectEval[id].first += eval;
        m_objectEval[id].second++;
      }
      else
      {
        m_objectEval[id].first = eval;
        m_objectEval[id].second = 1;
      }
    }
    
    size_t MapLength() const{return m_objectEval.size();}
    const std::map<ObjectID_t, std::pair<double, int> > GetMap() const {return m_objectEval;}

    /******************************************************************************
    * Save                                                                       */
    /******************************************************************************
    *  @brief Saves all elemens to a serializeable XMLTag
    *  @param name [optional]
    *  @remarks return value must be deleted
    *******************************************************************************/
    XMLTag* Save(std::string name = "");

    Algorithm<T>*	                           m_algorithm;
    int				                   m_algorithmType;
    double			                   m_eval;
    double			                   m_avgRunTime;
  private:
    std::map<ObjectID_t, std::pair<double, int> >  m_objectEval;

  };

  class Evaluator
  {
  public:
    /**
    *	Sets the evaluation for an algorithm
    */
    virtual void EvalAlgorithm(Evaluable* alg, double eval, double time, Signature* relatedElemg) = 0;
    /**
    *   SetCaller information for logginf info
    */
    std::string GetName(){return caller_name;}
    void SetName(std::string name){caller_name = name;}


    std::string caller_name;
  };

}
#endif /*ALGORITHMEVAL_H*/
