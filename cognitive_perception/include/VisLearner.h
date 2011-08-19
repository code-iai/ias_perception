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
                        VisLearner.h - Copyright klank

**************************************************************************/


#ifndef VISLEARNER_H
#define VISLEARNER_H

#include <string>
#include <vector>
#include "SignatureDB.h"
#include "ImageInputSystem.h"
#include "Statistics.h"
#include "RefineAlgorithm.h"
#include "AlgorithmSelector.h"
#include "PerceptionPrimitive.h"
#ifdef BOOST_THREAD
#include "boost/thread.hpp"
#else
#endif



#define XML_NODE_VISLEARNER "VisLearner"
namespace cop
{
  typedef int TaskID;

  /**
   * class VisLearner
   *  @brief Basic class for improving model knowledge
    */
  class VisLearner
  {
  public:

    // Constructors/Destructors
    //

    /**
    * Constructor
    */
    VisLearner ( XMLTag* tag, SignatureDB& sigDB, ImageInputSystem& imgSys,
  #ifdef LOGFILE
         LogFile& log,
  #endif /*LOGFILE*/
          bool bLearning = true);


    /**
    * Empty Destructor
    */
    virtual ~VisLearner ( );
  private:
    VisLearner();

    // Methods
    //
  public:
    XMLTag* Save();
    /**
    * @brief
    * @return double
    * @param  object
    */
    double RefineObject (Signature& object);


    /**
    * RefineObject
    * @brief tries to gather information of the given Signature
    * @return the signature and similar signatures with their locations
    * @param lastKnownPoses a list of estimated location to start with, can be empty
    * @param visPrim        the object that should be refined
    * @param numOfObjects  maximal number of similar objects to be returned, will be overwritten on return
    */
    SignatureLocations_t RefineObject (PossibleLocations_t* lastKnownPoses, PerceptionPrimitive &visPrim, int &numOfObjects);
    /**
    * ProoveObject
    * @brief tries to validate information of the given Signature
    * @return the signature and similar signatures with their locations
    * @param lastKnownPoses a list of estimated location to start with, can be empty
    * @param visPrim        the object that should be refined
    * @param numOfObjects  maximal number of similar objects to be returned, will be overwritten on return
    */
    SignatureLocations_t ProoveObject(PossibleLocations_t* lastKnownPoses, PerceptionPrimitive &visPrim, int &numOfObjects);

    void AddAlgorithm(Algorithm<Descriptor*>*);
    void AddAlgorithm(Algorithm<ImprovedPose >*);

    /**
     * @return Signature*
     * @param  index
     */
    Signature* GetObjectSignature (int index );


    void threadfunc();

    const AlgorithmSelector<Descriptor* >& GetRefineAlgorithmSelection(){return m_refinements;}
    const AlgorithmSelector<ImprovedPose>& GetProoveAlgorithmSelection(){return m_checks;}


  private:

    // Private attributes
    //
    SignatureDB&		m_signatureDB;
    ImageInputSystem&	m_imageSys;
    Statistics			m_stats;


    std::vector<std::pair<TaskID, Signature*> > m_taskList;
    AlgorithmSelector<Descriptor* >	m_refinements;
    AlgorithmSelector<ImprovedPose>	m_checks;
  #ifdef BOOST_THREAD
    boost::thread* m_learningThread;
  #else
  #endif
    static bool s_Learning;
    static bool s_Running;
  private:
    VisLearner& operator=(VisLearner&){throw "Error";}

  };
}
#endif // VISLEARNER_H
