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
                        AlgorithmSelector.h - Copyright klank


**************************************************************************/


#ifndef ALGORITHMSELECTOR_H
#define ALGORITHMSELECTOR_H

#include <string>
#include <vector>

#include "AlgorithmEval.h"
#include "Signature.h"
#ifdef LOGFILE
#include "LogFile.h"
#define STD_LOGFILENAME "AlgActions.log"
#endif /*LOGFILE*/

/** ANN search for comparing with prior detection with algorithms.*/
#include <ANN/ANN.h>

/**
*	typedefs:
*/
typedef double Probability_1D_t; /* < Type for storing Probability of a location to occur*/


namespace cop
{

  typedef std::vector<std::pair<RelPose*, Probability_1D_t> > PossibleLocations_t;/* < Type for a list of relposes with their a-priori probability */
  typedef std::vector<std::pair<RelPose*, Signature*> > SignatureLocations_t;

  /***/

  #define XML_NODE_ALGORITHMSELECTOR "AlgorithmSelector"
  /**
    * class AlgorithmSelector
    *	@brief Provides an interface for selecting an optmial algorithm
    */
  template<typename T>
  class AlgorithmSelector : public Evaluator
  {
  public:
    // Constructors/Destructors
    //
    /**
     * Constructor
     */
      AlgorithmSelector(
  #ifdef LOGFILE
          LogFile& log
  #endif /*LOGFILE*/
          )  :
          m_tree(NULL),
          m_paLength(0),
          m_paAllocatedLength(0),
          m_paDim(4)
#ifdef LOGFILE
          ,m_logfile(log)
  #endif /*LOGFILE*/
          {
            m_mutexAlgEval = new boost::mutex();
          }
    AlgorithmSelector (XMLTag* node
  #ifdef LOGFILE
          , LogFile& log
  #endif /*LOGFILE*/
          );
    /**
     * Empty Destructor
     */
    ~AlgorithmSelector ( ){delete m_mutexAlgEval;}
    /**
    *	AddAlgorithm
    *	@brief
    *	@param alg	A pointer to a algorithm class
    *	@param nType	the algorithm type (e.g. )
    *	@param dEval
    *	@return the index in the algorithm list
    */
    int AddAlgorithm(Algorithm<T>* alg, int nType, double dEval, double dTime);

    /**
    *	Save
    *	@return a XMLTag* that saves the evaluation of the algorithms and the algorithms themselves
    *	@param name sets the name of the node returned
    */
    XMLTag* Save(std::string name = "");
    /**
    *	BestAlgorithm
    *	@brief Selects from a list the best algorithm
    *	@param type specifies the type of algorithm that should be searched, normally a multiple of 0x100 modulo a special case
    *	@param sig specifies the signature that has to be searched
    * @param sensors the list of available sensors seeing the target region
    * @return An algorithm with the highest score for the current situation
    */
    virtual Algorithm<T>* BestAlgorithm(int type, const Signature& sig, const std::vector<Sensor*> &sensors);
    /**
    *	BestAlgorithmList
    *	@brief Selects all algorithm which are applicable
    *	@param type specifies the type of algorithm that should be searched, normally a multiple of 0x100 modulo a special case
    *	@param sig specifies the signature that has to be searched
    * @param sensors the list of available sensors seeing the target region
    * @return A list of algorithm with score greaer than 0
    */
    std::vector<Algorithm<T>*> BestAlgorithmList(int type, const Signature &sig, const std::vector<Sensor*> &sensors);

    /**
    *	Return num of Algorithms in the visual finder
    */
    int CountAlgorithms(){return m_algorithmlist.size();}
    /**
    *	Get the algorithm list
    */
    const std::vector<AlgorithmEval<T> >& GetAlgorithmList() const {return m_algorithmlist;};
    /**
    *  Replace the evaluatro interface function
    */
    virtual void EvalAlgorithm(Evaluable* alg, double eval, double time, Signature* relatedElemg);
    /**
    * Aposteriori evaulation of poses, if there were previous results with this algorithm at a similar position,
      to judge the reliability
    */
    void ReevaluatePose(Algorithm<T>* alg, RelPose*& pose);


  protected:
    /**
    *	Insets in the list of a special type of algorithms
    **/
    int InsertInList(AlgorithmEval<T> eval);

    /**
    *	test compatibility of an algorithm with a task
    */
    bool CheckTypeCompatibility(int listedType, int askedType);

    /** Helper for ReevaluatePose*/
    double ReEvalKernel(double val);


    /**
    *	Get the algorithm list
    */
    std::vector<AlgorithmEval<T> >& GetAlgorithmList(std::vector<AlgorithmEval<T> >* eval = NULL) ;

    /**
    * Get the value of m_algorithmlist
    * @return the value of m_algorithmlist
    */
    Algorithm<T>* getAlgorithm(int index);

  private:
      /**
      *   List of Algorithms, including their evaluation
      */
    std::vector<AlgorithmEval<T> > m_algorithmlist;

      /**
      *
      */
      ANNkd_tree    *m_tree;
      ANNpointArray m_pointArray;
      double        *m_pointEval;
      int           m_paLength;
      int           m_paAllocatedLength;
      int           m_paDim;

      typedef boost::mutex::scoped_lock lock;
      boost::mutex  *m_mutexAlgEval;    

      /**
      *   LogFile
      */
  public:
  #ifdef LOGFILE
      LogFile& m_logfile;
  #endif /*LOGFILE*/
  };
}
#endif // ALGORITHMSELECTOR_H
