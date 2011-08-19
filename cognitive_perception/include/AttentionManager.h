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
                        AttentionManager.h - Copyright klank


**************************************************************************/


#ifndef ATTENTIONMANAGER_H
#define ATTENTIONMANAGER_H

#include <string>
#include <vector>
#include "XMLTag.h"
#include "LogFile.h"
#include "Signature.h"
#include "AlgorithmSelector.h"
#include "RelPose.h"
#include "Comm.h"
#include "PerceptionPrimitive.h"

#ifdef BOOST_THREAD
#include "boost/thread.hpp"
#else
#endif
#define XML_NODE_ATTENTIONMANAGER "AttentionManager"


namespace cop
{
  class VisFinder;
  class SignatureDB;
  class ImageInputSystem;

  class AttendedObjects
  {
  public:
    AttendedObjects(PerceptionPrimitive* proto_in, PossibleLocations_t* poses_in, Comm* comm_in) : proto(proto_in), poses(poses_in), comm(comm_in) {}
    PerceptionPrimitive* proto;
    PossibleLocations_t* poses;
    Comm* comm;
  };
  /**
    * class AttentionManager
    *	@brief tries to detect constantly new objects in the visible area
    *
    *
    * Nodename: XML_NODE_ATTENTIONMANAGER "AttentionManager"  Manages passive mechanims for attention handling
    *
    * Properties: XML_NODE_ATTENTIONALGORITHMS "Attendants"   Contains the list of methods that will be called cyclicly this node is of type AlgorithmSelector<std::vector<Signature*> > which means that it contains a list of \see AttetionAlgorithm
    *
    */
  class AttentionManager
  {
  public:
    /**
     * Constructor
     */
    AttentionManager ( XMLTag* config , SignatureDB& sig_db, ImageInputSystem& imginsys
  #ifdef LOGFILE
          , LogFile& log
  #endif /*LOGFILE*/
          );

    /**
     * Destructor
     */
    virtual ~AttentionManager ( );

    XMLTag* Save();
    /**
    *  SetAttentionLevel
    * @param attentionLevel
    * @param visfinder
    */
    void SetAttentionLevel (double attentionLevel, VisFinder* visfinder);
    void SetObjectToAttend (PerceptionPrimitive* prototype, PossibleLocations_t* pointOfInterest, Comm* comm);
    void StopAttend (Comm* comm);

    void PerformAttentionAlg( std::vector<Sensor*> &sensors, RelPose* pose, Signature* sig, AttendedObjects& objProto);
    const AlgorithmSelector< std::vector<Signature*> > GetAttentionAlgorithmSelection(){return m_attendants;}

    void threadfunc();
  #ifdef BOOST_THREAD
    boost::thread* m_learningThread;
  #endif
    bool m_Attending;
    ImageInputSystem&   m_imginsys;
  private:
    AlgorithmSelector< std::vector<Signature*> >	m_attendants;
    std::vector<AttendedObjects> m_attendedObjectPrototypes;


  #ifdef LOGFILE
      LogFile&            m_logFile;
  #endif /*LOGFILE*/

  private:
    AttentionManager& operator=(AttentionManager&){throw "Error";}
    SignatureDB& m_sigDB;
  };
}
#endif // ATTENTIONMANAGER_H
