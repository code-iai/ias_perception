
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
                        VisFinder.h - Copyright klank

**************************************************************************/


#ifndef VISFINDER_H
#define VISFINDER_H

#include <string>
#include <vector>

#include "AlgorithmSelector.h"
#include "RelPoseFactory.h"
#include "AttentionManager.h"
#include "VisLearner.h"
#include "TrackAlgorithm.h"
#include "PerceptionPrimitive.h"

#define XML_NODE_VISFINDER "VisualFinder"

namespace cop
{
  /**
    * class VisFinder
    * @brief Basic class for loacting objects
    */
  class VisFinder
  {

  public:

    // Constructors/Destructors
    //

    /**
     * Constructor VisFinder
     * @brief constructs a visual finder
     * @param configFile configuation of the visual finder, menaing which algorithms are in it
     * @param imageSystem reference to the camera setup ussed to get images from
     * @param db reference the the signature database used for looking up model information
     * @param manager Attention Manager, not specified
     * @param visLearner reference to the model improver can be triggered by the visual finder
      #ifdef LOGFILE
     * @param log the central logfile
     #endif
     */
    VisFinder ( XMLTag*  configFile, ImageInputSystem* imageSystem, SignatureDB* db, AttentionManager* manager, VisLearner*	visLearner
  #ifdef LOGFILE
          , LogFile& log
  #endif
          );

    /**
     * Empty Destructor
     */
    virtual ~VisFinder ( );



    VisFinder& operator=(VisFinder&){throw "Error";}
    /**
    *	Locate
    *	@brief locates the class or object that is specified by object, updates its signature if it succeeds,
    *			sets the numOfObjects, returns a SignatureLocations_t.
    *	@param  lastKnownPoses a list of possible positions and the a-prioi probabilitiers for the object beeing at this position
    *	@param  object         the signature that will be searched for
    *	@param  numOfObjects
    *	@return a list of position, sorted by score
    */
    virtual SignatureLocations_t Locate (PossibleLocations_t* lastKnownPoses, PerceptionPrimitive& object, int &numOfObjects);

    /**
    *  Checks if there is a the possibility to create search spaces, if there are no given
    *  @param poses   resulting poses
    *  @param obj     type of this could be relevant
    *  @param sensors list of available sensors, should contain a 3D sensor
    */
    bool GetPlaneClusterCall(PossibleLocations_t* poses, RelPose*, PerceptionPrimitive& obj, const std::vector<Sensor*> &sensors);

    /**
     *   Starts a thread tracks the object specified with object
     *   @param  object contains a trackable signature and the vision primitive id
     *   @param pose  intial guess of the pose of the trackable object
     */
    virtual void StartTrack (PerceptionPrimitive& object, RelPose* pose);
    /**
     *  Stops the thread that tracks the object specified with object
     *  @param  object contains a trackable signature
     */
    virtual void StopTrack (Signature& object);


    /**
     *  Not yet finished, concept not yet integrated
     * @param  pose
     * @param  sig1
     * @param  sig2
     *
     * @return RelPose
     *
     */
    virtual RelPose* RelTwoObjects (const RelPose& pose, Signature& sig1, Signature& sig2);
    /**
    *	Saves the Visual Finder
    */
    virtual XMLTag* Save();
    /**
    * AddAlgorithm
    * @brief Adds a new Algorithm to the visual finder
    * @param alg the algorithm that should be added to the list of algorithms available
    */
    void AddAlgorithm(Algorithm<std::vector<RelPose*> >* alg);
    /**
    *	Return num of Algorithms in the visual finder
    */
    int CountAlgorithms(){return m_selLocate.CountAlgorithms();}

    const AlgorithmSelector<std::vector<RelPose*> >& GetAlgorithmSelection(){return m_selLocate;}
  private:
      /**
     *	The list of algorithms and their type and evaluation
     */
      AlgorithmSelector<std::vector<RelPose*> > m_selLocate;
      std::map<int, TrackAlgorithm*> m_runningTracks;
  public:
    ImageInputSystem* m_imageSys;
    SignatureDB*      m_sigdb;
    VisLearner*       m_visLearner;
    AttentionManager* m_attentionMan;


  };
}
#endif // VISFINDER_H
