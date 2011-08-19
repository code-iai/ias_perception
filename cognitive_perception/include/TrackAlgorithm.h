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
                        TrackAlgorithm.h - Copyright klank

**************************************************************************/


#ifndef TRACKALGORITHM_H
#define TRACKALGORITHM_H

#include "Algorithm.h"

#include <string>
#include <vector>

#include "ImageInputSystem.h"
#include "PerceptionPrimitive.h"

namespace cop
{
  /**
    * class TrackAlgorithm
    */

  class TrackAlgorithm
  {
  public:
    /**
     * Constructor
     */
    TrackAlgorithm (PerceptionPrimitive& prim, Evaluator* eval, Algorithm<std::vector<RelPose*> > *alg, ImageInputSystem* imageSys);

    /**
     * Empty Destructor
     */
    virtual ~TrackAlgorithm ( );

    XMLTag* Save();
    /**
    *    function tha performs the tracking
    */
    void threadfunc();
    /**
    *    a variable marking this tracking as still running or not
    */
    volatile bool m_Running;
  private:
    /**
    *    the algorithm that was passed by the constructor
    */
    Algorithm<std::vector<RelPose* > > *m_alg;
    /**
    *    The object that should be tracked
    */
    PerceptionPrimitive& m_curPrim;
    /**
    *    Reference to the sensor system
    */
    ImageInputSystem*	m_imageSys;
    /**
    *   Pointer to the calling entity that retrieves the evaluation
    */
    Evaluator* m_eval;
  private:
  #ifdef BOOST_THREAD
    boost::thread* m_trackingThread;
  #else
  #endif
    TrackAlgorithm& operator=(TrackAlgorithm&){throw "private";}

  };
}
#endif // TRACKALGORITHM_H
