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
                        LocateAlgorithm.h - Copyright klank


**************************************************************************/


#ifndef LOCATEALGORITHM_H
#define LOCATEALGORITHM_H
#include "Algorithm.h"
#include "Reading.h"
#include <string>
#include <vector>


namespace cop
{
  /**
    * class LocateAlgorithm
    * @brief Specialisation of algoithms that return a 3d position as an result
    */
  class LocateAlgorithm : public Algorithm<std::vector<RelPose*> >
  {
  public:

    // Constructors/Destructors
    //
    static LocateAlgorithm* LocAlgFactory(XMLTag* tag);


    /**
     * Constructor
     */
    LocateAlgorithm (){};

    /**
     * Destructor
     */
    virtual ~LocateAlgorithm ( ){};

    /**
     *  Perform
     *  @brief  the function that is called to execute this current algorithm
     *  @param   cam    list of cameras available for locating
     *  @param   pose   current pose hypothesis that could be used for search space restriction
     *  @param   Object Description of the object to search for and the receiver of information updates
     *  @param   numOfObjects      number of objects that should be found and on return that were found
     */
    virtual std::vector<RelPose*> Perform(std::vector<Sensor*> sensors, RelPose* pose, Signature& Object, int &numOfObjects, double& qualityMeasure) = 0;//{throw "Perform for LocateAlgorithm not implemented\n";}

    virtual double CheckSignature(const Signature& object, const std::vector<Sensor*> &sensors){return 0.0;}

    virtual bool TrackingPossible(const Reading& img, const Signature& sig, RelPose* pose);
    virtual XMLTag* Save() = 0;//{throw "Save for LocateAlgorithm not implemented\n";}
    virtual void SetData(XMLTag* tag){}

  };
}
#endif // LOCATEALGORITHM_H
