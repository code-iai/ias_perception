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
                        Algorithm.h - Copyright klank


**************************************************************************/



#ifndef ALGORITHM_H
#define ALGORITHM_H

#include <string>
#include <vector>
#include "Sensor.h"
#include "Signature.h"

namespace cop
{

  class Evaluable
  {
  public:
    /**
    *	CheckSignature
    *	@brief evaluate shortly of the signature contains necessary model information to perform the algorithm
    *	@param object the signature that has to be checked
    * @param sensors  a list of available sensors seeing the target area
    *	@return an evaluation from 0 to 1: 1 saying that there is all needed information ,
    *			0 saying it is impossible to perform the algorithm,
    *			everything in between says that it will take longer or will be inaccurate
    */
    virtual double CheckSignature(const Signature& object, const std::vector<Sensor*> &sensors) = 0;
    /**
    *	Show: show the algorithm results in a camera
    */
    virtual void Show(Sensor* ){};
    /**
    *
    */
    virtual std::string GetName(){return "Algorithm";}
    /**
    *	Save
    *	@brief saves the algorithms name and a parameter set
    */
    virtual XMLTag* Save() = 0;

    AlgorithmID_t m_AlgIndex;
  };

  /**
    * class Algorithm
    *	@brief interface for all algorithms that can be executed by the AlgorithmSelector
    */
  template<typename T>
  class Algorithm : public Evaluable
  {
  public:

    // Constructors/Destructors
    //
    /**
    * Constructor
    */
    Algorithm ( ){}
    /**
    *	AlgFactory
    *
    *	@brief Factory for all knwon Algorithms, this function has to be adapted when a new algorithm is added!
    *	@param tag contains the identifier and the information about the algorithm
    */
    static Algorithm<T>* AlgFactory(XMLTag* tag);

    /**
    * Empty Destructor
    */
    virtual ~Algorithm( ){}
    /**
    *	Perform
    *	@brief Performs the main action of the algorithm (online)
    *	@param sensors a list of sensors that look at the specified position
    *	@param pose Estimated position of the scene, mostly the center of the image (if the camera can be moved)
    *	@param object in/out A signature class that contains all model information about a certain object or a class of objects. This object will retrieve the results of the algorithm
    *	@param numOfObjects in/out The number of instances that should be detected of this signature atmost, it will be filled with 0 .. numOfObjects on exit and should contain the information how many objects were found
    *	@param qualityMeasure out this param will should a score that can be used to evaluate the algorithm in this situation
    *	@return An instanz of T
    */
    virtual T Perform(std::vector<Sensor*> sensors, RelPose* pose, Signature& object, int &numOfObjects, double& qualityMeasure) = 0;
  };
}
#endif // ALGORITHM_H
