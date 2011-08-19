
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
                        AttentionAlgorithm.h - Copyright klank


**************************************************************************/


#ifndef ATTENTIONALGORITHM_H
#define ATTENTIONALGORITHM_H
#include "Algorithm.h"
#include "Reading.h"

#include <string>
#include <vector>


namespace cop
{
  /**
    * class AttentionAlgorithm
    * @brief Specialisation of algoithms that return a 3d position as an result
    */
  class AttentionAlgorithm : public Algorithm<std::vector<Signature*> >
  {
  public:

    // Constructors/Destructors
    //
    static AttentionAlgorithm* AttentionAlgFactory(XMLTag* tag);

    /**
     * Constructor
     */
    AttentionAlgorithm (){};

    /**
     * Destructor
     */
    virtual ~AttentionAlgorithm ( ){};

    /**
     *  Perform
     *  @brief  the function that is called to execute this current algorithm
     *  @param   sensors    list of available sensors for attending
     *  @param   pose          Position that can restict the  search
     *  @param   prototype              Description of the object to search for
     *  @param   numOfObjects      number of objects that should be found and on return that were found
     *  @param   qualityMeasure  Takes threshold limiting results quality and receives resulting quality
     *  @return a list of objects which should be attended
     */
    virtual std::vector<Signature*> Perform(std::vector<Sensor*> sensors, RelPose* pose, Signature& prototype, int &numOfObjects, double& qualityMeasure) = 0;//{throw "Perform for AttentionAlgorithm not implemented\n";}

    virtual double CheckSignature(const Signature& object, const std::vector<Sensor*> &sensors){return 0.0;}

    virtual XMLTag* Save() = 0;
    virtual void SetData(XMLTag* tag){}

  };

}
#endif // ATTENTIONALGORITHM_H

