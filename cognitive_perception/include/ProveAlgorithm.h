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


#ifndef PROVEALGORITHM_H
#define PROVEALGORITHM_H

#include "Algorithm.h"

#define XML_NODE_PRASP "ProveShapevsDescr"


namespace cop
{
  /**
  *	typedefs:
  */
  typedef std::pair<RelPose*, Probability_1D_t> ImprovedPose;

    /**
    * class ProveAlgorithm
    *	@brief gives a verification of an earlier measurement
    */
  class ProveAlgorithm :
    public Algorithm<ImprovedPose>
  {
  public:
    ProveAlgorithm(void);
    ~ProveAlgorithm(void);

    virtual ImprovedPose Perform(std::vector<Sensor*> sensors, RelPose* pose, Signature& Object, int &numOfObjects, double& qualityMeasure) = 0;//{throw "Perform for LocateAlgorithm not implemented\n";}

    double CheckSignature(const Signature& object, const std::vector<Sensor*> &sensors) = 0;

    virtual XMLTag* Save() = 0;//{throw "Save for LocateAlgorithm not implemented\n";}
    virtual void SetData(XMLTag* ) = 0;//{throw "Save for LocateAlgorithm not implemented\n";}

    static ProveAlgorithm* ProveAlgFactory(XMLTag* tag);

    virtual std::string GetName(){return "ProveAlgorithm";}

  };
}
#endif
