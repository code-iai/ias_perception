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
                        LocateAlgorithm.cpp - Copyright klank
**************************************************************************/

#include "LocateAlgorithm.h"
#include "XMLTag.h"

#include <pluginlib/class_loader.h>


// Constructors/Destructors
//
#include <time.h>
#define TRACKING_POSSIBLE_MS 2

using namespace cop;



pluginlib::ClassLoader<LocateAlgorithm> s_localg_loader("cognitive_perception", "LocateAlgorithm");

LocateAlgorithm* LocateAlgorithm::LocAlgFactory(XMLTag* tag)
{
  std::string name = tag->GetName();
  LocateAlgorithm* alg = NULL;
  try
  {
    alg = s_localg_loader.createClassInstance(name);
    if(alg != NULL)
      alg->SetData(tag);
  }
  catch(pluginlib::PluginlibException& ex)
  {
  //handle the class failing to load
    ROS_WARN("LocateAlgorithm: The plugin failed to load for some reason. Error: %s\n", ex.what());
    ROS_WARN("LocateAlgorithm: Tag failed: %s\n", tag->GetName().c_str());
  }
	return alg;
}


bool LocateAlgorithm::TrackingPossible(const Reading& img, const Signature& sig, RelPose* pose)
{
	unsigned long l = (unsigned long)time(NULL);
	l -= sig.date();
#ifdef _DEBUG
	printf("TrackingPossible? Last Match diff: %ld\n", l);
#endif
        const RelPose* old_pose = sig.GetObjectPose();
        if(old_pose != NULL)
        {
          RelPose* copy = RelPoseFactory::FRelPose(old_pose->m_uniqueID);
	  if(l > 0 && l < TRACKING_POSSIBLE_MS && pose != NULL && copy != NULL)
	  {
		Matrix m = copy->GetCovarianceMatrix(0);
		//TODO Check Timestamp
#ifdef _DEBUG
		printf("Check 3 Cov Values of %ld: %f < 0.1; %f < 0.1;%f < 0.1;\n", old_pose->m_uniqueID, m.data()[0],m.data()[7], m.data()[14]);
#endif
		return m.data()[0] < 0.1 && m.data()[7] < 0.1 && m.data()[14] < 0.1;
	  }
	  RelPoseFactory::FreeRelPose(&copy);
  }
	return false;

}


// Other methods
//


