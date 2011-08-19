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
                        AttentionAlgorithm.cpp - Copyright klank
**************************************************************************/

#include "AttentionAlgorithm.h"

#include "XMLTag.h"

#include <pluginlib/class_loader.h>


// Constructors/Destructors
//
#include <time.h>
#define TRACKING_POSSIBLE_MS 2

using namespace cop;

 pluginlib::ClassLoader<AttentionAlgorithm> s_attalg_loader("cognitive_perception", "AttentionAlgorithm");


AttentionAlgorithm* AttentionAlgorithm::AttentionAlgFactory(XMLTag* tag)
{
  std::string name = tag->GetName();
  AttentionAlgorithm* alg = NULL;
  try
  {
    ROS_DEBUG("Load AttentionAlgorithm: %s\n", name.c_str());
    alg = s_attalg_loader.createClassInstance(name);
    alg->SetData(tag);
  }
  catch(pluginlib::PluginlibException& ex)
  {
  //handle the class failing to load
    ROS_WARN("The plugin failed to load for some reason. Error: %s\n", ex.what());
    ROS_WARN("Tag failed: %s\n", tag->GetName().c_str());
  }
	return alg;
}

#include "RemoteAttention.h"
#include <std_msgs/UInt64.h>
namespace cop
{

  template <> std::vector<Signature*>  RemoteAttention<std_msgs::UInt64>::MessageToSignature(boost::shared_ptr<std_msgs::UInt64 const> msg)
  {
     std::vector<Signature*> results;

     return results;
  }

}

