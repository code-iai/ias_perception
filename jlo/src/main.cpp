/**********************************************************************************************/
/**
*              Jennifer's Located Object
*              Copyright  (C) 2008, U. Klank
*
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*
*
 **********************************************************************************************/
using namespace std;

#include "RosLoService.h"

#include <tr1/functional>
#include <stdio.h>

bool (*funcpt)(vision_srvs::srvjlo::Request& request, vision_srvs::srvjlo::Response&  answer);

int main(int argc, char** argv)
{
  ros::init(argc, argv, "jlo");
  ros::NodeHandle n("jlo");
  std::string config_file;

  n.param<std::string>("config_file", config_file, "bla.ini");

  if(argc > 1)
  {
    config_file = argv[1];
  }
  else
  {
    printf("Usage: %s configfile servicename\n", argv[0]);
  }
  RosLoService lo ("jlo", n, config_file );
  ros::spin();
  return 0;
}
