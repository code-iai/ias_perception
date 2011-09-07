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


#include "CollisionInterface.h"

#define STD_CONFIG_FILENAME "bla.xml"

#include <ros/ros.h>

using namespace cop;


int main(int argc, char* argv[])
{
#ifndef USE_YARP_COMM
    std::string nodename = "cop";
    if(argc > 2)
      nodename = argv[2];
    ros::init(argc, argv, nodename);
#endif
    try
    {
    if(argc > 0)
    {
        XMLTag* config = NULL;
        if(argc > 1)
            config = XMLTag::ReadFromFile(argv[1]);
        else
            config = XMLTag::ReadFromFile(STD_CONFIG_FILENAME);
        if(config == NULL)
            config = new XMLTag("config");
        /*Blob* blob = new Blob(235, 255, 0, 35, 0, 40, 750, 5000, 0.4, 0.095, 0.05);*/

        printf("Init start:\n");
        ros::NodeHandle nh;
        cop_world copWorld(config, nodename);
        CollisionInterface collision(copWorld, nh);
        /*XMLTag tag(XML_NODE_CAMERADRIVER);
        tag.AddProperty(XML_ATTRIBUTE_GRABBERNAME, "DirectShow");
        tag.AddProperty(XML_ATTRIBUTE_EXTERNALTRIGGER, "false");
        tag.AddProperty(XML_ATTRIBUTE_CAMERATYPE, "RGB24 (640x480)");
        tag.AddProperty(XML_ATTRIBUTE_DEVICE, "USB PC Camera (SN9C102)");
        tag.AddProperty(XML_ATTRIBUTE_CALIBFILE, "C:/__CODE/radigsvn/cop/bin/fakeparam_usbwebcam.dat");
        tag.AddProperty(XML_ATTRIBUTE_CAMPORT, "0");
        tag.AddProperty(XML_ATTRIBUTE_COLORSPACE, "rgb");
        copWorld.s_inputSystem->AddCamera(new CameraDriver(&tag));
        copWorld.s_visFinder->AddAlgorithm(new ShapeBased3D());   */
/*        copWorld.s_visFinder->AddAlgorithm(new BlobLocalizer());
        Signature* sig = new Signature();
        Class* cl = new Class();
        Class* clOra = new Class();
        cl->SetName("EasterEgg");
        clOra->SetName("Orange");
        sig->SetClass(cl);
        sig->SetClass(clOra);
        sig->SetElem(blob);
        copWorld.s_sigDb->AddSignature(sig);*/
        printf("Inited\n");
#ifdef USE_YARP_COMM
        copWorld.StartListeningYarpPort((char*)"/tracking/in");
#else
        if(copWorld.s_visFinder != NULL)
        {
          if(copWorld.s_visFinder->CountAlgorithms() == 0)
             printf("Warning: no algorithm loaded\n");
          copWorld.StartNodeSubscription((char*)"in");

          printf("Returned successfully\n");
  #endif
          if(g_stopall == true)
          {
            if(argc > 1)
            {
              copWorld.SaveCop(STD_CONFIG_FILENAME);
              exit(0);
            }
            else
            {
              copWorld.SaveCop(STD_CONFIG_FILENAME);
              exit(0);
            }
          }
        }
        else
        {
            ROS_ERROR("cop: Loading of main modules failed. Check the configuration and the folder cop is executed in!");
        }
      }
    }
    catch(char* text)
    {
      printf("Error in cop_main: %s\n", text);
      return 1;

    }
    //catch(...)
    //{
    //  printf("Unhandled Exception: stopping program\n");
    //  return 1;
    //}
    printf("exiting %s\n", argv[0]);
    return 0;
}

