/* 
 * Copyright (c) 2010, Ulrich Klank, Dejan Pangercic
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

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
DetectedFaces.cpp
**************************************************************************/


#include "DetectedFace.h"
#include "IplImageReading.h"
#include "ImageSubscription.h"


#include "cv.h"
#include "highgui.h"

using namespace cop;

#define XML_NODE_LASTMATCHEDFACES "LastMatchedFaces"

void DetectedFace::Show(RelPose* pose, Sensor* camin)
{
  const static cv::Scalar colors[] =  { CV_RGB(0,0,255),
                                  CV_RGB(0,128,255),
                                  CV_RGB(0,255,255),
                                  CV_RGB(0,255,0),
                                  CV_RGB(255,128,0),
                                  CV_RGB(255,255,0),
                                  CV_RGB(255,0,0),
                                  CV_RGB(255,0,255)} ;
  if(camin != NULL && camin->GetName().compare("ImageSubscription") == 0)
  {
    IplImageReading *reading = (IplImageReading*)(camin->GetReading(-1));
    cv::Mat &img = reading->m_image;
    RelPose* pose = GetLastMatchedPose();
    if(m_lastlyDetectedFaces.find(pose->m_uniqueID) != m_lastlyDetectedFaces.end())
    {
      std::vector<double> &vec = m_lastlyDetectedFaces[pose->m_uniqueID];
      if(vec.size() > 3)
      {
        circle( img, cv::Point2f(vec[0], vec[1]), vec[2], colors[pose->m_uniqueID % 8], 3, 8, 0 );
      }
    }
    //cv::imshow(((ImageSubscription*)camin)->GetWindowName().c_str() , img);
  }
  else
  {
    printf("No camera available\n");
  }
}

void DetectedFace::SaveTo(XMLTag* tag)
{

  Descriptor::SaveTo(tag);

  tag->AddChild(XMLTag::Tag(m_lastlyDetectedFaces, XML_NODE_LASTMATCHEDFACES));

}

void DetectedFace::SetData(XMLTag* tag)
{
  Descriptor::SetData(tag);
  XMLTag* lastly_faces_node = tag->GetChild(XML_NODE_LASTMATCHEDFACES);
  m_lastlyDetectedFaces = XMLTag::Load(lastly_faces_node, &m_lastlyDetectedFaces);
};
