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
                        Descriptor.cpp - Copyright klank


**************************************************************************/

#include "Descriptor.h"
#include "XMLTag.h"

#define XML_ATTRIBUTE_QUALITY "DescriptorQuality"

using namespace cop;


// Constructors/Destructors
//
Descriptor::Descriptor (Class *classref ) :
  Elem(),
	m_class(classref),
	m_imgLastMatchReading(NULL),
	m_poseLastMatchReading(NULL),
	m_qualityMeasure(1.0)
{
}

Descriptor::Descriptor():
	m_class(NULL),
	m_imgLastMatchReading(NULL),
	m_poseLastMatchReading(NULL),
	m_qualityMeasure(1.0)
{
}

void Descriptor::SetData ( XMLTag* tag)
{
  Elem::SetData(tag);
  ElemWriteLock lk(m_mutexElems);
  m_imgLastMatchReading = (NULL);
  m_poseLastMatchReading = (NULL);
  m_class = (Class*)Elem::ElemFactory(tag->GetChild(0));
  m_qualityMeasure = tag->GetPropertyDouble(XML_ATTRIBUTE_QUALITY, 1.0);
  
  try
  {
    XMLTag* img = tag->GetChild(XML_NODE_IMAGEFILE);
    if(img != NULL)
    {
      try
      {
        m_imgLastMatchReading = Reading::ReadingFactory(img); /*TODO: implement factory for Readings*/
      }
      catch(...)
      {
        printf("Error reading XML: Last matched image of a descriptor could not be found\n");
      }
    }
    XMLTag* tagPose = tag->GetChild(XML_NODE_RELPOSE);
    if(tagPose != NULL)
      m_poseLastMatchReading = RelPoseFactory::FRelPose(tagPose);

  }
  catch(char const* ch)
  {
    delete m_imgLastMatchReading;
    m_imgLastMatchReading = NULL;
    ROS_WARN("Error reading XML: %s\n", ch);
  }
}

Class* Descriptor::GetClass()
{
	return m_class;
}

Descriptor::~Descriptor ( )
{
	if(m_imgLastMatchReading != NULL)
	{
		m_imgLastMatchReading->Free();
		if(m_imgLastMatchReading->m_usageCount < 0)
			delete m_imgLastMatchReading;
		//RelPoseFactory::FreeRelPose(m_poseLastMatchImage);
    delete m_poseLastMatchReading;
	}
}

//
// Methods
//
void Descriptor::SaveTo(XMLTag* tag)
{
	tag->AddProperty(XML_ATTRIBUTE_QUALITY, m_qualityMeasure);
	if(m_imgLastMatchReading != NULL)
          tag->AddChild(m_imgLastMatchReading->Save());
        if(m_poseLastMatchReading != NULL)           
          tag->AddChild(m_poseLastMatchReading->Save());

	if(m_class != NULL)
		tag->AddChild(m_class->Save(), 0);
	else
		ROS_WARN("Trying to save a broken Descriptor\n");
}

void Descriptor::SetLastMatchedImage(Reading* img, RelPose* pose)
{
  ElemWriteLock lk(m_mutexElems);
  if(img != NULL)
  {
    if( m_imgLastMatchReading != NULL)
    {
      m_imgLastMatchReading->Free();
      RelPoseFactory::FreeRelPose(&m_poseLastMatchReading);
    }
     m_imgLastMatchReading = img->Clone();
  }
  try
  {
	   m_poseLastMatchReading = RelPoseFactory::CloneRelPose(pose);
  }
  catch(...)
  {
    ROS_WARN("Tying to copy a singular position\n");
    m_poseLastMatchReading = RelPoseFactory::FRelPoseWorld();
  }
  this->Touch();
}


