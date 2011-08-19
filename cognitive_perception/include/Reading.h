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
                        READING_H - Copyright klank

**************************************************************************/


#ifndef READING_H
#define READING_H

#include <string>
#include <map>
#include <stdio.h>
/**
  * class Reading
  */

#define XML_NODE_IMAGEFILE "ImageFile"
#define XML_ATTRIBUTE_FILENAME "FileName"
#define XML_ATTRIBUTE_IMGTYPE "ImgType"

enum ReadingType_t
{
  ReadingType_HalconImage,
  ReadingType_IplImage,
  ReadingType_PointCloud
};

namespace cop
{
  class XMLTag;
  class RelPose;
  class ReadingConverter;

  /******************************************************************
  *           class Reading                                         */
  /** ****************************************************************
  *   @brief Abstraction of an reading, can be derived for different
              images or acquisition devices
  *   Provides Halcon Image and IplImage
  *******************************************************************/
  class Reading
  {
  public:

    // Constructors/Destructors
    //


    /**
     * Empty Constructor
     */
    Reading (ReadingType_t type) :
        m_readingType(type),
        m_usageCount(0),
        m_timestamp((unsigned long)time(NULL)),
        m_relPose(NULL)
    {}

    /**
    * Constructor Reading
    *   @param tag contains a file name or similar information
    *   @throws char* with an error message in case of failure
    */
    static Reading* ReadingFactory( XMLTag* tag);

    /**
    *  This can be overwritten to get the data necessary to reconstruct a saved reading
    */
    virtual void SetData(XMLTag* tag){}

    /**
     * Empty Destructor
     *   @throws char* with an error message in case of failure
     */
    virtual ~Reading ( );

    /**
     *   Save the image in a file and put a string in the xml
     */
    virtual XMLTag* Save() = 0;

    /**
     *   Copy the image
     */
    virtual Reading* Clone() = 0;

    /**
     *   Reference Counter
     */
    void Hold()
    {
        m_usageCount++;
    }
    /**
     *   Decrease Reference Counter
     */
    void Free(){
        m_usageCount--;
    }
    /**
     *   Type of the image in memory
     */
    ReadingType_t GetType() const{printf("Requested reading type: %d\n", m_readingType); return m_readingType;}
    /**
    *  Set pose stamp of this image
    */
    void SetPose(RelPose* parent);
    /**
    *  Get pose stamp of this image
    */
    RelPose* GetPose() const
    {
        return m_relPose;
    }

    Reading* ConvertTo(ReadingType_t type);

    /**
    *  Time stamp of this image
    */
    virtual unsigned long date() const
    {
        return m_timestamp;
    }


  public:
      static std::map<std::pair<ReadingType_t, ReadingType_t> , ReadingConverter*> s_conv;
      ReadingType_t m_readingType;
      int m_usageCount;
      unsigned long m_timestamp;
      RelPose* m_relPose;
  };

  /******************************************************************
  *           class ReadingConverter                                */
  /** ****************************************************************
  *   @brief Interface to Allows conversion from one reading type
  *          to another
  *******************************************************************/
  class ReadingConverter
  {
  public:
    static ReadingConverter* ReadingConverterFactory(std::string name);
    virtual Reading* Convert(Reading* in) = 0;
    virtual ReadingType_t TypeIn() = 0;
    virtual ReadingType_t TypeOut() = 0;
  };

}
#endif // READING_H
