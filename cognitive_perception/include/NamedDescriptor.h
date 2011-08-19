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
                       NamedDescriptor.h - Copyright klank

**************************************************************************/


#ifndef NAMEDDESCRIPTOR_H
#define NAMEDDESCRIPTOR_H

#include <string>
#include <ros/ros.h>
#include <sstream>
#include <Descriptor.h>


namespace cop
{

  template <class Content>  class NamedDescriptor: public Descriptor
  {
  public:
    NamedDescriptor(const char* Name, const char* ClassName, const ElemType_t DescriptorType) :
      m_stXMLName(Name),
      m_stClassName(ClassName),
      m_elemType(DescriptorType)
     {
       m_class = new Class();
       m_class->SetName(m_stClassName);
     }
     std::string BuildFileName(){
        std::ostringstream os;
        os << GetNodeName().c_str() << "_" <<m_ID << ".msg";
        std::string filename = os.str();
        return filename;
     }
     void SetClass(std::string name)
     {
       delete m_class;
       m_stClassName = name;
       m_class = new Class();
       m_class->SetName(m_stClassName);
     }
     std::string GetNodeName() const{return m_stXMLName;}
     ElemType_t GetType() const {return m_elemType;}

     virtual void ContentToFileFunction(std::string filename)
     {
        uint32_t length = m_content.serializationLength();
        uint8_t* write_pointer = new uint8_t[length];
        uint8_t seq = 0, *outp;
        outp = m_content.serialize(write_pointer, seq);
        FILE* file = fopen(filename.c_str(), "w");
        if(file != NULL)
        {
          fwrite(write_pointer, 1, length, file);
          fclose(file);
        }
        else
        {
          ROS_ERROR("Could not write message to file (filename: %s in Class %s)\n", filename.c_str(), GetNodeName().c_str());
        }
        delete write_pointer;
     };
     virtual void FileToContentFunction(std::string filename)
     {
       if(filename.length() > 0)
       {
          FILE* file = fopen(filename.c_str(), "r");
          if (file==NULL)
          {
            ROS_ERROR("Could not read message from file (filename: %s in Class %s)\n", filename.c_str(), GetNodeName().c_str());
            throw("Error reading class");
          }

          // obtain file size:
          fseek (file, 0 , SEEK_END);
          long lSize = ftell (file);
          rewind (file);
          uint8_t*  buffer = new uint8_t[lSize];
          if (buffer == NULL)
          {
            ROS_ERROR("Could not read message from file (filename: %s in Class %s)\n", filename.c_str(), GetNodeName().c_str());
            throw("Error reading class");
          }

          // copy the file into the buffer:
          long result = fread (buffer,1,lSize,file);
          if (result != lSize)
          {
            ROS_ERROR("Could not read message from file (filename: %s in Class %s)\n", filename.c_str(), GetNodeName().c_str());
            delete buffer;
            throw("Error reading class");
          }
          m_content.deserialize(buffer);
          fclose (file);
          delete buffer;
       }
     }
     void SetContent(Content cont){m_content = cont;};
     Content GetContent() { return m_content;}
     std::string m_stXMLName;
     std::string m_stClassName;
     ElemType_t m_elemType;
     Content m_content;
  };

#define CREATE_NAMED_DESCRIPTOR(DescriptorName, XMLDescriptorName, ClassName, DescriptorType, Content) \
  class DescriptorName : public NamedDescriptor<Content>                                               \
  {                                                                                                    \
  public:                                                                                              \
    DescriptorName() : NamedDescriptor<Content>(XMLDescriptorName, ClassName, DescriptorType){}        \
    void SaveTo(XMLTag* tag)                                                                           \
    {                                                                                                  \
        Descriptor::SaveTo(tag);                                                                       \
        try                                                                                            \
        {                                                                                              \
	   std::string filename = BuildFileName();                                                      \
	   printf("Trying to save to %s\n", filename.c_str());                                     \
	   tag->AddProperty(XML_ATTRIBUTE_FILENAME, filename);                                          \
          ContentToFileFunction(filename);                                                    \
        }                                                                                              \
        catch(...)                                                                                     \
        {                                                                                              \
          ROS_ERROR("%s:Failed to save data\n", GetNodeName().c_str());                                \
        }                                                                                              \
     }                                                                                                 \
  protected:                                                                                           \
    void SetData(XMLTag* tag)                                                                          \
    {                                                                                                  \
        Descriptor::SetData(tag);                                                                      \
        FileToContentFunction(tag->GetProperty(XML_ATTRIBUTE_FILENAME));                             \
    }                                                                                                 \
  };                                  \

#define CREATE_NAMED_DESCRIPTOR_SHOW(DescriptorName, XMLDescriptorName, ClassName, DescriptorType, Content) \
  class DescriptorName : public NamedDescriptor<Content>                                               \
  {                                                                                                    \
  public:                                                                                              \
    DescriptorName() : NamedDescriptor<Content>(XMLDescriptorName, ClassName, DescriptorType){}        \
    void SaveTo(XMLTag* tag)                                                                           \
    {                                                                                                  \
        Descriptor::SaveTo(tag);                                                                       \
        try                                                                                            \
        {                                                                                              \
	   std::string filename = BuildFileName();                                                      \
	   printf("Trying to save to %s\n", filename.c_str());                                     \
	   tag->AddProperty(XML_ATTRIBUTE_FILENAME, filename);                                          \
          ContentToFileFunction(filename);                                                    \
        }                                                                                              \
        catch(...)                                                                                     \
        {                                                                                              \
          ROS_ERROR("%s:Failed to save data\n", GetNodeName().c_str());                                \
        }                                                                                              \
     }                                                                                                 \
     void Show(RelPose* pose, Sensor* sens);                                                           \
  protected:                                                                                           \
    void SetData(XMLTag* tag)                                                                          \
    {                                                                                                  \
        Descriptor::SetData(tag);                                                                      \
        FileToContentFunction(tag->GetProperty(XML_ATTRIBUTE_FILENAME));                             \
    }                                                                                                 \
  };                                  \

}
#endif // NAMEDDESCRIPTOR_H

