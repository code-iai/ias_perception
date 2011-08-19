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
                        XMLTag.cpp - Copyright klank


**************************************************************************/

#include "XMLTag.h"
#include <libxml/xmlwriter.h>
#include <libxml/xmlreader.h>
#include <libxml/parser.h>

#include "AlgorithmSelector.h"
#include <time.h>
// Constructors/Destructors
//
#define MY_ENCODING "ISO-8859-1"

#define XML_NODE_FILE_REFERENCE "FileReference"
#define XML_ATTRIBUTE_TIMESTAMP "TimeStamp"

#ifndef WIN32
#define  sprintf_s(_DstBuf, size, _Format, whatever) sprintf(_DstBuf, _Format, whatever)
#endif


using namespace cop;


XMLTag::XMLTag ( std::string name) :
    m_name(name)
{
    Touch();
}

XMLTag* XMLTag::Tag(int n, std::string name)
{
    XMLTag* tag = new XMLTag(name.compare("") == 0 ? XML_NODE_INT : name);
    if(tag == NULL)
     throw "Out of memory";
    tag->SetCData(n);
    return tag;
}


XMLTag* XMLTag::Tag(long n, std::string name)
{
    XMLTag* tag = new XMLTag(name.compare("") == 0 ? XML_NODE_INT : name);
    if(tag == NULL)
     throw "Out of memory";
    tag->SetCData(n);
    return tag;
}

XMLTag* XMLTag::Tag(unsigned long n, std::string name)
{
    XMLTag* tag = new XMLTag(name.compare("") == 0 ? XML_NODE_ULONG : name);
    if(tag == NULL)
     throw "Out of memory";
    tag->SetCData(n);
    return tag;
}

XMLTag* XMLTag::Tag(double d, std::string name)
{
    XMLTag* tag = new XMLTag(name.compare("") == 0 ? XML_NODE_DOUBLE : name);
    if(tag == NULL)
     throw "Out of memory";
    tag->SetCData(d);
    return tag;
}

XMLTag* XMLTag::Tag(std::string value, std::string name)
{
    XMLTag* tag = new XMLTag(name.compare("") == 0 ? XML_NODE_STRING : name);
    if(tag == NULL)
     throw "Out of memory";
    tag->SetCData(value);
    return tag;
}

XMLTag* XMLTag::Tag(Elem* e, std::string name)
{
    return e->Save();
}


XMLTag* XMLTag::Tag(Algorithm<ImprovedPose>* alg, std::string name)
{
    if(alg == NULL)
        return NULL;
    return alg->Save();
}

XMLTag* XMLTag::Tag(Algorithm<std::vector<Signature*> >* alg, std::string name)
{
    if(alg == NULL)
        return NULL;
    return alg->Save();
}


XMLTag* XMLTag::Tag(Algorithm<std::vector<RelPose* > >* alg, std::string name)
{
    if(alg == NULL)
        return NULL;
    return alg->Save();
}

XMLTag* XMLTag::Tag(Algorithm<Descriptor*>* alg, std::string name)
{
    if(alg == NULL)
        return NULL;
    return alg->Save();
}


XMLTag* XMLTag::Tag(Sensor* sensor, std::string name)
{
   XMLTag* tag = sensor != NULL ? sensor->Save() : NULL;
  return tag;
}

XMLTag* XMLTag::Tag(const Matrix& alg, std::string name)
{
    XMLTag* tag = new XMLTag(name.compare("") == 0 ? XML_NODE_MATRIX : name);
    if(tag == NULL)
     throw "Out of memory";
    int rows = alg.nrows();
    int cols = alg.ncols();
    tag->AddProperty(XML_ATTRIBUTE_ROWS, rows);
    tag->AddProperty(XML_ATTRIBUTE_COLS, cols);
    for(int r = 0; r < rows; r++)
    {
        for(int c = 0; c < cols; c++)
        {
            tag->AddChild(Tag(alg.element(r,c)));
        }
    }
    return tag;
}

Matrix XMLTag::Load(XMLTag* tag, Matrix* )
{
    if(tag != NULL)
    {
        int rows = tag->GetPropertyInt(XML_ATTRIBUTE_ROWS);
        int cols = tag->GetPropertyInt(XML_ATTRIBUTE_ROWS);
        Matrix m(rows, cols);
        double counter = 0;
        for(int r = 0; r < rows; r++)
        {
            for(int c = 0; c < cols; c++)
            {
                m.Store()[r * rows + c] = Load(tag->GetChild((int)counter), &counter);
                counter++;
            }
        }
        return m;
    }
    throw "Wrong Node";
}

XMLTag::~XMLTag ( )
{
        for(std::vector<XMLTag*>::iterator children = m_children.begin(); children != m_children.end(); children++)
        {
            delete (*children);
            (*children) = NULL;
        }
        m_children.clear();
}

//
// Methods
//

  //static public Methods

    void XMLTag::WriteToFile(const std::string &stFile, XMLTag** fileReference ) const
    {

        //Open File
        xmlTextWriter* pWriter = xmlNewTextWriterFilename(stFile.c_str(), 0);
        int nError = xmlTextWriterStartDocument(pWriter, NULL, MY_ENCODING, NULL);
        if(nError >= 0 )

        {
            //Write File
            Write(pWriter);
            //CloseFile
            nError = xmlTextWriterEndDocument(pWriter);
        }
        xmlFreeTextWriter(pWriter);
        if(fileReference != NULL)
        {
            *fileReference = new XMLTag(XML_NODE_FILE_REFERENCE);
            (*fileReference)->m_cData = stFile;
        }
        pWriter = NULL;

    }

    char* XMLTag::WriteToString()
    {
        int buffersize;
#if defined(LIBXML_TREE_ENABLED) && defined(LIBXML_OUTPUT_ENABLED)
        m_doc = xmlNewDoc(BAD_CAST "1.0");
        xmlNodePtr n = xmlNewNode(NULL, BAD_CAST (GetName().c_str()));
        DocWrite(n);
        xmlDocSetRootElement(m_doc, n);
        xmlDocDumpMemory(m_doc, &m_xmlbuff, &buffersize);
#else
        printf("Error\n");
#endif
        return (char*)m_xmlbuff;
    }

    void XMLTag::DocWrite(xmlNodePtr n) const
    {
        for(std::map<std::string,std::string>::const_iterator iter = m_properties.begin(); iter != m_properties.end(); iter++)
        {
            xmlNewProp(n, (xmlChar*)(*iter).first.c_str(), (xmlChar*)(*iter).second.c_str());
        }
        xmlNodeSetContent(n, BAD_CAST m_cData.c_str());
        for(std::vector<XMLTag*>::const_iterator children = m_children.begin(); children != m_children.end(); children++)
        {
          xmlNodePtr child = xmlNewChild(n, NULL, BAD_CAST ((*children)->GetName().c_str()), NULL);
          (*children)->DocWrite(child);
        }

    }

    void XMLTag::FreeAfterWriteToString()
    {
       /*
       * Free associated memory.
       */
      xmlFree(m_xmlbuff);
      xmlFreeDoc(m_doc);
    }


    void XMLTag::Write(xmlTextWriter* pWriter) const
    {
        xmlTextWriterWriteRaw(pWriter, (xmlChar*)"\n");
        xmlTextWriterStartElement(pWriter, (xmlChar*)m_name.c_str());
        for(std::map<std::string,std::string>::const_iterator iter = m_properties.begin(); iter != m_properties.end(); iter++)
        {
            xmlTextWriterWriteAttribute(pWriter, (xmlChar*)(*iter).first.c_str(), (xmlChar*)(*iter).second.c_str());
            xmlTextWriterEndAttribute(pWriter);
        }
        if(m_cData.length() > 0)
            xmlTextWriterWriteFormatRaw(pWriter, m_cData.c_str());
        for(std::vector<XMLTag*>::const_iterator children = m_children.begin(); children != m_children.end(); children++)
        {
            (*children)->Write(pWriter);
        }

        xmlTextWriterEndElement(pWriter);
        //xmlTextWriterWriteRaw(pWriter, (xmlChar*)"\n");
    }

    XMLTag* XMLTag::ReadFromFile(const std::string &stFile)
    {
        XMLTag* tag = NULL;
        xmlTextReader* pReader= xmlNewTextReaderFilename(stFile.c_str());
        if(pReader != NULL)
        {
            xmlGetCompressMode();

            xmlTextReaderRead(pReader);
            xmlDocPtr doc = xmlTextReaderCurrentDoc(pReader);
            xmlDocGetRootElement(doc);
            //xmlChar* text =  xmlNodeGetContent(node);
            tag = Read(pReader);

        }
        xmlFreeTextReader(pReader);
        return tag;
    }

    XMLTag* XMLTag::Clone() const
    {
        XMLTag* tag			= new XMLTag(this->m_name);
        tag->m_properties	= this->m_properties;
        tag->m_cData		= this->m_cData;
        tag->m_name			= this->m_name;

        for(std::vector<XMLTag*>::const_iterator children = m_children.begin(); children != m_children.end(); children++)
        {
          if((*children) != NULL)
            tag->AddChild((*children)->Clone());
        }
        return tag;
    }

    XMLTag* XMLTag::Read(xmlTextReader* pReader)
    {
        XMLTag* tag = NULL;
        xmlNode* node = xmlTextReaderCurrentNode(pReader);
        if(node != NULL && node->type != XML_TEXT_NODE)
        {
            tag = new XMLTag(std::string((char*)node->name));
            if(xmlTextReaderHasAttributes(pReader))
            {
                int nAttr = xmlTextReaderAttributeCount(pReader);
                xmlAttr* attr = node->properties;
                for(int i = 0; i < nAttr ; i++, attr = attr->next)
                {

                    std::string attrValue = (char*)xmlTextReaderGetAttributeNo(pReader, i);
                    std::string attrName = (char*)attr->name;
                    tag->AddProperty(attrName, attrValue);
                }
            }
            if(!xmlTextReaderIsEmptyElement(pReader))
            {
                int ChildrenDepth = xmlTextReaderDepth(pReader) + 1;
                while(true)
                {
                    xmlTextReaderRead(pReader);
                    xmlNode* check = xmlTextReaderCurrentNode(pReader);
                    if(ChildrenDepth != xmlTextReaderDepth(pReader))
                    {
                        break;
                    }
                    if(check->type != XML_TEXT_NODE)
                    {
                        XMLTag* child = Read(pReader);
                        tag->AddChild(child);
                    }
                    else
                        tag->SetCData(std::string((char*)check->content));

                }
            }
        }
        return tag;
    }

    void XMLTag::AddProperty(const std::string &name, const std::string &value)
    {
        m_properties[name] = value;
        Touch();
    }

#define MAXINTLONGSIZE 40
    void XMLTag::AddProperty(const std::string &name, const unsigned long &value)
    {
        char longBuf[MAXINTLONGSIZE];
        sprintf_s(longBuf,MAXINTLONGSIZE, "%ld", value);
        m_properties[name] = longBuf;
        Touch();
    }

    void XMLTag::AddProperty(const std::string& name, const long &value)
    {
        char intBuf[MAXINTLONGSIZE];
        sprintf_s(intBuf, MAXINTLONGSIZE, "%ld", value);
        m_properties[name] = intBuf;
        Touch();
    }

#define MAXINTSIZE 20
    void XMLTag::AddProperty(const std::string& name, const int &value)
    {
        char intBuf[MAXINTSIZE];
        sprintf_s(intBuf,MAXINTSIZE, "%d", value);
        m_properties[name] = intBuf;
        Touch();
    }

#define MAXDOUBLESIZE 190
    void XMLTag::AddProperty(const std::string &name, const double &value)
    {
        char intBuf[MAXDOUBLESIZE];
        sprintf_s(intBuf,MAXDOUBLESIZE, "%f", value);
        m_properties[name] = intBuf;
        Touch();
    }

    void XMLTag::SetCData(const double &value)
    {
        char intBuf[MAXDOUBLESIZE];
        sprintf_s(intBuf,MAXDOUBLESIZE, "%f", value);
        m_cData = intBuf;
        Touch();
    }

    int XMLTag::GetCDataInt() const
    {
        return atoi(m_cData.c_str());
    }

    unsigned long XMLTag::GetCDataUlong() const
    {
        return atol(m_cData.c_str());
    }

    double XMLTag::GetCDataDouble() const
    {
        return atof(m_cData.c_str());
    }
    std::string XMLTag::GetCDataST() const
    {
        return m_cData;
    }

    void XMLTag::SetCData(const int &value)
    {
        char intBuf[MAXINTSIZE];
        sprintf_s(intBuf,MAXINTSIZE, "%d", value);
        m_cData = intBuf;
        Touch();
    }

    void XMLTag::SetCData(const unsigned long &value)
    {
        char intBuf[MAXINTSIZE];
        sprintf_s(intBuf,MAXINTSIZE, "%ld", value);
        m_cData = intBuf;
        Touch();
    }

    void XMLTag::SetCData(const long &value)
    {
        char intBuf[MAXINTSIZE];
        sprintf_s(intBuf,MAXINTSIZE, "%ld", value);
        m_cData = intBuf;
        Touch();
    }


    void XMLTag::SetCData(const std::string& value)
    {
        m_cData = value;
        Touch();
    }

    std::string XMLTag::GetProperty(const std::string &name, std::string default_value)
    {
        if(m_properties.find(name) == m_properties.end())
            return default_value;
        return m_properties[name];
    }

    double XMLTag::GetPropertyDouble(const std::string &name , double default_value)
    {
        if(m_properties.find(name) == m_properties.end())
            return default_value;
        return atof(m_properties[name].c_str());
    }

    int XMLTag::GetPropertyInt(const std::string& name, int default_value)
    {
        if(m_properties.find(name) == m_properties.end())
            return default_value;
        return atoi(m_properties[name].c_str());
    }

    int XMLTag::AddChild(XMLTag* child, int position)
    {
        if(child != NULL)
        {
            if(position < 0 || (unsigned)position > m_children.size())
                m_children.push_back(child);
            else
                m_children.insert(m_children.begin() += position, child);
            Touch();
            return (signed int)m_children.size() - 1;
        }
        return -1;
    }

    void XMLTag::ReplaceChild(XMLTag* child, int position)
    {
        if(child != NULL)
        {
            if(position < 0 || (unsigned)position > m_children.size())
                m_children.push_back(child);
            else
            {
                delete m_children[position];
                m_children[position] = child;
            }
        }
        Touch();
    }

    unsigned int XMLTag::CountChildren() const
    {
        return (int)m_children.size();
    }

    void XMLTag::RemoveChild(const unsigned int &position)
    {
        if(position < m_children.size())
        {
            delete m_children[position];
            m_children.erase(m_children.begin() += position);
        }
    }

    void XMLTag::RemoveChild(const std::string &name)
    {
        for(std::vector<XMLTag*>::iterator iter = m_children.begin();
            iter != m_children.end(); iter++)
        {
            if((*iter)->GetName().compare(name) == 0)
            {
                delete *iter;
                m_children.erase(iter);
                break;
            }
        }
    }


    XMLTag* XMLTag::GetChild(const unsigned int &position)
    {
        if(position < m_children.size())
        {
            if(m_children[position]->GetName().compare(XML_NODE_FILE_REFERENCE) == 0)
            {
                XMLTag* child = ReadFromFile(m_children[position]->m_cData);
                delete m_children[position];
                m_children[position] = child;
                m_children[position]->Touch();
            }
            return m_children[position];
        }
        return NULL;
    }

    XMLTag* XMLTag::GetChild(const std::string& name)
    {
        XMLTag* tag = NULL;
        int counter = 0;
        for(std::vector<XMLTag*>::const_iterator iter = m_children.begin();
            iter  != m_children.end(); iter++, counter++)
        {
            if((*iter)->GetName().compare(name) == 0)
            {
                return GetChild(counter);
            }
        }
        return tag;
    }

    XMLTag* XMLTag::GetChild(const std::string& name, int innerindex)
    {
        XMLTag* tag = NULL;
        int counter = 0;
        int innercounter = 0;
        for(std::vector<XMLTag*>::const_iterator iter = m_children.begin();
            iter  != m_children.end(); iter++, counter++ )
        {
            if((*iter)->GetName().compare(name) == 0)
            {
                if(innercounter == innerindex)
                    return GetChild(counter);
                innercounter++;
            }
        }
        return tag;
    }

    unsigned long XMLTag::date()
    {
        return m_lastChanged;
    }
    //Methods
    int XMLTag::getQueryType()
    {
        return 0;
    }

    void XMLTag::Touch()
    {
        m_lastChanged = (unsigned long)time(NULL);
    }

std::vector<std::string> XMLTag::GetSubFilenames()
{
  std::vector<std::string> strings;
  GetSubFilenames(strings);
  return strings;
}


void XMLTag::GetSubFilenames(std::vector<std::string> &strings)
{

  for(std::map<std::string, std::string>::const_iterator iter_pro = m_properties.begin();
      iter_pro != m_properties.end(); iter_pro++)
  {
    if((*iter_pro).first.compare(XML_ATTRIBUTE_FILENAME) == 0)
    {
      strings.push_back((*iter_pro).second);
      printf("Found a filename in %s: %s\n", m_name.c_str(), (*iter_pro).second.c_str());
    }
    else if((*iter_pro).first.find(XML_ATTRIBUTE_FILENAME) != (*iter_pro).first.npos)
    {
      strings.push_back((*iter_pro).second);
      printf("Found a filename in %s: %s\n", m_name.c_str(), (*iter_pro).second.c_str());
    }
    else if((*iter_pro).first.find("path") != (*iter_pro).first.npos)
    {
      strings.push_back((*iter_pro).second);
      printf("Found a filename in %s: %s\n", m_name.c_str(), (*iter_pro).second.c_str());
    }
  }

  if(m_cData.find(".dxf")!= m_cData.npos || m_cData.find(".DXF") != m_cData.npos|| m_cData.find(".Dxf") != m_cData.npos ||
     m_cData.find(".off")!= m_cData.npos)
  {
    strings.push_back(m_cData);
    printf("Found a filename in %s: %s\n", m_name.c_str(), m_cData.c_str());
  }
  
  if(m_name.compare("filename") == 0)
  {
    strings.push_back(m_cData);
    printf("Found a filename in %s: %s\n", m_name.c_str(), m_cData.c_str());
  }


  for(std::vector<XMLTag*>::const_iterator iter_child = m_children.begin();
      iter_child != m_children.end(); iter_child++)
  {
    (*iter_child)->GetSubFilenames(strings);
  }

}



void XMLTag::ReplaceSubFilenames(std::string global_path)
{

  std::map<std::string, std::string>::iterator iter_pro = m_properties.begin();
  for(;
      iter_pro != m_properties.end(); iter_pro++)
  {
   if((*iter_pro).second.length() > 0 &&  (*iter_pro).second[0] != '/')
   {
    if((*iter_pro).first.compare(XML_ATTRIBUTE_FILENAME) == 0)
    {
      (*iter_pro).second = global_path + (*iter_pro).second;
      printf("Found a filename in %s: %s\n", m_name.c_str(), (*iter_pro).second.c_str());
    }
    else if((*iter_pro).first.find(XML_ATTRIBUTE_FILENAME) != (*iter_pro).first.npos)
    {
      (*iter_pro).second = global_path + (*iter_pro).second;
      printf("Found a filename in %s: %s\n", m_name.c_str(), (*iter_pro).second.c_str());
    }
    else if((*iter_pro).first.find("path") != (*iter_pro).first.npos)
    {
      (*iter_pro).second = global_path + (*iter_pro).second;
      printf("Found a filename in %s: %s\n", m_name.c_str(), (*iter_pro).second.c_str());
    }
   }
  }

  if(m_cData.find(".dxf")!= m_cData.npos || m_cData.find(".DXF") != m_cData.npos|| m_cData.find(".Dxf") != m_cData.npos ||
     m_cData.find(".off")!= m_cData.npos  )
  {
    if(m_cData.length() > 0 &&  m_cData[0] != '/')
    { 
      m_cData = global_path + m_cData;
      printf("Found a filename in %s: %s\n", m_name.c_str(), m_cData.c_str());
    }
  }
  
  if(m_name.compare("filename") == 0)
  {
    if(m_cData.length() > 0 &&  m_cData[0] != '/')
    {
      m_cData = global_path + m_cData;
      printf("Found a filename in %s: %s\n", m_name.c_str(), m_cData.c_str());
    }
  }


  for(std::vector<XMLTag*>::const_iterator iter_child = m_children.begin();
      iter_child != m_children.end(); iter_child++)
  {
    (*iter_child)->ReplaceSubFilenames(global_path);
  }

}

