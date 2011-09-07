/**********************************************************************************************/
/**
*              Located Object
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

#include "lo/ObjectContainer.h"
#include "lo/ServiceInterface.h"

/************************************************************************
*                    XMLTag.cpp - Copyright klank
*
*        copied from cop
**************************************************************************/

#include "libxml/xmlwriter.h"
#include "libxml/xmlreader.h"
#include <time.h>
// Constructors/Destructors
//
#define MY_ENCODING "ISO-8859-1"

#define XML_NODE_FILE_REFERENCE "FileReference"
#define XML_ATTRIBUTE_TIMESTAMP "TimeStamp"

#define MAX_TREE_HEIGHT 150

#define XML_PROPERTY_NUM "Num"
#define XML_NODE_LO_TYPE "LoType"
#define XML_NODE_LOLIST "LoList"
#define XML_NODE_LO "LO"
#define XML_ATTRIBUTE_NAMEMAPPING "NameIDMapping"

#ifndef WIN32
#define  sprintf_s(_DstBuf, size, _Format, whatever) sprintf(_DstBuf, _Format, whatever)
#endif

using namespace jlo;

XMLTag::XMLTag ( std::string name) :
	m_name(name)
{
	Touch();
}

XMLTag* XMLTag::Tag(int n, std::string name)
{
	XMLTag* tag = new XMLTag(name.compare("") == 0 ? XML_NODE_INT : name);
	tag->SetCData(n);
	return tag;
}

XMLTag* XMLTag::Tag(double d, std::string name)
{
	XMLTag* tag = new XMLTag(name.compare("") == 0 ? XML_NODE_DOUBLE : name);
	tag->SetCData(d);
	return tag;
}

XMLTag* XMLTag::Tag(std::string value, std::string name)
{
	XMLTag* tag = new XMLTag(name.compare("") == 0 ? XML_NODE_STRING : name);
	tag->SetCData(value);
	return tag;
}


XMLTag* XMLTag::Tag(const Matrix& mat, std::string name)
{
	XMLTag* tag = new XMLTag(name.compare("") == 0 ? XML_NODE_MATRIX : name);
	int rows = mat.nrows();
	int cols = mat.ncols();
	tag->AddProperty(XML_ATTRIBUTE_ROWS, rows);
	tag->AddProperty(XML_ATTRIBUTE_COLS, cols);
	for(int r = 0; r < rows; r++)
	{
		for(int c = 0; c < cols; c++)
		{
			tag->AddChild(Tag(mat.element(r,c)));
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
    char* XMLTag::WriteToString() const
    {
        xmlOutputBufferPtr xmlbuf = xmlAllocOutputBuffer(NULL);
        xmlTextWriter* pWriter = xmlNewTextWriterMemory(xmlbuf->buffer, 0);
		int nError = 0;/* xmlTextWriterStartDocument(pWriter, NULL, MY_ENCODING, NULL);*/
		if(nError >= 0 )

		{
			//Write File
			Write(pWriter);
			//CloseFile
			nError = xmlTextWriterEndDocument(pWriter);
		}
		xmlFreeTextWriter(pWriter);
		pWriter = NULL;
        /*TODO: solve memory leaking here ...*/
        return (char*)xmlbuf->buffer->content;
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
			int i = xmlGetCompressMode();

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
		sprintf_s(longBuf,MAXINTLONGSIZE, "%d", value);
		m_properties[name] = longBuf;
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

	void XMLTag::SetCData(const std::string& value)
	{
		m_cData = value;
		Touch();
	}

	std::string XMLTag::GetProperty(const std::string &name)
	{
		if(m_properties.find(name) == m_properties.end())
			return "";
		return m_properties[name];
	}

	double XMLTag::GetPropertyDouble(const std::string &name)
	{
		if(m_properties.find(name) == m_properties.end())
			return 0.0;
		return atof(m_properties[name].c_str());
	}

	int XMLTag::GetPropertyInt(const std::string& name)
	{
		if(m_properties.find(name) == m_properties.end())
			return 0;
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
			return (signed int)m_children.size() - 1;
		}
		Touch();
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

	void XMLTag::Touch()
	{
		m_lastChanged = (unsigned long)time(NULL);
	}

  jlo::LocatedObjectLoader s_loadLocatedObjectParent;
  /**
  * @brief the list of lo
  */
/*  std::vector<ServiceLocatedObject*> s_ServiceLocatedObjects;*/
  std::map<unsigned long, ServiceLocatedObject*> s_ServiceLocatedObjectMap;
  std::map<std::string, unsigned long> s_ServiceLocatedObjectNameMap;

unsigned long ServiceInterface::SetServiceLocatedObject(jlo::ServiceLocatedObject* pose)
{
  /*s_ServiceLocatedObjects.push_back(pose);*/
  s_ServiceLocatedObjectMap[pose->m_uniqueID] = pose;
  jlo::ServiceLocatedObject::SetLastID(pose->m_uniqueID);
  return (unsigned long)pose->m_uniqueID;
}

void jlo::ServiceLocatedObject::UpdateParent(jlo::ServiceLocatedObject* new_parent)
{

  ServiceLocatedObject* old_parent = ServiceInterface::GetServiceLocatedObject(m_parentID);
  old_parent->RemoveAttachedObject(this);
  m_parentID = new_parent->m_uniqueID;
  new_parent->AddAttachedObject(this);
}


void jlo::ServiceLocatedObject::Update(Matrix m, Matrix cov, ServiceLocatedObject*(* copy)(ServiceLocatedObject*, ServiceLocatedObject*), unsigned long (*del)(ServiceLocatedObject*), void (*updated)(unsigned long))
{
  m_relation->TellParentNeedNoCopy();
  PropagateMovement(copy, del, updated, NULL);
  Set(m, cov);
}


XMLTag* jlo::ServiceLocatedObject::SaveComplete()
{
  XMLTag* ret = new XMLTag(XML_NODE_LO);
  ret->AddProperty(XML_NODE_LO_TYPE, GetLOType());
  if(m_uniqueID > (m_relation == NULL ? 0 : m_relation->m_uniqueID))
    ret->AddProperty(XML_ATTRIBUTE_LOID, m_uniqueID);
  else
  {
    printf("Not saving temporary objects, that were relocated (%d)\n", m_uniqueID);
    delete ret;
    return NULL;
  }
  if(m_uniqueID != ID_WORLD)
    ret->AddProperty(XML_ATTRIBUTE_LOIDFATHER, m_relation == NULL ? ID_WORLD : m_relation->m_uniqueID);
  ret->AddChild(XMLTag::Tag(GetMatrix(0), XML_NODE_MATRIX));
  ret->AddChild(XMLTag::Tag(GetCovarianceMatrix(0), XML_NODE_COVARIANCE));
  ret->AddProperty(XML_ATTRIBUTE_NAMEMAPPING, m_mapstring);
  return ret;
}

/**
 * @return LocatedObject
 * @param  bigObject
 */
jlo::ServiceLocatedObject ServiceLocatedObject::operator- (jlo::ServiceLocatedObject &smallObj )
{
  unsigned long id = smallObj.FindCommonFather(this);
  Matrix m, n;
  Matrix mCov, nCov;
  m = smallObj.GetMatrix(id);
  mCov = smallObj.GetCovarianceMatrix(id);
  n = GetInvMatrix(id);
  nCov = GetCovarianceMatrix(id);
  Matrix temp = n * m;
  return jlo::ServiceLocatedObject(this, temp, UnscentedTrans(nCov+mCov, temp, n));
}




unsigned long ServiceInterface::FreeServiceLocatedObject(jlo::ServiceLocatedObject* pose)
{
  if(pose == NULL)
    return 0;
  unsigned long id = pose->m_uniqueID;
  if(id != ID_WORLD)
  {
    pose->DecreaseReferenceCounter();
    if(pose->GetReferenceCounter() <= 0)
    {
      if(pose->m_mapstring.length() >  0)
      {
         ServiceInterface::RemoveMapString(pose->m_mapstring);
      }
      /**
        free also the parent, if and only if they are movement artefacts
      **/
      jlo::ServiceLocatedObject* parent = GetServiceLocatedObject(pose->m_parentID);
      if(parent != NULL)
      {
        parent->RemoveAttachedObject(pose);
      }
      delete pose;
      s_ServiceLocatedObjectMap.erase(s_ServiceLocatedObjectMap.find(id));
      pose = NULL;
      if(parent != NULL)
      {
        if(parent->GetReferenceCounter() <= 0)
        {
          FreeServiceLocatedObject(parent);
        }
        else
        {
           /*printf("Not deleting a parent of %ld cause it is still in use: %ld\n", id, parent->m_uniqueID);*/
        }
      }
    }
  }
  return id;
}


void ServiceInterface::DisposeList()
{
	for(std::map<unsigned long, jlo::ServiceLocatedObject*>::iterator iter = s_ServiceLocatedObjectMap.begin();
		iter != s_ServiceLocatedObjectMap.end(); iter++)
	{
		delete (*iter).second;
	}
	s_ServiceLocatedObjectMap.clear();

}


XMLTag*	ServiceInterface::SaveList()
{
  XMLTag* tag = new XMLTag(XML_NODE_LOLIST);
  unsigned long counter = 0;
  /*tag->AddProperty(XML_PROPERTY_NUM, (int)s_ServiceLocatedObjects.size());*/
  for(std::map<unsigned long, jlo::ServiceLocatedObject*>::iterator iter = s_ServiceLocatedObjectMap.begin();
          iter != s_ServiceLocatedObjectMap.end(); iter++)
  {
    if((*iter).second == NULL)
    {
      printf("NULL entry in lo list\n");
      continue;
    }
    XMLTag* child = (*iter).second->SaveComplete();
    if(child != NULL)
    {
      tag->AddChild(child);
      counter++;
    }
  }
  tag->AddProperty(XML_PROPERTY_NUM, (int)counter);
  return tag;
}


ServiceInterface::ServiceInterface(const char* configfile)
{
  XMLTag* tag =  XMLTag::ReadFromFile(configfile);
  if(tag != NULL && tag->GetName().compare(XML_NODE_LOLIST) == 0)
    LoadList(tag);
  else
    FServiceLocatedObjectWorld();
}
ServiceInterface::~ServiceInterface()
{
  DisposeList();
}

jlo::ServiceLocatedObject* ServiceInterface ::FServiceLocatedObjectWorld()
{
  ServiceLocatedObject* pose = GetServiceLocatedObject(ID_WORLD);
  if(pose == NULL)
  {
    pose = new ServiceLocatedObject();
    SetServiceLocatedObject(pose);
  }
  return pose;
}


ServiceLocatedObject* ServiceInterface::FServiceLocatedObject(XMLTag* tag)
{
  if(tag != NULL && tag->GetName().compare(XML_NODE_LO) == 0)
  {
    unsigned long id = tag->GetPropertyInt(XML_ATTRIBUTE_LOID);
    if(id == ID_WORLD)
    {
      ServiceLocatedObject* world = GetServiceLocatedObject(id);
      world = ((world == NULL) ? FServiceLocatedObjectWorld() : world);
      world->m_mapstring = tag->GetProperty(XML_ATTRIBUTE_NAMEMAPPING);
      if(world->m_mapstring.length() != 0)
        s_ServiceLocatedObjectNameMap[world->m_mapstring] = ID_WORLD;

      return world;
    }
    ServiceLocatedObject* dingens = GetServiceLocatedObject(id) ;
    if(dingens != NULL)
      return dingens;
    //throw "Trying to load already existing LocatedObject";
    unsigned long father = tag->GetPropertyInt(XML_ATTRIBUTE_LOIDFATHER);
    ServiceLocatedObject* fatherRP = GetServiceLocatedObject(father);
    if(fatherRP == NULL)
    {
      if(father == ID_WORLD)
        fatherRP = FServiceLocatedObjectWorld();
      else
      {
        printf("ServiceLocatedObject: %d with unknown father %d\n", id, father);
        return NULL;
      }
    }
    Matrix m, cov;
    m = tag->Load(tag->GetChild(XML_NODE_MATRIX), &m);
    cov = tag->Load(tag->GetChild(XML_NODE_COVARIANCE), &cov);
    unsigned long type = tag->GetPropertyInt(XML_NODE_LO_TYPE);
    if(id > jlo::ServiceLocatedObject::s_lastID)
      jlo::ServiceLocatedObject::SetLastID(id-1);
    ServiceLocatedObject* pose = FServiceLocatedObject(fatherRP, m, cov, type);
    AddMapString(pose, tag->GetProperty(XML_ATTRIBUTE_NAMEMAPPING));
    return pose;
  }
  //TOCHECK return world?
  return FServiceLocatedObjectWorld();
}

void ServiceInterface::AddMapString(ServiceLocatedObject* pose, std::string mapstring)
{
    pose->m_mapstring = mapstring;
    if(pose->m_mapstring.length() != 0)
      s_ServiceLocatedObjectNameMap[pose->m_mapstring] = pose->m_uniqueID;
}

void ServiceInterface::RemoveMapString(std::string mapstring)
{
   s_ServiceLocatedObjectNameMap[mapstring] = 0;
}

/***
  Copy is always an Perceived Object
*/
ServiceLocatedObject* ServiceInterface::FServiceLocatedObjectCopy(ServiceLocatedObject* obj_to_copy, ServiceLocatedObject* parent_copy)
{
  ServiceLocatedObject* ret = NULL;
  if(parent_copy == NULL)
    ret = FServiceLocatedObject(obj_to_copy->m_relation, obj_to_copy->GetMatrix(), obj_to_copy->GetCovarianceMatrix(), obj_to_copy->GetLOType());
  else
    ret = FServiceLocatedObject(parent_copy, obj_to_copy->GetMatrix(), obj_to_copy->GetCovarianceMatrix(), obj_to_copy->GetLOType());
  return ret;
}


unsigned long jlo::ServiceLocatedObject::s_lastID = ID_WORLD;
// Constructors/Destructors
//

jlo::ServiceLocatedObject::ServiceLocatedObject ( ) :
    LocatedObject(&s_loadLocatedObjectParent),
    m_relation(NULL),
    m_needCopy(0),
    referenceCounter(0)
{
  s_lastID = s_lastID > ID_WORLD ? s_lastID : ID_WORLD + 1 ;

  //set the matrix to identity in R.
}

jlo::ServiceLocatedObject::ServiceLocatedObject (jlo::ServiceLocatedObject* locatedObject, double x , double y , double z , double roll , double pitch , double yaw ,
                          double sigmaX, double sigmaY , double sigmaZ,
                          double sigmaRoll, double sigmaPitch , double sigmaYaw) :
    LocatedObject(&s_loadLocatedObjectParent, s_lastID, locatedObject->m_uniqueID, x,y,z,roll,pitch,yaw,sigmaX, sigmaY, sigmaZ,sigmaRoll,sigmaPitch, sigmaYaw),
    m_relation(locatedObject),
    m_needCopy(0),
    referenceCounter(0)
{
  s_lastID++;
}


jlo::ServiceLocatedObject::ServiceLocatedObject (jlo::ServiceLocatedObject* locatedObject, const Matrix &matrix , const Matrix &covariance) :
    LocatedObject(&s_loadLocatedObjectParent, s_lastID, locatedObject->m_uniqueID, matrix, covariance),
    m_relation(locatedObject),
    m_needCopy(0),
    referenceCounter(0)
{
        //Check for circular references
        if(!CheckWorldCoordinates(MAX_TREE_HEIGHT))
        {
                m_relation = NULL; //This relation will not be accepted
                m_parentID = 1;
        }
        s_lastID++;
}

jlo::ServiceLocatedObject::~ServiceLocatedObject ( )
{
  //if(m_relation != NULL)
  // {
  //    m_relation->RemoveAttachedObject(this);
  // }
}



ServiceLocatedObject* ServiceInterface::FServiceLocatedObject(ServiceLocatedObject* pose, Matrix m, Matrix cov, unsigned long type)
{
  ServiceLocatedObject* ret = NULL;
  if(type == LO_TYPE_PHYSICAL)
    ret = new ObjectContainer(pose, m, cov);
  else
    ret = new ServiceLocatedObject(pose, m, cov);

  SetServiceLocatedObject(ret);
  pose->AddAttachedObject(ret);
  return ret;
}


void ServiceInterface::LoadList(XMLTag* tag)
{
  if(tag != NULL)
  {
    int num = tag->GetPropertyInt(XML_PROPERTY_NUM);
    for(int i = 0; i< num; i++)
    {
      FServiceLocatedObject(tag->GetChild(i));
      /*TODO build map here*/
    }
  }
}

ServiceLocatedObject* ServiceInterface::GetServiceLocatedObject(unsigned long id)
{
  if(id == 0)
    return NULL;
   std::map<unsigned long, jlo::ServiceLocatedObject*>::iterator it = s_ServiceLocatedObjectMap.find(id);
  if(it != s_ServiceLocatedObjectMap.end())
    return (*it).second;
  return NULL;
  /*
  for(std::vector<ServiceLocatedObject*>::iterator iter = s_ServiceLocatedObjects.begin();
		iter != s_ServiceLocatedObjects.end(); iter++)
	{
		if((*iter)->m_uniqueID == id)
			return (*iter);
	}
	return NULL;
	*/
}


unsigned long ServiceInterface::GetServiceLocatedObjectID(std::string st)
{
  std::map<std::string, unsigned long>::iterator it = s_ServiceLocatedObjectNameMap.find(st);
  if(it == s_ServiceLocatedObjectNameMap.end())
  {
    return 0;
  }
  return (*it).second;
}


ServiceLocatedObject* ServiceInterface::GetServiceLocatedObjectIndex(unsigned long index)
{
  printf("Deprectaded call\n");
  return NULL;
  /*return s_ServiceLocatedObjects[index];*/
}


