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
                        SignatureDB.cpp - Copyright klank


**************************************************************************/

#include "SignatureDB.h"

#include <time.h>

#include <boost/bind.hpp>
#include <boost/algorithm/string.hpp>


#define XML_NODE_SIGDB_INTERN_DB	"SigDBRoot"
#define XML_NODE_SIGDB_INTERN_INDEX	"SigDBIndex"
#define XML_NODE_SIGDB_INTERN_IDLIST "SigDBIDList"


#define XML_NODE_SIGDB_INTERN_INDEX_CLASS2ID	"SigDBIndexClass2ID"
#define XML_NODE_SIGDB_INTERN_INDEX_ID2CLASS	"SigDBIndexID2Class"

#define XML_NODE_SIGDB_INTERN_INDEX_CLASS2ID_MEM	"idMem"
#define XML_NODE_SIGDB_INTERN_INDEX_ID2CLASS_MEM	"classMem"

#define XML_ATTRIBUTE_SIGDB_CLASS "Class"
#define XML_ATTRIBUTE_SIGDB_ID "ID"



using namespace cop;


class Class2ID : public XMLTag
{
public:
	Class2ID(int idClass, int id) :
		XMLTag(XML_NODE_SIGDB_INTERN_INDEX_CLASS2ID_MEM)
	{
		this->AddProperty(XML_ATTRIBUTE_SIGDB_CLASS, idClass);
		this->AddProperty(XML_ATTRIBUTE_SIGDB_ID, id);
	}
};

// Constructors/Destructors
//

SignatureDB::SignatureDB ( XMLTag* config )
{
	if(config != NULL)
	{
		try
		{
			try
			{
				XMLTag* tagSBRoot = config->GetChild(XML_NODE_SIGDB_INTERN_DB);
				if(tagSBRoot != NULL)
					m_dbStarter = tagSBRoot->Clone();
				else
					m_dbStarter = new XMLTag(XML_NODE_SIGDB_INTERN_DB);
			}
			catch(...)
			{
        printf("Creating Signature DB: Error in node %s\n", XML_NODE_SIGDB_INTERN_DB);
				throw XML_NODE_SIGDB_INTERN_DB;
			}
			try
			{
				XMLTag* tagSBIndex= config->GetChild(XML_NODE_SIGDB_INTERN_INDEX);
				if(tagSBIndex != NULL)
				{
					m_index = tagSBIndex->Clone();
					if(m_index->GetChild(XML_NODE_SIGDB_INTERN_INDEX_CLASS2ID) == NULL)
						m_index->AddChild(new XMLTag(XML_NODE_SIGDB_INTERN_INDEX_CLASS2ID));
				}
				else
				{
					m_index = new XMLTag(XML_NODE_SIGDB_INTERN_INDEX);
					m_index->AddChild(new XMLTag(XML_NODE_SIGDB_INTERN_INDEX_CLASS2ID));
				}
				XMLTag * idx = m_index->GetChild(XML_NODE_SIGDB_INTERN_INDEX_CLASS2ID);
				int size = idx->CountChildren();
				for(int t = 0; t < size;  t++)
				{
				  ObjectID_t idClass = idx->GetChild(t)->GetPropertyInt(XML_ATTRIBUTE_SIGDB_CLASS, 0);
				  ObjectID_t idObject =  idx->GetChild(t)->GetPropertyInt(XML_ATTRIBUTE_SIGDB_ID, 0);
                                  SetClassSignature(idClass, idObject);
				}
			}
			catch(...)
			{
        printf("Creating Signature DB: Error in node %s\n", XML_NODE_SIGDB_INTERN_INDEX);
				throw XML_NODE_SIGDB_INTERN_INDEX;
			}
			try
			{
				XMLTag* tagSBIdList= config->GetChild(XML_NODE_SIGDB_INTERN_IDLIST);
				if(tagSBIdList != NULL)
				{
					m_ids = XMLTag::Load(tagSBIdList, &m_ids);
					if(m_ids.size() != m_dbStarter->CountChildren())
						UpdateIDList();
				}
				else
					UpdateIDList();
			}
			catch(char const* pListError)
			{
				printf("Error reading Singature DB: %s\n", pListError);
				throw XML_NODE_SIGDB_INTERN_IDLIST;
			}
            catch(...)
            {
				printf("Error reading Singature DB.\n");
				throw XML_NODE_SIGDB_INTERN_IDLIST;
            }
		}
		catch(char const* exception)
		{
			printf("Loading of config file failed: %s\n", exception);
			m_dbStarter = new XMLTag(XML_NODE_SIGDB_INTERN_DB);
			m_index = new XMLTag(XML_NODE_SIGDB_INTERN_INDEX);
			m_index->AddChild(new XMLTag(XML_NODE_SIGDB_INTERN_INDEX_CLASS2ID));
		}
	}
	else
	{
		m_dbStarter = new XMLTag(XML_NODE_SIGDB_INTERN_DB);
		m_index = new XMLTag(XML_NODE_SIGDB_INTERN_INDEX);
		m_index->AddChild(new XMLTag(XML_NODE_SIGDB_INTERN_INDEX_CLASS2ID));
	}

}

SignatureDB::~SignatureDB ( )
{
  delete m_dbStarter;
  delete m_index;
  std::map<PerceptionPrimitiveID_t, PerceptionPrimitive*>::iterator iter = m_ppMap.begin();
  for(; iter != m_ppMap.end(); )
  {
    delete (*iter).second;
    m_ppMap.erase(iter);
    iter = m_ppMap.begin();
  }
}

//
// Methods
//
#ifdef BOOST_THREAD

bool read_env = true;
bool showing_on = false;
void SignatureDB::AddAndShowSignatureAsync(Signature* sig, Sensor* sens)
{
  boost::thread(boost::bind(&SignatureDB::AddSignature, this, sig));
#ifdef WIN32
#else
  if(read_env)
  {
    char* value =  getenv ("DISPLAY");
    printf("Read Env Display: %s\n", value);
    if(value && strlen(value) > 0)
      showing_on = true;
  }
  if(showing_on)
    boost::thread(boost::bind(&Signature::Show, sig, sens));
#endif
}


void SignatureDB::AddAndShowSignaturesAsync(std::vector<Results_t> &all_matches, const int &num_results_accepted)
{
  std::vector<Results_t> vec;
  vec.resize(num_results_accepted);
  std::copy(all_matches.begin(), all_matches.begin() + num_results_accepted, vec.begin());
  boost::thread(boost::bind(&SignatureDB::AddSignatures, this, vec));
}

#endif

int SignatureDB::AddSignature(Signature* sig)
{
  int error = -1;
#ifndef WIN32
  sleep(0.01);
#endif
  if(sig != NULL)
  {
    if(!Check(sig->m_ID, error))
    {
        error = m_dbStarter->AddChild(sig->Save());
        m_ids.push_back(sig->m_ID);
        for(size_t objsubsc = 0; objsubsc < m_newObjectSubscriber.size(); objsubsc++)
        {
          m_newObjectSubscriber[objsubsc]->NotifyNewObject(sig, sig->GetObjectPose());
        }

        ObjectID_t idObject = sig->m_ID;
        for(unsigned int classes = 0; classes < sig->CountClasses(); classes++)
        {
          Class* cl = sig->GetClass(classes);
          ObjectID_t idClass = cl->m_ID;
          cl->m_ID = AddClass(cl->GetName(), idClass);
          SetClassSignature(cl->m_ID, idObject);
        }
    }
    else
    {
       UpdateNodes(sig, error);

       for(size_t objsubsc = 0; objsubsc < m_newObjectSubscriber.size(); objsubsc++)
       {
         m_newObjectSubscriber[objsubsc]->NotifyNewObject(sig, sig->GetObjectPose());
       }
    }
    if(error != -1)
    {
      CleanUpActiveSignatureList();
      AddSignatureToActiveList(sig, error);
    }
  }
  return error;
}



void SignatureDB::AddSignatures(std::vector<Results_t> matches)
{
  int error = -1;
  std::vector<Results_t>::iterator it = matches.begin();
  for(;it != matches.end(); it++)
  {
     error = AddSignature((*it).signature);
  }


#ifdef WIN32
#else
  if(read_env)
  {
    char* value =  getenv ("DISPLAY");
    printf("Read Env Display: %s\n", value);
    if(value && strlen(value) > 0)
      showing_on = true;
  }
#endif
  if(showing_on)
  {

    std::vector<Results_t>::iterator it = matches.begin();
    for(;it != matches.end(); it++)
    {
      (*it).signature->Show((*it).camera);
    }
  }
}

void SignatureDB::AddSignatureToActiveList(Signature* sig, int index)
{
  m_currentlyActiveSignatures.push_back(std::pair<Signature*, int>(sig, 1));
  m_activeMap[index] = m_currentlyActiveSignatures.size() - 1;
}


void  SignatureDB::SetClassSignature(ObjectID_t idClass, ObjectID_t idObject)
{
  m_classToSignature[idClass].push_back(idObject);
}


void SignatureDB::UpdateNodes(Signature* sig, int index)
{
  printf("SignatureDB::UpdateNodes\n");
  ObjectID_t idObject = sig->m_ID;
  for(unsigned int classes = 0; classes < sig->CountClasses(); classes++)
  {
    Class* cl = sig->GetClass(classes);
    cl->m_ID = AddClass(cl->GetName(), cl->m_ID);
    SetClassSignature(cl->m_ID, idObject);
  }

  m_dbStarter->ReplaceChild(sig->Save(), index);
}

Signature* SignatureDB::GetSignatureByIndex(unsigned int index)
{
  if(index < m_dbStarter->CountChildren())
  {
    if(m_activeMap.find(index) != m_activeMap.end())
    {
      int index_active = m_activeMap[index];
      m_currentlyActiveSignatures[index_active].second++;
      return m_currentlyActiveSignatures[index_active].first;
    }
    else
    {
     try
     {
       Signature* sig = (Signature*)Elem::ElemFactory(m_dbStarter->GetChild(index));
       AddSignatureToActiveList(sig, index);
       return sig;
     }
     catch(char const* error_text)
     {
         printf("Error loading signature: %s\n", error_text);
         return NULL;
     }
    }
  }
  else
    return NULL;
}

#define TIME_OUT_ACTIVE_SIG 30000

void SignatureDB::CleanUpActiveSignatureList()
{
  std::vector<std::pair<Signature*, int> >::iterator it = m_currentlyActiveSignatures.begin();
  unsigned long timeStamp = (unsigned long)time(NULL);
  int index = 0;
  for(;it != m_currentlyActiveSignatures.end(); it++)
  {
    if((*it).second == 0 && timeStamp - (*it).first->date() > TIME_OUT_ACTIVE_SIG)
    {
      delete (*it).first;
      it = m_currentlyActiveSignatures.erase(it);
      m_activeMap.erase(m_activeMap[index]);
    }
    else
      index++;
  }
}


Signature* SignatureDB::GetSignatureByID(ObjectID_t ElemID)
{
  int index;
  Check(ElemID, index);
  return GetSignatureByIndex(index);
}


int SignatureDB::AddClass(std::string stname, int id)
{
  if(CheckClass(id).length() == 0)
  {
    int id_same_name = CheckClass(stname);
    if(id_same_name == -1)
    {
      printf("Add Class %s -> %d\n", stname.c_str(), id);
      m_classes.push_back(std::pair<std::string, int>(stname, id));
      return id;
    }
    else
    {
      printf("Class %s  already defined %d\n", stname.c_str(), id_same_name);
      return id_same_name;
    }
  }
  else
  {
     printf("Already defined class id %d: %s\n", id, CheckClass(id).c_str());
     if(stname.compare(CheckClass(id)) != 0)
     {
       m_classes.push_back(std::pair<std::string, int>(stname, Elem::m_LastID++));
       return  Elem::m_LastID - 1;
     }
     return id;
  }
}

void SignatureDB::CompleteSignature(Signature* sig_max, std::vector<ObjectID_t> class_ids)
{
  size_t size = class_ids.size();
  if(sig_max != NULL)
  {
    for(unsigned int k = 0 ; k < size; k++)
    {
      bool bIn = false;
      for(unsigned int j = 0; j < sig_max->CountClasses(); j++)
      {
        if(sig_max->GetClass(j) != NULL && class_ids[k] == sig_max->GetClass(j)->m_ID)
        {
          bIn = true;
          break;
        }
      }
      if(!bIn)
      {
        try
        {
          /** Add known Descriptior*/
          Elem* elem = FindCreateDescriptor(class_ids[k]);
          if(elem != NULL)
            sig_max->SetElem(elem);
          else
          {
            printf("No descriptor could be found or created for class %ld (=%s)\n", class_ids[k], CheckClass(class_ids[k]).c_str());
            sig_max->SetClass(GetClassByID(class_ids[k]));
          }
        }
        catch(char const* text)
        {
          printf("Problem while building signature: %s\n", text);
        }
      }
    }
  }
}

Signature* SignatureDB::GetSignature(std::vector<ObjectID_t> class_ids)
{
  size_t size = class_ids.size();
  int score_max = 0, score_temp = 100;
  Signature* sig_max = NULL;
  int index = -1;
  printf("Entering Get Signature\n");
  for(unsigned int i = 0 ; i < size; i++)
  {
    Signature* sig = NULL;
    int offset = 0;
    /** Get a signature for the given class*/
    while((sig = GetSignatureByClass(class_ids[i], offset++)) != NULL)
    {
      /** Check this signature for containing more of the given classes*/
      for(unsigned int j = 0; j < sig->CountClasses(); j++)
      {
        bool bFoundThis = false;
        for(unsigned int k = 0 ; k < size; k++)
        {
          if(k == i)
          {
            score_temp++;
            continue;
          }
          if(sig->GetClass(j) != NULL && class_ids[k] == sig->GetClass(j)->m_ID)
          {
            score_temp++;
            bFoundThis = true;
            break;
          }
        }
        if(!bFoundThis)
        {
          score_temp--;
        }
      }
      /** Keep the best signature*/
      if(score_temp > score_max)
      {
        score_max = score_temp;
        index = i;
        if(sig_max != NULL)
          FreeActiveSignature(sig_max);
        /** Copy the best*/
        sig_max = (Signature*)sig;
      }
      else
        FreeActiveSignature(sig);
    }
    score_temp = 0;
  }
  /** if there was no signature, put all classes to a new sig*/
 if(sig_max != NULL)
    printf("Signature selected: %p: id: %ld\n", sig_max, sig_max->m_ID);

  if(sig_max == NULL && size != 0)
  {
    return NULL;
  }
  CompleteSignature(sig_max, class_ids);
  /** Returning the best*/
  return sig_max;
}

Elem* SignatureDB::FindCreateDescriptor(ObjectID_t class_id)
{
  Elem* result = NULL;
  Signature* sample = GetSignatureByClass(class_id);
  if(sample != NULL)
  {
    int count_cont = 1;
    std::vector<Descriptor*> cands;
    while(true)
    {
      for(size_t i = 0; i < sample->CountElems(); i++)
      {
        Descriptor* descr = (Descriptor*)sample->GetElement(i, ELEM);
        if(descr == NULL)
          continue;
        if(descr->GetClass() == NULL)
         continue;
        if(descr->GetClass()->m_ID == class_id)
        {
          cands.push_back(descr);
        }
      }
      Signature* sample = GetSignatureByClass(class_id, count_cont++);
      if(sample == NULL)
        break;
    }
    if(cands.size() > 0)
    {
      /*TODO introduce metric for different descriptors for a class or add all...*/
      /*std::sort(cands.begin(), cands.end());*/
      result = cands[0];
    }
  }
  else
  {
    std::string searchstring = CheckClass(class_id);
  }
  return result;
}


void SignatureDB::SetNewObjectCallback(Comm* comm, bool wait_for_new)
{
  for(size_t i = 0; i < m_dbStarter->CountChildren(); i++)
  {
    try
    {
     Signature* sig = (Signature*)Elem::ElemFactory(m_dbStarter->GetChild(i));
     if(!sig)
       continue;
     comm->NotifyNewObject(sig, sig->GetObjectPose());
     delete sig;
    }
    catch(...)
    {
      continue;
    }
  }
  if(wait_for_new)
    m_newObjectSubscriber.push_back(comm);
}

void SignatureDB::FreeActiveSignature(Signature* sig)
{
  std::vector<std::pair<Signature*, int> >::iterator it = m_currentlyActiveSignatures.begin();
  //unsigned long timeStamp = (unsigned long)time(NULL);
  /*TODO, decide whats active*/
  for(;it != m_currentlyActiveSignatures.end(); it++)
  {
    if(sig == (*it).first)
      (*it).second--;
  }
}


Class* SignatureDB::GetClassByID(ObjectID_t id)
{
  std::string st = CheckClass(id);
  if(st.length() == 0)
    throw "Unknown Class";
  return new Class(st, id);
}


ObjectID_t SignatureDB::CheckClass(std::string name)
{
	std::vector<std::pair<std::string, ObjectID_t> > ::const_iterator it;
	for(it = m_classes.begin(); it != m_classes.end(); it++)
	{
		/*cerr<<(*it).first<<endl;*/
		if(boost::iequals((*it).first,name))
			return (*it).second;

	}
	return -1;
}

std::string SignatureDB::CheckClass(ObjectID_t id_c)
{
	std::vector<std::pair<std::string,ObjectID_t> > ::const_iterator it;
	for(it = m_classes.begin(); it != m_classes.end(); it++)
	{
		if((*it).second == id_c)
			return (*it).first;
	}
	return "";
}

int SignatureDB::GetElemIdByClass(ObjectID_t ClassID, int index)
{
	ObjectID_t id  = -1;
	std::map<ObjectID_t, std::vector<ObjectID_t> >::iterator idlist = m_classToSignature.find(ClassID);
	if(idlist != m_classToSignature.end())
	{
	  if(index < (signed)(*idlist).second.size())
	  {
	    id = (*idlist).second[index];
	  }
        }
        else
        {
          printf("Class id not in Map\n");
        }
	return id;
}

Signature* SignatureDB::GetSignatureByClass(ObjectID_t ClassID, int index)
{
	ObjectID_t id = GetElemIdByClass(ClassID, index);
	if(id == -1)
		return NULL;
	return GetSignatureByID(id);
}

XMLTag* SignatureDB::Save()
{
	XMLTag* tag = new XMLTag(XML_NODE_SIGNATUREDB);
	tag->AddChild(m_dbStarter->Clone());
	XMLTag* index = new XMLTag(m_index->GetName());
	std::map<ObjectID_t, std::vector<ObjectID_t> >::iterator it =  m_classToSignature.begin();
	for(; it != m_classToSignature.end(); it++)
	{
	  for(size_t i = 0; i < (*it).second.size(); i++)
	  {
	    index->AddChild(  new Class2ID((*it).first, (*it).second[i])  );
	  }
	}
	tag->AddChild(index);
	return tag;
}
// Accessor methods
//
XMLTag* SignatureDB::Query(std::string stQueryString)
{
	//TODO:
	int id = atoi(stQueryString.c_str());
	int index;
	XMLTag* tag = new XMLTag(XML_NODE_SIGNATURE_VEC);
	if(Check(id, index))
	{
		tag->AddChild(m_dbStarter->GetChild(index));
	}
	else
	{
		int classID = CheckClass(stQueryString);
		if(classID == -1)
			if(CheckClass(id).length() != 0)
				classID = id;
		int count = 0;
		int id_elem = -1;
		do
		{
			id_elem = GetElemIdByClass(classID, count);
			if(Check(id_elem, index))
			{
				tag->AddChild(m_dbStarter->GetChild(index));
				count++;
			}
		}
		while(id_elem != -1);
	}
	return tag;
}

bool SignatureDB::Check(ObjectID_t sigID, int& index) const
{
	size_t children = m_ids.size();
	for(unsigned int i = 0; i < children; i++)
	{
		if(m_ids[i] == sigID)
		{
			index = i;
			return true;
		}
	}
	index = -1;
	return false;
}

void SignatureDB::UpdateIDList()
{
  int children = m_dbStarter->CountChildren();
  m_ids.clear();
  for(int i = 0; i < children; i++)
  {
    Signature* sig = GetSignatureByIndex(i);
    if(sig != NULL)
    {
      m_ids.push_back(sig->m_ID);
      ObjectID_t id = sig->m_ID;
      for(unsigned int classes = 0; classes < sig->CountClasses(); classes++)
      {
        Class* cl = sig->GetClass(classes);
        SetClassSignature(cl->m_ID, id);
        AddClass(cl->GetName(), cl->m_ID);
      }
      FreeActiveSignature(sig);
    }
  }
}

PerceptionPrimitive& SignatureDB::CreateNewPerceptionPrimitive(Signature* sig)
{
  PerceptionPrimitive* pp = new PerceptionPrimitive(sig);
  m_ppMap[pp->GetID()] = pp;
  return *pp;
}

void SignatureDB::EvaluatePerceptionPrimitive(PerceptionPrimitiveID_t id, double value, double weight, std::vector<ObjectID_t> whitelist)
{

  if(m_ppMap.find(id) != m_ppMap.end())
  {
    PerceptionPrimitive *pp =  m_ppMap[id];
    pp->SetEvaluating();
    for(size_t i = 0; i < pp->m_signatures.size(); i++)
    {
       for(size_t j = 0; j < pp->m_signatures[i]->CountElems(); j++)
       {
         Elem* elem = pp->m_signatures[i]->GetElement(j, ELEM);
         elem->Evaluate(value, weight);
         if(weight > PROP_DECAY)
         {
           PerceptionPrimitiveID_t id_inner = elem->GetLastPerceptionPrimitive();
           printf("Elem %s is connected with PP %ld \n", elem->GetNodeName().c_str() ,id_inner);
           std::vector<ObjectID_t> whitelist_inner;
           whitelist_inner.push_back(elem->m_ID);
           if(id_inner != id)
             EvaluatePerceptionPrimitive(id_inner, value, weight / 2, whitelist_inner);
         }
       }
       printf("This pp has %ld algs\n" , pp->m_AlgorithmIDs.size());
       for(size_t alg = 0; alg < pp->m_AlgorithmIDs.size(); alg++)
       {
          pp->m_AlgorithmIDs[alg].second->EvalAlgorithm(pp->m_AlgorithmIDs[alg].first, value, pp->m_timing, pp->m_signatures[i]);
       }

    }
    for(size_t i = 0; i < pp->m_results.size(); i++)
    {
      int index;
      if(CheckClass(pp->m_results[i]).length() == 0 && Check(pp->m_results[i], index))
      {
        if(whitelist.size() != 0 && !find_in_vec(whitelist, (pp->m_results[i])))
        {
          printf("Skip %ld in Eval due to white list\n", pp->m_results[i]);
          continue;
        }
        Signature* sig = GetSignatureByID(pp->m_results[i]);
        std::vector<ObjectID_t> whitelist_inner;
        whitelist_inner.push_back(sig->m_ID);
        for(size_t j = 0; j < sig->CountElems(); j++)
        {
          Elem* elem = sig->GetElement(j, ELEM);
          elem->Evaluate(value, weight);
          if(weight > PROP_DECAY)
          {
            PerceptionPrimitiveID_t id_inner = elem->GetLastPerceptionPrimitive();
            printf("Elem %s is connected with PP %ld \n", elem->GetNodeName().c_str() ,id_inner);
            if(id_inner != id)
                EvaluatePerceptionPrimitive(id_inner, value, weight / 2, whitelist_inner);
          }
        }
        for(size_t i = 0; i < pp->m_AlgorithmIDs.size(); i++)
        {
          pp->m_AlgorithmIDs[i].second->EvalAlgorithm(pp->m_AlgorithmIDs[i].first, value, pp->m_timing, sig);
        }
      }
    }
    pp->SetEvaluated(); /** TODO Check if the PP should continue existing.*/
  }

  std::map<PerceptionPrimitiveID_t, PerceptionPrimitive*>::iterator iter = m_ppMap.begin();
  for(; iter != m_ppMap.end(); iter++)
  {
    if((*iter).second->GetCurrState() == PP_DELETABLE || ((*iter).second->GetCurrState() == PP_TERMINATED && (*iter).second->m_startTime < time(NULL) - 600))
    {
      if(((*iter).second->GetCurrState() == PP_TERMINATED && (*iter).second->m_startTime < time(NULL) - 600))
        printf("delete perception primitive for timing %ld < %ld + 600 = %ld\n", (*iter).second->m_startTime, time(NULL), time(NULL) - 600);
      delete (*iter).second;
      m_ppMap.erase(iter);
      iter = m_ppMap.begin();
      if(iter == m_ppMap.end())
        break;
	  }
  }
}




std::vector<std::pair <PerceptionPrimitiveID_t, PerceptionPrimitiveState> > SignatureDB::GetCurrentRunState()
{
  std::vector<std::pair <PerceptionPrimitiveID_t, PerceptionPrimitiveState> > ret;
  std::map<PerceptionPrimitiveID_t, PerceptionPrimitive*>::iterator iter = m_ppMap.begin();
  for(; iter != m_ppMap.end(); iter++)
  {
	  ret.push_back(std::pair <PerceptionPrimitiveID_t, PerceptionPrimitiveState>((*iter).first, (*iter).second->GetCurrState()));

  }
  return ret;
}

