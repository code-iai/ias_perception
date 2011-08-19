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


#include "RelPoseFactory.h"
#include "XMLTag.h"
#define XML_PROPERTY_NUM "Num"

using namespace cop;

#ifdef NO_LO_SERVICE_AVAILABLE
std::vector<RelPose*> RelPoseFactory::s_relPoses;
#else  /*NO_LO_SERVICE_AVAILABLE*/
Comm* RelPoseFactory::s_loService = NULL;
#endif /*NO_LO_SERVICE_AVAILABLE*/

RelPoseFactory::RelPoseFactory(void)
{
}

RelPoseFactory::~RelPoseFactory(void)
{
}

void RelPoseFactory::DisposeList()
{
#ifdef NO_LO_SERVICE_AVAILABLE
  for(std::vector<RelPose*>::iterator iter = s_relPoses.begin();
    iter != s_relPoses.end(); iter++)
  {
    delete (*iter);
  }
  s_relPoses.clear();
#endif /*NO_LO_SERVICE_AVAILABLE*/
}

#ifdef NO_LO_SERVICE_AVAILABLE
XMLTag*	RelPoseFactory::SaveList()
{
  XMLTag* tag = new XMLTag(XML_NODE_RELPOSELIST);
  tag->AddProperty(XML_PROPERTY_NUM, (int)s_relPoses.size());
  for(std::vector<RelPose*>::iterator iter = s_relPoses.begin();
    iter != s_relPoses.end(); iter++)
  {
    tag->AddChild((*iter)->Save());
  }
  return tag;
}

void RelPoseFactory::LoadList(XMLTag* tag)
{
  if(tag != NULL)
  {
    int num = tag->GetPropertyInt(XML_PROPERTY_NUM);
    for(int i = 0; i< num; i++)
    {
      FRelPose(tag->GetChild(i));
    }
  }
}
#else
#endif /*NO_LO_SERVICE_AVAILABLE*/


RelPose* RelPoseFactory::FRelPoseWorld()
{
  RelPose* pose = GetRelPose(ID_WORLD);
#ifdef NO_LO_SERVICE_AVAILABLE
  if(pose == NULL)
    return GetRelPoseIndex(SetRelPose(new RelPose()));
  else
    return pose;
#else
  return pose;
#endif /*NO_LO_SERVICE_AVAILABLE*/
}

RelPose* RelPoseFactory::FRelPose(XMLTag* tag)
{
  if(tag != NULL && tag->GetName().compare(XML_NODE_RELPOSE) == 0)
  {
    int id = tag->GetPropertyInt(XML_ATTRIBUTE_LOID);
    if(id == 0)
    {
      std::string name = tag->GetProperty(XML_ATTRIBUTE_LOID);
      return GetRelPose(name);
    }
    if(id == ID_WORLD)
    {
      RelPose* world = GetRelPose(id);

      return world == NULL ? FRelPoseWorld() : world;
    }
    RelPose* dingens = GetRelPose(id) ;
    if(dingens != NULL)
      return dingens;
    printf("Trying to load non existing LO!\n");
    return NULL;
  }
  else
   return NULL;
}

inline RelPose* RelPoseFactory::GetRelPose(LocatedObjectID_t id)
{
#ifdef NO_LO_SERVICE_AVAILABLE
  for(std::vector<RelPose*>::iterator iter = s_relPoses.begin();
    iter != s_relPoses.end(); iter++)
  {
    if((*iter)->m_uniqueID == id)
      return (*iter);
  }
  return NULL;
#else /*NO_LO_SERVICE_AVAILABLE*/
  return s_loService->GetPose(id);
#endif /*NO_LO_SERVICE_AVAILABLE*/
}

RelPose* RelPoseFactory::GetRelPose(LocatedObjectID_t poseId, LocatedObjectID_t parentPoseId)
{
#ifdef NO_LO_SERVICE_AVAILABLE
  throw "Not yet implemented";
#else /*NO_LO_SERVICE_AVAILABLE*/
  if(poseId != parentPoseId)
    return s_loService->GetPoseRelative(poseId, parentPoseId);
  else
    return s_loService->GetPose(poseId);
#endif /*NO_LO_SERVICE_AVAILABLE*/
}


RelPose* RelPoseFactory::GetRelPose(std::string name)
{
#ifdef NO_LO_SERVICE_AVAILABLE
  throw "Not yet implemented";
#else /*NO_LO_SERVICE_AVAILABLE*/
    return s_loService->GetPose(name);
#endif /*NO_LO_SERVICE_AVAILABLE*/
}

#ifdef NO_LO_SERVICE_AVAILABLE
inline RelPose* RelPoseFactory::GetRelPoseIndex(LocatedObjectID_t index)
{
  return s_relPoses[index];
}
#endif /*NO_LO_SERVICE_AVAILABLE*/


#ifdef NO_LO_SERVICE_AVAILABLE
inline int RelPoseFactory::SetRelPose(RelPose* pose)
{
  s_relPoses.push_back(pose);
  jlo::LocatedObject::SetLastID(pose->m_uniqueID);
  return (int)s_relPoses.size() - 1;
}
#endif /*NO_LO_SERVICE_AVAILABLE*/

RelPose* RelPoseFactory::FRelPose(jlo::LocatedObject& pose)
{
#ifdef NO_LO_SERVICE_AVAILABLE
  RelPose* relpose = GetRelPose(pose.m_uniqueID);
  if(relpose == NULL)
  {
    relpose = GetRelPoseIndex(SetRelPose(new RelPose(pose)));
  }
  return relpose;
#else /*NO_LO_SERVICE_AVAILABLE*/
  return new RelPose(pose);
#endif /*NO_LO_SERVICE_AVAILABLE*/
}

RelPose* RelPoseFactory::FRelPoseIdentityChild(RelPose* parent)
{
#ifdef NO_LO_SERVICE_AVAILABLE
  throw "Not yet implemented: FRelPoseIdentityChild";
#else /*NO_LO_SERVICE_AVAILABLE*/
  IdentityMatrix m(4);
  Matrix cov(6,6);
  cov << 0.0<< 0.0<< 0.0<< 0.0<< 0.0<< 0.0<<
         0.0<< 0.0<< 0.0<< 0.0<< 0.0<< 0.0<<
         0.0<< 0.0<< 0.0<< 0.0<< 0.0<< 0.0<<
         0.0<< 0.0<< 0.0<< 0.0<< 0.0<< 0.0<<
         0.0<< 0.0<< 0.0<< 0.0<< 0.0<< 0.0<<
         0.0<< 0.0<< 0.0<< 0.0<< 0.0<< 0.0;
  return RelPoseFactory::FRelPose(parent, m, cov);
#endif /*NO_LO_SERVICE_AVAILABLE*/
}


RelPose* RelPoseFactory::CloneRelPose(RelPose* pose)
{
#ifdef NO_LO_SERVICE_AVAILABLE
  if(pose->m_uniqueID == ID_WORLD)
    return pose;
  RelPose* relpose = new RelPose(pose->m_relation, pose->GetMatrix(), pose->GetCovariance());
  relpose = GetRelPoseIndex(SetRelPose(relpose));
  return relpose;
#else /*NO_LO_SERVICE_AVAILABLE*/
  if(pose == NULL)
    throw "Invalid pose to clone!";
  return new RelPose(*pose);
#endif /*NO_LO_SERVICE_AVAILABLE*/
}

RelPose* RelPoseFactory::CloneRelPose(LocatedObjectID_t uniqueID)
{
#ifdef NO_LO_SERVICE_AVAILABLE
  RelPose* pose = GetRelPose(uniqueID);

  if(pose != NULL)
  {
    try
    {
      return CloneRelPose(pose);
    }
    catch(...)
    {
      pose = NULL;
    }
  }
  return pose;
#else /*NO_LO_SERVICE_AVAILABLE*/
  return GetRelPose(uniqueID);
#endif /*NO_LO_SERVICE_AVAILABLE*/
}

RelPose* RelPoseFactory::FRelPose(RelPose* pose, Matrix m, Matrix cov)
{
#ifdef NO_LO_SERVICE_AVAILABLE
  return GetRelPoseIndex(SetRelPose(new RelPose(pose, m, cov)));
#else /*NO_LO_SERVICE_AVAILABLE*/
  return s_loService->CreateNewPose(pose, &m, &cov);
#endif /*NO_LO_SERVICE_AVAILABLE*/
}


RelPose* RelPoseFactory::FRelPose(LocatedObjectID_t parent, Matrix m, Matrix cov)
{
  #ifdef NO_LO_SERVICE_AVAILABLE
  throw "Not yet implemeted";
#else /*NO_LO_SERVICE_AVAILABLE*/
  return s_loService->CreateNewPose(parent, &m, &cov);
#endif /*NO_LO_SERVICE_AVAILABLE*/

}

RelPose* RelPoseFactory::FRelPose(RelPose* pose, LocatedObjectID_t parent, Matrix m, Matrix cov)
{
#ifdef NO_LO_SERVICE_AVAILABLE
  throw "Error: not yet implemented";
#else /*NO_LO_SERVICE_AVAILABLE*/
  return s_loService->UpdatePose(pose,parent, &m, &cov);
#endif /*NO_LO_SERVICE_AVAILABLE*/
}



RelPose* RelPoseFactory::FRelPose(LocatedObjectID_t id)
{
  return GetRelPose(id);
}

void RelPoseFactory::FreeRelPose(RelPose** pose, bool temporary )
{
  if(pose == NULL || *pose == NULL)
     return ;
  int id = (*pose)->m_uniqueID;
  if(id != ID_WORLD)
  {
#ifdef NO_LO_SERVICE_AVAILABLE
    for(std::vector<RelPose*>::iterator iter = s_relPoses.begin();
    iter != s_relPoses.end(); iter++)
    {
      if((*iter)->m_uniqueID == id)
      {
        printf("Release relpose id: %d\n", id);
        delete (*iter);
        *pose = NULL;
        s_relPoses.erase(iter);
        break;
      }
    }
#else /*NO_LO_SERVICE_AVAILABLE*/

    if(!s_loService->FreePose((*pose)->m_uniqueID))
    {
      if(temporary)
       {
//    ROS_ERROR("A pose (%ld) marked as temporary could not be deleted", (*p
       }
    }
#endif  /*NO_LO_SERVICE_AVAILABLE*/
  }
  if((*pose) != NULL && id != ID_WORLD)
  {
    delete (*pose);
    (*pose) = NULL;
  }
}
