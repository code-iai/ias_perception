/*
 * Copyright (c) 2010, Ulrich Klank
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

/************************************************************************
TableObjectDescriptors.cpp
**************************************************************************/

#include "ElemTypes.h"
#include "NamedDescriptor.h"
#include "RemoteAttention.h"
#include "XMLTag.h"
#include <pluginlib/class_list_macros.h>
#include <ias_table_msgs/TableObject.h>
#include <std_msgs/String.h>

namespace cop
{
   CREATE_NAMED_DESCRIPTOR(TableObjectDescriptor, "TableObjectDescriptor", "DefaultTableObject", DESCRIPTOR_TABLEOBJ, ias_table_msgs::TableObject)
   CREATE_NAMED_DESCRIPTOR(ShapeTypeDescriptor, "ShapeType", "DefaultShapeType", DESCRIPTOR_SHAPETYPE, std_msgs::String)

   class  TableObjectAttention : public RemoteAttention< ias_table_msgs::TableObject >
   {
     public:
        TableObjectAttention(){SetObjectType(DESCRIPTOR_TABLEOBJ);}
      virtual std::vector<Signature*> MessageToSignature(boost::shared_ptr< ias_table_msgs::TableObject const> msg)
      {
        std::vector<Signature*> results;
        printf("TableObjectAttention::MessageToSignature\n");
        Signature* obj = new Signature();
        obj->m_ID = msg->object_cop_id;
        TableObjectDescriptor* descr = new TableObjectDescriptor();
        ShapeTypeDescriptor* descr2 = new ShapeTypeDescriptor();
        descr2->SetClass(msg->object_geometric_type);
        descr->SetClass(msg->object_type);
        /** Depending on the content set here the right Class*/
       //  descr->SetClass();
        descr->SetContent(*msg);
        obj->SetElem(descr);
        obj->SetElem(descr2);
        try
        {
          RelPose* pose = RelPoseFactory::FRelPose(msg->lo_id);
          if(pose != NULL)
           obj->SetPose(pose);
        }
        catch(...)
        {
          printf("no pose here\n");
        }
        results.push_back(obj);
        printf("Returning object with id %ld\n", obj->m_ID);
        return results;
      }
      virtual std::string GetName(){return "TableObjectAttention";}
   };
   class TableObjectRedetector : public LocateAlgorithm
   {
    public:
      TableObjectRedetector()
      {
      }

      void SetData(XMLTag* tag)
      {
        printf("Loading Algorithm %s \n", GetName().c_str());
        if(tag != NULL && tag->CountChildren() > 0)
        {
          m_firstAlg = LocateAlgorithm::LocAlgFactory(tag->GetChild(0));
        }
        else
           throw "Error loading TwoInOneAlg";
      }


      ~TableObjectRedetector(void)
      {
        delete m_firstAlg;
      }


        XMLTag* Save()
        {
        XMLTag* tag = new XMLTag(GetName());
        tag->AddChild(m_firstAlg->Save());
        return tag;
        }

        double RelateMatrices(const Matrix& m, const Matrix& m2)
        {
          double val = 0.0;
          double sumM1 = m.element(0,3)*m.element(0,3) + m.element(1,3)*m.element(1,3) + m.element(2,3)*m.element(2,3);
          double sumM2 = m2.element(0,3)*m2.element(0,3) + m2.element(1,3)*m2.element(1,3) + m2.element(2,3)*m2.element(2,3);
          val =  (sumM2 - sumM1)  / sumM2;
          return val < 0 ? 0.0 : val;
        }
        std::vector<RelPose*> Perform(std::vector<Sensor*> cam, RelPose* posein, Signature& object, int &numOfObjects, double& qualityMeasure)
        {
          int numOfObjects_first = numOfObjects;
          double qualityMeasure_first = qualityMeasure;
          std::vector<RelPose*> result_first = m_firstAlg->Perform(cam, posein, object, numOfObjects_first, qualityMeasure_first);
          std::vector<RelPose*> results;
          if(numOfObjects_first > 0)
          {
            for(unsigned int i = 0; i < result_first.size(); i++)
            {
              RelPose* pose = result_first[i];
              Matrix m = posein->GetMatrix(pose->m_uniqueID);
              Matrix m2 = pose->GetMatrix(pose->m_parentID);
              double val = RelateMatrices(m, m2);

              pose->m_qualityMeasure = val;
              ROS_WARN("TableObjectRedetector Debug out: Score is scaled by val: %f (which represents the distance) and is now %f\n", val, pose->m_qualityMeasure);
              results.push_back(pose);
              if(i == 0)
              {
                 qualityMeasure = result_first[i]->m_qualityMeasure * val;
                 numOfObjects = 1;
              }
              else
                numOfObjects++;
            }
          }
          return results;
        }

        double CheckSignature(const Signature& object, const std::vector<Sensor*> &sensors)
        {
          double val1 = m_firstAlg->CheckSignature(object, sensors);
          if(val1 > 0.0)
          {
            if(object.GetElement(0,DESCRIPTOR_TABLEOBJ))
              return 1.0;
          }
          return  0.0;
        }

      virtual std::string GetName(){return "TableObjectRedetector";}
    private:
      LocateAlgorithm* m_firstAlg;
   };

}

using namespace cop;

PLUGINLIB_REGISTER_CLASS(TableObjectDescriptor, cop::TableObjectDescriptor, Descriptor);
PLUGINLIB_REGISTER_CLASS(ShapeTypeDescriptor, cop::ShapeTypeDescriptor, Descriptor);
PLUGINLIB_REGISTER_CLASS(TableObjectAttention, cop::TableObjectAttention, AttentionAlgorithm);
PLUGINLIB_REGISTER_CLASS(TableObjectRedetector, cop::TableObjectRedetector, LocateAlgorithm);

