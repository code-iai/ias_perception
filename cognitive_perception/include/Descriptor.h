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
                        Descriptor.h - Copyright klank

**************************************************************************/


#ifndef DESCRIPTOR_H
#define DESCRIPTOR_H


#include "Class.h"
#include "GeometricShape.h"


#define XML_NODE_DESCRIPTOR "Descriptor"
#define XML_ATTRIBUTE_MODELID "ModelID"


namespace cop
{

  class RelPose;
  class Reading;


  /**
    * class Descriptor
    *  @brief Represents visual models in the SignatureDB, part of the Elem hirarchy
    */
  class Descriptor : public Elem
  {
  public:

    // Constructors/Destructors
    //
    /**
    * Empty Constructor
    */
    Descriptor ( Class* classref);
    Descriptor ( );

    /**
    * Empty Destructor
    */
    virtual ~Descriptor ( );

    /***********************************************************************
      *   SetLastMatchedImage                                             */
    /************************************************************************
    *  @brief sets the last iamge on that this model was matched
    *  @param img  an image
    *  @param pose a pose that specifies the result in the given image
    *  of a locating algorithm
    *  @throw char* with an error message in case of failure
    **************************************************************************/
    void SetLastMatchedImage(Reading* img, RelPose* pose);
    /**
    */
    /***********************************************************************
    * GetLastMatchedImage                                      */
    /************************************************************************
    *	@returns the last iamge on that this model was matched
    *************************************************************************/
    Reading* GetLastMatchedImage(){return m_imgLastMatchReading;}
    /***********************************************************************
    * GetLastMatchedPose                                                     */
    /************************************************************************
    * @brief The descrtiptor remebers the last matched pose/image combination for refinement.
    *************************************************************************/
    const RelPose* GetLastMatchedPose() const  {return m_poseLastMatchReading;}
    RelPose* GetLastMatchedPose(){return m_poseLastMatchReading;}
    /***********************************************************************
    * GetQuality                                                     */
    /************************************************************************
    * @returns the quality of the descriptor                                *
    *************************************************************************/
    double GetQuality(){return m_qualityMeasure;}
    /***********************************************************************
    * Evaluate                                                     */
    /************************************************************************
    * @brief Puts a result to a descriptor to set its quality.
    * @param eval a value from 0.0 (bad) to 1.0 (good)
    * @param weight a value describing the influence of the former
    *        m_qualityMeasure
    *************************************************************************/
    virtual void Evaluate(const double eval, const double weight){m_qualityMeasure = (eval  + m_qualityMeasure * weight) / (1+weight); } //TOCHECK: how to combine and TODO if(m_qualityMeasure == 0.0) delete this;

    /***********************************************************************
    * Show                                                     */
    /************************************************************************
    * @brief if this Descriptor can be showed, show it.
    * @param pose A position where this descriptor should be displayed,
    * @param camera that took the picture where the descriptor was displayed
    *************************************************************************/
    virtual void Show(RelPose* , Sensor* ){};
    /***********************************************************************
    * GetNodeName                                                     */
    /************************************************************************
    * @brief  The node name for saving and loading to XML
    *************************************************************************/
    virtual std::string GetNodeName() const{return XML_NODE_DESCRIPTOR;}

    /***********************************************************************
    * SaveTo                                                    */
    /************************************************************************
    * @brief  Adds the descriptors parameter to a XMLTag
    *************************************************************************/
    virtual void SaveTo(XMLTag* tag);
    // Public attribute accessor methods
    //
    Class* GetClass();
    void SetClass(Class* cl){m_class = cl;}
    /***********************************************************************
    *  GetShape                                                       */
    /************************************************************************
    * @brief fills in  the object's shape
    * @param objectShape retrieves the shape of a descriptor
    * @return true if a Shape was added, false if not. Mesh overrides Cylinder
    *         overrides  Box
    *************************************************************************/
    virtual bool GetShape(GeometricShape &objectShape) const {return false;}
    virtual void PropagatePose(RelPose* pose){}
  protected:
    virtual void SetData(XMLTag* tag);


  protected:
    Class* m_class;
    Reading* m_imgLastMatchReading;
    RelPose* m_poseLastMatchReading;

    double m_qualityMeasure;
  };

}
#endif // DESCRIPTOR_H

