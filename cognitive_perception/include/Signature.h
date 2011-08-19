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
                        Signature.h - Copyright klank


**************************************************************************/


#ifndef SIGNATURE_H
#define SIGNATURE_H


#include "Class.h"
#include "Object.h"

/**
 typedefs
*/
typedef double Probability_1D_t; /* < Type for storing Probability of a location to occur*/



#ifndef XML_NODE_SIGNATURE
#define XML_NODE_SIGNATURE "Signature"
#define XML_NODE_SIGNATURE_VEC "SignatureVector"
#endif

namespace cop
{
  class RelPose;
  class Sensor;


  /**
    * class Signature
    *	@brief contains the information about an object in the SignatureDB, part of the Elem hirarchy
    */
  class Signature : public Object
  {
  public:

    // Constructors/Destructors
    //
    /**
    * Empty Constructor
    */
    Signature ( );

    virtual ~Signature ( );

    // Static Public attributes
    //

    // Public attributes
    //

    // Public attribute accessor methods
    //


    // Public attribute accessor methods
    //



    /**
    * @return Elem
    * @param  index
    * @param  type
    */
    Elem* GetElement (const int &id, const ElemType_t &type ) const;
    /***
    *    RemoveElem
    *    @param elemToRemove  Element that shoul be taken out of the list
    */
    void RemoveElem(Elem* elemToRemove);

    size_t CountClasses() const {return m_class.size();}
    size_t CountElems() const {return m_elems.size();}
    Class* GetClass(int index);
    bool HasClass(Class* classToSet);

    virtual std::string GetNodeName() const{return XML_NODE_SIGNATURE;}
    /**
    * @return int
    * @param  elemToSet
    */
    long SetElem (Elem* elemToSet );
    long SetClass ( Class* classToSet);
    void Show(Sensor* cam);
    virtual ElemType_t GetType(){return SIGNATURE;}
    virtual void SetPose(RelPose* pose);
    /**
    *	Duplicates an elem (only possible for elems that can be constructed with ElemFactory or that have overriden this member)
    */
    virtual Elem* Duplicate(bool bStaticCopy);

    /***********************************************************************
    * Evaluate                                                     */
    /************************************************************************
    * @brief Puts a result to a descriptor to set its quality.
    * @param eval a value from 0.0 (bad) to 1.0 (good)
    * @param weight a value describing the influence of the former
    *        m_qualityMeasure
    *************************************************************************/
    virtual void Evaluate(const double eval, const double weight);

  protected:
    virtual void SaveTo(XMLTag* tag);
    /**
    *   SetData
    *   @param tag
    *   @throw char* with an error message in case of failure
    */
    virtual void SetData(XMLTag* tag);

    // Protected attribute accessor methods
    //


  private:

    std::vector<Elem*> m_elems;
    std::vector<Class*> m_class;
    void initAttributes ( ) ;

    boost::mutex m_mutexClasses;
  };
}
#endif // SIGNATURE_H
