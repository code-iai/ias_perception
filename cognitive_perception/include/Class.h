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
                        Class.h - Copyright klank



**************************************************************************/


#ifndef CLASS_H
#define CLASS_H

#include "Elem.h"

#define XML_NODE_CLASS "Class"
#define XML_ATTRIBUTE_CLASSNAME "ClassName"


#define DEFINED_CLASS_NAME_SUPPORTING_PLANE "SupportingPlane"


namespace cop
{
  /**
    * class Class
    * @brief Class represents high level descriptions of objects in the signature database, it is part of the Elem hirarchy
    *
    *  Nodename:   XML_NODE_CLASS "Class"                Saves the semantic connection with highlevel for Descriptor and Signature \n
    *  Attributes: XML_ATTRIBUTE_CLASSNAME "ClassName"   The actual value that could be queried by highlevel
    */
  class Class : public Elem
  {
  public:

    // Constructors/Destructors
    //
    /**
    * Empty Constructor
    */
    Class ();

    Class(std::string name, int id);

    Class (XMLTag* tag );

    /**
    * Empty Destructor
    */
    virtual ~Class ( );


    virtual std::string GetNodeName() const{return XML_NODE_CLASS;}

    void SetName(std::string name);

     std::string GetName(){return m_name;}
    /**
    *   GetType
    *   @brief returns the element specialisation
    */
    virtual ElemType_t GetType(){return CLASS;}
   
     virtual Elem* Duplicate(bool staticcopy);  
    
    // Static Public attributes
    //

    // Public attributes
    //


    // Public attribute accessor methods
    //


    // Public attribute accessor methods
    //


  protected:
    virtual void SaveTo(XMLTag* tag);

    virtual void SetData( XMLTag* tag );
  private:

    // Static Private attributes
    //
    std::string m_name;
    // Private attributes
    //


    // Private attribute accessor methods
    //


    // Private attribute accessor methods
    //



  };
}
#endif // CLASS_H
