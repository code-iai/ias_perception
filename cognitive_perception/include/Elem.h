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
                        Elem.h - Copyright klank


**************************************************************************/


#ifndef ELEM_H
#define ELEM_H

#include <string>
#include <vector>
#include "ElemTypes.h"


#include <boost/thread.hpp>

#define XML_PROPERTY_ELEMID "ElemID"
#define XML_PROPERTY_PPID "CreatorPP"

#define XML_NODE_ELEMENT "Elem"


#define FORBIDDEN_ID_RANGE_MIN 700000

namespace cop
{

  class XMLTag;
  class Sensor;

  /**
    * class Elem
    * @brief basic class for all entries in the SignatureDB
    *
    * Nodename: XML_NODE_ELEMENT "Elem" Prototype for Descriptor, Class and Signature
    * Properties:
    *       XML_PROPERTY_ELEMID "ElemID"  a unique id, same id means same semantic
    *
    *
    */
  class Elem
  {
  public:
    /**
     * Constructor that creates a new element
     */
    Elem( );
    /**
    * Constructor that restores an certain element
    */
    Elem ( ObjectID_t id );

    /**
     * Empty Destructor
     */
    virtual ~Elem( );

    // Static Public attributes
    //
    static ObjectID_t m_LastID;
    // Public attributes
    //
    ObjectID_t m_ID;
    /**
    *   Save the current element, the function will call SaveTo().
    *   @param full_pose if set to true poses will not be saved as jlo reference but as a matrix
    */
    XMLTag* Save(bool full_pose = false);
    /**
    *	Can Creates most of the possible elements, returns null if the node name was not known
    *	@param tag xmltag that has an name of an elem, see ElemType.h for the possible elements
    *	@returns a inheritation of elem.
      *   @throws char* with an error message in case of failure
    */
    static Elem* ElemFactory(XMLTag* tag);
    /**
    *	Duplicates an elem (only possible for elems that can be constructed with ElemFactory or that have overriden this member)
    */
    virtual Elem* Duplicate(bool bStaticCopy = true);
    /**
    *   returns the name that a will be used in the xml. This should be replaced by any derivative of Elem
    */
    virtual std::string GetNodeName() const {return XML_NODE_ELEMENT;}

    /**
    *   Returns the type which is used for checkign for certain descriptor types, should be overwritten by any derivative of Elem
    */
    virtual ElemType_t GetType() const {return ELEM;}
    /**
    *   Shows the current Element relative to the given Camera
    */
    virtual void Show(Sensor* ){};
    /**
    *   Set the vision primitive that created this elem
    */
    virtual void SetLastPerceptionPrimitive(PerceptionPrimitiveID_t id){m_creator = id;}
    /**
    *   Get the vision primitive that created this elem
    */
    virtual PerceptionPrimitiveID_t GetLastPerceptionPrimitive(){return m_creator;}
    /**
    *	Age of the newest information carried by this element
    */
    virtual unsigned long date() const
    {
      return m_timestamp;
    }
    void Touch();
    void SetTimeStamp(unsigned long timestamp)
    {
      m_timestamp = timestamp;
    }

    void SetFullPose(bool fullPose){m_fullPose = fullPose;}

    /***********************************************************************
    * Evaluate                                                     */
    /************************************************************************
    * @brief Puts a result to a descriptor to set its quality.
    * @param eval a value from 0.0 (bad) to 1.0 (good)
    * @param weight a value describing the influence of the former
    *        m_qualityMeasure
    *************************************************************************/
    virtual void Evaluate(const double eval, const double weight){}


  protected:
    /**
    * Overwrite to save additional data, will be called during saving
    */
    virtual void SaveTo(XMLTag* )      {}
    /**
    * Overwrite to set additional data on creation, will be called directly after creation
    */
    virtual void SetData(XMLTag* data);

    bool m_fullPose;

    typedef boost::mutex::scoped_lock ElemWriteLock;
    boost::mutex m_mutexElems;

  private:
   unsigned long m_timestamp;
   PerceptionPrimitiveID_t m_creator;

  };
}
#endif // ELEM___H
