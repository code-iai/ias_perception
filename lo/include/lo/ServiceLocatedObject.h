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

#ifndef SERVICELOCATEDOBJECT_H
#define SERVICELOCATEDOBJECT_H

#include "lo/LocatedObject.h"


#include <string>
#include <vector>
#include <map>

#define LO_TYPE_PHYSICAL 1


#define XML_NODE_SaveableLo "SaveableLo"
#define XML_ATTRIBUTE_LOID "loid"
#define XML_ATTRIBUTE_LOIDFATHER "loidfather"
#define XML_NODE_MATPOSE "Pose"
#define XML_NODE_COVARIANCE "Cov"
#ifndef ID_WORLD
#define ID_WORLD 1
#endif

/************************************************************************
                        XMLTag.h - Copyright klank


**************************************************************************/
typedef struct _xmlTextWriter xmlTextWriter;
typedef struct _xmlTextReader xmlTextReader;
#define XML_NODE_STD_VECTOR "std_vector"
#define XML_NODE_STD_PAIR "std_pair"
#define XML_NODE_INT "int"
#define XML_NODE_DOUBLE "double"
#define XML_NODE_STRING "std_string"
#define XML_NODE_MATRIX "newmat_Matrix"
#define XML_ATTRIBUTE_ROWS "rows"
#define XML_ATTRIBUTE_COLS "cols"

/**********************************************************************************************/
/** class XMLTag    intern class for handling xml transfer
 **********************************************************************************************
 * @brief stores data, to files or memory.
 * Contains functionality to read from files and create any xml representation using libxml2
 *
 **********************************************************************************************
 *
 **********************************************************************************************/
namespace jlo
{
class ServiceInterface;

class XMLTag
{
public:

  // Constructors/Destructors
  //


  /**
  * Constructor, sets the tag name
  */
  XMLTag ( std::string Name);

  /**
  * Empty Destructor
  */
  virtual ~XMLTag ( );
  ///
  /// Methods
  ///
/****************************************************************************/
/** WriteToFile: Writes an XMLTag to an File
 ****************************************************************************
 *
 * Writes an XMLTag to an File, can return a XMLTag* that refers the file.
 * Such a file reference will be resolved when it is a child of another node
 * and GetChild is called
 *
 * @param     stFile      absolut filename
 *
 * @paramo    fileReference	a pointer that receives the pointer to the new
 *   						file reference tag
 *
 * @param                    error code
 *
 *
 *****************************************************************************
 *
 * \remark              file name will not be checked
 *
 *****************************************************************************/
  void WriteToFile(const std::string& stFile, XMLTag** fileReference = NULL) const;
/****************************************************************************/
/** SetCData: sets the current content of the tag
 ****************************************************************************
 *
 * sets the current content of a tag
 *
 * @param     value      content of the tag
 *
 *
 *****************************************************************************
 *
 *
 *****************************************************************************/
    char* WriteToString() const;
/****************************************************************************/
/** ReadFromFile: Creates a new XMLTag from a file
 ****************************************************************************
 *
 * As constructor this would have had the same function signature as the std. constructor
 * so its a static function
 *
 *
 * @param     stFile      absolut filename
 *
 *
 *
 *****************************************************************************
 *
 * \remark              can return NULL if the file does not contain a tag
 *
 *****************************************************************************/
  static XMLTag* ReadFromFile(const std::string &stFile);
/****************************************************************************/
/** Clone: clones the current tag and returns it
 ****************************************************************************
 *
 * clone includes recursive copying of all children
 *
 *****************************************************************************
 *
 *
 *****************************************************************************/
  XMLTag* Clone() const;

/****************************************************************************/
/** Write: helping function for WriteToFile
 ****************************************************************************
 *
 * Writes an XMLTag to an File,
 *
 *
 *
 * @param     pWriter			pointer to the xmlWriter
 *
 *
 *****************************************************************************
 *
 *
 *****************************************************************************/
  virtual void Write(xmlTextWriter* pWriter) const;
/****************************************************************************/
/** Read: Reads an XMLTag from a readeer
 ****************************************************************************
 *
 * Supporting function for ReadFromFile
 *
 * @param     pReader			pointer to an libxml2 xmlTextReader
 *
 *
 *
 *****************************************************************************
 *
 *
 *****************************************************************************/
  static XMLTag* Read(xmlTextReader* pReader);
/****************************************************************************/
/** Tag: creates XMLTags from standard types, like vector pair, int, double, Elem
 ****************************************************************************
 *	@param T	contains the element that should be saved in the tag
 *	@param name defines the name of the node,
 *				there are standard names for every implemented tag
 *****************************************************************************
 * \remarks pairs and vectors of any supported Type T are allowed, too
 *
 *****************************************************************************/

  template<typename T> static XMLTag* Tag(std::vector<T> vector, std::string name = "")
  {
    XMLTag* tag = new XMLTag(name.compare("") == 0 ? XML_NODE_STD_VECTOR : name);
#ifdef WIN32
    std::vector<T>::const_iterator iter;
    for(iter = vector.begin(); iter != vector.end(); iter++)
    {
      tag->AddChild(XMLTag::Tag((*iter)));
    }
#else
    for(unsigned int i = 0; i < vector.size(); i++)
    {
      tag->AddChild(XMLTag::Tag(vector[i]));
    }
#endif
    return tag;
  }
  template<typename T1, typename T2> static XMLTag* Tag(std::pair<T1, T2> p, std::string name = "")
  {
    XMLTag* tag = new XMLTag(name.compare("") == 0 ? XML_NODE_STD_PAIR : name);
    tag->AddChild(XMLTag::Tag(p.first));
    tag->AddChild(XMLTag::Tag(p.second));
    return tag;
  }


  static XMLTag* Tag(int n, std::string name = "");
  static XMLTag* Tag(double d, std::string name = "");
  static XMLTag* Tag(std::string value, std::string name = "");
  static XMLTag* Tag(const Matrix& alg, std::string name = "");


/****************************************************************************/
/** Load: Load a Type T from a XMLTag
 ****************************************************************************
 *
 * Supported Types are for example: vector, pair, double, int, string, Matrix
 *
 * @param     tag			the tag that was read from a file or created with a Tag function
 * @param		T*			a pointer that specifies the wished class (necessary for resolving the template)
 *
 * @throws char* with an error message in case of failure
 *****************************************************************************
 *
 *
 *****************************************************************************/
  template<typename T>
  static T Load(XMLTag* tag, T* t )
  {
    throw "Load Failed: Expected type has no loading function implemented";
  };

  template<typename T>
   static std::vector<T> Load(XMLTag* tag, std::vector<T>*  )
  {
    std::vector<T> vectorTemp;
    for(unsigned int i = 0; i < tag->CountChildren(); i++)
    {
      try
      {
        vectorTemp.push_back(Load(tag->GetChild(i), (T*)NULL));
      }
      catch(...)
      {
        printf("!!!!!!!!! Error reading a vector!!!!!!!!!! \n\n\n");
        /* Do nothing, try to get the others*/
      }
    }
    return vectorTemp;
  };

  template<typename T1, typename T2>
  static std::pair<T1, T2> Load(XMLTag* tag, std::pair<T1, T2>*  )
  {
    std::pair<T1, T2> p;
    int childCount = tag->CountChildren();
    if(childCount > 0)
      p.first = Load(tag->GetChild(0), (T1*)NULL);
    if(childCount > 1)
      p.second = Load(tag->GetChild(1), (T2*)NULL);

    return p;
  };

  static int Load(XMLTag* tag, int* )
  {
    if(tag == NULL)
      return 0;
    return  tag->GetCDataInt();
  };

  static double Load(XMLTag* tag,double *)
  {
    if(tag == NULL)
      return 0.0;
    return tag->GetCDataDouble();
  };

  static std::string Load(XMLTag* tag, std::string *)
  {
    if(tag == NULL)
      return "";
    return tag->GetCDataST();
  };

  static Matrix Load(XMLTag* tag, Matrix* );

/****************************************************************************/
/** AddProperty: adds a property as an attribute to the current tag
 ****************************************************************************
 *
 * adds a property as an attribute to the current tag
 *
 * @param     name      name of the property, works as a key
 *
 * @param    value			value of the property
 *
 *
 *****************************************************************************
 *
 *
 *****************************************************************************/
  void AddProperty(const std::string& name, const std::string& value);
  void AddProperty(const std::string &name, const unsigned long &value);
  void AddProperty(const std::string& name, const int &value);
  void AddProperty(const std::string& name, const double &value);

/****************************************************************************/
/** SetCData: sets the current content of the tag
 ****************************************************************************
 *
 * sets the current content of a tag
 *
 * @param     value      content of the tag
 *
 *
 *****************************************************************************
 *
 *
 *****************************************************************************/
  void SetCData(const int &value);
  void SetCData(const double &value);
  void SetCData(const std::string &value);

/****************************************************************************/
/** GetCData*: returns the current content
 ****************************************************************************
 *
 * returns the current content
 *
 *
 * @param		the content or empty("", 0, 0.0)
 *
 *
 *****************************************************************************
 *
 *
 *****************************************************************************/
  int			GetCDataInt() const;
  double		GetCDataDouble() const;
  std::string GetCDataST() const;


/****************************************************************************/
/** GetProperty*: returns the value of a property
 ****************************************************************************
 *
 * Returns the current value of a property
 *
 * @param     name      name of the property
 *
 *
 * @param                    the value or empty ("", 0, 0.0)
 *
 *
 *****************************************************************************
 *
 *
 *****************************************************************************/
  std::string GetProperty(const std::string& name);
  double GetPropertyDouble(const std::string& name);
  int GetPropertyInt(const std::string& name);


/****************************************************************************/
/** AddChild: adds a child node to a tag
 ****************************************************************************
 *
 * Adds a child node to a tag, the position where to put it can be influenced
 * if the position is larger or smaller than the possible range the element
 *  will be put at the end of the list.
 *
 *
 * @param     child		pointer to an existing tag, a null child will not be appended
 *
 * @param     position		position where to put the cild in the child list
 *
 * @param                 final position that was selected, or -1
 *
 *
 *****************************************************************************
 *
 * \remark              child = NULL will not be appended and -1 will be returned
 *
 *****************************************************************************/
  int AddChild(XMLTag* child, int position = -1);
  void ReplaceChild(XMLTag* child, int position);
/****************************************************************************/
/** CountChildren: counts the childdren-tags of the current tag
 ****************************************************************************
 *
 * Counts the children
 *
 *
 * @param                 number of children
 *
 *
 *****************************************************************************
 *
 *
 *****************************************************************************/
  unsigned int CountChildren() const;
/****************************************************************************/
/** RemoveChild: removes an XMLTag* from the list
 ****************************************************************************
 *
 * Adds a child node to a tag, the position where to put it can be influenced
 * if the position is larger or smaller than the possible range the element
 *  will be put at the end of the list.
 *
 *
 * @param     position		position where to erase a cild
 *
 *
 *
 *****************************************************************************
 *
 * \remark              the tag will not be deleted!
 *
 *****************************************************************************/
  void RemoveChild(const unsigned int &position);
  void RemoveChild(const std::string &name);

/****************************************************************************/
/** GetChild: returns a child tag
 ****************************************************************************
 *
 * returns a child tag
 *
 * @param position/name   descriptor for the child
 *
 * @param		a  child tag
 *
 *
 *****************************************************************************
 *
 *
 *****************************************************************************/
  XMLTag* GetChild(const unsigned int &position);
  XMLTag* GetChild(const std::string &name);
  XMLTag* GetChild(const std::string& name, int innerindex);

/****************************************************************************/
/** GetName: returns the name of a tag
 ****************************************************************************
 *
 *  The name of the tag (<name></name>)
 *
 *
 *
 *****************************************************************************
 *
 *
 *****************************************************************************/
  std::string& GetName(){return m_name;}

  //static public Methods
  static unsigned int OldTag()
  {
    return 0;
  }

  unsigned long date();


  private:
    /**
    *	finction to reset the timestamp
    */
    void Touch();

  // Private attributes
  //
  unsigned long							m_lastChanged;
  std::vector<XMLTag*>					m_children;
  std::string								m_cData;
  std::string								m_name;
  std::map<std::string, std::string>		m_properties;
};

/**
*   class ServiceLocatedObject
*   @brief Specialization of jlo::LocatedObject, manages a list of such objects
*  internal service version of located object
*/
class ServiceLocatedObject :  public LocatedObject
{
public:
  //
  // Constructors/Destructors
  //


/********************************************************************/
/**      Empty constructor
*********************************************************************
*
*      	 initial coordinates will be 0,...,0 without relation
*
********************************************************************/
  ServiceLocatedObject ( );

  ServiceLocatedObject(ServiceLocatedObject* locobj, const Matrix &matrix , const Matrix &covariance);


  ServiceLocatedObject(ServiceLocatedObject* locobj, double x = 0.0, double y = 0.0, double z = 0.0,
	  double roll = 0.0, double pitch = 0.0, double yaw = 0.0,
	  double sigmaX = 0.0, double sigmaY = 0.0, double sigmaZ = 0.0,
	  double sigmaRoll = 0.0, double sigmaPitch = 0.0, double sigmaYaw = 0.0);

#ifdef _DEBUG
  void Print();
#endif
public:
  XMLTag* SaveComplete() ;
/*	void TransformPointLocally(const double& x_in, const double& y_in, const double& z_in, double& x_out, double& y_out, double& z_out, const double& scale);*/
  XMLTag* Save() ;

  /**
  *   UpdateParent
  *
  *   @brief Sets a new parent to an updated pose. it removes the object from its parents attachment list and adds it to the new parent
  *   @param new_parent the new parent that should replace the old.
  */
  void UpdateParent(ServiceLocatedObject* new_parent);
  /**
  *   Update
  *
  *    @brief Updates the position of a located object
  *
  *    @param m 4x4 matrix containing the complete transformation ( ~ R*t) from the parent to the child
  *    @param cov the uncertainty in a 6x6 matrix in child coordinates as [covx, covz, covz, covRoll, covPitch, covYaw]^T [covx, covz, covz, covRoll, covPitch, covYaw]
  *    @param copy a function to copy a ServiceLocatedObject* before update, if there are attached child nodes
  *    @param del  a function to delete parents that are not needed anymore in case of detachment of all attached children
  *    @param updated a function to inform subscribers of one of the touched nodes in the tree of an update
  **/
  void Update(Matrix m, Matrix cov,  ServiceLocatedObject*(* copy)(ServiceLocatedObject*, ServiceLocatedObject*), unsigned long(*del)(ServiceLocatedObject*), void (*updated)(unsigned long));
  /**
  *  Functions for Reference Counter
  */
  void DecreaseReferenceCounter(){if(referenceCounter>0)referenceCounter--;}
  void IncreaseReferenceCounter(){referenceCounter++;}
  unsigned long GetReferenceCounter(){return referenceCounter;}
  /********************************************************************/
  /**      Destructor notifying the parent
  *********************************************************************
  *
  ********************************************************************/
  virtual ~ServiceLocatedObject ( );

  /********************************************************************/
  /**     operator-
  *********************************************************************
  *       \brief Calculates the distance between two objects,
  *
  *	The calculation will be relative to the nearest common
  *	preceeding node in the LocatedObject Tree
  *
  *
  *       \pi     smallObject
  *       \ret    LocatedObject
  ********************************************************************/
  ServiceLocatedObject operator- (ServiceLocatedObject &smallObject ) ;

  /********************************************************************/
  /**     AddAttachedObject
  *********************************************************************
  *       \brief Adds an object to the list of dependant objects
  *               In case of the object beeing moved, all attached objects
  *               will be put to a copy *this
  *       \remarks if such an object is of type LO_TYPE_PHYSICAL
  *                  it will stay attached even if
  ********************************************************************/
  virtual void AddAttachedObject(ServiceLocatedObject* lo){IncreaseReferenceCounter();}

  /********************************************************************/
  /**     RemoveAttachedObject
  *********************************************************************
  *       \brief Removes an object from the list of dependant objects
  ********************************************************************/
  virtual void RemoveAttachedObject(ServiceLocatedObject* lo){DecreaseReferenceCounter();}
  /********************************************************************/
  /**     Move this
  *********************************************************************
  *       \brief Adds an object to the list of dependant objects
  *               In case of the object beeing moved, all attached objects
  *               will be put to a copy *this
  *       \remarks if such an object is of type LO_TYPE_PHYSICAL
  *                  it will stay attached even if
  ********************************************************************/
  virtual void PropagateMovement(ServiceLocatedObject*(* copy)(ServiceLocatedObject*, ServiceLocatedObject*),
                                unsigned long (*del)(ServiceLocatedObject*), void (*updated)(unsigned long), ServiceLocatedObject*)
  {
    updated(m_uniqueID);
  }

  virtual bool NeedCopy (){return false;}
  virtual void TellParentNeedCopy(){return;}
  virtual void TellParentNeedNoCopy(){return;}

  /********************************************************************/
  /**     GetLOType
  *********************************************************************
  *       \brief returns if the object needs its children to be moved
  ********************************************************************/
  virtual unsigned long GetLOType(){return LO_TYPE_PERCEIVED;}


  /********************************************************************/
  /**	 m_relation
  *********************************************************************
  *
  *	The preeceding node in the LocatedObject Tree
  *
  ********************************************************************/
  ServiceLocatedObject* m_relation;
  /********************************************************************/
  /**	Loading: set lastid
  *********************************************************************
  *
  * 	\pi lastid
  ********************************************************************/
  static void SetLastID(unsigned long lastID){if(lastID >= s_lastID) s_lastID = lastID + 1;}
  /**
  *  Maps a string also to this index
  */
  std::string m_mapstring;
protected:
  unsigned long referenceCounter;
  long m_needCopy;
private:
  friend class ServiceInterface;
  static unsigned long s_lastID;
};
/**
*   Class LocatedObjectLoader
*   implements the simplest loader that is used inside the lo service.
 *  do not use outside
*/
  class LocatedObjectLoader : public LazyLocatedObjectLoader
  {
  public:
    LocatedObjectLoader(){}
    LocatedObject* GetParent(const LocatedObject& child){return ((ServiceLocatedObject*)&child)->m_relation;}
  };


}

#endif /*SERVICELOCATEDOBJECT*/

