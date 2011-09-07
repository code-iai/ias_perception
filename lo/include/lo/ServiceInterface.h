#ifndef SERVICEINTERFACE_H
#define SERVICEINTERFACE_H


//#include "lo/ObjectContainer.h"

#ifndef LO_TYPE_PERCEIVED
#define LO_TYPE_PERCEIVED 1
#endif

#include <string>

class Matrix;
namespace jlo
{
   class ServiceLocatedObject;
   class XMLTag;

/**
* The lo pose service
* Holds a list of poses
* The poses contain tree information
*/
class ServiceInterface
{
public:
  /**
  * Constructor
  * @brief reads config file
  */
  ServiceInterface(const char* configFile);
  ~ServiceInterface();


  /**
  * @brief Saves all known los to a list
  */
  XMLTag*	SaveList();
  /**
  * @brief Puts entries of an xmltag to the list
  */
  void LoadList(XMLTag* tag);

  /**
  * @brief Creates a ServiceLocatedObject out of an xmltag
  */
  static ServiceLocatedObject* FServiceLocatedObject(XMLTag* tag);
  /**
  *  @brief Creates an lookup entry for a pose in the list of names
  */
  static void AddMapString(ServiceLocatedObject* pose, std::string mapstring);
  /**
  *  @brief Removes the lookup entry for a name
  */
  static void RemoveMapString(std::string mapstring);
  /**
  * @brief Creates a ServiceLocatedObject taking an parent and matrix information
  */
  static ServiceLocatedObject* FServiceLocatedObject(ServiceLocatedObject* pose, Matrix m, Matrix cov, unsigned long type = LO_TYPE_PERCEIVED);
  /**
  * @brief Creates a ServiceLocatedObject with the world
  */
  static ServiceLocatedObject* FServiceLocatedObjectWorld();
  /**
  * @brief Creates a ServiceLocatedObject taking another one
  */
  static ServiceLocatedObject* FServiceLocatedObjectCopy(ServiceLocatedObject* pose, ServiceLocatedObject* parent = NULL);

  /**
  * @brief Deletes all Lo
  */
  static void DisposeList();

  /**
  * @brief Deletes one Lo
  */
  static unsigned long FreeServiceLocatedObject(ServiceLocatedObject* pose);

  /**
  * @brief Looks up an lo
  */
  static ServiceLocatedObject* GetServiceLocatedObject(unsigned long id);
  /**
  * @brief Looks up an lo
  */
  static unsigned long GetServiceLocatedObjectID(std::string);
  /**
  * @brief Internal function: puts a lo to the list
  */
  static unsigned long SetServiceLocatedObject(ServiceLocatedObject* pose);
  /**
  * @brief Internal function: takes an lo from the list
  */
  static ServiceLocatedObject* GetServiceLocatedObjectIndex(unsigned long index);
};
} /**end namespace jlo*/
#endif /*ServiceInterface*/

