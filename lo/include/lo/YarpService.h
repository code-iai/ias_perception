#ifndef YARPSERVICE_H
#define YARPSERVICE_H

#include "lo/ServiceInterface.h"
#include "lo/ServiceLocatedObject.h"

namespace yarp
{
  namespace os
  {
    class Bottle;
  }
}

namespace jlo
{
/**
*     class YarpService
*     Implementation for yarp reading bottles and calling the lo ServiceInterface functionality
*/
class YarpService : public ServiceInterface
{
public:
  /**
  *     Start the service listening to listeningPort, g_stopAll can stop the loop after next read
  */
  YarpService (const char* listeningPort, const char* configFile);
  /**
  *   Closing of the ports
  */
  ~YarpService ();
private:
  /**
  * Parses the bottle and reacts on the input, by generating the answer
  * @param botIn incoming request
  * @param bot outgoing answer
  */
  bool FillBottle(yarp::os::Bottle& botIn, yarp::os::Bottle& bot);
  /**
  *   Write an Lo to a bottle
  */
  void PutIntoBottle(ServiceLocatedObject* lo, yarp::os::Bottle& bot);

};
}
#endif /*YARPSERVICE_H*/
