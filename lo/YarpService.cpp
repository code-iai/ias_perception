#include "lo/YarpService.h"
#include <yarp/os/all.h>

using namespace yarp::os;


volatile bool g_stopall = false;
Port g_ListenPort;
#ifdef _DEBUG
#define PRINTF_DEBUG(A) printf(A)
#define PRINTF_DEBUG_2(A,B) printf(A,B)
#define PRINTF_DEBUG_3(A,B,C) printf(A,B,C)
#else
#define PRINTF_DEBUG(A)
#define PRINTF_DEBUG_2(A,B)
#define PRINTF_DEBUG_3(A,B,C)
#endif 
#include <signal.h>
void CTRLC(int)
{
  g_stopall = true;
  g_ListenPort.interrupt();
}

/**
*     Main program starting the service
*/
int main(int argc, char* argv[])
{
  signal(SIGINT, CTRLC);
  if(argc > 2)
  {
    jlo::YarpService serv(argv[2], argv[1]);
    serv.SaveList()->WriteToFile(argv[1]);
  }
  else if(argc > 1)
  {
    jlo::YarpService serv("/lo/in", argv[1]);
    serv.SaveList()->WriteToFile(argv[1]);
  }
  else
  {
    PRINTF_DEBUG_2("Usage: %s configFile PortName\n", argv[0]);
    jlo::YarpService serv("/lo/in", "lo-config.ini");
    serv.SaveList()->WriteToFile("lo-config.ini");
  }
  
}

namespace jlo
{
YarpService::YarpService (const char* listeingport, const char* configFile) :
  ServiceInterface(configFile)
{
  Network net;
  net.init();
  if(!net.queryName(listeingport).isValid())
  {
    if(g_ListenPort.open(listeingport))
    {
      while(!g_stopall)
      {
        Bottle botIn;
        g_ListenPort.read(botIn, true);
        Bottle bot;
        if(FillBottle(botIn, bot))
          g_ListenPort.reply(bot);
#ifdef _DEBUG
        PRINTF_DEBUG_2("Got Bottle:%s\n", botIn.toString().c_str());
        PRINTF_DEBUG_2("Sending: %s\n", bot.toString().c_str());
#endif /* _DEBUG*/
      }
    }
    else
    {
      printf("Opening of port %s failed! Check if the service already runs in your namespace or run yarp clean\n", listeingport);
    }
  }
  else
  {
    printf("Port %s already in use! Check if the service already runs in your namespace or run yarp clean\n", listeingport);
  }
}

  bool YarpService::FillBottle(Bottle& botIn, Bottle& bot)
  {
        int num = botIn.size();
        switch(num)
        {
        case 1: /** One number can only mean an id: ID-Query*/
          {
            Value id =  botIn.get(0);
            if(id.isInt())
            {
              ServiceLocatedObject* lo = YarpService::GetServiceLocatedObject(id.asInt());
              if (lo == NULL)
              {
                PRINTF_DEBUG("Error Parsing Bottle: Id does not exist!\n");
                bot.addString("Error Parsing Bottle: Id does not exist!\n");
                break;
              }
              PutIntoBottle(lo, bot);
            }
            else
            {
              PRINTF_DEBUG("Error Parsing Bottle: Wrong Type in bottle for id query!\n");
              bot.addString("Error Parsing Bottle: Wrong Type in bottle for id query!\n");
            }
            break;
          }
        case 2: /** Two numbers are interpreted as id + parent pair: Parent-Query*/
          {
            Value id =  botIn.get(0);
            if(id.isInt())
            {
              Value parent_id =  botIn.get(1);
              if(parent_id.isInt())
              {
                int parentID = parent_id.asInt();
                ServiceLocatedObject* lo = GetServiceLocatedObject(id.asInt());
                if (lo == NULL)
                {
                  PRINTF_DEBUG("Error Parsing Bottle: Id does not exist!\n");
                  bot.addString("Error Parsing Bottle: Id does not exist!\n");
                  break;
                }
                
                bot.clear();
                if(lo->m_uniqueID != ID_WORLD && lo->m_relation->m_uniqueID == parentID)
                {
                  PutIntoBottle(lo, bot);
                }
                else
                {
                  ServiceLocatedObject* parent = GetServiceLocatedObject(parentID);
                  if (parent == NULL)
                  {
                    PRINTF_DEBUG("Error Parsing Bottle: Parent Id does not exist!\n");
                    bot.addString("Error Parsing Bottle: Parent Id does not exist!\n");
                    break;
                  }
                  Matrix mat = lo->GetMatrix(*parent);
                  Matrix cov = lo->GetCovarianceMatrix(*parent);
                  PutIntoBottle(FServiceLocatedObject(parent, mat, cov), bot);
                }
              }
              else
              {
                    PRINTF_DEBUG("Error Parsing Bottle: Parent Id was not an integer!\n");
                    bot.addString("Error Parsing Bottle: Parent Id was not an integer!\n");
                    break;
              }
            }
            else if(id.isString())
            {
              std::string  st(id.asString().c_str());
              if(st.compare("del")==0)
              {
                if(botIn.get(1).isInt())
                {
                  int id = botIn.get(1).asInt();
                  if(id != ID_WORLD)
                  {
                    bot.addInt(FreeServiceLocatedObject(GetServiceLocatedObject(id)));
                  }
                  else
                  {
                    PRINTF_DEBUG("Error: Can't delete ID_WORLD!\n");
                    bot.addString("Error: Can't delete ID_WORLD!\n");
                  }
                }
              }
              else
              {
                PRINTF_DEBUG("Error Parsing Bottle: Wrong Type in bottle for delete query!\n");
                bot.addString("Error Parsing Bottle: Wrong Type in bottle for delete query!\n");
              }
            }
            else
            {
              PRINTF_DEBUG("Error Parsing Bottle: Wrong Type in bottle for parent_id query!\n");
              bot.addString("Error Parsing Bottle: Wrong Type in bottle for parent_id query!\n");
            }
            break;
          }
        case 0: /** Empty Bottle are not good*/
          PRINTF_DEBUG("Empty Bottle!\n");
          return false;
        default: /** More than 2 values should be data: Set Lo*/
          {
             Value parent_id, id;
             int counter = 0;
             bool update = false;
              if(botIn.get(0).isInt() && botIn.get(1).isInt())
              {
                id =  botIn.get(0);
                parent_id =  botIn.get(1);
                update = true;
                counter = 2;
              }
              else
              {
                parent_id =  botIn.get(0);
                counter = 1;
              }
              if(parent_id.isInt())
              {
                int parentID = parent_id.asInt();
                Matrix mat(4,4), cov(6,6);
                Value mat_in = botIn.get(counter);
                Value cov_in = botIn.get(counter + 1);
                int type = 1;
                if(botIn.size() > counter + 1)
                  type = botIn.get(counter +2).asInt();
                if(!mat_in.isList() || !cov_in.isList())
                {
                  PRINTF_DEBUG("Error Parsing Bottle: Matrix or covariance not send as a list!\n");
                  bot.addString("Error Parsing Bottle: Matrix or covariance not send as a list!\n");
                  break;
                }
                yarp::os::Bottle* mat_in_bot = mat_in.asList();
                if(mat_in_bot->size() < 16)
                {
                  PRINTF_DEBUG("Error Parsing Bottle: Matrix list is too short!\n");
                  bot.addString("Error Parsing Bottle: Matrix list is too short!\n");
                  break;
                }

                int width = 4;
                for(int r = 0; r < width; r++)
                {
                  for(int c = 0; c < width; c++)
                  {
                    yarp::os::Value& val = mat_in_bot->get(r * width + c);
                    if(val.isDouble())
                      mat.element(r,c) = val.asDouble();
                    else if(val.isInt())
                      mat.element(r,c) = val.asInt();
                    else
                      mat.element(r,c) = 0;
                  }
                }
                width = 6;
                yarp::os::Bottle* cov_in_bot = cov_in.asList();
                if(cov_in_bot->size() < 36)
                {
                  PRINTF_DEBUG("Error Parsing Bottle: Covariance list is too short!\n");
                  bot.addString("Error Parsing Bottle: Covariance list is too short!\n");
                  break;
                }
                for(int r = 0; r < width; r++)
                {
                  for(int c = 0; c < width; c++)
                  {
                    yarp::os::Value& val = cov_in_bot->get(r * width + c);
                    if(val.isDouble())
                      cov.element(r,c) = val.asDouble();
                    else if(val.isInt())
                      cov.element(r,c) = val.asInt();
                    else
                      cov.element(r,c) = 0;
                  }
                }
                
                if(update)
                {
                  ServiceLocatedObject* pose = GetServiceLocatedObject(id.asInt());
                  if(pose == NULL)
                  {
                    ServiceLocatedObject* parent = GetServiceLocatedObject(parentID);
                    if(parent == NULL)
                    {
                      PRINTF_DEBUG("Error Parsing Bottle: Requested parent does not exist!\n");
                      bot.addString("Error Parsing Bottle: Requested parent does not exist!\n");
                      break;
                    }
                    PutIntoBottle(FServiceLocatedObject(parent, mat, cov, type), bot);
                  }
                  else
                  {
                    ServiceLocatedObject* parent = GetServiceLocatedObject(parentID);
                    if(parent == NULL || (parent->m_uniqueID == ID_WORLD && pose->m_uniqueID == ID_WORLD))
                    {
                      PRINTF_DEBUG("Error Parsing Bottle: Requested parent does not exist!\n");
                      bot.addString("Error Parsing Bottle: Requested parent does not exist!\n");
                      break;
                    }
                    pose->Update(mat, cov, ServiceInterface::FServiceLocatedObjectCopy);
                    PutIntoBottle(pose, bot);
                  }
                }
                else
                {
                  ServiceLocatedObject* parent = GetServiceLocatedObject(parentID);
                  if(parent == NULL)
                  {
                    PRINTF_DEBUG("Error Parsing Bottle: Requested parent does not exist!\n");
                    bot.addString("Error Parsing Bottle: Requested parent does not exist!\n");
                    break;
                  }
                  PutIntoBottle(FServiceLocatedObject(parent, mat, cov, type), bot);
                }
              }
              else
              {
                PRINTF_DEBUG("Error Parsing Bottle: Wrong Type in bottle for setting lo!\n");
                bot.addString("Error Parsing Bottle: Wrong Type in bottle for setting lo!\n");
              }
          }
          break;
        }
        return true;
  }

YarpService::~YarpService()
{
  g_ListenPort.close();
}

void YarpService::PutIntoBottle(ServiceLocatedObject* pose, Bottle& bot)
{
  pose->IncreaseReferenceCounter();
  bot.addInt(pose->m_uniqueID);
  if(pose->m_uniqueID != ID_WORLD)
      bot.addInt(pose->m_parentID);
  else
    bot.addInt(pose->m_uniqueID);
  Matrix m = pose->GetMatrix();
  Matrix cov = pose->GetCovarianceMatrix();
  /* Bottle per matrix*/
  int width = 4;
  yarp::os::Bottle &mat_b = bot.addList();;
  for(int r = 0; r < width; r++)
  {
    for(int c = 0; c < width; c++)
    {
        mat_b.addDouble(m.element(r,c));
    }
  }
  
  width = 6;
  yarp::os::Bottle &cov_b = bot.addList();;
  for(int r = 0; r < width; r++)
  {
    for(int c = 0; c < width; c++)
    {
      cov_b.addDouble( cov.element(r,c));
    }
  }
return;
}

} /* end namespace jlo*/
