//////////////////////////////////////////////////////////////////////////////
//
// for class: UT_Timer
//
// Copyright (c) Frank Althof, August 2000
// Lehrstuhl fuer Mensch-Maschine-Kommunikation
// Technische Universitaet Muenchen
// Arcisstrasse 21, D-80290 Muenchen
//
//////////////////////////////////////////////////////////////////////////////
#ifndef WIN32
/***************************   include   ************************************/

#include "lo/UT_Timer.h"
#include <iostream>
#include <iomanip>

/**************************   constructors   ********************************/

UT_Timer::UT_Timer( void )
{
  gettimeofday( &tv_start, NULL );
  gettimeofday( &tv_end, NULL );

  iClkTck = sysconf( _SC_CLK_TCK );
}

/*******************   destructor   *****************************************/

/******************   other element functions   *****************************/

double UT_Timer::end( char *message )
{
  gettimeofday( &tv_end, NULL );
  usecs    = tv_end.tv_usec - tv_start.tv_usec;
  secs     = tv_end.tv_sec  - tv_start.tv_sec;
  ges_usecs = 1000000*secs  + usecs;
  if( message!=NULL )
    std::cerr << message << "  " << std::setw(10) << std::setfill(' ')
         << 0.001*ges_usecs << " ms";
  return  0.001*ges_usecs;
}

#endif

/*********************   END OF FILE *********************************/
