
//////////////////////////////////////////////////////////////////////////////
//
// definition of the class :  UT_Timer
//
// Copyright (c) Frank Althof, August 2000
// slightly modified by Derik Schroeter
// Lehrstuhl fuer Mensch-Maschine-Kommunikation
// Technische Universitaet Muenchen
// Arcisstrasse 21, D-80290 Muenchen
//
// Remarks :
//     - partly based on old c code generated by Thorsten Bomberg
//     - enhanced and adapted to c++ class
//
//////////////////////////////////////////////////////////////////////////////

#ifndef _UT_TIMER_HH
#define _UT_TIMER_HH

/***************************   include   ************************************/
#ifndef WIN32
#include <unistd.h>
#include <stdio.h>
#include <time.h>
#include <sys/types.h>
#include <sys/times.h>
#include <sys/time.h>

/****************************************************************************/

/*! 
 * \class UT_Timer
 * \brief Provides time measuring functionality.
 * \author Frank Althof & Derik Schroeter
 * \version 1.0
 * \date August 2000
 */
class UT_Timer
{
 public :
  // --- constructors and destructor --------------------------------------
  UT_Timer( void );
  virtual ~UT_Timer( void ) {}

  // --- member functions -------------------------------------------------

  inline void start( void )  
         { gettimeofday(&tv_start, NULL); }
  double end( char *message=NULL );
  inline char* getDateTimeStr( void )
         { time_t t = time(NULL); return (ctime(&t)); }

 protected :
  struct timeval tv_start;
  struct timeval tv_end;
  long usecs;
  long secs;
  long ges_usecs;

  // clock tick measure originally by MarcL
  long    iClkTck;
  struct  tms currtime;
  struct  tms lasttime;
  struct  tms difftime;

  clock_t clock, start_clock;  
};

#endif
#endif /*WIN32*/

/*********************   END OF FILE *********************************/
