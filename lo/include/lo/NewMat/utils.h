#ifndef NEW_MAT_UTILS_H
#define NEW_MAT_UTILS_H

/*!
  @file utils.h
  @brief Utility header file.
*/

//! @brief RCS/CVS version.
static const char header_utils_rcsid[] = "$Id: utils.h,v 1.10 2006/05/16 16:11:15 gourdeau Exp $";

#ifdef _MSC_VER                         // Microsoft
#pragma warning (disable:4786)  /* Disable decorated name truncation warnings */
#endif
#include <stdio.h>
#include <limits>
#define WANT_STRING                  /* include.h will get string fns */
#define WANT_STREAM                  /* include.h will get stream fns */
#define WANT_FSTREAM                 /* include.h will get fstream fns */
#define WANT_MATH                    /* include.h will get math fns */
                                     /* newmatap.h will get include.h */
#endif

