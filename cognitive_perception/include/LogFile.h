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


#ifdef LOGFILE
#ifndef LOGFILE_H
#define LOGFILE_H

#include "XMLTag.h"

#define XML_NODE_LOGFILE "LogFile"
#define XML_NODE_LOGENTRY "LogEntry"

#define XML_ATTRIBUTE_TIMESTAMP_LOG "Time"
#define XML_ATTRIBUTE_CALLER    "Caller"
#define XML_ATTRIBUTE_DURATION  "Duration"
#define XML_ATTRIBUTE_SUCCESS   "Success"

#define XML_NODE_ACTION_DESCRIPTION     "ActionName"
#define XML_NODE_RELEATED_ELEM     "RelatedElem"


namespace cop
{
  /**
  *   class LogFile
  *   @brief Class to log data into a given file
  */
  class LogFile
  {
  public:
      LogFile(std::string filename);
      ~LogFile(void);

      /**
      *   ReportError
      *   @brief Printf the Error info to screen, report to an eventuall Error Node
      *   @param actionName string identifier for the action that was performed, mainly detection, or learning
      *   @param caller string identifier of the caller
      *   @param timestamp when did this happen
      *   @param risk estimated risk of the action
      *   @param relatedElem Involved Element
      *   @param problemDescripion A Error summary, human readable
      *
      */
      void ReportError(std::string action_name,  std::string caller, long timestamp, double risk, Elem* relatedElem, std::string problemDescription);
      void ReportEvent(unsigned long ppid, std::string ppalg);
      /**
      *   Log
      *   @brief Write information to the Log
      *   @param actionName string identifier for the action that was performed, mainly detection, or learning
      *   @param caller string identifier of the caller
      *   @param dureation_s duration of the process in seconds
      *   @param success evaluation of the action 0.000000000000000-1.000000000000000
      *
      */
      void Log(std::string actionName,  std::string caller = "VisFinder", double duration_s = 0.0, double success = 1.0, Elem* relatedElem = NULL);
  #ifdef BOOST_THREAD
      void LogThread(std::string action_name,  std::string caller, double duration_s, double success, Elem* relatedElem);
  #endif /*BOOST_THREAD*/

  private:
      std::string m_filename;
  };
}
#endif /*LOGFILE_H*/
#endif /*LOGFILE*/
