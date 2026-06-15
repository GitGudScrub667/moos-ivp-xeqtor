/************************************************************/
/*    NAME: XeQtor                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: Odometry.cpp                                        */
/*    DATE: June 15th, 2026                             */
/************************************************************/

#include <iterator>
#include <cmath>
#include "MBUtils.h"
#include "ACTable.h"
#include "Odometry.h"

using namespace std;

//---------------------------------------------------------
// Constructor()

Odometry::Odometry()
{
  // State variables
  m_first_reading  = true;
  m_nav_received   = false;

  m_current_x      = 0;
  m_current_y      = 0;
  m_previous_x     = 0;
  m_previous_y     = 0;

  m_total_distance = 0;
}

//---------------------------------------------------------
// Destructor

Odometry::~Odometry()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool Odometry::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    string key    = msg.GetKey();

#if 0 // Keep these around just for template
    string comm  = msg.GetCommunity();
    double dval  = msg.GetDouble();
    string sval  = msg.GetString(); 
    string msrc  = msg.GetSource();
    double mtime = msg.GetTime();
    bool   mdbl  = msg.IsDouble();
    bool   mstr  = msg.IsString();
#endif

     if(key == "NAV_X") {
       m_current_x    = msg.GetDouble();
       m_nav_received = true;
     }
     else if(key == "NAV_Y") {
       m_current_y    = msg.GetDouble();
       m_nav_received = true;
     }

     else if(key != "APPCAST_REQ") // handled by AppCastingMOOSApp
       reportRunWarning("Unhandled Mail: " + key);
   }
	
   return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool Odometry::OnConnectToServer()
{
   registerVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool Odometry::Iterate()
{
  AppCastingMOOSApp::Iterate();

  // Can't measure anything until the first NAV position has arrived.
  if(m_nav_received) {

    // On the very first reading, just seed the "previous" position.
    // There is no prior point yet, so no leg distance is added.
    if(m_first_reading) {
      m_previous_x    = m_current_x;
      m_previous_y    = m_current_y;
      m_first_reading = false;
    }
    // Otherwise add the length of the leg from the previous position
    // (last Iterate) to the current position to the running total.
    else {
      double dx        = m_current_x - m_previous_x;
      double dy        = m_current_y - m_previous_y;
      double leg_dist  = hypot(dx, dy);

      m_total_distance += leg_dist;

      m_previous_x = m_current_x;
      m_previous_y = m_current_y;
    }

    Notify("ODOMETRY_DIST", m_total_distance);
  }

  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool Odometry::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp();

  STRING_LIST sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  if(!m_MissionReader.GetConfiguration(GetAppName(), sParams))
    reportConfigWarning("No config block found for " + GetAppName());

  STRING_LIST::iterator p;
  for(p=sParams.begin(); p!=sParams.end(); p++) {
    string orig  = *p;
    string line  = *p;
    string param = tolower(biteStringX(line, '='));
    string value = line;

    bool handled = false;
    if(param == "foo") {
      handled = true;
    }
    else if(param == "bar") {
      handled = true;
    }

    if(!handled)
      reportUnhandledConfigWarning(orig);

  }
  
  registerVariables();	
  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables()

void Odometry::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register("NAV_X", 0);
  Register("NAV_Y", 0);
}


//------------------------------------------------------------
// Procedure: buildReport()

bool Odometry::buildReport()
{
  m_msgs << "Odometry Report                             " << endl;
  m_msgs << "============================================" << endl;

  if(!m_nav_received) {
    m_msgs << "Awaiting first NAV_X/NAV_Y reading..." << endl;
    return(true);
  }

  ACTable actab(2);
  actab << "Quantity | Value";
  actab.addHeaderLines();
  actab << "Current Position:" << "(" + doubleToStringX(m_current_x,2) + ", " +
                                        doubleToStringX(m_current_y,2) + ")";
  actab << "Total Distance:"   << doubleToStringX(m_total_distance,2) + " m";
  m_msgs << actab.getFormattedString();

  return(true);
}




