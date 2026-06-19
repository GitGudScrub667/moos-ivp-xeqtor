/************************************************************/
/*    NAME: Mike Benjamin                                   */
/*    ORGN: MIT                                             */
/*    FILE: GenRescue.cpp                                   */
/*    DATE: April 18th, 2022                                */
/************************************************************/

#include <iterator>
#include "GenRescue.h"
#include "MBUtils.h"
#include "ACTable.h"
#include "ColorParse.h"
#include "XYPoint.h"
#include "XYSegList.h"
#include "GeomUtils.h"
#include "PathUtils.h"
#include "XYFormatUtilsPoly.h"
#include "XYFieldGenerator.h"

using namespace std;

//---------------------------------------------------------
// Constructor()

GenRescue::GenRescue()
{
  // Initialize state variables
  m_nav_x = 0;
  m_nav_y = 0;
  m_nav_x_set = 0;
  m_nav_y_set = 0;
  m_plan_pending = false;
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool GenRescue::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    string key  = msg.GetKey();
    string sval = msg.GetString();

    bool handled = true;
    if(key == "SWIMMER_ALERT") 
      handled = handleMailNewSwimmer(sval);
    else if(key == "FOUND_SWIMMER") 
      handled = handleMailFoundSwimmer(sval);
    else if(key == "NAV_X") {
      m_nav_x = msg.GetDouble();
      m_nav_x_set = true;
    }
    else if(key == "NAV_Y") {
      m_nav_y = msg.GetDouble();
      m_nav_y_set = true;
    }

    else if(key != "APPCAST_REQ") // handle by AppCastingMOOSApp
      handled = false;
    
    if(!handled)
      reportRunWarning("Unhandled Mail: " + key +"=" + sval);
    
  }
  return(true);
}
 
//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool GenRescue::OnConnectToServer()
{
  RegisterVariables();
  return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()

bool GenRescue::Iterate()
{
  AppCastingMOOSApp::Iterate();

  // Replan only when a new swimmer has arrived, and only once we
  // actually have a NAV fix and at least one swimmer to visit.
  if(m_plan_pending && m_nav_x_set && m_nav_y_set &&
     (m_swimmers.size() > 0)) {
    postShortestPath();
    m_plan_pending = false;
  }

  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()

bool GenRescue::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp(); 

  STRING_LIST sParams;
  m_MissionReader.GetConfiguration(GetAppName(), sParams);
  
  STRING_LIST::iterator p;
  for(p=sParams.begin(); p!=sParams.end(); p++) {
    string sLine  = *p;
    string param  = tolower(biteStringX(sLine, '='));
    string value  = sLine;
    if(param == "vname")
      m_vname = value;
  }
  
  RegisterVariables();	
  return(true);
}

//---------------------------------------------------------
// Procedure: RegisterVariables()

void GenRescue::RegisterVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register("SWIMMER_ALERT", 0);
  Register("FOUND_SWIMMER", 0);
  Register("NAV_X", 0);
  Register("NAV_Y", 0);
}


//---------------------------------------------------------
// Procedure: handleMailNewSwimmer()

bool GenRescue::handleMailNewSwimmer(string str)
{
  // Expected format:  x=23, y=54, id=04
  double xval = 0;
  double yval = 0;
  string id;
  bool   x_set = false;
  bool   y_set = false;

  vector<string> svector = parseString(str, ',');
  for(unsigned int i=0; i<svector.size(); i++) {
    string param = tolower(biteStringX(svector[i], '='));
    string value = svector[i];
    if(param == "x") {
      xval  = atof(value.c_str());
      x_set = true;
    }
    else if(param == "y") {
      yval  = atof(value.c_str());
      y_set = true;
    }
    else if(param == "id")
      id = value;
  }

  // Reject a malformed alert (missing any field)
  if(!x_set || !y_set || (id == ""))
    return(false);

  // Already known? Ignore it (alerts repeat every 15s). No replan.
  if(m_swimmers.count(id) != 0)
    return(true);

  // New swimmer: remember it and flag that we need to replan.
  XYPoint pt(xval, yval);
  pt.set_label(id);
  m_swimmers[id] = pt;
  m_plan_pending = true;

  return(true);
}

//---------------------------------------------------------
// Procedure: handleMailFoundSwimmer()

bool GenRescue::handleMailFoundSwimmer(string str)
{
  return(true);
}

//---------------------------------------------------------
// Procedure: postShortestPath()

void GenRescue::postShortestPath()
{
  // Build a seglist from every swimmer we currently know about.
  XYSegList swim_pts;
  map<string, XYPoint>::iterator it;
  for(it=m_swimmers.begin(); it!=m_swimmers.end(); it++) {
    XYPoint pt = it->second;
    swim_pts.add_vertex(pt.x(), pt.y());
  }

  // Order the swimmer points into a short tour, greedily, starting
  // from ownship's current position (nearest-neighbor ordering).
  m_path = greedyPath(swim_pts, m_nav_x, m_nav_y);
  m_path.set_label("rescue");

  // Draw the planned route in the viewer.
  Notify("VIEW_SEGLIST", m_path.get_spec());

  // Update the waypoint behavior with the new path.
  string update_var = "SURVEY_UPDATE";
  string update_str = "points = " + m_path.get_spec_pts();

  Notify(update_var, update_str);
  reportEvent("SURVEY_UPDATE=" + update_str);
}

//---------------------------------------------------------
// Procedure: postNullPath()
//   Purpose: If a found swimmer represents the last swimmer
//            to be found, then post a survey update essentially
// 

void GenRescue::postNullPath()
{
#if 0
  if(!m_nav_x_set || !m_nav_y_set)
    return;
  if(m_map_pts.size() != 0)
    return;
  
  XYSegList segl;
  segl.add_vertex(m_nav_x, m_nav_y);
  
  // Seglist needs a name, refer when drawging and erasing
  segl.set_label("one");
  Notify("VIEW_SEGLIST", segl.get_spec());

  string update_var = "SURVEY_UPDATE";
  string update_str = "points = " + segl.get_spec_pts();

  Notify(update_var, update_str);
  reportEvent("SURVEY_UPDATE=" + update_str);
#endif
}


//---------------------------------------------------------
// Procedure: buildReport()

bool GenRescue::buildReport()
{
  m_msgs << "Vehicle Name:     " << m_vname                        << endl;
  m_msgs << "Swimmers Known:   " << m_swimmers.size()              << endl;
  m_msgs << "NAV_X/Y Received: " << boolToString(m_nav_x_set && m_nav_y_set) << endl;
  m_msgs << "Plan Pending:     " << boolToString(m_plan_pending)   << endl;
  m_msgs << "Path Size:        " << m_path.size()                  << endl;
  m_msgs << endl;

  ACTable actab(2);
  actab << "Swimmer ID | Location (x, y)";
  actab.addHeaderLines();
  map<string, XYPoint>::iterator it;
  for(it=m_swimmers.begin(); it!=m_swimmers.end(); it++) {
    string  id = it->first;
    XYPoint pt = it->second;
    string  loc = doubleToStringX(pt.x(),1) + ", " + doubleToStringX(pt.y(),1);
    actab << id << loc;
  }
  m_msgs << actab.getFormattedString();

  return(true);
}
