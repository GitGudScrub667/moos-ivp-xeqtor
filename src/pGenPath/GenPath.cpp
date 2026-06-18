/************************************************************/
/*    NAME: XeQtor                                          */
/*    ORGN: MIT 2.680                                       */
/*    FILE: GenPath.cpp                                     */
/*    DATE: 2026                                            */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "ACTable.h"
#include "XYSegList.h"
#include "GenPath.h"

using namespace std;

//---------------------------------------------------------
// Constructor()

GenPath::GenPath()
{
  m_wpt_update_var = "WPT_UPDATE";
  m_visit_radius   = 3;

  m_collecting       = false;
  m_path_generated   = false;
  m_visited_count    = 0;
  m_first_received   = false;
  m_last_received    = false;
  m_total_received   = 0;
  m_invalid_received = 0;

  m_nav_x        = 0;
  m_nav_y        = 0;
  m_nav_received = false;
}

//---------------------------------------------------------
// Destructor

GenPath::~GenPath()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool GenPath::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    string key = msg.GetKey();

    if(key == "VISIT_POINT")
      handleVisitPoint(msg.GetString());

    else if(key == "NAV_X") {
      m_nav_x = msg.GetDouble();
      m_nav_received = true;
    }
    else if(key == "NAV_Y")
      m_nav_y = msg.GetDouble();

    else if(key != "APPCAST_REQ") // handled by AppCastingMOOSApp
      reportRunWarning("Unhandled Mail: " + key);
  }

  return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool GenPath::OnConnectToServer()
{
  registerVariables();
  return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool GenPath::Iterate()
{
  AppCastingMOOSApp::Iterate();

  // Handshake (lab 3.6.1): announce readiness to the shoreside pPointAssign
  // until points start arriving. Repeating it (rather than a one-shot) survives
  // the shore->vehicle bridge still coming up: pPointAssign holds the point
  // burst until it has heard PGENPATH_READY from every vehicle.
  if(!m_first_received)
    Notify("PGENPATH_READY", m_host_community);

  updateVisited();
  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool GenPath::OnStartUp()
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
    if(param == "wpt_update_var") {
      m_wpt_update_var = value;
      handled = true;
    }
    else if(param == "visit_radius") {
      handled = setDoubleOnString(m_visit_radius, value);
    }

    if(!handled)
      reportUnhandledConfigWarning(orig);
  }

  registerVariables();
  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables()

void GenPath::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register("VISIT_POINT", 0);
  Register("NAV_X", 0);
  Register("NAV_Y", 0);
}

//---------------------------------------------------------
// Procedure: handleVisitPoint()
//   One incoming VISIT_POINT message. The shoreside brackets the set with
//   "firstpoint" / "lastpoint"; in between are "x=..,y=..,id=.." points.
//     firstpoint -> start a fresh set
//     x=..,y=..   -> store the point
//     lastpoint   -> the set is complete (tour is built in the next step)

void GenPath::handleVisitPoint(const string& sval)
{
  string val = stripBlankEnds(sval);

  if(val == "firstpoint") {
    m_collecting       = true;
    m_first_received   = true;
    m_last_received    = false;
    m_points.clear();
    m_total_received   = 0;
    m_invalid_received = 0;
    return;
  }

  if(val == "lastpoint") {
    m_last_received = true;
    m_collecting    = false;
    generatePath();
    return;
  }

  // Otherwise it should be a point "x=..,y=..,id=..".
  if(!m_collecting) {
    reportRunWarning("Point received outside firstpoint/lastpoint: " + val);
    return;
  }

  double x = 0, y = 0;
  if(!tokParse(val, "x", ',', '=', x) || !tokParse(val, "y", ',', '=', y)) {
    m_invalid_received++;
    reportRunWarning("Malformed point: " + val);
    return;
  }

  string id;
  tokParse(val, "id", ',', '=', id);

  XYPoint pt(x, y);
  if(id != "")
    pt.set_label(id);
  m_points.push_back(pt);
  m_total_received++;
}

//---------------------------------------------------------
// Procedure: generatePath()
//   Greedy nearest-neighbor tour: starting from the vehicle's current
//   position, repeatedly hop to the closest not-yet-used point. The
//   resulting order is packed into an XYSegList and posted as a
//   "points = ..." update to the waypoint behavior.

void GenPath::generatePath()
{
  unsigned int n = m_points.size();
  if(n == 0)
    return;

  vector<bool> used(n, false);
  double cx = m_nav_x;   // virtual start = where the vehicle is right now
  double cy = m_nav_y;

  XYSegList seglist;
  m_tour.clear();

  for(unsigned int step=0; step<n; step++) {
    int    best      = -1;
    double best_dist = 0;
    for(unsigned int i=0; i<n; i++) {
      if(used[i])
        continue;
      double dx = m_points[i].x() - cx;
      double dy = m_points[i].y() - cy;
      double d  = (dx*dx) + (dy*dy);  // squared dist: fine for comparing, no sqrt
      if((best < 0) || (d < best_dist)) {
        best      = (int)i;
        best_dist = d;
      }
    }

    XYPoint pt = m_points[best];
    used[best] = true;
    seglist.add_vertex(pt.x(), pt.y());
    m_tour.push_back(pt);
    cx = pt.x();   // advance the "current position" to the chosen point
    cy = pt.y();
  }

  m_visited.assign(n, false);   // fresh tour -> nothing visited yet
  m_visited_count = 0;

  string update_str = "points = " + seglist.get_spec();
  Notify(m_wpt_update_var, update_str);
  m_path_generated = true;
}

//---------------------------------------------------------
// Procedure: updateVisited()
//   Each tick, mark any not-yet-visited tour point that the vehicle is now
//   within m_visit_radius of. Runs only once a tour exists and we have a
//   position fix.

void GenPath::updateVisited()
{
  if(!m_path_generated || !m_nav_received)
    return;

  double r2 = m_visit_radius * m_visit_radius;  // compare squared distances
  for(unsigned int i=0; i<m_tour.size(); i++) {
    if(m_visited[i])
      continue;
    double dx = m_tour[i].x() - m_nav_x;
    double dy = m_tour[i].y() - m_nav_y;
    if((dx*dx + dy*dy) <= r2) {
      m_visited[i] = true;
      m_visited_count++;
    }
  }
}

//------------------------------------------------------------
// Procedure: buildReport()

bool GenPath::buildReport()
{
  m_msgs << "Visit Radius:            " << m_visit_radius            << endl;
  m_msgs << "Total Points Received:   " << m_total_received          << endl;
  m_msgs << "Invalid Points Received: " << m_invalid_received        << endl;
  m_msgs << "First Point Received:    " << boolToString(m_first_received) << endl;
  m_msgs << "Last Point Received:     " << boolToString(m_last_received)  << endl;
  m_msgs << "NAV_X/Y Received:        " << boolToString(m_nav_received)   << endl;
  m_msgs << endl;
  m_msgs << "Path posted to " << m_wpt_update_var << ": "
         << boolToString(m_path_generated) << endl;
  m_msgs << endl;

  unsigned int unvisited = m_tour.size() - m_visited_count;
  m_msgs << "Tour Status"              << endl;
  m_msgs << "------------------------" << endl;
  m_msgs << "   Points Visited:    " << m_visited_count << endl;
  m_msgs << "   Points Unvisited:  " << unvisited       << endl;
  return(true);
}
