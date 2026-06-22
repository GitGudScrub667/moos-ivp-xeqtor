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

  // Cluster-aware path tuning (starting values; tune by rebuild).
  m_cluster_radius = 20;   // meters: radius for counting neighbors
  m_cluster_weight = 0.6;  // 0 = plain greedy; higher = favor packs more
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

  // Already rescued? Ignore forever. SWIMMER_ALERT keeps repeating
  // every ~15s even after a swimmer is saved, so without this check
  // a found swimmer would be re-added and re-enter the path.
  if(m_rescued.count(id) != 0)
    return(true);

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
  // Expected format:  id=04, finder=abe
  string id;
  string finder;

  vector<string> svector = parseString(str, ',');
  for(unsigned int i=0; i<svector.size(); i++) {
    string param = tolower(biteStringX(svector[i], '='));
    string value = svector[i];
    if(param == "id")
      id = value;
    else if(param == "finder")
      finder = value;
  }

  // Malformed alert (no id): reject so it logs a run warning.
  if(id == "")
    return(false);

  // We don't know this swimmer (never alerted, or already removed):
  // nothing to do, but the mail itself was well-formed.
  if(m_swimmers.count(id) == 0)
    return(true);

  // The swimmer has been rescued -- by us OR by the opponent. Record
  // it as rescued forever (so a repeating SWIMMER_ALERT can't revive
  // it), drop it from the active set, and flag a replan so our tour
  // stops routing toward a swimmer that no longer needs saving.
  m_rescued.insert(id);
  m_swimmers.erase(id);
  m_plan_pending = true;

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

  // Order the swimmer points into a short tour, starting from
  // ownship's position. Like nearest-neighbor, but distance is
  // discounted by local swimmer density so we dive into packs first.
  m_path = clusterPath(swim_pts, m_nav_x, m_nav_y);
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
// Procedure: clusterPath()
//   Like greedyPath (nearest-neighbor), but at each step a
//   candidate's distance is DISCOUNTED by how many other unvisited
//   swimmers sit within m_cluster_radius of it. This biases the boat
//   toward diving into dense packs first instead of chasing a lone
//   nearby swimmer. With m_cluster_weight = 0 it reduces to greedy.

XYSegList GenRescue::clusterPath(XYSegList swim_pts, double sx, double sy)
{
  unsigned int i, j, k, vsize = swim_pts.size();

  // Pull vertices into plain arrays + a visited flag (mirrors greedyPath).
  vector<double> vx, vy;
  vector<bool>   visited;
  for(i=0; i<vsize; i++) {
    vx.push_back(swim_pts.get_vx(i));
    vy.push_back(swim_pts.get_vy(i));
    visited.push_back(false);
  }

  XYSegList new_segl;

  // Build the tour one vertex at a time.
  for(i=0; i<vsize; i++) {
    double       best_score = -1;   // -1 = not yet set
    unsigned int best_ix    = 0;

    for(j=0; j<vsize; j++) {
      if(visited[j])
        continue;

      double d = distPointToPoint(sx, sy, vx[j], vy[j]);

      // Count OTHER still-unvisited swimmers within radius of j.
      unsigned int neighbors = 0;
      for(k=0; k<vsize; k++) {
        if((k == j) || visited[k])
          continue;
        if(distPointToPoint(vx[j], vy[j], vx[k], vy[k]) <= m_cluster_radius)
          neighbors++;
      }

      // Discount distance by local density. neighbors == 0 -> raw dist.
      double score = d / (1.0 + (m_cluster_weight * neighbors));

      if((best_score < 0) || (score < best_score)) {
        best_score = score;
        best_ix    = j;
      }
    }

    // Commit the winning vertex and step the "current position" to it.
    if(best_score >= 0) {
      new_segl.add_vertex(vx[best_ix], vy[best_ix]);
      visited[best_ix] = true;
      sx = vx[best_ix];
      sy = vy[best_ix];
    }
  }

  return(new_segl);
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
