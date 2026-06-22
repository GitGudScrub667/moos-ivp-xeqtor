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
#include "AngleUtils.h"
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
  m_nav_heading = 0;
  m_nav_x_set = 0;
  m_nav_y_set = 0;
  m_nav_heading_set = false;
  m_plan_pending = false;

  // Cluster-aware path tuning (starting values; tune by rebuild).
  m_cluster_radius = 20;   // meters: radius for counting neighbors
  m_cluster_weight = 0.6;  // 0 = plain greedy; higher = favor packs more

  // Time-to-target tuning (starting values; tune by rebuild).
  m_speed     = 1.2;       // m/s: assumed transit speed (survey speed)
  m_turn_rate = 30;        // deg/s: <= 0 disables the heading penalty
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
    else if(key == "NAV_HEADING") {
      m_nav_heading = msg.GetDouble();
      m_nav_heading_set = true;
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

  // Replan when a new swimmer has arrived (or one was removed), once we
  // actually have a NAV fix. If there are no swimmers left to plan for
  // we still clear the flag so it can't stick "true" after the last
  // rescue.
  if(m_plan_pending && m_nav_x_set && m_nav_y_set) {
    if(m_swimmers.size() > 0)
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
  Register("NAV_HEADING", 0);
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
  // it), remembering the finder for the scoreboard. Drop it from the
  // active set and flag a replan so our tour stops routing toward a
  // swimmer that no longer needs saving.
  m_rescued[id] = finder;
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
  // ownship's position and heading. Like nearest-neighbor, but cost
  // is travel+turn time discounted by local swimmer density, so we
  // dive into packs first and avoid U-turns to swimmers behind us.
  m_path = clusterPath(swim_pts, m_nav_x, m_nav_y, m_nav_heading);
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

XYSegList GenRescue::clusterPath(XYSegList swim_pts, double sx, double sy, double sh)
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

  // Working heading walks along the tour: it starts at the boat's real
  // heading (if known), then becomes the direction of each leg we commit.
  double cur_h  = sh;
  bool   have_h = m_nav_heading_set;

  XYSegList new_segl;

  // Build the tour one vertex at a time.
  for(i=0; i<vsize; i++) {
    double       best_score = -1;   // -1 = not yet set
    unsigned int best_ix    = 0;

    for(j=0; j<vsize; j++) {
      if(visited[j])
        continue;

      // Time to reach j = travel time + time spent turning toward it.
      double d           = distPointToPoint(sx, sy, vx[j], vy[j]);
      double travel_time = d / m_speed;

      double turn_time = 0;
      if(have_h && (m_turn_rate > 0)) {
        double brg  = relAng(sx, sy, vx[j], vy[j]);  // bearing to candidate
        double aoff = angleDiff(cur_h, brg);         // [0,180] off the bow
        turn_time   = aoff / m_turn_rate;
      }
      double cost = travel_time + turn_time;

      // Count OTHER still-unvisited swimmers within radius of j.
      unsigned int neighbors = 0;
      for(k=0; k<vsize; k++) {
        if((k == j) || visited[k])
          continue;
        if(distPointToPoint(vx[j], vy[j], vx[k], vy[k]) <= m_cluster_radius)
          neighbors++;
      }

      // Discount the time-to-target by local density.
      double score = cost / (1.0 + (m_cluster_weight * neighbors));

      if((best_score < 0) || (score < best_score)) {
        best_score = score;
        best_ix    = j;
      }
    }

    // Commit the winning vertex. The leg we just traveled defines the
    // heading we will arrive on, so use it for the next step's turn cost.
    if(best_score >= 0) {
      double bx = vx[best_ix];
      double by = vy[best_ix];
      new_segl.add_vertex(bx, by);
      visited[best_ix] = true;
      cur_h  = relAng(sx, sy, bx, by);
      have_h = true;
      sx = bx;
      sy = by;
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
  string vname = m_host_community;

  // Tally rescues by us vs by an opponent (finder == ownship name).
  unsigned int resc_us = 0, resc_other = 0;
  map<string, string>::iterator rp;
  for(rp=m_rescued.begin(); rp!=m_rescued.end(); rp++) {
    if(rp->second == vname) resc_us++;
    else                    resc_other++;
  }

  // --- Section 1: Strategy (live view of the path-tuning constants) ---
  m_msgs << "Strategy (path tuning)"                                  << endl;
  m_msgs << "  Cluster radius / weight:  " << doubleToStringX(m_cluster_radius,1)
         << " / " << doubleToStringX(m_cluster_weight,2)              << endl;
  m_msgs << "  Speed / turn rate:        " << doubleToStringX(m_speed,2)
         << " m/s / " << doubleToStringX(m_turn_rate,1) << " deg/s"   << endl;
  m_msgs << endl;

  // --- Section 2: Rescue progress (the scoreboard) ---
  m_msgs << "Rescue progress"                                         << endl;
  m_msgs << "  Vehicle:          " << vname;
  if(m_nav_x_set && m_nav_y_set) {
    m_msgs << "  pos=(" << doubleToStringX(m_nav_x,1) << ", "
           << doubleToStringX(m_nav_y,1) << ")";
    if(m_nav_heading_set)
      m_msgs << " hdg=" << doubleToStringX(m_nav_heading,0);
  }
  m_msgs << endl;
  m_msgs << "  Swimmers active:  " << m_swimmers.size()               << endl;
  m_msgs << "  Rescued total:    " << m_rescued.size()
         << "   (by us: " << resc_us << ", by opponent: " << resc_other << ")" << endl;
  m_msgs << "  Plan pending:     " << boolToString(m_plan_pending)    << endl;

  // Next planned target = head of the current path; resolve its swimmer
  // id by matching coordinates back to the active swimmer set.
  if(m_path.size() > 0) {
    double fx = m_path.get_vx(0);
    double fy = m_path.get_vy(0);
    string tid = "?";
    map<string, XYPoint>::iterator sp;
    for(sp=m_swimmers.begin(); sp!=m_swimmers.end(); sp++) {
      if((sp->second.x() == fx) && (sp->second.y() == fy)) {
        tid = sp->first;
        break;
      }
    }
    m_msgs << "  Next target:      id=" << tid << " at ("
           << doubleToStringX(fx,1) << ", " << doubleToStringX(fy,1) << ")";
    if(m_nav_x_set && m_nav_y_set)
      m_msgs << "  range " << doubleToStringX(distPointToPoint(m_nav_x,m_nav_y,fx,fy),0) << " m";
    m_msgs << endl;
  }
  m_msgs << "  Path legs:        " << m_path.size()                   << endl;
  m_msgs << endl;

  // --- Section 3: Active swimmers (id | location | range-from-ownship) ---
  m_msgs << "Active swimmers"                                         << endl;
  ACTable actab(3);
  actab << "ID | Location (x, y) | Range";
  actab.addHeaderLines();
  map<string, XYPoint>::iterator it;
  for(it=m_swimmers.begin(); it!=m_swimmers.end(); it++) {
    string  id  = it->first;
    XYPoint pt  = it->second;
    string  loc = doubleToStringX(pt.x(),1) + ", " + doubleToStringX(pt.y(),1);
    string  rng = "--";
    if(m_nav_x_set && m_nav_y_set)
      rng = doubleToStringX(distPointToPoint(m_nav_x,m_nav_y,pt.x(),pt.y()),0);
    actab << id << loc << rng;
  }
  m_msgs << actab.getFormattedString();
  m_msgs << endl << endl;

  // --- Section 4: Rescued swimmers (id | finder) ---
  m_msgs << "Rescued swimmers"                                        << endl;
  ACTable rtab(2);
  rtab << "ID | Finder";
  rtab.addHeaderLines();
  for(rp=m_rescued.begin(); rp!=m_rescued.end(); rp++) {
    string finder = rp->second;
    if(finder == "")
      finder = "(unknown)";
    rtab << rp->first << finder;
  }
  m_msgs << rtab.getFormattedString();

  return(true);
}
