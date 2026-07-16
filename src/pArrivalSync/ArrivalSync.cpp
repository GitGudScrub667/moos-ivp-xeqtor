/************************************************************/
/*    NAME: skara2                                          */
/*    ORGN: moos-ivp-xeqtor                                 */
/*    FILE: ArrivalSync.cpp                                 */
/*    DATE: 2026-07-16                                      */
/************************************************************/

#include <cmath>
#include <algorithm>
#include <utility>
#include "MBUtils.h"
#include "ACTable.h"
#include "ArrivalSync.h"

using namespace std;

//---------------------------------------------------------
// Constructor

ArrivalSync::ArrivalSync()
{
  // Configuration defaults
  m_max_speed        = 3.0;
  m_min_speed        = 0.4;
  m_min_arrival_time = 8.0;
  m_capture_dist     = 6.0;
  m_stagger_time     = 5.0;
  m_deploy_var       = "DEPLOY_ALL";
  m_return_var       = "RETURN_ALL";
  m_update_var       = "SLOT_UPDATE";

  // State
  m_deployed    = false;
  m_returning   = false;
  m_staggered   = false;
  m_deploy_time = 0;
  m_curr_T      = 0;
  m_posts       = 0;
}

//---------------------------------------------------------
// Destructor

ArrivalSync::~ArrivalSync()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool ArrivalSync::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    string key = msg.GetKey();

    if(key == "NODE_REPORT")
      handleNodeReport(msg.GetString());

    else if(key == m_deploy_var) {
      bool was = m_deployed;
      m_deployed = (tolower(msg.GetString()) == "true");
      // A fresh deploy starts a new run-in: everyone is un-arrived again.
      if(m_deployed && !was) {
        for(unsigned int i=0; i<m_vehicles.size(); i++)
          m_arrived[m_vehicles[i]] = false;
      }
    }
    else if(key == m_return_var)
      m_returning = (tolower(msg.GetString()) == "true");

    else if(key != "APPCAST_REQ")
      reportRunWarning("Unhandled Mail: " + key);
  }

  return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool ArrivalSync::OnConnectToServer()
{
  registerVariables();
  return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//   Closed-loop: every tick, for each boat still running in, set
//   its speed so all boats share one arrival time T. T is paced by
//   the farthest remaining boat running at m_max_speed; everyone
//   else is scaled down to match, so ETAs are equal.

bool ArrivalSync::Iterate()
{
  AppCastingMOOSApp::Iterate();

  bool active = m_deployed && !m_returning;

  if(active && (m_max_speed > 0)) {

    // First active tick of this deploy: fix the deploy time and the
    // staggered release schedule (farthest boat released first).
    if(!m_staggered) {
      m_deploy_time = m_curr_time;
      computeReleaseOffsets();
      m_staggered = true;
    }
    double elapsed = m_curr_time - m_deploy_time;

    // Pass 1: for each boat not yet arrived: mark arrivals, hold the
    // boats whose release time hasn't come (speed 0), and collect the
    // rest as "running" (already released, still transiting).
    vector<string> running;
    double dmax = 0;
    for(unsigned int i=0; i<m_vehicles.size(); i++) {
      string v = m_vehicles[i];
      if(!m_have_nav[v] || m_arrived[v])
        continue;

      double dx = m_slot_x[v] - m_nav_x[v];
      double dy = m_slot_y[v] - m_nav_y[v];
      double d  = hypot(dx, dy);
      m_dist[v] = d;

      if(d <= m_capture_dist) {   // close enough: let the behavior capture + orbit
        m_arrived[v] = true;
        commandSpeed(v, 0.0);     // clear any leftover command
        continue;
      }
      if(elapsed < m_release_offset[v]) {  // not released yet: hold at the cluster
        commandSpeed(v, 0.0);
        continue;
      }
      running.push_back(v);
      if(d > dmax)
        dmax = d;
    }

    // Pass 2: common arrival time from the farthest RELEASED boat, then
    // per-boat speed so all released boats share that arrival time.
    if(!running.empty()) {
      double T = dmax / m_max_speed;
      if(T < m_min_arrival_time)
        T = m_min_arrival_time;
      m_curr_T = T;

      for(unsigned int i=0; i<running.size(); i++) {
        string v = running[i];
        double spd = m_dist[v] / T;
        if(spd > m_max_speed) spd = m_max_speed;
        if(spd < m_min_speed) spd = m_min_speed;
        commandSpeed(v, spd);
      }
    }
  }
  else {
    // Not active (idle or returning): re-arm the stagger for the next deploy.
    m_staggered = false;
    m_curr_T = 0;
  }

  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: computeReleaseOffsets()
//   Assign each boat a release delay (secs after deploy). The boat with
//   the farthest run-in leaves first, the nearest leaves last, spaced
//   m_stagger_time apart. Farthest-first keeps the group synced: the
//   longest-transit boat stays the pace-setter, so no short-hop boat
//   reaches the ring ahead of the pack.

void ArrivalSync::computeReleaseOffsets()
{
  // (distance, vname) for every boat with a known position.
  vector<pair<double,string> > order;
  for(unsigned int i=0; i<m_vehicles.size(); i++) {
    string v = m_vehicles[i];
    m_release_offset[v] = 0;   // default: release immediately
    if(!m_have_nav[v])
      continue;
    double d = hypot(m_slot_x[v]-m_nav_x[v], m_slot_y[v]-m_nav_y[v]);
    order.push_back(make_pair(d, v));
  }

  // Sort by distance descending (farthest first).
  sort(order.begin(), order.end(),
       [](const pair<double,string>& a, const pair<double,string>& b){
         return a.first > b.first; });

  for(unsigned int k=0; k<order.size(); k++)
    m_release_offset[order[k].second] = k * m_stagger_time;
}

//---------------------------------------------------------
// Procedure: commandSpeed()
//   Post a run-in speed for one boat, throttled so we only notify when
//   the command meaningfully changes.

void ArrivalSync::commandSpeed(const string& v, double spd)
{
  bool have_last = (m_cmd_speed.count(v) > 0);
  if(!have_last || fabs(spd - m_cmd_speed[v]) > 0.05) {
    Notify(m_update_var + "_" + toupper(v), "speed=" + doubleToStringX(spd, 2));
    m_cmd_speed[v] = spd;
    m_posts++;
  }
}

//---------------------------------------------------------
// Procedure: handleNodeReport()
//   Pull NAME, X, Y out of a NODE_REPORT and store this boat's
//   latest position.

void ArrivalSync::handleNodeReport(const string& report)
{
  string name;
  double x = 0, y = 0;
  bool ok = true;
  ok = ok && tokParse(report, "NAME", ',', '=', name);
  ok = ok && tokParse(report, "X",    ',', '=', x);
  ok = ok && tokParse(report, "Y",    ',', '=', y);
  if(!ok)
    return;

  name = tolower(stripBlankEnds(name));
  m_nav_x[name]    = x;
  m_nav_y[name]    = y;
  m_have_nav[name] = true;
}

//---------------------------------------------------------
// Procedure: addVehicle()
//   Parse a config line "name, slot_x, slot_y", e.g. "abe, 87.43, -105".

bool ArrivalSync::addVehicle(const string& value)
{
  vector<string> parts = parseString(value, ',');
  if(parts.size() != 3)
    return(false);

  string name = tolower(stripBlankEnds(parts[0]));
  string sx   = stripBlankEnds(parts[1]);
  string sy   = stripBlankEnds(parts[2]);
  if(name == "" || !isNumber(sx) || !isNumber(sy))
    return(false);

  m_vehicles.push_back(name);
  m_slot_x[name]   = atof(sx.c_str());
  m_slot_y[name]   = atof(sy.c_str());
  m_arrived[name]  = false;
  m_have_nav[name] = false;
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()

bool ArrivalSync::OnStartUp()
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
    if(param == "max_speed")
      handled = setDoubleOnString(m_max_speed, value);
    else if(param == "min_speed")
      handled = setDoubleOnString(m_min_speed, value);
    else if(param == "min_arrival_time")
      handled = setDoubleOnString(m_min_arrival_time, value);
    else if(param == "capture_dist")
      handled = setDoubleOnString(m_capture_dist, value);
    else if(param == "stagger_time")
      handled = setDoubleOnString(m_stagger_time, value);
    else if(param == "deploy_var") {
      m_deploy_var = value; handled = true;
    }
    else if(param == "return_var") {
      m_return_var = value; handled = true;
    }
    else if(param == "update_var") {
      m_update_var = value; handled = true;
    }
    else if(param == "vehicle") {
      handled = addVehicle(value);
      if(!handled)
        reportConfigWarning("Bad vehicle line (need name, x, y): " + orig);
    }

    if(!handled)
      reportUnhandledConfigWarning(orig);
  }

  if(m_vehicles.empty())
    reportConfigWarning("No vehicle configured: nothing to synchronize.");

  registerVariables();
  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables()

void ArrivalSync::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register("NODE_REPORT", 0);
  Register(m_deploy_var, 0);
  Register(m_return_var, 0);
}

//------------------------------------------------------------
// Procedure: buildReport()

bool ArrivalSync::buildReport()
{
  string phase = "IDLE";
  if(m_deployed && m_returning) phase = "RETURNING (paused)";
  else if(m_deployed)           phase = "RUN-IN (active)";

  double elapsed = (m_staggered ? (m_curr_time - m_deploy_time) : 0);

  m_msgs << "Phase:          " << phase << endl;
  m_msgs << "max_speed:      " << doubleToStringX(m_max_speed,2) << " m/s" << endl;
  m_msgs << "stagger_time:   " << doubleToStringX(m_stagger_time,1) << " s (farthest released first)" << endl;
  m_msgs << "elapsed:        " << doubleToStringX(elapsed,1) << " s since deploy" << endl;
  m_msgs << "common ETA (T): " << doubleToStringX(m_curr_T,1) << " s" << endl;
  m_msgs << "speed cmds sent:" << m_posts << endl;
  m_msgs << "============================================" << endl;

  ACTable actab(6);
  actab << "Boat | Slot(x,y) | Dist(m) | Release@ | Cmd Spd | State";
  actab.addHeaderLines();
  for(unsigned int i=0; i<m_vehicles.size(); i++) {
    string v = m_vehicles[i];
    string slot = doubleToStringX(m_slot_x[v],1) + "," + doubleToStringX(m_slot_y[v],1);
    string dist = m_dist.count(v) ? doubleToStringX(m_dist[v],1) : "-";
    string rel  = m_release_offset.count(v) ? (doubleToStringX(m_release_offset[v],0)+"s") : "-";
    string spd  = m_cmd_speed.count(v) ? doubleToStringX(m_cmd_speed[v],2) : "-";
    string st;
    if(!m_have_nav[v])          st = "no-report";
    else if(m_arrived[v])       st = "ARRIVED";
    else if(m_staggered && (elapsed < m_release_offset[v])) st = "held";
    else                        st = "running";
    actab << v << slot << dist << rel << spd << st;
  }
  m_msgs << actab.getFormattedString();

  return(true);
}
