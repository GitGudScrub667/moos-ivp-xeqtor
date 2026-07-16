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

  // Orbit phase-lock config (opt-in)
  m_enable_orbit_lock = false;
  m_circle_x    = 0;
  m_circle_y    = 0;
  m_circle_rad  = 0;
  m_orbit_speed = 2.0;
  m_orbit_gain  = 0.03;   // m/s per degree of phase error
  m_orbit_min   = 1.0;
  m_orbit_max   = 3.0;
  m_orbit_var   = "ENCIRCLE_UPDATE";

  // State
  m_deployed    = false;
  m_returning   = false;
  m_staggered   = false;
  m_deploy_time = 0;
  m_orbit_active = false;
  m_orbit_t0    = 0;
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

      if(d <= m_capture_dist) {   // close enough: hand off to the ring
        m_arrived[v] = true;
        // Clear the run-in command. If orbit-lock is on, the orbit pass
        // below takes over; otherwise the loiter runs at its own speed.
        if(!m_enable_orbit_lock)
          postThrottled(m_update_var, m_cmd_speed, v, 0.0);
        continue;
      }
      if(elapsed < m_release_offset[v]) {  // not released yet: hold at the cluster
        postThrottled(m_update_var, m_cmd_speed, v, 0.0);
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
        postThrottled(m_update_var, m_cmd_speed, v, spd);
      }
    }

    // Pass 3 (opt-in): ORBIT PHASE-LOCK. Each arrived boat tracks a
    // virtual point sweeping the ring at omega = orbit_speed/radius. Its
    // target angle is its slot phase plus omega*(t-t0); we modulate its
    // orbit speed by the phase error to null it. No neighbour coupling:
    // each boat's command depends only on its own angle + the clock.
    if(m_enable_orbit_lock && (m_circle_rad > 0)) {
      double omega_deg = (m_orbit_speed / m_circle_rad) * (180.0 / M_PI);
      for(unsigned int i=0; i<m_vehicles.size(); i++) {
        string v = m_vehicles[i];
        if(!m_have_nav[v] || !m_arrived[v])
          continue;

        // Start the shared clock at the first arrival.
        if(!m_orbit_active) {
          m_orbit_active = true;
          m_orbit_t0 = m_curr_time;
        }

        double target = slotAngleDeg(v) + omega_deg * (m_curr_time - m_orbit_t0);
        double actual = actualAngleDeg(v);
        double err = target - actual;
        while(err > 180)  err -= 360;   // wrap to [-180,180]
        while(err < -180) err += 360;
        m_phase_err[v] = err;

        double spd = m_orbit_speed + (m_orbit_gain * err);
        if(spd > m_orbit_max) spd = m_orbit_max;
        if(spd < m_orbit_min) spd = m_orbit_min;
        postThrottled(m_orbit_var, m_orbit_cmd, v, spd);
      }
    }
  }
  else {
    // Not active (idle or returning): re-arm for the next deploy.
    m_staggered = false;
    m_orbit_active = false;
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
// Procedure: postThrottled()
//   Post "<var_base>_<VNAME> = speed=X" for one boat, but only when the
//   value in 'cache' has meaningfully changed (keeps the DB quiet).

void ArrivalSync::postThrottled(const string& var_base,
                                map<string,double>& cache,
                                const string& v, double spd)
{
  bool have_last = (cache.count(v) > 0);
  if(!have_last || fabs(spd - cache[v]) > 0.05) {
    Notify(var_base + "_" + toupper(v), "speed=" + doubleToStringX(spd, 2));
    cache[v] = spd;
    m_posts++;
  }
}

//---------------------------------------------------------
// Procedure: slotAngleDeg()
//   Phase phi_i: the angle of this boat's slot on the ring, measured
//   from the ring center (E=0, N=90, W=180, S=270), in [0,360).

double ArrivalSync::slotAngleDeg(const string& v) const
{
  map<string,double>::const_iterator sx = m_slot_x.find(v);
  map<string,double>::const_iterator sy = m_slot_y.find(v);
  if(sx == m_slot_x.end() || sy == m_slot_y.end())
    return(0);
  double a = atan2(sy->second - m_circle_y, sx->second - m_circle_x) * (180.0 / M_PI);
  if(a < 0) a += 360;
  return(a);
}

//---------------------------------------------------------
// Procedure: actualAngleDeg()
//   The boat's live angle on the ring, from the ring center, in [0,360).

double ArrivalSync::actualAngleDeg(const string& v) const
{
  map<string,double>::const_iterator nx = m_nav_x.find(v);
  map<string,double>::const_iterator ny = m_nav_y.find(v);
  if(nx == m_nav_x.end() || ny == m_nav_y.end())
    return(0);
  double a = atan2(ny->second - m_circle_y, nx->second - m_circle_x) * (180.0 / M_PI);
  if(a < 0) a += 360;
  return(a);
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
    else if(param == "enable_orbit_lock")
      handled = setBooleanOnString(m_enable_orbit_lock, value);
    else if(param == "circle_x")
      handled = setDoubleOnString(m_circle_x, value);
    else if(param == "circle_y")
      handled = setDoubleOnString(m_circle_y, value);
    else if(param == "circle_rad")
      handled = setDoubleOnString(m_circle_rad, value);
    else if(param == "orbit_speed")
      handled = setDoubleOnString(m_orbit_speed, value);
    else if(param == "orbit_gain")
      handled = setDoubleOnString(m_orbit_gain, value);
    else if(param == "orbit_min")
      handled = setDoubleOnString(m_orbit_min, value);
    else if(param == "orbit_max")
      handled = setDoubleOnString(m_orbit_max, value);
    else if(param == "orbit_var") {
      m_orbit_var = value; handled = true;
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
  if(m_enable_orbit_lock) {
    m_msgs << "orbit-lock:     ON  (center " << doubleToStringX(m_circle_x,0) << ","
           << doubleToStringX(m_circle_y,0) << " r" << doubleToStringX(m_circle_rad,1)
           << ", " << doubleToStringX(m_orbit_speed,1) << " m/s, gain "
           << doubleToStringX(m_orbit_gain,3) << ") clock "
           << (m_orbit_active ? "running" : "idle") << endl;
  }
  m_msgs << "speed cmds sent:" << m_posts << endl;
  m_msgs << "============================================" << endl;

  ACTable actab(7);
  actab << "Boat | Slot(x,y) | Dist(m) | Release@ | RunIn Spd | Phase err | Orbit Spd";
  actab.addHeaderLines();
  for(unsigned int i=0; i<m_vehicles.size(); i++) {
    string v = m_vehicles[i];
    string slot = doubleToStringX(m_slot_x[v],1) + "," + doubleToStringX(m_slot_y[v],1);
    string dist = m_dist.count(v) ? doubleToStringX(m_dist[v],1) : "-";
    string rel  = m_release_offset.count(v) ? (doubleToStringX(m_release_offset[v],0)+"s") : "-";
    string spd  = m_cmd_speed.count(v) ? doubleToStringX(m_cmd_speed[v],2) : "-";
    string perr, ospd;
    if(m_enable_orbit_lock && m_arrived[v]) {
      perr = m_phase_err.count(v) ? (doubleToStringX(m_phase_err[v],1)+"deg") : "-";
      ospd = m_orbit_cmd.count(v)  ? doubleToStringX(m_orbit_cmd[v],2) : "-";
    }
    else {
      string st;
      if(!m_have_nav[v])          st = "no-report";
      else if(m_staggered && (elapsed < m_release_offset[v])) st = "held";
      else                        st = "running";
      perr = st; ospd = "-";
    }
    actab << v << slot << dist << rel << spd << perr << ospd;
  }
  m_msgs << actab.getFormattedString();

  return(true);
}
