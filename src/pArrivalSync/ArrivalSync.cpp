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
#include "XYPoint.h"
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

  // Target investigation config (opt-in)
  m_enable_investigate = false;
  m_loop_radius        = 8.0;
  m_loop_margin        = 4.0;   // boats fly ~2 m wider than commanded, + buffer
  m_loop_points        = 8;
  m_investigate_speed  = 2.0;
  m_rejoin_speed       = 2.5;    // radial re-entry onto the ring (a touch above orbit)
  m_rejoin_capture     = 6.0;    // > the boat's waypoint capture_radius (4) so the
                                 // shoreside ends the detour before the boat captures
  m_target_var         = "TARGET_DETECT";
  m_inv_flag_var       = "INVESTIGATE";
  m_inv_update_var     = "INVESTIGATE_UPDATE";
  m_inv_done_var       = "INVESTIGATE_DONE";

  // DISPERSE/ASSEMBLE square formation (opt-in)
  m_enable_disperse    = false;
  m_square_radius      = 8.0;
  m_disperse_speed     = 2.0;
  m_disperse_cmd_var   = "DISPERSE_CMD";
  m_disp_flag_var      = "DISPERSE";
  m_disp_update_var    = "DISPERSE_UPDATE";
  m_slotted_var        = "SLOTTED";
  m_disperse_forward_bias = false;   // off => cyclicAssign is pure nearest, as before
  m_disperse_fwd_penalty  = 0.4;     // m per degree-behind; only used when bias is on

  // State
  m_target_count = 0;
  m_rejoin_px   = 1e18;   // sentinel: forces the first rejoin-point post
  m_rejoin_py   = 1e18;
  m_dispersed   = false;
  m_pending_cmd = CMD_NONE;
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
      // A fresh deploy starts a new run-in: everyone is un-arrived again, and
      // the phase offsets reset to the canonical cardinals (a prior mission's
      // rejoin may have re-anchored them).
      if(m_deployed && !was) {
        for(unsigned int i=0; i<m_vehicles.size(); i++) {
          string v = m_vehicles[i];
          m_arrived[v] = false;
          // A previous ASSEMBLE may have handed this boat a different cardinal;
          // a fresh deploy puts everyone back on their canonical slot.
          m_slot_x[v] = m_slot0_x[i];
          m_slot_y[v] = m_slot0_y[i];
          Notify(m_update_var + "_" + toupper(v),
                 "point=" + doubleToStringX(m_slot_x[v],2) + "," +
                            doubleToStringX(m_slot_y[v],2));
          m_phase_off[v] = slotAngleDeg(v);
        }
      }
    }
    else if(key == m_return_var)
      m_returning = (tolower(msg.GetString()) == "true");

    else if(key == m_target_var)
      handleTargetDetect(msg.GetString());

    else if(key == m_disperse_cmd_var)
      handleDisperseCmd(tolower(msg.GetString()) == "true");

    else if(key == m_inv_done_var) {
      // The investigator finished circling the target. Begin the rejoin
      // leg: the shoreside now steers it back into its (empty, moving)
      // slot on the ring so it re-enters with a near-zero phase error.
      string who = tolower(stripBlankEnds(msg.GetString()));
      if(who == m_investigator) {
        if(m_dispersed) {
          // Out on the square: there is no ring to rejoin. Just release the
          // boat -- its own disperse_loiter re-acquires its corner.
          Notify(m_inv_flag_var + "_" + toupper(who), "false");
          m_investigating[who] = false;
          eraseTarget(m_cur_label);
          m_investigator = "";
          m_cur_label = "";
        }
        else if(m_enable_orbit_lock && (m_circle_rad > 0)) {
          m_rejoining[who] = true;         // handleRejoin() takes it from here
          m_rejoin_px = 1e18;              // force a fresh entry-point post
          m_rejoin_py = 1e18;
        }
        else {
          // No ring geometry to rejoin: finish the old way (clear now).
          Notify(m_inv_flag_var + "_" + toupper(who), "false");
          m_investigating[who] = false;
          eraseTarget(m_cur_label);
          m_investigator = "";
          m_cur_label = "";
        }
      }
    }

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

  // Keep every boat's distance-to-slot live, for the report. (The run-in pass
  // below stops looking at a boat once it has arrived, so this can't live
  // there or the figure would freeze at its arrival value.)
  for(unsigned int i=0; i<m_vehicles.size(); i++) {
    string v = m_vehicles[i];
    if(m_have_nav[v])
      m_dist[v] = hypot(m_slot_x[v]-m_nav_x[v], m_slot_y[v]-m_nav_y[v]);
  }

  // The ring passes (run-in + orbit phase-lock) only make sense when the
  // boats belong to the ring. Out on the DISPERSE square each boat is held
  // by its own loiter behaviour, so they are skipped entirely.
  if(active && (m_max_speed > 0) && !m_dispersed) {

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

      double d = m_dist[v];       // kept live at the top of Iterate()

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
    m_curr_T = 0;                 // 0 = nobody running in (report shows "-")
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
        if(!m_have_nav[v] || !m_arrived[v] || m_investigating[v])
          continue;   // skip a boat that is off investigating a target

        // Start the shared clock at the first arrival.
        if(!m_orbit_active) {
          m_orbit_active = true;
          m_orbit_t0 = m_curr_time;
        }

        double base   = m_phase_off.count(v) ? m_phase_off[v] : slotAngleDeg(v);
        double target = base + omega_deg * (m_curr_time - m_orbit_t0);
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

  // The detour subsystem runs in BOTH formations: on the ring (rejoin via
  // the moving slot) and out on the square (the boat simply returns to its
  // corner). handleRejoin() no-ops unless a ring rejoin is actually pending.
  if(active && m_enable_investigate) {
    handleRejoin();
    assignInvestigation();
  }

  // A DISPERSE/ASSEMBLE pressed mid-detour fires once nobody is out.
  if(active && m_enable_disperse && (m_investigator == "") && (m_pending_cmd != CMD_NONE))
    runPendingCmd();

  if(!active) {
    // Not active (idle or returning): cancel any in-progress investigation
    // and drop the square -- both clear the boats' flags so a later deploy
    // starts clean -- then re-arm.
    if(m_enable_investigate)
      cancelInvestigation();
    if(m_dispersed)
      cancelDisperse();
    m_pending_cmd  = CMD_NONE;
    m_staggered    = false;
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
// Procedure: handleTargetDetect()
//   A left-click on the viewer arrives as "x=..,y=..". If it's inside
//   the op-region, queue it and drop a pending marker.

void ArrivalSync::handleTargetDetect(const string& sval)
{
  double x = 0, y = 0;
  if(!tokParse(sval, "x", ',', '=', x) || !tokParse(sval, "y", ',', '=', y)) {
    reportRunWarning("Bad TARGET_DETECT: " + sval);
    return;
  }
  if(!pointInRegion(x, y)) {
    reportRunWarning("Target outside op-region, ignored: " + sval);
    return;
  }

  // The click is inside, but the LOOP around it also has to fit: a target
  // near the edge would otherwise walk the boat across the boundary. Pull
  // it inward just far enough.
  double ox = x, oy = y;
  if(!nudgeIntoRegion(x, y))
    reportRunWarning("Target too near the op-region edge for a full loop; "
                     "using the best fit: " + sval);
  if((fabs(x-ox) > 0.1) || (fabs(y-oy) > 0.1))
    reportEvent("Target nudged inward to fit its loop: " +
                doubleToStringX(ox,1) + "," + doubleToStringX(oy,1) + " -> " +
                doubleToStringX(x,1)  + "," + doubleToStringX(y,1));

  string label = "target_" + uintToString(m_target_count++);
  m_queue_x.push_back(x);
  m_queue_y.push_back(y);
  m_queue_label.push_back(label);
  drawTarget(label, x, y, "orange");   // pending (queued) color
}

//---------------------------------------------------------
// Procedure: pointInRegion()
//   Ray-cast point-in-polygon. If no region is configured, accept all.

bool ArrivalSync::pointInRegion(double x, double y) const
{
  unsigned int n = m_region_x.size();
  if(n < 3)
    return(true);
  bool inside = false;
  for(unsigned int i=0, j=n-1; i<n; j=i++) {
    double xi = m_region_x[i], yi = m_region_y[i];
    double xj = m_region_x[j], yj = m_region_y[j];
    if(((yi > y) != (yj > y)) && (x < (xj-xi)*(y-yi)/(yj-yi) + xi))
      inside = !inside;
  }
  return(inside);
}

//---------------------------------------------------------
// Procedure: regionClearance()
//   Distance from (x,y) to the nearest point on the op-region boundary,
//   and that boundary point in (nx,ny). Assumes the point is inside.

double ArrivalSync::regionClearance(double x, double y, double& nx, double& ny) const
{
  unsigned int n = m_region_x.size();
  nx = x; ny = y;
  if(n < 3)
    return(1e9);

  double best = 1e18;
  for(unsigned int i=0, j=n-1; i<n; j=i++) {
    double ax = m_region_x[j], ay = m_region_y[j];
    double bx = m_region_x[i], by = m_region_y[i];
    double dx = bx - ax, dy = by - ay;
    double L2 = (dx*dx) + (dy*dy);
    double t  = (L2 <= 0) ? 0.0 : (((x-ax)*dx) + ((y-ay)*dy)) / L2;
    if(t < 0) t = 0;
    if(t > 1) t = 1;
    double px = ax + t*dx, py = ay + t*dy;
    double d  = hypot(x-px, y-py);
    if(d < best) {
      best = d;
      nx = px; ny = py;
    }
  }
  return(best);
}

//---------------------------------------------------------
// Procedure: nudgeIntoRegion()
//   A target clicked near the op-region edge would send the boat round a
//   loop that crosses the boundary (the click itself being inside is not
//   enough -- the whole circle has to fit). Pull the point straight in,
//   away from the nearest edge, until loop_radius+loop_margin of water
//   fits around it. Returns false if even that can't be achieved.

bool ArrivalSync::nudgeIntoRegion(double& x, double& y) const
{
  if(m_region_x.size() < 3)
    return(true);
  double need = m_loop_radius + m_loop_margin;

  for(int iter=0; iter<12; iter++) {
    double nx = 0, ny = 0;
    double d = regionClearance(x, y, nx, ny);
    if(d >= need)
      return(true);

    // Push away from the nearest edge point (that direction is inward,
    // since the caller has already checked the point is inside).
    double dx = x - nx, dy = y - ny;
    double L  = hypot(dx, dy);
    if(L < 1e-6) {
      // Sitting exactly on the edge: aim at the region's centroid instead.
      double cx = 0, cy = 0;
      for(unsigned int k=0; k<m_region_x.size(); k++) {
        cx += m_region_x[k];
        cy += m_region_y[k];
      }
      cx /= (double)m_region_x.size();
      cy /= (double)m_region_y.size();
      dx = cx - x; dy = cy - y;
      L  = hypot(dx, dy);
      if(L < 1e-6)
        return(false);
    }
    x += (dx/L) * (need - d + 0.5);
    y += (dy/L) * (need - d + 0.5);
  }

  double nx = 0, ny = 0;
  return(regionClearance(x, y, nx, ny) >= need);
}

//---------------------------------------------------------
// Procedure: closestFreeBoat()
//   Nearest boat to (tx,ty) that is on the ring and not investigating.

string ArrivalSync::closestFreeBoat(double tx, double ty) const
{
  string best = "";
  double bestd = 1e18;
  for(unsigned int i=0; i<m_vehicles.size(); i++) {
    string v = m_vehicles[i];
    map<string,bool>::const_iterator hn = m_have_nav.find(v);
    map<string,bool>::const_iterator ar = m_arrived.find(v);
    map<string,bool>::const_iterator in = m_investigating.find(v);
    if(hn == m_have_nav.end() || !hn->second) continue;
    // On the ring, only boats that actually reached their slot may be sent.
    // Out on the square that test doesn't apply -- any boat holding a corner
    // is fair game.
    if(!m_dispersed && (ar == m_arrived.end() || !ar->second)) continue;
    if(in != m_investigating.end() && in->second) continue;
    double nx = m_nav_x.find(v)->second;
    double ny = m_nav_y.find(v)->second;
    double d  = hypot(tx-nx, ty-ny);
    if(d < bestd) { bestd = d; best = v; }
  }
  return(best);
}

//---------------------------------------------------------
// Procedure: loopSpec()
//   A BHV_Waypoint "points=..." spec: m_loop_points around the target
//   at m_loop_radius -> the boat circles the target once.

string ArrivalSync::loopSpec(double tx, double ty) const
{
  int n = (m_loop_points < 3) ? 3 : m_loop_points;
  string pts = "points=";
  for(int k=0; k<n; k++) {
    double a  = 2.0 * M_PI * k / n;
    double px = tx + m_loop_radius * cos(a);
    double py = ty + m_loop_radius * sin(a);
    if(k > 0) pts += ":";
    pts += doubleToStringX(px,2) + "," + doubleToStringX(py,2);
  }
  return(pts);
}

//---------------------------------------------------------
// Procedure: assignInvestigation()
//   One at a time: if nobody is out and the queue has a target with a
//   free boat available, dispatch it.

void ArrivalSync::assignInvestigation()
{
  if(m_investigator != "")   // someone is already out
    return;
  if(m_queue_x.empty())
    return;

  double tx = m_queue_x.front();
  double ty = m_queue_y.front();
  string label = m_queue_label.front();

  string boat = closestFreeBoat(tx, ty);
  if(boat == "")             // no free boat yet; keep the target queued
    return;

  m_queue_x.erase(m_queue_x.begin());
  m_queue_y.erase(m_queue_y.begin());
  m_queue_label.erase(m_queue_label.begin());

  Notify(m_inv_update_var + "_" + toupper(boat),
         loopSpec(tx, ty) + " # speed=" + doubleToStringX(m_investigate_speed,2));
  Notify(m_inv_flag_var   + "_" + toupper(boat), "true");

  m_investigating[boat] = true;
  m_investigator = boat;
  m_cur_label = label;
  drawTarget(label, tx, ty, "red");   // active (being investigated)
}

//---------------------------------------------------------
// Procedure: handleRejoin()
//   Bring the investigator back onto the ring at the NEAREST ring point
//   (the ring point at its current angle) -- a purely radial move, so it
//   re-enters from wherever it is without ever chording across the
//   interior. On arrival we re-anchor the whole formation (respaceFormation)
//   so the returning boat keeps the spot it just re-entered and the other
//   three re-even around it, then hand it back to ENCIRCLING.

void ArrivalSync::handleRejoin()
{
  string v = m_investigator;
  if(v == "" || !m_rejoining[v])   // nobody out, or still circling the target
    return;
  if(!m_have_nav[v] || (m_circle_rad <= 0))
    return;

  // Nearest point on the ring = the ring point at the boat's current angle.
  double cur = actualAngleDeg(v);
  double rad = cur * (M_PI / 180.0);
  double ex  = m_circle_x + m_circle_rad * cos(rad);
  double ey  = m_circle_y + m_circle_rad * sin(rad);

  // Steer straight there (radial). Re-post only when the point has moved
  // enough (it barely moves, since the boat's angle holds on a radial run).
  if(hypot(ex - m_rejoin_px, ey - m_rejoin_py) > 1.0) {
    Notify(m_inv_update_var + "_" + toupper(v),
           "points=" + doubleToStringX(ex,2) + "," + doubleToStringX(ey,2) +
           " # speed=" + doubleToStringX(m_rejoin_speed,2));
    m_rejoin_px = ex;
    m_rejoin_py = ey;
  }

  // Reached the ring? Re-even the formation around this boat, then hand it
  // back to ENCIRCLING. m_rejoin_capture is deliberately larger than the
  // boat's own waypoint capture_radius so we end the detour before the boat
  // captures and re-fires INVESTIGATE_DONE.
  double d = hypot(ex - m_nav_x[v], ey - m_nav_y[v]);
  if(d <= m_rejoin_capture) {
    respaceFormation(v);
    Notify(m_inv_flag_var + "_" + toupper(v), "false");   // -> ENCIRCLING
    m_investigating[v] = false;
    m_rejoining[v]     = false;
    eraseTarget(m_cur_label);
    m_investigator = "";
    m_cur_label    = "";
    m_rejoin_px = 1e18;
    m_rejoin_py = 1e18;
  }
}

//---------------------------------------------------------
// Procedure: respaceFormation()
//   Re-anchor the four phase offsets so the just-returned boat 'inv' keeps
//   the ring angle it re-entered at (no chasing a far-away slot), and the
//   other three take the remaining even slots (inv+90/180/270). The others
//   are assigned to those slots IN CYCLIC ORDER (sorted by current angle,
//   CCW from inv) so the ring re-evens without any boat crossing another --
//   each just speeds up or eases off under the phase-lock.

void ArrivalSync::respaceFormation(const string& inv)
{
  if(!m_orbit_active || (m_circle_rad <= 0))
    return;
  double omega_deg = (m_orbit_speed / m_circle_rad) * (180.0 / M_PI);
  double tt        = m_curr_time - m_orbit_t0;

  // The returning boat's slot = its current (re-entry) angle: pick the phase
  // offset whose target equals that angle right now.
  double inv_ang = actualAngleDeg(inv);
  m_phase_off[inv] = inv_ang - omega_deg * tt;

  // Gather the other boats by their CCW angle measured from inv, then hand
  // them the +90 / +180 / +270 slots in that order.
  vector<pair<double,string> > others;
  for(unsigned int i=0; i<m_vehicles.size(); i++) {
    string p = m_vehicles[i];
    if(p == inv)
      continue;
    if(!m_have_nav[p] || !m_arrived[p])
      continue;
    double a = actualAngleDeg(p) - inv_ang;
    while(a < 0)    a += 360;
    while(a >= 360) a -= 360;
    others.push_back(make_pair(a, p));
  }
  sort(others.begin(), others.end(),
       [](const pair<double,string>& a, const pair<double,string>& b){
         return a.first < b.first; });

  for(unsigned int k=0; k<others.size(); k++) {
    double slot_ang = inv_ang + 90.0 * (double)(k + 1);   // +90, +180, +270
    m_phase_off[others[k].second] = slot_ang - omega_deg * tt;
  }
}

//---------------------------------------------------------
// Procedure: cancelInvestigation()
//   Abort any in-progress investigation (called when the field returns or
//   goes idle). Clears the boat's INVESTIGATE flag so it doesn't redeploy
//   stuck in INVESTIGATING, and tidies the marker + state.

void ArrivalSync::cancelInvestigation()
{
  if(m_investigator != "") {
    Notify(m_inv_flag_var + "_" + toupper(m_investigator), "false");
    m_investigating[m_investigator] = false;
    m_rejoining[m_investigator]     = false;
    eraseTarget(m_cur_label);
    m_investigator = "";
    m_cur_label    = "";
  }
  m_rejoin_px = 1e18;
  m_rejoin_py = 1e18;
}

//---------------------------------------------------------
// Procedure: squareSpec()
//   A BHV_Loiter update: the small circle this boat holds at its assigned
//   square corner, plus the transit-out / loiter speed.

string ArrivalSync::squareSpec(double cx, double cy, const string& v) const
{
  string spec = "polygon=radial:: x=" + doubleToStringX(cx,2) +
                ", y=" + doubleToStringX(cy,2) +
                ", radius=" + doubleToStringX(m_square_radius,1) +
                ", pts=8, snap=1, label=" + v + "_sq";
  spec += " # speed=" + doubleToStringX(m_disperse_speed,2);
  return(spec);
}

//---------------------------------------------------------
// Procedure: handleDisperseCmd()
//   A DISPERSE (true) / ASSEMBLE (false) button press. If a target detour
//   is in flight the command is QUEUED: changing formation mid-detour would
//   strand the investigator between the ring and the square.

void ArrivalSync::handleDisperseCmd(bool on)
{
  if(!m_enable_disperse)
    return;
  if(m_investigator != "") {
    m_pending_cmd = (on ? CMD_DISPERSE : CMD_ASSEMBLE);
    return;
  }
  if(on && !m_dispersed)
    doDisperse();
  else if(!on && m_dispersed)
    doAssemble();
}

//---------------------------------------------------------
// Procedure: runPendingCmd()
//   Fire a button press that had been queued behind a target detour.

void ArrivalSync::runPendingCmd()
{
  int cmd = m_pending_cmd;
  m_pending_cmd = CMD_NONE;
  if((cmd == CMD_DISPERSE) && !m_dispersed)
    doDisperse();
  else if((cmd == CMD_ASSEMBLE) && m_dispersed)
    doAssemble();
}

//---------------------------------------------------------
// Procedure: cyclicAssign()
//   Hand out a set of target points so each boat takes the nearest one AND
//   no two boats cross paths. Both sets are sorted by angle round the ring
//   centre, then every cyclic rotation is tried and the one with the least
//   total travel wins. Sorting by angle is what rules out crossings: the
//   boats keep their rotational order. Fills 'boats' (names) and 'tidx'
//   (index into tx/ty) in parallel.

void ArrivalSync::cyclicAssign(const vector<double>& tx, const vector<double>& ty,
                               vector<string>& boats, vector<unsigned int>& tidx,
                               double turn_penalty)
{
  boats.clear();
  tidx.clear();
  unsigned int n = tx.size();
  if((n == 0) || (ty.size() != n))
    return;

  // Boats with a known position, by angle round the ring centre.
  vector<pair<double,string> > bs;
  for(unsigned int i=0; i<m_vehicles.size(); i++) {
    string v = m_vehicles[i];
    if(!m_have_nav[v])
      continue;
    bs.push_back(make_pair(actualAngleDeg(v), v));
  }
  if(bs.empty())
    return;
  sort(bs.begin(), bs.end(),
       [](const pair<double,string>& a, const pair<double,string>& b){
         return a.first < b.first; });

  // Targets, by angle round that same centre.
  vector<pair<double,unsigned int> > ts;
  for(unsigned int k=0; k<n; k++) {
    double a = atan2(ty[k]-m_circle_y, tx[k]-m_circle_x) * (180.0/M_PI);
    if(a < 0) a += 360;
    ts.push_back(make_pair(a, k));
  }
  sort(ts.begin(), ts.end(),
       [](const pair<double,unsigned int>& a, const pair<double,unsigned int>& b){
         return a.first < b.first; });

  // The rotation with the least total travel.
  unsigned int nb = bs.size();
  unsigned int best_rot = 0;
  double best_total = -1;
  for(unsigned int rot=0; rot<n; rot++) {
    double total = 0;
    for(unsigned int i=0; i<nb; i++) {
      unsigned int j  = (i+rot) % n;
      unsigned int ti = ts[j].second;
      string v = bs[i].second;
      total += hypot(tx[ti]-m_nav_x[v], ty[ti]-m_nav_y[v]);
      // Bias toward corners ahead in the orbit (CCW) direction: a corner
      // behind the boat's current ring angle would make it turn back, so
      // charge for it. Every rotation here is still order-preserving (no
      // crossing); this only decides WHICH non-crossing rotation wins.
      if(turn_penalty > 0) {
        double disp = ts[j].first - bs[i].first;   // +ve => corner is CCW-ahead
        while(disp > 180)  disp -= 360;
        while(disp < -180) disp += 360;
        if(disp < 0)
          total += turn_penalty * (-disp);
      }
    }
    if((best_total < 0) || (total < best_total)) {
      best_total = total;
      best_rot   = rot;
    }
  }

  for(unsigned int i=0; i<nb; i++) {
    boats.push_back(bs[i].second);
    tidx.push_back(ts[(i+best_rot) % n].second);
  }
}

//---------------------------------------------------------
// Procedure: doDisperse()
//   Send every boat outward to its own corner of the fixed square, each
//   taking the nearest corner without crossing anyone (see cyclicAssign).

void ArrivalSync::doDisperse()
{
  if(m_square_x.empty()) {
    reportRunWarning("DISPERSE ignored: no square configured");
    return;
  }

  vector<string> boats;
  vector<unsigned int> idx;
  double tp = m_disperse_forward_bias ? m_disperse_fwd_penalty : 0.0;
  cyclicAssign(m_square_x, m_square_y, boats, idx, tp);
  if(boats.empty()) {
    reportRunWarning("DISPERSE ignored: no boat positions yet");
    return;
  }

  for(unsigned int i=0; i<boats.size(); i++) {
    string v = boats[i];
    unsigned int ci = idx[i];
    Notify(m_disp_update_var + "_" + toupper(v),
           squareSpec(m_square_x[ci], m_square_y[ci], v));
    Notify(m_disp_flag_var + "_" + toupper(v), "true");
    m_corner_of[v] = (int)ci;
  }

  m_dispersed    = true;
  m_orbit_active = false;   // the ring clock stops; it restarts on ASSEMBLE
}

//---------------------------------------------------------
// Procedure: doAssemble()
//   Drop the square and re-form the ring by re-running the proven run-in.
//   Each boat is given the ring slot NEAREST its corner (cyclic, so nobody
//   crosses) rather than its canonical cardinal -- that slot can be
//   diagonally opposite, which used to send all four straight through the
//   middle of the formation. The slot point is pushed to goto_slot via
//   SLOT_UPDATE, and the phase offsets are re-anchored to match.

void ArrivalSync::doAssemble()
{
  vector<string> boats;
  vector<unsigned int> idx;
  // Same forward-bias as DISPERSE: hand each boat the ring slot it can ENTER
  // moving forward (CCW), so it slides onto the orbit tangentially instead of
  // reaching the ring and U-turning inward toward the centre. On the diagonal-
  // corner geometry the two nearest slots tie, and the "behind" one forces the
  // U-turn; the bias breaks the tie toward the slot ahead. Off => plain nearest.
  double tp = m_disperse_forward_bias ? m_disperse_fwd_penalty : 0.0;
  cyclicAssign(m_slot0_x, m_slot0_y, boats, idx, tp);

  for(unsigned int i=0; i<boats.size(); i++) {
    string v = boats[i];
    unsigned int si = idx[i];
    m_slot_x[v] = m_slot0_x[si];
    m_slot_y[v] = m_slot0_y[si];
    Notify(m_update_var + "_" + toupper(v),
           "point=" + doubleToStringX(m_slot_x[v],2) + "," +
                      doubleToStringX(m_slot_y[v],2));
  }

  for(unsigned int i=0; i<m_vehicles.size(); i++) {
    string v = m_vehicles[i];
    Notify(m_disp_flag_var + "_" + toupper(v), "false");   // -> ENCIRCLING
    Notify(m_slotted_var   + "_" + toupper(v), "false");   // -> re-run goto_slot
    m_arrived[v]        = false;
    m_phase_off[v]      = slotAngleDeg(v);  // uses the slot just assigned
    m_release_offset[v] = 0;                // everyone released at once
    m_cmd_speed.erase(v);                   // clear throttles so fresh speeds post
    m_orbit_cmd.erase(v);
  }
  m_corner_of.clear();
  m_dispersed    = false;
  m_orbit_active = false;   // the clock restarts at the first arrival
  m_staggered    = true;    // offsets already set (all zero): don't recompute
  m_deploy_time  = m_curr_time;
}

//---------------------------------------------------------
// Procedure: cancelDisperse()
//   Drop the square without re-forming (used when the field returns or goes
//   idle), so no boat redeploys stuck in DISPERSED.

void ArrivalSync::cancelDisperse()
{
  for(unsigned int i=0; i<m_vehicles.size(); i++)
    Notify(m_disp_flag_var + "_" + toupper(m_vehicles[i]), "false");
  m_corner_of.clear();
  m_dispersed = false;
}

//---------------------------------------------------------
// Procedure: drawTarget() / eraseTarget()

void ArrivalSync::drawTarget(const string& label, double x, double y,
                             const string& color)
{
  XYPoint pt(x, y);
  pt.set_label(label);
  pt.set_color("vertex", color);
  pt.set_param("vertex_size", "9");
  Notify("VIEW_POINT", pt.get_spec());
}

void ArrivalSync::eraseTarget(const string& label)
{
  if(label == "")
    return;
  XYPoint pt(0, 0);
  pt.set_label(label);
  pt.set_active(false);
  Notify("VIEW_POINT", pt.get_spec());
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
  m_slot0_x.push_back(atof(sx.c_str()));   // canonical: restored on each deploy
  m_slot0_y.push_back(atof(sy.c_str()));
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
    else if(param == "enable_investigate")
      handled = setBooleanOnString(m_enable_investigate, value);
    else if(param == "loop_radius")
      handled = setDoubleOnString(m_loop_radius, value);
    else if(param == "loop_margin")
      handled = setDoubleOnString(m_loop_margin, value);
    else if(param == "loop_points")
      handled = setIntOnString(m_loop_points, value);
    else if(param == "investigate_speed")
      handled = setDoubleOnString(m_investigate_speed, value);
    else if(param == "rejoin_speed")
      handled = setDoubleOnString(m_rejoin_speed, value);
    else if(param == "rejoin_capture")
      handled = setDoubleOnString(m_rejoin_capture, value);
    else if(param == "target_var") {
      m_target_var = value; handled = true;
    }
    else if(param == "region") {
      // "x1,y1:x2,y2:..." -> op-region polygon for the in-area check.
      m_region_x.clear();
      m_region_y.clear();
      vector<string> verts = parseString(value, ':');
      for(unsigned int k=0; k<verts.size(); k++) {
        string vx = biteStringX(verts[k], ',');
        string vy = verts[k];
        if(isNumber(vx) && isNumber(vy)) {
          m_region_x.push_back(atof(vx.c_str()));
          m_region_y.push_back(atof(vy.c_str()));
        }
      }
      handled = (m_region_x.size() >= 3);
      if(!handled)
        reportConfigWarning("Bad region (need >=3 x,y vertices): " + orig);
    }
    else if(param == "enable_disperse")
      handled = setBooleanOnString(m_enable_disperse, value);
    else if(param == "square_radius")
      handled = setDoubleOnString(m_square_radius, value);
    else if(param == "disperse_speed")
      handled = setDoubleOnString(m_disperse_speed, value);
    else if(param == "disperse_forward_bias")
      handled = setBooleanOnString(m_disperse_forward_bias, value);
    else if(param == "disperse_fwd_penalty")
      handled = setDoubleOnString(m_disperse_fwd_penalty, value);
    else if(param == "disperse_cmd_var") {
      m_disperse_cmd_var = value; handled = true;
    }
    else if(param == "square") {
      // "x1,y1:x2,y2:..." -> the fixed corners of the DISPERSE formation.
      m_square_x.clear();
      m_square_y.clear();
      vector<string> verts = parseString(value, ':');
      for(unsigned int k=0; k<verts.size(); k++) {
        string vx = biteStringX(verts[k], ',');
        string vy = verts[k];
        if(isNumber(vx) && isNumber(vy)) {
          m_square_x.push_back(atof(vx.c_str()));
          m_square_y.push_back(atof(vy.c_str()));
        }
      }
      handled = (m_square_x.size() >= 3);
      if(!handled)
        reportConfigWarning("Bad square (need >=3 x,y corners): " + orig);
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
  if(m_enable_investigate) {
    Register(m_target_var, 0);
    Register(m_inv_done_var, 0);
  }
  if(m_enable_disperse)
    Register(m_disperse_cmd_var, 0);
}

//------------------------------------------------------------
// Procedure: buildReport()

bool ArrivalSync::buildReport()
{
  string phase = "IDLE";
  if(m_deployed && m_returning) phase = "RETURNING (paused)";
  else if(m_deployed && m_dispersed) phase = "DISPERSED (square)";
  else if(m_deployed)           phase = "RUN-IN (active)";

  double elapsed = (m_staggered ? (m_curr_time - m_deploy_time) : 0);

  m_msgs << "Phase:          " << phase << endl;
  m_msgs << "max_speed:      " << doubleToStringX(m_max_speed,2) << " m/s" << endl;
  m_msgs << "stagger_time:   " << doubleToStringX(m_stagger_time,1) << " s (farthest released first)" << endl;
  m_msgs << "elapsed:        " << doubleToStringX(elapsed,1) << " s since deploy" << endl;
  m_msgs << "common ETA (T): "
         << ((m_curr_T > 0) ? (doubleToStringX(m_curr_T,1) + " s")
                            : string("-  (nobody running in)")) << endl;
  if(m_enable_orbit_lock) {
    m_msgs << "orbit-lock:     ON  (center " << doubleToStringX(m_circle_x,0) << ","
           << doubleToStringX(m_circle_y,0) << " r" << doubleToStringX(m_circle_rad,1)
           << ", " << doubleToStringX(m_orbit_speed,1) << " m/s, gain "
           << doubleToStringX(m_orbit_gain,3) << ") clock "
           << (m_orbit_active ? "running" : "idle") << endl;
  }
  if(m_enable_investigate) {
    string inv = "(none)";
    if(m_investigator != "") {
      bool rj = (m_rejoining.count(m_investigator) && m_rejoining[m_investigator]);
      inv = m_investigator + (rj ? " [rejoining]" : " [looping]");
    }
    m_msgs << "investigate:    ON  queue=" << m_queue_x.size()
           << "  investigator=" << inv
           << "  loop r" << doubleToStringX(m_loop_radius,1)
           << " x" << m_loop_points
           << "  rejoin_spd=" << doubleToStringX(m_rejoin_speed,1) << endl;
  }
  if(m_enable_disperse) {
    string pend = "(none)";
    if(m_pending_cmd == CMD_DISPERSE) pend = "DISPERSE queued (waiting on detour)";
    if(m_pending_cmd == CMD_ASSEMBLE) pend = "ASSEMBLE queued (waiting on detour)";
    m_msgs << "disperse:       " << (m_dispersed ? "SQUARE" : "ring")
           << "  corners=" << m_square_x.size()
           << "  loiter r" << doubleToStringX(m_square_radius,1)
           << "  fwd-bias=" << (m_disperse_forward_bias ? "on" : "off")
           << "  pending=" << pend << endl;
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
    // Run-in speed only means something while the boat is actually running in.
    string spd  = "-";
    if(!m_arrived[v] && !m_dispersed && m_cmd_speed.count(v))
      spd = doubleToStringX(m_cmd_speed[v],2);
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
