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
#include "NodeRecord.h"
#include "NodeRecordUtils.h"
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
  m_turn_rate = 20;        // deg/s: real-boat turn rate; <= 0 disables heading penalty

  // Coverage-reduction tuning (starting value; 0 disables).
  m_cover_range = 6;       // m: a visit point covers swimmers within this

  // Boundary safety (starting values; 0 disables).
  m_boundary_margin = 4;   // m: base inset (small -> hug swimmers on safe passes)
  m_overshoot_max   = 10;  // m: extra turn-overshoot inset (reduced: was 18 -> hug edge swimmers; OpRegion is the hard backstop)
  m_region_set = false;

  m_buoy_ignore_radius = 6; // m: ignore swimmers within this of a buoy CENTER (4m octagon + 2m)

  m_use_two_opt = true;    // uncross the tour after ordering
  m_use_or_opt  = true;    // relocate short runs after uncrossing

  // Opponent-aware contest tuning (starting values; tune vs 2 boats).
  m_lose_margin     = 8;   // s: skip a swimmer the opponent beats us to by >this
  m_contest_window  = 10;  // s: within this ETA margin -> contested
  m_contest_boost   = 0.6; // score multiplier for a contested swimmer
  m_contact_max_age = 12;  // s: ignore opponent contacts older than this
  m_replan_interval = 5;   // s: replan at least this often when opponent known

  m_last_replan_utc = 0;
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
    else if(key == "NODE_REPORT")
      handled = handleMailNodeReport(sval);
    else if(key == "NODE_REPORT_LOCAL") {
      // Our own report: learn our team color to tell teammates from foes.
      string c = colorOfReport(sval);
      if(c != "")
        m_my_color = c;
    }
    else if(key == "RESCUE_REGION")
      handled = handleMailRescueRegion(sval);
    else if(key == "VIEW_POLYGON")
      handled = handleMailViewPolygon(sval);

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

  // Periodic replan / re-sweep: while any swimmer remains un-rescued,
  // regenerate the tour every m_replan_interval seconds. Capture is a
  // coin toss, so a single pass can miss a swimmer; because missed
  // swimmers stay in m_swimmers, each replan re-tours them and the boat
  // comes back to try again. Also keeps target/contest decisions fresh
  // as ownship and any opponent move.
  if(!m_plan_pending && (m_swimmers.size() > 0)) {
    if((MOOSTime() - m_last_replan_utc) >= m_replan_interval)
      m_plan_pending = true;
  }

  // Replan when a new swimmer has arrived (or one was removed), once we
  // actually have a NAV fix. If there are no swimmers left to plan for
  // we still clear the flag so it can't stick "true" after the last
  // rescue.
  if(m_plan_pending && m_nav_x_set && m_nav_y_set) {
    if(m_swimmers.size() > 0) {
      postShortestPath();
      m_last_replan_utc = MOOSTime();
    }
    m_plan_pending = false;
  }

  // Keep the boat working while any RESCUABLE swimmer remains. The survey
  // behavior fires its endflag (RETURN=true) the instant it has VISITED
  // every waypoint, but a single pass is a coin toss -- visited swimmers
  // can still be un-rescued. That would latch the boat into RETURNING and
  // send it home with known swimmers still out there. Forcing RETURN=false
  // re-activates the perpetual survey so the boat re-tours the misses.
  // Swimmers on/by a buoy are ignored (swimmerNearBuoy) so they DON'T count
  // -- otherwise an un-rescuable on-buoy swimmer would trap the boat out
  // forever. When the last rescuable swimmer is saved the count hits 0, we
  // stop overriding, and the boat returns home normally (as in Lab 9).
  unsigned int active = 0;
  map<string, XYPoint>::iterator sit;
  for(sit=m_swimmers.begin(); sit!=m_swimmers.end(); sit++) {
    if(!swimmerNearBuoy(sit->second.x(), sit->second.y()))
      active++;
  }
  if(active > 0)
    Notify("RETURN", "false");

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
  Register("NODE_REPORT", 0);
  Register("NODE_REPORT_LOCAL", 0);
  Register("RESCUE_REGION", 0);
  Register("VIEW_POLYGON", 0);
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
// Procedure: handleMailNodeReport()
//   Ingest another vehicle's NODE_REPORT. Ownship is skipped; every
//   other craft is tracked as a possible opponent for contest scoring.

bool GenRescue::handleMailNodeReport(string str)
{
  NodeRecord record = string2NodeRecord(str);
  string name = record.getName();

  // Skip our own report (we already track ownship via NAV_*).
  if((name == "") || (name == m_host_community))
    return(true);

  // Skip TEAMMATES (same team color, e.g. our own scout). The contest
  // logic concedes swimmers a faster *opponent* will reach first -- but a
  // teammate scout never rescues, so conceding to it would just throw
  // rescues away. Only different-team craft are real opponents. (Until we
  // learn our own color, treat everyone as a potential opponent.)
  if((m_my_color != "") && (colorOfReport(str) == m_my_color))
    return(true);

  Contact c;
  c.x       = record.getX();
  c.y       = record.getY();
  c.heading = record.getHeading();
  c.speed   = record.getSpeed();
  c.utc     = MOOSTime();   // local receipt time, robust to clock skew
  m_contacts[name] = c;

  return(true);
}

//---------------------------------------------------------
// Procedure: colorOfReport()
//   Pull the COLOR field (lowercased) out of a NODE_REPORT spec. We parse
//   the raw spec because string2NodeRecord does not populate color.
//   Returns "" if no color field is present.

string GenRescue::colorOfReport(string str)
{
  vector<string> svector = parseString(str, ',');
  for(unsigned int i=0; i<svector.size(); i++) {
    string param = tolower(biteStringX(svector[i], '='));
    if(param == "color")
      return(tolower(svector[i]));
  }
  return("");
}

//---------------------------------------------------------
// Procedure: handleMailRescueRegion()
//   Ingest the rescue-region boundary so we can keep waypoints inside it.

bool GenRescue::handleMailRescueRegion(string str)
{
  XYPolygon poly = string2Poly(str);
  if(poly.size() < 3)
    return(false);

  m_region     = poly;
  m_region_set = true;
  return(true);
}

//---------------------------------------------------------
// Procedure: insetIntoRegion()
//   Return (x,y) moved to sit at least m_boundary_margin inside the
//   region. A no-op if the region is unknown, the margin is off, or the
//   point is already safely interior.

XYPoint GenRescue::insetIntoRegion(double x, double y, double margin)
{
  if(!m_region_set || (margin <= 0))
    return(XYPoint(x, y));

  bool   inside = m_region.contains(x, y);
  double d      = m_region.dist_to_poly(x, y);   // distance to boundary
  if(inside && (d >= margin))
    return(XYPoint(x, y));                        // already safe

  double rx, ry;
  if(!m_region.closest_point_on_poly(x, y, rx, ry))
    return(XYPoint(x, y));

  // Inward unit direction (away from the edge for an interior point,
  // toward the edge for an exterior one).
  double ux = inside ? (x - rx) : (rx - x);
  double uy = inside ? (y - ry) : (ry - y);
  double mag = distPointToPoint(0, 0, ux, uy);
  if(mag < 1e-6)
    return(XYPoint(x, y));                        // on the boundary; leave it

  ux /= mag;
  uy /= mag;
  return(XYPoint(rx + (ux * margin), ry + (uy * margin)));
}

//---------------------------------------------------------
// Procedure: tightenForTurns()
//   Re-inset each waypoint of an ordered path. A sharp turn overshoots
//   more, so the inset grows with the turn angle at that waypoint.

XYSegList GenRescue::tightenForTurns(XYSegList path)
{
  unsigned int n = path.size();
  if((n == 0) || !m_region_set)
    return(path);

  XYSegList out;
  double px = m_nav_x;     // the boat is the "previous" point for vertex 0
  double py = m_nav_y;

  for(unsigned int i=0; i<n; i++) {
    double cx = path.get_vx(i);
    double cy = path.get_vy(i);

    // Turn angle here: 0 = straight through, 180 = full reversal.
    double turn = 0;
    if((i+1) < n) {
      double in_brg  = relAng(px, py, cx, cy);
      double out_brg = relAng(cx, cy, path.get_vx(i+1), path.get_vy(i+1));
      turn = angleDiff(in_brg, out_brg);
    }

    double margin = m_boundary_margin + (m_overshoot_max * (turn / 180.0));
    XYPoint w = insetIntoRegion(cx, cy, margin);
    out.add_vertex(w.x(), w.y());
    px = w.x();
    py = w.y();
  }
  return(out);
}

//---------------------------------------------------------
// Procedure: handleMailViewPolygon()
//   Capture buoy obstacle polygons (those whose label contains "buoy")
//   so the path planner can keep waypoints clear of them.

bool GenRescue::handleMailViewPolygon(string str)
{
  XYPolygon poly = string2Poly(str);
  if(poly.size() < 3)
    return(true);                       // not a usable polygon; ignore

  string label = poly.get_label();
  if(label.find("buoy") == string::npos)
    return(true);                       // not a buoy

  // Track ONLY the real buoy octagon (~4m). The AvoidObstacle behavior
  // also posts inflated influence rings -- buoy_N_mid_poly (~7m) and
  // buoy_N_rim_poly (~12m) -- plus active=false erase placeholders. We
  // skip those so the buoy "center" we ignore swimmers around is the real
  // one. (Centers are concentric anyway, but this keeps m_buoys clean.)
  if(label.find("rim") != string::npos)
    return(true);
  if(label.find("mid") != string::npos)
    return(true);
  if(str.find("active=false") != string::npos)
    return(true);

  m_buoys[label] = poly;                 // keyed by label -> repeats update
  return(true);
}

//---------------------------------------------------------
// Procedure: swimmerNearBuoy()
//   True if (x,y) is within m_buoy_ignore_radius of any buoy CENTER. Such
//   a swimmer sits on/right by a buoy; we drop it from the tour entirely
//   (the rescue rarely completes and the boat wastes time fighting the
//   avoidance), rather than chasing it.

bool GenRescue::swimmerNearBuoy(double x, double y)
{
  map<string, XYPolygon>::iterator it;
  for(it=m_buoys.begin(); it!=m_buoys.end(); it++) {
    XYPolygon buoy = it->second;
    if(buoy.size() < 3)
      continue;
    double bcx = 0, bcy = 0;
    for(unsigned int v=0; v<buoy.size(); v++) { bcx += buoy.get_vx(v); bcy += buoy.get_vy(v); }
    bcx /= buoy.size();
    bcy /= buoy.size();
    if(distPointToPoint(bcx, bcy, x, y) <= m_buoy_ignore_radius)
      return(true);
  }
  return(false);
}

//---------------------------------------------------------
// Procedure: etaToPoint()
//   Seconds for a craft at (px,py) heading ph at the given speed to
//   reach (tx,ty): straight-line travel time plus the time to turn
//   onto the bearing. Speed is floored so a stationary craft still
//   yields a finite estimate.

double GenRescue::etaToPoint(double px, double py, double ph,
                             double speed, double tx, double ty)
{
  if(speed < m_speed)
    speed = m_speed;

  double d           = distPointToPoint(px, py, tx, ty);
  double travel_time = d / speed;

  double turn_time = 0;
  if(m_turn_rate > 0) {
    double brg  = relAng(px, py, tx, ty);
    double aoff = angleDiff(ph, brg);
    turn_time   = aoff / m_turn_rate;
  }
  return(travel_time + turn_time);
}

//---------------------------------------------------------
// Procedure: ownshipETA()

double GenRescue::ownshipETA(double tx, double ty)
{
  return(etaToPoint(m_nav_x, m_nav_y, m_nav_heading, m_speed, tx, ty));
}

//---------------------------------------------------------
// Procedure: opponentETA()
//   Smallest ETA over all FRESH opponent contacts. have_opp is set
//   false if no contact is fresh enough to count.

double GenRescue::opponentETA(double tx, double ty, bool &have_opp)
{
  have_opp = false;
  double best = -1;
  double now  = MOOSTime();

  map<string, Contact>::iterator p;
  for(p=m_contacts.begin(); p!=m_contacts.end(); p++) {
    Contact c = p->second;
    if((now - c.utc) > m_contact_max_age)   // stale -> ignore
      continue;
    double eta = etaToPoint(c.x, c.y, c.heading, c.speed, tx, ty);
    if(!have_opp || (eta < best)) {
      best = eta;
      have_opp = true;
    }
  }
  return(best);
}

//---------------------------------------------------------
// Procedure: contestVerdict()
//   Compare our ETA to the best opponent ETA for a swimmer. Fills the
//   score multiplier (factor) and the ETA margin (opp - us, so >0 means
//   we are faster), and returns a short label for the report.

string GenRescue::contestVerdict(double tx, double ty,
                                 double &factor, double &margin)
{
  factor = 1.0;
  margin = 0;

  bool   have_opp = false;
  double opp_eta  = opponentETA(tx, ty, have_opp);
  if(!have_opp)
    return("-");                     // no opponent -> neutral (today's behavior)

  double our_eta = ownshipETA(tx, ty);
  margin = opp_eta - our_eta;

  if(margin < -m_lose_margin)        // opponent clearly wins -> skip it
    return("LOST");
  if(margin < m_contest_window) {    // close either way -> fight for it
    factor = m_contest_boost;
    return("CONTESTED");
  }
  return("WIN");                     // we win comfortably -> no rush
}

//---------------------------------------------------------
// Procedure: haveFreshOpponent()

bool GenRescue::haveFreshOpponent()
{
  double now = MOOSTime();
  map<string, Contact>::iterator p;
  for(p=m_contacts.begin(); p!=m_contacts.end(); p++) {
    if((now - p->second.utc) <= m_contact_max_age)
      return(true);
  }
  return(false);
}

//---------------------------------------------------------
// Procedure: pathTime()
//   Total time to drive ownship through the waypoint list in order,
//   using the same travel + turn model as the ordering cost. The turn
//   cost at each leg depends on the heading carried from the prior leg.

double GenRescue::pathTime(const vector<double> &vx, const vector<double> &vy)
{
  double t  = 0;
  double cx = m_nav_x;
  double cy = m_nav_y;
  double ch = m_nav_heading;
  for(unsigned int i=0; i<vx.size(); i++) {
    t += etaToPoint(cx, cy, ch, m_speed, vx[i], vy[i]);
    ch = relAng(cx, cy, vx[i], vy[i]);   // heading we arrive on
    cx = vx[i];
    cy = vy[i];
  }
  return(t);
}

//---------------------------------------------------------
// Procedure: twoOptImprove()
//   Repeatedly reverse a stretch of the tour whenever doing so lowers
//   total traversal time. This uncrosses the greedy tour and removes
//   U-turns. Waypoint 0 is pinned so the contest-first pick stays put.

XYSegList GenRescue::twoOptImprove(XYSegList path)
{
  unsigned int n = path.size();
  if(!m_use_two_opt || (n < 3))
    return(path);

  // Work on plain arrays of the vertices.
  vector<double> vx, vy;
  for(unsigned int i=0; i<n; i++) {
    vx.push_back(path.get_vx(i));
    vy.push_back(path.get_vy(i));
  }

  bool         improved = true;
  unsigned int guard    = 0;
  while(improved && (guard < 100)) {
    improved = false;
    guard++;
    double base = pathTime(vx, vy);

    // i starts at 1: waypoint 0 is pinned (the contest-first target).
    for(unsigned int i=1; (i+1)<n; i++) {
      for(unsigned int j=i+1; j<n; j++) {
        // Candidate: reverse the [i..j] stretch.
        vector<double> cx = vx;
        vector<double> cy = vy;
        unsigned int a = i, b = j;
        while(a < b) {
          double tx = cx[a]; cx[a] = cx[b]; cx[b] = tx;
          double ty = cy[a]; cy[a] = cy[b]; cy[b] = ty;
          a++;
          b--;
        }
        double t = pathTime(cx, cy);
        if((t + 1e-6) < base) {
          vx = cx;
          vy = cy;
          base = t;
          improved = true;
        }
      }
    }
  }

  XYSegList out;
  for(unsigned int i=0; i<n; i++)
    out.add_vertex(vx[i], vy[i]);
  return(out);
}

//---------------------------------------------------------
// Procedure: orOptImprove()
//   Relocate a run of 1-3 consecutive waypoints to a better slot
//   (orientation kept; reversal is 2-opt's job) whenever it lowers
//   total traversal time. Waypoint 0 stays pinned.

XYSegList GenRescue::orOptImprove(XYSegList path)
{
  unsigned int n = path.size();
  if(!m_use_or_opt || (n < 3))
    return(path);

  vector<double> vx, vy;
  for(unsigned int i=0; i<n; i++) {
    vx.push_back(path.get_vx(i));
    vy.push_back(path.get_vy(i));
  }

  bool         improved = true;
  unsigned int guard    = 0;
  while(improved && (guard < 100)) {
    improved = false;
    guard++;
    double base = pathTime(vx, vy);

    // Try moving a run of length L (1..3), starting at i (>=1, pinned 0).
    for(unsigned int L=1; (L<=3) && (L<n); L++) {
      for(unsigned int i=1; (i+L)<=n; i++) {
        // The run being moved, and the remaining tour without it.
        vector<double> sx(vx.begin()+i, vx.begin()+i+L);
        vector<double> sy(vy.begin()+i, vy.begin()+i+L);
        vector<double> rx, ry;
        for(unsigned int t=0; t<n; t++) {
          if((t < i) || (t >= i+L)) {
            rx.push_back(vx[t]);
            ry.push_back(vy[t]);
          }
        }
        // Reinsert the run at each gap p (>=1 keeps waypoint 0 pinned).
        for(unsigned int p=1; p<=rx.size(); p++) {
          vector<double> cx, cy;
          for(unsigned int t=0; t<p; t++)        { cx.push_back(rx[t]); cy.push_back(ry[t]); }
          for(unsigned int t=0; t<L; t++)        { cx.push_back(sx[t]); cy.push_back(sy[t]); }
          for(unsigned int t=p; t<rx.size(); t++){ cx.push_back(rx[t]); cy.push_back(ry[t]); }

          double tt = pathTime(cx, cy);
          if((tt + 1e-6) < base) {
            vx = cx;
            vy = cy;
            base = tt;
            improved = true;
          }
        }
      }
    }
  }

  XYSegList out;
  for(unsigned int i=0; i<vx.size(); i++)
    out.add_vertex(vx[i], vy[i]);
  return(out);
}

//---------------------------------------------------------
// Procedure: postShortestPath()

void GenRescue::postShortestPath()
{
  // Stage 1 (idea A): contest filter. Each swimmer gets an opponent
  // factor; swimmers the opponent clearly beats us to ("LOST") are
  // dropped so we don't waste travel on them.
  vector<double> sx_list, sy_list, f_list;
  map<string, XYPoint>::iterator it;
  for(it=m_swimmers.begin(); it!=m_swimmers.end(); it++) {
    XYPoint pt = it->second;
    if(swimmerNearBuoy(pt.x(), pt.y()))   // on/by a buoy -> ignore entirely
      continue;
    double  factor, margin;
    string  verdict = contestVerdict(pt.x(), pt.y(), factor, margin);
    if(verdict == "LOST")
      continue;
    sx_list.push_back(pt.x());
    sy_list.push_back(pt.y());
    f_list.push_back(factor);
  }

  // Safety net: never idle. If the contest filter dropped everyone,
  // re-add all swimmers with neutral factors and go after them anyway
  // (still excluding the on/by-buoy ones we permanently ignore).
  if(sx_list.empty()) {
    for(it=m_swimmers.begin(); it!=m_swimmers.end(); it++) {
      XYPoint pt = it->second;
      if(swimmerNearBuoy(pt.x(), pt.y()))
        continue;
      sx_list.push_back(pt.x());
      sy_list.push_back(pt.y());
      f_list.push_back(1.0);
    }
  }

  // If every remaining swimmer is on/by a buoy, there is nothing worth
  // touring. Return without posting so the survey can complete and (with
  // no rescuable swimmers left -- see Iterate) the boat heads home.
  if(sx_list.empty())
    return;

  // Stage 2 (idea B): coverage reduction. Collapse the survivors to a
  // minimal set of visit points so every swimmer is within
  // m_cover_range of one. A covered swimmer needs no waypoint of its
  // own -- the boat sweeps through its detection ring in passing. Each
  // visit point inherits the strongest (lowest) factor it covers and
  // counts a multiplicity (how many swimmers it stands in for) so the
  // cluster density below still reflects true swimmer counts.
  vector<double> kx, ky, kf, km;
  for(unsigned int i=0; i<sx_list.size(); i++) {
    int cover_ix = -1;
    for(unsigned int j=0; j<kx.size(); j++) {
      if(distPointToPoint(sx_list[i], sy_list[i], kx[j], ky[j]) <= m_cover_range) {
        cover_ix = (int)j;
        break;
      }
    }
    if(cover_ix >= 0) {
      if(f_list[i] < kf[cover_ix])
        kf[cover_ix] = f_list[i];
      km[cover_ix] += 1.0;
    }
    else {
      kx.push_back(sx_list[i]);
      ky.push_back(sy_list[i]);
      kf.push_back(f_list[i]);
      km.push_back(1.0);
    }
  }

  // Stage 3: assemble the candidate seglist + aligned factors + mults.
  // True swimmer coordinates are used for ordering; the boundary inset
  // is applied afterward (tightenForTurns) once turn angles are known.
  XYSegList      swim_pts;
  vector<double> factors, mults;
  for(unsigned int i=0; i<kx.size(); i++) {
    swim_pts.add_vertex(kx[i], ky[i]);
    factors.push_back(kf[i]);
    mults.push_back(km[i]);
  }

  // Order the swimmer points into a short tour, starting from
  // ownship's position and heading. Like nearest-neighbor, but cost
  // is travel+turn time discounted by local swimmer density (and by
  // the contest factor), so we dive into packs first, avoid U-turns,
  // and prioritize swimmers we can deny the opponent.
  m_path = clusterPath(swim_pts, m_nav_x, m_nav_y, m_nav_heading, factors, mults);

  // Uncross the greedy tour (2-opt), then relocate runs (Or-opt), to cut
  // traversal time and remove U-turns, before the boundary inset reads
  // the final turn angles.
  m_path = twoOptImprove(m_path);
  m_path = orOptImprove(m_path);

  // Pull waypoints inside the boundary, more where the tour turns hard,
  // so turn overshoot can never carry the boat out of bounds.
  m_path = tightenForTurns(m_path);

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

XYSegList GenRescue::clusterPath(XYSegList swim_pts, double sx, double sy, double sh,
                                 vector<double> factors, vector<double> mults)
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

      // Local density = number of OTHER still-unvisited swimmers near j.
      // Visit points carry a multiplicity, so a coverage-merged pack
      // contributes its full swimmer count (mults[j]-1 for the swimmers
      // sharing j's own visit point, plus the neighbors' multiplicities).
      double density = ((j < mults.size()) ? (mults[j] - 1.0) : 0.0);
      for(k=0; k<vsize; k++) {
        if((k == j) || visited[k])
          continue;
        if(distPointToPoint(vx[j], vy[j], vx[k], vy[k]) <= m_cluster_radius)
          density += ((k < mults.size()) ? mults[k] : 1.0);
      }

      // Discount the time-to-target by local density, then apply the
      // opponent-contest factor (1.0 when no opponent / not contested).
      double score = cost / (1.0 + (m_cluster_weight * density));
      if(j < factors.size())
        score = score * factors[j];

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
  m_msgs << "  Cover range:              " << doubleToStringX(m_cover_range,1)
         << " m"                                                      << endl;
  m_msgs << "  Boundary margin:          " << doubleToStringX(m_boundary_margin,1)
         << " +" << doubleToStringX(m_overshoot_max,1) << "/turn m  (region "
         << (m_region_set ? "known" : "unknown") << ")"               << endl;
  m_msgs << "  Buoys known:              " << m_buoys.size()
         << "  (ignore swimmers within " << doubleToStringX(m_buoy_ignore_radius,1)
         << " m of center)" << endl;
  m_msgs << "  Tour cleanup:             2-opt " << (m_use_two_opt ? "on" : "off")
         << ", Or-opt " << (m_use_or_opt ? "on" : "off")              << endl;
  m_msgs << "  Re-sweep interval:        every " << doubleToStringX(m_replan_interval,1)
         << " s"                                                       << endl;
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

  // Opponent summary (fresh contacts only).
  m_msgs << "  Opponent:         ";
  if(!haveFreshOpponent())
    m_msgs << "none seen";
  else {
    double now = MOOSTime();
    map<string, Contact>::iterator cp;
    for(cp=m_contacts.begin(); cp!=m_contacts.end(); cp++) {
      Contact c = cp->second;
      if((now - c.utc) > m_contact_max_age)
        continue;
      m_msgs << cp->first << " pos=(" << doubleToStringX(c.x,1) << ", "
             << doubleToStringX(c.y,1) << ") spd=" << doubleToStringX(c.speed,1)
             << " hdg=" << doubleToStringX(c.heading,0)
             << " age=" << doubleToStringX(now - c.utc,0) << "s  ";
    }
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
  m_msgs << "  Path legs:        " << m_path.size()
         << "  (visit points; " << m_swimmers.size() << " swimmers covered)" << endl;
  m_msgs << endl;

  // --- Section 3: Active swimmers (id | location | range-from-ownship) ---
  m_msgs << "Active swimmers"                                         << endl;
  ACTable actab(4);
  actab << "ID | Location (x, y) | Range | Contest";
  actab.addHeaderLines();
  map<string, XYPoint>::iterator it;
  for(it=m_swimmers.begin(); it!=m_swimmers.end(); it++) {
    string  id  = it->first;
    XYPoint pt  = it->second;
    string  loc = doubleToStringX(pt.x(),1) + ", " + doubleToStringX(pt.y(),1);
    string  rng = "--";
    if(m_nav_x_set && m_nav_y_set)
      rng = doubleToStringX(distPointToPoint(m_nav_x,m_nav_y,pt.x(),pt.y()),0);

    // Contest verdict (+margin in seconds when an opponent is known).
    double factor, margin;
    string verdict = "-";
    if(m_nav_x_set && m_nav_y_set) {
      verdict = contestVerdict(pt.x(), pt.y(), factor, margin);
      if(verdict != "-")
        verdict += " (" + doubleToStringX(margin,0) + "s)";
    }
    actab << id << loc << rng << verdict;
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
