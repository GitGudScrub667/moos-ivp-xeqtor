/*****************************************************************/
/*    NAME: M.Benjamin                                           */
/*    ORGN: Dept of Mechanical Eng / CSAIL, MIT Cambridge MA     */
/*    FILE: BHV_Scout.cpp                                        */
/*    DATE: April 30th 2022                                      */
/*****************************************************************/

#include <cstdlib>
#include <math.h>
#include "BHV_Scout.h"
#include "MBUtils.h"
#include "AngleUtils.h"
#include "BuildUtils.h"
#include "GeomUtils.h"
#include "ZAIC_PEAK.h"
#include "OF_Coupler.h"
#include "XYFormatUtilsPoly.h"
#include "NodeRecord.h"
#include "NodeRecordUtils.h"

using namespace std;

//-----------------------------------------------------------
// Constructor()

BHV_Scout::BHV_Scout(IvPDomain gdomain) : 
  IvPBehavior(gdomain)
{
  IvPBehavior::setParam("name", "scout");
 
  // Default values for behavior state variables
  m_osx  = 0;
  m_osy  = 0;

  // All distances are in meters, all speed in meters per second
  // Default values for configuration parameters 
  m_desired_speed  = 2;    // faster search -> more area per minute
  m_capture_radius = 10;
  m_sweep_radius   = 15;   // grid cell + mark radius: tight enough for real coverage

  m_pt_set = false;
  m_grid_ready = false;
  m_order_idx = 0;
  m_pushing = false;

  // COLREGS push tuning (defaults).
  m_push_enabled      = true;
  m_push_edge_dist    = 15;   // m: opponent this close to the boundary is push-eligible
  m_push_engage_range = 55;   // m: ... and this close to ben -> engage
  m_push_standoff     = 10;   // m: how far interior of the opponent ben aims

  addInfoVars("NAV_X, NAV_Y");
  addInfoVars("RESCUE_REGION");
  addInfoVars("SCOUTED_SWIMMER");
  addInfoVars("NODE_REPORT");        // positions of teammate + opponents
  addInfoVars("NODE_REPORT_LOCAL");  // our own report -> learn our team color
}

//---------------------------------------------------------------
// Procedure: setParam() - handle behavior configuration parameters

bool BHV_Scout::setParam(string param, string val) 
{
  // Convert the parameter to lower case for more general matching
  param = tolower(param);
  
  bool handled = true;
  if(param == "capture_radius")
    handled = setPosDoubleOnString(m_capture_radius, val);
  else if(param == "desired_speed")
    handled = setPosDoubleOnString(m_desired_speed, val);
  else if(param == "sweep_radius")
    handled = setPosDoubleOnString(m_sweep_radius, val);
  else if(param == "tmate")
    handled = setNonWhiteVarOnString(m_tmate, val);
  else if(param == "push_enabled")
    handled = setBooleanOnString(m_push_enabled, val);
  else if(param == "push_edge_dist")
    handled = setPosDoubleOnString(m_push_edge_dist, val);
  else if(param == "push_engage_range")
    handled = setPosDoubleOnString(m_push_engage_range, val);
  else if(param == "push_standoff")
    handled = setPosDoubleOnString(m_push_standoff, val);
  else
    handled = false;

  srand(time(NULL));
  
  return(handled);
}

//-----------------------------------------------------------
// Procedure: onEveryState()

void BHV_Scout::onEveryState(string str) 
{
  if(!getBufferVarUpdated("SCOUTED_SWIMMER"))
    return;

  string report = getBufferStringVal("SCOUTED_SWIMMER");
  if(report == "")
    return;

  if(m_tmate == "") {
    postWMessage("Mandatory Teammate name is null");
    return;
  }
  postOffboardMessage(m_tmate, "SWIMMER_ALERT", report);
}

//-----------------------------------------------------------
// Procedure: onIdleState()

void BHV_Scout::onIdleState() 
{
  m_curr_time = getBufferCurrTime();
}

//-----------------------------------------------------------
// Procedure: onRunState()

IvPFunction *BHV_Scout::onRunState() 
{
  // Part 1: Get vehicle position from InfoBuffer and post a 
  // warning if problem is encountered
  bool ok1, ok2;
  m_osx = getBufferDoubleVal("NAV_X", ok1);
  m_osy = getBufferDoubleVal("NAV_Y", ok2);
  if(!ok1 || !ok2) {
    postWMessage("No ownship X/Y info in info_buffer.");
    return(0);
  }
  
  // Part 2: Build the coverage grid once the region is known, refresh the
  // tracked vehicles, and mark cells swept by any vehicle (us + contacts).
  if(!m_grid_ready)
    buildGrid();
  updateContacts();
  markVehiclesSwept();

  // Part 2.5: COLREGS push. If an opponent is hugging the region edge near
  // us, shoulder it toward the boundary this tick instead of searching.
  if(m_push_enabled) {
    double px, py;
    if(findPushTarget(px, py)) {
      m_pushing = true;
      m_ptx = px; m_pty = py; m_pt_set = true;
      postViewPoint(true);
      IvPFunction *ipf = buildFunction();
      if(ipf == 0)
        postWMessage("Problem Creating the IvP Function");
      return(ipf);
    }
  }
  // Push just ended -> drop the stale push target so search re-picks fresh.
  if(m_pushing) {
    m_pushing = false;
    m_pt_set  = false;
  }

  // Part 3: Choose / refresh the scout target (nearest unswept cell).
  updateScoutPoint();

  // Part 4: If we've reached the target, clear it so a new one is picked
  // next tick. If we have no target (region fully swept), just idle.
  double dist = hypot((m_ptx-m_osx), (m_pty-m_osy));
  if(m_pt_set && (dist <= m_capture_radius)) {
    m_pt_set = false;
    postViewPoint(false);
    return(0);
  }
  if(!m_pt_set) {
    postViewPoint(false);
    return(0);
  }

  // Part 5: Post the target for the viewer and build the IvP function.
  postViewPoint(true);
  IvPFunction *ipf = buildFunction();
  if(ipf == 0)
    postWMessage("Problem Creating the IvP Function");

  return(ipf);
}

//-----------------------------------------------------------
// Procedure: updateScoutPoint()

void BHV_Scout::updateScoutPoint()
{
  if(!m_grid_ready)
    return;

  // Abandon the current target if its grid cell has since been swept
  // (by us or anyone else) -- no reason to keep driving to covered area.
  if(m_pt_set) {
    int    best = -1;
    double bestd = -1;
    for(unsigned int i=0; i<m_cell_x.size(); i++) {
      double d = hypot(m_cell_x[i]-m_ptx, m_cell_y[i]-m_pty);
      if((best < 0) || (d < bestd)) { bestd = d; best = (int)i; }
    }
    if((best >= 0) && m_cell_covered[best])
      m_pt_set = false;
  }

  if(m_pt_set)
    return;   // still pursuing a valid (unswept) target

  // Pick the next unswept cell in serpentine order as the new target.
  double rx, ry;
  if(pickNextInOrder(rx, ry)) {
    m_ptx = rx;
    m_pty = ry;
    m_pt_set = true;
    postEventMessage("Scout pt: " + doubleToStringX(rx,1) + "," + doubleToStringX(ry,1));
  }
  // else: every cell swept -> leave m_pt_set false (idle); region done.
}

//-----------------------------------------------------------
// Procedure: buildGrid()
//   Lay a grid over the rescue region (cell size = sweep radius),
//   keeping only cells whose center is inside the region. Done once.

void BHV_Scout::buildGrid()
{
  string region_str = getBufferStringVal("RESCUE_REGION");
  if(region_str == "") {
    postWMessage("Unknown RESCUE_REGION");
    return;
  }
  postRetractWMessage("Unknown RESCUE_REGION");

  XYPolygon region = string2Poly(region_str);
  if(!region.is_convex() || (region.size() < 3)) {
    postWMessage("Badly formed RESCUE_REGION");
    return;
  }
  m_rescue_region = region;

  m_cell_x.clear();
  m_cell_y.clear();
  m_cell_covered.clear();

  double step = m_sweep_radius;
  if(step <= 0)
    step = 25;

  // Bounding box from the polygon vertices.
  double minx = region.get_vx(0), maxx = minx;
  double miny = region.get_vy(0), maxy = miny;
  for(unsigned int i=1; i<region.size(); i++) {
    double vx = region.get_vx(i), vy = region.get_vy(i);
    minx = fmin(minx, vx); maxx = fmax(maxx, vx);
    miny = fmin(miny, vy); maxy = fmax(maxy, vy);
  }

  for(double x=minx; x<=maxx; x+=step) {
    for(double y=miny; y<=maxy; y+=step) {
      if(region.contains(x, y)) {
        m_cell_x.push_back(x);
        m_cell_y.push_back(y);
        m_cell_covered.push_back(false);
      }
    }
  }

  if(m_cell_x.size() > 0) {
    buildSweepOrder();
    m_grid_ready = true;
    postEventMessage("Scout grid built: " + uintToString(m_cell_x.size()) + " cells");
  }
}

//-----------------------------------------------------------
// Procedure: buildSweepOrder()
//   Order the grid cells into a serpentine (boustrophedon) lawnmower
//   path: sweep in rows along the region's WIDER axis (so each row is
//   long and there are few turns), and reverse the direction every other
//   row so the scout snakes back and forth instead of jumping to the row
//   start. Stored as m_order (cell indices); m_order_idx is the cursor.

void BHV_Scout::buildSweepOrder()
{
  m_order.clear();
  m_order_idx = 0;
  unsigned int n = m_cell_x.size();
  if(n == 0)
    return;

  double minx = m_cell_x[0], maxx = minx;
  double miny = m_cell_y[0], maxy = miny;
  for(unsigned int i=1; i<n; i++) {
    minx = fmin(minx, m_cell_x[i]); maxx = fmax(maxx, m_cell_x[i]);
    miny = fmin(miny, m_cell_y[i]); maxy = fmax(maxy, m_cell_y[i]);
  }

  double step = (m_sweep_radius > 0) ? m_sweep_radius : 25;

  // Sweep along the wider axis -> long rows, fewer turns.
  bool sweep_x = ((maxx - minx) >= (maxy - miny));

  // Walk row by row along the stepping (narrower) axis.
  double rmin = sweep_x ? miny : minx;
  double rmax = sweep_x ? maxy : maxx;
  int rownum = 0;
  for(double r = rmin; r <= rmax + 1e-6; r += step) {
    // Gather the cells belonging to this row (within half a step of r).
    vector<unsigned int> row;
    for(unsigned int i=0; i<n; i++) {
      double rc = sweep_x ? m_cell_y[i] : m_cell_x[i];
      if(fabs(rc - r) <= (step / 2.0))
        row.push_back(i);
    }
    // Sort the row by the sweep coordinate, ascending (selection sort;
    // rows are tiny so this is plenty fast and easy to read).
    for(unsigned int a=0; a<row.size(); a++) {
      unsigned int best = a;
      for(unsigned int b=a+1; b<row.size(); b++) {
        double sb    = sweep_x ? m_cell_x[row[b]]    : m_cell_y[row[b]];
        double sbest = sweep_x ? m_cell_x[row[best]] : m_cell_y[row[best]];
        if(sb < sbest) best = b;
      }
      unsigned int tmp = row[a]; row[a] = row[best]; row[best] = tmp;
    }
    // Serpentine: even rows left-to-right, odd rows right-to-left.
    if((rownum % 2) == 0) {
      for(unsigned int a=0; a<row.size(); a++)
        m_order.push_back(row[a]);
    }
    else {
      for(int a=(int)row.size()-1; a>=0; a--)
        m_order.push_back(row[a]);
    }
    rownum++;
  }
}

//-----------------------------------------------------------
// Procedure: markSwept()
//   Mark every cell within m_sweep_radius of (px,py) as swept.

void BHV_Scout::markSwept(double px, double py)
{
  for(unsigned int i=0; i<m_cell_x.size(); i++) {
    if(m_cell_covered[i])
      continue;
    if(hypot(m_cell_x[i]-px, m_cell_y[i]-py) <= m_sweep_radius)
      m_cell_covered[i] = true;
  }
}

//-----------------------------------------------------------
// Procedure: markVehiclesSwept()
//   Mark cells swept by ownship and by every tracked vehicle (teammate +
//   opponents). m_contacts is refreshed each tick by updateContacts().

void BHV_Scout::markVehiclesSwept()
{
  if(!m_grid_ready)
    return;

  markSwept(m_osx, m_osy);   // our own track

  map<string, Contact>::iterator it;
  for(it=m_contacts.begin(); it!=m_contacts.end(); it++)
    markSwept(it->second.x, it->second.y);
}

//-----------------------------------------------------------
// Procedure: updateContacts()
//   Refresh the tracked-vehicle map from the latest NODE_REPORT (sampling
//   each tick rotates through all craft over time), and learn our own team
//   color from NODE_REPORT_LOCAL.

void BHV_Scout::updateContacts()
{
  // Our own team color (string2NodeRecord doesn't parse COLOR -> read spec).
  if(getBufferIsKnown("NODE_REPORT_LOCAL")) {
    string c = colorOfReport(getBufferStringVal("NODE_REPORT_LOCAL"));
    if(c != "")
      m_my_color = c;
  }

  if(getBufferIsKnown("NODE_REPORT")) {
    string rep = getBufferStringVal("NODE_REPORT");
    if(rep != "") {
      NodeRecord rec = string2NodeRecord(rep);
      string name = rec.getName();
      if((name != "") && (name != m_us_name)) {
        Contact c;
        c.x       = rec.getX();
        c.y       = rec.getY();
        c.heading = rec.getHeading();
        c.utc     = getBufferCurrTime();
        c.color   = colorOfReport(rep);
        m_contacts[name] = c;
      }
    }
  }
}

//-----------------------------------------------------------
// Procedure: colorOfReport()
//   Pull the COLOR field (lowercased) out of a NODE_REPORT spec. "" if
//   absent. (string2NodeRecord does not populate color.)

string BHV_Scout::colorOfReport(string str)
{
  vector<string> svector = parseString(str, ',');
  for(unsigned int i=0; i<svector.size(); i++) {
    string param = tolower(biteStringX(svector[i], '='));
    if(param == "color")
      return(tolower(svector[i]));
  }
  return("");
}

//-----------------------------------------------------------
// Procedure: findPushTarget()
//   If a different-team opponent is hugging the region edge AND near us,
//   return a target on its interior-forward quarter so our approach forces
//   its give-way turn toward/over the boundary. Returns false otherwise
//   (-> keep searching). Never targets a teammate or an unknown craft.

bool BHV_Scout::findPushTarget(double& tx, double& ty)
{
  if(!m_grid_ready || (m_rescue_region.size() < 3))
    return(false);

  double now = getBufferCurrTime();

  // Region centroid (interior reference).
  double rcx = 0, rcy = 0;
  for(unsigned int v=0; v<m_rescue_region.size(); v++) {
    rcx += m_rescue_region.get_vx(v);
    rcy += m_rescue_region.get_vy(v);
  }
  rcx /= m_rescue_region.size();
  rcy /= m_rescue_region.size();

  // Nearest engage-able opponent.
  bool   have = false;
  double best_d = 0, ox = 0, oy = 0, oh = 0;
  map<string, Contact>::iterator it;
  for(it=m_contacts.begin(); it!=m_contacts.end(); it++) {
    Contact c = it->second;
    // Opponent only if KNOWN to be a different team color (never a teammate
    // or an unknown -- we must not push our own rescue boat).
    if((m_my_color == "") || (c.color == "") || (c.color == m_my_color))
      continue;
    if(it->first == m_tmate)
      continue;
    if((now - c.utc) > 10)                 // stale contact
      continue;
    if(!m_rescue_region.contains(c.x, c.y)) // already outside; nothing to do
      continue;
    if(m_rescue_region.dist_to_poly(c.x, c.y) > m_push_edge_dist)
      continue;                            // not near the edge
    double d = hypot(c.x - m_osx, c.y - m_osy);
    if(d > m_push_engage_range)            // too far from us -> keep searching
      continue;
    if(!have || (d < best_d)) {
      have = true; best_d = d;
      ox = c.x; oy = c.y; oh = c.heading;
    }
  }

  if(!have)
    return(false);

  // Interior direction at the opponent (toward the region centroid).
  double iux = rcx - ox, iuy = rcy - oy;
  double imag = hypot(iux, iuy);
  if(imag < 1e-6)
    return(false);
  iux /= imag; iuy /= imag;

  // Opponent forward direction (compass heading: 0=N, clockwise).
  const double PI = 3.141592653589793;
  double ahx = sin(oh * PI / 180.0);
  double ahy = cos(oh * PI / 180.0);

  // Shoulder in: interior of the opponent by the standoff, plus a lead
  // ahead of it so our approach forces an edge-ward give-way turn.
  double lead = m_push_standoff * 0.8;
  tx = ox + (iux * m_push_standoff) + (ahx * lead);
  ty = oy + (iuy * m_push_standoff) + (ahy * lead);

  // Keep our own target inside the region (OpRegion is the hard backstop,
  // but don't ask the boat to chase a point out of bounds).
  if(!m_rescue_region.contains(tx, ty)) {
    double cx, cy;
    if(m_rescue_region.closest_point_on_poly(tx, ty, cx, cy)) {
      double ux = rcx - cx, uy = rcy - cy, m = hypot(ux, uy);
      if(m > 1e-6) { tx = cx + (ux/m)*4; ty = cy + (uy/m)*4; }
    }
  }

  postEventMessage("Scout PUSH opponent at " + doubleToStringX(ox,0) + "," +
                   doubleToStringX(oy,0));
  return(true);
}

//-----------------------------------------------------------
// Procedure: pickNextInOrder()
//   Advance the serpentine cursor to the next still-unswept cell and
//   return its center. Walks the whole order (wrapping) so cells that
//   reopened behind the cursor are not missed. Returns false only if the
//   order is empty.

bool BHV_Scout::pickNextInOrder(double& rx, double& ry)
{
  if(m_order.empty())
    return(false);

  // If every cell is covered, start a fresh sweep from the top of the
  // order. Detection is probabilistic (a single close pass often misses a
  // swimmer), so the scout re-sweeps rather than idling -- repeat passes
  // find more.
  bool any_uncovered = false;
  for(unsigned int i=0; i<m_cell_covered.size(); i++) {
    if(!m_cell_covered[i]) { any_uncovered = true; break; }
  }
  if(!any_uncovered) {
    for(unsigned int i=0; i<m_cell_covered.size(); i++)
      m_cell_covered[i] = false;
    m_order_idx = 0;
    postEventMessage("Scout: region fully swept -> re-sweeping");
  }

  // From the cursor, find the next uncovered cell in serpentine order.
  unsigned int n = m_order.size();
  for(unsigned int k=0; k<n; k++) {
    unsigned int pos = (m_order_idx + k) % n;
    unsigned int ci  = m_order[pos];
    if(!m_cell_covered[ci]) {
      m_order_idx = pos;
      rx = m_cell_x[ci];
      ry = m_cell_y[ci];
      return(true);
    }
  }
  return(false);
}

//-----------------------------------------------------------
// Procedure: postViewPoint()

void BHV_Scout::postViewPoint(bool viewable) 
{

  XYPoint pt(m_ptx, m_pty);
  pt.set_vertex_size(5);
  pt.set_vertex_color("orange");
  pt.set_label(m_us_name + "'s next waypoint");
  
  string point_spec;
  if(viewable)
    point_spec = pt.get_spec("active=true");
  else
    point_spec = pt.get_spec("active=false");
  postMessage("VIEW_POINT", point_spec);
}


//-----------------------------------------------------------
// Procedure: buildFunction()

IvPFunction *BHV_Scout::buildFunction() 
{
  if(!m_pt_set)
    return(0);
  
  ZAIC_PEAK spd_zaic(m_domain, "speed");
  spd_zaic.setSummit(m_desired_speed);
  spd_zaic.setPeakWidth(0.5);
  spd_zaic.setBaseWidth(1.0);
  spd_zaic.setSummitDelta(0.8);  
  if(spd_zaic.stateOK() == false) {
    string warnings = "Speed ZAIC problems " + spd_zaic.getWarnings();
    postWMessage(warnings);
    return(0);
  }
  
  double rel_ang_to_wpt = relAng(m_osx, m_osy, m_ptx, m_pty);
  ZAIC_PEAK crs_zaic(m_domain, "course");
  crs_zaic.setSummit(rel_ang_to_wpt);
  crs_zaic.setPeakWidth(0);
  crs_zaic.setBaseWidth(180.0);
  crs_zaic.setSummitDelta(0);  
  crs_zaic.setValueWrap(true);
  if(crs_zaic.stateOK() == false) {
    string warnings = "Course ZAIC problems " + crs_zaic.getWarnings();
    postWMessage(warnings);
    return(0);
  }

  IvPFunction *spd_ipf = spd_zaic.extractIvPFunction();
  IvPFunction *crs_ipf = crs_zaic.extractIvPFunction();

  OF_Coupler coupler;
  IvPFunction *ivp_function = coupler.couple(crs_ipf, spd_ipf, 50, 50);

  return(ivp_function);
}
