/************************************************************/
/*    NAME: G Souchlas                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: PointAssign.cpp                                        */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#include <iterator>
#include <algorithm>
#include "MBUtils.h"
#include "ACTable.h"
#include "XYPoint.h"
#include "PointAssign.h"

using namespace std;

//---------------------------------------------------------
// Constructor()

PointAssign::PointAssign()
{
  m_input_var       = "VISIT_POINT";
  m_assign_by_region = true;   // default: split east-west by region
  m_unpause_var      = "UTS_PAUSE";
  m_ready_timeout    = 30;      // fail-safe: un-pause anyway after this many secs

  m_collecting      = false;
  m_unpaused_sent   = false;
  m_first_iter_time = 0;
  m_batches_done    = 0;
}

//---------------------------------------------------------
// Destructor

PointAssign::~PointAssign()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool PointAssign::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    string key = msg.GetKey();

    if(key == m_input_var)
      handleVisitPoint(msg.GetString());

    else if(key == "PGENPATH_READY")
      m_ready_vehicles.insert(tolower(stripBlankEnds(msg.GetString())));

    else if(key != "APPCAST_REQ") // handled by AppCastingMOOSApp
      reportRunWarning("Unhandled Mail: " + key);
  }

  return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool PointAssign::OnConnectToServer()
{
  registerVariables();
  return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool PointAssign::Iterate()
{
  AppCastingMOOSApp::Iterate();

  // Two-level handshake (lab 3.6.1). We un-pause the timer (which releases the
  // whole point pipeline) only once BOTH are true:
  //   - we are connected + registered for the input var (true from iter 1), AND
  //   - every configured vehicle's pGenPath has announced PGENPATH_READY, so the
  //     shore->vehicle bridges are up and the burst won't be missed downstream.
  // A timeout is a fail-safe so a missing handshake can't deadlock the mission.
  if(!m_unpaused_sent && (m_unpause_var != "")) {
    if(m_first_iter_time == 0)
      m_first_iter_time = m_curr_time;

    bool timed_out = (m_curr_time - m_first_iter_time) > m_ready_timeout;
    if(allVehiclesReady() || timed_out) {
      Notify(m_unpause_var, "false");
      m_unpaused_sent = true;
    }
  }

  // All other work is event-driven in OnNewMail; nothing else to do per-tick.
  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: allVehiclesReady()
//   True once every configured vehicle has announced PGENPATH_READY.

bool PointAssign::allVehiclesReady() const
{
  for(unsigned int v=0; v<m_vehicles.size(); v++) {
    if(m_ready_vehicles.count(tolower(m_vehicles[v])) == 0)
      return(false);
  }
  return(true);
}

//---------------------------------------------------------
// Procedure: handleVisitPoint()
//   Handles one incoming VISIT_POINT message. The timer script
//   brackets each batch of points with "firstpoint" / "lastpoint":
//     firstpoint  -> start a fresh batch
//     x=..,y=..,id=..  -> buffer the point
//     lastpoint   -> split the batch and publish

void PointAssign::handleVisitPoint(const string& sval)
{
  string val = stripBlankEnds(sval);

  if(val == "firstpoint") {
    m_collecting = true;
    m_pt_str.clear();
    m_pt_x.clear();
    return;
  }

  if(val == "lastpoint") {
    if(m_collecting)
      assignAndPublish();
    m_collecting = false;
    return;
  }

  // Otherwise it should be a point "x=..,y=..,id=..".
  if(!m_collecting) {
    reportRunWarning("Point received outside firstpoint/lastpoint: " + val);
    return;
  }

  double x = 0;
  if(!tokParse(val, "x", ',', '=', x)) {
    reportRunWarning("Malformed point (no x): " + val);
    return;
  }

  m_pt_str.push_back(val);
  m_pt_x.push_back(x);
}

//---------------------------------------------------------
// Procedure: assignAndPublish()
//   Distributes the buffered points among the configured vehicles
//   in one of two configurable ways:
//     assign_by_region = true  -> split east-west by x. Indices are
//        sorted by x and cut into equal contiguous chunks, so the
//        first vehicle gets the western-most points, etc.
//     assign_by_region = false -> alternating (round-robin) in arrival
//        order: pt 0 -> vehicle 0, pt 1 -> vehicle 1, wrapping around.
//   Each vehicle's share is wrapped in its own firstpoint/lastpoint,
//   posted even when that vehicle's share is empty, so the receiving
//   side always gets the start/end markers.

void PointAssign::assignAndPublish()
{
  unsigned int nveh = m_vehicles.size();
  if(nveh == 0)
    return;

  unsigned int npts = m_pt_str.size();

  // buckets[v] = list of point indices assigned to vehicle v
  vector<vector<unsigned int> > buckets(nveh);

  if(m_assign_by_region) {
    // Order point indices by ascending x (west -> east), then cut into
    // nveh contiguous chunks. With 2 vehicles this is west-half/east-half.
    vector<unsigned int> order(npts);
    for(unsigned int i=0; i<npts; i++)
      order[i] = i;
    sort(order.begin(), order.end(),
         [&](unsigned int a, unsigned int b){ return m_pt_x[a] < m_pt_x[b]; });

    for(unsigned int k=0; k<npts; k++) {
      unsigned int v = (k * nveh) / npts;   // 0..nveh-1, in x order
      buckets[v].push_back(order[k]);
    }
  }
  else {
    // Alternating: round-robin in arrival order.
    for(unsigned int k=0; k<npts; k++)
      buckets[k % nveh].push_back(k);
  }

  // One distinct viewer color per vehicle (wraps if more vehicles than colors).
  vector<string> palette;
  palette.push_back("yellow");
  palette.push_back("dodger_blue");
  palette.push_back("red");
  palette.push_back("green");
  palette.push_back("magenta");
  palette.push_back("orange");

  // Publish each vehicle's share to <input_var>_<VEHICLE>. The suffix is
  // UPPERCASED because uFldShoreBroker's "qbridge = VISIT_POINT" expands $V
  // via toupper(community) -> it bridges VISIT_POINT_HENRY, not _henry.
  m_last_counts.assign(nveh, 0);
  for(unsigned int v=0; v<nveh; v++) {
    string out_var = m_input_var + "_" + toupper(m_vehicles[v]);
    string color   = palette[v % palette.size()];

    Notify(out_var, "firstpoint");
    for(unsigned int j=0; j<buckets[v].size(); j++) {
      string spec = m_pt_str[buckets[v][j]];
      Notify(out_var, spec);

      // Draw the point in pMarineViewer, colored by which vehicle got it.
      // Label = the point's id so each marker is unique (else they overwrite).
      double x = 0, y = 0;
      string id;
      tokParse(spec, "x",  ',', '=', x);
      tokParse(spec, "y",  ',', '=', y);
      tokParse(spec, "id", ',', '=', id);
      postViewPoint(x, y, id, color);
    }
    Notify(out_var, "lastpoint");
    m_last_counts[v] = buckets[v].size();
  }

  m_batches_done++;
}

//---------------------------------------------------------
// Procedure: postViewPoint()
//   Posts a VIEW_POINT so pMarineViewer renders the visit point.
//   label must be unique per point or the viewer overwrites markers.

void PointAssign::postViewPoint(double x, double y, string label, string color)
{
  XYPoint point(x, y);
  point.set_label(label);
  point.set_color("vertex", color);
  point.set_param("vertex_size", "4");

  string spec = point.get_spec();
  Notify("VIEW_POINT", spec);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool PointAssign::OnStartUp()
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
    if(param == "input_var") {
      m_input_var = value;
      handled = true;
    }
    else if(param == "vname") {
      // Each "vname" line adds one vehicle to the distribution list.
      m_vehicles.push_back(value);
      handled = true;
    }
    else if(param == "assign_by_region") {
      handled = setBooleanOnString(m_assign_by_region, value);
    }
    else if(param == "unpause_var") {
      m_unpause_var = value;
      handled = true;
    }
    else if(param == "ready_timeout") {
      handled = setDoubleOnString(m_ready_timeout, value);
    }

    if(!handled)
      reportUnhandledConfigWarning(orig);
  }

  if(m_vehicles.empty())
    reportConfigWarning("No vname configured: no vehicles to distribute points to.");

  registerVariables();
  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables()

void PointAssign::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register(m_input_var, 0);
  Register("PGENPATH_READY", 0);
}

//------------------------------------------------------------
// Procedure: buildReport()

bool PointAssign::buildReport()
{
  m_msgs << "Subscribing to:   " << m_input_var << endl;
  m_msgs << "Vehicles ready:   " << m_ready_vehicles.size() << "/"
         << m_vehicles.size() << " (" << boolToString(allVehiclesReady()) << ")" << endl;
  m_msgs << "Timer un-pause:   " << m_unpause_var << "=false "
         << (m_unpaused_sent ? "(sent)" : "(pending)") << endl;
  m_msgs << "Assign mode:      "
         << (m_assign_by_region ? "by region (east-west)" : "alternating") << endl;
  m_msgs << "Vehicles:         " << m_vehicles.size() << endl;
  m_msgs << "Batches assigned: " << m_batches_done << endl;
  m_msgs << "Collecting now:   " << (m_collecting ? "yes" : "no");
  if(m_collecting)
    m_msgs << " (" << m_pt_str.size() << " points buffered)";
  m_msgs << endl;
  m_msgs << "============================================" << endl;

  ACTable actab(3);
  actab << "Vehicle | Output Var | Points (last batch)";
  actab.addHeaderLines();
  for(unsigned int v=0; v<m_vehicles.size(); v++) {
    string cnt = (v < m_last_counts.size()) ? uintToString(m_last_counts[v]) : "0";
    actab << m_vehicles[v] << (m_input_var + "_" + toupper(m_vehicles[v])) << cnt;
  }
  m_msgs << actab.getFormattedString();

  return(true);
}
