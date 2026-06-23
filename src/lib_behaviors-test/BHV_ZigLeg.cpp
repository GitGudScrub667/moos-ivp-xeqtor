/************************************************************/
/*    NAME: XeQtor                                              */
/*    ORGN: MIT                                             */
/*    FILE: BHV_ZigLeg.cpp                                    */
/*    DATE:                                                 */
/************************************************************/

#include <iterator>
#include <cstdlib>
#include "MBUtils.h"
#include "BuildUtils.h"
#include "XYRangePulse.h"
#include "ZAIC_PEAK.h"
#include "AngleUtils.h"
#include "BHV_ZigLeg.h"

using namespace std;

//---------------------------------------------------------------
// Constructor

BHV_ZigLeg::BHV_ZigLeg(IvPDomain domain) :
  IvPBehavior(domain)
{
  // Provide a default behavior name
  IvPBehavior::setParam("name", "defaultname");

  // Declare the behavior decision space
  m_domain = subDomain(m_domain, "course,speed");

  // Configuration defaults (overridable in the .bhv file)
  m_pulse_range    = 20;
  m_pulse_duration = 4;
  m_zig_angle      = 45;   // degrees off the heading at zig start
  m_zig_duration   = 10;   // seconds to hold the zig objective

  // State initialization
  m_osx        = 0;
  m_osy        = 0;
  m_osh        = 0;
  m_curr_time  = 0;
  m_prev_index = 0;
  m_index_set  = false;
  m_mark_time  = -1;       // -1 means "no zig currently pending"

  m_zig_active     = false;
  m_zig_start_time = 0;
  m_zig_heading    = 0;

  // Subscribe for ownship position, heading, and the waypoint index
  // published by the sister waypoint behavior.
  addInfoVars("NAV_X, NAV_Y, NAV_HEADING, WPT_INDEX");
}

//---------------------------------------------------------------
// Procedure: setParam()

bool BHV_ZigLeg::setParam(string param, string val)
{
  // Convert the parameter to lower case for more general matching
  param = tolower(param);

  // Get the numerical value of the param argument for convenience once
  double double_val = atof(val.c_str());

  if((param == "zig_angle") && isNumber(val)) {
    m_zig_angle = double_val;
    return(true);
  }
  else if((param == "zig_duration") && isNumber(val)) {
    m_zig_duration = double_val;
    return(true);
  }
  else if((param == "pulse_range") && isNumber(val)) {
    m_pulse_range = double_val;
    return(true);
  }
  else if((param == "pulse_duration") && isNumber(val)) {
    m_pulse_duration = double_val;
    return(true);
  }

  // If not handled above, then just return false;
  return(false);
}

//---------------------------------------------------------------
// Procedure: onSetParamComplete()
//   Purpose: Invoked once after all parameters have been handled.
//            Good place to ensure all required params have are set.
//            Or any inter-param relationships like a<b.

void BHV_ZigLeg::onSetParamComplete()
{
}

//---------------------------------------------------------------
// Procedure: onHelmStart()
//   Purpose: Invoked once upon helm start, even if this behavior
//            is a template and not spawned at startup

void BHV_ZigLeg::onHelmStart()
{
}

//---------------------------------------------------------------
// Procedure: onIdleState()
//   Purpose: Invoked on each helm iteration if conditions not met.

void BHV_ZigLeg::onIdleState()
{
}

//---------------------------------------------------------------
// Procedure: onCompleteState()

void BHV_ZigLeg::onCompleteState()
{
}

//---------------------------------------------------------------
// Procedure: postConfigStatus()
//   Purpose: Invoked each time a param is dynamically changed

void BHV_ZigLeg::postConfigStatus()
{
}

//---------------------------------------------------------------
// Procedure: onIdleToRunState()
//   Purpose: Invoked once upon each transition from idle to run state

void BHV_ZigLeg::onIdleToRunState()
{
}

//---------------------------------------------------------------
// Procedure: onRunToIdleState()
//   Purpose: Invoked once upon each transition from run to idle state

void BHV_ZigLeg::onRunToIdleState()
{
}

//---------------------------------------------------------------
// Procedure: updateInfoIn()
//   Purpose: Refresh ownship position and current time from the buffer.

void BHV_ZigLeg::updateInfoIn()
{
  bool ok1, ok2, ok3;
  m_osx = getBufferDoubleVal("NAV_X", ok1);
  m_osy = getBufferDoubleVal("NAV_Y", ok2);
  m_osh = getBufferDoubleVal("NAV_HEADING", ok3);
  if(!ok1 || !ok2 || !ok3)
    postWMessage("No ownship NAV_X/NAV_Y/NAV_HEADING in info_buffer.");

  m_curr_time = getBufferCurrTime();
}

//---------------------------------------------------------------
// Procedure: postRangeZigLeg()
//   Purpose: Post one expanding range pulse centered at ownship.

void BHV_ZigLeg::postRangeZigLeg()
{
  XYRangePulse pulse;
  pulse.set_x(m_osx);
  pulse.set_y(m_osy);
  pulse.set_label("bhv_pulse");
  pulse.set_rad(m_pulse_range);
  pulse.set_time(m_curr_time);
  pulse.set_color("edge", "yellow");
  pulse.set_color("fill", "yellow");
  pulse.set_duration(m_pulse_duration);

  postMessage("VIEW_RANGE_PULSE", pulse.get_spec());
}

//---------------------------------------------------------------
// Procedure: buildZigFunction()
//   Purpose: A heading objective (ZAIC peak) at the locked zig heading.

IvPFunction* BHV_ZigLeg::buildZigFunction()
{
  ZAIC_PEAK zaic(m_domain, "course");
  zaic.setSummit(m_zig_heading);   // prefer the locked target heading
  zaic.setPeakWidth(0);
  zaic.setBaseWidth(180);
  zaic.setSummitDelta(50);
  zaic.setValueWrap(true);         // course is cyclic (0..359)

  IvPFunction *ipf = zaic.extractIvPFunction();
  if(ipf)
    ipf->setPWT(m_priority_wt);
  return(ipf);
}

//---------------------------------------------------------------
// Procedure: onRunState()
//   Logic: When WPT_INDEX changes, mark the time. 5 seconds later,
//          post a pulse and begin the zig: lock a target heading
//          zig_angle degrees off the current heading, and produce a
//          heading objective for zig_duration seconds.

IvPFunction* BHV_ZigLeg::onRunState()
{
  updateInfoIn();

  // Detect a waypoint change (the trigger), as in the Pulse behavior.
  if(getBufferIsKnown("WPT_INDEX")) {
    bool ok;
    double curr_index = getBufferDoubleVal("WPT_INDEX", ok);
    if(ok) {
      if(!m_index_set) {                     // first reading: baseline
        m_prev_index = curr_index;
        m_index_set  = true;
      }
      else if(curr_index != m_prev_index) {  // waypoint changed
        m_prev_index = curr_index;
        m_mark_time  = m_curr_time;           // schedule the zig 5s out
      }
    }
  }

  // 5 seconds after the waypoint changed: start the zig.
  if((m_mark_time >= 0) && ((m_curr_time - m_mark_time) >= 5)) {
    postRangeZigLeg();                              // visual pulse
    m_zig_active     = true;
    m_zig_start_time = m_curr_time;
    m_zig_heading    = angle360(m_osh + m_zig_angle); // lock target heading
    m_mark_time      = -1;                          // consume the trigger
  }

  // While the zig window is open, produce the heading objective.
  if(m_zig_active) {
    if((m_curr_time - m_zig_start_time) < m_zig_duration)
      return(buildZigFunction());
    m_zig_active = false;                           // window elapsed
  }

  // Otherwise produce nothing; the waypoint behavior steers.
  return(0);
}

