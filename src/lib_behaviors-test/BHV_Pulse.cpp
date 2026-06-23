/************************************************************/
/*    NAME: XeQtor                                              */
/*    ORGN: MIT                                             */
/*    FILE: BHV_Pulse.cpp                                    */
/*    DATE:                                                 */
/************************************************************/

#include <iterator>
#include <cstdlib>
#include "MBUtils.h"
#include "BuildUtils.h"
#include "XYRangePulse.h"
#include "BHV_Pulse.h"

using namespace std;

//---------------------------------------------------------------
// Constructor

BHV_Pulse::BHV_Pulse(IvPDomain domain) :
  IvPBehavior(domain)
{
  // Provide a default behavior name
  IvPBehavior::setParam("name", "defaultname");

  // Declare the behavior decision space
  m_domain = subDomain(m_domain, "course,speed");

  // Configuration defaults (overridable in the .bhv file)
  m_pulse_range    = 20;
  m_pulse_duration = 4;

  // State initialization
  m_osx        = 0;
  m_osy        = 0;
  m_curr_time  = 0;
  m_prev_index = 0;
  m_index_set  = false;
  m_mark_time  = -1;     // -1 means "no pulse currently pending"

  // Subscribe for ownship position and the waypoint index that the
  // sister waypoint behavior publishes.
  addInfoVars("NAV_X, NAV_Y, WPT_INDEX");
}

//---------------------------------------------------------------
// Procedure: setParam()

bool BHV_Pulse::setParam(string param, string val)
{
  // Convert the parameter to lower case for more general matching
  param = tolower(param);

  // Get the numerical value of the param argument for convenience once
  double double_val = atof(val.c_str());

  if((param == "pulse_range") && isNumber(val)) {
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

void BHV_Pulse::onSetParamComplete()
{
}

//---------------------------------------------------------------
// Procedure: onHelmStart()
//   Purpose: Invoked once upon helm start, even if this behavior
//            is a template and not spawned at startup

void BHV_Pulse::onHelmStart()
{
}

//---------------------------------------------------------------
// Procedure: onIdleState()
//   Purpose: Invoked on each helm iteration if conditions not met.

void BHV_Pulse::onIdleState()
{
}

//---------------------------------------------------------------
// Procedure: onCompleteState()

void BHV_Pulse::onCompleteState()
{
}

//---------------------------------------------------------------
// Procedure: postConfigStatus()
//   Purpose: Invoked each time a param is dynamically changed

void BHV_Pulse::postConfigStatus()
{
}

//---------------------------------------------------------------
// Procedure: onIdleToRunState()
//   Purpose: Invoked once upon each transition from idle to run state

void BHV_Pulse::onIdleToRunState()
{
}

//---------------------------------------------------------------
// Procedure: onRunToIdleState()
//   Purpose: Invoked once upon each transition from run to idle state

void BHV_Pulse::onRunToIdleState()
{
}

//---------------------------------------------------------------
// Procedure: updateInfoIn()
//   Purpose: Refresh ownship position and current time from the buffer.

void BHV_Pulse::updateInfoIn()
{
  bool ok1, ok2;
  m_osx = getBufferDoubleVal("NAV_X", ok1);
  m_osy = getBufferDoubleVal("NAV_Y", ok2);
  if(!ok1 || !ok2)
    postWMessage("No ownship NAV_X/NAV_Y in info_buffer.");

  m_curr_time = getBufferCurrTime();
}

//---------------------------------------------------------------
// Procedure: postRangePulse()
//   Purpose: Post one expanding range pulse centered at ownship.

void BHV_Pulse::postRangePulse()
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
// Procedure: onRunState()
//   Purpose: Invoked each iteration when run conditions have been met.
//   Logic:   When the sister waypoint behavior's WPT_INDEX changes,
//            mark the time; 5 seconds later, post one range pulse.

IvPFunction* BHV_Pulse::onRunState()
{
  updateInfoIn();

  // Read the waypoint index published by the waypoint behavior. Guard
  // with getBufferIsKnown() so we don't warn in the first iterations
  // before the waypoint behavior has posted WPT_INDEX yet.
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
        m_mark_time  = m_curr_time;           // schedule a pulse
      }
    }
  }

  // Fire the scheduled pulse once, 5 seconds after the waypoint changed.
  if((m_mark_time >= 0) && ((m_curr_time - m_mark_time) >= 5)) {
    postRangePulse();
    m_mark_time = -1;                         // clear the pending pulse
  }

  // The Pulse behavior only draws a visual; it produces no IvP function.
  return(0);
}

