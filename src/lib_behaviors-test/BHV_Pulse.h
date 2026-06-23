/************************************************************/
/*    NAME: XeQtor                                              */
/*    ORGN: MIT                                             */
/*    FILE: BHV_Pulse.h                                      */
/*    DATE:                                                 */
/************************************************************/

#ifndef Pulse_HEADER
#define Pulse_HEADER

#include <string>
#include "IvPBehavior.h"

class BHV_Pulse : public IvPBehavior {
public:
  BHV_Pulse(IvPDomain);
  ~BHV_Pulse() {};
  
  bool         setParam(std::string, std::string);
  void         onSetParamComplete();
  void         onCompleteState();
  void         onIdleState();
  void         onHelmStart();
  void         postConfigStatus();
  void         onRunToIdleState();
  void         onIdleToRunState();
  IvPFunction* onRunState();

protected: // Local Utility functions
  void   updateInfoIn();      // pull NAV/WPT/time from the info buffer
  void   postRangePulse();    // post one VIEW_RANGE_PULSE at ownship

protected: // Configuration parameters
  double m_pulse_range;       // radius the pulse expands to  (pulse_range)
  double m_pulse_duration;    // seconds the pulse animates   (pulse_duration)

protected: // State variables
  double m_osx;               // ownship x (NAV_X)
  double m_osy;               // ownship y (NAV_Y)
  double m_curr_time;         // buffer current time
  double m_prev_index;        // last WPT_INDEX seen
  bool   m_index_set;         // have we seen a WPT_INDEX yet
  double m_mark_time;         // time the waypoint last changed (-1 = none pending)
};

#define IVP_EXPORT_FUNCTION

extern "C" {
  IVP_EXPORT_FUNCTION IvPBehavior * createBehavior(std::string name, IvPDomain domain) 
  {return new BHV_Pulse(domain);}
}
#endif
