/************************************************************/
/*    NAME: XeQtor                                              */
/*    ORGN: MIT                                             */
/*    FILE: BHV_ZigLeg.h                                      */
/*    DATE:                                                 */
/************************************************************/

#ifndef ZigLeg_HEADER
#define ZigLeg_HEADER

#include <string>
#include "IvPBehavior.h"

class BHV_ZigLeg : public IvPBehavior {
public:
  BHV_ZigLeg(IvPDomain);
  ~BHV_ZigLeg() {};
  
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
  void         updateInfoIn();      // pull NAV/WPT/heading/time from buffer
  void         postRangeZigLeg();   // post a VIEW_RANGE_PULSE at ownship
  IvPFunction* buildZigFunction();  // heading objective during the zig

protected: // Configuration parameters
  double m_pulse_range;       // radius the pulse expands to  (pulse_range)
  double m_pulse_duration;    // seconds the pulse animates   (pulse_duration)
  double m_zig_angle;         // heading offset, degrees      (zig_angle)
  double m_zig_duration;      // seconds the zig is held      (zig_duration)

protected: // State variables
  double m_osx;               // ownship x (NAV_X)
  double m_osy;               // ownship y (NAV_Y)
  double m_osh;               // ownship heading (NAV_HEADING)
  double m_curr_time;         // buffer current time
  double m_prev_index;        // last WPT_INDEX seen
  bool   m_index_set;         // have we seen a WPT_INDEX yet
  double m_mark_time;         // time waypoint last changed (-1 = none pending)

  bool   m_zig_active;        // currently producing the zig objective?
  double m_zig_start_time;    // when the zig began
  double m_zig_heading;       // target heading (locked at zig start)
};

#define IVP_EXPORT_FUNCTION

extern "C" {
  IVP_EXPORT_FUNCTION IvPBehavior * createBehavior(std::string name, IvPDomain domain) 
  {return new BHV_ZigLeg(domain);}
}
#endif
