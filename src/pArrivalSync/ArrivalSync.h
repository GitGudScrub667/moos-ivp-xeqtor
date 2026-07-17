/************************************************************/
/*    NAME: skara2                                          */
/*    ORGN: moos-ivp-xeqtor                                 */
/*    FILE: ArrivalSync.h                                   */
/*    DATE: 2026-07-16                                      */
/*                                                          */
/*  Shoreside coordinator for the skara2 encircle mission.  */
/*  Continuously commands each boat's run-in speed so that   */
/*  all boats reach their assigned ring slots at the SAME    */
/*  time (simultaneous arrival), computed live from actual   */
/*  positions -- robust to real, varying start distances.    */
/************************************************************/

#ifndef ArrivalSync_HEADER
#define ArrivalSync_HEADER

#include <string>
#include <vector>
#include <map>
#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"

// A DISPERSE/ASSEMBLE button press that arrived while a target detour was
// still running gets parked in m_pending_cmd and fired once the boat rejoins.
static const int CMD_NONE     = 0;
static const int CMD_DISPERSE = 1;
static const int CMD_ASSEMBLE = 2;

class ArrivalSync : public AppCastingMOOSApp
{
 public:
   ArrivalSync();
   ~ArrivalSync();

 protected: // Standard MOOSApp functions to overload
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();

 protected: // Standard AppCastingMOOSApp function to overload
   bool buildReport();

 protected:
   void registerVariables();
   void handleNodeReport(const std::string& report);
   bool addVehicle(const std::string& value);   // "abe, 87.43, -105"
   void computeReleaseOffsets();                 // stagger: farthest boat first
   void postThrottled(const std::string& var_base,
                      std::map<std::string,double>& cache,
                      const std::string& v, double spd);
   double slotAngleDeg(const std::string& v) const;   // phase phi_i on the ring
   double actualAngleDeg(const std::string& v) const; // boat's live ring angle
   // Target investigation (left-click detour)
   void handleTargetDetect(const std::string& sval);
   bool pointInRegion(double x, double y) const;
   std::string closestFreeBoat(double tx, double ty) const;
   void assignInvestigation();
   std::string loopSpec(double tx, double ty) const;   // "points=..." around target
   void handleRejoin();          // bring the investigator radially back to the ring
   void respaceFormation(const std::string& inv);  // re-even the ring around 'inv'
   void cancelInvestigation();   // clear any in-progress investigation (return/idle)
   // DISPERSE / ASSEMBLE (square formation)
   void handleDisperseCmd(bool on);   // button press; may be queued behind a detour
   void runPendingCmd();              // fire a queued DISPERSE/ASSEMBLE
   void doDisperse();                 // assign corners + send the boats out
   void doAssemble();                 // drop the square, re-run the ring run-in
   void cancelDisperse();             // clear the square (return/idle)
   std::string squareSpec(double cx, double cy, const std::string& v) const;
   void drawTarget(const std::string& label, double x, double y,
                   const std::string& color);
   void eraseTarget(const std::string& label);

 private: // Configuration (run-in / arrival sync)
   double m_max_speed;          // cruise cap; the farthest boat runs at this
   double m_min_speed;          // floor, so a near boat keeps steerage
   double m_min_arrival_time;   // T is never smaller than this (avoids blow-up)
   double m_capture_dist;       // within this of the slot = "arrived"
   double m_stagger_time;       // secs between successive boat releases (0 = off)
   std::string m_deploy_var;    // when true, run-in is active (DEPLOY_ALL)
   std::string m_return_var;    // when true, pause commanding (RETURN_ALL)
   std::string m_update_var;    // run-in speed base var (SLOT_UPDATE)

 private: // Configuration (orbit phase-lock, opt-in)
   bool   m_enable_orbit_lock;  // false => app behaves exactly as before
   double m_circle_x;           // ring center
   double m_circle_y;
   double m_circle_rad;
   double m_orbit_speed;        // nominal ring speed (= omega*r), controller center
   double m_orbit_gain;         // m/s of speed correction per degree of phase error
   double m_orbit_min;          // clamp on the modulated orbit speed
   double m_orbit_max;
   std::string m_orbit_var;     // orbit speed base var (ENCIRCLE_UPDATE)

 private: // Configuration (target investigation, opt-in)
   bool   m_enable_investigate; // false => no left-click detours
   double m_loop_radius;        // radius of the small circle around a target
   int    m_loop_points;        // number of points in that circle
   double m_investigate_speed;  // detour transit + loop speed
   double m_rejoin_speed;       // rejoin: radial re-entry speed onto the ring
   double m_rejoin_capture;     // rejoin: within this of the ring = re-entered
   std::string m_target_var;    // subscribe: TARGET_DETECT (from the viewer)
   std::string m_inv_flag_var;  // post: INVESTIGATE_<VNAME> = true
   std::string m_inv_update_var;// post: INVESTIGATE_UPDATE_<VNAME> = points=...
   std::string m_inv_done_var;  // subscribe: INVESTIGATE_DONE (from the boat)
   std::vector<double> m_region_x;  // op-region polygon (clicks outside are ignored)
   std::vector<double> m_region_y;

 private: // Configuration (DISPERSE/ASSEMBLE square formation, opt-in)
   bool   m_enable_disperse;    // false => the buttons do nothing
   std::vector<double> m_square_x;   // the fixed square corners
   std::vector<double> m_square_y;
   double m_square_radius;      // radius of the small loiter circle at a corner
   double m_disperse_speed;     // transit-out + loiter speed
   std::string m_disperse_cmd_var;  // subscribe: DISPERSE_CMD (from the buttons)
   std::string m_disp_flag_var;     // post: DISPERSE_<VNAME> = true/false
   std::string m_disp_update_var;   // post: DISPERSE_UPDATE_<VNAME> = polygon=...
   std::string m_slotted_var;       // post: SLOTTED_<VNAME> = false (on assemble)

   std::vector<std::string>       m_vehicles;   // vehicle names, in config order
   std::map<std::string, double>  m_slot_x;     // vname -> slot x
   std::map<std::string, double>  m_slot_y;     // vname -> slot y

 private: // State
   bool m_deployed;
   bool m_returning;
   bool m_staggered;            // release offsets computed for the current deploy
   double m_deploy_time;        // MOOSTime of the current deploy
   std::map<std::string, double> m_nav_x;
   std::map<std::string, double> m_nav_y;
   std::map<std::string, bool>   m_have_nav;
   std::map<std::string, bool>   m_arrived;
   std::map<std::string, double> m_dist;          // last computed distance to slot
   std::map<std::string, double> m_cmd_speed;     // last run-in speed commanded
   std::map<std::string, double> m_release_offset; // secs after deploy this boat starts

   bool   m_orbit_active;       // orbit clock running
   double m_orbit_t0;           // MOOSTime the orbit clock started (first arrival)
   std::map<std::string, double> m_orbit_cmd;     // last orbit speed commanded
   std::map<std::string, double> m_phase_err;     // last phase error (deg), for report
   std::map<std::string, double> m_phase_off;     // per-boat phase offset (deg); re-
                                                  // anchored on rejoin so the returning
                                                  // boat slots in where it comes back

   // Investigation state
   std::vector<double>      m_queue_x;    // pending targets (FIFO)
   std::vector<double>      m_queue_y;
   std::vector<std::string> m_queue_label;
   std::string m_investigator;            // boat currently out ("" = none)
   std::string m_cur_label;               // marker label of the active investigation
   std::map<std::string, bool> m_investigating;
   std::map<std::string, bool> m_rejoining; // investigator transiting back to the ring
   double m_rejoin_px;                    // last posted rejoin entry point (throttle)
   double m_rejoin_py;
   unsigned int m_target_count;           // running count, for unique marker labels

   // Square-formation state
   bool m_dispersed;                      // boats are out on the square
   int  m_pending_cmd;                    // queued button press (see CMD_* below)
   std::map<std::string, int> m_corner_of;  // vname -> square corner index (report)

   double m_curr_T;             // last computed common arrival time (for report)
   unsigned int m_posts;        // count of speed commands sent (for report)
};

#endif
