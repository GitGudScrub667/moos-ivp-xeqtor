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
   void commandSpeed(const std::string& v, double spd);  // throttled Notify

 private: // Configuration
   double m_max_speed;          // cruise cap; the farthest boat runs at this
   double m_min_speed;          // floor, so a near boat keeps steerage
   double m_min_arrival_time;   // T is never smaller than this (avoids blow-up)
   double m_capture_dist;       // within this of the slot = "arrived", stop cmd
   double m_stagger_time;       // secs between successive boat releases (0 = off)
   std::string m_deploy_var;    // when true, run-in is active (DEPLOY_ALL)
   std::string m_return_var;    // when true, pause commanding (RETURN_ALL)
   std::string m_update_var;    // base var; posts <update_var>_<VNAME>

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
   std::map<std::string, double> m_cmd_speed;     // last commanded speed
   std::map<std::string, double> m_release_offset; // secs after deploy this boat starts

   double m_curr_T;             // last computed common arrival time (for report)
   unsigned int m_posts;        // count of speed commands sent (for report)
};

#endif
