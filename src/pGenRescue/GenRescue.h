/************************************************************/
/*    NAME: Mike Benjamin                                   */
/*    ORGN: MIT                                             */
/*    FILE: GenRescue.h                                     */
/*    DATE: April 18th, 2022                                */
/************************************************************/

#ifndef P_GEN_RESCUE_HEADER
#define P_GEN_RESCUE_HEADER

#include <vector>
#include <string>
#include <map>
#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include "XYPoint.h"
#include "XYPolygon.h"

class GenRescue : public AppCastingMOOSApp
{
 public:
   GenRescue();
   ~GenRescue() {};

 protected:
  bool OnNewMail(MOOSMSG_LIST &NewMail);
  bool Iterate();
  bool OnConnectToServer();
  bool OnStartUp();
  bool buildReport();
  void RegisterVariables();
  
 protected:
  bool handleMailNewSwimmer(std::string);
  bool handleMailFoundSwimmer(std::string);
  bool handleMailRescueRegion(std::string);
  void postShortestPath();
  void postNullPath();

  // Cluster-aware ordering: like a greedy nearest-neighbor tour, but
  // each candidate's distance is discounted by how many other swimmers
  // sit nearby, so the boat prefers to dive into dense packs first.
  // sh = boat heading, used for the time-to-target cost so a swimmer
  // behind the bow is penalized for the turn it would take to reach.
  XYSegList clusterPath(XYSegList swim_pts, double sx, double sy, double sh);

 private: // Config variables
  std::string m_vname;

  // Cluster-aware path tuning (hardcoded; mission .moos is staff-owned
  // so we keep these in the app and tune by rebuild).
  //   m_cluster_radius : how close (m) counts as a swimmer's "neighbor"
  //   m_cluster_weight : how strongly density discounts distance (0 = plain greedy)
  double m_cluster_radius;
  double m_cluster_weight;

  // Time-to-target tuning (idea #5). Cost to reach a swimmer is
  // travel time plus turning time:
  //   m_speed     : assumed transit speed (m/s) -> travel_time = dist/speed
  //   m_turn_rate : assumed turn rate (deg/s)   -> turn_time   = angle/turn_rate
  //                 set <= 0 to ignore heading entirely (recovers idea #1)
  double m_speed;
  double m_turn_rate;
  
 private: // State variables
  XYSegList  m_path;
  double     m_nav_x;
  double     m_nav_y;
  double     m_nav_heading;
  bool       m_nav_x_set;
  bool       m_nav_y_set;
  bool       m_nav_heading_set;

  // Swimmers we have been alerted to, keyed by swimmer id.
  // Keying by id means repeated SWIMMER_ALERTs for the same id
  // are automatically ignored (no duplicate insertion).
  std::map<std::string, XYPoint> m_swimmers;

  // Swimmers already rescued, keyed by id -> finder (vehicle name that
  // made the rescue, us or an opponent). Because SWIMMER_ALERT repeats
  // every ~15s, erasing a found swimmer is not enough -- the next
  // repeat would look "new" and get re-added. This map is permanent
  // memory so a rescued id is ignored forever, and the finder lets us
  // tally rescues-by-us vs rescues-by-opponent in the report.
  std::map<std::string, std::string> m_rescued;

  // Set true when a new (previously unknown) swimmer arrives, so
  // Iterate() knows to regenerate the path and re-post the update.
  bool m_plan_pending;
};

#endif 
