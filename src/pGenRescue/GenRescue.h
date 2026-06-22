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
  bool handleMailNodeReport(std::string);
  void postShortestPath();
  void postNullPath();

  // Cluster-aware ordering: like a greedy nearest-neighbor tour, but
  // each candidate's distance is discounted by how many other swimmers
  // sit nearby, so the boat prefers to dive into dense packs first.
  // sh = boat heading, used for the time-to-target cost so a swimmer
  // behind the bow is penalized for the turn it would take to reach.
  // factors[j] is an opponent-contest multiplier on vertex j's score
  // (1.0 = neutral; <1 boosts a contested swimmer's priority).
  // mults[j] is how many swimmers visit-point j represents (>=1), so a
  // coverage-merged pack still contributes its full density.
  XYSegList clusterPath(XYSegList swim_pts, double sx, double sy, double sh,
                        std::vector<double> factors, std::vector<double> mults);

  // Time (seconds) for a craft at (px,py) heading ph, moving at the
  // given speed, to reach (tx,ty): straight-line travel + turn time.
  double etaToPoint(double px, double py, double ph, double speed,
                    double tx, double ty);
  // Our ETA from ownship to a swimmer.
  double ownshipETA(double tx, double ty);
  // Best (smallest) ETA of any fresh opponent contact to a swimmer.
  // have_opp is set false if no fresh contact exists.
  double opponentETA(double tx, double ty, bool &have_opp);
  // Contest verdict for a swimmer: fills factor + margin, returns a
  // short label ("WIN"/"CONTESTED"/"LOST"/"-").
  std::string contestVerdict(double tx, double ty, double &factor, double &margin);
  // True if at least one opponent contact is fresh (not stale).
  bool haveFreshOpponent();

  // Pull a point to sit at least 'margin' meters inside the rescue
  // region, so a turn near the edge can't carry the boat out of bounds
  // (a disqualification). No-op if the region is unknown or the point
  // is already safely interior.
  XYPoint insetIntoRegion(double x, double y, double margin);

  // Re-inset an ordered path's waypoints away from the boundary, more
  // aggressively where the tour turns hard (a sharp turn overshoots
  // more). Returns the tightened path.
  XYSegList tightenForTurns(XYSegList path);

  // 2-opt local search: uncross the ordered tour to minimize traversal
  // time (travel + turning), pinning waypoint 0 so the contest-first
  // pick is preserved. Returns the improved path.
  XYSegList twoOptImprove(XYSegList path);
  // Or-opt local search: relocate a run of 1-3 consecutive waypoints to
  // a better position (no reversal), again minimizing traversal time
  // and pinning waypoint 0. Complements 2-opt.
  XYSegList orOptImprove(XYSegList path);
  // Total time to traverse a waypoint list from ownship's pose, using
  // the same travel+turn model as the ordering cost.
  double pathTime(const std::vector<double> &vx, const std::vector<double> &vy);

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

  // Coverage-reduction tuning (idea B). Before ordering, swimmers are
  // collapsed to a minimal set of visit points such that every swimmer
  // is within m_cover_range of one. Detection reaches 25m, so a visit
  // point covers its near neighbors without a separate waypoint.
  //   set to 0 to disable (every swimmer becomes its own visit point)
  double m_cover_range;

  // Keep planned waypoints at least this many meters inside the rescue
  // region boundary, so turn overshoot can't take the boat out of
  // bounds (= disqualification). 0 disables the inset.
  //   m_boundary_margin : base inset applied to every waypoint
  //   m_overshoot_max   : extra inset added for a full 180-deg turn,
  //                       scaled down for gentler turns
  double m_boundary_margin;
  double m_overshoot_max;

  // Tour cleanup local search. When true, the ordered tour is improved
  // to cut traversal time; false leaves clusterPath's order.
  //   m_use_two_opt : uncross segments (idea C)
  //   m_use_or_opt  : relocate short runs of waypoints
  bool m_use_two_opt;
  bool m_use_or_opt;

  // Opponent-aware contest tuning (idea A). All dormant when no fresh
  // opponent contact exists (then behavior == idea #1/#5 exactly).
  //   m_lose_margin     : if an opponent beats our ETA by more than this
  //                       (s), the swimmer is a lost cause -> skip it
  //   m_contest_window  : within this ETA margin (s) the swimmer is
  //                       "contested" -> boost its priority to deny it
  //   m_contest_boost   : score multiplier for a contested swimmer (<1)
  //   m_contact_max_age : ignore opponent contacts older than this (s)
  //   m_replan_interval : replan at least this often (s) while swimmers
  //                       remain -- re-sweeps swimmers missed on a pass
  //                       (capture is probabilistic) and tracks opponent
  double m_lose_margin;
  double m_contest_window;
  double m_contest_boost;
  double m_contact_max_age;
  double m_replan_interval;
  
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

  // A tracked opponent (or any other vehicle) from NODE_REPORT.
  struct Contact {
    double x;
    double y;
    double heading;
    double speed;
    double utc;       // local receipt time, for staleness checks
  };
  // Other vehicles we have seen, keyed by name (ownship excluded).
  std::map<std::string, Contact> m_contacts;

  // Last time we regenerated the path, for the opponent-driven
  // periodic replan in Iterate().
  double m_last_replan_utc;

  // The rescue-region boundary (from RESCUE_REGION). Waypoints are kept
  // inside it by m_boundary_margin to avoid out-of-bounds breaches.
  XYPolygon m_region;
  bool      m_region_set;
};

#endif 
