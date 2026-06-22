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
#include <set>
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

 private: // Config variables
  std::string m_vname;
  
 private: // State variables
  XYSegList  m_path;
  double     m_nav_x;
  double     m_nav_y;
  bool       m_nav_x_set;
  bool       m_nav_y_set;

  // Swimmers we have been alerted to, keyed by swimmer id.
  // Keying by id means repeated SWIMMER_ALERTs for the same id
  // are automatically ignored (no duplicate insertion).
  std::map<std::string, XYPoint> m_swimmers;

  // Ids of swimmers already rescued (by us or the opponent). Because
  // SWIMMER_ALERT repeats every ~15s, erasing a found swimmer is not
  // enough -- the next repeat would look "new" and get re-added. This
  // set is permanent memory so a rescued id is ignored forever.
  std::set<std::string> m_rescued;

  // Set true when a new (previously unknown) swimmer arrives, so
  // Iterate() knows to regenerate the path and re-post the update.
  bool m_plan_pending;
};

#endif 
