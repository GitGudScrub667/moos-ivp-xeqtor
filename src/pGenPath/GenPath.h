/************************************************************/
/*    NAME: XeQtor                                          */
/*    ORGN: MIT 2.680                                       */
/*    FILE: GenPath.h                                       */
/*    DATE: 2026                                            */
/************************************************************/

#ifndef GenPath_HEADER
#define GenPath_HEADER

#include <string>
#include <vector>
#include "XYPoint.h"
#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"

class GenPath : public AppCastingMOOSApp
{
 public:
   GenPath();
   ~GenPath();

 protected: // Standard MOOSApp functions to overload
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();

 protected: // Standard AppCastingMOOSApp function to overload
   bool buildReport();

 protected:
   void registerVariables();
   void handleVisitPoint(const std::string& sval);
   void generatePath();
   void updateVisited();

 private: // Configuration variables
   std::string m_wpt_update_var; // var the waypoint behavior listens on (updates=)
   double      m_visit_radius;   // a point counts as visited within this range

 private: // State variables
   bool                 m_collecting;       // true between firstpoint and lastpoint
   std::vector<XYPoint> m_points;           // the visit points of the current set
   std::vector<XYPoint> m_tour;             // points in greedy visiting order
   bool                 m_path_generated;   // have we built+posted a tour yet?
   std::vector<bool>    m_visited;          // parallel to m_tour: reached yet?
   unsigned int         m_visited_count;    // how many of m_tour are visited

   bool                 m_first_received;   // saw "firstpoint" for the current set
   bool                 m_last_received;    // saw "lastpoint" -> set is complete
   unsigned int         m_total_received;   // valid points stored in the current set
   unsigned int         m_invalid_received; // malformed point strings seen

   double               m_nav_x;            // latest vehicle position
   double               m_nav_y;
   bool                 m_nav_received;     // have we heard NAV_X/NAV_Y yet?
};

#endif
