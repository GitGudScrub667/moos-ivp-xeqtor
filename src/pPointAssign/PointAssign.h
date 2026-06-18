/************************************************************/
/*    NAME: G Souchlas                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: PointAssign.h                                          */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#ifndef PointAssign_HEADER
#define PointAssign_HEADER

#include <string>
#include <vector>
#include <set>
#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"

class PointAssign : public AppCastingMOOSApp
{
 public:
   PointAssign();
   ~PointAssign();

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
   void assignAndPublish();
   bool allVehiclesReady() const;
   void postViewPoint(double x, double y, std::string label, std::string color);

 private: // Configuration variables
   std::string              m_input_var;        // var to subscribe to (default VISIT_POINT)
   std::vector<std::string> m_vehicles;         // vehicles to distribute to (from "vname")
   bool                     m_assign_by_region; // true = east-west region, false = alternating
   std::string              m_unpause_var;      // var posted =false to un-pause the timer script
   double                   m_ready_timeout;    // secs to wait for vehicles before un-pausing anyway

 private: // State variables
   bool                     m_collecting;  // true between firstpoint and lastpoint
   bool                     m_unpaused_sent; // have we sent the timer-script un-pause yet?
   std::set<std::string>    m_ready_vehicles; // vehicles that have announced PGENPATH_READY
   double                   m_first_iter_time; // MOOSTime of our first Iterate (for timeout)
   std::vector<std::string> m_pt_str;      // raw point strings, in arrival order
   std::vector<double>      m_pt_x;        // parsed x for each point (for the split)

   // bookkeeping for the appcast report
   unsigned int              m_batches_done;
   std::vector<unsigned int> m_last_counts; // points sent to each vehicle, last batch
};

#endif
