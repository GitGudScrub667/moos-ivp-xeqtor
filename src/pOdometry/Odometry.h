/************************************************************/
/*    NAME: XeQtor                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: Odometry.h                                          */
/*    DATE: June 15th, 2026                             */
/************************************************************/

#ifndef Odometry_HEADER
#define Odometry_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"

class Odometry : public AppCastingMOOSApp
{
 public:
   Odometry();
   ~Odometry();

 protected: // Standard MOOSApp functions to overload  
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();

 protected: // Standard AppCastingMOOSApp function to overload 
   bool buildReport();

 protected:
   void registerVariables();

 private: // Configuration variables

 private: // State variables
   bool   m_first_reading;   // true until the first NAV pair is captured
   bool   m_nav_received;    // true once any NAV_X/NAV_Y has arrived

   double m_current_x;       // most recent NAV_X
   double m_current_y;       // most recent NAV_Y
   double m_previous_x;      // NAV_X at the previous Iterate
   double m_previous_y;      // NAV_Y at the previous Iterate

   double m_total_distance;  // accumulated path length, published as ODOMETRY_DIST
};

#endif 
