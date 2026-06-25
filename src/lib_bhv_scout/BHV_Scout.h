/*****************************************************************/
/*    NAME: M.Benjamin,                                          */
/*    ORGN: Dept of Mechanical Eng / CSAIL, MIT Cambridge MA     */
/*    FILE: BHV_Scout.h                                          */
/*    DATE: April 30th 2022                                      */
/*                                                               */
/* This program is free software; you can redistribute it and/or */
/* modify it under the terms of the GNU General Public License   */
/* as published by the Free Software Foundation; either version  */
/* 2 of the License, or (at your option) any later version.      */
/*                                                               */
/* This program is distributed in the hope that it will be       */
/* useful, but WITHOUT ANY WARRANTY; without even the implied    */
/* warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR       */
/* PURPOSE. See the GNU General Public License for more details. */
/*                                                               */
/* You should have received a copy of the GNU General Public     */
/* License along with this program; if not, write to the Free    */
/* Software Foundation, Inc., 59 Temple Place - Suite 330,       */
/* Boston, MA 02111-1307, USA.                                   */
/*****************************************************************/
 
#ifndef BHV_SCOUT_HEADER
#define BHV_SCOUT_HEADER

#include <string>
#include <vector>
#include <map>
#include "IvPBehavior.h"
#include "XYPoint.h"
#include "XYPolygon.h"

class BHV_Scout : public IvPBehavior {
public:
  BHV_Scout(IvPDomain);
  ~BHV_Scout() {};
  
  bool         setParam(std::string, std::string);
  void         onIdleState();
  IvPFunction* onRunState();
  void         onEveryState(std::string);
  
protected:
  IvPFunction* buildFunction();
  void         updateScoutPoint();
  void         postViewPoint(bool viewable=true);

  // Coverage-grid helpers (Step 1)
  void         buildGrid();           // lay grid over the region (once)
  void         markSwept(double px, double py);  // mark cells near (px,py)
  void         markVehiclesSwept();   // mark cells near ALL known vehicles

  // Boustrophedon sweep (Step 2): order the cells into a serpentine
  // lawnmower path so the scout sweeps in clean rows instead of darting
  // to whatever cell is nearest. Covered cells are skipped, so areas any
  // vehicle already visited are not re-searched.
  void         buildSweepOrder();              // build m_order (once, after grid)
  bool         pickNextInOrder(double& rx, double& ry); // next unswept in order

  // COLREGS push (Step 3): when an opponent hugs the region edge near us,
  // position ben on its interior-forward quarter so the opponent's own
  // collision-avoidance gives way toward/over the boundary (their DQ).
  void         updateContacts();                       // refresh m_contacts + own color
  bool         findPushTarget(double& tx, double& ty); // true if pushing an opponent
  std::string  colorOfReport(std::string);             // COLOR field of a node report

protected: // State variables
  double   m_osx;
  double   m_osy;
  double   m_curr_time;

  double   m_ptx;
  double   m_pty;
  bool     m_pt_set;

  XYPolygon m_rescue_region;

  // Coverage grid: parallel arrays of cell centers + a "swept" flag.
  // A cell is swept once ANY vehicle (ours, teammate, opponents) has
  // been within m_sweep_radius of it. Built once the region is known.
  bool                 m_grid_ready;
  std::vector<double>  m_cell_x;
  std::vector<double>  m_cell_y;
  std::vector<bool>    m_cell_covered;

  // Serpentine (boustrophedon) visit order over the grid cells, plus a
  // cursor into it. Built once with the grid; the scout walks the cursor
  // forward to the next still-unswept cell as it sweeps.
  std::vector<unsigned int> m_order;
  unsigned int              m_order_idx;

  // Other vehicles seen via NODE_REPORT (for the COLREGS push), keyed by
  // name and refreshed each tick from the latest report.
  struct Contact { double x; double y; double heading; double utc; std::string color; };
  std::map<std::string, Contact> m_contacts;

  // Our own team color, learned from NODE_REPORT_LOCAL. Different-color
  // craft are opponents we may push; same-color (our rescue teammate) we
  // never push. Empty until learned.
  std::string m_my_color;

  // True while actively pushing an opponent, so when the push ends we
  // re-pick a fresh sweep cell instead of driving to a stale push point.
  bool m_pushing;

protected: // Config variables
  double m_capture_radius;
  double m_desired_speed;
  double m_sweep_radius;   // detection/coverage radius (~scout reliable range)

  // COLREGS push tuning.
  bool   m_push_enabled;       // master on/off
  double m_push_edge_dist;     // opponent within this of the boundary -> push-eligible
  double m_push_engage_range;  // ... and within this of ben -> engage (else keep searching)
  double m_push_standoff;      // how far interior of the opponent ben aims

  std::string m_tmate;
};

#define IVP_EXPORT_FUNCTION
extern "C" {
  IVP_EXPORT_FUNCTION IvPBehavior * createBehavior(std::string name, IvPDomain domain) 
  {return new BHV_Scout(domain);}
}
#endif
