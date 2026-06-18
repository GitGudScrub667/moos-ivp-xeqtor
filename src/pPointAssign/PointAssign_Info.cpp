/****************************************************************/
/*   NAME: G Souchlas                                             */
/*   ORGN: MIT, Cambridge MA                                    */
/*   FILE: PointAssign_Info.cpp                               */
/*   DATE: December 29th, 1963                                  */
/****************************************************************/

#include <cstdlib>
#include <iostream>
#include "PointAssign_Info.h"
#include "ColorParse.h"
#include "ReleaseInfo.h"

using namespace std;

//----------------------------------------------------------------
// Procedure: showSynopsis

void showSynopsis()
{
  blk("SYNOPSIS:                                                       ");
  blk("------------------------------------                            ");
  blk("  pPointAssign collects a batch of visit points posted on a     ");
  blk("  single MOOS variable and distributes them among a list of     ");
  blk("  configured vehicles. The incoming batch is bracketed by the   ");
  blk("  string postings \"firstpoint\" and \"lastpoint\"; on receipt of   ");
  blk("  \"lastpoint\" the buffered points are assigned and re-posted,   ");
  blk("  one variable per vehicle (VISIT_POINT_<vname>), each share     ");
  blk("  again wrapped in its own firstpoint/lastpoint markers.        ");
  blk("                                                                ");
  blk("  Points are assigned either by region (east-west halves) or    ");
  blk("  in an alternating round-robin fashion, selectable via the     ");
  blk("  assign_by_region configuration parameter.                     ");
}

//----------------------------------------------------------------
// Procedure: showHelpAndExit

void showHelpAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("Usage: pPointAssign file.moos [OPTIONS]                   ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("Options:                                                        ");
  mag("  --alias","=<ProcessName>                                      ");
  blk("      Launch pPointAssign with the given process name         ");
  blk("      rather than pPointAssign.                           ");
  mag("  --example, -e                                                 ");
  blk("      Display example MOOS configuration block.                 ");
  mag("  --help, -h                                                    ");
  blk("      Display this help message.                                ");
  mag("  --interface, -i                                               ");
  blk("      Display MOOS publications and subscriptions.              ");
  mag("  --version,-v                                                  ");
  blk("      Display the release version of pPointAssign.        ");
  blk("                                                                ");
  blk("Note: If argv[2] does not otherwise match a known option,       ");
  blk("      then it will be interpreted as a run alias. This is       ");
  blk("      to support pAntler launching conventions.                 ");
  blk("                                                                ");
  exit(0);
}

//----------------------------------------------------------------
// Procedure: showExampleConfigAndExit

void showExampleConfigAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("pPointAssign Example MOOS Configuration                   ");
  blu("=============================================================== ");
  blk("                                                                ");
  blk("ProcessConfig = pPointAssign                                    ");
  blk("{                                                               ");
  blk("  AppTick   = 4                                                 ");
  blk("  CommsTick = 4                                                 ");
  blk("                                                                ");
  blk("  input_var        = VISIT_POINT  // var to collect points on   ");
  blk("  vname            = henry        // add a vehicle (repeatable) ");
  blk("  vname            = gilda                                      ");
  blk("  assign_by_region = true         // true=east-west, false=alt  ");
  blk("}                                                               ");
  blk("                                                                ");
  exit(0);
}


//----------------------------------------------------------------
// Procedure: showInterfaceAndExit

void showInterfaceAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("pPointAssign INTERFACE                                    ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("SUBSCRIPTIONS:                                                  ");
  blk("------------------------------------                            ");
  blk("  VISIT_POINT (or whatever input_var is set to)                 ");
  blk("    \"firstpoint\"          -- start of a batch                   ");
  blk("    \"x=<d>,y=<d>,id=<n>\"  -- one visit point                    ");
  blk("    \"lastpoint\"           -- end of batch; triggers assignment  ");
  blk("                                                                ");
  blk("PUBLICATIONS:                                                   ");
  blk("------------------------------------                            ");
  blk("  VISIT_POINT_<vname> -- one per configured vehicle, carrying   ");
  blk("    that vehicle's share, each wrapped in firstpoint/lastpoint. ");
  blk("                                                                ");
  exit(0);
}

//----------------------------------------------------------------
// Procedure: showReleaseInfoAndExit

void showReleaseInfoAndExit()
{
  showReleaseInfo("pPointAssign", "gpl");
  exit(0);
}

