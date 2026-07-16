/****************************************************************/
/*   NAME: skara2                                               */
/*   ORGN: moos-ivp-xeqtor                                      */
/*   FILE: ArrivalSync_Info.cpp                                 */
/*   DATE: 2026-07-16                                           */
/****************************************************************/

#include <cstdlib>
#include <iostream>
#include "ArrivalSync_Info.h"
#include "ColorParse.h"
#include "ReleaseInfo.h"

using namespace std;

//----------------------------------------------------------------
// Procedure: showSynopsis

void showSynopsis()
{
  blk("SYNOPSIS:                                                       ");
  blk("------------------------------------                            ");
  blk("  pArrivalSync is a shoreside coordinator for the skara2        ");
  blk("  encircle mission. It watches every boat's NODE_REPORT and,    ");
  blk("  during the run-in, commands each boat's speed so that all     ");
  blk("  boats reach their assigned ring slots at the SAME time. The   ");
  blk("  farthest remaining boat paces the group at max_speed; the     ");
  blk("  others are scaled down to match its ETA. Recomputed each      ");
  blk("  tick, so it self-corrects for real, varying distances and     ");
  blk("  for perturbation from collision avoidance.                    ");
}

//----------------------------------------------------------------
// Procedure: showHelpAndExit

void showHelpAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("Usage: pArrivalSync file.moos [OPTIONS]                         ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("Options:                                                        ");
  mag("  --alias","=<ProcessName>                                      ");
  blk("      Launch pArrivalSync with the given process name.          ");
  mag("  --example, -e                                                 ");
  blk("      Display example MOOS configuration block.                 ");
  mag("  --help, -h                                                    ");
  blk("      Display this help message.                                ");
  mag("  --interface, -i                                               ");
  blk("      Display MOOS publications and subscriptions.              ");
  mag("  --version,-v                                                  ");
  blk("      Display release version of pArrivalSync.                  ");
  exit(0);
}

//----------------------------------------------------------------
// Procedure: showExampleConfigAndExit

void showExampleConfigAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("pArrivalSync Example MOOS Configuration                         ");
  blu("=============================================================== ");
  blk("                                                                ");
  blk("ProcessConfig = pArrivalSync                                    ");
  blk("{                                                               ");
  blk("  AppTick   = 4                                                 ");
  blk("  CommsTick = 4                                                 ");
  blk("                                                                ");
  blk("  max_speed        = 3.0    // farthest boat runs at this       ");
  blk("  min_speed        = 0.4    // floor, keeps steerage            ");
  blk("  min_arrival_time = 8      // T never smaller than this (s)    ");
  blk("  capture_dist     = 6      // within this of slot = arrived    ");
  blk("  stagger_time     = 5      // secs between boat releases        ");
  blk("                                                                ");
  blk("  deploy_var = DEPLOY_ALL   // run-in active while true         ");
  blk("  return_var = RETURN_ALL   // pause commanding while true      ");
  blk("  update_var = SLOT_UPDATE  // posts SLOT_UPDATE_<VNAME>=speed=X");
  blk("                                                                ");
  blk("  vehicle = abe, 87.43, -105    // name, slot_x, slot_y         ");
  blk("  vehicle = ben, 45, -62.57                                     ");
  blk("  vehicle = cal, 2.57, -105                                     ");
  blk("  vehicle = deb, 45, -147.43                                    ");
  blk("}                                                               ");
  exit(0);
}

//----------------------------------------------------------------
// Procedure: showInterfaceAndExit

void showInterfaceAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("pArrivalSync INTERFACE                                          ");
  blu("=============================================================== ");
  blk("                                                                ");
  blk("SUBSCRIPTIONS:                                                  ");
  blk("------------------------------------                            ");
  blk("  NODE_REPORT     (every boat's live position)                  ");
  blk("  DEPLOY_ALL      (run-in active while true)                    ");
  blk("  RETURN_ALL      (pause commanding while true)                 ");
  blk("                                                                ");
  blk("PUBLICATIONS:                                                   ");
  blk("------------------------------------                            ");
  blk("  SLOT_UPDATE_<VNAME> = speed=X   (per-boat run-in speed)       ");
  exit(0);
}

//----------------------------------------------------------------
// Procedure: showReleaseInfoAndExit

void showReleaseInfoAndExit()
{
  showReleaseInfo("pArrivalSync", "gpl");
  exit(0);
}
