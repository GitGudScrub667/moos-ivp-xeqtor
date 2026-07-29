#!/bin/bash
#------------------------------------------------------------
#  Script: init_field.sh   (skara_N_boats)
#  Emits the FIXED 5-boat POOL for the Skaramangas encircle mission.
#  Any 1-4 of these may actually be on the water; pArrivalSync on the
#  shoreside "locks in" whichever are connected when DEPLOY is pressed and
#  lays N evenly-spaced slots on the ring for exactly that set (dynamic
#  roster). So NO per-boat ring slot is baked here anymore -- vslotpos.txt
#  now holds only a harmless pre-deploy PLACEHOLDER (the East ring point),
#  overridden live by SLOT_UPDATE the moment the boats deploy.
#
#  One value per line, one line per boat, same order:
#      asha  bama  chip  flex  ewan
#  Ring: center (-144,-43), radius 21.5 (scaled x1.15), sited in the
#  widened trapezoid op-region.
#------------------------------------------------------------

# Home / start + return positions, one per POOL boat, looked up BY NAME by
# launch_vehicle.sh (RETURN goes here; a boat always returns to its own home,
# independent of how many boats deployed).
# TODO(skara geometry): the 5 rows below are the PIREAS home cluster + headings
# (a tight cluster E of the ring, all headed W toward the pireas ring center).
# Re-place the cluster near the skara launch/recovery point and re-aim each
# heading toward the NEW ring center. Keep them clear of the MIO station + no-go.
echo "x=-63,y=-37,heading=266" >  vpositions.txt   # asha
echo "x=-73,y=-37,heading=265" >> vpositions.txt   # bama
echo "x=-73,y=-45,heading=272" >> vpositions.txt   # chip
echo "x=-63,y=-45,heading=271" >> vpositions.txt   # flex
echo "x=-68,y=-41,heading=268" >> vpositions.txt   # ewan

# Placeholder ring slot only (East point). The REAL slot for each boat is
# computed + pushed live by pArrivalSync on DEPLOY (even 360/N spacing over
# whichever boats connected). Same placeholder for all -- never used to steer,
# just a valid pre-deploy default for the goto_slot waypoint.
for i in asha bama chip flex ewan; do echo "-122.5,-43"; done > vslotpos.txt

printf "asha\nbama\nchip\nflex\newan\n"                   >  vnames.txt
# Sim tints only; real boats use get_robot_info_greece.sh --color.
printf "green\norange\ndodger_blue\ncoral\nyellow\n"      >  vcolors.txt

exit 0
