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
#  Ring: center (55,-50), radius 33, in the central Skaramangas basin.
#------------------------------------------------------------

# Home / start + return positions, one per POOL boat, looked up BY NAME by
# launch_vehicle.sh (RETURN goes here; a boat always returns to its own home,
# independent of how many boats deployed).
# Skaramangas: a tight cluster off the SOUTH beach, ~100 m S of the ring, each
# headed just about due N at the ring center (55,-50) for a clean deploy run-in.
# Headings are the exact bearing home -> ring centre (MOOS: 0=N, CW), so they
# MUST be recomputed whenever the ring centre or a home position moves.
echo "x=47,y=-150,heading=5"   >  vpositions.txt   # asha  (bearing 4.57)
echo "x=57,y=-150,heading=359" >> vpositions.txt   # bama  (bearing 358.85)
echo "x=67,y=-150,heading=353" >> vpositions.txt   # chip  (bearing 353.16)
echo "x=52,y=-158,heading=2"   >> vpositions.txt   # flex  (bearing 1.59)
echo "x=62,y=-158,heading=356" >> vpositions.txt   # ewan  (bearing 356.29)

# Placeholder ring slot only (East point). The REAL slot for each boat is
# computed + pushed live by pArrivalSync on DEPLOY (even 360/N spacing over
# whichever boats connected). Same placeholder for all -- never used to steer,
# just a valid pre-deploy default for the goto_slot waypoint.
for i in asha bama chip flex ewan; do echo "-122.5,-43"; done > vslotpos.txt

printf "asha\nbama\nchip\nflex\newan\n"                   >  vnames.txt
# Sim tints only; real boats use get_robot_info_greece.sh --color.
printf "green\norange\ndodger_blue\ncoral\nyellow\n"      >  vcolors.txt

exit 0
