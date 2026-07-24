#!/bin/bash
#------------------------------------------------------------
#  Script: init_field.sh   (pireas_2boat)
#  Emits the fixed 2-boat field for the Pireas encircle mission.
#  One value per line, one line per boat, same order:
#      asha(E)  bama(W)
#  Ring: center (-144,-43), radius 21.5 (scaled x1.15), sited in the
#  widened trapezoid op-region. The two slots are the E and W points on
#  that ring -- diametrically opposite, so the orbit phase-lock self-spaces
#  the boats 180 deg apart (slotAngleDeg derives phase from the slot coords).
#  pArrivalSync computes run-in speeds live so both boats reach their slots
#  simultaneously (farthest -- bama, on the far W side -- released first).
#------------------------------------------------------------

# Start positions: clustered E of the formation around (-68,-41), clear of the
# permanent MIO station + its no-go at (-90.43,-52.25). Verified clear (4-boat
# cluster): op-edge >=11.5 m, MIO station >=18.9 m, buoys >=15 m; headed W
# toward the ring. Two of the four cluster points are reused here.
echo "x=-63,y=-37,heading=266" >  vpositions.txt   # asha
echo "x=-73,y=-45,heading=272" >> vpositions.txt   # bama

# Slot points on the common ring (center -144,-43 r 21.5). E and W only.
echo "-122.5,-43"   >  vslotpos.txt   # asha -> E  (phase   0 deg)
echo "-165.5,-43"   >> vslotpos.txt   # bama -> W  (phase 180 deg)

# NOTE: transit speeds are NO LONGER pre-baked here. pArrivalSync on the
# shoreside computes each boat's run-in speed live from actual positions so
# both arrive at their slots simultaneously.

printf "asha\nbama\n"                 >  vnames.txt
printf "green\norange\n"         >  vcolors.txt   # sim tint only; real boats use get_robot_info_greece.sh --color

exit 0
