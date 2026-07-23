#!/bin/bash
#------------------------------------------------------------
#  Script: init_field.sh   (pireas_v2)
#  Emits the fixed 4-boat field for the Pireas encircle
#  mission. One value per line, one line per boat, same order:
#      abe(E)  ben(N)  cal(W)  deb(S)
#  Ring: center (-144,-43), radius 21.5 (scaled x1.15), sited in the
#  widened trapezoid op-region. Slots are the 4 cardinal points on that ring.
#  pArrivalSync computes run-in speeds live so all 4 boats reach
#  their slots simultaneously.
#------------------------------------------------------------

# Start positions: clustered E of the formation around (-68,-41), moved here
# (was ~(-86,-56)) to clear the permanent MIO station + its no-go at
# (-90.43,-52.25). Verified clear: op-edge >=11.5 m, MIO station >=18.9 m,
# buoys >=15 m, square footprint >=38 m; headed W toward the ring.
echo "x=-63,y=-37,heading=266" >  vpositions.txt
echo "x=-73,y=-37,heading=265" >> vpositions.txt
echo "x=-73,y=-45,heading=272" >> vpositions.txt
echo "x=-63,y=-45,heading=271" >> vpositions.txt

# Slot (cardinal) points on the common ring (center -144,-43 r 21.5).
echo "-122.5,-43"   >  vslotpos.txt   # abe -> E
echo "-144,-21.5"   >> vslotpos.txt   # ben -> N
echo "-165.5,-43"   >> vslotpos.txt   # cal -> W
echo "-144,-64.5"   >> vslotpos.txt   # deb -> S

# NOTE: transit speeds are NO LONGER pre-baked here. pArrivalSync on
# the shoreside computes each boat's run-in speed live from actual
# positions so all four arrive at their slots simultaneously.

printf "abe\nben\ncal\ndeb\n"                  >  vnames.txt
printf "coral\ndodger_blue\ngreen\norange\n"   >  vcolors.txt

exit 0
