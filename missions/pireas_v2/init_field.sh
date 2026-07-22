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

# Start positions: clustered ESE of the ring around (-86,-56), well
# clear of the DISPERSE square footprint (>=18 m), the no-go buoys
# (>=18 m) and the op-region edge (>=18 m); headed W toward the ring.
echo "x=-81,y=-52,heading=278" >  vpositions.txt
echo "x=-90,y=-50,heading=277" >> vpositions.txt
echo "x=-92,y=-60,heading=288" >> vpositions.txt
echo "x=-82,y=-62,heading=287" >> vpositions.txt

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
