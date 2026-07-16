#!/bin/bash
#------------------------------------------------------------
#  Script: init_field.sh   (skara3)
#  Emits the fixed 4-boat field for the Skaramangas encircle
#  mission. One value per line, one line per boat, same order:
#      abe(E)  ben(N)  cal(W)  deb(S)
#  Ring: center (45,-105), radius 42.43 (circumscribes the base
#  square). Slots are the 4 cardinal points on that ring.
#  Transit speeds are pre-computed so all 4 boats reach their
#  slots at the same time (~34 s), from the home cluster.
#------------------------------------------------------------

# Start positions: clustered near home (18,-173), all inside the op-region.
echo "x=22,y=-165,heading=45"  >  vpositions.txt
echo "x=32,y=-165,heading=15"  >> vpositions.txt
echo "x=42,y=-166,heading=340" >> vpositions.txt
echo "x=30,y=-172,heading=20"  >> vpositions.txt

# Slot (cardinal) points on the common ring (center 45,-95 r 36.07).
echo "81.07,-95"   >  vslotpos.txt   # abe -> E
echo "45,-58.93"   >> vslotpos.txt   # ben -> N
echo "8.93,-95"    >> vslotpos.txt   # cal -> W
echo "45,-131.07"  >> vslotpos.txt   # deb -> S

# NOTE: transit speeds are NO LONGER pre-baked here. pArrivalSync on
# the shoreside computes each boat's run-in speed live from actual
# positions so all four arrive at their slots simultaneously.

printf "abe\nben\ncal\ndeb\n"                  >  vnames.txt
printf "coral\ndodger_blue\ngreen\norange\n"   >  vcolors.txt

exit 0
