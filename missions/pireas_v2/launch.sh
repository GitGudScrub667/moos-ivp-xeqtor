#!/bin/bash
#------------------------------------------------------------
#  Script: launch.sh   (pireas_v2 - 4-boat encircle, Pireas)
#  Launches a shoreside + 4 sim boats. On DEPLOY each boat
#  transits to its N/E/S/W slot (arriving together) then orbits
#  the common ring, evenly spaced.  Usage: ./launch.sh [warp]
#------------------------------------------------------------
vecho() { if [ "$VERBOSE" != "" ]; then echo "$ME: $1"; fi }
on_exit() { echo; echo "$ME: Halting all apps"; kill -- -$$; }
trap on_exit SIGINT

ME=$(basename "$0")
TIME_WARP=1
VERBOSE=""
JUST_MAKE=""
XLAUNCHED="no"
SHORE_ONLY="no"
IP_ADDR=""

#------------------------------------------------------------
for ARGI; do
    if [ "${ARGI}" = "--help" -o "${ARGI}" = "-h" ]; then
        echo "$ME [OPTIONS] [time_warp]"
        echo "  --help, -h        Show this help"
        echo "  --just_make, -j   Only create targ files (no launch)"
        echo "  --verbose, -v     Verbose"
        echo "  --xlaunched, -x   Launch but skip the trailing uMAC"
        echo "  --shoreside, -sh  Launch ONLY the shoreside (field use: the"
        echo "                    real boats launch themselves on their own"
        echo "                    Pablos via ./launch_vehicle.sh)"
        echo "  --ip=<addr>       Shoreside IP the boats will connect to"
        echo "                    (required with --shoreside in the field)"
        exit 0
    elif [ "${ARGI//[^0-9]/}" = "$ARGI" -a "$TIME_WARP" = 1 ]; then
        TIME_WARP=$ARGI
    elif [ "${ARGI}" = "--verbose" -o "${ARGI}" = "-v" ]; then
        VERBOSE="-v"
    elif [ "${ARGI}" = "--just_make" -o "${ARGI}" = "-j" ]; then
        JUST_MAKE="-j"
    elif [ "${ARGI}" = "--xlaunched" -o "${ARGI}" = "-x" ]; then
        XLAUNCHED="yes"
    elif [ "${ARGI}" = "--shoreside" -o "${ARGI}" = "-sh" ]; then
        SHORE_ONLY="yes"
    elif [ "${ARGI:0:5}" = "--ip=" ]; then
        IP_ADDR="${ARGI#--ip=*}"
    else
        echo "$ME: Bad arg: $ARGI. Exit Code 1."
        exit 1
    fi
done

#------------------------------------------------------------
#  Part 1: (Re)generate the fixed 4-boat field.
#------------------------------------------------------------
./init_field.sh

VNAMES=($(cat vnames.txt))
VCOLOR=($(cat vcolors.txt))
VEHPOS=($(cat vpositions.txt))
VSLOT=($(cat vslotpos.txt))
VAMT=${#VNAMES[@]}

CSV_VNAMES=$(IFS=:; echo "${VNAMES[*]}")

#------------------------------------------------------------
#  Part 2: Launch the shoreside.
#------------------------------------------------------------
vecho "Launching shoreside (vnames=$CSV_VNAMES)"
IPARG=""
if [ "${IP_ADDR}" != "" ]; then IPARG="--ip=$IP_ADDR"; fi
./launch_shoreside.sh --auto --mport=9000 --pshare=9200 $IPARG \
    --vnames=$CSV_VNAMES $JUST_MAKE $VERBOSE $TIME_WARP

#------------------------------------------------------------
#  Part 3: Launch the vehicles.
#    Field use (--shoreside): skipped entirely. Each real boat is started
#    on its own Pablo with ./launch_vehicle.sh (no --sim), which detects
#    its own name/type/front-seat IP and connects back to --shore=<this IP>.
#------------------------------------------------------------
if [ "${SHORE_ONLY}" = "yes" ]; then
    if [ "${JUST_MAKE}" = "-j" ]; then
        echo "$ME: Shoreside targ file made; exiting without launch."
        exit 0
    fi
    echo "$ME: Shoreside only. Start each boat on its Pablo with:"
    echo "$ME:   ./launch_vehicle.sh --shore=${IP_ADDR:-<this-machine-IP>} 1"
    if [ "${XLAUNCHED}" != "yes" ]; then
        uMAC --paused targ_shoreside.moos
        trap "" SIGINT
        echo; echo "$ME: Halting all apps"
        kill -- -$$
    fi
    exit 0
fi

for ((IX=0; IX<VAMT; IX++)); do
    MPORT=$((9001 + IX))
    PSHARE=$((9201 + IX))
    vecho "Launching ${VNAMES[$IX]} (mport=$MPORT)"
    ./launch_vehicle.sh --auto --sim --mport=$MPORT --pshare=$PSHARE \
        --vname=${VNAMES[$IX]}   --color=${VCOLOR[$IX]}    \
        --start_pos=${VEHPOS[$IX]} --slotpos=${VSLOT[$IX]} \
        $JUST_MAKE $VERBOSE $TIME_WARP
    sleep 0.4
done

if [ "${JUST_MAKE}" = "-j" ]; then
    echo "$ME: All targ files made; exiting without launch."
    exit 0
fi

#------------------------------------------------------------
#  Part 4: Unless -x, hold on uMAC until the mission is quit.
#------------------------------------------------------------
if [ "${XLAUNCHED}" != "yes" ]; then
    uMAC --paused targ_shoreside.moos
    trap "" SIGINT
    echo; echo "$ME: Halting all apps"
    kill -- -$$
fi

exit 0
