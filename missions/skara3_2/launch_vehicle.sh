#!/bin/bash
#------------------------------------------------------------
#  Script: launch_vehicle.sh   (skara3_2 - one sim boat)
#------------------------------------------------------------
vecho() { if [ "$VERBOSE" != "" ]; then echo "$ME: $1"; fi }
on_exit() { echo; echo "$ME: Halting all apps"; kill -- -$$; }
trap on_exit SIGINT

ME=$(basename "$0")
TIME_WARP=1
VERBOSE=""
JUST_MAKE="no"
AUTO_LAUNCHED="no"

IP_ADDR="localhost"
MOOS_PORT="9001"
PSHARE_PORT="9201"
SHORE_IP="localhost"
SHORE_PSHARE="9200"

VNAME="abe"
COLOR="coral"
START_POS="x=0,y=0,heading=0"
SLOT_POS="0,0"
RETURN_POS=""
ORBIT_SPD="2.0"
MAX_SPD="4"

# The common ring (same for every boat).
CIRCLE_X="45"
CIRCLE_Y="-95"
CIRCLE_RAD="27.05"

#------------------------------------------------------------
for ARGI; do
    if [ "${ARGI}" = "--help" -o "${ARGI}" = "-h" ]; then
        echo "$ME [OPTIONS] [time_warp]"
        echo "  --auto, -a          Script launch, no uMAC"
        echo "  --just_make, -j     Only create targ files"
        echo "  --verbose, -v       Verbose"
        echo "  --sim, -s           Sim vehicle (default here)"
        echo "  --mport=<9001>      Veh MOOSDB port"
        echo "  --pshare=<9201>     Veh pShare listen port"
        echo "  --shore=<localhost> Shoreside IP"
        echo "  --shore_pshare=<9200> Shoreside pShare port"
        echo "  --vname=<abe>       Vehicle name"
        echo "  --color=<coral>     Vehicle color"
        echo "  --start_pos=<X,Y,H> Sim start pose"
        echo "  --slotpos=<X,Y>     This boat's ring slot"
        echo "  --orbit_spd=<m/s>   Ring orbit speed"
        exit 0
    elif [ "${ARGI//[^0-9]/}" = "$ARGI" -a "$TIME_WARP" = 1 ]; then
        TIME_WARP=$ARGI
    elif [ "${ARGI}" = "--verbose" -o "${ARGI}" = "-v" ]; then
        VERBOSE="yes"
    elif [ "${ARGI}" = "--just_make" -o "${ARGI}" = "-j" ]; then
        JUST_MAKE="yes"
    elif [ "${ARGI}" = "--auto" -o "${ARGI}" = "-a" ]; then
        AUTO_LAUNCHED="yes"
    elif [ "${ARGI}" = "--sim" -o "${ARGI}" = "-s" ]; then
        : # sim is the only mode here
    elif [ "${ARGI:0:5}" = "--ip=" ]; then
        IP_ADDR="${ARGI#--ip=*}"
    elif [ "${ARGI:0:8}" = "--mport=" ]; then
        MOOS_PORT="${ARGI#--mport=*}"
    elif [ "${ARGI:0:9}" = "--pshare=" ]; then
        PSHARE_PORT="${ARGI#--pshare=*}"
    elif [ "${ARGI:0:8}" = "--shore=" ]; then
        SHORE_IP="${ARGI#--shore=*}"
    elif [ "${ARGI:0:15}" = "--shore_pshare=" ]; then
        SHORE_PSHARE="${ARGI#--shore_pshare=*}"
    elif [ "${ARGI:0:8}" = "--vname=" ]; then
        VNAME="${ARGI#--vname=*}"
    elif [ "${ARGI:0:8}" = "--color=" ]; then
        COLOR="${ARGI#--color=*}"
    elif [ "${ARGI:0:12}" = "--start_pos=" ]; then
        START_POS="${ARGI#--start_pos=*}"
    elif [ "${ARGI:0:10}" = "--slotpos=" ]; then
        SLOT_POS="${ARGI#--slotpos=*}"
    elif [ "${ARGI:0:12}" = "--orbit_spd=" ]; then
        ORBIT_SPD="${ARGI#--orbit_spd=*}"
    else
        echo "$ME: Bad Arg:[$ARGI]. Exit Code 1."
        exit 1
    fi
done

# Return to the start position by default (x,y only - drop any heading).
if [ "${RETURN_POS}" = "" ]; then
    RETURN_POS=$(echo "${START_POS}" | sed 's/,heading=[^,]*//')
fi

#------------------------------------------------------------
if [ "${VERBOSE}" = "yes" ]; then
    echo "=========== launch_vehicle.sh : $VNAME ==========="
    echo "MOOS_PORT=$MOOS_PORT  PSHARE=$PSHARE_PORT  COLOR=$COLOR"
    echo "START_POS=$START_POS"
    echo "SLOT_POS=$SLOT_POS  ORBIT_SPD=$ORBIT_SPD (transit spd set live by pArrivalSync)"
    echo "RETURN_POS=$RETURN_POS"
    echo "RING = ($CIRCLE_X,$CIRCLE_Y) r=$CIRCLE_RAD"
fi

#------------------------------------------------------------
NSFLAGS="--strict --force"
if [ "${AUTO_LAUNCHED}" = "no" ]; then
    NSFLAGS="--interactive --force"
fi

nsplug meta_vehicle.moos targ_$VNAME.moos $NSFLAGS WARP=$TIME_WARP \
       IP_ADDR=$IP_ADDR           MOOS_PORT=$MOOS_PORT     \
       PSHARE_PORT=$PSHARE_PORT   SHORE_IP=$SHORE_IP       \
       SHORE_PSHARE=$SHORE_PSHARE VNAME=$VNAME             \
       COLOR=$COLOR               START_POS="$START_POS"   \
       MAX_SPD=$MAX_SPD

nsplug meta_vehicle.bhv targ_$VNAME.bhv $NSFLAGS           \
       VNAME=$VNAME               START_POS="$START_POS"   \
       SLOT_POS="$SLOT_POS"       RETURN_POS="$RETURN_POS" \
       ORBIT_SPD=$ORBIT_SPD       COLOR=$COLOR             \
       CIRCLE_X=$CIRCLE_X  CIRCLE_Y=$CIRCLE_Y  CIRCLE_RAD=$CIRCLE_RAD

if [ "${JUST_MAKE}" = "yes" ]; then
    echo "$ME: targ files for $VNAME made."
    exit 0
fi

#------------------------------------------------------------
echo "Launching $VNAME MOOS Community. WARP=$TIME_WARP"
pAntler targ_$VNAME.moos >& /dev/null &

if [ "${AUTO_LAUNCHED}" = "yes" ]; then
    exit 0
fi

uMAC targ_$VNAME.moos
trap "" SIGINT
kill -- -$$
