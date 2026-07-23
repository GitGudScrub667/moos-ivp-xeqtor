#!/bin/bash
#------------------------------------------------------------
#  Script: launch_vehicle.sh   (pireas_v2 - one sim boat)
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

# XMODE: SIM (simulated boat) or BBOAT (real BlueBoat backseat stack).
# Left EMPTY means "this is a real robot" -- identity, type and front-seat
# IP are then read from the vehicle itself in Part 4 below, the same way
# rescue_pireas does on these Pablos. --sim forces SIM.
XMODE=""
FSEAT_IP=""

# The common ring (same for every boat). Sited in the widened trapezoid
# op-region. Scaled up x1.15 (was 18.7): corner loiters now ~4.2 m clear of
# the op-edge, ~10.4 m clear of the buoy zones.
CIRCLE_X="-144"
CIRCLE_Y="-43"
CIRCLE_RAD="21.5"

#------------------------------------------------------------
for ARGI; do
    if [ "${ARGI}" = "--help" -o "${ARGI}" = "-h" ]; then
        echo "$ME [OPTIONS] [time_warp]"
        echo "  --auto, -a          Script launch, no uMAC"
        echo "  --just_make, -j     Only create targ files"
        echo "  --verbose, -v       Verbose"
        echo "  --sim, -s           Simulated vehicle (XMODE=SIM). OMIT this"
        echo "                      on a real boat: name, color, type and"
        echo "                      front-seat IP are then auto-detected, and"
        echo "                      the ring slot is looked up by boat name."
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
        XMODE="SIM"
    elif [ "${ARGI}" = "--bboat" -o "${ARGI}" = "-b" ]; then
        XMODE="BBOAT"   # force hardware config (bench-test the targ files
                        # off-robot; a real boat needs no flag at all)
    elif [ "${ARGI:0:5}" = "--ip=" ]; then
        IP_ADDR="${ARGI#--ip=*}"
    elif [ "${ARGI:0:8}" = "--mport=" ]; then
        MOOS_PORT="${ARGI#--mport=*}"
    elif [ "${ARGI:0:9}" = "--pshare=" ]; then
        PSHARE_PORT="${ARGI#--pshare=*}"
    elif [ "${ARGI:0:8}" = "--shore=" ]; then
        SHORE_IP="${ARGI#--shore=*}"
    elif [ "${ARGI:0:8}" = "--fseat=" ]; then
        FSEAT_IP="${ARGI#--fseat=*}"   # override the auto-detected front seat
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

#------------------------------------------------------------
#  Part 4: If --sim was NOT given, this is a real robot. Take identity,
#  type and front-seat IP from the vehicle itself (get_robot_info.sh),
#  the same mechanism rescue_pireas uses on these Pablos.
#------------------------------------------------------------
if [ "${XMODE}" = "" ]; then
    COLOR=`get_robot_info.sh --color`
    IP_ADDR=`get_robot_info.sh --ip`
    FSEAT_IP=`get_robot_info.sh --fseat`
    VNAME=`get_robot_info.sh --name`
    XMODE=`get_robot_info.sh --TYPE`
    if [ "$XMODE" = "" -o "$IP_ADDR" = "localhost" ]; then
        echo "$ME: Problem getting robot info. Exit Code 2."
        exit 2
    fi

    # Look this boat's ring slot and home point up BY NAME, so a real boat
    # can never be launched carrying another boat's slot. Needs the field
    # files, so run ./init_field.sh on the vehicle first.
    if [ ! -f vnames.txt -o ! -f vslotpos.txt -o ! -f vpositions.txt ]; then
        echo "$ME: field files missing -- run ./init_field.sh. Exit Code 4."
        exit 4
    fi
    LN=$(grep -n "^${VNAME}$" vnames.txt | cut -d: -f1)
    if [ "$LN" = "" ]; then
        echo "$ME: '$VNAME' is not listed in vnames.txt. Exit Code 5."
        exit 5
    fi
    SLOT_POS=$(sed -n "${LN}p" vslotpos.txt)
    # RETURN_POS must be a real point. Falling through to the sim default
    # below would resolve it to START_POS = 0,0 = the DATUM, which is on
    # the east shore, outside the op-region.
    if [ "${RETURN_POS}" = "" ]; then
        RETURN_POS=$(sed -n "${LN}p" vpositions.txt | sed 's/,heading=[^,]*//')
    fi
fi

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
    echo "XMODE=$XMODE  FSEAT_IP=$FSEAT_IP  IP_ADDR=$IP_ADDR"
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
       MAX_SPD=$MAX_SPD           XMODE=$XMODE             \
       FSEAT_IP=$FSEAT_IP

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
