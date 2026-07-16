#!/bin/bash
#------------------------------------------------------------
#  Script: launch_shoreside.sh   (skara2)
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
MOOS_PORT="9000"
PSHARE_PORT="9200"
VNAMES=""

#------------------------------------------------------------
for ARGI; do
    if [ "${ARGI}" = "--help" -o "${ARGI}" = "-h" ]; then
        echo "$ME [OPTIONS] [time_warp]"
        echo "  --auto, -a         Script launch, no uMAC"
        echo "  --just_make, -j    Only create targ files"
        echo "  --verbose, -v      Verbose"
        echo "  --mport=<9000>     Shoreside MOOSDB port"
        echo "  --pshare=<9200>    Shoreside pShare port"
        echo "  --vnames=<a:b:c>   Colon-separated vehicle names"
        exit 0
    elif [ "${ARGI//[^0-9]/}" = "$ARGI" -a "$TIME_WARP" = 1 ]; then
        TIME_WARP=$ARGI
    elif [ "${ARGI}" = "--verbose" -o "${ARGI}" = "-v" ]; then
        VERBOSE="yes"
    elif [ "${ARGI}" = "--just_make" -o "${ARGI}" = "-j" ]; then
        JUST_MAKE="yes"
    elif [ "${ARGI}" = "--auto" -o "${ARGI}" = "-a" ]; then
        AUTO_LAUNCHED="yes"
    elif [ "${ARGI:0:5}" = "--ip=" ]; then
        IP_ADDR="${ARGI#--ip=*}"
    elif [ "${ARGI:0:8}" = "--mport=" ]; then
        MOOS_PORT="${ARGI#--mport=*}"
    elif [ "${ARGI:0:9}" = "--pshare=" ]; then
        PSHARE_PORT="${ARGI#--pshare=*}"
    elif [ "${ARGI:0:9}" = "--vnames=" ]; then
        VNAMES="${ARGI#--vnames=*}"
    else
        echo "$ME: Bad Arg:[$ARGI]. Exit Code 1."
        exit 1
    fi
done

#------------------------------------------------------------
NSFLAGS="--strict --force"
if [ "${AUTO_LAUNCHED}" = "no" ]; then
    NSFLAGS="--interactive --force"
fi

nsplug meta_shoreside.moos targ_shoreside.moos $NSFLAGS WARP=$TIME_WARP \
       IP_ADDR=$IP_ADDR         MOOS_PORT=$MOOS_PORT    \
       PSHARE_PORT=$PSHARE_PORT VNAMES=$VNAMES

if [ "${JUST_MAKE}" = "yes" ]; then
    echo "$ME: shoreside targ file made."
    exit 0
fi

#------------------------------------------------------------
echo "Launching Shoreside MOOS Community. WARP=$TIME_WARP"
pAntler targ_shoreside.moos >& /dev/null &

if [ "${AUTO_LAUNCHED}" = "yes" ]; then
    exit 0
fi

uMAC targ_shoreside.moos
trap "" SIGINT
kill -- -$$
