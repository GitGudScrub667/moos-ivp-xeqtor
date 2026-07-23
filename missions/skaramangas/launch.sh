#!/bin/bash -e
#----------------------------------------------------------
#  Script: launch.sh   (Skaramangas cove viewer mission)
#----------------------------------------------------------
TIME_WARP=1
COMMUNITY="skara"

for ARGI; do
    if [ "${ARGI}" = "--help" -o "${ARGI}" = "-h" ] ; then
        echo "launch.sh [SWITCHES] [time_warp]"
        echo "  --help, -h   Show this help message"
        exit 0;
    elif [ "${ARGI//[^0-9]/}" = "$ARGI" -a "$TIME_WARP" = 1 ]; then
        TIME_WARP=$ARGI
    else
        echo "launch.sh Bad arg:" $ARGI " Exiting with code: 1"
        exit 1
    fi
done

echo "Launching $COMMUNITY MOOS Community with WARP:" $TIME_WARP
pAntler $COMMUNITY.moos --MOOSTimeWarp=$TIME_WARP >& ./skara_launch.log &

uMAC -t $COMMUNITY.moos
kill -- -$$
