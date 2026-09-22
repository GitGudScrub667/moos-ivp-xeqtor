#!/bin/bash -e
#----------------------------------------------------------
#  Script: launch.sh
#  Author: Michael Benjamin
#  LastEd: May 20th 2019
#----------------------------------------------------------
#  Part 1: Set Exit actions and declare global var defaults
#----------------------------------------------------------
TIME_WARP=1
COMMUNITY="bravo"
GUI="yes"

#  Assignment 8 BONUS: both are nsplug macros, overridable per launch.
#  Defaults are the values the lab sheet asks for.
RETURN_DIST=200      # m of at-depth travel before heading home
DEPTH_THRESH=25      # m; pOdometry only accumulates deeper than this

#----------------------------------------------------------
#  Part 2: Check for and handle command-line arguments
#----------------------------------------------------------
for ARGI; do
    if [ "${ARGI}" = "--help" -o "${ARGI}" = "-h" ] ; then
	echo "launch.sh [SWITCHES] [time_warp]   "
	echo "  --dist=<meters>      Return after this much at-depth travel (default 200)"
	echo "  --depth=<meters>     Only count distance below this depth  (default 25)"
	echo "  --nogui              Run without pMarineViewer"
	echo "  --help, -h           Show this help message            " 
	exit 0;
    elif [ "${ARGI}" = "--nogui" ] ; then
	GUI="no"
    elif [ "${ARGI:0:7}" = "--dist=" ] ; then
        RETURN_DIST="${ARGI#--dist=}"
        if [ "${RETURN_DIST//[^0-9]/}" != "$RETURN_DIST" -o -z "$RETURN_DIST" ]; then
            echo "launch.sh: --dist needs a positive integer. Exit 1."; exit 1
        fi
    elif [ "${ARGI:0:8}" = "--depth=" ] ; then
        DEPTH_THRESH="${ARGI#--depth=}"
        if [ "${DEPTH_THRESH//[^0-9]/}" != "$DEPTH_THRESH" -o -z "$DEPTH_THRESH" ]; then
            echo "launch.sh: --depth needs a positive integer. Exit 1."; exit 1
        fi
    elif [ "${ARGI//[^0-9]/}" = "$ARGI" -a "$TIME_WARP" = 1 ]; then 
        TIME_WARP=$ARGI
    else 
        echo "launch.sh Bad arg:" $ARGI " Exiting with code: 1"
        exit 1
    fi
done


#----------------------------------------------------------
#  Part 3: Launch the processes
#----------------------------------------------------------
#  Sanity check the depth threshold against the mission geometry.
#  This mission only ever sits at two depths: the west loiter at 30m and the
#  east loiter at 10m (meta_bravo.bhv const_dep_west / const_dep_east). If
#  DEPTH_THRESH is at or above the deepest one, pOdometry never accumulates
#  anything, ODOMETRY_DIST_AT_DEPTH stays 0, and the vehicle NEVER returns.
#  Note the lab sheet's own example, --depth=40, falls into exactly that hole.
DEEPEST_LOITER=30
if [ "$DEPTH_THRESH" -ge "$DEEPEST_LOITER" ]; then
    echo "launch.sh WARNING: --depth=$DEPTH_THRESH is >= the deepest loiter "
    echo "  (${DEEPEST_LOITER}m). ODOMETRY_DIST_AT_DEPTH will stay at 0 and the"
    echo "  vehicle will never trigger its return. Either lower --depth, or"
    echo "  deepen const_dep_west in meta_bravo.bhv to match."
fi

#  Expand the meta_ files into targ_ files. clean.sh already removes targ_*.
nsplug meta_$COMMUNITY.moos targ_$COMMUNITY.moos -f \
       DEPTH_THRESH=$DEPTH_THRESH

nsplug meta_$COMMUNITY.bhv  targ_$COMMUNITY.bhv  -f \
       RETURN_DIST=$RETURN_DIST  DEPTH_THRESH=$DEPTH_THRESH

echo "Launching $COMMUNITY with WARP:" $TIME_WARP \
     " return after" $RETURN_DIST "m deeper than" $DEPTH_THRESH "m"
pAntler targ_$COMMUNITY.moos --MOOSTimeWarp=$TIME_WARP >& /dev/null &

uMAC -t targ_$COMMUNITY.moos
kill -- -$$
