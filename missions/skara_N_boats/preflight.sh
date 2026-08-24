#!/bin/bash
#------------------------------------------------------------
#  Script: preflight.sh   (skara_N_boats)
#  BOAT-SIDE preflight. READ-ONLY: it changes nothing, builds nothing,
#  pulls nothing, launches nothing. Safe to run at the dock, offline.
#
#  Run it on EACH Pablo before launching that boat:
#      ./preflight.sh --shore=<LAPTOP_IP>
#  On the laptop, to check the shoreside instead:
#      ./preflight.sh --shoreside
#
#  It answers four questions:
#    1. Is this machine on the same code as the show build?  (repo pins)
#    2. Are the apps this mission needs actually installed?  (PATH)
#    3. Does the boat know who it is, and is it a pool boat? (identity)
#    4. Can it reach its front seat and the shoreside?       (links)
#
#  See BOAT_OPS.txt for what to do about anything it flags.
#------------------------------------------------------------

#------------------------------------------------------------
#  THE PINS. These are the shas the show build was validated on.
#  >> UPDATE THESE if the show build is ever re-pinned. <<
#  Core moos-ivp and moos-ivp-greece are deliberately NOT tracked to
#  upstream -- do not "fix" a mismatch by pulling them (see BOAT_OPS.txt).
#------------------------------------------------------------
EXP_CORE_BRANCH="water-tested-5f591ca6e"
EXP_CORE_SHA="5f591ca6e"
EXP_GREECE_BRANCH="main"
EXP_GREECE_SHA="6f51ca1"
EXP_XEQTOR_BRANCH="main"

SHORE_IP=""
PROBLEMS=0
MODE="boat"        # boat | shore

for ARGI; do
    if [ "${ARGI}" = "--help" -o "${ARGI}" = "-h" ]; then
        echo "preflight.sh [--shore=<LAPTOP_IP>] [--shoreside]"
        echo "  Read-only preflight. Changes nothing."
        echo "  --shore=<ip>  the laptop's IP, so this boat can ping it"
        echo "  --shoreside   check THIS machine as the shoreside laptop"
        echo "                (skips the boat-only apps and identity)"
        exit 0
    elif [ "${ARGI}" = "--shoreside" ]; then
        MODE="shore"
    elif [ "${ARGI:0:8}" = "--shore=" ]; then
        SHORE_IP="${ARGI#--shore=*}"
    else
        echo "preflight.sh: Bad Arg:[$ARGI]."
        exit 1
    fi
done

ok()    { echo "  ok      $1"; }
flag()  { echo "  CHECK>> $1"; PROBLEMS=$((PROBLEMS+1)); }
head_() { echo; echo "-- $1 ------------------------------------------------"; }

echo "==========================================================="
if [ "$MODE" = "shore" ]; then WHAT="SHORESIDE"; else WHAT="BOAT"; fi
echo " skara_N_boats PREFLIGHT [$WHAT]   ($(hostname))   $(date '+%Y-%m-%d %H:%M')"
echo "==========================================================="

#------------------------------------------------------------
head_ "1. REPO PINS (all boats + the laptop must match)"
#------------------------------------------------------------
check_repo() {
    local dir="$1" exp_branch="$2" exp_sha="$3" label="$4" watch="$5"
    if [ ! -d "$dir/.git" ]; then
        flag "$label: $dir is not a git repo -- cannot verify"
        return
    fi
    local br sha
    br=$(git -C "$dir" branch --show-current 2>/dev/null)
    sha=$(git -C "$dir" rev-parse HEAD 2>/dev/null)
    local shown="${sha:0:9}"
    if [ "$exp_sha" = "" ]; then
        # No pin to compare against: just report it for cross-machine comparison.
        echo "  info    $label: branch=$br sha=$shown  (must MATCH the laptop)"
        if [ "$br" != "$exp_branch" ]; then
            flag "$label: on branch '$br', expected '$exp_branch'"
        fi
        return
    fi
    if [ "$br" = "$exp_branch" ] && [ "${sha:0:${#exp_sha}}" = "$exp_sha" ]; then
        ok "$label: $br @ $shown"
    else
        flag "$label: has $br @ $shown -- expected $exp_branch @ $exp_sha"
    fi
    # Only flag edits to the paths that affect what a BOAT runs. A dirty
    # mission-data file elsewhere in the repo is not this mission's business.
    local dirty
    dirty=$(git -C "$dir" status --porcelain -- $watch 2>/dev/null | grep -v '^??')
    if [ -n "$dirty" ]; then
        flag "$label: LOCAL EDITS to code this boat runs:"
        echo "$dirty" | sed 's/^/            /'
    fi
}
#             repo                 branch               sha               label                     paths that matter to a boat
check_repo "$HOME/moos-ivp"        "$EXP_CORE_BRANCH"   "$EXP_CORE_SHA"   "moos-ivp (core, stock)" "."
check_repo "$HOME/moos-ivp-greece" "$EXP_GREECE_BRANCH" "$EXP_GREECE_SHA" "moos-ivp-greece"        "src scripts"
check_repo "$HOME/moos-ivp-xeqtor" "$EXP_XEQTOR_BRANCH" ""                "moos-ivp-xeqtor"        "missions/skara_N_boats src"

#------------------------------------------------------------
head_ "2. APPS ON PATH"
#------------------------------------------------------------
APPS="nsplug uMAC pAntler MOOSDB pShare pHostInfo uProcessWatch pRealm pLogger"
if [ "$MODE" = "shore" ]; then
    APPS="$APPS pMarineViewer pArrivalSync uFldShoreBroker uFldNodeComms uTimerScript"
else
    APPS="$APPS pHelmIvP pNodeReporter pMarinePIDV22 pContactMgrV20 uFldNodeBroker"
    APPS="$APPS iBackSeatBroker pBB_DGPS_EKF pBB_Health pThrustMix pDeadManPost"
    APPS="$APPS get_robot_info_greece.sh"
fi
for app in $APPS ; do
    if command -v "$app" >/dev/null 2>&1; then
        ok "$app"
    else
        flag "$app NOT on PATH  (source ~/.bashrc, or it was never built)"
    fi
done

#------------------------------------------------------------
head_ "3. BUILD FRESHNESS (were the binaries built from the checked-out code?)"
#------------------------------------------------------------
# Classic make-style staleness: is any SOURCE FILE newer than the binary
# built from it? That catches the case that matters -- "pulled but never
# rebuilt" -- because a pull rewrites the source files' timestamps.
# Deliberately NOT compared against commit dates: the normal edit -> build ->
# test -> commit order leaves the binary older than its own commit, which
# would flag every healthy machine.
# Limits: it cannot detect a checkout of OLDER code that was never rebuilt,
# and a fresh clone restamps every source file. Clean here means "nothing
# obviously stale", not "provably in sync".
check_fresh() {
    # args: <label> then one or more "binary|source-dir"
    local label="$1"; shift
    local pair bin src newer found=0
    for pair in "$@"; do
        bin="${pair%%|*}"
        src="${pair#*|}"
        [ -f "$bin" ] || continue
        [ -d "$src" ] || continue
        found=1
        newer=$(find "$src" -type f -newer "$bin" -print -quit 2>/dev/null)
        if [ -n "$newer" ]; then
            flag "$label: $(basename "$bin") is OLDER than its source"
            echo "            e.g. $newer"
            echo "            -> that repo was updated and never rebuilt (./build.sh)"
        else
            ok "$label: $(basename "$bin") newer than its source"
        fi
    done
    [ $found -eq 0 ] && echo "  info    $label: no binaries found to date-check"
    return 0
}
check_fresh "moos-ivp" \
    "$HOME/moos-ivp/bin/pHelmIvP|$HOME/moos-ivp/ivp/src" \
    "$HOME/moos-ivp/bin/pMarineViewer|$HOME/moos-ivp/ivp/src"
if [ "$MODE" = "shore" ]; then
    check_fresh "moos-ivp-xeqtor" \
        "$HOME/moos-ivp-xeqtor/bin/pArrivalSync|$HOME/moos-ivp-xeqtor/src/pArrivalSync"
else
    check_fresh "moos-ivp-greece" \
        "$HOME/moos-ivp-greece/bin/pBB_DGPS_EKF|$HOME/moos-ivp-greece/src/pBB_DGPS_EKF" \
        "$HOME/moos-ivp-greece/bin/iBackSeatBroker|$HOME/moos-ivp-greece/src/iBackSeatBroker" \
        "$HOME/moos-ivp-greece/bin/pThrustMix|$HOME/moos-ivp-greece/src/pThrustMix" \
        "$HOME/moos-ivp-greece/bin/pBB_Health|$HOME/moos-ivp-greece/src/pBB_Health"
fi

#------------------------------------------------------------
head_ "4. MISSION FIELD FILES"
#------------------------------------------------------------
# Work against the real mission dir. This script is also kept as a loose copy
# on the Desktop, so if it is not sitting in the mission itself, go find it.
MDIR="$(cd "$(dirname "$0")" && pwd)"
if [ ! -f "$MDIR/launch_vehicle.sh" ]; then
    MDIR="$HOME/moos-ivp-xeqtor/missions/skara_N_boats"
    echo "  info    run from outside the mission -- checking $MDIR"
fi
if [ ! -d "$MDIR" ]; then
    flag "mission dir not found: $MDIR"
    MDIR="."
fi
cd "$MDIR" || exit 1
MISSING=""
for f in vnames.txt vpositions.txt vcolors.txt vslotpos.txt; do
    [ -f "$f" ] || MISSING="$MISSING $f"
done
if [ "$MISSING" = "" ]; then
    ok "vnames/vpositions/vcolors/vslotpos present"
else
    flag "missing:$MISSING  -- run ./init_field.sh (in $(pwd))"
fi

#------------------------------------------------------------
head_ "5. IDENTITY (who does this boat think it is?)"
#------------------------------------------------------------
echo "  hostname -I: $(hostname -I)"
if [ "$MODE" = "shore" ]; then
    echo "  info    shoreside mode: no boat identity to detect."
    echo "          Give the boats the field-subnet address from the line above:"
    echo "            ./launch.sh --shoreside --ip=<that IP> 1"
    VNAME=""; FSEAT=""
elif ! command -v get_robot_info_greece.sh >/dev/null 2>&1; then
    flag "get_robot_info_greece.sh not on PATH -- cannot determine identity"
    VNAME=""; FSEAT=""
else
    VNAME=$(get_robot_info_greece.sh --name 2>/dev/null)
    VTYPE=$(get_robot_info_greece.sh --TYPE 2>/dev/null)
    VIP=$(get_robot_info_greece.sh --ip   2>/dev/null)
    FSEAT=$(get_robot_info_greece.sh --fseat 2>/dev/null)
    if [ "$VNAME" = "" ] || [ "$VIP" = "" ] || [ "$VIP" = "localhost" ]; then
        flag "identity NOT detected. The 10.3N.1.100 address must be FIRST in"
        echo "          hostname -I above. (launch_vehicle.sh -> Exit Code 2)"
    else
        ok "name=$VNAME  type=$VTYPE  ip=$VIP  fseat=$FSEAT"
        [ "$VTYPE" = "BBOAT" ] || flag "type is '$VTYPE', expected BBOAT -- this will not drive hardware"
        if [ -f vnames.txt ]; then
            if grep -qx "$VNAME" vnames.txt; then
                ok "'$VNAME' is in this mission's pool"
            else
                flag "'$VNAME' is NOT in the pool ($(tr '\n' ' ' < vnames.txt))"
                echo "          -> launch_vehicle.sh will stop with Exit Code 5."
            fi
        fi
    fi
fi

#------------------------------------------------------------
head_ "6. LINKS"
#------------------------------------------------------------
ping_check() {
    local ip="$1" what="$2"
    [ "$ip" = "" ] && return
    if ping -c2 -W2 "$ip" >/dev/null 2>&1; then
        ok "$what $ip reachable"
    else
        flag "$what $ip NOT reachable"
    fi
}
if [ "$MODE" = "shore" ]; then
    echo "  info    shoreside mode: boats ping THIS machine, not the reverse."
elif [ "$FSEAT" = "" ]; then
    echo "  info    front seat unknown (identity not detected)"
else
    ping_check "$FSEAT" "front seat"
    echo "          (front seat down = this boat cannot move. RC and the boat's"
    echo "           own wifi AP being up do NOT prove this link is up.)"
fi
if [ "$SHORE_IP" = "" ]; then
    echo "  info    no --shore=<LAPTOP_IP> given, shoreside not tested"
else
    ping_check "$SHORE_IP" "shoreside"
fi

#------------------------------------------------------------
echo
echo "==========================================================="
if [ $PROBLEMS -eq 0 ]; then
    if [ "$MODE" = "shore" ]; then
        echo " PREFLIGHT CLEAN [SHORESIDE].  Launch with:"
        echo "   ./launch.sh --shoreside --ip=<LAPTOP_IP> 1"
    else
        echo " PREFLIGHT CLEAN [BOAT].  Launch with:"
        echo "   ./launch_vehicle.sh --shore=${SHORE_IP:-<LAPTOP_IP>} -v 1"
    fi
else
    echo " PREFLIGHT: $PROBLEMS item(s) marked CHECK>> above."
    if [ "$MODE" = "boat" ]; then
        echo " (On the LAPTOP? The BlueBoat-only apps and the identity check flag"
        echo "  there by design -- use --shoreside to check the laptop instead.)"
    fi
    if [ "$MODE" = "shore" ]; then
        echo " See FIELD_OPS.txt before launching the shoreside."
    else
        echo " See BOAT_OPS.txt section 5 before launching this boat."
    fi
fi
echo "==========================================================="
[ $PROBLEMS -eq 0 ]
