#!/bin/bash
#  Script: clean.sh   (skara3_2 - remove generated + run artifacts)
rm -rf  MOOSLog_* LOG_* XLOG_* *~
rm -f   targ_*.moos targ_*.bhv
rm -f   .LastOpenedMOOSLogDirectory
rm -f   vpositions.txt vslotpos.txt vtransitspd.txt vnames.txt vcolors.txt
