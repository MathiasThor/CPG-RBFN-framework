#!/bin/bash
set -e

PROGNAME="run_sim.sh"

die() {
    echo "$PROGNAME: $*" >&2
    exit 1
}

usage() {
    if [ "$*" != "" ] ; then
        echo "Error: $*"
    fi

    cat << EOF

Usage: $PROGNAME [OPTION ...] [foo] [bar]
Options:
-h, --help                          display this usage message and exit
-t, --time [INTEGER]                length of simulation in seconds
-i, --idsim [INEGER]                ID of the simulation that should host the simulation

EOF
    exit 1
}

simtime=10 # length of the simulation instance (seconds)
simID=1    # simulation ID that should host the simulation
rollout=-1 # uses no noise
blackout=0 # dont black out the simulation
policy_selector=-1 # 1="only parameters", 2="only feedback", 3="both", -1="base parameters"
behaviour_selector=-1
rollout_selector=-1
# TODO behaviour!

while [ $# -gt 0 ] ; do
    case "$1" in
    -h|--help)
        usage
        ;;
    -t|--time)
        simtime=$2
        shift
        ;;
    -i|--idsim)
        simID=$2
        shift
        ;;
    -*)
        usage "Unknown option '$1'"
        ;;
    *)
        usage "Invalid parameter was provided: '$1'"
        ;;
    esac
    shift
done

echo "*******************"
echo " "1 = "RUN base controller"
echo " "2 = "RUN module controller"
read -p 'Option: ' policy_selector
echo ""

if [ $policy_selector -eq 2 ]
then
  echo " "1 = "Obstacle behaviour"
  echo " "2 = "Tilt behaviour"
  echo " "3 = "Direction behaviour"
  echo " "4 = "High behaviour"
  echo " "5 = "Low behaviour"
  echo " "6 = "Pipe behaviour"
  echo " "7 = "Wall behaviour"
  echo " "8 = "Narrow behaviour"
  echo " "9 = "Advanced behaviors"
  echo " "10 = "Primitive behaviors"
  read -p 'Option: ' behaviour_selector
  echo ""
fi

if [ $policy_selector -eq 1 ] || [ $policy_selector -eq 2 ]
then
  echo "Specify ## for RL_job_##.json"
  echo " "0 = "use stored job file"
  echo " # = use RL_job_##.json"
  read -p 'Option: ' rollout_selector
  echo ""

  if [ $rollout_selector -eq 0 ]
  then
    rollout_selector="-1"
  else
    rollout_selector="-${rollout_selector}"
  fi
fi




behaviour="none"

case $behaviour_selector in

  1)
    behaviour="obstacle"
    ;;

  2)
    behaviour="tilt"
    ;;

  3)
    behaviour="direction"
    ;;

  4)
    behaviour="high"
    ;;

  5)
    behaviour="low"
    ;;

  6)
    behaviour="pipe"
    ;;

  7)
    behaviour="wall"
    ;;

  8)
    behaviour="narrow"
    ;;

  9)
    behaviour="multiple"
    ;;

  10)
    behaviour="multiple_primitive"
    ;;

esac

echo " "
echo "*** CONTROLLER OUTPUT ***"

#echo $behaviour
#echo $policy_selector
#echo "./../interfaces/morf/sim/build_dir/bin/morf_controller $simID $rollout $simtime $blackout $policy_selector $behaviour"
./../interfaces/morf/sim/build_dir/bin/morf_controller $simID $rollout_selector $simtime $blackout $policy_selector $behaviour
