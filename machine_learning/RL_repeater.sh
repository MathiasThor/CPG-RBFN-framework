# ----------------------------------------------------------------------------
# "THE BEER-WARE LICENSE" (Revision 44):
# This software was written by Mathias Thor <mathias@mmmi.sdu.dk> in 2018
# As long as you retain this notice you can do whatever you want with it.
# If we meet some day, and you think this stuff is worth it, you can
# buy me a beer in return.
# ----------------------------------------------------------------------------

#!/bin/bash
set -e

PROGNAME="RL_repeater.sh"

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
-t, --trails [INTEGER]              number of trails
-n, --testname [STRING]             name of the test
-e, --encoding [STRING]             encoding of CPGRBFN
-r, --robot [STRING]                robot type

EOF
    exit 1
}

sleeptime=2
trail=1
dataname="noname-"
timestamp=`date "+%d%m%H%M"`
encoding="noinput"
robot="noinput"

if [ -z "$1" ]
  then
    usage "No argument supplied"
fi

while [ $# -gt 0 ] ; do
    case "$1" in
    -h|--help)
        usage
        ;;
    -t|--trails)
        trail=$2
        shift
        ;;
    -n|--testname)
        dataname="$2-"
        shift
        ;;
    -e|--encoding)
        encoding="$2"
        shift
        ;;
    -r|--robot)
        robot="$2"
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

parent_path=$( cd "$(dirname "${BASH_SOURCE[0]}")" ; pwd -P )
cd "$parent_path"

i=1
timestamp=`date "+D%d%mT%H%M%S"`
newdirname="RL_data_${timestamp}.dat"
mkdir ./../data/storage/${newdirname}

while [ $i -le $trail ]; do
    # Delete everything in Jobs
    find $parent_path/../data/jobs/ -type f -delete

    # Perform Trial
    echo " "
    echo "----"
    echo "trail $i of $trail!"
    echo "----"
    if [ $encoding == "noinput" ] && [ $robot == "noinput" ]; then
        python3 RL_master.py # >/dev/null
    elif [ $encoding == "noinput" ]; then
        python3 RL_master.py -r $robot # >/dev/null
    elif [ $robot == "noinput" ]; then
        python3 RL_master.py -e $encoding # >/dev/null
    else
        python3 RL_master.py -e $encoding -t $robot # >/dev/null
    fi

    # Move result to storage
    newname="RL_data-t$i.dat"
    cp -r $parent_path/../data/jobs/ $parent_path/../data/storage/${newdirname}/${newname}

    i=$(( i + 1 ))
done

echo "Ending Program!"
echo "Brought to you by M. Thor."
