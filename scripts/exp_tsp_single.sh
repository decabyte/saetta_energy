#!/bin/bash
# References:
#	http://answers.ros.org/question/10714/start-and-stop-rosbag-within-a-python-script/
#	http://stackoverflow.com/questions/3332043/obtaining-pid-of-child-process
#   http://stackoverflow.com/questions/965053/extract-filename-and-extension-in-bash

# usage
print_usage() {
    echo "Usage: $(basename $0).sh <mission_config>"
    echo "Execute the mission specified in the config file while recording .bag and .csv files."
    echo ""
    echo "Mandatory arguments:"
    echo "  <mission_config>: path to the mission config file (JSON format)"
}

# script body
if [[ ! -n $1 ]]; then
    print_usage
    exit 1
fi

# config
OUTPUT="$(pwd)"
MISSION=$1

#filename="${1##*/}"
#extension="${filename##*.}"
#name="${filename%.*}"
#MISSION_OPT="$(dirname $1)/${name}_opt.$extension"

# check mission file
if [ ! -f $MISSION ]; then
    echo "Mission file not available: $MISSION"
    exit 2
fi

# water current speed
declare -a WATER_SPEED

# fixed currents (value is setting maximum boundary, average is WS / 2)
#WATER_SPEED[0]=0.00  #0.00   (real value)
#WATER_SPEED[1]=0.20  #0.10   (real value)
#WATER_SPEED[2]=0.40  #0.20   (real value)
WATER_SPEED[3]=0.60  #0.30   (real value)
#WATER_SPEED[4]=0.80  #0.40   (real value)
#WATER_SPEED[5]=1.00  #0.50   (real value)

# water direction
WD=0.0

# slow varying currents (value is setting maximum boundary, average is WS / 2)
#WATER_SPEED[0]=0.00
#WATER_SPEED[1]=0.10
#WATER_SPEED[2]=0.20
#WATER_SPEED[3]=0.30
#WATER_SPEED[4]=0.40
##WATER_SPEED[5]=0.50
##WATER_SPEED[6]=0.60

# allows communication with children
trap "kill 0" SIGINT

# utils
function vehicle_reset() {
    rostopic pub -1 /nav/sim/water vehicle_interface/FloatArrayStamped "values: [0.0, 0.001, 0.0]"

	rosparam set /pilot/fault_control false

	rosrun vehicle_core fault_clear.sh
	rosservice call /thrusters/monitor/reset
	
	rosservice call /pilot/switch "request: false"
	rosservice call /pilot/fault_control "request: false"

    rosservice call /path/control "command: reset"
	rosservice call /nav/reset
}

function recording_start() {
	# start rosbag record using subshell
	(rosbag record -a -O "$1_$2") &
    pid=$!

    # wait a bit to allow the rosbag initialization
    sleep 2

    return $pid
}

function recording_stop() {
	# use rosnode to grab the rosbag record node name
	nodes=$(rosnode list | grep /record)

    # wait a bit to allow the proper flush
	sleep 1

	for node in $nodes
	do
		echo "terminating node $node ..."
		rosnode kill $node
	done

	# send kill signal to all children
    #kill -INT $PID_BAG
    
    # wait a bit to allow the rosbag cleanup
    sleep 1
}

# initial cleanup
vehicle_reset


## STANDARD RUNS ##
for index in ${!WATER_SPEED[*]}
do
    rosservice call /saetta/map/reset
    rosservice call /saetta/map/switch "request: true"

	# reference run config
	WS="${WATER_SPEED[$index]}"
	TAG="reference"

    # enable recording
    recording_start $TAG $WS

	# reset configuration
	#rosparam set /pilot/fault_control false
	#rosparam set /pilot/optimal_allocation false

    # adjust water current (fixed)
    #rostopic pub -1 /nav/sim/water vehicle_interface/FloatArrayStamped "values: [$WS, 0.0, 0.0, $WD, 0.0001]"

    # adjust water current (slow varying)
    rostopic pub -1 /nav/sim/water vehicle_interface/FloatArrayStamped "values: [$WS, 0.025, 0.001, $WD, 0.0001]"

    # adjust water current (high varying)
	#rostopic pub -1 /nav/sim/water vehicle_interface/FloatArrayStamped "values: [$WS, 0.050, 0.001, $WD, 0.0001]"

    # reset failure and fault mitigation
	# rosrun vehicle_core fault_clear.sh
	# rosservice call /pilot/fault_control "request: false"

    echo "starting ${TAG}_${WS} navigation experiment"
    rosrun saetta_energy node_executor.py --output="$OUTPUT" --label="${TAG}_${WS}" $MISSION
    echo "${TAG} run[$index]: exit code $?"

    # save map
    rosservice call /saetta/map/switch "request: false"
    rosservice call /saetta/map/dump "$OUTPUT/map_${TAG}_${WS}.json"

    # disable recording
    sleep 2
   	recording_stop

    # reset the vehicle state
    vehicle_reset
done

# final cleanup
# vehicle_reset

kill 0
exit 0
