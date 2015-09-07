#!/bin/bash
# References:
#	http://answers.ros.org/question/10714/start-and-stop-rosbag-within-a-python-script/
#	http://stackoverflow.com/questions/3332043/obtaining-pid-of-child-process
#   http://stackoverflow.com/questions/965053/extract-filename-and-extension-in-bash

## usage
#print_usage() {
#    echo "Usage: $(basename $0).sh <mission_config>"
#    echo "Execute the mission specified in the config file while recording .bag and .csv files."
#    echo ""
#    echo "Mandatory arguments:"
#    echo "  <mission_config>: path to the mission config file (JSON format)"
#}
#
## script body
#if [[ ! -n $1 ]]; then
#    print_usage
#    exit 1
#fi

# config
OUTPUT="$(pwd)"

# water current speed
declare -a WSS

# fixed currents (value is setting maximum boundary, average is WS / 2)
#WSS[0]=0.00  #0.00   (real value)
WSS[1]=0.20  #0.10   (real value)
WSS[2]=0.40  #0.20   (real value)
WSS[3]=0.60  #0.30   (real value)
WSS[4]=0.80  #0.40   (real value)
#WSS[5]=1.00  #0.50   (real value)

# current direction
WD=0.00     # radians

# allows communication with children
trap "kill 0" SIGINT

# utils
function vehicle_reset() {
    rostopic pub -1 /nav/sim/water vehicle_interface/FloatArrayStamped "values: [0.0, 0.001, 0.0]"

	rosservice call /pilot/switch "request: false"
    rosservice call /path/control "command: reset"
	rosservice call /nav/reset
}

function recording_start() {
	# start rosbag record using subshell
	(rosbag record -a -O "$1_$2") &
    pid=$!

    # wait a bit to allow the rosbag initialization
    sleep 1

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

# clean estimator
rosservice call /saetta/map/reset
rosservice call /saetta/map/switch "request: true"

TAG="current"
LBL="$(date '+%H%M')"

# enable recording
recording_start $TAG $LBL

# send path request
rostopic pub -1 /path/request vehicle_interface/PathRequest """
command: 'path'
points:
- values: [0.0, 0.0, 1.0, 0.0, 0.0, 0.0]
- values: [250.0, 0.0, 1.0, 0.0, 0.0, 0.0]
- values: [500.0, 0.0, 1.0, 0.0, 0.0, 0.0]
- values: [750.0, 0.0, 1.0, 0.0, 0.0, 0.0]
- values: [1000.0, 0.0, 1.0, 0.0, 0.0, 0.0]
options:
- {key: 'mode', value: 'fast'}
"""

for index in ${!WSS[*]}
do
    # get speed value
    WS="${WSS[$index]}"

    echo "Setting Water Speed: $WS (m/s)"

    # adjust water current (fixed)
    rostopic pub -1 /nav/sim/water vehicle_interface/FloatArrayStamped "values: [$WS, 0.025, 0.0, $WD, 0.0001]"

    # wait
    sleep 300
done

# disable monitor
rosservice call /saetta/map/switch "request: false"
rosservice call /saetta/map/dump "$OUTPUT/map_${TAG}_${LBL}.json"

# disable recording
sleep 1
recording_stop

# reset the vehicle state
vehicle_reset

kill 0
exit 0
