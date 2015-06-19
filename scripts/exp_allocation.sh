#!/bin/bash

# References:
#	http://answers.ros.org/question/10714/start-and-stop-rosbag-within-a-python-script/
#	http://stackoverflow.com/questions/3332043/obtaining-pid-of-child-process

# allows communication with children
trap "kill 0" SIGINT

# config
MODE="lines"
TRJ="../data/wavetank_eight.json"

# utils
function vehicle_reset() {
	rosparam set /pilot/fault_control false

	rosrun vehicle_core fault_clear.sh
	rosservice call /thrusters/monitor/reset
	
	rosservice call /pilot/switch "request: false"
	rosservice call /pilot/fault_control "request: false"
	rosservice call /nav/reset
}

function recording_start() {
	# start rosbag record using subshell
	(rosbag record -a -o "$1_$2") &
    pid=$!

    # wait a bit to allow the rosbag initialization
    sleep 1

    return $pid
}

function recording_stop() {
	# use rosnode to grab the rosbag record node name
	nodes=$(rosnode list | grep /record)

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
# ...

## REFERENCE RUNS ##
for i in {1..5}
do
    # reset the vehicle state
    vehicle_reset

	# run config
	TAG="ref"
	rosparam set /pilot/fault_control false
	rosparam set /pilot/optimal_allocation true

	# rosrun vehicle_core fault_clear.sh
	# rosservice call /pilot/fault_control "request: false"

    # enable recording
    recording_start $TAG $i 

    echo "starting ${TAG} navigation experiment"
    rosrun vehicle_core path_executor.py 0 --mode="$MODE" --path="$TRJ" --output="$TAG"
    echo "${TAG} run[$i]: exit code $?"

    # disable recording
   	recording_stop

    # reset the vehicle state
    vehicle_reset
done

# final cleanup
# ...

kill 0
exit 0
