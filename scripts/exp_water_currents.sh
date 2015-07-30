#!/bin/bash

# References:
#	http://answers.ros.org/question/10714/start-and-stop-rosbag-within-a-python-script/
#	http://stackoverflow.com/questions/3332043/obtaining-pid-of-child-process

# allows communication with children
trap "kill 0" SIGINT

# store data
declare -a WATER_SPEED

# water current speed
WATER_SPEED[0]=0.00
WATER_SPEED[1]=0.10
WATER_SPEED[2]=0.20
WATER_SPEED[3]=0.30
WATER_SPEED[4]=0.40
WATER_SPEED[5]=0.50
WATER_SPEED[6]=0.60

# config
MISSION="/home/valerio/src/saetta_energy/data/mission_config.json"
OUTPUT="$(pwd)"

# utils
function vehicle_reset() {
    rostopic pub -1 /nav/sim/water vehicle_interface/FloatArrayStamped "values: [0.0, 0.001, 0.0]"

	rosparam set /pilot/fault_control false

	rosrun vehicle_core fault_clear.sh
	rosservice call /thrusters/monitor/reset
	
	rosservice call /pilot/switch "request: false"
	rosservice call /pilot/fault_control "request: false"
	rosservice call /nav/reset

	rosservice call /saetta/map/reset
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


## REFERENCE STANDARD RUNS ##
for index in ${!WATER_SPEED[*]}
do
	# run config
	WS="${WATER_SPEED[$index]}"
	TAG="reference"

	# reset configuration
	rosparam set /pilot/fault_control false
	rosparam set /pilot/optimal_allocation false

    # adjust water
    rostopic pub -1 /nav/sim/water vehicle_interface/FloatArrayStamped "values: [$WS, 0.025, 0.001]"
	#rostopic pub -1 /nav/sim/water vehicle_interface/FloatArrayStamped "values: [$WS, 0.05, 0.01]"

    # reset failure and fault mitigation
	# rosrun vehicle_core fault_clear.sh
	# rosservice call /pilot/fault_control "request: false"

    # enable recording
    recording_start $TAG $WS

    echo "starting ${TAG}_${index} navigation experiment"
    rosrun saetta_energy node_executor.py --output="$OUTPUT" --label="${TAG}_${WS}" $MISSION
    echo "${TAG} run[$index]: exit code $?"

    # disable recording
    sleep 1
   	recording_stop

    # reset the vehicle state
    vehicle_reset
done

# final cleanup
# vehicle_reset

kill 0
exit 0
