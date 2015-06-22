#!/bin/bash

# References:
#	http://answers.ros.org/question/10714/start-and-stop-rosbag-within-a-python-script/
#	http://stackoverflow.com/questions/3332043/obtaining-pid-of-child-process

# allows communication with children
trap "kill 0" SIGINT

# store data
declare -a FAIL_LVL

# config
TRJ="/home/valerio/src/saetta_energy/data/test_fast.json"
MODE="fast"

# thrusters degradation levels
#FAIL_LVL[0]=85         # 100 %
FAIL_LVL[0]=68          # 80 %
FAIL_LVL[1]=51          # 60 %
FAIL_LVL[2]=34          # 40 %
FAIL_LVL[3]=17          # 20 %
FAIL_LVL[4]=0           # 0 %


# utils
function vehicle_reset() {
    rostopic pub -1 /nav/sim/water vehicle_interface/FloatArrayStamped "values: [0.0, 0.0, 0.0]"

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
for i in {1..1}
do
	# run config
	TAG="ref_std"

	# reset configuration
	rosparam set /pilot/fault_control false
	rosparam set /pilot/optimal_allocation false

    # adjust water
	#rostopic pub -1 /nav/sim/water vehicle_interface/FloatArrayStamped "values: [0.${i}, 0.0, 0.0]"

    # reset failure and fault mitigation
	# rosrun vehicle_core fault_clear.sh
	# rosservice call /pilot/fault_control "request: false"

    # enable recording
    recording_start $TAG $i 

    echo "starting ${TAG}_${i} navigation experiment"
    rosrun vehicle_core path_executor.py 0 --mode="$MODE" --path="$TRJ" --output="$TAG" --label="${TAG}_${i}"
    echo "${TAG} run[$i]: exit code $?"

    # disable recording
    sleep 1
   	recording_stop

    # reset the vehicle state
    vehicle_reset
done

## FAILED STANDARD RUNS ##
for index in ${!FAIL_LVL[*]}
do
	# run config
	LVL="${FAIL_LVL[$index]}"
	TAG="fail_std_${LVL}"
	i=0

    # reset configuration
	rosparam set /pilot/fault_control false
	rosparam set /pilot/optimal_allocation false

    # adjust water
	#rostopic pub -1 /nav/sim/water vehicle_interface/FloatArrayStamped "values: [0.${i}, 0.0, 0.0]"

	# reset failure and fault mitigation
	rosrun vehicle_core fault_clear.sh
	rosservice call /pilot/fault_control "request: false"

    # inject the failure and (optionally) enable fault control
	rosrun vehicle_core fault_inject.sh $LVL 100
	#rosservice call /pilot/fault_control "request: true"

    # enable recording
    recording_start $TAG $i

    echo "starting ${TAG}_${i} navigation experiment"
    rosrun vehicle_core path_executor.py 0 --mode="$MODE" --path="$TRJ" --output="$TAG" --label="${TAG}_${i}"
    echo "${TAG} run[$i]: exit code $?"

    # disable recording
    sleep 1
   	recording_stop

    # reset the vehicle state
    vehicle_reset
done

## MITIGATED STANDARD RUNS ##
for index in ${!FAIL_LVL[*]}
do
	# run config
	LVL="${FAIL_LVL[$index]}"
	TAG="mit_std_${LVL}"
	i=0

    # reset configuration
	rosparam set /pilot/fault_control true
	rosparam set /pilot/optimal_allocation false

    # adjust water
	#rostopic pub -1 /nav/sim/water vehicle_interface/FloatArrayStamped "values: [0.${i}, 0.0, 0.0]"

	# reset failure and fault mitigation
	rosrun vehicle_core fault_clear.sh
	rosservice call /pilot/fault_control "request: false"

    # inject the failure and (optionally) enable fault control
	rosrun vehicle_core fault_inject.sh $LVL 100
	rosservice call /pilot/fault_control "request: true"

    # enable recording
    recording_start $TAG $i

    echo "starting ${TAG}_${i} navigation experiment"
    rosrun vehicle_core path_executor.py 0 --mode="$MODE" --path="$TRJ" --output="$TAG" --label="${TAG}_${i}"
    echo "${TAG} run[$i]: exit code $?"

    # disable recording
    sleep 1
   	recording_stop

    # reset the vehicle state
    vehicle_reset
done



## REFERENCE OPTIMAL RUNS ##
for i in {1..1}
do
	# run config
	TAG="ref_opt"

	# reset configuration
	rosparam set /pilot/fault_control false
	rosparam set /pilot/optimal_allocation true

    # adjust water
	#rostopic pub -1 /nav/sim/water vehicle_interface/FloatArrayStamped "values: [0.${i}, 0.0, 0.0]"

    # reset failure and fault mitigation
	# rosrun vehicle_core fault_clear.sh
	# rosservice call /pilot/fault_control "request: false"

    # enable recording
    recording_start $TAG $i

    echo "starting ${TAG}_${i} navigation experiment"
    rosrun vehicle_core path_executor.py 0 --mode="$MODE" --path="$TRJ" --output="$TAG" --label="${TAG}_${i}"
    echo "${TAG} run[$i]: exit code $?"

    # disable recording
    sleep 1
   	recording_stop

    # reset the vehicle state
    vehicle_reset
done

## FAILED OPTIMAL RUNS ##
for index in ${!FAIL_LVL[*]}
do
	# run config
	LVL="${FAIL_LVL[$index]}"
	TAG="fail_opt_${LVL}"
	i=0

    # reset configuration
	rosparam set /pilot/fault_control false
	rosparam set /pilot/optimal_allocation true

    # adjust water
	#rostopic pub -1 /nav/sim/water vehicle_interface/FloatArrayStamped "values: [0.${i}, 0.0, 0.0]"

	# reset failure and fault mitigation
	rosrun vehicle_core fault_clear.sh
	rosservice call /pilot/fault_control "request: false"

    # inject the failure and (optionally) enable fault control
	rosrun vehicle_core fault_inject.sh $LVL 100
	#rosservice call /pilot/fault_control "request: true"

    # enable recording
    recording_start $TAG $i

    echo "starting ${TAG}_${i} navigation experiment"
    rosrun vehicle_core path_executor.py 0 --mode="$MODE" --path="$TRJ" --output="$TAG" --label="${TAG}_${i}"
    echo "${TAG} run[$i]: exit code $?"

    # disable recording
    sleep 1
   	recording_stop

    # reset the vehicle state
    vehicle_reset
done

## MITIGATION OPTIMAL RUNS ##
for index in ${!FAIL_LVL[*]}
do
	# run config
	LVL="${FAIL_LVL[$index]}"
	TAG="mit_opt_${LVL}"
	i=0

    # reset configuration
	rosparam set /pilot/fault_control true
	rosparam set /pilot/optimal_allocation true

    # adjust water
	#rostopic pub -1 /nav/sim/water vehicle_interface/FloatArrayStamped "values: [0.${i}, 0.0, 0.0]"

	# reset failure and fault mitigation
	rosrun vehicle_core fault_clear.sh
	rosservice call /pilot/fault_control "request: false"

    # inject the failure and (optionally) enable fault control
	rosrun vehicle_core fault_inject.sh $LVL 100
	rosservice call /pilot/fault_control "request: true"

    # enable recording
    recording_start $TAG $i

    echo "starting ${TAG}_${i} navigation experiment"
    rosrun vehicle_core path_executor.py 0 --mode="$MODE" --path="$TRJ" --output="$TAG" --label="${TAG}_${i}"
    echo "${TAG} run[$i]: exit code $?"

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
