#!/bin/bash

# References:
#	http://answers.ros.org/question/10714/start-and-stop-rosbag-within-a-python-script/
#	http://stackoverflow.com/questions/3332043/obtaining-pid-of-child-process

# allows communication with children
trap "kill 0" SIGINT

# config
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

# initial cleanup
#vehicle_reset

## REFERENCE RUNS ##
for i in {1..5}
do
    # reset the vehicle state
    vehicle_reset

	# run config
	# rosparam set /pilot/fault_control false
	# rosrun vehicle_core fault_clear.sh
	# rosservice call /pilot/fault_control "request: false"

    # record the experiment
    (rosbag record -a -o "ref_${i}") &
    PID_BAG=$!
    sleep 1

    python wavetank_exp.py 0 --mode=lines --path="$TRJ" --output=ref
    echo "run[$i]: exit code $?"

    # send kill signal to all children
    kill $PID_BAG
    sleep 1

    # reset the vehicle state
    vehicle_reset
done

# final cleanup
#vehicle_reset

kill 0
exit 0
