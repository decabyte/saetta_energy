#!/bin/bash
trap "kill 0" SIGINT

# record the experiment
(rosbag record -a -o wavetank_exp) &
PID_BAG=$!

sleep 1


## REFERENCE RUNS ##
bash fault_clear.sh
rosservice call /pilot/fault_control "request: false"

for i in {1..5}
do
    python wavetank_exp.py 0 --output=reference
    echo "run[$i]: exit code $?"
done

sleep 1


## FAULTY RUNS ##
bash fault_inject.sh 30 100
rosservice call /pilot/fault_control "request: false"

for i in {1..5}
do
    python wavetank_exp.py 0 --output=faulty
    echo "run[$i]: exit code $?"
done

sleep 1


## FAULT CONTROL RUNS ##
bash fault_inject.sh 30 100
rosservice call /pilot/fault_control "request: true"

for i in {1..5}
do
    python wavetank_exp.py 0 --output=control
    echo "run[$i]: exit code $?"

    # resetting fault tolerant controller
    rosservice call /pilot/fault_control "request: false"
    rosservice call /pilot/fault_control "request: true"
done

sleep 1


## ADAPT CONTROL RUNS ##
bash fault_inject.sh 30 100
rosservice call /pilot/fault_control "request: true"

for i in {1..5}
do
    python wavetank_exp.py 0 --output=speeds
    echo "run[$i]: exit code $?"

    # resetting fault tolerant controller
    rosservice call /pilot/fault_control "request: false"
    rosservice call /pilot/fault_control "request: true"
done

sleep 1


# final reset
bash fault_clear.sh
rosservice call /pilot/fault_speeds "request: false"

# send kill signal to all children
kill 0
exit 0
