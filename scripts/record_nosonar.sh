#!/bin/bash

# options
BAGS_OPTS="-a -x \"/bvt(.*)\""

# usage
print_usage() {
    echo "Usage: $0 <bag_name>"
    echo "Record all topics in a given bag file without record sonar topics."
    echo ""
    echo "Optional arguments:"
    echo "  <bag_name>: name of the .bag file to record"
}

# script body
if [[ $1 == "-h" ]]; then
    print_usage
    exit 1
fi

if [[ -n $1 ]]; then
	echo 
    BAGS_OPTS="$BAGS_OPTS -O $1"
fi

# info
echo "Recording all topics using rosbag record tool excluding sonar data."
echo "You can interrupt the recording by pressing Ctrl+C on this terminal."
echo "rosbag record $BAGS_OPTS"
echo ""

# start recording
rosbag record $BAGS_OPTS
