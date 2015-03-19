#!/bin/bash

# usage
print_usage() {
    echo "Usage: $(basename $0) <true/false/reset>"
    echo ""
    echo "This script is used to enable, disable or reset the diagnostics framework."
    echo "A reset will cause the system to be disabled and then enabled again."
    echo ""
}

# script body
if [[ $1 == "-h" ]]; then
    print_usage
    exit 1
fi

case "$1" in
    true)
        echo "Enabling diagnostics ..."
        rosservice call /pilot/fault_control "request: true"
        ;;

    false)
        echo "Disabling diagnostics ..."
        rosservice call /pilot/fault_control "request: false"
        ;;
    reset)
        echo "Resetting diagnostics ..."
        rosservice call /pilot/fault_control "request: false"
        rosservice call /pilot/fault_control "request: true"
        ;;
    *)
        print_usage
        exit 1
        ;;

esac

#echo -n "Reconfiguring the pilot ..."
#rosservice call /pilot/reload "{}"

exit 0
