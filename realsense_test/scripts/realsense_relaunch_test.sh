#!/bin/bash

# NOTE: This script should not be used using rosrun nor as a roslaunch node
# Instead use the script as
# `roscd realsense_test`
# `./realsense_relaunch_test.sh`

# Default arguments.
# If assigning, do in the following format:
# realsense_relaunch_test.sh <ITERATION_LIMIT> <CAM1_SERIAL> <CAM2_SERIAL> <SLEEP_TIME> <CAM1_TOPIC> <CAM2_TOPIC>

ITERATION_LIMIT=${1:-1000}
CAM1_SERIAL=${2:-746112060759}
CAM2_SERIAL=${3:-746112061703}
SLEEP_TIME=${4:-15}
CAM1_TOPIC=${5:-/pick/depth_registered/points}
CAM2_TOPIC=${6:-/place/depth_registered/points}

counter=0
while [ ${counter} -le ${ITERATION_LIMIT} ]
do
	printf "\n\n\nRunning iteration:${counter}\n"
	roslaunch realsense_test bolles_dual_d415.launch serial_no_camera1:=${CAM1_SERIAL} serial_no_camera2:=${CAM2_SERIAL} initial_reset:=true
	printf "Echoing 10 messages from ${CAM1_TOPIC}\n"
        r1=`timeout --foreground 30s rostopic echo ${CAM1_TOPIC} --noarr -p -n 10 | grep "optical_frame" | wc -l`
        printf "Echoing 10 messages from ${CAM2_TOPIC}\n"
        r2=`timeout --foreground 30s rostopic echo ${CAM2_TOPIC} --noarr -p -n 10 | grep "optical_frame" | wc -l`
        rosnode kill -a
        killall -9 roscore
        killall -9 rosmaster
        printf "Killed all nodes\n"
        if [[ "$r1" -eq "10" ]] && [[ "$r2" -eq "10" ]]; then
                printf "RESULT: ${counter} Success! Echo returned: ${r1},${r2}\n"

        else
                printf "RESULT: ${counter} Fail! Echo returned: ${r1},${r2}\n"
        fi
        sleep ${SLEEP_TIME}
	((counter++))
done
