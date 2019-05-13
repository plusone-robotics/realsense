#!/bin/bash

# Default arguments
# If configuring new arguments, usef the following format:
# realsense_reboot_test.sh <ITERATION_LIMIT> <LOGFILE_NAME> <CAM1_SERIAL> <CAM2_SERIAL> <SLEEP_TIME> <CAM1_TOPIC> <CAM2_TOPIC>

ITERATION_LIMIT=${1:-100}
LOGFILE=${HOME}/${2:-realsense.log}
CAM1_SERIAL=${2:-746112060759}
CAM2_SERIAL=${3:-746112061703}
SLEEP_TIME=${4:-4}
CAM1_TOPIC=${5:-/pick/depth_registered/points}
CAM2_TOPIC=${6:-/place/depth_registered/points}

# Exit script if ITERATION_LIMIT met
current_itr=`cat ${LOGFILE} | wc -l`
if [ ${current_itr} -ge ${ITERATION_LIMIT} ]; then
	echo "Exiting since iteration limit met">>${LOGFILE}
	exit 0
fi

# Source the installed workspace again
source /opt/por/setup.bash
# Launch the cameras as per required configuration
roslaunch realsense_test bolles_dual_d415.launch serial_no_camera1:=${CAM1_SERIAL} serial_no_camera2:=${CAM2_SERIAL} initial_reset:=true
# Wait for topics to be published and write them to a file
r1=`timeout 10m rostopic echo ${CAM1_TOPIC} --noarr -p -n 10 | grep "optical_frame" | wc -l`
r2=`timeout 10s rostopic echo ${CAM2_TOPIC} --noarr -p -n 10 | grep "optical_frame" | wc -l`
success="Failed"
if [[ "$r1" -eq "10" ]] && [[ "$r2" -eq "10" ]]; then
	success="Succeded"
fi
echo "RESULT : ${current_itr} : ${r1},${r2} : ${success}">>${LOGFILE}
# Sleep for some time for a user to log and debug or stop the script(avoid reboot)
sleep ${SLEEP_TIME}m

sudo reboot

exit 0
