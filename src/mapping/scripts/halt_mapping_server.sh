# source ../../../devel/setup.sh

LAUNCH_FILE="run_mapping.launch"
ROSLAUNCH_PID=$(ps aux | grep roslaunch | grep "$LAUNCH_FILE" | grep -v grep | awk '{print $2}')

[ -z "$ROSLAUNCH_PID" ] || kill -INT $ROSLAUNCH_PID