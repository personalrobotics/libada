#!/bin/bash
# Start ADA functionalities in tmux sessions
# to kill all sessions: pkill -f tmux

function launcher {
    tmux has-session -t $1 2>/dev/null
    if [ "$?" -eq 0 ] ; then
        echo "WARNING! $1 is already running! (tmux session)"
    elif screen -ls | grep -q $1 ; then
        echo "WARNING! $1 is already running! (screen session)"
    else
        echo "Creating a new tmux session: $1"
	tmux new-session -d -s $1
	tmux send -t $1 "source $(catkin locate)/devel/setup.bash && $2$(printf \\r)"
    fi
}

function pr_launcher {
    launcher $1 "roslaunch ada_launch $2 --wait"
}

launcher    "core"         "roscore"
sleep 5s
pr_launcher "ada_state_pub"    "state_publisher.launch"
pr_launcher "ada_ros_control"  "ros_control.launch"
