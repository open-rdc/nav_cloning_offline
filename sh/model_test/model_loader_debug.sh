#!/bin/bash

# Ctrl+C を押されたときの終了処理
function ctrl_c() {
    echo "Ctrl+C detected, exiting..."
    exit 1
}

trap ctrl_c SIGINT

for i in $(seq 1 100)
do
    echo "$i"
    roslaunch nav_cloning nav_cloning_sim.launch script:="model_loader_debug.py" use_waypoint_nav:=false use_cmd_vel:=false model_num:="$i"
    sleep 10
done