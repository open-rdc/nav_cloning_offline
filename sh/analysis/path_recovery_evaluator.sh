#!/bin/bash

# Ctrl+C を押されたときの終了処理
function ctrl_c() {
    echo "Ctrl+C detected, exiting..."
    exit 1
}

trap ctrl_c SIGINT

for i in $(seq 1 10)
do
    echo "$i"
    roslaunch nav_cloning nav_cloning_sim.launch script:="path_recovery_evaluator.py" use_waypoint_nav:=false model_num:="$i"
    sleep 10
done