#!/bin/bash

# Ctrl+C を押されたときの終了処理
function ctrl_c() {
    echo "Ctrl+C detected, exiting..."
    exit 1
}

trap ctrl_c SIGINT

model_num=1

roslaunch nav_cloning nav_cloning_sim.launch script:="model_loader_debug.py" use_waypoint_nav:=false use_cmd_vel:=false model_num:="$model_num"