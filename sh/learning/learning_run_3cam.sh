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
    rosrun nav_cloning offline_learning_run_3cam.py "$i"
    sleep 10
done