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
    roslaunch nav_cloning nav_cloning.launch script:="model_test_real.py" model_num:="$i"
    sleep 10
done