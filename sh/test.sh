for i in `seq 1 10`
do
    echo "$i"
    roslaunch nav_cloning nav_cloning_sim.launch cmd_vel_topic:=/nav_vel script:="test_mode.py" num:="$i"
sleep 10
done