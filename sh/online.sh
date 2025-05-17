for i in `seq 1 10`
do
    echo "$i"
    roslaunch nav_cloning nav_cloning_sim.launch script:="nav_cloning_node.py" num:="$i"
sleep 10
done