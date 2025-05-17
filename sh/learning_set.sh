for i in `seq 1 10`
do
    echo "$i"
    rosrun nav_cloning offline_learning_set.py "$i"
    sleep 10s
done
# rosrun nav_cloning offline_learning.py 1
    