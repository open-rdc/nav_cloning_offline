# for i in `seq 1 10`
# do
#     echo "$i"
#     rosrun nav_cloning offline_learning_3cam.py "$i"
#     # rosrun nav_cloning offline_learning_3cam_random.py "$i"
#     sleep 10s
# done

rosrun nav_cloning offline_learning_1cam.py 1
    