import csv
import math

# path = '/home/y-takahashi/catkin_ws/src/nav_cloning/data/ang/offset_ang/straight/ang.csv'
# path = '/home/y-takahashi/catkin_ws/src/nav_cloning/data/ang/offset_ang/620_666/ang.csv'
path = '/home/y-takahashi/catkin_ws/src/nav_cloning/data/ang/00_02/ang.csv'
pos_list = []

with open(path, 'r') as csvfile:
        for row in csvfile:
            pos_list.append(row)

def calc_avg():
    no = 8
    while no < len(pos_list):  
        cur_pos = pos_list[no]
        pos = cur_pos.split(',')
        print(pos)
        value = float(pos[1])
        value = [value]
        no += 9
        average = sum(value) / len(value)
    print(f"平均値: {average}")

calc_avg()

