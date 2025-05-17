import csv
import numpy as np
import os

file_path = '/home/y-takahashi/catkin_ws/src/nav_cloning/data/loss/00_03/'
list = np.array([])
sum = np.zeros(4000)
file_count = 0

for file_name in os.listdir(file_path):
    file_count += 1
print(file_count)

for i in range(1, file_count + 1):
    with open(file_path + str(i) + '.csv', 'r') as f:
        reader = csv.reader(f)
        for row in reader:
            list = np.append(list, float(row[0]))
        sum = sum + list
        list = np.array([])
avg = sum / file_count
print(avg)
np.savetxt(file_path + 'loss_average.csv', avg, delimiter=',')