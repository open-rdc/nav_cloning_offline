from __future__ import print_function
import roslib
roslib.load_manifest('nav_cloning')
import rospy
import csv
import math

class cal_offset_ang_vel:
    def __init__(self):
        rospy.init_node('calc_ang_vel_node', anonymous=True)
        self.path = roslib.packages.get_pkg_dir('nav_cloning') + '/data/'
        self.time = 2.0
        self.off_ang = 0

    def calc_pos(self):
        with open(self.path + 'path/amcl_pose_1.csv', 'r') as fr:
            reader = csv.reader(fr)
            amcl_pos_list = list(reader)
        
        with open(self.path + 'path/amcl_pose_02.csv', 'r') as fs:
            reader = csv.reader(fs)
            pos_list_02 = list(reader)

        i = 0
        while i < len(amcl_pos_list) - 1:
            x0, y0 = float(amcl_pos_list[i][0]), float(amcl_pos_list[i][1])
            j = i + 1
            while j < len(amcl_pos_list):
                x, y = float(amcl_pos_list[j][0]), float(amcl_pos_list[j][1])
                distance = math.sqrt((x - x0)**2 + (y - y0)**2)
                if distance >= 0.36:
                    for k in range(1, len(pos_list_02)):
                        pos_x, pos_y = float(pos_list_02[k][0]), float(pos_list_02[k][1])
                        the = math.atan2(pos_y - y, pos_x - x) 
                        for self.off_ang in [-0.0872665, 0, 0.0872665]:
                            ang_vel = the + self.off_ang/ self.time
                            line = [str(ang_vel)]
                        with open(self.path + 'ang/ang_02.csv', "a") as f:
                            writer = csv.writer(f, lineterminator="\n")
                            writer.writerow(line)
                    i = j  # Update 'i' to 'j' to move x0, y0 to the current x, y
                    break
                j += 1
            else:
                # If no sufficient distance found, break the loop
                break

if __name__ == '__main__':
    node = cal_offset_ang_vel()
    node.calc_pos()
