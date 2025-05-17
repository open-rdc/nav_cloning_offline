#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from pynput import keyboard

# 初期化
rospy.init_node('keyboard_teleop')
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

# 速度の初期値
linear_speed = 0.2  # 前進速度
angular_speed = 0.25  # 角速度

# キー押下時の動作
def on_press(key):
    global linear_speed, angular_speed

    twist = Twist()
    
    try:
        twist.linear.x = linear_speed
        twist.angular.z = 0.0

        if key.char == 'w':  # 前進
            twist.angular.z = 0.0
        
        elif key.char == 's':  # 後退
            twist.linear.x = -linear_speed
            twist.angular.z = 0.0
        
        elif key.char == 'a':  # 左旋回
            twist.angular.z = angular_speed
        
        elif key.char == 'd':  # 右旋回
            twist.angular.z = -angular_speed
        
        elif key.char == 'q':  # 停止
            twist.angular.z = 0.0
        
        # パブリッシュ
        pub.publish(twist)
    except AttributeError:
        pass

# キー離したときの動作
def on_release(key):
    if key == keyboard.Key.esc:
        # ESCキーで終了
        return False

# リスナーの設定
listener = keyboard.Listener(on_press=on_press, on_release=on_release)
listener.start()

# メインループ
rospy.spin()