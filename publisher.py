# publisher.py

import rospy
from std_msgs.msg import Float32
import random
import time

if __name__ == "__main__":
    rospy.init_node("test_data_publisher")

    speed_pub = rospy.Publisher("/robot_speed", Float32, queue_size=10)
    battery_pub = rospy.Publisher("/battery_level", Float32, queue_size=10)
    load_pub = rospy.Publisher("/load_level", Float32, queue_size=10)

    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        speed = round(random.uniform(0.1, 1.5), 2)
        battery = round(random.uniform(20, 100), 1)
        load = round(random.uniform(10, 90), 1)

        rospy.loginfo(f"Speed: {speed} | Battery: {battery} | Load: {load}")
        speed_pub.publish(speed)
        battery_pub.publish(battery)
        load_pub.publish(load)

        rate.sleep()
