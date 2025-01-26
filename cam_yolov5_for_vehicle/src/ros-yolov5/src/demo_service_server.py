#!/home/tristan/anaconda3/envs/yolo01/bin/python
import rospy
from yolo_bridge.yolo_bridge import Ros2Yolo

if __name__ == "__main__":
    yoloBridge = Ros2Yolo()
    rospy.spin()
