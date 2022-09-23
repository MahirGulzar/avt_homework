#!/usr/bin/env python
# coding=utf-8
from traffic_light_fetcher.tl_detector import Detector
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool
import numpy as np
import rospy
import cv2


class TrafficLightFetcher:
    def __init__(self):
        
        # Input video path
        # video_path = rospy.get_param('~video_path')

        # TODO Enclose in try catch
        # self.__video_capturer = cv2.VideoCapture(video_path)

        # -----------
        # Publishers
        # -----------
        self._tl_status_pub = rospy.Publisher(
            "traffic_light_detected", Bool, queue_size=1)
        self._tl_size_pub = rospy.Publisher(
            "traffic_light_size", Vector3, queue_size=1)

          # Load a video
        # cap = cv2.VideoCapture(filename)
        

    def _process(self):
        pass

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('traffic_light_fetcher',
                    anonymous=False, log_level=rospy.INFO)
    node = TrafficLightFetcher()
    node.run()
