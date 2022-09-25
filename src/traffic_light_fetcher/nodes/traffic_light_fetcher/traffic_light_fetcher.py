#!/usr/bin/env python
# coding=utf-8

from traffic_light_fetcher.tl_detection import TrafficLightDetector
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool
import rospy
import cv2


class TrafficLightFetcher:
    def __init__(self):

        # Input video path
        self.__video_path = rospy.get_param('~video_path')

        # -----------
        # Publishers
        # -----------
        self._tl_status_pub = rospy.Publisher(
            "traffic_light_detected", Bool, queue_size=1)
        self._tl_size_pub = rospy.Publisher(
            "traffic_light_size", Vector3, queue_size=1)

        # ----------------
        # Node Attributes
        # ----------------
        rospy.loginfo(self.__class__.__name__ + " - Loading detector...")
        self.__tl_detector = TrafficLightDetector()
        rospy.loginfo(self.__class__.__name__ +
                      " - Detector loaded successfully! ")
        # Publish frequnecy by default is one frame per second
        self.__rate = rospy.Rate(1)

    def _process_stream(self):
        """ 
        Processes video stream and publishes traffic light information
        """

        video_capturer = cv2.VideoCapture(self.__video_path)
        success, image = video_capturer.read()

        while success and not rospy.is_shutdown():
            try:
                if not video_capturer.isOpened():
                    raise ConnectionError

                is_tl_detected = False

                bounds = self.__tl_detector.detect(image)

                if (len(bounds) > 0):
                    is_tl_detected = True

                    ymin, xmin, ymax, xmax = tuple(bounds)

                    vector_msg = Vector3()
                    vector_msg.x = xmax - xmin
                    vector_msg.y = ymax - ymin
                    vector_msg.z = 0

                    # Only publish bounds when traffic light is detected
                    self._tl_size_pub.publish(vector_msg)

                # Always publish detection status
                self._tl_status_pub.publish(Bool(is_tl_detected))

                success, image = video_capturer.read()

            except ConnectionError:
                video_capturer.release()
                rospy.logerr(self.__class__.__name__ +
                             " video connection dropped..")

            self.__rate.sleep()

    def run(self):
        self._process_stream()


if __name__ == '__main__':
    rospy.init_node('traffic_light_fetcher',
                    anonymous=False, log_level=rospy.ERROR)
    node = TrafficLightFetcher()
    node.run()
