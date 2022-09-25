#!/usr/bin/env python
# coding=utf-8

from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32, Bool
import rospy


class TrafficLightAnalysis:
    def __init__(self):
        # -----------
        # Publishers
        # -----------
        self._tl_zone_height_pub = rospy.Publisher(
            "zone_height", Float32, queue_size=1)

        # ----------------
        # Subscribers
        # ----------------
        rospy.Subscriber('traffic_light_detected',
                         Bool, self._tl_status_callback)
        rospy.Subscriber('traffic_light_size', Vector3, self._tl_size_callback)

        # Since both subscribed topics don't have time stamps and size message is published
        # only when traffic light is detected, we cache latest size message and use it later
        self.__last_tl_size_msg = None

    def _tl_status_callback(self, msg):
        """ Callback for traffic light status

        Parameters
        ----------
        msg : std_msgs/Bool
            Trafic light detected status
        """

        if (msg.data):
            # Check if size message is available
            if self.__last_tl_size_msg is None:
                rospy.logwarn(self.__class__.__name__ +
                              " - A traffic light is detected but size information not available! ")
                return

            zone_height_msg = Float32(self.__last_tl_size_msg.y/3)
            self._tl_zone_height_pub.publish(zone_height_msg)

    def _tl_size_callback(self, msg):
        """ Callback for traffic light size

        Parameters
        ----------
        msg : geometry_msgs/Vector3
            Trafic light size message
        """
        self.__last_tl_size_msg = msg

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('traffic_light_analysis',
                    anonymous=False, log_level=rospy.WARN)
    node = TrafficLightAnalysis()
    node.run()
