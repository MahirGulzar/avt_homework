#!/usr/bin/env python
# coding=utf-8

import os
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'    # Supress tensorflow log clutter
import tensorflow as tf
import tensorflow_hub as hub
import numpy as np

class TrafficLightDetector:

    TRAFFIC_LIGHT_LABEL = 24

    def __init__(self):

        # Load object detection model from tensorflow hub by default we use faster_rcnn
        model_url = "https://tfhub.dev/google/faster_rcnn/openimages_v4/inception_resnet_v2/1"
        self.__detector = hub.load(model_url).signatures['default']

    def detect(self, image):
        """ Takes RGB image and returns traffic light bounds if detected

        Parameters
        ----------
        image : np.ndarray
            An RGB image of shape (width, height, 3)

        Returns
        -------
        np.ndarray
            An np.ndarray of shape (4, ) if traffic light is detected otherwise (0, )
        """

        converted_img = tf.image.convert_image_dtype(image, tf.float32)[
            tf.newaxis, ...]

        result = self.__detector(converted_img)
        # Extract detection results into numpy arrays
        result = {key: value.numpy() for key, value in result.items()}

        # Filter out detected traffic lights
        indices = np.where(result["detection_class_labels"]
                           == self.TRAFFIC_LIGHT_LABEL)[0]

        if len(indices) > 0:

            # For the task we are only interested in one traffic light
            # So, we take the traffic light with highest scores

            max_score_idx = np.argmax(
                np.take(result["detection_scores"], indices, axis=0))

            return result["detection_boxes"][indices[max_score_idx]]
