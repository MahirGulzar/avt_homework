#!/usr/bin/env python
# coding=utf-8

import numpy as np
import os
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'    # Supress tensorflow log clutter
import tensorflow as tf
import tensorflow_hub as hub


class TrafficLightDetector:

    TRAFFIC_LIGHT_LABEL = 10

    def __init__(self):

        # Load object detection model from tensorflow hub by default we use efficientdet-lite
        model_url = "https://tfhub.dev/tensorflow/efficientdet/lite4/detection/2"
        self.__detector = hub.load(model_url)

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

        converted_img = tf.image.convert_image_dtype(image, tf.uint8)[
            tf.newaxis, ...]

        boxes, scores, classes, _ = self.__detector(converted_img)

        # Filter out detected traffic lights
        indices = np.where(classes[0]
                           == self.TRAFFIC_LIGHT_LABEL)[0]

        if len(indices) > 0:

            # For the task we are only interested in one traffic light
            # So, we take the traffic light with highest scores

            max_score_idx = np.argmax(
                np.take(scores[0], indices))

            return np.take(boxes, indices, axis=1)[0][max_score_idx]

        return np.empty(0)
