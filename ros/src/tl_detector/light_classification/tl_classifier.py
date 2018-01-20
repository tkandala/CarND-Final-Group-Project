from styx_msgs.msg import TrafficLight
import cv2
import os
import sys
import time
import scipy.misc
import numpy as np
import rospy
import tensorflow as tf
from collections import defaultdict
from io import StringIO

class TLClassifier(object):
    def __init__(self):
        self.faster_rcnn_sim_model = 'resources/frozen_sim/frozen_inference_graph.pb' 
        self.faster_rcnn_real_model = 'resources/frozen_real/frozen_inference_graph.pb'
        self.category_index = {
            1: {'id': 1, 'name': u'Green'}, 
            2: {'id': 2, 'name': u'Red'}, 
            3: {'id': 3, 'name': u'Yellow'}, 
            4: {'id': 4, 'name': u'off'}
        }

        self.detection_graph = tf.Graph()
        self.session = tf.Session(graph=self.detection_graph)

        with self.detection_graph.as_default():
            od_graph_def = tf.GraphDef()

            with tf.gfile.GFile(self.faster_rcnn_sim_model, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')

            # Definite input and output Tensors for detection_graph
            self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
            
            # Each box represents a part of the image where a particular object was detected.
            self.detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
            
            # Each score represent how level of confidence for each of the objects.
            # Score is shown on the result image, together with the class label.
            self.detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
            self.detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
            self.num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if image is None:
            return TrafficLight.UNKNOWN

        with self.detection_graph.as_default(), self.session.as_default():
            # Fix dimensions since the model expects images to have shape: [1, None, None, 3]
            image_expanded = np.expand_dims(image, axis=0)

            time0 = time.time()

            # Do the inference 
            (boxes, scores, classes, num) = self.session.run(
                [self.detection_boxes, self.detection_scores, self.detection_classes, self.num_detections],
                feed_dict = { self.image_tensor: image_expanded }
            )

            time1 = time.time()

            boxes = np.squeeze(boxes)
            scores = np.squeeze(scores)
            classes = np.squeeze(classes).astype(np.int32)
            
            min_score_thresh = .50
            for i in range(boxes.shape[0]):
                if scores is not None and scores[i] > min_score_thresh:
                    class_name = self.category_index[classes[i]]['name']
                    rospy.loginfo('{}, {}, {}'.format(class_name, scores[i], (time1 - time0) * 1000))
                    if class_name == 'Red':
                        return TrafficLight.RED
                    elif class_name == 'Green':
                        return TrafficLight.GREEN
                    elif class_name == 'Yellow':
                        return TrafficLight.YELLOW
                    else:
                        return TrafficLight.UNKNOWN

        return TrafficLight.UNKNOWN
