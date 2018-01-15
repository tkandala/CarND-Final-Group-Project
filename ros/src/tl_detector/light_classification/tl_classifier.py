from styx_msgs.msg import TrafficLight
import os
import tensorflow as tf
import numpy as np
import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError



# Load a graph from file (needs to be protobuf - .pb)
def _load_graph(graph, config):
    with tf.Session(graph=tf.Graph(), config=config) as sess:   # https://www.tensorflow.org/api_docs/python/tf/Graph
        assert tf.get_default_session() is sess
        # check functions here:
        # - https://www.tensorflow.org/api_docs/python/tf/Assert
        # - https://www.tensorflow.org/versions/r0.12/api_docs/python/client/session_management
        gradef = tf.GraphDef()
        with tf.gfile.Open(graph_file, 'rb') as f:
            data = f.read()
            gradef.ParseFromString(data)
            # some reading https://developers.google.com/protocol-buffers/docs/pythontutorial
        tf.import_graph_def(gradef, name='')
        graph = tf.get_default_graph()
        return graph


class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        # NEEDS ADDITION!
        # Needs to add Model directory and classifier model as args (model_directory, 'Model')
        classification_model_path = os.path.join()
        if not os.path.exists(classification_model_path):
            rospy.logerr('Classification model not found at {}'.format(classification_model_path))

        # Optimization set up
        self.config = tf.ConfigProto()

        # Load graph
        self.graph_classification = _load_graph(classification_model_path, self.config)

        # Create the tensorflow session
        self.sess_classification = tf.Session(graph=self.graph_classification, config=self.config)

        # Tensorflow publisher
        self.bridge = CvBridge()
        self.traffic_light_pub = rospy.Publisher('/tld/traffic_light', Image, queue_size = 1)

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction

         """  Cassification """
        with self.sess_classification.as_default(), self.graph_classification.as_default():
            sfmax = list(self.sess_classification.run(tf.nn.softmax(self.out_graph.eval(feed_dict={self.in_graph: [image]}))))
            sf_ind = sfmax.index(max(sfmax))

            if rospy.get_param('~publish_traffic_light', False):
    
                self.traffic_light_pub.publish(self.bridge.cv2_to_imgmsg(image, "rgb8"))

        return self.index2msg[sf_ind]


        #return TrafficLight.UNKNOWN
