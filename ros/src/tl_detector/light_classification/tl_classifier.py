from styx_msgs.msg import TrafficLight
import cv2
import scipy.misc
import numpy as np

class TLClassifier(object):
    def __init__(self):
        pass

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if image is not None:
            # image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) 
            h_channel = image_hsv[:,:,0]
            reds = h_channel[h_channel > 354.0/2]
            # yellows = h_channel[(h_channel > 50.0/2) & (h_channel < 70.0/2)]
            # greens = h_channel[(h_channel > 135.0/2) & (h_channel < 150.0/2)]
            print("count of reds: %d", np.shape(reds)[0])
            if np.shape(reds)[0] > 50:
                print("red light!!!!")
        return TrafficLight.UNKNOWN
