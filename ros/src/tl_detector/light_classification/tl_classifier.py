from styx_msgs.msg import TrafficLight

import rospy
import cv2
import tensorflow as tf
import numpy as np
#import os
import yaml

SCORE_THRESHOLD = 0.5

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        
        self.tl_colors = {1: TrafficLight.GREEN, 2: TrafficLight.YELLOW, 3: TrafficLight.RED, 4: TrafficLight.UNKNOWN}
        
        self.session = None
        self.tl_model = None
        
        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)
        
        self.load_model('/home/workspace/CarND-Capstone/ros/src/tl_detector/light_classification/frozen_inference_graph.pb')
        
    def load_model(self, model_path):
        config = tf.ConfigProto()
        config.graph_options.optimizer_options.global_jit_level = tf.OptimizerOptions.ON_1
        
        # This is technically not needed, but good practice: https://stackoverflow.com/questions/39614938/why-do-we-need-tensorflow-tf-graph
        self.tl_model = tf.Graph()
        
        # Very useful for this section: https://leimao.github.io/blog/Save-Load-Inference-From-TF-Frozen-Graph/
        with tf.Session(graph = self.tl_model, config = config) as sess:
            self.session = sess
            tl_graph_def = tf.GraphDef()
            with tf.gfile.GFile(model_path, 'rb') as f:
                tl_graph_def.ParseFromString(f.read())
                tf.import_graph_def(tl_graph_def, name = '')
                
    def get_classification(self, image):
        """Determines the color of the traffic light in the image
        Args:
            image (cv::Mat): image containing the traffic light
        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """
        #TODO implement light color prediction
        image = cv2.resize(image, (300, 300))
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        
        image_tensor = self.tl_model.get_tensor_by_name('image_tensor:0')
        model_boxes = self.tl_model.get_tensor_by_name('detection_boxes:0')
        model_scores = self.tl_model.get_tensor_by_name('detection_scores:0')
        model_classes = self.tl_model.get_tensor_by_name('detection_classes:0')
        
        # Use the model to predict 
        boxes, scores, classes = self.session.run([model_boxes, model_scores, model_classes],
                                                  feed_dict = {image_tensor: np.expand_dims(image, axis = 0)})
                                                  
        boxes = np.squeeze(boxes)
        scores = np.squeeze(scores)
        classes = np.squeeze(classes)
        
        for i, b in enumerate(boxes):
            if scores[i] > SCORE_THRESHOLD:
                traffic_light_class = self.tl_colors[classes[i]]
                rospy.loginfo("Traffic light detected {}".format(traffic_light_class))
                return traffic_light_class
            else:
                rospy.loginfo("Traffic light: unknown detection!")
        
        return TrafficLight.UNKNOWN
    

        
        
        
