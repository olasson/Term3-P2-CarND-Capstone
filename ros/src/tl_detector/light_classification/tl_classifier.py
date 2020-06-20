from styx_msgs.msg import TrafficLight

#olasson imports
import rospy
import cv2
import os
import yaml
import tensorflow as tf
import numpy as np

SCORE_THRESHOLD = 0.5

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        self.session = None
        self.graph_model = None
       
        self.tl_colors = {1: TrafficLight.GREEN, 2: TrafficLight.YELLOW, 3: TrafficLight.RED, 4: TrafficLight.UNKNOWN}
        
        # As shown in "Traffic Light Detection Node Overview" section
        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)
        
        #model_path = os.path.dirname(os.path.realpath(__file__) + self.config['model'])
        model_path = '/home/workspace/CarND-Capstone/ros/src/tl_detector/light_classification/frozen_inference_graph.pb'
        self.load_model(model_path)
        
    def load_model(self, model_path):
        """
        Loads the chosen model, depending on whether it is the site (real) or styx (simulation)
        """
        model_config = tf.ConfigProto()
        model_config.graph_options.optimizer_options.global_jit_level = tf.OptimizerOptions.ON_1
        
        # This is technically not needed, but good practice: https://stackoverflow.com/questions/39614938/why-do-we-need-tensorflow-tf-graph
        self.tl_model = tf.Graph()
        
        # Very useful for this section: https://leimao.github.io/blog/Save-Load-Inference-From-TF-Frozen-Graph/
        with tf.Session(graph = self.tl_model, config = model_config) as sess:
            self.session = sess
            tl_graph = tf.GraphDef()
            with tf.gfile.GFile(model_path, 'rb') as f:
                tl_graph.ParseFromString(f.read())
                tf.import_graph_def(tl_graph, name = '')        

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        
        # Prepare image
        image = cv2.resize(image, (300, 300))
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        
        # Input/output tensor for frozen graph
        image_tensor = self.tl_model.get_tensor_by_name('image_tensor:0')
        
        
        model_scores = self.tl_model.get_tensor_by_name('detection_scores:0')
        model_boxes = self.tl_model.get_tensor_by_name('detection_boxes:0')
        model_classes = self.tl_model.get_tensor_by_name('detection_classes:0')
        
        
        
        # Use the model to predict
        boxes, scores, classes = self.session.run([model_boxes, model_scores, model_classes],
                                                  feed_dict = {image_tensor: np.expand_dims(image, axis = 0)})     
        
        # Remove single-dimensional entries from the output arrays.
        boxes = np.squeeze(boxes)
        scores = np.squeeze(scores)
        classes = np.squeeze(classes)
        
        for i, b in enumerate(boxes):
            if scores[i] > SCORE_THRESHOLD:
                traffic_light_class = self.tl_colors[classes[i]]
                rospy.loginfo("Traffic light was {}".format(traffic_light_class))
                return traffic_light_class
            else:
                #rospy.loginfo("Traffic light UNKNOWN!")
                pass
        return TrafficLight.UNKNOWN
    

        
        
        
