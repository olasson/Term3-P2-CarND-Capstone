import rospy
from pid import PID

# Constants
GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
        """
        Lesson 8: DBW Walkthrough in Project: Programming a real Self-Driving Car was very useful here
        """
        # TODO: Implement
        Kp = 0.3 # Proportional gain
        Ki = 0.1 # Intergral gain
        Kd = 0.0 # Derivative gain
        
        min_throttle = 0.0
        max_throttle = 0.0
        
        self.throttle_controller = PID(Kp, Ki, Kd, min_throttle, max_throttle)
        
        pass

    def control(self, *args, **kwargs):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        return 1., 0., 0.
