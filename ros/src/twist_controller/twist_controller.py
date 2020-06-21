#!/usr/bin/env python

import rospy
from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController

import time

# Constants
GAS_DENSITY = 2.858
ONE_MPH = 0.44704

TAU = 0.5 # Cutoff frequency
T_SAMPLE = 0.02 # Sample time

MIN_SPEED = 0.1


class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit,
                 accel_limit, wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):
        """
        Lesson 8: DBW Walkthrough in Project: Programming a real Self-Driving Car was very useful here
        """
        
        
        # TODO: Implement
        
        # Member variables
        self.vehicle_mass = vehicle_mass 
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband 
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit

        self.wheel_radius = wheel_radius
        self.wheel_base = wheel_base
        self.steer_ratio = steer_ratio
        self.max_lat_accel = max_lat_accel
        self.max_steer_angle = max_steer_angle
        
        # PID
        Kp = 0.3 # Proportional gain
        Ki = 0.1 # Intergral gain
        Kd = 0.0 # Derivative gain
        
        min_throttle = 0.0
        max_throttle = 0.2
        
        # Init controllers
        self.throttle_controller = PID(Kp, Ki, Kd, min_throttle, max_throttle)
        self.yaw_controller = YawController(self.wheel_base, self.steer_ratio, MIN_SPEED, self.max_lat_accel, self.max_steer_angle)
        
        # Init lowpass filter
        self.velocity_lowpass = LowPassFilter(TAU, T_SAMPLE)
        
        self.last_time = rospy.get_time()
        
        

    def control(self, current_vel, dbw_enabled, linear_vel, angular_vel):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        if not dbw_enabled:
            self.throttle_controller.reset()
            return 0.0, 0.0, 0.0
        
        current_vel = self.velocity_lowpass.filt(current_vel)    
        
        steering = self.yaw_controller.get_steering(linear_vel, angular_vel, current_vel)
        
        vel_error = linear_vel - current_vel
        self.last_vel = current_vel

        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time
        
        throttle = self.throttle_controller.step(vel_error, sample_time)
        brake = 0
        
        if linear_vel == 0. and current_vel < 0.1:
            throttle = 0
            brake = 400 
            
        elif throttle < 0.1 and vel_error < 0:
            throttle = 0
            decel = max(vel_error, self.decel_limit)
            brake = abs(decel) * self.vehicle_mass * self.wheel_radius
        
        return throttle, brake, steering