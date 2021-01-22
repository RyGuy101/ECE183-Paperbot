#!/usr/bin/env python3
# Segway Dimensions 
# d = 502mm, w = 530mm
# Paperbot Dimensions
# d = 50mm, w = 90mm
# unit of length is mm

import numpy as np
import math

class Bot:
    def __init__(self, xW, xE, yN, yS, d=50, w=90, Dr=0, Df=0, H=np.zeros([3,3]), V=np.zeros([3,3]), init_state=np.array([-200.0, -200.0, 0.0])):
        self.xW = xW # western wall position
        self.xE = xE # eastern wall position
        self.yN = yN # northern wall position
        self.yS = yS # southern wall position
        self.d = d # wheel diameter
        self.w = w # robot width (distance between wheels)
        self.Dr = Dr # right distance sensor offset
        self.Df = Df # front distance sensor offset
        # Above distance sensor offsets unknown for now
        self.H = H # actuator/state transition covariance
        self.V = V # sensor/output covariance
        self.state = init_state

    def iterate(self, u):
        next_state_mean, next_state_cov = self.next_state_mean_and_cov(u)
        output_mean, output_cov = self.output_mean_and_cov(u)
        self.state = next_state_mean + self.generate_noise(next_state_cov)
        return output_mean + self.generate_noise(output_cov)

    def next_state_mean_and_cov(self, u):
        return np.array([0.0,0,0]), self.H

    def output_mean_and_cov(self, u):
        return np.array([0.0,0,0,0,0]), self.V

    def pwm_to_wheel_angular_velocity(self, pwm):
        return 0 # return array with left and right angular velocity

    def robot_angular_velocity(self, u):
        return (self.d/2)*(self.pwm_to_wheel_angular_velocity(u[1]) - self.pwm_to_wheel_angular_velocity(u[0]))/self.w

    def robot_center_velocity(self, u):
        return ((self.d/2)*self.pwm_to_wheel_angular_velocity(u[0]) + (self.d/2)*self.pwm_to_wheel_angular_velocity(u[1]))/2

    def expected_right_sensor_measurement(self):
        # Following y - y_t = (tan theta)(x - x_t)
        theta = self.state[2]
        xR = 0
        yR = 0
        if theta < 0:
            raise ValueError("ERROR: In-plane velocity angle incorrect")
        elif theta < math.pi/2:
            xR1 = self.xE 
            yR2 = self.yN
        elif theta < math.pi:
            xR1 = self.xW
            yR2tuple = self.yN
        elif theta < 3*math.pi/2:
            xR1 = self.xW
            yR2 = self.yS
        elif theta < 2*math.pi:
            xR1 = self.xE
            yR2 = self.yS
        else:
            raise ValueError("ERROR: In-plane velocity angle incorrect")

        yR1 = math.tan(theta)(xR1 - self/state[0])


        return 

    def expected_front_sensor_measurement(self):
        # Following y - y_t = (cot theta)(x - x_t)
        
        
        return min()

    def expected_magnetometer_measurement(self):
        return 0

    def generate_noise(self, cov):
        return np.random.multivariate_normal(np.zeros(cov.shape[0]), cov)

