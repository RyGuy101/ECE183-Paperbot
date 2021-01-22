#!/usr/bin/env python3
# Segway Dimensions 
# d = 502mm, w = 530mm
# Paperbot Dimensions
# d = 50mm, w = 90mm
# unit of length is mm

import numpy as np
import math

class Bot:
    def __init__(self, xW, xE, yN, yS, d=50, w=90, Dr=0, Df=0, H=np.zeros([3,3]), V=np.zeros([3,3]), dt=0.1, init_state=np.array([-200.0, -200.0, 0.0])):
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
        self.dt = dt # sample period
        self.state = init_state

    def iterate(self, u):
        next_state_mean, next_state_cov = self.next_state_mean_and_cov(u)
        output_mean, output_cov = self.output_mean_and_cov(u)
        self.state = next_state_mean + self.generate_noise(next_state_cov)
        return output_mean + self.generate_noise(output_cov)

    def next_state_mean_and_cov(self, u):
        next_state_mean = np.zeros(3)
        next_state_mean[2] = self.state[2] + self.robot_angular_velocity(u)*self.dt
        robot_center_velocity = self.robot_center_velocity(u)
        next_state_mean[0] = self.state[0] + robot_center_velocity*math.cos(self.state[2])*self.dt
        next_state_mean[1] = self.state[1] + robot_center_velocity*math.sin(self.state[2])*self.dt
        return next_state_mean, self.H

    def output_mean_and_cov(self, u):
        return np.array([0.0,0,0,0,0]), self.V

    def pwm_to_wheel_angular_velocity(self, pwm):
        return 0 # return array with left and right angular velocity

    def robot_angular_velocity(self, u):
        return (self.d/2)*(self.pwm_to_wheel_angular_velocity(u[1])[1] - self.pwm_to_wheel_angular_velocity(u[0])[0])/self.w

    def robot_center_velocity(self, u):
        return ((self.d/2)*self.pwm_to_wheel_angular_velocity(u[0])[0] + (self.d/2)*self.pwm_to_wheel_angular_velocity(u[1])[1])/2

    def expected_right_sensor_measurement(self):
        # Following y - y_t = (tan theta)(x - x_t)
        theta = self.state[2]
        xR1 = 0
        yR2 = 0
        if theta < 0:
            raise ValueError("ERROR: In-plane velocity angle incorrect")
        elif theta < math.pi/2:
            xR1 = self.xE 
            yR2 = self.yN
        elif theta < math.pi:
            xR1 = self.xW
            yR2 = self.yN
        elif theta < (3*math.pi/2):
            xR1 = self.xW
            yR2 = self.yS
        elif theta < (2*math.pi):
            xR1 = self.xE
            yR2 = self.yS
        else:
            raise ValueError("ERROR: In-plane velocity angle incorrect")
        
        yR1 = math.tan(theta)*(xR1 - self.state[0]) + self.state[1]
        cot_theta = math.tan(math.pi/2)
        if not math.tan(theta) == 0:
            cot_theta = 1/math.tan(theta)
        xR2 = (cot_theta)*(yR2 - self.state[1]) + self.state[0]

        # E or W wall intersection
        ew_wall_dist = math.sqrt(np.square(xR1 - self.state[0]) + np.square(yR1 - self.state[1]))
        # N or S wall intersection
        nw_wall_dist = math.sqrt(np.square(xR2 - self.state[0]) + np.square(yR2 - self.state[1]))
        return min(ew_wall_dist, nw_wall_dist) - self.Dr
    
    def expected_front_sensor_measurement(self):
        # Following y - y_t = (cot theta)(x - x_t)
        theta = self.state[2]
        xF1 = 0
        yF2 = 0
        if theta < 0:
            raise ValueError("ERROR: In-plane velocity angle incorrect")
        elif theta < math.pi/2:
            xF1 = self.xW 
            yF2 = self.yN
        elif theta < math.pi:
            xF1 = self.xW
            yF2 = self.yS
        elif theta < (3*math.pi/2):
            xF1 = self.xE
            yF2 = self.yS
        elif theta < (2*math.pi):
            xF1 = self.xE
            yF2 = self.yN
        else:
            raise ValueError("ERROR: In-plane velocity angle incorrect")
        
        xF2 = -math.tan(theta)*(yF2 - self.state[1]) + self.state[0]
        cot_theta = math.tan(math.pi/2)
        if not math.tan(theta) == 0:
            cot_theta = 1/math.tan(theta)
        yF1 = -(cot_theta)*(xF1 - self.state[0]) + self.state[1]

        # E or W wall intersection
        ew_wall_dist = math.sqrt(np.square(xF1 - self.state[0]) + np.square(yF1 -self.state[1]))
        # N or S wall intersection
        nw_wall_dist = math.sqrt(np.square(xF2 - self.state[0]) + np.square(yF2 -self.state[1]))
        return min(ew_wall_dist, nw_wall_dist) - self.Df

    def expected_magnetometer_measurement(self):
        return 0

    def generate_noise(self, cov):
        return np.random.multivariate_normal(np.zeros(cov.shape[0]), cov)

