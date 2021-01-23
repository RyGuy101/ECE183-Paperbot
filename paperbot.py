#!/usr/bin/env python3
# Segway Dimensions 
# d = 502mm, w = 530mm
# Paperbot Dimensions
# d = 50mm, w = 90mm
# unit of length is mm

import numpy as np
import math
import time
import keyboard
import pygame, sys
from pygame.locals import *

class Bot:
    def __init__(self, xW=-500, xE=0, yN=0, yS=-500, d=50, w=90, Dr=0, Df=0, max_wheel_speed=2*math.pi*130/60, dt=0.1, init_state=np.array([-250.0, -250.0, 0.0])):
        self.xW = xW # western wall position
        self.xE = xE # eastern wall position
        self.yN = yN # northern wall position
        self.yS = yS # southern wall position
        self.d = d # wheel diameter
        self.w = w # robot width (distance between wheels)
        self.Dr = Dr # right distance sensor offset
        self.Df = Df # front distance sensor offset
        self.max_wheel_speed = max_wheel_speed
        # Above distance sensor offsets unknown for now
        self.dt = dt # sample period
        self.state = init_state

    def iterate(self, u):
        next_state_mean, next_state_cov = self.next_state_mean_and_cov(u)
        output_mean, output_cov = self.output_mean_and_cov(u)
        self.state = self.add_noise(next_state_mean, next_state_cov)

        # constrain theta to [0, 2pi)
        while (self.state[2] < 0):
            self.state[2] += 2*math.pi
        while (self.state[2] >= 2*math.pi):
            self.state[2] -= 2*math.pi

        return self.add_noise(output_mean, output_cov)

    def next_state_mean_and_cov(self, u):
        robot_center_velocity = self.robot_center_velocity(u)
        dx = robot_center_velocity*math.cos(self.state[2])*self.dt
        dy = robot_center_velocity*math.sin(self.state[2])*self.dt
        dtheta = self.robot_angular_velocity(u)*self.dt

        next_state_mean = np.zeros(3)
        next_state_mean[0] = self.state[0] + dx
        next_state_mean[1] = self.state[1] + dy
        next_state_mean[2] = self.state[2] + dtheta

        return next_state_mean, np.diag([(dx**2)*(0.1**2), (dy**2)*(0.1**2), (dtheta**2)*(0.1**2)])

    def output_mean_and_cov(self, u):
        output_mean = np.zeros(5)
        output_mean[0] = self.expected_right_sensor_measurement()
        output_mean[1] = self.expected_front_sensor_measurement()
        output_mean[2] = self.robot_angular_velocity(u)
        output_mean[3:5] = self.expected_magnetometer_measurement()

        var_dr = (25/2)**2
        if output_mean[0] >= 5000:
            var_dr = (100/2)**2
        var_df = (25/2)**2
        if output_mean[1] >= 5000:
            var_df = (100/2)**2
        
        var_theta = ((math.radians(0.005) / math.sqrt(self.dt)) + math.radians(0.04))**2

        var_bx = ((0.0004/math.sqrt(self.dt)) + 0.003)**2
        var_by = ((0.0004/math.sqrt(self.dt)) + 0.003)**2

        return output_mean, np.diag([var_dr, var_df, var_theta, var_bx, var_by])

    def pwm_to_wheel_angular_velocity(self, pwm):
        if abs(pwm) > 0.05625:
            return (self.max_wheel_speed*pwm) * (1 - 0.4*abs(pwm))
        else:
            return 0

    def robot_angular_velocity(self, u):
        return (self.d/2)*(self.pwm_to_wheel_angular_velocity(u[1]) - self.pwm_to_wheel_angular_velocity(u[0]))/self.w

    def robot_center_velocity(self, u):
        return ((self.d/2)*self.pwm_to_wheel_angular_velocity(u[0]) + (self.d/2)*self.pwm_to_wheel_angular_velocity(u[1]))/2

    def expected_right_sensor_measurement(self):
        # Following y - y_t = (tan theta)(x - x_t)
        theta = self.state[2]
        xR1 = 0
        yR2 = 0
        if theta < 0:
            raise ValueError("ERROR: In-plane velocity angle incorrect")
        elif theta < math.pi/2:
            xR1 = self.xE
            yR2 = self.yS
        elif theta < math.pi:
            xR1 = self.xE 
            yR2 = self.yN
        elif theta < (3*math.pi/2):
            xR1 = self.xW
            yR2 = self.yN
        elif theta < (2*math.pi):
            xR1 = self.xW
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
            xF1 = self.xE
            yF2 = self.yN
        elif theta < math.pi:
            xF1 = self.xW 
            yF2 = self.yN
        elif theta < (3*math.pi/2):
            xF1 = self.xW
            yF2 = self.yS
        elif theta < (2*math.pi):
            xF1 = self.xE
            yF2 = self.yS
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
        B_E = 0.4 # between 0.3 and 0.6 Gauss
        return np.array([B_E*math.cos(-self.state[2]), B_E*math.sin(-self.state[2])])

    def add_noise(self, mean, cov):
        return np.random.multivariate_normal(mean, cov)


def rotate_vector(v, radians):
    x, y = v
    xx = x * math.cos(radians) + y * math.sin(radians)
    yy = -x * math.sin(radians) + y * math.cos(radians)
    return np.array([xx, yy])

def main():
    paperbot = Bot()

    pygame.init()
    DISPLAY=pygame.display.set_mode((-paperbot.xW, -paperbot.yS), 0, 32)
    WHITE=(255,255,255)
    BLUE=(0,0,255)
    
    while True:
        u = np.zeros(2)
        if keyboard.is_pressed('up arrow'):
            u[0] = 0.5
            u[1] = 0.5
        elif keyboard.is_pressed('down arrow'):
            u[0] = -0.5
            u[1] = -0.5
        elif keyboard.is_pressed('left arrow'):
            u[0] = -0.5
            u[1] = 0.5
        elif keyboard.is_pressed('right arrow'):
            u[0] = 0.5
            u[1] = -0.5
        
        output = paperbot.iterate(u)

        DISPLAY.fill(WHITE)

        rect_center = np.array([-paperbot.xW + paperbot.state[0], -paperbot.state[1]])

        rect_points = [
            np.array([-paperbot.d/2, -paperbot.w/2]),
            np.array([-paperbot.d/2, +paperbot.w/2]),
            np.array([+paperbot.d/2, +paperbot.w/2]),
            np.array([+paperbot.d/2, -paperbot.w/2])
        ]

        for i in range(len(rect_points)):
            rect_points[i] = rect_center + rotate_vector(rect_points[i], paperbot.state[2])

        pygame.draw.polygon(DISPLAY, BLUE, rect_points)
        for event in pygame.event.get():
            if event.type==QUIT:
                pygame.quit()
                sys.exit()
        pygame.display.update()

        time.sleep(paperbot.dt)

main()
