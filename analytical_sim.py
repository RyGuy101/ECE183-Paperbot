#!/usr/bin/env python3
# Segway Dimensions 
# d = 502mm, w = 530mm
# Paperbot Dimensions
# d = 50mm, w = 90mm
# unit of length is mm

# Questions for TA
# - Any hints for what actuator nonlinearity / noise should look like? Is our model ok?
# - Should we have intentional differences between segway & paperbot in simulation, such as actuator noise?
# - When we analyze sensors, should we simulate any noise that is not strictly independent at each time step (like bias)
# - btw in the last assignment I forgot to divide RPM by 60!

import numpy as np
import math
import time
import keyboard
import os
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"
import pygame, sys
from pygame.locals import *
import matplotlib
from scipy.io import savemat
import trajectories

class Bot:
    def __init__(self, xW, yS, d, w, Dr, Df, init_state, xE=0, yN=0, max_wheel_speed=2*math.pi*130/60, dt=1/125, init_state_solidworks=False, init_state_webots=False):
        self.xW = xW # western wall position
        self.xE = xE # eastern wall position
        self.yN = yN # northern wall position
        self.yS = yS # southern wall position
        self.d = d # wheel diameter
        self.w = w # robot width (distance between wheels)
        self.Dr = Dr # right distance sensor offset
        self.Df = Df # front distance sensor offset
        self.max_wheel_speed = max_wheel_speed
        self.dt = dt # sample period
        self.solidworks = init_state_solidworks
        self.webots = init_state_webots

        if init_state_solidworks:
            self.state = np.array([init_state[0]+xW, init_state[1]+yS, np.deg2rad(init_state[2])+math.pi/2, np.deg2rad(init_state[3])], dtype=np.float64)
        elif init_state_webots:
            self.state = np.array([init_state[0]*1000+xW/2, -init_state[1]*1000+yS/2, init_state[2]+math.pi/2, init_state[3]], dtype=np.float64)
        else:
            self.state = np.array(init_state, dtype=np.float64)
        
        # constrain theta to [0, 2pi)
        while (self.state[2] < 0):
            self.state[2] += 2*math.pi
        while (self.state[2] >= 2*math.pi):
            self.state[2] -= 2*math.pi

    def get_observation(self):
        output_mean, output_cov = self.output_mean_and_cov()
        return self.add_noise(output_mean, output_cov)

    def iterate_state(self, u):
        next_state_mean, next_state_cov = self.next_state_mean_and_cov(u)
        self.state = self.add_noise(next_state_mean, next_state_cov)

        # constrain theta to [0, 2pi)
        while (self.state[2] < 0):
            self.state[2] += 2*math.pi
        while (self.state[2] >= 2*math.pi):
            self.state[2] -= 2*math.pi

    def next_state_mean_and_cov(self, u):
        expected_robot_center_velocity = self.expected_robot_center_velocity(u)
        expected_dx = expected_robot_center_velocity*math.cos(self.state[2])*self.dt
        expected_dy = expected_robot_center_velocity*math.sin(self.state[2])*self.dt
        expected_robot_angular_velocity = self.expected_robot_angular_velocity(u)

        next_state_mean = np.zeros(4)
        next_state_mean[0] = self.state[0] + expected_dx
        next_state_mean[1] = self.state[1] + expected_dy
        next_state_mean[2] = self.state[2] + self.state[3]*self.dt # already noise in state[3], so no noise added here - angular position is directly integrated from angular velocity
        next_state_mean[3] = expected_robot_angular_velocity

        # Approximate the robot as a circle with diameter w for collisions
        if next_state_mean[0] < self.xW + self.w/2:
            next_state_mean[0] = self.xW + self.w/2
        elif next_state_mean[0] > self.xE - self.w/2:
            next_state_mean[0] = self.xE - self.w/2

        if next_state_mean[1] < self.yS + self.w/2:
            next_state_mean[1] = self.yS + self.w/2
        elif next_state_mean[1] > self.yN - self.w/2:
            next_state_mean[1] = self.yN - self.w/2

        percent_noise = 0 # no noise to compare with Webots (since this noise is not based on physics)
        return next_state_mean, np.diag([(expected_dx**2)*(percent_noise**2), (expected_dy**2)*(percent_noise**2), 0, (expected_robot_angular_velocity**2)*(percent_noise**2)])

    def output_mean_and_cov(self):
        output_mean = np.zeros(5)
        output_mean[0] = self.expected_right_sensor_measurement()
        output_mean[1] = self.expected_front_sensor_measurement()
        output_mean[2] = self.state[3]
        output_mean[3:5] = self.expected_magnetometer_measurement()

        # var_dr = (25/2)**2
        # if output_mean[0] >= 5000:
        #     var_dr = (100/2)**2
        # var_df = (25/2)**2
        # if output_mean[1] >= 5000:
        #     var_df = (100/2)**2
        
        # var_gyro = ((math.radians(0.005) / math.sqrt(self.dt)) + math.radians(0.04))**2

        # var_bx = ((0.0004/math.sqrt(self.dt)) + 0.003)**2
        # var_by = ((0.0004/math.sqrt(self.dt)) + 0.003)**2

        # Use Webots noise model:
        var_dr = (0.05*output_mean[0])**2
        var_df = (0.05*output_mean[1])**2
        var_gyro = (0.05*output_mean[2])**2
        var_bx = (0.05*output_mean[3])**2
        var_by = (0.05*output_mean[4])**2

        return output_mean, np.diag([var_dr, var_df, var_gyro, var_bx, var_by])

    def pwm_to_wheel_angular_velocity(self, pwm):
        if self.solidworks:
            pwm = np.deg2rad(pwm)
        elif self.webots:
            pwm = -pwm

        return pwm # for now, assume wheel speed = PWM

        # if abs(pwm) > 0.05625:
        #     return (self.max_wheel_speed*pwm) * (1 - 0.4*abs(pwm))
        # else:
        #     return 0

    def expected_robot_angular_velocity(self, u):
        return (self.d/2)*(self.pwm_to_wheel_angular_velocity(u[1]) - self.pwm_to_wheel_angular_velocity(u[0]))/self.w

    def expected_robot_center_velocity(self, u):
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
        
        xR2 = -math.tan(theta)*(yR2 - self.state[1]) + self.state[0]
        cot_theta = math.tan(math.pi/2)
        if not math.tan(theta) == 0:
            cot_theta = 1/math.tan(theta)
        yR1 = -(cot_theta)*(xR1 - self.state[0]) + self.state[1]

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
        
        yF1 = math.tan(theta)*(xF1 - self.state[0]) + self.state[1]
        cot_theta = math.tan(math.pi/2)
        if not math.tan(theta) == 0:
            cot_theta = 1/math.tan(theta)
        xF2 = (cot_theta)*(yF2 - self.state[1]) + self.state[0]

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

def run_sim(bot, pixels_per_millimeter, inputs=None, name="no_name", real_time=True):
    ppm = pixels_per_millimeter
    pygame.init()
    DISPLAY=pygame.display.set_mode(np.rint(np.array([-bot.xW, -bot.yS])*ppm).astype(int), 0, 32)
    WHITE=(255,255,255)
    BLUE=(0,0,255)

    DISPLAY.fill(WHITE)
    pygame.display.update()
    time.sleep(3)

    # print("i", "time", "PWM_l", "PWM_r", "x", "y", "theta", sep=",") SolidWorks format

    matlab_vars = {
        "py_t": [],
        "py_vL": [],
        "py_vR": [],
        "py_position_tot": [[], [], []],
        "py_anglex_tot": [[], [], []],
        "py_linVel_tot": [[], [], []],
        "py_angVel_tot": [[], [], []],
        "py_compass_tot": [[], [], []],
        "py_gyro_tot": [[], [], []],
        "py_lidarF_tot": [],
        "py_lidarR_tot": []
    }

    t = 0
    next_time = time.time()
    while inputs is None or t < len(inputs):
        u = np.zeros(2)

        if inputs is None:
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
        else:
            u = inputs[t]
        
        # Robot Stuff
        observation = bot.get_observation()

        # Webots format:
        matlab_vars['py_t'].append(t*bot.dt*1000)

        matlab_vars['py_vL'].append(u[0])
        matlab_vars['py_vR'].append(u[1])

        matlab_vars['py_position_tot'][0].append((bot.state[0]-bot.xW/2)/1000)
        matlab_vars['py_position_tot'][1].append(0) # ignore y position
        matlab_vars['py_position_tot'][2].append((bot.yS/2-bot.state[1])/1000)

        matlab_vars['py_anglex_tot'][0].append(math.sin(bot.state[2]))
        matlab_vars['py_anglex_tot'][1].append(0)
        matlab_vars['py_anglex_tot'][2].append(-math.cos(bot.state[2]))

        prev_pos_0 = matlab_vars['py_position_tot'][0][-1]
        prev_pos_1 = matlab_vars['py_position_tot'][1][-1]
        prev_pos_2 = matlab_vars['py_position_tot'][2][-1]
        if len(matlab_vars['py_position_tot'][0]) >= 2:
            prev_pos_0 = matlab_vars['py_position_tot'][0][-2]
            prev_pos_1 = matlab_vars['py_position_tot'][1][-2]
            prev_pos_2 = matlab_vars['py_position_tot'][2][-2]
        matlab_vars['py_linVel_tot'][0].append((matlab_vars['py_position_tot'][0][-1] - prev_pos_0) / bot.dt)
        matlab_vars['py_linVel_tot'][1].append((matlab_vars['py_position_tot'][1][-1] - prev_pos_1) / bot.dt)
        matlab_vars['py_linVel_tot'][2].append((matlab_vars['py_position_tot'][2][-1] - prev_pos_2) / bot.dt)

        matlab_vars['py_angVel_tot'][0].append(0)
        matlab_vars['py_angVel_tot'][1].append(bot.state[3])
        matlab_vars['py_angVel_tot'][2].append(0)

        matlab_vars['py_compass_tot'][0].append(-observation[4]/0.4)
        matlab_vars['py_compass_tot'][1].append(0)
        matlab_vars['py_compass_tot'][2].append(-observation[3]/0.4)

        matlab_vars['py_gyro_tot'][0].append(0)
        matlab_vars['py_gyro_tot'][1].append(observation[2])
        matlab_vars['py_gyro_tot'][2].append(0)
        
        matlab_vars['py_lidarF_tot'].append(observation[1]/1000/10*4096)
        matlab_vars['py_lidarR_tot'].append(observation[0]/1000/10*4096)

        # SoldWorks format:
        # print(t+1, t*bot.dt, np.rad2deg(u[0]), np.rad2deg(u[1]), bot.state[0]-bot.xW, bot.state[1]-bot.yS, np.rad2deg(bot.state[2]-math.pi/2), sep=",")
        
        bot.iterate_state(u)

        if t % 3 == 0:
            DISPLAY.fill(WHITE)

            rect_center = np.array([-bot.xW + bot.state[0], -bot.state[1]])

            rect_points = [
                np.array([-bot.d/2, -bot.w/2]),
                np.array([-bot.d/2, +bot.w/2]),
                np.array([+bot.d/2, +bot.w/2]),
                np.array([+bot.d/2, -bot.w/2])
            ]

            for i in range(len(rect_points)):
                rect_points[i] = np.rint((rect_center + rotate_vector(rect_points[i], bot.state[2]))*ppm).astype(int)

            pygame.draw.polygon(DISPLAY, BLUE, rect_points)
            for event in pygame.event.get():
                if event.type==QUIT:
                    pygame.quit()
                    sys.exit()
            pygame.display.update()

        t += 1

        if real_time:
            next_time += bot.dt
            delay = next_time - time.time()
            if (delay > 0):
                time.sleep(delay)

    savemat(name+'.mat', matlab_vars)


def make_paperbot(init_state=[-500,-500,0,0], **kwargs):
    # TODO distance sensor offsets Dr and Df unknown for now
    return Bot(xW=-1000, yS=-1000, d=50, w=90, Dr=0, Df=0, init_state=init_state, **kwargs)

def make_segway(init_state=[-5000,-5000,0,0], **kwargs):
    # TODO distance sensor offsets Dr and Df unknown for now
    return Bot(xW=-10000, yS=-10000, d=502, w=530, Dr=0, Df=0, init_state=init_state, **kwargs)

PAPERBOT_PIXELS_PER_MM = 0.7
SEGWAY_PIXELS_PER_MM = 0.07

def paperbot_interactive():
    run_sim(make_paperbot(), PAPERBOT_PIXELS_PER_MM)

def segway_interactive():
    run_sim(make_segway(), SEGWAY_PIXELS_PER_MM)

def webots_paperbot_sim(name, dt, init_state, input):
    bot = make_paperbot(init_state=init_state, init_state_webots=True, dt=dt)
    run_sim(bot, PAPERBOT_PIXELS_PER_MM, input, name)

def webots_segway_sim(name, dt, init_state, input):
    bot = make_segway(init_state=init_state, init_state_webots=True, dt=dt)
    run_sim(bot, SEGWAY_PIXELS_PER_MM, input, name)

# webots_paperbot_sim(*trajectories.webots_paperbot_1())
# webots_paperbot_sim(*trajectories.webots_paperbot_2())
# webots_paperbot_sim(*trajectories.webots_paperbot_3())
# webots_paperbot_sim(*trajectories.webots_paperbot_4())
# webots_paperbot_sim(*trajectories.webots_paperbot_5())
# webots_paperbot_sim(*trajectories.webots_paperbot_6())
# webots_paperbot_sim(*trajectories.webots_paperbot_7())
# webots_paperbot_sim(*trajectories.webots_paperbot_8())
# webots_paperbot_sim(*trajectories.webots_paperbot_9())
# webots_paperbot_sim(*trajectories.webots_paperbot_10())
# webots_paperbot_sim(*trajectories.webots_paperbot_11())
# webots_paperbot_sim(*trajectories.webots_paperbot_12())
# webots_paperbot_sim(*trajectories.webots_paperbot_13())
# webots_paperbot_sim(*trajectories.webots_paperbot_14())
# webots_paperbot_sim(*trajectories.webots_paperbot_15())
# webots_paperbot_sim(*trajectories.webots_paperbot_16())
# webots_paperbot_sim(*trajectories.webots_paperbot_17())
# webots_paperbot_sim(*trajectories.webots_paperbot_18())

# webots_segway_sim(*trajectories.webots_segway_19())
# webots_segway_sim(*trajectories.webots_segway_20())
# webots_segway_sim(*trajectories.webots_segway_21())
# webots_segway_sim(*trajectories.webots_segway_22())
# webots_segway_sim(*trajectories.webots_segway_23())
# webots_segway_sim(*trajectories.webots_segway_24())
# webots_segway_sim(*trajectories.webots_segway_25())
webots_segway_sim(*trajectories.webots_segway_26())
# webots_segway_sim(*trajectories.webots_segway_27())
# webots_segway_sim(*trajectories.webots_segway_28())
# webots_segway_sim(*trajectories.webots_segway_29())
# webots_segway_sim(*trajectories.webots_segway_30())
# webots_segway_sim(*trajectories.webots_segway_31())
# webots_segway_sim(*trajectories.webots_segway_32())
# webots_segway_sim(*trajectories.webots_segway_33())
# webots_segway_sim(*trajectories.webots_segway_34())
# webots_segway_sim(*trajectories.webots_segway_35())
# webots_segway_sim(*trajectories.webots_segway_36())


# paperbot_interactive()
