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

def run_sim(bot, pixels_per_millimeter, inputs=None, real_time=True, name="no_name"):
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

def make_input(points, dt):
    assert(points[0][0] == 0)
    input = [points[0][1]]
    for i in range(1, len(points)):
        point = points[i]
        start_t = len(input)-1
        end_t = int(point[0]//dt)
        slope = (point[1]-input[start_t])/(end_t-start_t)
        for t in range(start_t+1, end_t):
            input.append(slope*(t-start_t) + input[start_t])
        input.append(point[1])
    return input


PAPERBOT_PIXELS_PER_MM = 0.7
SEGWAY_PIXELS_PER_MM = 0.07

def paperbot_interactive():
    run_sim(make_paperbot(), PAPERBOT_PIXELS_PER_MM)

def segway_interactive():
    run_sim(make_segway(), SEGWAY_PIXELS_PER_MM)

def paperbot_figure_eight():
    inputs = []
    inputs += [[0.5, 0.25]]*480
    inputs += [[0.25, 0.5]]*960
    inputs += [[0.5, 0.25]]*960
    inputs += [[0.25, 0.5]]*960
    inputs += [[0.5, 0.25]]*480
    run_sim(make_paperbot(init_state=[-900,-500,math.pi/2,0]), PAPERBOT_PIXELS_PER_MM, inputs)

# SolidWorks tests:
def paperbot_trajectory_1(): # Megan's PaperBot
    input = [
        [0.0, np.array([0.0, 0.0])],
        [0.5, np.array([500.0, 500.0])],
        [1.0, np.array([500.0, 500.0])],
        [2.0, np.array([100.0, 500.0])],
        [2.5, np.array([100.0, 500.0])],
        [3.0, np.array([-300.0, -300.0])],
        [3.5, np.array([-300.0, -300.0])],
        [4.0, np.array([500.0, 0.0])],
        [4.5, np.array([500.0, 0.0])],
        [5.0, np.array([500.0, 500.0])]
    ]                              # x     y      angular displacements
    bot = make_paperbot(init_state=[402.897777478867,300.0,-15,0], init_state_solidworks=True)
    run_sim(bot, PAPERBOT_PIXELS_PER_MM, make_input(input, bot.dt))

def paperbot_trajectory_2(): # Laurens's PaperBot
    input = [
        [0.0, np.array([0.0, 0.0])],
        [0.5, np.array([450.0, 270.0])],
        [1.0, np.array([0.0, 270.0])],
        [1.5, np.array([0.0, -100.0])],
        [2.0, np.array([-100.0, -200.0])],
        [2.5, np.array([450.0, 200.0])],
        [3.0, np.array([450.0, 200.0])]
    ]                              # x     y      angular displacements
    bot = make_paperbot(init_state=[401.190069694331,301.354154393942,-15,0], init_state_solidworks=True)
    run_sim(bot, PAPERBOT_PIXELS_PER_MM, make_input(input, bot.dt))

def paperbot_trajectory_3(): # Cheryl's PaperBot
    input = [
        [0.0, np.array([25.0, 100.0])],
        [1.0, np.array([100.0, 500.0])],
        [2.0, np.array([25.0, 600.0])],
        [3.0, np.array([150.0, 20.0])],
        [4.0, np.array([500.0, 0.0])],
        [5.0, np.array([300.0, 100.0])]
    ]                              # x     y      angular displacements
    bot = make_paperbot(init_state=[497.0,300.0,4.9e-18,0], init_state_solidworks=True)
    run_sim(bot, PAPERBOT_PIXELS_PER_MM, make_input(input, bot.dt))

def segway_trajectory_1(): # Megan's Segway 
    input = [
        [0.0, np.array([0.0, 0.0])],
        [0.5, np.array([0.0, -300.0])],
        [1.0, np.array([0.0, 0.0])],
        [1.5, np.array([300.0, 300.0])],
        [2.0, np.array([300.0, 300.0])],
        [2.5, np.array([100.0, 400.0])],
        [3.0, np.array([300.0, 300.0])],
        [4.0, np.array([400.0, 100.0])],
        [5.0, np.array([400.0, 100.0])]
    ]                            # x      y       angular displacements
    bot = make_segway(init_state=[1500.00,2000.00,-15,0], init_state_solidworks=True, dt=0.02564102564)
    run_sim(bot, SEGWAY_PIXELS_PER_MM, make_input(input, bot.dt))

def segway_trajectory_2(): # Laurens's Segway
    input = [
        [0.0, np.array([0.0, 0.0])],
        [0.5, np.array([450.0, 270.0])],
        [1.0, np.array([0.0, 270.0])],
        [1.5, np.array([0.0, -100.0])],
        [2.0, np.array([-100.0, -200.0])],
        [2.5, np.array([450.0, 200.0])],
        [3.0, np.array([450.0, 200.0])]
    ]                            # x      y       angular displacements
    bot = make_segway(init_state=[1500.00,2000.00,-15,0], init_state_solidworks=True, dt=0.02727272727)
    run_sim(bot, SEGWAY_PIXELS_PER_MM, make_input(input, bot.dt))

def segway_trajectory_3(): # Cheryl's Segway
    input = [
        [0.0, np.array([400.0, 400.0])],
        [0.5, np.array([500.0, 500.0])],
        [1.0, np.array([100.0, 300.0])],
        [1.5, np.array([150.0, 100.0])],
        [2.0, np.array([200.0, 200.0])],
        [2.5, np.array([600.0, 150.0])],
        [3.0, np.array([250.0, 400.0])],
        [3.5, np.array([300.0, 550.0])],
        [4.0, np.array([400.0, 500.0])],
        [4.5, np.array([200.0, 600.0])],
        [5.0, np.array([100.0, 550.0])]
    ]                            # x      y       angular displacements
    bot = make_segway(init_state=[1500.00,2000.00,-15,0], init_state_solidworks=True, dt=0.02403846153)
    run_sim(bot, SEGWAY_PIXELS_PER_MM, make_input(input, bot.dt))

# Megan's Trajectories
# paperbot_trajectory_1()
# segway_trajectory_1()

# Laurens's Trajectories
# paperbot_trajectory_2()
# segway_trajectory_2()

# Cheryl's Trajectories
# paperbot_trajectory_3()

# Webots tests:
def webots_paperbot_1():
    input = [
        [ 0, np.array([0, 0])],
        [ 2, np.array([-4, -4])],
        [ 4, np.array([-4, -4])],
        [ 8, np.array([-1, -4])],
        [10, np.array([-1, -4])],
        [12, np.array([2, 2])],
        [14, np.array([2, 2])],
        [16, np.array([-4, 0])],
        [18, np.array([-4, 0])],
        [20, np.array([-4, -4])]
    ]
    bot = make_paperbot(init_state=[0.25,0.3,0,0], init_state_webots=True, dt=0.002)
    run_sim(bot, PAPERBOT_PIXELS_PER_MM, make_input(input, bot.dt), name='paperbot_1')

def webots_paperbot_2():
    input = [
        [ 0, np.array([0, 0])],
        [ 2, np.array([7.853981634, 4.71238898])],
        [ 4, np.array([0, 4.71238898])],
        [ 6, np.array([0, -1.745329252])],
        [ 8, np.array([-1.745329252, -3.490658504])],
        [10, np.array([7.853981634, 3.490658504])],
        [12, np.array([7.853981634, 3.490658504])],
        [14, np.array([0, 0])]
    ]
    bot = make_paperbot(init_state=[-0.15,-0.1,0,0], init_state_webots=True, dt=0.01)
    run_sim(bot, PAPERBOT_PIXELS_PER_MM, make_input(input, bot.dt), name='paperbot_2')

def webots_paperbot_3():
    input = [
        [ 0, np.array([-0.563667687, -1.745329252])],
        [ 2, np.array([0.745329252, -8.72664626])],
        [ 4, np.array([-0.563667687, -10.47197551])],
        [ 6, np.array([1.617993878, -0.3490658504])],
        [ 8, np.array([7.72664626, 0])],
        [10, np.array([4.235987756, -0.1745329252])],
        [12, np.array([0, 0])]
    ]
    bot = make_paperbot(init_state=[0,0,0,0], init_state_webots=True, dt=0.01)
    run_sim(bot, PAPERBOT_PIXELS_PER_MM, make_input(input, bot.dt), name='paperbot_3')

def webots_paperbot_straight_and_turn():
    input = [
        [ 0, np.array([0, 0])],
        [ 5, np.array([-3, -3])],
        [ 7, np.array([0, -5])],
        [10, np.array([0, -5])],
    ]
    bot = make_paperbot(init_state=[0,0.3,0,0], init_state_webots=True, dt=0.002)
    run_sim(bot, PAPERBOT_PIXELS_PER_MM, make_input(input, bot.dt), name='paperbot_straight_and_turn')

def webots_segway_4():
    input = [
        [ 0, np.array([0, 0])],
        [ 2, np.array([0, 4])],
        [ 4, np.array([0, 0])],
        [ 6, np.array([-4, -4])],
        [ 8, np.array([-4, -4])],
        [10, np.array([-2, -6])],
        [12, np.array([-4, -4])],
        [16, np.array([-6, -2])],
        [20, np.array([-6, -2])]
    ]
    bot = make_segway(init_state=[-1,3,0,0], init_state_webots=True, dt=0.002)
    run_sim(bot, SEGWAY_PIXELS_PER_MM, make_input(input, bot.dt), name='segway_4')

def webots_segway_5():
    input = [
        [ 0, np.array([0, 0])],
        [ 2, np.array([7.853981634, 4.71238898])],
        [ 4, np.array([0, 4.71238898])],
        [ 6, np.array([0, -1.745329252])],
        [ 8, np.array([-1.745329252, -3.490658504])],
        [10, np.array([7.853981634, 3.490658504])],
        [12, np.array([7.853981634, 3.490658504])],
        [14, np.array([0, 0])],
    ]
    bot = make_segway(init_state=[0,0,0,0], init_state_webots=True, dt=0.002)
    run_sim(bot, SEGWAY_PIXELS_PER_MM, make_input(input, bot.dt), name='segway_5')

def webots_segway_6():
    input = [
        [ 0, np.array([3.490658504, 3.490658504])],
        [ 2, np.array([4.36332313, 4.36332313])],
        [ 4, np.array([0.872664626, 2.617993878])],
        [ 6, np.array([1.308996939, 0.872664626])],
        [ 8, np.array([1.745329252, 1.745329252])],
        [10, np.array([5.235987756, 1.308996939])],
        [12, np.array([2.181661565, 2.243994753])],
        [14, np.array([2.617993878, 3.926990817])],
        [16, np.array([3.490658504, 4.36332313])],
        [18, np.array([1.745329252, 5.235987756])],
        [20, np.array([0.872664626, 4.799655443])]
    ]
    bot = make_segway(init_state=[4,4,0,0], init_state_webots=True, dt=0.002)
    run_sim(bot, SEGWAY_PIXELS_PER_MM, make_input(input, bot.dt), name='segway_6')

def webots_segway_straight_and_turn():
    input = [
        [ 0, np.array([0, 0])],
        [ 5, np.array([-3, -3])],
        [ 7, np.array([0, -5])],
        [10, np.array([0, -5])],
    ]
    bot = make_segway(init_state=[0,4,0,0], init_state_webots=True, dt=0.002)
    run_sim(bot, SEGWAY_PIXELS_PER_MM, make_input(input, bot.dt), name='segway_straight_and_turn')

webots_paperbot_1()
webots_paperbot_2()
webots_paperbot_3()
webots_paperbot_straight_and_turn()
webots_segway_4()
webots_segway_5()
webots_segway_6()
webots_segway_straight_and_turn()

# paperbot_interactive()
