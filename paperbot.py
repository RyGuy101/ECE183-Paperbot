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

class Bot:
    def __init__(self, xW, yS, d, w, Dr, Df, init_state, xE=0, yN=0, max_wheel_speed=2*math.pi*130/60, dt=1/125, init_state_solidworks=False):
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

        if init_state_solidworks: 
            self.state = np.array([init_state[0]+xW, init_state[1]+yS, np.deg2rad(init_state[2])+math.pi/2], dtype=np.float64)
        else:
            self.state = np.array(init_state, dtype=np.float64)
        
        # constrain theta to [0, 2pi)
        while (self.state[2] < 0):
            self.state[2] += 2*math.pi
        while (self.state[2] >= 2*math.pi):
            self.state[2] -= 2*math.pi


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

        # Approximate the robot as a circle with diameter w for collisions
        if next_state_mean[0] < self.xW + self.w/2:
            next_state_mean[0] = self.xW + self.w/2
        elif next_state_mean[0] > self.xE - self.w/2:
            next_state_mean[0] = self.xE - self.w/2

        if next_state_mean[1] < self.yS + self.w/2:
            next_state_mean[1] = self.yS + self.w/2
        elif next_state_mean[1] > self.yN - self.w/2:
            next_state_mean[1] = self.yN - self.w/2

        percent_noise = 0 # no noise to compare with SolidWorks
        return next_state_mean, np.diag([(dx**2)*(percent_noise**2), (dy**2)*(percent_noise**2), (dtheta**2)*(percent_noise**2)])

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
        return np.deg2rad(pwm) # assume wheel speed [radians/s] = PWM [deg/s] to compare with SolidWorks
        # if abs(pwm) > 0.05625:
        #     return (self.max_wheel_speed*pwm) * (1 - 0.4*abs(pwm))
        # else:
        #     return 0

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

def run_sim(bot, pixels_per_millimeter, inputs=None, real_time=True):
    ppm = pixels_per_millimeter
    pygame.init()
    DISPLAY=pygame.display.set_mode(np.rint(np.array([-bot.xW, -bot.yS])*ppm).astype(int), 0, 32)
    WHITE=(255,255,255)
    BLUE=(0,0,255)

    DISPLAY.fill(WHITE)
    pygame.display.update()
    time.sleep(3)

    print("i", "time", "PWM_l", "PWM_r", "x", "y", "theta", sep=",")

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
        # print(t, "{:.2f}".format(t*bot.dt), u[0],u[1],bot.state[0],bot.state[1],bot.state[2], sep=",")
        # SoldWorks format:
        print(t+1, t*bot.dt, np.rad2deg(u[0]), np.rad2deg(u[1]), bot.state[0]-bot.xW, bot.state[1]-bot.yS, np.rad2deg(bot.state[2]-math.pi/2), sep=",")
        output = bot.iterate(u)

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


def make_paperbot(init_state=[-500,-500,0], **kwargs):
    # TODO distance sensor offsets Dr and Df unknown for now
    return Bot(xW=-1000, yS=-1000, d=50, w=90, Dr=0, Df=0, init_state=init_state, **kwargs)

def make_segway(init_state=[-5000,-5000,0], **kwargs):
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
    run_sim(make_paperbot(init_state=[-900,-500,math.pi/2]), PAPERBOT_PIXELS_PER_MM, inputs)

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
    bot = make_paperbot(init_state=[402.897777478867,300.0,-15], init_state_solidworks=True)
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
    bot = make_paperbot(init_state=[401.190069694331,301.354154393942,-15], init_state_solidworks=True)
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
    bot = make_paperbot(init_state=[497.0,300.0,4.9e-18], init_state_solidworks=True)
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
    bot = make_segway(init_state=[1500.00,2000.00,-15], init_state_solidworks=True, dt=0.02564102564)
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
    bot = make_segway(init_state=[1500.00,2000.00,-15], init_state_solidworks=True, dt=0.02727272727)
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
    bot = make_segway(init_state=[1500.00,2000.00,-15], init_state_solidworks=True, dt=0.02403846153)
    run_sim(bot, SEGWAY_PIXELS_PER_MM, make_input(input, bot.dt))

# Megan's Trajectories
# paperbot_trajectory_1()
# segway_trajectory_1()

# Laurens's Trajectories
# paperbot_trajectory_2()
# segway_trajectory_2()

# Cheryl's Trajectories
# paperbot_trajectory_3()
segway_trajectory_3()
