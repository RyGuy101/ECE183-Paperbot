import numpy as np

# make input for each time step using linear interpolation between data points
def make_input(points, dt):
    assert(points[0][0] == 0)
    input = [np.array(points[0][1])]
    for i in range(1, len(points)):
        point = points[i]
        start_t = len(input)-1
        end_t = int(point[0]//dt)
        slope = (np.array(point[1])-input[start_t])/(end_t-start_t)
        for t in range(start_t+1, end_t):
            input.append(slope*(t-start_t) + input[start_t])
        input.append(np.array(point[1]))
    return input

def make_input_str(input_str, dt):
    points = []
    for line in input_str.strip().split('\n'):
        line = line.strip().split('\t')
        points.append([float(line[0]), [float(line[1]), float(line[2])]])
    return make_input(points, dt)

# Webots tests:
def webots_paperbot_1():
    name = 'paperbot_1'
    dt = 0.002
    init_state = [0.25,0.3,0,0]
    input = make_input([
        [ 0, [0, 0]],
        [ 2, [-4, -4]],
        [ 4, [-4, -4]],
        [ 8, [-1, -4]],
        [10, [-1, -4]],
        [12, [2, 2]],
        [14, [2, 2]],
        [16, [-4, 0]],
        [18, [-4, 0]],
        [20, [-4, -4]]
    ], dt)
    return name, dt, init_state, input

def webots_paperbot_2():
    name = 'paperbot_2'
    dt = 0.01
    init_state = [-0.15,-0.1,0,0]
    input = make_input([
        [ 0, [0, 0]],
        [ 2, [7.853981634, 4.71238898]],
        [ 4, [0, 4.71238898]],
        [ 6, [0, -1.745329252]],
        [ 8, [-1.745329252, -3.490658504]],
        [10, [7.853981634, 3.490658504]],
        [12, [7.853981634, 3.490658504]]
    ], dt)
    return name, dt, init_state, input

def webots_paperbot_3():
    name = 'paperbot_3'
    dt = 0.01
    init_state = [0,0,0,0]
    input = make_input([
        [ 0, [-0.563667687, -1.745329252]],
        [ 2, [0.745329252, -8.72664626]],
        [ 4, [-0.563667687, -10.47197551]],
        [ 6, [1.617993878, -0.3490658504]],
        [ 8, [7.72664626, 0]],
        [10, [4.235987756, -0.1745329252]]
    ], dt)
    return name, dt, init_state, input

def webots_paperbot_straight_and_turn():
    name = 'paperbot_straight_and_turn'
    dt = 0.002
    init_state = [0,0.3,0,0]
    input = make_input([
        [ 0, [0, 0]],
        [ 5, [-3, -3]],
        [ 7, [0, -5]],
        [10, [0, -5]]
    ], dt)
    return name, dt, init_state, input

def webots_segway_19(): # note: hits wall
    name = 'segway_19'
    dt = 0.002
    init_state = [-1, 3, 0, 0]
    input = make_input_str("""0	0	0
2	0	4
4	0	0
6	-4	-4
8	-4	-4
10	-2	-6
12	-4	-4
16	-6	-2
20	-6	-2""", dt)
    return name, dt, init_state, input

def webots_segway_20():
    name = 'segway_20'
    dt = 0.01
    init_state = [0, 0, 0, 0]
    input = make_input_str("""0	0	0
2	7.853981634	4.71238898
4	0	4.71238898
6	0	-1.745329252
8	-1.745329252	-3.490658504
10	7.853981634	3.490658504
12	7.853981634	3.490658504""", dt)
    return name, dt, init_state, input

def webots_segway_21():
    name = 'segway_21'
    dt = 0.002
    init_state = [4, 4, 0, 0]
    input = make_input_str("""0	3.490658504	3.490658504
2	4.36332313	4.36332313
4	0.872664626	2.617993878
6	1.308996939	0.872664626
8	1.745329252	1.745329252
10	5.235987756	1.308996939
12	2.181661565	2.243994753
14	2.617993878	3.926990817
16	3.490658504	4.36332313
18	1.745329252	5.235987756
20	0.872664626	4.799655443""", dt)
    return name, dt, init_state, input

def webots_segway_22():
    name = 'segway_22'
    dt = 0.002
    init_state = [0, -3, 0, 0]
    input = make_input_str("""0	0	0
4	0	2
8	0	0
12	-2	-2
16	-2	-2
20	-1	-3
24	-2	-2
32	-3	-1
40	-3	-1""", dt)
    return name, dt, init_state, input

def webots_segway_23(): # straight and turn
    name = 'segway_23'
    dt = 0.002
    init_state = [0, 4, 0, 0]
    input = make_input_str("""0	0	0
5	-3	-3
7	0	-5
10	0	-5""", dt)
    return name, dt, init_state, input

def webots_segway_24(): # straight and turn
    name = 'segway_24'
    dt = 0.002
    init_state = [0, 4, 0, 0]
    input = make_input_str("""0	0	0
5	-3	-3
7	-5	0
10	-5	0""", dt)
    return name, dt, init_state, input

def webots_segway_25():
    name = 'segway_25'
    dt = 0.002
    init_state = [0, 0, 0, 0]
    input = make_input_str("""0	0	-3
2	-1	-4
4	-2	-3
6	-3	-2
8	-4	-1
10	-3	0
12	-2	1
14	0	3
16	-2	3
18	-2	1
20	0	0""", dt)
    return name, dt, init_state, input

def webots_segway_26():
    name = 'segway_26'
    dt = 0.002
    init_state = [0, 0, 0, 0]
    input = make_input_str("""0	1	-2
2	1	-1.5
4	2	0
6	2	2
8	3	1
10	3	1
12	2	0
14	0	0
16	-2	-2
18	-3	-3
20	-4	-4""", dt)
    return name, dt, init_state, input

def webots_segway_27():
    name = 'segway_27'
    dt = 0.002
    init_state = [0, 4, 0, 0]
    input = make_input_str("""0	-1	-1
2	-1.5	-1.5
4	-2	-2
6	-2.5	-2.5
8	-2	-2
10	-3	-2
12	-4	-3
14	-2	-4
16	-1	-4
18	1	-3
20	1	-2""", dt)
    return name, dt, init_state, input

def webots_segway_28():
    name = 'segway_28'
    dt = 0.002
    init_state = [0, 0, 0, 0]
    input = make_input_str("""0	1	1
2	1	1
4	1	1
6	1	1
8	1	1
10	2	1
12	2	1
14	2	1
16	3	2
18	3	2
20	3	2""", dt)
    return name, dt, init_state, input

def webots_segway_29():
    name = 'segway_29'
    dt = 0.002
    init_state = [4, 4, 0.785, 0]
    input = make_input_str("""0	-1	-1
2	-1	-1
4	-1	-1
6	-1	-1
8	-1	-1
10	-2	-1
12	-2	-1
14	-2	-1
16	-3	-2
18	-3	-2
20	-3	-2""", dt)
    return name, dt, init_state, input

def webots_segway_30():
    name = 'segway_30'
    dt = 0.002
    init_state = [-4, 4, 5.498, 0]
    input = make_input_str("""0	-1	-1
2	-1	-1
4	-1	-1
6	-1	-1
8	-1	-1
10	-2	-1
12	-2	-1
14	-2	-1
16	-3	-2
18	-3	-2
20	-3	-2""", dt)
    return name, dt, init_state, input

def webots_segway_31():
    name = 'segway_31'
    dt = 0.002
    init_state = [0, 3, 0, 0]
    input = make_input_str("""0	-1	-3
2	-2	-3
4	-3	-3
6	-3	-3
8	-1	-1
10	0	0
12	1	1
14	1	1
16	2	2
18	2	3
20	3	3""", dt)
    return name, dt, init_state, input

def webots_segway_32():
    name = 'segway_32'
    dt = 0.002
    init_state = [3, -3, 2.356, 0]
    input = make_input_str("""0	-1	-3
2	-2	-3
4	-3	-3
6	-3	-3
8	-1	-1
10	0	0
12	1	1
14	1	1
16	2	2
18	2	3
20	3	3""", dt)
    return name, dt, init_state, input

def webots_segway_33():
    name = 'segway_33'
    dt = 0.002
    init_state = [-3, -3, 3.927, 0]
    input = make_input_str("""0	-1	-3
2	-2	-3
4	-3	-3
6	-3	-3
8	-1	-1
10	0	0
12	1	1
14	1	1
16	2	2
18	2	3
20	3	3""", dt)
    return name, dt, init_state, input

def webots_segway_34():
    name = 'segway_34'
    dt = 0.002
    init_state = [-3, 0, 0, 0]
    input = make_input_str("""0	-4	-1
2	-4	-1
4	-4	-1
6	-4	-1
8	-4	-2
10	-4	-2
12	-3	-3
14	-2	-3
16	-2	-3
18	-1	-3
20	-1	-3""", dt)
    return name, dt, init_state, input

def webots_segway_35():
    name = 'segway_35'
    dt = 0.002
    init_state = [3, 0, 0, 0]
    input = make_input_str("""0	-1	-4
2	-1	-4
4	-1	-4
6	-1	-4
8	-2	-4
10	-2	-4
12	-3	-3
14	-3	-2
16	-3	-2
18	-3	-1
20	-3	-1""", dt)
    return name, dt, init_state, input

def webots_segway_36():
    name = 'segway_36'
    dt = 0.002
    init_state = [0, 0, 0, 0]
    input = make_input_str("""0	0	0
4	2	1
6	2	1
10	0	0
14	0	-1
18	-1	-1
20	-2	-2""", dt)
    return name, dt, init_state, input

# Archived Code:

# def paperbot_figure_eight():
#     inputs = []
#     inputs += [[0.5, 0.25]]*480
#     inputs += [[0.25, 0.5]]*960
#     inputs += [[0.5, 0.25]]*960
#     inputs += [[0.25, 0.5]]*960
#     inputs += [[0.5, 0.25]]*480
#     run_sim(make_paperbot(init_state=[-900,-500,math.pi/2,0]), PAPERBOT_PIXELS_PER_MM, inputs)

# # SolidWorks tests:
# def paperbot_trajectory_1(): # Megan's PaperBot
#     input = [
#         [0.0, np.array([0.0, 0.0])],
#         [0.5, np.array([500.0, 500.0])],
#         [1.0, np.array([500.0, 500.0])],
#         [2.0, np.array([100.0, 500.0])],
#         [2.5, np.array([100.0, 500.0])],
#         [3.0, np.array([-300.0, -300.0])],
#         [3.5, np.array([-300.0, -300.0])],
#         [4.0, np.array([500.0, 0.0])],
#         [4.5, np.array([500.0, 0.0])],
#         [5.0, np.array([500.0, 500.0])]
#     ]                              # x     y      angular displacements
#     bot = make_paperbot(init_state=[402.897777478867,300.0,-15,0], init_state_solidworks=True)
#     run_sim(bot, PAPERBOT_PIXELS_PER_MM, make_input(input, bot.dt))

# def paperbot_trajectory_2(): # Laurens's PaperBot
#     input = [
#         [0.0, np.array([0.0, 0.0])],
#         [0.5, np.array([450.0, 270.0])],
#         [1.0, np.array([0.0, 270.0])],
#         [1.5, np.array([0.0, -100.0])],
#         [2.0, np.array([-100.0, -200.0])],
#         [2.5, np.array([450.0, 200.0])],
#         [3.0, np.array([450.0, 200.0])]
#     ]                              # x     y      angular displacements
#     bot = make_paperbot(init_state=[401.190069694331,301.354154393942,-15,0], init_state_solidworks=True)
#     run_sim(bot, PAPERBOT_PIXELS_PER_MM, make_input(input, bot.dt))

# def paperbot_trajectory_3(): # Cheryl's PaperBot
#     input = [
#         [0.0, np.array([25.0, 100.0])],
#         [1.0, np.array([100.0, 500.0])],
#         [2.0, np.array([25.0, 600.0])],
#         [3.0, np.array([150.0, 20.0])],
#         [4.0, np.array([500.0, 0.0])],
#         [5.0, np.array([300.0, 100.0])]
#     ]                              # x     y      angular displacements
#     bot = make_paperbot(init_state=[497.0,300.0,4.9e-18,0], init_state_solidworks=True)
#     run_sim(bot, PAPERBOT_PIXELS_PER_MM, make_input(input, bot.dt))

# def segway_trajectory_1(): # Megan's Segway 
#     input = [
#         [0.0, np.array([0.0, 0.0])],
#         [0.5, np.array([0.0, -300.0])],
#         [1.0, np.array([0.0, 0.0])],
#         [1.5, np.array([300.0, 300.0])],
#         [2.0, np.array([300.0, 300.0])],
#         [2.5, np.array([100.0, 400.0])],
#         [3.0, np.array([300.0, 300.0])],
#         [4.0, np.array([400.0, 100.0])],
#         [5.0, np.array([400.0, 100.0])]
#     ]                            # x      y       angular displacements
#     bot = make_segway(init_state=[1500.00,2000.00,-15,0], init_state_solidworks=True, dt=0.02564102564)
#     run_sim(bot, SEGWAY_PIXELS_PER_MM, make_input(input, bot.dt))

# def segway_trajectory_2(): # Laurens's Segway
#     input = [
#         [0.0, np.array([0.0, 0.0])],
#         [0.5, np.array([450.0, 270.0])],
#         [1.0, np.array([0.0, 270.0])],
#         [1.5, np.array([0.0, -100.0])],
#         [2.0, np.array([-100.0, -200.0])],
#         [2.5, np.array([450.0, 200.0])],
#         [3.0, np.array([450.0, 200.0])]
#     ]                            # x      y       angular displacements
#     bot = make_segway(init_state=[1500.00,2000.00,-15,0], init_state_solidworks=True, dt=0.02727272727)
#     run_sim(bot, SEGWAY_PIXELS_PER_MM, make_input(input, bot.dt))

# def segway_trajectory_3(): # Cheryl's Segway
#     input = [
#         [0.0, np.array([400.0, 400.0])],
#         [0.5, np.array([500.0, 500.0])],
#         [1.0, np.array([100.0, 300.0])],
#         [1.5, np.array([150.0, 100.0])],
#         [2.0, np.array([200.0, 200.0])],
#         [2.5, np.array([600.0, 150.0])],
#         [3.0, np.array([250.0, 400.0])],
#         [3.5, np.array([300.0, 550.0])],
#         [4.0, np.array([400.0, 500.0])],
#         [4.5, np.array([200.0, 600.0])],
#         [5.0, np.array([100.0, 550.0])]
#     ]                            # x      y       angular displacements
#     bot = make_segway(init_state=[1500.00,2000.00,-15,0], init_state_solidworks=True, dt=0.02403846153)
#     run_sim(bot, SEGWAY_PIXELS_PER_MM, make_input(input, bot.dt))

# # Megan's Trajectories
# # paperbot_trajectory_1()
# # segway_trajectory_1()

# # Laurens's Trajectories
# # paperbot_trajectory_2()
# # segway_trajectory_2()

# # Cheryl's Trajectories
# # paperbot_trajectory_3()
# # segway_trajectory_3()
