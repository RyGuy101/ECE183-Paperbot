% MATLAB controller for Webots
% File:          lab3_controller.m
% Date:
% Description:
% Author:
% Modifications:

% uncomment the next two lines if you want to use
% MATLAB's desktop to interact with the controller:
desktop;
keyboard;

% Get simulation timer step
TIME_STEP = wb_robot_get_basic_time_step;

% Create instances of motor sensors, nodes;
motor_R = wb_robot_get_device('motor_R');
motor_L = wb_robot_get_device('motor_L');

compass = wb_robot_get_device('compass');
gyro = wb_robot_get_device('gyro');
lidar_F = wb_robot_get_device('lidar_F');
lidar_R = wb_robot_get_device('lidar_R');

robot = wb_supervisor_node_get_from_def('robot');
rotation = wb_supervisor_node_get_field(robot,'rotation');

% Enable sensors
wb_compass_enable(compass, TIME_STEP);
wb_gyro_enable(gyro, TIME_STEP);
wb_distance_sensor_enable(lidar_F, TIME_STEP);
wb_distance_sensor_enable(lidar_R, TIME_STEP);

% Make the motors non-position control mode
wb_motor_set_position(motor_R, inf);
wb_motor_set_position(motor_L, inf);

% Set time array
tfinal = 20*1e3;
t = 0:TIME_STEP:tfinal;
tsteps = length(t);

% Set motor input data points
t_data = [0 4 6 10 14 18 20];
t_data = t_data*1e3;
vL_data = [0 2 2 0 0 -1 -2];
vR_data = [0 1 1 0 -1 -1 -2];

vL = interp1(t_data,vL_data,t);
vR = interp1(t_data,vR_data,t);

% Set indexing variable
i = 0;

% Set arrays to store sensor values
compass_tot = zeros(3,tsteps);
gyro_tot = zeros(3,tsteps);
lidarF_tot = zeros(1,tsteps);
lidarR_tot = zeros(1,tsteps);

% Set arrays to store ground truth values
position_tot = zeros(3,tsteps);
anglex_tot = zeros(3,tsteps);
linVel_tot = zeros(3,tsteps);
angVel_tot = zeros(3,tsteps);

% main loop:
% perform simulation steps of TIME_STEP milliseconds
% and leave the loop when Webots signals the termination
% need to call wb_robot_step periodically to communicate to the simulator
while wb_robot_step(TIME_STEP) ~= -1
  % Increase the index value
  i = i + 1;
  
  wb_motor_set_velocity(motor_R,vR(i));
  wb_motor_set_velocity(motor_L,vL(i));
  
  % Read sensor data
  compass_data = wb_compass_get_values(compass);
  gyro_data = wb_gyro_get_values(gyro);
  lidarF_data = wb_distance_sensor_get_value(lidar_F);
  lidarR_data = wb_distance_sensor_get_value(lidar_R);
  
  % Store sensor data in arrays
  compass_tot(1,i) = compass_data(1);
  compass_tot(2,i) = compass_data(2);
  compass_tot(3,i) = compass_data(3);
  
  gyro_tot(1,i) = gyro_data(1);
  gyro_tot(2,i) = gyro_data(2);
  gyro_tot(3,i) = gyro_data(3);
  
  lidarF_tot(i) = lidarF_data;
  lidarR_tot(i) = lidarR_data;
  
  % Get ground truth data
  position = wb_supervisor_node_get_position(robot);
  angle = wb_supervisor_node_get_orientation(robot);
  velocity = wb_supervisor_node_get_velocity(robot);
  
  %Store ground truth data in arrays
  position_tot(1,i) = position(1); %x
  position_tot(2,i) = position(2); %y
  position_tot(3,i) = position(3); %z
  
  anglex_tot(:,i) = angle(:,1);
  
  linVel_tot(1,i) = velocity(1); %x
  linVel_tot(2,i) = velocity(2); %y
  linVel_tot(3,i) = velocity(3); %z
  
  angVel_tot(1,i) = velocity(4); %x
  angVel_tot(2,i) = velocity(5); %y
  angVel_tot(3,i) = velocity(6); %z
  
  % Ending the simulation
  if t(i) >= tfinal
    wb_robot_step(TIME_STEP) = -1;
  end

end

% Close your controller
wb_robot_cleanup();
% cleanup code goes here: write data to files, etc.

% plots motor inputs
figure(1)
plot(t/1000, vL);
hold on
plot(t/1000, vR);
title('Motor Inputs');
xlabel('Time [sec]');
ylabel('Angular Velocity Input [rad/s]');
legend('Left','Right');
hold off

% plots ground truth
figure(2)
subplot(2,2,1);
plot(t/1000, position_tot(1,:));
hold on
plot(t/1000, position_tot(2,:));
plot(t/1000, position_tot(3,:));
title('True Position Output');
xlabel('Time [sec]');
ylabel('Position [m]');
legend('X','Y','Z');
hold off

subplot(2,2,2)
plot(t/1000, linVel_tot(1,:));
hold on
plot(t/1000, linVel_tot(2,:));
plot(t/1000, linVel_tot(3,:));
title('True Linear Velocity Output');
xlabel('Time [sec]');
ylabel('Velocity [m/s]');
legend('V_x','V_y','V_z');
hold off

subplot(2,2,3)
plot(t/1000, anglex_tot(1,:));
hold on
plot(t/1000, anglex_tot(2,:));
plot(t/1000, anglex_tot(3,:));
title('True Angular Position Output');
xlabel('Time [sec]');
ylabel('Angle [rad]');
legend('$\theta_x$','$\theta_y$','$\theta_z$','Interpreter','Latex');
hold off

subplot(2,2,4)
plot(t/1000, angVel_tot(1,:));
hold on
plot(t/1000, angVel_tot(2,:));
plot(t/1000, angVel_tot(3,:));
title('True Angular Velocity Output');
xlabel('Time [sec]');
ylabel('Velocity [rad/s]');
legend('$\omega_x$','$\omega_y$','$\omega_z$', 'Interpreter','Latex');
hold off

% sensor data
figure(3)
% distance
subplot(3,1,1)
plot(t/1000, lidarF_tot*10/4096);
hold on
plot(t/1000, lidarR_tot*10/4096);
title('Lidar Distance Output');
xlabel('Time [sec]');
ylabel('Distance to wall [m]');
legend('Front', 'Right');
hold off

% compass
subplot(3,1,2)
plot(t/1000, compass_tot(1,:));
hold on
plot(t/1000, compass_tot(2,:));
plot(t/1000, compass_tot(3,:));
title('Angular Position Output');
xlabel('Time [sec]');
ylabel('Angle [rad]');
legend('$\theta_x$','$\theta_y$','$\theta_z$','Interpreter','Latex');
hold off

% gyro
subplot(3,1,3)
plot(t/1000, gyro_tot(1,:));
hold on
plot(t/1000, gyro_tot(2,:));
plot(t/1000, gyro_tot(3,:));
title('Angular Velocity Output');
xlabel('Time [sec]');
ylabel('Velocity [rad/s]');
legend('$\omega_x$','$\omega_y$','$\omega_z$','Interpreter','Latex');
hold off