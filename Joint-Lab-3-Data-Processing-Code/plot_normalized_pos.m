close all;
clearvars;

for i=20:21 %plots 2+3 variable names different
    fname_fig = 'C:\Users\mjwil\Desktop\Current Classes\mae 162D\Labs\Lab 3\Code\Plots';
    
    if i <= 18
        load(['paperbot_',num2str(i),'.mat']);
        load(['paperbot_data_path',num2str(i),'.mat']); % change to relevant .mat file
    else
        load(['segway_',num2str(i),'.mat']);
        load(['segway_data_path',num2str(i),'.mat']); % change to relevant .mat file
    end

% load('segway_21.mat')
% load('segway_data_path21.mat');
% t = py_t;
    
    %webots_data = load(['paperbot_data_path',i,'.mat']);
    %Change Variables
%     position_tot = webots_data.true_position;
%     linVel_tot = webots_data.true_lin_vel;
%     anglex_tot = webots_data.true_angels;
%     
%     angVel_tot = webots_data.true_ang_vel;
%     lidarF_tot = webots_data.distanceF_values;
%     lidarR_tot = webots_data.distanceR_values;
%     ompass_tot = webots_data.compass_values;
%     gyro_tot = webots_data.gyro_values;
    if( i == 2)
                %Change Variables
                position_tot = true_position(:,1:1200);
                linVel_tot = true_lin_vel(:,1:1200);
                angVel_tot = true_ang_vel(:,1:1200);
                anglex_tot = true_angels(:,1:1200);
                t = time(:,1:1200);
                lidarF_tot = distanceF_values(:,1:1200);
                lidarR_tot = distanceR_values(:,1:1200);
                compass_tot = compass_values(:,1:1200);
                gyro_tot = gyro_values(:,1:1200);
     end
        
     if( i == 3)
                %Change Variables
                position_tot = true_position(:,1:1000);
                linVel_tot = true_lin_vel(:,1:1000);
                angVel_tot = true_ang_vel(:,1:1000);
                anglex_tot = true_angels(:,1:1000);
                t = time(:,1:1000);
                lidarF_tot = distanceF_values(:,1:1000);
                lidarR_tot = distanceR_values(:,1:1000);
                compass_tot = compass_values(:,1:1000);
                gyro_tot = gyro_values(:,1:1000);
      end
        
      if i > 4 && i <= 12
                anglex_tot = angelx_tot;
      end
      
     if( i == 20)
            %Change Variables
            position_tot = true_position(:,1:6000);
            linVel_tot = true_lin_vel(:,1:6000);
            angVel_tot = true_ang_vel(:,1:6000);
            anglex_tot = true_angels(:,1:6000);
            t = time(:,1:6000);
            lidarF_tot = distanceF_values(:,1:6000);
            lidarR_tot = distanceR_values(:,1:6000);
            compass_tot = compass_values(:,1:6000);
            gyro_tot = gyro_values(:,1:6000);
     end
     if( i == 21)
            position_tot = pos;
            linVel_tot = lin_vel;
            angVel_tot = ang_vel;
            anglex_tot = ang_pos;
            t=py_t;
            lidarF_tot = distanceF_data;
            lidarR_tot = distanceR_data;
            compass_tot = compass_data;
            gyro_tot = gyro_data;
     end
     if( i == 23)
            %Change Variables
            position_tot = pos;
            linVel_tot = lin_vel;
            angVel_tot = ang_vel;
            anglex_tot = ang_pos;
            t=py_t;
            lidarF_tot = distanceF_data;
            lidarR_tot = distanceR_data;
            compass_tot = compass_data;
            gyro_tot = gyro_data;
     end
    
%     %plots ground truth
    figure(1)
    plot(t/1000, position_tot(1,:), 'g');
    hold on
    plot(t/1000, position_tot(2,:), 'r');
    plot(t/1000, position_tot(3,:), 'b');
    plot(py_t/1000, py_position_tot(1,:), '--g');
    plot(py_t/1000, py_position_tot(2,:), '--r');
    plot(py_t/1000, py_position_tot(3,:), '--b');
    title('Webots vs. Analytical Simulation True Position');
    xlabel('Time [sec]');
    ylabel('Position [m]');
    legend('Webots-X','Webots-Y','Webots-Z','Analytical-X','Analytical-Y','Analytical-Z');
    hold off
    saveas(gcf, fullfile(fname_fig,['true_pos_path', num2str(i)]),'png');

    figure(2)
    plot(t/1000, linVel_tot(1,:), 'g');
    hold on
    plot(t/1000, linVel_tot(2,:), 'r');
    plot(t/1000, linVel_tot(3,:), 'b');
    plot(py_t/1000, py_linVel_tot(1,:), '--g');
    plot(py_t/1000, py_linVel_tot(2,:), '--r');
    plot(py_t/1000, py_linVel_tot(3,:), '--b');
    title('Webots vs. Analytical Simulation True Linear Velocity');
    xlabel('Time [sec]');
    ylabel('Velocity [m/s]');
    legend('Webots-V_x','Webots-V_y','Webots-V_z','Analytical-V_x','Analytical-V_y','Analytical-V_z');
    hold off
    saveas(gcf, fullfile(fname_fig,['true_linvel_path', num2str(i)]),'png');
%     
    figure(3)
    plot(t/1000, anglex_tot(1,:), 'g');
    hold on
    plot(t/1000, anglex_tot(2,:), 'r');
    plot(t/1000, anglex_tot(3,:), 'b');
    plot(py_t/1000, py_anglex_tot(1,:), '--g');
    plot(py_t/1000, py_anglex_tot(2,:), '--r');
    plot(py_t/1000, py_anglex_tot(3,:), '--b');
    title('Webots vs. Analytical Simulation True Angular Position');
    xlabel('Time [sec]');
    ylabel('$\hat{i}$ \sf{Components [unitless]}','Interpreter','Latex');
    legend('Webots-$\hat{i}_x$','Webots-$\hat{i}_y$','Webots-$\hat{i}_z$',...
        'Analytical-$\hat{i}_x$','Analytical-$\hat{i}_y$','Analytical-$\hat{i}_z$',...
        'Interpreter','Latex');
    hold off
    saveas(gcf, fullfile(fname_fig,['true_angpos_path', num2str(i)]),'png');
%     
    figure(4)
    plot(t/1000, angVel_tot(1,:), 'g');
    hold on
    plot(t/1000, angVel_tot(2,:), 'r');
    plot(t/1000, angVel_tot(3,:), 'b');
    plot(py_t/1000, py_angVel_tot(1,:), '--g');
    plot(py_t/1000, py_angVel_tot(2,:), '--r');
    plot(py_t/1000, py_angVel_tot(3,:), '--b');
    title('Webots vs. Analytical Simulation True Angular Velocity');
    xlabel('Time [sec]');
    ylabel('Velocity [rad/s]');
    legend('Webots-$\omega_x$','Webots-$\omega_y$','Webots-$\omega_z$',...
        'Analytical-$\omega_x$','Analytical-$\omega_y$','Analytical-$\omega_z$',...
        'Interpreter','Latex');
    hold off
    saveas(gcf, fullfile(fname_fig,['true_angvel_path', num2str(i)]),'png');
%     
%     % sensor data
%     % distance
%     figure(5)
%     plot(t/1000, lidarF_tot*10/4096, 'b');
%     hold on
%     plot(t/1000, lidarR_tot*10/4096, 'r');
%     plot(py_t/1000, py_lidarF_tot*10/4096, 'g');
%     plot(py_t/1000, py_lidarR_tot*10/4096, 'y');
%     title('Webots vs. Analytical Simulation Lidar Distance Output');
%     xlabel('Time [sec]');
%     ylabel('Distance to Wall [m]');
%     legend('Webots-Front', 'Webots-Right', 'Analytical-Front', 'Analytical-Right');
%     hold off
%     saveas(gcf, fullfile(fname_fig,['sens_lidar_path', num2str(i)]),'png');
%     
    % compass
    figure(6)
    plot(t/1000, compass_tot(1,:), 'g');
    hold on
    plot(t/1000, compass_tot(2,:), 'r');
    plot(t/1000, compass_tot(3,:), 'b');
    plot(py_t/1000, py_compass_tot(1,:),'y');
    plot(py_t/1000, py_compass_tot(2,:), 'm');
    plot(py_t/1000, py_compass_tot(3,:), 'c');
    title('Webots vs. Analytical Simulation Compass Output');
    xlabel('Time [sec]');
    ylabel('$\hat{i}$ \sf{Components [unitless]}','Interpreter','Latex');
    legend('Webots-$\hat{i}_x$','Webots-$\hat{i}_y$','Webots-$\hat{i}_z$',...
        'Analytical-$\hat{i}_x$','Analytical-$\hat{i}_y$','Analytical-$\hat{i}_z$', ...
        'Interpreter','Latex');
    hold off
    saveas(gcf, fullfile(fname_fig,['sens_angpos_path', num2str(i)]),'png');
%     
    % gyro
    figure(7)
    plot(t/1000, gyro_tot(1,:), 'g');
    hold on
    plot(t/1000, gyro_tot(2,:), 'r');
    plot(t/1000, gyro_tot(3,:), 'b');
    plot(py_t/1000, py_gyro_tot(1,:),'y');
    plot(py_t/1000, py_gyro_tot(2,:), 'm');
    plot(py_t/1000, py_gyro_tot(3,:), 'c');
    title('Webots vs. Analytical Simulation Gyro Output');
    xlabel('Time [sec]');
    ylabel('Velocity [rad/s]');
    legend('Webots-$\omega_x$','Webots-$\omega_y$','Webots-$\omega_z$',...
        'Analytical-$\omega_x$','Analytical-$\omega_y$','Analytical-$\omega_z$',...
        'Interpreter','Latex');
    hold off
    saveas(gcf, fullfile(fname_fig,['sens_angvel_path', num2str(i)]),'png');

%      %plots motor inputs
%      figure(8)
%      plot(py_t/1000, py_vL,'b');
%      hold on
%      plot(py_t/1000, py_vR,'r');
%      title(['Path ',num2str(i),' Motor Inputs']);
%      xlabel('Time [sec]');
%      ylabel('Angular Velocity Input [rad/s]');
%      legend('Left','Right','Location','best');
%      if i<= 18
%          saveas(gcf, fullfile(fname_fig,['paperbot',num2str(i),'_motor']),'png');
%      else
%          saveas(gcf, fullfile(fname_fig,['segway',num2str(i),'_motor']),'png')
%      end
%      hold off
end