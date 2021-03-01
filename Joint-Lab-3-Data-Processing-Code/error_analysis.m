clc; clear; clear all;
%for ssaving to latex
rlabel = {'Trial 1','Trial 2','Trial 3','Trial 4','Trial 5','Trial 6','Trial 7','Trial 8','Trial 9','Trial 10','Trial 11','Trial 12','Trial 13','Trial 14','Trial 15','Trial 16','Trial 17','Trial 18'};
%rlabel = {'Trial 19','Trial 20','Trial 21','Trial 22','Trial 23','Trial 24','Trial 25','Trial 26','Trial 27','Trial 28','Trial 29','Trial 30','Trial 31','Trial 32','Trial 33','Trial 34','Trial 35','Trial 36'};
clabel_true_pos = {'X', 'Y', 'Z', 'True ix', 'True iy', 'True iz'};
clabel_true_vel = {'Vx', 'Vy', 'Vz', 'wx', 'wy', 'wz'};
clabel_sens = {'Front Lidar', 'Right Lidar', 'Compass ix', 'Compass iy', 'Compass iz', 'Gyro wx', 'Gyro wy', 'Gyro wz'};

len = length(rlabel);

%create error storage arrays
lidarF_error = zeros(1,len) ;
lidarR_error = zeros(1,len) ;

angVel_x_error = zeros(1,len) ;
angVel_y_error = zeros(1,len) ;
angVel_z_error = zeros(1,len) ;

anglex_i_error = zeros(1,len) ;
anglex_j_error = zeros(1,len) ;
anglex_k_error = zeros(1,len) ;

compass_i_error = zeros(1,len) ;
compass_j_error = zeros(1,len) ;
compass_k_error = zeros(1,len) ;

gyro_x_error = zeros(1,len) ;
gyro_y_error = zeros(1,len) ;
gyro_z_error = zeros(1,len) ;

linVel_x_error = zeros(1,len) ;
linVel_y_error = zeros(1,len) ;
linVel_z_error = zeros(1,len) ;

position_x_error = zeros(1,len) ;
position_y_error = zeros(1,len) ;
position_z_error = zeros(1,len) ;

%for normalization
%wheel_dia = .50187 ; %segway
wheel_dia = 50/1000 ; %paperbot

%y_off = 0.288 ; %segway
y_off = 0.035 ; %paperbot

j = 1;
for i=1:18
    if i ~= 37
        fname_tab = 'C:\Users\mjwil\Desktop\Current Classes\mae 162D\Labs\Lab 3\Code\Latex Tables';
        
        load(['paperbot_',num2str(i),'.mat']);
        load(['paperbot_data_path',num2str(i),'.mat'])
        
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
            position_tot = true_position(:,1:5:6000);
            linVel_tot = true_lin_vel(:,1:5:6000);
            angVel_tot = true_ang_vel(:,1:5:6000);
            anglex_tot = true_angels(:,1:5:6000);
            t = time(:,1:5:6000);
            lidarF_tot = distanceF_values(:,1:5:6000);
            lidarR_tot = distanceR_values(:,1:5:6000);
            compass_tot = compass_values(:,1:5:6000);
            gyro_tot = gyro_values(:,1:5:6000);
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
        
        dt = (t(2)-t(1))/1000 ;
        dur = t(end)*1e-3; %duration of sim
        
        area_lidarF = 0;
        area_lidarR = 0;
        
        area_angVel_x = 0 ; %omega x (true)
        area_angVel_y = 0 ; %omega y (true)
        area_angVel_z = 0 ; %omega z (true)
        
        area_anglex_i = 0 ; %i comp of orientation of x axis (true)
        area_anglex_j = 0 ; %j comp of orientation of x axis (true)
        area_anglex_k = 0 ; %k comp of orientation of x axis (true)
        
        area_compass_i = 0 ; %i comp of orientation of x axis (sensor)
        area_compass_j = 0 ; %j comp of orientation of x axis (sensor)
        area_compass_k = 0 ; %k comp of orientation of x axis (sensor)
        
        area_gyro_x = 0 ; %omega x (sensor)
        area_gyro_y = 0 ; %omega y (sensor)
        area_gyro_z = 0 ; %omega z (sensor)
        
        area_linVel_x = 0 ; %Vx
        area_linVel_y = 0 ; %Vy
        area_linVel_z = 0 ; %Vz
        
        area_position_x = 0 ; %X
        area_position_y = 0 ; %Y
        area_position_z = 0 ; %Z
        
        for n = 1:length(t)-1
            area_lidarF = area_lidarF + (abs(lidarF_tot(n)-py_lidarF_tot(n))*dt) ;
            area_lidarR = area_lidarR + (abs(lidarR_tot(n)-py_lidarR_tot(n))*dt) ;
            
            area_angVel_x = area_angVel_x + (abs(angVel_tot(1,n)-py_angVel_tot(1,n))*dt) ;
            area_angVel_y = area_angVel_y + (abs(angVel_tot(2,n)-py_angVel_tot(2,n))*dt) ;
            area_angVel_z = area_angVel_z + (abs(angVel_tot(3,n)-py_angVel_tot(3,n))*dt) ;
            
            area_anglex_i = area_anglex_i + (abs(anglex_tot(1,n)-py_anglex_tot(1,n))*dt) ;
            area_anglex_j = area_anglex_j + (abs(anglex_tot(2,n)-py_anglex_tot(2,n))*dt) ;
            area_anglex_k = area_anglex_k + (abs(anglex_tot(3,n)-py_anglex_tot(3,n))*dt) ;
            
            area_compass_i = area_compass_i + (abs(compass_tot(1,n)-py_compass_tot(1,n))*dt) ;
            area_compass_j = area_compass_j + (abs(compass_tot(2,n)-py_compass_tot(2,n))*dt) ;
            area_compass_k = area_compass_k + (abs(compass_tot(3,n)-py_compass_tot(3,n))*dt) ;
            
            area_gyro_x = area_gyro_x + (abs(gyro_tot(1,n)-py_gyro_tot(1,n))*dt) ;
            area_gyro_y = area_gyro_y + (abs(gyro_tot(2,n)-py_gyro_tot(2,n))*dt) ;
            area_gyro_z = area_gyro_z + (abs(gyro_tot(3,n)-py_gyro_tot(3,n))*dt) ;
            
            area_linVel_x = area_linVel_x + (abs(linVel_tot(1,n)-py_linVel_tot(1,n))*dt) ;
            area_linVel_y = area_linVel_y + (abs(linVel_tot(2,n)-py_linVel_tot(2,n))*dt) ;
            area_linVel_z = area_linVel_z + (abs(linVel_tot(3,n)-py_linVel_tot(3,n))*dt) ;
            
            area_position_x = area_position_x + (abs(position_tot(1,n)-py_position_tot(1,n))*dt) ;
            area_position_y = area_position_y + (abs(position_tot(2,n)-y_off-py_position_tot(2,n))*dt) ;
            area_position_z = area_position_z + (abs(position_tot(3,n)-py_position_tot(3,n))*dt) ;
        end
        
        %normalizing values and rounding to 3 sig figs
        lidarF_error(j) = round(area_lidarF*10/(4096*wheel_dia)/dur,3,'significant') ;
        lidarR_error(j) = round(area_lidarR*10/(4096*wheel_dia)/dur,3,'significant') ;
        
        angVel_x_error(j) = round(area_angVel_x/dur,3,'significant') ;
        angVel_y_error(j) = round(area_angVel_y/dur,3,'significant') ;
        angVel_z_error(j) = round(area_angVel_z/dur,3,'significant') ;
        
        anglex_i_error(j) = round(area_anglex_i/dur,3,'significant') ;
        anglex_j_error(j) = round(area_anglex_j/dur,3,'significant') ;
        anglex_k_error(j) = round(area_anglex_k/dur,3,'significant') ;
        
        compass_i_error(j) = round(area_compass_i/dur,3,'significant') ;
        compass_j_error(j) = round(area_compass_j/dur,3,'significant') ;
        compass_k_error(j) = round(area_compass_k/dur,3,'significant') ;
        
        gyro_x_error(j) = round(area_gyro_x/dur,3,'significant') ;
        gyro_y_error(j) = round(area_gyro_y/dur,3,'significant') ;
        gyro_z_error(j) = round(area_gyro_z/dur,3,'significant') ;
        
        linVel_x_error(j) = round(area_linVel_x/wheel_dia/dur,3,'significant') ;
        linVel_y_error(j) = round(area_linVel_y/wheel_dia/dur,3,'significant') ;
        linVel_z_error(j) = round(area_linVel_z/wheel_dia/dur,3,'significant') ;
        
        position_x_error(j) = round(area_position_x/wheel_dia/dur,3,'significant') ;
        position_y_error(j) = round(area_position_y/wheel_dia/dur,3,'significant') ;
        position_z_error(j) = round(area_position_z/wheel_dia/dur,3,'significant') ;
    end
%     if i ==20
%         lidarF_error(j) = 0;
%         lidarR_error(j) = 0;
%         angVel_error(j) = 0;
%         anglex_error(j) = 0;
%         compass_error(j) = 0;
%         gyro_error(j) = 0;
%         linVel_error(j) = 0;
%         position_error(j) = 0;
%     end
    j=j+1;
end

%creates total array
error_true_pos = zeros(len,6);
error_true_vel = zeros(len,6);
error_sens = zeros(len,8);

error_true_pos(:,1) = position_x_error';
error_true_pos(:,2) = position_y_error';
error_true_pos(:,3) = position_z_error';
error_true_pos(:,4) = anglex_i_error';
error_true_pos(:,5) = anglex_j_error';
error_true_pos(:,6) = anglex_k_error';

error_true_vel(:,7) = linVel_x_error';
error_true_vel(:,8) = linVel_y_error';
error_true_vel(:,9) = linVel_z_error';
error_true_vel(:,10) = angVel_x_error';
error_true_vel(:,11) = angVel_y_error';
error_true_vel(:,12) = angVel_z_error';


error_sens(:,13) = lidarF_error';
error_sens(:,14) = lidarR_error';
error_sens(:,15) = compass_i_error';
error_sens(:,16) = compass_j_error';
error_sens(:,17) = compass_k_error';
error_sens(:,18) = gyro_x_error';
error_sens(:,18) = gyro_y_error';
error_sens(:,18) = gyro_z_error';

error_sens_matrix = [lidarF_error; lidarR_error; compass_i_error; compass_j_error; compass_k_error; gyro_x_error; gyro_y_error; gyro_z_error];
error_true_pos_matrix = [position_x_error; position_y_error; position_z_error; anglex_i_error; anglex_j_error; anglex_k_error];
error_true_vel_matrix = [linVel_x_error; linVel_y_error; linVel_z_error; angVel_x_error; angVel_y_error; angVel_z_error];

matrix2latex(transpose(error_true_pos_matrix), fullfile(fname_tab,'error_paper_true_pos.tex'),'rowLabels', rlabel, 'columnLabels', clabel_true_pos);
matrix2latex(transpose(error_true_vel_matrix), fullfile(fname_tab,'error_paper_true_vel.tex'),'rowLabels', rlabel, 'columnLabels', clabel_true_vel);
matrix2latex(transpose(error_sens_matrix), fullfile(fname_tab,'error_paper_sensors.tex'),'rowLabels', rlabel, 'columnLabels', clabel_sens);
