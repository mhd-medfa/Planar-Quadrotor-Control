clear;
close all;

addpath('utils');
addpath('trajectories');

% controlhandle = @pd_controller;
%controlhandle = @pd_fl_controller;
%controlhandle = @passivity_controller;
controlhandle = @robust_controller;

% Choose which trajectory you want to test with
%trajhandle = @traj_line;
trajhandle = @traj_sine;

global control_activity; 
control_activity = nan(500, 2);

[t, state, des_state, wind_disturbances] = simulation_2d(controlhandle, trajhandle);

figure
subplot(2,2,1);
plot(t, state(:,1), 'b', t, des_state(:,1), 'r-.', 'LineWidth', 2);
% Add axis labels
xlabel('Time t(s)')
ylabel('Y pos')
% Add a legend in the best location
legend('actual y', 'desired y','Location', 'Best')

% figure
subplot(2,2,2);
plot(t, des_state(:,1) - state(:,1), 'r-.', 'LineWidth', 2);
% Add axis labels
xlabel('Time t(s)')
ylabel('Error (y_{des} - y_{actual})')
% Add a legend in the best location
legend('Error (y_{des} - y_{actual})','Location', 'Best')

% figure
subplot(2,2,3);
plot(t, state(:, 2), 'b', t, des_state(:, 2), 'r-.', 'LineWidth', 2);
% Add axis labels
xlabel('Time t(s)')
ylabel('Z pos')
% Add a legend in the best location
legend('actual z', 'desired z','Location', 'Best')

% figure
subplot(2,2,4);
plot(t, des_state(:, 2) - state(:, 2), 'r-.', 'LineWidth', 2);
% Add axis labels
xlabel('Time t(s)')
ylabel('Error (z_{des} - z_{actual})')
% Add a legend in the best location
legend('Error (z_{des} - z_{actual})','Location', 'Best')

figure
subplot(2,2,3);
plot(t, wind_disturbances(:,1), 'b', 'LineWidth', 1.2);
% Add axis labels
xlabel('Time t(s)')
ylabel('Y Wind Disturbances','interpreter','latex')
% Add a legend in the best location
legend('Y Wind Disturbances','Location', 'SouthEast' ,'interpreter','latex')

subplot(2,2,4);
plot(t, wind_disturbances(:,2), 'b', 'LineWidth', 1.2);
% Add axis labels
xlabel('Time t(s)')
ylabel('Z Wind Disturbances','interpreter','latex')
% Add a legend in the best location
legend('Z Wind Disturbances','Location', 'SouthEast' ,'interpreter','latex')

subplot(2,2,1);
plot(t(1:5:500), control_activity(1:100, 1), 'g', 'LineWidth', 1.5);
% Add axis labels
xlabel('Time t(s)')
ylabel('Control activity ($u_1$)','interpreter','latex')
% Add a legend in the best location
legend('$u_1$','Location', 'SouthEast' ,'interpreter','latex')

subplot(2,2,2);
plot(t(1:5:500), control_activity(1:100, 2), 'g', 'LineWidth', 1.5);
% Add axis labels
xlabel('Time t(s)')
ylabel('Control activity($u_2$)','interpreter','latex')
% Add a legend in the best location
legend('$u_2$','Location', 'SouthEast' ,'interpreter','latex')
