function [ u1, u2 ] = controller(~, state, des_state, params)
%CONTROLLER  Controller for the planar quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [y; z], state.vel = [y_dot; z_dot], state.rot = [phi],
%   state.omega = [phi_dot]
%
%   des_state: The desired states are:
%   des_state.pos = [y; z], des_state.vel = [y_dot; z_dot], des_state.acc =
%   [y_ddot; z_ddot]
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls
m = 1.0*params.mass;
g = params.gravity;
Ixx = params.Ixx;

% Desired States
y_ddot_des = des_state.acc(1);
z_ddot_des = des_state.acc(2);
y_dot_des = des_state.vel(1);
z_dot_des = des_state.vel(2);
y_des = des_state.pos(1);
z_des = des_state.pos(2);

% Current States
y_dot = state.vel(1);
z_dot = state.vel(2);
y = state.pos(1);
z = state.pos(2);
phi = state.rot(1);
phi_dot = state.omega(1);

% Gains
Kp = [10; 10; 25];
Kd = [10; 10; 25];
Kp_rot = [1000; 10; 10];
Kd_rot = [100; .1; .1];

% Control Equations
phi_c = -(Kd(2)*(y_dot_des - y_dot) + Kp(2)*(y_des - y))/g;
% phi_c_dot = 0; %near hovering
phi_c_dot = -(Kd(2)*(y_ddot_des + g*phi) + Kp(2)*(y_dot_des - y_dot))/g;
u1 = m*(Kd(3)*(z_dot_des - z_dot) + Kp(3)*(z_des - z));
u2 = Ixx*(Kp_rot(1)*(phi_c-phi)+ Kd_rot(1)*(phi_c_dot-phi_dot));

end

