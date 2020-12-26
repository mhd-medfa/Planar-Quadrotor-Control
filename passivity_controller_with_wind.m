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
Ixx = 1.0*params.Ixx;
W = params.W;

% Desired States
y_ddot_des = des_state.acc(1);
z_ddot_des = des_state.acc(2);
y_dot_des = des_state.vel(1);
z_dot_des = des_state.vel(2);
y_des = des_state.pos(1);
z_des = des_state.pos(2);
phi_des = -y_ddot_des/g;

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
Kp_rot = [10; 10; 10];
Kd_rot = [1; .1; .1];

%Kp = [10; 10; 80];
%Kd = [10; 10; 35];
%Kp_rot = [200; 200; 200];
%Kd_rot = [.1; .1; .1];
% Control Equations


% 
% Lambda = 10;
%Lambda = [35 350 3];
%Lambda = [20 200 3];
Lambda = [35 250 0.3];

e_Y = y_des - y;
e_Z = z_des - z;
e_phi = phi_des - phi;
de_Y = y_dot_des - y_dot;
de_Z = z_dot_des - z_dot;
de_phi = 0-phi_dot;

% Change of variables
y_star = de_Y + Lambda(1)*e_Y;
z_star = de_Z + Lambda(2)*e_Z;
dy_s = z_dot_des + Lambda(1)*e_Y;
dz_s = z_dot_des + Lambda(2)*e_Z;
dphi_s = 0 + Lambda(3)*e_phi;
ddy_s = y_ddot_des + Lambda(1)*de_Y;
ddz_s = z_ddot_des + Lambda(2)*de_Z;
ddphi_s = Lambda(3)*de_phi;

% %regressor
Y = [(ddz_s+g)/(cos(phi)+1e-5) 0
    0, ddphi_s];
% parameters
P = [m, Ixx];

Y_dot_q = Y*P'+W;

%u = dot(regressor(q, dq_s, ddq_s , l, g), p) + dot(K, q_star)

phi_c = -(ddy_s + Kd(2)*dy_s + Kp(2)*y_star)/g;
phi_c_dot = 0; %near hovering

u1 = Y_dot_q(1) + Kd(3)*dz_s + Kp(3)*z_star;
u2 = Y_dot_q(2) + Kp_rot(1)*(phi_c-phi)+ Kd_rot(1)*ddphi_s;%*(phi_c_dot-phi_dot);

end

