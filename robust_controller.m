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
m = 0.75*params.mass;
g = params.gravity;
Ixx = 0.8*params.Ixx;
W = params.W;
%W = atmoshwm(55.7535,48.7434,250,'day',350,'seconds',39600,'apindex',80,'model','total', 'version', '14');

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
Kp = [10; 5; 50];
Kd = [10; 5; 50];
Kp_rot = [1000; 10; 10];
Kd_rot = [100; .1; .1];

%Kp = [10; 10; 80];
%Kd = [10; 10; 35];
%Kp_rot = [200; 200; 200];
%Kd_rot = [.1; .1; .1];
% Control Equations



Lambda = [20 250 0.00025];

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

Wind_Force_min = -0.5*params.W;
Wind_Force_max = 0.5*params.W;
Wind_Force_hat = (Wind_Force_min+Wind_Force_max)/2;

epsilon=0.0087;
%regressor
if abs(cos(phi))<=epsilon
    epsilon = 0.0087;
else
    epsilon=cos(phi);
end

%regressor
% Y = [ddz_s+g
%     0];
% % parameters
% P = [m];
% 
% Y_dot_q = Y(1)*P(1);
Y = [(ddz_s+g+Wind_Force_hat(1))/(epsilon) 0
    Wind_Force_hat(2), ddphi_s];
% parameters
P = [m, Ixx];

Y_dot_q = Y*P';
eps = 0.01;
rho = 1;

if norm(Y_dot_q)>eps
  delta_p = rho*Y_dot_q/norm(Y_dot_q);
else
  delta_p = rho*Y_dot_q/eps;
end
%u = dot(regressor(q, dq_s, ddq_s , l, g), p) + dot(K, q_star)
Y_dot_q_dlta_q = Y*(P+delta_p)';
phi_c = -(ddy_s + Kd(2)*dy_s + Kp(2)*y_star)/g;
% phi_c_dot = 0; %near hovering
phi_c_dot = -(Kd(2)*(y_ddot_des + g*phi) + Kp(2)*(y_dot_des - y_dot))/g;
%e_phi = (phi_c-phi);
%dphi_s = 0 + Lambda(3)*e_phi;

Y_dot_q_dlta_q = Y_dot_q_dlta_q ;%+ Wind_Force_hat;

u1 = Y_dot_q_dlta_q(1) + m*(Kd(3)*dz_s + Kp(3)*z_star);
u2 = Y_dot_q_dlta_q(2) + Ixx*(Kp_rot(1)*(phi_c-phi)+ Kd_rot(1)*(phi_c_dot-phi_dot));

end

