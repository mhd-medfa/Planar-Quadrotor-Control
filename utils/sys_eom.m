function [ sdot ] = sys_eom(t, s, controlhandle, trajhandle, params)
% sys_eom Differential equation for the 2D quadrtotor system
% State:
%   [y; z; phi; y_dot; z_dot; phi_dot]


current_state.pos = s(1:2);
current_state.rot = s(3);
current_state.vel = s(4:5);
current_state.omega = s(6);

desired_state = trajhandle(t, current_state);
[F, M] = controlhandle(t, current_state, desired_state, params);
W = params.W;

%    u1      u2
%  _____    _____
%    |________|
%
% F = u1 + u2
% M = (u2 - u1)*arm_length

u1 = 0.5*(F - M/params.arm_length);
u2 = 0.5*(F + M/params.arm_length);

u1_clamped = min(max(params.minF/2, u1), params.maxF/2);
u2_clamped = min(max(params.minF/2, u2), params.maxF/2);
F_clamped = u1_clamped + u2_clamped;
M_clamped = (u2_clamped - u1_clamped)*params.arm_length;
u1_clamped = min(max(params.minF/2, u1), params.maxF/2);
u2_clamped = min(max(params.minF/2, u2), params.maxF/2);
i = params.step;
global control_activity;
control_activity(i, 1:2) = [F, M];
% F_clamped = F;
% M_clamped = M;
% if norm(F_clamped - F) > 0.01
%   disp('Force clamped');
% end
% if norm(M_clamped - M) > 0.01
%   disp('Moment clamped');
% end
epsilon=0.0087;
epsilon2=0.0087;
%regressor
if abs(sin(s(3)))<=epsilon
    epsilon = 0.00087;
else
    epsilon = sin(s(3));
end
if abs(cos(s(3)))<=epsilon2
    epsilon2 = 0.0087;
else
    epsilon2=cos(s(3));
end
sdot = [s(4);
        s(5);
        s(6);
        -(F_clamped)*sin(s(3))/params.mass + W(2);
        (F_clamped)*cos(s(3))/params.mass - params.gravity + W(1);
        M_clamped/params.Ixx];

end