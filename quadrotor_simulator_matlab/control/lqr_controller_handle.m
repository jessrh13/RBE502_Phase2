function [F, M, trpy, drpy] = lqr_controller_handle(K, qd, qn, t, params, trajhandle)
m = params.mass;
g = params.grav;

l = params.arm_length;

if ~isempty(t)
    desired_state = trajhandle(t, qn);
end

X = [qd{qn}.pos; qd{qn}.vel; qd{qn}.euler; qd{qn}.omega];

euler_des = [0; 0; desired_state.yaw];
omega_des = [0; 0; desired_state.yawdot];
% 
X_des = [desired_state.pos; desired_state.vel; euler_des; omega_des];

u = -K * (X - X_des);

% F = 0;
% M = [0; 0; 0];

F = m*g + u(1);

M = u(2:4);

%Output trpy and drpy as in hardware
trpy = [0, 0, 0, 0];
drpy = [0, 0,       0,         0];
end
