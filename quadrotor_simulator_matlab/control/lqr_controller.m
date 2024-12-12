function [F, M, trpy, drpy] = lqr_controller(qd, t, qn, params, trajhandle)
% CONTROLLER quadrotor controller
% The current states are:
% qd{qn}.pos, qd{qn}.vel, qd{qn}.euler = [roll;pitch;yaw], qd{qn}.omega
% The desired states are:
% qd{qn}.pos_des, qd{qn}.vel_des, qd{qn}.acc_des, qd{qn}.yaw_des, qd{qn}.yawdot_des
% Using these current and desired states, you have to compute the desired controls

% =================== Your code goes here ===================
persistent gd;
persistent icnt;
 if isempty(gd)
     gd = zeros(0,3);
     icnt = 0;
 end
 icnt = icnt + 1;

%% Parameter Initialization

if ~isempty(t)
desired_state = trajhandle(t, qn);
end

% F = 0;
% M = [0;0;0];

X = [qd{qn}.pos; qd{qn}.vel; qd{qn}.euler; qd{qn}.omega];

euler_des = [0; 0; desired_state.yaw];
omega_des = [0; 0; desired_state.yawdot];
% 
X_des = [desired_state.pos; desired_state.vel; euler_des; omega_des];
 
% params

m = params.mass;
g = params.grav;

l = params.arm_length;

Ixx = 1.43e-5;
Iyy = 1.43e-5;         
Izz = 2.89e-5;

% phi = qd{qn}.euler(1);
% theta = qd{qn}.euler(2);
% psi = qd{qn}.euler(3);
% 
% v = [cos(theta) 0 -cos(phi)*sin(theta);...
%     0 1 sin(phi);...
%     sin(theta) 0 cos(phi)*cos(theta)];

 
A = [zeros(3), eye(3), zeros(3,6);
     zeros(3,6), [0 g 0; -g 0 0; 0 0 0], zeros(3)
     zeros(3, 9),eye(3);
     zeros(3, 12)];

B = [zeros(5, 4);
    [1/m, 0, 0, 0];
    zeros(3, 4);
    zeros(3, 1), diag([l/Ixx, l/Iyy, 1/Izz])];
 

R = diag([0.01, 0.1, 0.1, 0.1]);                                               
Q = diag([0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.01, 0.01, 0.01]);
 
[K, ~, ~] = lqr(A,B,Q,R);

u = -K * (X - X_des);

F = u(1);

M = u(2:4);




%Output trpy and drpy as in hardware
trpy = [0, 0, 0, 0];
drpy = [0, 0,       0,         0];

end
