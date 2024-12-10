function [F, M, trpy, drpy] = mpc_controller(qd, t, qn, params, trajhandle)
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

F = 0;
M = [0;0;0];

% X = [x; y; z;x_dot; y_dot; z_dot; r; p; y; p; q; r] X_dot = [x_dot; y_dot; z_dot; xdd; ydd; zdd; p; q; r; p_dot; q_dot; r_dot]

m = params.mass;
g = params.grav;

Ixx = 1.43;
Iyy = 1.43;
Izz = 2.89;


A = [zeros(3), eye(3), zeros(3,6);
     zeros(3,6), diag([1, -1, 0]), zeros(3)
     zeros(3, 9), eye(3);
     zeros(3, 12)];

B = [zeros(5, 4);
    [1/m, 0, 0, 0];
    zeros(3, 4);
    zeros(3, 1), diag([1/Ixx, 1/Iyy, 1/Izz])];



%Output trpy and drpy as in hardware
trpy = [0, 0, 0, 0];
drpy = [0, 0,       0,         0];

end
