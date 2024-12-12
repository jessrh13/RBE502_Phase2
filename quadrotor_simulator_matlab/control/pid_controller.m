function [F, M, trpy, drpy] = pid_controller(qd, t, qn, params)
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
 
% =================== Your code starts here ===================
%% Parameter Initialization
% Position control gains
kp = [4.15; 4.15; 3];
kd = [10.25; 10.25; 3.5];

% Attitude control gains
kpr = 0.5;
kpp = 0.5;
kpy = 1.2;

kdr = 3.0;
kdp = 3.0;
kdy = 1.8;


% params
qd{qn}.acc_des = kd.*(-qd{qn}.vel) + kp.*(qd{qn}.pos_des-qd{qn}.pos);


m = params.mass;
g = params.grav;

F = m*g - m*(kd(3) * qd{qn}.vel(3) + kp(3)*(qd{qn}.pos(3) - qd{qn}.pos_des(3)))


roll_des = (1/params.grav)*(qd{qn}.acc_des(1)*sin(qd{qn}.euler(3)) - ...
    qd{qn}.acc_des(2)*cos(qd{qn}.euler(3)))

pitch_des = (1/params.grav)*(qd{qn}.acc_des(1)*cos(qd{qn}.euler(3))...
    + qd{qn}.acc_des(2)*sin(qd{qn}.euler(3)))

p_des = 0;
q_des = 0;


kMoment =[kpr*(roll_des - qd{qn}.euler(1))+kdr*(p_des - qd{qn}.omega(1));...
    kpp*(pitch_des-qd{qn}.euler(2))+kdp*(q_des - qd{qn}.omega(2));...
    kpy*(qd{qn}.yaw_des -qd{qn}.euler(3))+kdy*(qd{qn}.yawdot_des - qd{qn}.omega(3))];

params.I
M = params.I * kMoment;


% =================== Your code ends here ===================

%Output trpy and drpy as in hardware
trpy = [0, 0, 0, 0];
drpy = [0, 0, 0, 0];

end