% Problem Set 2 | Alec Trela | Sept. 28, 2022
clc
clear
% Question 1.b
clc

% Find the Poles and Zeros of T(s)

% Create the tf G(s)
G = tf(200, [1, 22, 141, 2]);

% Create T, noting that T = G / (1 + G) during unity feedback
T = G / (1 + G);

% get the num, den for the tf2zp function
T_num = cell2mat(T.Numerator);
T_den = cell2mat(T.Denominator);

% Find the Poles and Zeros of the TF
[Z, P] = tf2zp(T_num, T_den);

% Print
disp("The zeros of T_cl are:")
disp(Z)
disp("The poles of the T_cl are:")
disp(P)

%Question 1.c
clc

% Create the step response of T
figure(1)
step(T)

% Plot the root locus to visualize poles, zeros
figure(2)
rlocus(T)
 
% Question 2
clc
% Create PID transfer function
kd = 587;
kp = 1108;
ki = 0.001;
H = tf([kd, kp, ki], [1, 0]);

% create the Tf from the problem set
G_2 = tf([1, 10], [1, 71, 1070, 1000, 0]);

% Create the closed loop tf
T_cl_2 = (G_2 * H) / (1 + (G_2 * H));

% plot the step response and get the transient response params
figure(3);
step(T_cl_2);
[y, t] = step(T_cl_2);
params = stepinfo(T_cl_2);
sserror = abs(1 - y(end));

disp("Transient Response Parameters:")
disp(params)
disp("Steady State Error:")
disp(sserror)

%% Question 3
clc
clear

% create the reference square wave
t200 = 0:0.01:200;
ref_pos = (20) * square(2*pi*(1/100)*t200);

A = [0 0 1 0;
    0 0 0 1;
    0 1 -3 0;
    0 2 -3 0];


B = [0;
    0;
    1;
    1];

C = [1 0 0 0];

Q = [15 0 0 0; 0 1 0 0; 0 0 15 0; 0 0 0 1];
R = 25;

%%  Find the observability of the system
% Create the matrix W = [C | CA | CA^2]
CA = C * A;
CA2 = C * A ^ 2;
CA3 = C * A ^ 3;
W = [C;  CA;  CA2; CA3];

obsv = rank(W) == 4;

disp("The system is observable: ")
disp(obsv)

%% 

% initial conditions
x0 = [0; 0; 0; 0];
x0hat = [0.1; 0.8; 0.5; 0.6];

K = lqr(A, B, Q, R);

% place poles for observer
poles = [-6, -7 -8, -9];
Ktmp = place(transpose(A), transpose(C), poles);
K0 = transpose(Ktmp);

[xhatL, xL] = CartSys(0, 200, 0.01, x0, x0hat, K, K0, C);


figure(4)
subplot(121)
plot(t200, ref_pos, "Color", "black", "LineStyle","--")
hold on
plot(t200, xL(1, :), "Color", "red")
hold on
plot(t200, xhatL(1, :), "Color", "blue")
xlabel("Time [s]")
ylabel("Position [m]")
title("Position Comparison of Observed Cart System")
legend(["Reference Position", "Actual States", "Observed States"])
hold off
subplot(122)
plot(t200, (xL(1, :)-xhatL(1, :)) * 100)
title("Position Error Dyanmics")
xlabel("Time [s]")
ylabel("Position Error [%]")

figure(5)
plot(t200, xL)
hold on
plot(t200, xhatL)
hold off
title("Position Error Dyanmics")
xlabel("Time [s]")
ylabel("States")
legend(["x", "phi", "xdot", "phidot", "x_hat", "phi_hat", "xdot_hat", "phidot_hat"])

function [xhatL, xL] = CartSys(t0, tf, step, x0, xhat0, K, K0, C)

    % create the time inc of the whole model -> start:step:end
    time_inc = t0:step:tf;

    % init 3 steps to take in the smaller
    inner_step = step/3;

    % init x, xhat, u
    xL(:, 1) = x0;
    xhatL(:, 1) = xhat0;
    negKhat(1) = -K * xhat0;

    for t = 1:length(time_inc)-1

    % find time step of the outer loop
    time_at_step = time_inc(t);

    % create a small time interval to find x, xhat
    inner_inc = time_at_step:inner_step:(time_at_step+inner_step);

    % find the current state of
    x_curr = xL(:, t);

    % get the current input from the list of inputs
    negKhat_curr = negKhat(t);

    % integrate to find x across this step
    [~, step_x] = ode45(@(inner_inc, x_curr) cartController(inner_inc, x_curr, negKhat_curr, K), inner_inc, x_curr);

    % get the state at the end of the ode function
    new_x = step_x(end, :);

    % update x
    xL(:, t+1) = new_x;

    % get the output of the system at the end of the step
    curr_y = C * new_x.';

    % get the xhat for this current time step
    xhat_curr = xhatL(:, t);

    % integrate to find new xhat for th
    [~, step_xhat] = ode45(@(inner_inc, step_obs_state) cartObserver(inner_inc, step_obs_state, curr_y, negKhat_curr, K, K0), inner_inc, xhat_curr);

    % get the xhat at the end of the step
    new_xhat = step_xhat(end, :);

    % update xhat
    xhatL(:, t+1) = new_xhat;

    % update - k * xhat;
    negKhat(t+1) = -K * new_xhat.';

    end
end

function xhatdot = cartObserver(t, xhat, y, negKhat, K, K0)

    A = [0 0 1 0;
    0 0 0 1;
    0 1 -3 0;
    0 2 -3 0];


    B = [0;
        0;
        1;
        1];

    C = [1 0 0 0];

    ydes = (20) * square(2 * pi * (1/100) * t);

    v = inv(C * inv(A - B * K) * B) * ydes;

    y_des_last = (20) * square(2 * pi * (1/100) * (t - 0.001));

    u = v + negKhat;

    xhatdot = (A * xhat) + (B * u) + (K0 * (y - C * xhat));
end

% Cart Sys, change to car state
function xdot = cartController (t, x, negKhat, K)

    A = [0 0 1 0;
    0 0 0 1;
    0 1 -3 0;
    0 2 -3 0];

    B = [0;
        0;
        1;
        1];

    C = [1 0 0 0];

    ydes = (20) * square(2 * pi * (1/100) * (t));

    v = inv(C * inv(A - B * K) * B) * ydes;

    
    u = v + negKhat;
    
    xdot = (A * x) + (B * u);

end