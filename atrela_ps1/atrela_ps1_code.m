% MEC Problem Set 1 | Alec Trela | Sept. 13, 2022

clc
clear


%%  ----------------------- Question 1 -----------------------------

A_1 = [0 1 0; 
    0 0 1; 
    1 5 7];

B_1 = [1;
    0;
    0];

C_1 = [0 1 3];

% 1.a: Is the system stable?

% find the eigenvales, and check if their all real parts are negative.
e_val = eig(A_1);

stable = checkStability(e_val);

disp("The system in Q1.a is stable:")
disp(stable)

% 1.b: Is the system controllale?

% Create the matrix Q = [B | AB | A^2B]
AB = A_1 * B_1;
A2B = A_1 * A_1 * B_1;
Q = [B_1 AB A2B];

% Q is on dim n x nm  (3 x 2) and if it is full rank (3) it is controllable
controllable = rank(Q) == size(A_1, 2);
disp("The system in Q1.a is controllable:")
disp(controllable)

% 1.c Plot the unforced system t = [0 2]
x_o_1 = [0; 1; 0];
t2 = 0: 0.01: 2;

% obtain the linearization for the sys
for t = 1:length(t2)
    eAt = expm(A_1 .* t2(t));
    x_tc(:, t) = eAt * x_o_1;
end
y_tc = C_1 * x_tc;


% 1.d Place the following poles {−1 + i, −1 − i, −2}
p = [-1+i, -1-i, -2]; % desired poles
K_1 = place(A_1, B_1, p); % set K with desired poles

A_1stable = A_1 - B_1*K_1;

eval_1d = eig(A_1stable);
disp("Check for correct poles for 1.d:")
disp(eval_1d)

stable_1d = checkStability(eval_1d);

disp("The system in Q1.d (after pole placement) is stable:")
disp(stable_1d)

% 1.e create a plot w/ t[0 10]
sys_UC = ss(A_1, B_1, C_1, []); % uncontrolled system



t10 = 0: 0.01: 10;
for t = 1:length(t10)
    eAt_e = expm(A_1stable .* t10(t));
    x_tC(:, t) = eAt_e * x_o_1;
end

y_t = C_1 * x_tC;

%% ----------------------- Unforced System Q1  ------------------------------
figure(1)
plot(t2, y_tc)
title("Unforced System States")
xlabel("Time [s]")
ylabel("Output")

%% ------------------------ Unstable System Q1 -------------------------------- 
% Plot the unstable system over time
figure(2)
step(sys_UC, 10) % plot the system from 0-10
title("Uncontrolled System")
xlabel("Time")
ylabel("Output")
% Plot the controlled system, which is now stable
figure(3)
plot(t10, y_t)
title("Controlled System via Pole Placement")
xlabel("Time")
ylabel("Output")


%%  ----------------------- Question 2 -----------------------------
% state space in the form x = [x, phi, xdot, phidt]

% 2.c Check stability for linearized system

A_2 = [0 0 1 0;
    0 0 0 1;
    0 1 -3 0;
    0 2 -3 0];


B_2 = [0;
    0;
    1;
    1];

% 2.d Apply LQR & Plot under feedback law
Q_2 = [1 0 0 0; 0 5 0 0; 0 0 1 0; 0 0 0 5];
R_2 = 10;

e_val_2 = eig(A_2);

stable_2 = checkStability(e_val_2);

disp("The system in Q2 is stable:")
disp(stable_2)

% Create the matrix Q = [B | AB | A^2B | A^3B]
Q_ = ctrb(A_2, B_2); % substitute for 

controllable_2 = rank(Q_) == size(A_2, 2);

disp("The system in Q1 is controllable:")
disp(controllable_2)

%%
% Use ODE45 on Controlled System
xo_a = [0, 0.1, 0, 0];
xo_b = [0, 0.5, 0, 0];
xo_c = [0, 1.0886, 0, 0];
xo_d = [0, 1.1, 0, 0];

% 30 second time step for linear/nonlinear cart  state plots
t30 = 0:0.01:30;

% Solve the linearized sys at each init cond
[t_aL, x_aL] = ode45(@(t30, xo_a) linSys(t30, xo_a, A_2, B_2, Q_2, R_2), t30, xo_a);
[t_bL, x_bL] = ode45(@(t30, xo_b) linSys(t30, xo_b, A_2, B_2, Q_2, R_2), t30, xo_b);
[t_cL, x_cL] = ode45(@(t30, xo_c) linSys(t30, xo_c, A_2, B_2, Q_2, R_2), t30, xo_c);
[t_dL, x_dL] = ode45(@(t30, xo_d) linSys(t30, xo_d, A_2, B_2, Q_2, R_2), t30, xo_d);

% 2.e plot the full nonlinear system, with control law u = kx(t)
[t_aNL, x_aNL] = ode45(@(t30, xo_a) nonlinSys(t30, xo_a, A_2, B_2, Q_2, R_2), t30, xo_a);
[t_bNL, x_bNL] = ode45(@(t30, xo_b) nonlinSys(t30, xo_b, A_2, B_2, Q_2, R_2), t30, xo_b);
[t_cNL, x_cNL] = ode45(@(t30, xo_c) nonlinSys(t30, xo_c, A_2, B_2, Q_2, R_2), t30, xo_c);
% Computer Stalled out at t30, tried visualize over 7 sec
t7 = 0:0.01:7;
[t_dNL, x_dNL] = ode45(@(t7, xo_d) nonlinSys(t7, xo_d, A_2, B_2, Q_2, R_2), t7, xo_d);

%% ----------------------- Linearized System Plots (2d) ----------------------------------------

% Init conditions A: xo = [0, 0.1, 0, 0]
figure(4)
plot(t_aL, x_aL)
legend(["x", "phi", "x-dot", "phi-dot"])
title("LQR Control of Linear System: xo = [0, 0.1, 0, 0]")
xlabel("Time [s]")
% Init conditions B: xo = [0, 0.5, 0, 0]
figure(5)
plot(t_bL, x_bL)
legend(["x", "phi", "x-dot", "phi-dot"])
title("LQR Control of Linear System: xo = [0, 0.5, 0, 0]")
xlabel("Time [s]")
% Init conditions C: xo = [0, 1.0886, 0, 0] Yep
figure(6)
plot(t_cL, x_cL)
legend(["x", "phi", "x-dot", "phi-dot"])
title("LQR Control of Linear System: xo = [0, 1.0866, 0, 0]")
xlabel("Time [s]")
% Init conditions C: xo = [0, 1.1, 0, 0] Yep
figure(7)
plot(t_dL, x_dL)
legend(["x", "phi", "x-dot", "phi-dot"])
title("LQR Control of Linear System: xo = [0, 1.1, 0, 0]")
xlabel("Time [s]")

%% ------------------------- Nonlinear Sys Plots (2e) ------------------------------------
% Init conditions A: xo = [0, 0.1, 0, 0]
figure(8)
plot(t_aNL, x_aNL)
title("LQR Control of Nonlinear System: xo = [0, 0.1, 0, 0]")
legend(["x", "phi", "x-dot", "phi-dot"])
xlabel("Time [s]")
% Init conditions B: xo = [0, 0.5, 0, 0]
figure(9)
plot(t_bNL, x_bNL)
title("LQR Control of Nonlinear System: xo = [0, 0.5, 0, 0]")
legend(["x", "phi", "x-dot", "phi-dot"])
xlabel("Time [s]")
% Init conditions C: xo = [0, 1.0886, 0, 0]
figure(10)
plot(t_cNL, x_cNL)
title("LQR Control of Nonlinear System: xo = [0, 1.0886, 0, 0]")
legend(["x", "phi", "x-dot", "phi-dot"])
xlabel("Time [s]")
% Init conditions C: xo = [0, 1.1, 0, 0]
figure(11)
plot(t_dNL, x_dNL)
title("LQR Control of Nonlinear System: xo = [0, 1.1, 0, 0]")
legend(["x", "phi", "x-dot", "phi-dot"])
xlabel("Time [s]")

%% Design a  Cart Controller 
clc
% 2.f find C matrix, y = Cx
C_2 = [39.3701 0 0 0]; % need the first state only

% create the reference square wave
t200 = 0:0.01:200;
ref_pos = (20) * square(2*pi*(1/100)*t200);

% Calculate the Cart Traj & ref path
xo_cart = [0; 0; 0; 0];
 
% solve for the position of the cart with ode45
[t_cart, cartStates] = ode45(@(t200, xo_cart) cartSys(t200, xo_cart, A_2, B_2, C_2, Q_2, R_2), t200, xo_cart);
pos_cart = (C_2 * cartStates.');
%%
% Q2.h tune the LQR controller 
Q_2h= [15 0 0 0; 0 1 0 0; 0 0 15 0; 0 0 0 1];
R_2h = 25;
[t_cartAdj, cartStatesAdj] = ode45(@(t200, xo_cart) cartSys(t200, xo_cart, A_2, B_2, C_2, Q_2h, R_2h), t200, xo_cart);
pos_cartAdj = (C_2 * cartStatesAdj.');

%% ------------------------ Cart Plot: Original LQR -----------------------------------------
figure(12)
plot(t_cart, ref_pos, "LineWidth", 2)
title("Cart Path")
xlabel("Time [s]")
ylabel("Position [in]")
xlim([-0.01 200])
ylim([-30 30])
hold on
plot(t_cart, pos_cart, "LineWidth", 2)
legend(["Reference Path", "Controlled Path"])

%% -------------------------- Cart Plot: Full Nonlinear Dynamics -------------------------------- 
figure(13)
plot(t_cart, cartStates, "LineWidth", 2)
title("Full Nonlinear Dynamics")
xlabel("Time [s]")
ylabel("States")
xlim([-0.01 200])
legend(["X", "Phi", "Xdot", "Phidot"])


%% ----------------------- Cart Plot: Adjusted LQR ---------------------------------------------- 
figure(14)
plot(t200, ref_pos, "LineWidth", 2)
title("Cart Path")
xlabel("Time [s]")
ylabel("Position [in]")
xlim([-0.01 200])
ylim([-30 30])
hold on
plot(t_cartAdj, pos_cartAdj, "LineWidth", 2)
hold on
plot(t_cart, pos_cart, "LineWidth", 2)
legend(["Reference Path", "Controlled Path: Altered LQR", "Controlled Path: Original LQR"])

%% plot the new states as well
figure(15)
plot(t_cartAdj, cartStatesAdj, "LineWidth", 2)
title("Full Nonlinear Dynamics")
xlabel("Time [s]")
ylabel("States")
xlim([-0.01 200])
legend(["X", "Phi", "Xdot", "Phidot"])

%%% Define Helper Functions %%%
function stable = checkStability(e_val)

    stable = true;
    for i = 1:length(e_val)
    
        re_eig = real(e_val(i));
    
        if (re_eig > 0 && stable == true)
    
            stable = false;
        
        end  
        
    end
    
end 

% Finds the k matrix via LQR and the controlled
% matrix of the linearized system
function x_dot = linSys(t, x, A, B, Q, R)
          
    [K, ~, ~] = lqr(A, B, Q, R);

    A_contr = A - B*K;
    
    x_dot = A_contr * x; 

end


function x_dot = nonlinSys(t, x, A, B, Q, R)
    gamma = 2;
    alpha = 1;
    beta = 1;
    D = 1;
    mu = 3;

    % Use lqr on the uncontrolled system
    [K, ~, ~] = lqr(A, B, Q, R);
    F = -K * x;
    
    den = gamma*alpha-(beta*cos(x(2)))^2;
    linAcc_num = (F * alpha) - (alpha * beta * (x(4)^2) *sin(x(2))) -(alpha * mu* x(3)) + (D * beta * sin(x(2)) * cos(x(2)));
    angAcc_num = (F*beta*cos(x(2))) - ((beta^2)*(x(4)^2)*sin(x(2))*cos(x(2))) - (mu*x(3)*beta*cos(x(2))) + (D * gamma * sin(x(2))) ;

    x_dot = [x(3); x(4);
        linAcc_num/den;
        angAcc_num/den];
    
    

end

% Cart Traj, change to car state
function cartStates = cartSys(t, x, A, B, C, Q, R)
    
    % Find the desired position, the "v" in the equation
    pos_des = (20)* square(2*pi*(1/100)*t);

    % Use lqr on the uncontrolled system
    [K, ~, ~] = lqr(A, B, Q, R);

    % stabalize the system with new poles
    A_stable = A - B*K;

    % Apply state space tracking controller
    ss_track = (-C*(A_stable)^-1 * B)^-1;

    % get xdot
    v = ss_track * pos_des;
    
    x_dot = (B.*v) + (A_stable * x);
    % multiple C by the transpose of states to get position
    cartStates = x_dot;
end 
