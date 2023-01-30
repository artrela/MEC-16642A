% Title: PS4 Fall 2022
% File Topic: HTM, Jacobians, Manipulation
% Author: Alec Trela 
% Date: 28, November 2022
% ============================================


%% Question 1

clc 
clear

% G1: whole translation
R1 = diag([1, 1, 1]);
d1 = [0; 5; 6];

G1 = [R1, d1; 
      0 0 0 1];

% G2: align z axis w/ rotation of 90 deg about relative x
theta2 = 90 * (pi / 180);

d2 = [0; 0; 0];
R2 = [1,           0,            0;
      0, cos(theta2), -sin(theta2);
      0, sin(theta2),  cos(theta2)];

G2 = [R2, d2;
      0, 0, 0, 1];

% G3: align x, y w/ rotation of -60 about relative z
theta3 = -60 * (pi / 180);

d3 = [0; 0; 0];
R3 = [cos(theta3), -sin(theta3), 0;
      sin(theta3),  cos(theta3), 0;
      0          ,            0, 1];

G3 = [R3, d3;
      0, 0, 0, 1];


% Combine intermediate HTM to find final H0_1
H0_1 = G1 * G2 * G3;

disp("The Transformation H0_1:")
disp(H0_1)

%% Question 2
clc 
clear

A = [1, 0, 4;
    0, 1, 0; 
    0, 0, 1];

B = [0.866, 0.5, 0;
    -0.5, 0.866, 0;
    0, 0, 1];

shape = [[-1, 0, 1, 0, -1, -1]; [ 1, 1, 0, -1, -1, 1]; [1, 1, 1, 1, 1, 1]];

% (a) A, relative to the fixed frame

shape_a = zeros(3, 5);
for i = 1:size(shape, 2)

    shape_a(:, i) = A * shape(:, i);

end

% (b) A, relative to the fixed frame, followed by b, relative to the
% current frame

shape_b = zeros(3, 5);
for i = 1:size(shape, 2)

    shape_b(:, i) = A * B * shape(:, i) ;


end

% (c) A, relative to the fixed frame, followed by B, relative to the fixed frame.

shape_c = zeros(3, 5);
for i = 1:size(shape, 2)

    shape_c(:, i) = B * A * shape(:, i) ;

end

% (d) B, relative to the fixed frame

shape_d = zeros(3, 5);
for i = 1:size(shape, 2)

    shape_d(:, i) = B * shape(:, i) ;

end


% (e) B, relative to the fixed frame, followed by A, relative to the fixed frame.

shape_e = zeros(3, 5);
for i = 1:size(shape, 2)

    shape_e(:, i) = A * B * shape(:, i) ;

end


% (f) B, relative to the fixed frame, followed by A, relative to the current frame.
shape_f = zeros(3, 5);
for i = 1:size(shape, 2)

    shape_f(:, i) = B * A * shape(:, i) ;

end


% plot results
figure(1)
subplot(2, 3, 1)
plot(shape_a(1, :), shape_a(2, :), LineWidth=2)
hold on 
plot(shape(1, :), shape(2, :), LineWidth=2)
hold off
xlim([-2 6])
ylim([-4 2])
title("A, relative to the fixed frame")

subplot(2, 3, 2)
plot(shape_b(1, :), shape_b(2, :), LineWidth=2)
hold on 
plot(shape(1, :), shape(2, :), LineWidth=2)
hold off
xlim([-2 6])
ylim([-4 2])
title(["A, relative to the fixed frame,", "followed by b, relative to the current frame"])

subplot(2, 3, 3)
plot(shape_c(1, :), shape_c(2, :), LineWidth=2)
hold on 
plot(shape(1, :), shape(2, :), LineWidth=2)
hold off
xlim([-2 6])
ylim([-4 2])
title(["A, relative to the fixed frame,", "followed by b, relative to the fixed frame"])

subplot(2, 3, 4)
plot(shape_d(1, :), shape_d(2, :), LineWidth=2)
hold on 
plot(shape(1, :), shape(2, :), LineWidth=2)
hold off
xlim([-2 6])
ylim([-4 2])
title("B, relative to the fixed frame")

subplot(2, 3, 5)
plot(shape_e(1, :), shape_e(2, :), LineWidth=2)
hold on 
plot(shape(1, :), shape(2, :), LineWidth=2)
hold off
xlim([-2 6])
ylim([-4 2])
title(["B, relative to the fixed frame,", "followed by A, relative to the fixed frame"])

subplot(2, 3, 6)
plot(shape_f(1, :), shape_f(2, :), LineWidth=2)
hold on 
plot(shape(1, :), shape(2, :), LineWidth=2)
hold off
xlim([-2 6])
ylim([-4 2])
title(["B, relative to the fixed frame,", "followed by A, relative to the current frame"])


%% Question 6.1

syms d2;
syms theta1;
syms theta3;

theta2 = pi/2;

% use the standard form of taking row from dh table to htm 
H0_1 = DH2HTM(theta1, d2+10, 0, 0);

H1_2 = DH2HTM(0, 0, theta2, 9);

H2_3 = DH2HTM(theta3, 0, 0, 5);

% combine htm to obtain h0_3, from which d0_3 will be taken to obtain the 
% jacobian
H0_3 = vpa((H0_1 * H1_2 * H2_3), 2);
d0_3 = H0_3(1:3, 4);

disp("Displacement Vector d0_3 used for Direct Diff:")
disp(d0_3)


%% Question 6.2

clc

% Column 1: revolute joint 1: R0_i-1 [0;0;1] x (d0_n - d0_i-1)
R0_0 = diag([1;1;1]);
d0_0 = [0;0;0];
col1 = cross(R0_0 * [0; 0; 1], d0_3 - d0_0);

% Column 2: prismatic joint 2: R0_i-1 [0;0;1]
R0_1 = H0_1(1:3, 1:3);
col2 = R0_1 * [0;0;1];

% Column 3: revolute joint 3: R0_i-1 [0;0;1] x (d0_n - d0_i-1)
H0_2 = H0_1 * H1_2;
d0_2 = H0_2(1:3, 4);
R0_2 = H0_2(1:3, 1:3);
col3 = cross(R0_2 * [0;0;1], d0_3 - d0_2);

colbycolJ = vpa([col1, col2, col3], 2)


% fcn for converting a row of the dh table into an htm
function HTM = DH2HTM(theta_i, d_i, alpha_i, a_i)
    
    HTM = [cos(theta_i), -sin(theta_i) * cos(alpha_i), sin(theta_i) * sin(alpha_i), a_i * cos(theta_i);
            sin(theta_i), cos(theta_i) * cos(alpha_i), -cos(theta_i) * sin(alpha_i), a_i * sin(theta_i);
            0, sin(alpha_i), cos(alpha_i), d_i;
            0, 0, 0, 1];
end




