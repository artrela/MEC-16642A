% Title: Problem Set 3
% File Topic: EKF
% Author: Alec Trela 
% Date: 11, November 2022
% ============================================

clc
clear

% load sim data
load("calibration.mat")
load("kfData.mat")

% Time Step
T = 0.01;

% ===================== Find process noise ================================

% Linearize the system about each state
for i = 1:length(q_groundtruth) - 1

    % Linearized approx of the system: q[k+1] = F(q[k], u[k])q[k] + G(q[k])u[k] + Γ(q[k])v[k]
    F_lin(:, :, i) = [1, 0, -T * (u(1, i)) * sin(q_groundtruth(3, i));
                      0, 1,  T * (u(1, i)) * cos(q_groundtruth(3, i));
                      0, 0,                      1];

    G_lin(:, :, i) = [T * cos(q_groundtruth(3, i)), 0;
                      T * sin(q_groundtruth(3, i)), 0;
                      0            , T];

    tau_lin(:, :, i) = [T * cos(q_groundtruth(3, i)), 0;
                        T * sin(q_groundtruth(3, i)), 0;
                        0            , T];
    
    % Derive the process noise by rearranging terms: 
    % v[k] = pinv( Γ(q[k]) ) * ( q[k+1] - F(q[k], u[k])q[k] - G(q[k])u[k] )
    process_noise(i, :) = tau_lin(:, :, i) \ ( q_groundtruth(:, i + 1) - F_lin(:, :, i) * q_groundtruth(:, i) - G_lin(:, :, i) * u(:, i) );


end

% Generate the covarience of the process noise 
covV = cov(process_noise);

disp("Process Covariance (V):")
disp(covV)

% ======================== find sensor noise ===============================

% Find the y that correponds to q ground truth
% i.e. not all time points have sensor data
for i = 1:length(t_y)
    
    j = find(t_groundTruth == t_y(i));
    q_relevant(1:2, j) = q_groundtruth(1:2, j);   
    
end

% Additional filtering step: find() doesn't remove zeros
a = q_relevant(1, q_relevant(1, :) ~= 0);
b = q_relevant(2, q_relevant(2, :) ~= 0);
q_relevant = [a;b];

% Find the noise covariance
% i.e. how different is the sens
covW = cov(transpose(y - q_relevant));
% Scale down covariance W
%covW = covW / 100;

disp("Sensor Covariance (W):")
disp(covW)

% ======================= Dead Reckoner ==================================

% Generate random noise (Gaussian White: N(0, W), N(0, V)
w = mvnrnd([0, 0], covW, length(t));
v = mvnrnd([0, 0], covV, length(t));


% Set initial conditions
qhat_i = [0.355; -1.590; 0.682];
q_dr(:, 1) = qhat_i;


for i = 1:length(t) - 1

    % EKF doesn't need to linearize to predict the mean
    q_dr(1, i + 1) = q_dr(1, i) + T * (u(1, i) + v(i, 1)) * cos(q_dr(3, i));
    q_dr(2, i + 1) = q_dr(2, i) + T * (u(1, i) + v(i, 1)) * sin(q_dr(3, i));
    q_dr(3, i + 1) = q_dr(3, i) + T * (u(2, i) + v(i, 2));
   
end

% Plot Dead Reckoner
% Ground truth plot
figure(1)
plot(q_dr(1, :), q_dr(2, :))
hold on 
plot(q_groundtruth(1, :), q_groundtruth(2, :), LineWidth=2)
hold off
xlabel("Positon X (m)")
ylabel("Position Y (m)")
legend('Dead Reckoner Estimate', "Ground truth")
title("Dead Reckoner Position Estimate")

%  ================= Full Extended Kalman Filter ==========================

% Set initial conditions for covariance
P =  [25, 0, 0;
      0, 25, 0;
      0, 0, 0.154];

% H, h are the same here: we only want the first two states in the output
H = [1, 0, 0;
     0, 1, 0];

% init state 
q_ekf(:, 1) = qhat_i;
for i = 1:length(t) - 1

    % EKF doesn't need to linearize to predict the mean
    q_ekf(1, i + 1) = q_ekf(1, i) + T * (u(1, i) + v(i, 1)) * cos(q_ekf(3, i));
    q_ekf(2, i + 1) = q_ekf(2, i) + T * (u(1, i) + v(i, 1)) * sin(q_ekf(3, i));
    q_ekf(3, i + 1) = q_ekf(3, i) + T * (u(2, i) + v(i, 2));

   
     %update the covariance matrix (we have F, Tau from deriving process
     %noise)
     P = F_lin(:, :, i) * P * transpose(F_lin(:, :, i)) +  tau_lin(:, :, i) * covV * transpose(tau_lin(:, :, i));

    if ismember(t(i), t_y)
        
        GPS = y(:, i/10);

        %update step: find S
        S = H * P * transpose(H) + covW;
        
        %update x
        q_ekf(:, i + 1) = q_ekf(:, i + 1) + P * transpose(H) * inv(S) * (GPS - H * q_ekf(:, i + 1));
        
        %covarience update
        P = P - P * transpose(H) * inv(S) * H * P;

    end

end


% plot EKF
figure(2)
plot(q_groundtruth(1, :), q_groundtruth(2, :))
hold on
plot(q_ekf(1, :), q_ekf(2, :))
hold on 
scatter(y(1, :), y(2, :))
hold off
legend(["Ground Truth", "EKF Estimate of State", "GPS Readings"])
xlabel("Positon X (m)")
ylabel("Position Y (m)")
title("EKF Position Estimate")
