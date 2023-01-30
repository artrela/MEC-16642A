function M = pfTemplate()
% template and helper functions for 16-642 PS3 problem 2
clc 
clear

rng(0); % initialize random number generator

b1 = [5,5]; % position of beacon 1
b2 = [15,5]; % position of beacon 2

% load pfData.mat
load("pfData.mat", "q_groundTruth", "t", "u", "y")

% initialize movie array
numSteps = 60;
M(numSteps) = struct('cdata',[],'colormap',[]);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%         put particle filter initialization code here                    %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Create n particles with random values that could occupy the environment
nParticle = 300;
particles = [20 * rand(1, nParticle); 10 * rand(1, nParticle); (2 * pi) * rand(1, nParticle)];

% Set the time step
T = t(2) - t(1);


% =============== PARAMETERS TO TUNE: V , W ==================== 

covV = [1.1, 0;
        0, 1.25];

covW = [0.5, 0;
        0, 0.5];

% ===============================================================


% here is some code to plot the initial scene
figure(1)
plotParticles(particles); % particle cloud plotting helper function
hold on
plot([b1(1),b2(1)],[b1(2),b2(2)],'s',...
    'LineWidth',2,...
    'MarkerSize',10,...
    'MarkerEdgeColor','r',...
    'MarkerFaceColor',[0.5,0.5,0.5]);
drawRobot(q_groundTruth(:,1), 'cyan'); % robot drawing helper function
axis equal
axis([0 20 0 10])
M(1) = getframe; % capture current view as movie frame
pause
hold off
disp('hit return to continue')


% iterate particle filter in this loop

% Randomly sample process noise



for k = 2:numSteps - 1

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %              put particle filter prediction step here               %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Use the provided motion model to see how each particle moves
    for i = 1:length(particles)
        
        v = mvnrnd([0; 0], covV);

        particles(1, i) = particles(1, i) + T * (u(1, k) + v(1)) * cos(particles(3, i));
        particles(2, i) = particles(2, i) + T * (u(1, k) + v(1)) * sin(particles(3, i));
        particles(3, i) = particles(3, i) + T * (u(2, k) + v(2));

    end 
 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %                put particle filter update step here                 %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Our sensor readings are in distance to beacons, but are states are in
    % x, y, theta -> translate state into sensor reading values
    for i = 1:length(particles)

        particle_dist(1, i) = sqrt( (particles(1, i) - b1(1) )^2 + ( particles(2, i) - b1(2))^2 );
        particle_dist(2, i) = sqrt( (particles(1, i) - b2(1) )^2 + ( particles(2, i) - b2(2))^2 );
    
    end
   
    % Weight the particles: p(y | xhat)
    for i = 1:length(particles)
       
       weights(i) = mvnpdf(particle_dist(:, i), y(:, k), covW);

    end

    % find the heighest weight particle: most likely state
    highestPpart = find( weights == max(weights) );
    best_particle = particles(:, highestPpart(1));

    % find the average particle by summing all states
    average_particle(1) = sum(particles(1, :)) / length(particles);
    average_particle(2) = sum(particles(2, :)) / length(particles);
    average_particle(3) = sum(particles(3, :)) / length(particles);
    
    % Normalize the weights to 1
    weights = weights ./ sum(weights);

    % Take a cumulative sum
    CW = cumsum(weights);
  
    for i = 1:length(particles)

        % Generate random number 0, 1
        z = rand(1);
        
        % i so that CWi is the smallest cummulative weight in CW > z
        ind = (find(CW > z));
        ind = ind(1);
        
        % Assign the jth particle with the ith particle in the next cloud
        next_cloud(:, i) = particles(:, ind);

    end
    
    % reassign particles with the new cloud
    particles = next_cloud;
 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % plot particle cloud, robot, robot estimate, and robot trajectory here %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    clf(1)
    hold on
    plotParticles(particles)
    plot(average_particle(1), average_particle(2), "bo", LineWidth=3)
    plot(best_particle(1), best_particle(2), "ro", LineWidth=3)
    drawRobot(q_groundTruth(:, k), "cyan")
    plot([b1(1),b2(1)],[b1(2),b2(2)],'s',...
    'LineWidth',2,...
    'MarkerSize',10,...
    'MarkerEdgeColor','r',...
    'MarkerFaceColor',[0.5,0.5,0.5]);
    axis equal
    axis([0 20 0 10])
    plot(q_groundTruth(1, 1:k), q_groundTruth(2, 1:k), "black")
    hold off

    % capture current figure and pause
    M(k) = getframe; % capture current view as movie frame
    pause
    disp('hit return to continue')
        
end

% when you're ready, the following block of code will export the created 
% movie to an mp4 file
videoOut = VideoWriter('../result.mp4','MPEG-4');
videoOut.FrameRate=5;
open(videoOut);
for k=1:numSteps
  writeVideo(videoOut,M(k));
end
close(videoOut);



% helper function to plot a particle cloud
function plotParticles(particles)
plot(particles(1, :), particles(2, :),"go")
line_length = 1;
quiver(particles(1, :), particles(2, :), line_length * cos(particles(3, :)), line_length * sin(particles(3, :)))


% helper function to plot a differential drive robot
function drawRobot(pose, color)
    
% draws a SE2 robot at pose
x = pose(1);
y = pose(2);
th = pose(3);

% define robot shape
robot = [-1 .5 1 .5 -1 -1;
          1  1 0 -1  -1 1 ];
tmp = size(robot);
numPts = tmp(2);
% scale robot if desired
scale = 0.5;
robot = robot*scale;

% convert pose into SE2 matrix
H = [ cos(th)   -sin(th)  x;
      sin(th)    cos(th)  y;
      0          0        1];

% create robot in position
robotPose = H*[robot; ones(1,numPts)];

% plot robot
plot(robotPose(1,:),robotPose(2,:),'k','LineWidth',2);
rFill = fill(robotPose(1,:),robotPose(2,:), color);
alpha(rFill,.2); % make fill semi transparent

