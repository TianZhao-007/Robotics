addpath("../engn4627_pibot_simulator/"); % Add the simulator to the MATLAB path.
pb = piBotSim("floor_circle.jpg"); % Start a simulator


% Place the robot at the start of the line
pb.place([2.5;2.5], 0);


% Run the solution script
run("follow_line.m");

% Close figures and clear variables
close all
clear all

% Check the solution quality
% Load the robot trail
load("robot_trail.mat", "simRobotTrail");
% Remove nan values
simRobotTrailRows = ~any(isnan(simRobotTrail), 1);
simRobotTrail = simRobotTrail(:, simRobotTrailRows);

N = size(simRobotTrail, 2);
line_points = line_spiral(1000);

error_sum = 0.0;

for i = 1:N
    trail_point = simRobotTrail(1:2,i);
    error = min(vecnorm(line_points - trail_point));
    error_sum = error_sum + error^2;
end

RMSE = sqrt(error_sum / N);
time = N * 0.1;

function points = line_spiral(n)
    % Set t limits
    t = linspace(0, 1.25*pi, n);
    
    % Create parameter curve
    x = 0.5*t.*sin(exp(0.75*t)) + 2.5;
    y = 0.5*t.*cos(exp(0.75*t)) + 2.5;
    points = [x;y];
end
