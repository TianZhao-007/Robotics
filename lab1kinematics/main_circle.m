addpath("../engn4627_pibot_simulator/"); % Add the simulator to the MATLAB path

% Create a simulator instance with the circles floor
pb = piBotSim("floor_circle.jpg");

pb.place([2.5;1.5],0);

total_time = 15;
u = 2*pi / total_time;
q = 2*pi/total_time;

scale_parameter = 5.33e-3;
wheel_track = 0.156;

left_wheel_speed = (u * 2/scale_parameter - q * wheel_track / scale_parameter)/2;
right_wheel_speed = (u * 2/scale_parameter + q * wheel_track / scale_parameter)/2;

pb.setVelocity([left_wheel_speed  right_wheel_speed],total_time);

% Place the robot at the bottom of a circle
% Drive a circle of radius 1m
% Place the robot at the bottom of another circle
% Drive a circle of radius 2m
