addpath("../engn4627_pibot_simulator/"); % Add the simulator to the MATLAB path

% Create a simulator instance with the squares floor
pb = piBotSim("floor_square.jpg");

% Place the robot at the start of a square
pb.place([1.5;1.5], 0);


% Drive a square of side length 2m
drive_square(pb, 2);


% Place the robot at the start of another square
pb.place([2.0;2.0], 0);

% Drive a square of side length 1m
drive_square(pb, 1);
