addpath("../engn4627_pibot_simulator/"); % Add the simulator to the MATLAB path.

% Choose landmarks from a distribution
number_landmarks = 20 + randi(10);
landmark_positions = rand(2,number_landmarks) .* [4;3] + [0.5;0.5];

compute_armse(landmark_positions, landmark_positions + randn(size(landmark_positions)))

% Note that, to keep things fair, the landmarks will not be random during
% real testing. However, you cannot rely on knowledge of the number of
% landmarks in your code, or the positions of those landmarks.
% That is, your code must not contain any references to the number of
% landmarks or the landmark positions defined here. Your code must be able
% to add landmarks to the state as it completes the line course.


pb = piBotSim("floor_course.jpg", landmark_positions); % Start a simulator

% Place the robot at the start of the line
pb.place([1.0;1.0], 0.0);


% Run the solution script
run("ekf_slam.m");
pb.saveTrail();

% Your code must define the variables 'estimated_landmarks' and
% 'estimated_trajectory'. The first must be 2xN where there are N
% landmarks. The second must be 3xT where the robot has been running for T
% timesteps. If your code does not do this, the assertions below will raise
% errors.


% Close figures and clear unnecessary variables
close all
clearvars -except landmark_positions estimated_landmarks estimated_trajectory

assert(all(size(estimated_landmarks) == size(landmark_positions)), "Estimated and true landmarks are not the same size.");
assert(size(estimated_trajectory,1) == 3, "Estimated trajectory does not have 3 rows.");

% Check the solution quality

% Load the true robot trajectory
load("robot_trail.mat", "simRobotTrail");
simRobotTrailRows = ~any(isnan(simRobotTrail), 1);
simRobotTrail = simRobotTrail(:, simRobotTrailRows);

% Match the trajectories. Note they are assumed to start at the same time.
% The estimated trajectory MUST start as soon as the robot begins
% simulation.
trajectory_length = min(size(simRobotTrail, 2), size(estimated_trajectory,2));
simRobotTrail = simRobotTrail(:,1:trajectory_length);
estimated_trajectory = estimated_trajectory(:,1:trajectory_length);

% Compute the trajectory error
estimated_positions = estimated_trajectory(1:2,:);
true_positions = simRobotTrail(1:2,:);
trajectory_armse = compute_armse(true_positions, estimated_positions);

% Compute the landmark error
landmarks_armse = compute_armse(landmark_positions, estimated_landmarks);

% Print the results
fprintf("The trajectory and landmark RMSE are:\n%d, %d\n", trajectory_armse, landmarks_armse);

function armse = compute_armse(points1, points2)
% Compute the aligned RMSE between two matched sets of 2D points
n = size(points1,2);
assert(all(size(points1)==[2,n]));
assert(all(size(points2)==[2,n]));

mu1 = mean(points1,2);
mu2 = mean(points2,2);

Sig = 1/n * (points2-mu2) * (points1-mu1)';

[U,~,V] = svd(Sig);
A = eye(2);
if det(Sig) < 0
    A(2,2) = -1;
end

R = V * A * U';
x = mu1 - R * mu2;

points1_aligned = R' * (points1 - x);

armse = real(sqrt(1/n * sum((points1_aligned - points2).^2, 'all')));

end
