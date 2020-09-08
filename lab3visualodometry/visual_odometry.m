% Always begin by using addpath
addpath("../engn4627_pibot_simulator")

% For testing, we can tell the simulator where we want to place our
% landmarks. Let's try a grid formation
[lmx,lmy] = meshgrid(0.5:(4/3):4.5);
landmarks = [lmx(:)'; lmy(:)'];

% Now, we can start the simulator with these landmarks.
% You can also try set your own landmarks, or, if you leave it blank, the
% simulator will generate landmarks randomly.
pb = piBotSim("floor_spiral.jpg", landmarks);
pb.showLandmarks(true); % Use this to turn on/off showing the landmarks.
pb.showViewRange(true); % Use this to turn on/off showing the pibot view.

% Place your robot at the centre of the room
pb.place([2;2],0);

% Which landmarks can we measure? Try the measurement function
[landmarkPoints, landmarkIds] = pb.measureLandmarks()
% How accurate are these measurements? While testing, you can know where 
% the landmarks are, so you can check what kind of error the measurements
% have. How is the error distributed?


% Now you can use these measurements in your odometry observer!
% You will also need to use your known input velocity to the robot.
% I strongly suggest you try some simple examples before you try to follow
% a line, e.g. drive in a straight line for a few seconds.
% This will also help to evaluate your solution!