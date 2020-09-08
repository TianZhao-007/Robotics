% Calibrate the scale parameter and wheel track of the robot
addpath("../simulator/"); % Add the simulator to the MATLAB path.
pb = piBotSim("floor_spiral.jpg");

% Write your code to compute identify and follow lines below.

% Start by placing your robot at the start of the line
pb.place([2.5;2.5], 0);

% Create a window to visualise the robot camera
figure;
camAxes = axes();

% [position,angler] = pb.measure;
% state = [position(1);position(2);angler];

% Follow the line in a loop

while true
    % First, get the current camera frame
    img = pb.getCamera();
    imshow(img,'parent',camAxes);
    img = rgb2gray(img);
    img = img(end-30:end,:);
    line_img = ~imbinarize(img,0.2);
    %imshow(line_img, "Parent", camAxes); % Check the video
    
    [rows,cols] = find(line_img);
    
    if isempty(rows)||isempty(cols)
        u = 0.03;
        q = 0.0;
        [wl,wr] = inverse_kinematics(u,q);
        pb.setVelocity([wl,wr],1);
        break
    end
    
    x_mean = (mean(cols) - 400/2) / 400;
    y_mean = (mean(rows) - 31/2) /31;
    % Use the line centre to compute a velocity command
    u = 0.15 * (1-2*x_mean^2); % replace with computed values!
    q = -8 * x_mean; % replace with computed values!

   % new_state = integrate_kinematics(state, 0.1, u,q );
   % state = new_state
   % t = t +0.1
   
    % Compute the required wheel velocities
    [wl, wr] = inverse_kinematics(u,q);
    
    % Apply the wheel velocities
    pb.setVelocity(wl,wr);
    

end

pb.saveTrail();
% Save the trajectory of the robot to a file.
% You must use this function at the very end of your script, or the
% evaluation system will not recognise that you have completed the line
% following task.