% Always begin by using addpath
addpath("..\engn4627_pibot_simulator")

% data.mat Figure 4 case 2
landmarks = [[1.5;1],[4;2]...
                [4.5;1.75],[4.75;2.75],...
                [3;3],[2.25;2.25],[1.5;1.75]];

landmarks = [landmarks,[2;1],[2.5;1],[3.5;3],[4;3]];

% data.mat figure 4 case 3
% landmarks = [[1.5;1],[3;1],...
%                 [3.5;2.25],[4.5;1.75],[4.75;2.75],...
%                 [4;3.5],[3;3],[2.25;2.25],[1.5;1.75]];
% landmarks = [landmarks,[3.5;1],[3.5;1.5],[4;1.5],[4;2.5],[2.5;2.75]];
            
% data_1.mat Figure 3
% landmarks = [[1.5;2.5],[3;2.5],[4.5;2.5],[3;3.5],[3;4.5],[3;1.5],[3;0.5]];

% data_2.mat Figure 3 
% landmarks = 1*[cos(pi/4:pi/4:2*pi);sin(pi/4:pi/4:2*pi)] + [2.5;2.5];
% landmarks = [landmarks,2*[cos(pi/4:pi/4:2*pi);sin(pi/4:pi/4:2*pi)] + [2.5;2.5]];

data = [];
a= [];

% initialise the position 
x0 = 1;
y0 = 1;
theta0 = 0;

pb = piBotSim('floor_course.jpg', landmarks);
pb.place([x0;y0],theta0);
figure;
camAxes = axes();
start_time = tic;

while true

    % First, get the current camera frame
    img = pb.getCamera();
%     imshow(img,'parent',camAxes);
    img = rgb2gray(img);
    img = img(end-30:end,:);
    line_img = ~imbinarize(img,0.2);
    imshow(line_img, "Parent", camAxes); % Check the video
    
    [rows,cols] = find(line_img);
    
        % Integrate using expm
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
    u = 0.08 * (1-x_mean^2); % replace with computed values!
    q = -1.3* x_mean; % replace with computed values!
   
    % Compute the required wheel velocities
    [wl, wr] = inverse_kinematics(u,q);
     a.wheel_left_velocity = wl;
     a.wheel_right_velocity = wr;
     a.u = u;
     a.q = q;
    [lms, ids] = pb.measureLandmarks();
    a.lms = [lms;ones(1,numel(ids));ids];
    elapsed_time = toc(start_time);
    % Apply the wheel velocities
    pb.setVelocity(wl,wr);   
     start_time = tic;
    a.delta_time = elapsed_time;
    data = [data, a];
end
 for j = 1:size(data)
     if j == 1
         data(1).delta_time = 0;
     end
     if j == size(data)
         break
     else
     data(j).delta_time=data(j+1).delta_time;
     end
 end
 
save data_fig4case2.mat

function [wl, wr] = inverse_kinematics(u, q)

scale_parameter =  5.33e-3;
wheel_track =  0.156; 

wl = (1/scale_parameter) * (u-wheel_track/2*q);
wr = (1/scale_parameter) * (u+wheel_track/2*q);
wl = round(wl);
wr = round(wr);
end
