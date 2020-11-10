% Always begin with clear and close all
clear
close all
% Always begin by using addpath
addpath("../engn4627_pibot_simulator")
load("data.mat")

% landmarks for data.mat
landmarks = [[1.5;1],[3;1],...
    [3.5;2.25],[4.5;1.75],[4.75;2.75],...
    [4;3.5],[3;3],[2.25;2.25],[1.5;1.75]];

% landmarks for data_1.mat
% landmarks = [[1.5;2.5],[3;2.5],[4.5;2.5],[3;3.5],[3;4.5],[3;1.5],[3;0.5]];

% landmarks for data_2.mat
% landmarks = 1.5*[cos(pi/4:pi/4:2*pi);sin(pi/4:pi/4:2*pi)] + [2.5;2.5];

% landmarks for data_2.mat
% landmarks = 1*[cos(pi/4:pi/4:2*pi);sin(pi/4:pi/4:2*pi)] + [2.5;2.5];
% landmarks = [landmarks,2*[cos(pi/4:pi/4:2*pi);sin(pi/4:pi/4:2*pi)] + [2.5;2.5]];

% ground truth
t = 0:0.1:pi/2;
xTrace = [1,3,3+0.5*sin(t),4-0.5*cos(t),4+0.5*sin(2*t),4,3,3-0.5*sin(t),...
    2+0.5*cos(t),2-0.5*sin(t)];
yTrace = [1,1,1.5-0.5*cos(t),1.5+0.5*sin(t),2.5-0.5*cos(2*t),3,3,...
    2.5+0.5*cos(t),2.5-0.5*sin(t),1.5+0.5*cos(t)];

% please initialise the position
x0 = 1; y0 = 1; theta0 = 0;
pb = piBotSim('floor_course.jpg', landmarks);
pb.place([x0;y0],theta0);

%% wheel odometry 
figure;
trail_axes = gca();
trail = [];
plot(xTrace,yTrace,'b-','Parent',trail_axes);hold on;grid on;
%initial state: [x;y;theta]
state = [x0;y0;theta0];
% empty matrix for storing updated 'p'
pbar = zeros(3,numel(landmarks)/2);
xlim(trail_axes,[0,5]);
ylim(trail_axes,[0,5]);
axis(trail_axes,'manual');
% import data
for i = 1:numel(data)
    %     img = pb.getCamera();
    dt = data(i).delta_time;
    u = data(i).u;
    q = data(i).q;
    [wl,wr] = inverse_kinematics(u,q);
    pb.setVelocity(data(i).wheel_left_velocity,data(i).wheel_right_velocity);
    % [u,q] = wheel2rob(data(i).wheel_left_velocity,data(i).wheel_right_velocity);
    ybar = data(i).lms(1:4,:);
    % update state of robot
    state(:,i+1) = integrate_kinematics(state(:,i),dt,u,q);
    x = state(1,i+1);
    y = state(2,i+1);
    theta = state(3,i+1);
    pose = [cos(theta) -sin(theta) x;
        sin(theta) cos(theta) y;
        0 0 1];
    phat = pose * ybar(1:3,:);
    pbar(:,ybar(4,:)) = phat(1:3,:);
    plot(state(1,:),state(2,:),'r-','Parent',trail_axes); drawnow;hold on;
    % plot(pbar(1,:),pbar(2,:),'o','Parent',trail_axes);drawnow;hold on;
    
    len = size(data(i).lms);
    for j = 1:len(2)
        lm_serial = ybar(4,j);
        % plot the measured landmarks
        scatter(phat(1,j),phat(2,j),6,'MarkerFaceColor',box(lm_serial),...
            'MarkerEdgeColor','none');
        hold on;
        drawnow;
    end
    
end
%%
%  function [u,q] = wheel2rob(wl,wr)
%      scale_parameter = 5.33e-3;
%      wheel_track = 0.156;
%      u = scale_parameter/2 * (wl+wr);
%      q = scale_parameter/wheel_track*(wr-wl);
%  end

function new_state = integrate_kinematics(state, dt, lin_velocity, ang_velocity)

    new_state(3) = state(3) + dt * ang_velocity;
    if(ang_velocity == 0)
        new_state(1) = state(1) + dt * (lin_velocity*cos(state(3)));
        new_state(2) = state(2) + dt * (lin_velocity*sin(state(3)));
    else
        new_state(1) = state(1) + lin_velocity/ang_velocity*...
            (sin(new_state(3))-sin(state(3)));
        
        new_state(2) = state(2) + lin_velocity/ang_velocity*...
            (-cos(new_state(3))+cos(state(3)));
    end
    
end

function [wl, wr] = inverse_kinematics(u, q)

scale_parameter =  5.33e-3;
wheel_track =  0.156; 

wl = (1/scale_parameter) * (u-wheel_track/2*q);
wr = (1/scale_parameter) * (u+wheel_track/2*q);
wl = round(wl);
wr = round(wr);
end

 % enough color for presenting measured landmarks
 % for 100 landmarks
function [color] = box(num)
    map = [255 179 167; % red
        237 87 54;
        240 0 86;
        140 67 86;
        255 0 151;
        203 58 86;
        255 241 67; % yellow
        250 255 114;
        209 255 86;
        255 166 49;
        172 133 63;
        200 155 229;
        189 221 34; % green
        120 146 98;
        14 184 58;
        12 137 24;
        27 209 165;
        127 255 170;
        68 206 246; % blue
        23 124 176;
        255 179 167; % red
        237 87 54;
        240 0 86;
        140 67 86;
        255 0 151;
        203 58 86;
        255 241 67; % yellow
        250 255 114;
        209 255 86;
        255 166 49;
        172 133 63;
        200 155 229;
        189 221 34; % green
        120 146 98;
        14 184 58;
        12 137 24;
        27 209 165;
        127 255 170;
        68 206 246; % blue
        23 124 176;255 179 167; % red
        237 87 54;
        240 0 86;
        140 67 86;
        255 0 151;
        203 58 86;
        255 241 67; % yellow
        250 255 114;
        209 255 86;
        255 166 49;
        172 133 63;
        200 155 229;
        189 221 34; % green
        120 146 98;
        14 184 58;
        12 137 24;
        27 209 165;
        127 255 170;
        68 206 246; % blue
        23 124 176;255 179 167; % red
        237 87 54;
        240 0 86;
        140 67 86;
        255 0 151;
        203 58 86;
        255 241 67; % yellow
        250 255 114;
        209 255 86;
        255 166 49;
        172 133 63;
        200 155 229;
        189 221 34; % green
        120 146 98;
        14 184 58;
        12 137 24;
        27 209 165;
        127 255 170;
        68 206 246; % blue
        23 124 176;255 179 167; % red
        237 87 54;
        240 0 86;
        140 67 86;
        255 0 151;
        203 58 86;
        255 241 67; % yellow
        250 255 114;
        209 255 86;
        255 166 49;
        172 133 63;
        200 155 229;
        189 221 34; % green
        120 146 98;
        14 184 58;
        12 137 24;
        27 209 165;
        127 255 170;
        68 206 246; % blue
        23 124 176;255 179 167; % red
        237 87 54;
        240 0 86;
        140 67 86;
        255 0 151;
        203 58 86;
        ] / 255;
    color = map(num+1, :);
end
