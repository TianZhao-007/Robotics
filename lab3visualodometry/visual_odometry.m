% Always begin by using addpath
% addpath("../engn4627_pibot_simulator/")
load("data.mat")
% landmarks distribution
% data.mat
% landmarks = [[1.5;1],[3;1],...
%                 [3.5;2.25],[4.5;1.75],[4.75;2.75],...
%                 [4;3.5],[3;3],[2.25;2.25],[1.5;1.75]];

% datafig4case2.mat
% landmarks = [[1.5;1],[3;1],...
%                 [3.5;2.25],[4.5;1.75],[4.75;2.75],...
%                 [4;3.5],[3;3],[2.25;2.25],[1.5;1.75]];
% landmarks = [landmarks,[2;1],[2.5;1],[3.5;3],[4;3]];

%landmarks for data.mat figure 4 case 3
% landmarks = [[1.5;1],[3;1],...
%                 [3.5;2.25],[4.5;1.75],[4.75;2.75],...
%                 [4;3.5],[3;3],[2.25;2.25],[1.5;1.75]];
% landmarks = [landmarks,[3.5;1],[3.5;1.5],[4;1.5],[4;2.5],[2.5;2.75]];
%% plot the ground truth
% t = 0:0.1:pi/2;
% % the ground truth
% xTrace = [1,3,3+0.5*sin(t),4-0.5*cos(t),4+0.5*sin(2*t),4,3,3-0.5*sin(t),...
%     2+0.5*cos(t),2-0.5*sin(t)];
% yTrace = [1,1,1.5-0.5*cos(t),1.5+0.5*sin(t),2.5-0.5*cos(2*t),3,3,...
%     2.5+0.5*cos(t),2.5-0.5*sin(t),1.5+0.5*cos(t)];
% plot(xTrace,yTrace,'b-','Parent',trail_axes);hold on;grid on;
pb.showLandmarks(true); % Use this to turn on/off showing the landmarks.
pb.showViewRange(true); % Use this to turn on/off showing the pibot view.

% Place robot
% pb = piBotSim('floor_course.jpg', landmarks);
% pb.place([1;1],0);
%%
% show the estimate of robot
figure;
trail_axes = gca();
trail = [];

% intial state

% intital Phat
Phat = [1 0 1
        0 1 1
        0 0 1];
% empty matrix for storing updated 'p'
pbar = zeros(3,numel(landmark_positions)/2);
% this is a counter to record the number of each landmark
detected_num = zeros(1,numel(landmark_positions)/2);
% gain tuning except for ki
gain_k0 = 0.015;
gain_ci = 0.75;
for i = 1:numel(data)
    dt = data(i).delta_time;
    u = data(i).u;
    q = data(i).q;
    %[wl,wr] = inverse_kinematics(u,q);
    pb.setVelocity(a.wheel_left_velocity,a.wheel_right_velocity);
    % [u,q] = wheel2rob(data(i).wheel_left_velocity,data(i).wheel_right_velocity);
    W = [0 -q  u;
        q  0  0;
        0  0  0];
    % ybar stores info about landmarks and ids measured via robot
    ybar = data(i).lms(1:4,:);
    lm_num=ybar(4,:);
    % if the landmarks are detected, array is not empty
    if ~isempty(ybar)
        % count how many times landmarks have been detected
        detected_num(1,lm_num) = detected_num(1,lm_num) + 1;
    end
    % ybar_1 only contains landmarks
    ybar_1 = ybar(1:3,:);
    % phat stores info about landmarks position measured via
    % reference frame
    phat = Phat * ybar_1;
    % phat_chip contians info merely about position
    % localised vector
    phat_chip = pbar(:,ybar(4,:));
    estimated_landmarks=pbar(1:2,:);
    p_length = size(data(i).lms);
    for j = 1:p_length(2)
        if phat_chip(3,j) == 0
            % avoid "delay" error
            phat_chip(1:2,j) = phat(1:2,j);
            phat_chip(3,j) = phat(3,j);
        end
    end
    % landmarks measured from robot
    p_via_robot = Phat * ybar_1;
    ebar =  p_via_robot- phat_chip;
    [gain_ki] = gainControl(dt, ybar, detected_num);
    [Phat,phat] = observer(gain_k0,gain_ki,gain_ci,Phat,phat_chip,ybar_1,dt,W,ebar);
    pbar(:,ybar(4,:)) = phat;
    % Save the position to the trail
    % column 3 and row 1-2
    trail = [trail,Phat(1:2,3)];  
    plot(trail(1,:), trail(2,:), 'r-', 'Parent', trail_axes); 
    % plot
    drawnow;hold on;
    xlim(trail_axes, [0,5])
    ylim(trail_axes, [0,5])
end
    estimated_trajectory = zeros(3,size(data,2));
%%
function [Phat,phat] = observer(gain_k0,gain_ki,gain_ci,Phat,phat,ybar,dt,W,ebar)
    phat = phat + dt*(1-gain_k0)*ebar*gain_ki;
    Phat = Phat * expm(dt*(W-gain_k0*projector(Phat'*gain_ci*ebar*ybar')));
    
end
        
function W = projector(U)
    W = zeros(3);
    W(1:2,1:2) = 0.5 * (U(1:2,1:2) - U(1:2,1:2)');
    W(1:2,3) = U(1:2,3);
end

%set gain_ki
function [gain_ki] = gainControl(dt,ybar,observe_num)
len = size(ybar);
gain_ki = zeros(len(2));
sel_observed = observe_num(1,ybar(4,:));
if ~isempty(ybar)
    num = size(ybar);
    for k = 1:num(2)
        % exponential declay
        gain_ki(k,k) = 10*exp(-sel_observed(k)/50*dt);
        % inversely proportional
%        gain_ci(i,i) = 10*(sel_observed(i));
    end
end
end

function [u,q] = wheel2rob(wl,wr)
    scale_parameter = 5.33e-3;
    wheel_track = 0.156;
    u = scale_parameter/2 * (wl+wr);
    q = -scale_parameter/wheel_track * (wl-wr);
end

% function rmse = RMSE(phat,landmarks)
%     num = size(phat,2);
%     p_measure = phat(1:2,:);
%     rmse = sqrt(sum((p_measure-landmarks).^2,'all')/num);
% end