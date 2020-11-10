% clear all
% close all
% addpath('../engn4627_pibot_simulator/');
% [lmx,lmy] = meshgrid(0.5:(4/3):4.5);
% landmarks = [lmx(:)'; lmy(:)'];
% pb = piBotSim("floor_course.jpg",landmarks);
% % Start by placing your robot at the start of the line
% pb.place([1;1], 0);
 
zeta = [1;1;0];
Sigma = eye(3)*0.01;
state = zeta(1:3,:);  g = 1;
% store the measured landmarks
state_ids = [];
 
% plot ground truth
t = 0:0.1:pi/2;
xTrace = [1,3,3+0.5*sin(t),4-0.5*cos(t),4+0.5*sin(2*t),4,3,3-0.5*sin(t),...
    2+0.5*cos(t),2-0.5*sin(t)];
yTrace = [1,1,1.5-0.5*cos(t),1.5+0.5*sin(t),2.5-0.5*cos(2*t),3,3,...
    2.5+0.5*cos(t),2.5-0.5*sin(t),1.5+0.5*cos(t)];
figure;
trail_axes = gca();
trail = [];
 
plot(xTrace,yTrace,'b-','Parent',trail_axes);
hold on;
grid on;
xlim(trail_axes,[0,5]);
ylim(trail_axes,[0,5]);
axis(trail_axes,'manual');
% Set noise here
% controlling noise
sigma_u = sqrt(2/50); % linear velocity noise (m/sec)
sigma_q = sqrt(2/50); % angular velocity noise (rad/sec)
R = [sigma_u^2, 0; 0, sigma_q^2]; % controlling noise covariance
 
% measurement noise
sigma_x = sqrt(0.005); % x direction noise
sigma_y = sqrt(0.08); % y direction noise
Q = [sigma_x^2, 0; 0, sigma_y^2]; % measurement noise covariance
 
while true
    % First, get the current camera frame
    img = pb.getCamera();
    %   imshow(img,'parent',camAxes);
    img = rgb2gray(img);
    img = img(end-30:end,:);
    line_img = ~imbinarize(img,0.2);
    %   imshow(line_img, "Parent", camAxes); % Check the video
    
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
    u = 0.09 * (1-2*x_mean^2);
    q = -1.3 * x_mean;
    
    % Compute the required wheel velocities
    [wl, wr] = inverse_kinematics(u,q);
    
    % Apply the wheel velocities
    pb.setVelocity(wl,wr);
    dt = 0.1;
    
    % observe landmarks
    [lms, ids] = pb.measureLandmarks();
    
    % predict
    [zeta, Sigma] = ekf_predict(zeta, Sigma, u, q, R, dt);
    
    % Augment
    [zeta,Sigma,state_ids] = augment(zeta,Sigma,R,lms,ids,state_ids);
    
    % Update
    [zeta,Sigma] = update(zeta,Sigma,ids,Q,lms,state_ids);
    
    % plot the trajectroy SLAM
    trail = [trail,zeta(1:3,:)];
    plot(trail(1,:), trail(2,:), 'r-', 'Parent', trail_axes);
    drawnow;
    hold on;
    xlim(trail_axes, [0,5])
    ylim(trail_axes, [0,5])
    
    % plot the wheel odometry
    state(:,g+1) = integrate_kinematics(state(:,g),dt,u,q);
    x = state(1,g+1);
    y = state(2,g+1);
    g = g+1;
    plot(state(1,:),state(2,:),'k','Parent',trail_axes); 
    drawnow;
    hold on;
    
    % plot the landmarks
    %     for j = 1:size(lms,2)
    %         xlim(trail_axes, [0,5])
    %         ylim(trail_axes, [0,5])
    %         lm_serial = ids(j);
    %         pose = [zeta(1);zeta(2);zeta(3)];
    %         inertial_lm = convert_to_inertial(pose, lms(:,j));
    %         % plot the measured landmarks
    %         scatter(inertial_lm(1),inertial_lm(2),6,'MarkerFaceColor',box(lm_serial),...
    %             'MarkerEdgeColor','none');
    %
    %           hold on;
    %         drawnow;
    %     end
    
    %compute AMRSE
    
    % plot the error ellipse
    error_ellipse_rob(zeta,Sigma, 0.8,trail_axes);
    error_ellipse_lm(zeta,Sigma, 0.5);
end
 
% plot the ground truth + trial + WO
plot(trail(1,:), trail(2, :),'r-','Parent',trail_axes);
plot(xTrace,yTrace,'b-','Parent',trail_axes);
plot(state(1,:),state(2,:),'k','Parent',trail_axes);

% estimated landmarks and trajectory
n = java.util.HashMap;
final_landmarks = [];
for i = 1:length(state_ids)
    n.put(state_ids(i),zeta([2*i+2,2*i+3]));
end
 
for i = 1:length(state_ids)
    final_landmarks = [final_landmarks,n.get(i)];
end
 
estimated_landmarks = final_landmarks;
estimated_trajectory = trail;


