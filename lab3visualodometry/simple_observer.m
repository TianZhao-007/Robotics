close all

addpath('..\engn4627_pibot_simulator');
landmarks = 2*[cos(pi/4:pi/4:2*pi);sin(pi/4:pi/4:2*pi)] + [2.5;2.5];
landmarks = [landmarks, 1.5*[cos(pi/4:pi/4:2*pi);sin(pi/4:pi/4:2*pi)] + [2.5;2.5]];
pb = piBotSim('floor_circle.jpg', landmarks);

pb.place([2.5;1.5],0)
% theta = 0 , x = 2.5 , y = 1.5
% initial Phat
Phat = [1 0 2.5
        0 1 1.5
        0 0 1];

dt = 0.1;

% show the estimate of robot
figure;
trail_axes = gca();
trail = [];

gain_k = 0.05;

while true
    % Line following
    img = pb.getCamera();
    [u,q] = line_control(img, 3.0);
    [wl,wr] = inverse_kinematics(u,q);
    pb.setVelocity(wl,wr);
    
    % Integrate using expm
    % Integrate \dot{P} = P W
    W = [0 -q  u;
         q  0  0;
         0  0  0];
    Phat = Phat * expm(dt * W);
    % Save the position to the trail
    trail = [trail,Phat(1:2,3)];  % column 3 and row 1-2
    plot(trail(1,:), trail(2,:), 'b-', 'Parent', trail_axes);
    xlim(trail_axes, [0,5])
    ylim(trail_axes, [0,5])
    axis(trail_axes,'equal');
    
    % Compute innovation
    [lms, ids] = pb.measureLandmarks();
    ybar = [lms;ones(1,numel(ids))];
    ebar = Phat * ybar;
    pbar = [landmarks(:, ids); ones(1,numel(ids))];
    Delta = gain_k * projector((ebar - pbar) * ebar');
    % Integrate \dot{P} = - Delta P
    plot(ebar(1,:),ebar(2,:),'o','Parent',trail_axes);drawnow;hold on;
    Phat = expm(- dt * Delta) * Phat;
end


function W = projector(U)
    W = zeros(3);
    W(1:2,1:2) = 0.5 * (U(1:2,1:2) - U(1:2,1:2)');
    W(1:2,3) = U(1:2,3);
end