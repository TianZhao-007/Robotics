% Calibrate the scale parameter and wheel track of the robot
addpath("../simulator/"); % Add the simulator to the MATLAB path.
pb = piBotSim("floor.jpg");

% The answers must be saved with the following names!
% Otherwise, the evaluation system will not be able to recognise them.
scale_parameter =  0.005345095355462 ;
wheel_track =  0.155355064097791; 
                                                                                          
% Write your code to compute scale_parameter and wheel_track below.
% HINTS:
% - Start by placing your robot (pb.place). Then, drive forward for a known
% time, and measure the robot position (pb.measure) to compute the
% velocity. This will let you solve for the scale parameter.
% - Using multiple trials with different speeds is key to your success!

 %compute the scale_parameter
    % create matrix
    X = ones(100,1);    
    Y = ones(100,1);
    X1  = ones(100,1);
    Y1 = ones(100,1);
    
    % let car drive for 5s, wl = wr = w0  (from 5 to 100 with 5 interval)
    wl = 1;
    wr = 1;
    t = 1;
    for i = 1:100
    %place the car in (0.5,0.5) with angle = 0
    pb.place([0.5;0.5],0);
    [position_a,angle_a] = pb.measure;
    pb.setVelocity([wl wr],t);
    [position_b,angle_b] = pb.measure;
    distance = position_b(1) - position_a(1);
    w0 = (wl + wr)/2;
    u = distance/t;

    % store data in matrix
    X(i)= w0;
    Y(i) = u;

    wl = wl + 1;
    wr = wr + 1;
    end

    % APPLY least second method
    Sxx = sum((X-mean(X)).^2);
    Sxy  = sum((X-mean(X)).*(Y-mean(Y)));
    k0=Sxy/Sxx;
    b0=mean(Y)-k0*mean(X);
    y0=k0*X+b0;
    scale_parameter = k0;

 % compute wheel_track
 % let car drive for 5s, wl = -wr = w1 (wl from 5 to 100
    wl = 1;
    wr = -1;
    t = 1;
    for i = 1:100
    %place the car in (0.5,0.5) with angle = 0
    pb.place([0.5;0.5],0);
    [position_a,angle_a] = pb.measure;
    pb.setVelocity([wl wr],t);
    [position_b,angle_b] = pb.measure;
    delta_angle = angle_b(1) - angle_a(1);
    w1 = (wl - wr)/2;
    q = delta_angle/t;

    % store data in matrix
    X1(i)= w1;
    Y1(i) = q;

    wl = wl + 1;
    wr = wr - 1;
    end
    % apply least second
    Lxx=sum((X1-mean(X1)).^2);
    Lxy=sum((X1-mean(X1)).*(Y1-mean(Y1)));
    k1=Lxy/Lxx;
    b1=mean(Y1)-k1*mean(X1);
    y1=k1*X1+b1;
    wheel_track = -2*scale_parameter/k1;
    
    % ommit the plot code
 
   