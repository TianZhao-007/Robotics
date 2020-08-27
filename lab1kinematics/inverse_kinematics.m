function [wl, wr] = inverse_kinematics(u, q)
% Compute the left and right wheel velocities (wl, wr) required for the robot
% to achieve a forward speed u and angular speed q.

% The scale parameter and wheel base required to solve this are provided here.
% You can find these values in the robot simulator as well.
% In real-life, you would have to measure or calibrate them!

scale_parameter = 0.005356120444834 ;
wheel_track =  0.156919596451389; 

wl = (1/scale_parameter) * (u-wheel_track/2*q);
wr = (1/scale_parameter) * (u+wheel_track/2*q);

wl = round(wl);
wr = round(wr);

end