function new_state = integrate_kinematics(state, dt, lin_velocity, ang_velocity)
%INTEGRATE_KINEMATICS integrate the kinematics of the robot

%   state is the current state, and has the form [x;y;theta]
%   dt is the length of time for which to integrate
%   lin_velocity is the (forward) linear velocity of the robot
%   ang_velocity is the angular velocity of the robot

%   new_state is the state after integration, also in the form [x;y;theta]

new_state = state;

new_state(3) = state(3) + dt*ang_velocity;

if (ang_velocity ==0)
    new_state(1) = state(1) + dt * (lin_velocity * cos(state(3)));
    new_state(2) = state(2) + dt * (lin_velocity * sin(state(3)));
else
    new_state(1) = state(1) + lin_velocity * (sin(new_state(3)) - sin(state(3)));
    new_state(2) = state(2) + lin_velocity * (cos(new_state(3)) - cos(state(3)));

end

end

% here is the wrong solution 
% only meaningful for q = 0

 % a = [cos(state(3)) 0; sin(state(3)) 0; 0 1];
 
 % b = [lin_velocity ; ang_velocity];
 
 % new_state = state +  dt * a * b;

