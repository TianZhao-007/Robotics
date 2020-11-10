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


 function xdot = fcn(x,u)
mr=0.095;
mp=0.024;
lr=0.085;
lp=0.129;
jr=5.7e-5;
jp=3.3e-5;
kt=0.042;
rm=8.4;
dr=0.0015;
dp=0.0005;
k=0.017;
g=9.8;
jt=jp*mp*lr*lr+jr*jp+0.25*jr*mp*lp*lp;
a31=-1*(jp+0.25*mp*lp*lp)*k/jt;
a32=0.25*mp*mp*lp*lp*lr*g/jt;
a33=-1*(jp+0.25*mp*lp*lp)*dr/jt;
a34=0.5*mp*lp*lr*dp/jt;
a41=0.5*mp*lp*lr*k/jt;
a42=-0.5*mp*lp*g*(jr+mp*lr*lr)/jt;
a43=0.5*mp*lp*lr*dr/jt;
a44=-1*(jr+mp*lr*lr)*dp/jt;
b31=(jp+0.25*mp*lp*lp)/jt;
b41=-0.5*mp*lp*lr/jt;
The_A=[0 0 1 0
0 0 0 1
a31 a32 a33 a34
a41 a42 a43 a44];
The_B=[0 0 0 0
0 0 0 0
b31 0 0 0
b41 0 0 0];
u1=[u
    0
    0
    0];
xdot=The_A*x+The_B*u1;
end
